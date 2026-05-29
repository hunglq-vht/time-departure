"""Main SCRP orchestrator — resolve_conflict is pure (no side effects)."""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .energy import compute_E_cruise, compute_SoC_remaining
from .geometry import t_arrival_at_waypoint, t_land, waypoint_arrival_times
from .headway import compute_delta_C1
from .junction import compute_delta_C2
from .models import (
    ApproveResult,
    ApprovedPlan,
    FlightIntention,
    RejectResult,
    SCRPConfig,
    SCRPResult,
    SystemState,
)
from .slot import find_earliest_slot, slot_start_time


def _validate_fi(fi: FlightIntention) -> Optional[str]:
    """Return error message if FI is invalid, else None."""
    if not fi.drone_id:
        return "drone_id is empty"
    if not fi.lane or not fi.lane.waypoints:
        return "lane has no waypoints"
    if len(fi.v_waypoints) != len(fi.lane.waypoints):
        return f"v_waypoints length {len(fi.v_waypoints)} != waypoints length {len(fi.lane.waypoints)}"
    for i, v in enumerate(fi.v_waypoints[:-1]):  # last entry unused
        seg = fi.lane.segments[i]
        if v < seg.v_min or v > seg.v_max:
            return f"v_waypoints[{i}]={v} outside [{seg.v_min}, {seg.v_max}] for segment {i}"
    if not (0.0 <= fi.SoC_0 <= 1.0):
        return f"SoC_0={fi.SoC_0} not in [0, 1]"
    if fi.C_bat <= 0:
        return f"C_bat={fi.C_bat} must be positive"
    if fi.P_hover <= 0.0 and fi.flight_time_min <= 0.0:
        if not (fi.mtow_kg > 0.0 and fi.num_rotors > 0 and fi.propeller_diameter_m > 0.0):
            return (
                "hover power cannot be determined: provide P_hover, flight_time_min, "
                "or [mtow_kg + num_rotors + propeller_diameter_m]"
            )
    return None


def _determine_delay_source(
    delta_C1: float,
    delta_C2: float,
    delta_C3: float,
    delay_total: float,
) -> str:
    if delay_total == 0.0:
        return 'none'
    if delta_C3 >= delay_total:
        return 'C3_slot'
    if delta_C2 >= delay_total:
        return 'C2_junction'
    if delta_C1 >= delay_total:
        return 'C1_headway'
    return 'none'


def resolve_conflict(
    fi: FlightIntention,
    approved_plans: List[ApprovedPlan],
    system_state: SystemState,
    config: SCRPConfig,
) -> SCRPResult:
    """Compute optimal departure time t_dep* for the given FlightIntention.

    Pure function — does not mutate any state.
    The destination vertiport is derived from fi.lane.destination_vertiport_id.
    """
    # Step 1 — Validate
    err = _validate_fi(fi)
    if err:
        return RejectResult(
            status='REJECTED',
            reason='invalid_fi',
            earliest_possible=None,
            detail=err,
        )

    vertiport_id = fi.lane.destination_vertiport_id
    vertiport_state = system_state.vertiports.get(vertiport_id)
    if vertiport_state is None:
        return RejectResult(
            status='REJECTED',
            reason='unknown_vertiport',
            earliest_possible=None,
            detail=f"No vertiport found for id '{vertiport_id}' (lane '{fi.lane.id}')",
        )

    t_des = fi.t_des

    # Step 2 — Delta_C1 (headway)
    delta_C1 = compute_delta_C1(fi, approved_plans, t_des, system_state)

    # Step 3 — Delta_C2 (junction)
    delta_C2 = compute_delta_C2(fi, approved_plans, t_des, system_state, config)

    # Step 4 — Preliminary t_dep
    t_dep_prelim = t_des + max(0.0, delta_C1, delta_C2)

    # Step 5 — Delta_C3 (landing slot)
    t_land_prelim = t_land(fi, t_dep_prelim)
    slot_key, delta_C3 = find_earliest_slot(
        t_land_desired=t_land_prelim,
        vertiport_state=vertiport_state,
        uav_type=fi.uav_type,
        t_epoch=0.0,
        config=config,
    )

    if slot_key is None or math.isinf(delta_C3):
        return RejectResult(
            status='REJECTED',
            reason='no_slot_available',
            earliest_possible=None,
            detail='No compatible landing slot found within search window',
        )

    # Step 6 — Final t_dep*
    delay_total = max(0.0, delta_C1, delta_C2, delta_C3)
    t_dep_star = t_des + delay_total
    delay_source = _determine_delay_source(delta_C1, delta_C2, delta_C3, delay_total)

    # Reject if delay exceeds maximum acceptable
    if delay_total > config.MAX_ACCEPTABLE_DELAY_SEC:
        return RejectResult(
            status='REJECTED',
            reason='no_feasible_t_dep',
            earliest_possible=t_dep_star,
            detail=f'Required delay {delay_total:.1f}s exceeds maximum {config.MAX_ACCEPTABLE_DELAY_SEC:.1f}s',
        )

    # Re-find slot for the final t_dep (slot may shift)
    t_land_final = t_land(fi, t_dep_star)
    slot_key, _ = find_earliest_slot(
        t_land_desired=t_land_final,
        vertiport_state=vertiport_state,
        uav_type=fi.uav_type,
        t_epoch=0.0,
        config=config,
    )
    if slot_key is None:
        return RejectResult(
            status='REJECTED',
            reason='no_slot_available',
            earliest_possible=None,
            detail='No compatible landing slot found after delay adjustment',
        )

    pad_id, slot_index = slot_key
    t_slot_start = slot_start_time(slot_index, 0.0, vertiport_state.slot_duration)

    # Step 7 — SoC check (C4)
    E_cruise = compute_E_cruise(fi, config)
    SoC_remaining = compute_SoC_remaining(fi, t_dep_star, t_slot_start, E_cruise)

    if SoC_remaining < config.SOC_MIN:
        return RejectResult(
            status='REJECTED',
            reason='SoC_insufficient',
            earliest_possible=None,
            detail=(
                f'SoC after flight {SoC_remaining:.3f} below minimum {config.SOC_MIN}. '
                f'E_cruise={E_cruise:.3f} Wh, C_bat={fi.C_bat} Wh, SoC_0={fi.SoC_0}'
            ),
        )

    # Step 8 — Build waypoint_times for result
    arrival_times = waypoint_arrival_times(fi, t_dep_star)
    waypoint_times_out: List[Tuple[str, float]] = [
        (wp.id, arrival_times[i]) for i, wp in enumerate(fi.lane.waypoints)
    ]

    expires_at = system_state.t_now + config.T_RESPONSE_WINDOW_SEC

    # Step 9 — Return ApproveResult (no state mutation here)
    return ApproveResult(
        status='SOFT_RESERVED',
        t_dep_star=t_dep_star,
        t_land_assigned=t_land_final,
        pad_id=pad_id,
        slot_index=slot_index,
        vertiport_id=vertiport_id,
        waypoint_times=waypoint_times_out,
        expires_at=expires_at,
        delay_seconds=t_dep_star - t_des,
        delay_source=delay_source,
        energy_estimate=E_cruise,
        SoC_remaining=SoC_remaining,
    )
