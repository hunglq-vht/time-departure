"""Multi-drone takeoff deconfliction.

Resolves a batch of FlightIntentions whose drones are all queued at the same
takeoff vertiport (possibly on different pads) and will fly to the same or
different destination vertiports.

Algorithm
---------
1. Sort FIs by (priority ASC, t_des ASC) — highest-priority / earliest-desired
   drones are processed first and receive the smallest delays.
2. For each FI in priority order, call ``resolve_conflict`` against the union of:
     * the pre-existing approved_plans from the system state, and
     * the plans approved so far within this batch.
   This guarantees that every newly approved drone is treated as a committed
   obstacle for all subsequent drones in the same batch.
3. Approved plans are converted back into ``ApprovedPlan`` objects (SOFT_RESERVED)
   and appended to the running list before proceeding to the next FI.
4. Results are returned in the *original input order* inside a
   ``BatchDeconflictResult``.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

from .models import (
    ApprovedPlan,
    BatchDeconflictResult,
    DroneDeconflictEntry,
    ApproveResult,
    FlightIntention,
    SCRPConfig,
    SCRPResult,
    SystemState,
    Waypoint,
)
from .scrp import resolve_conflict


def _approved_plan_from_result(
    fi: FlightIntention,
    result: ApproveResult,
) -> ApprovedPlan:
    """Build an ApprovedPlan from a successful resolve_conflict result.

    The plan is tagged SOFT_RESERVED so subsequent drones treat it as a
    committed obstacle (pessimistic scheduling).
    """
    wp_map: Dict[str, Waypoint] = {w.id: w for w in fi.lane.waypoints}
    waypoint_times = [
        (wp_map[wid], t) for wid, t in result.waypoint_times if wid in wp_map
    ]
    return ApprovedPlan(
        drone_id=fi.drone_id,
        uav_type=fi.uav_type,
        lane=fi.lane,
        v_waypoints=list(fi.v_waypoints),
        t_land=result.t_land_assigned,
        pad_id=result.pad_id,
        slot_index=result.slot_index,
        status='SOFT_RESERVED',
        t_dep=result.t_dep_star,
        waypoint_times=waypoint_times,
        algorithm='SCRP',
        expires_at=result.expires_at,
        vertiport_id=result.vertiport_id,
        origin_vertiport_id=fi.origin_vertiport_id,
        takeoff_pad_id=fi.takeoff_pad_id,
        t_takeoff=fi.t_takeoff,
    )


def resolve_multi_drone_deconfliction(
    flight_intentions: List[FlightIntention],
    system_state: SystemState,
    config: SCRPConfig,
) -> BatchDeconflictResult:
    """Deconflict a group of drones queued at the same takeoff vertiport.

    Parameters
    ----------
    flight_intentions:
        All FIs to be resolved in this batch.  They need not share the same
        destination or lane; they only need to originate from the same
        takeoff vertiport (``fi.origin_vertiport_id``).
    system_state:
        Current global state (approved_plans from prior batches, vertiport
        slot maps, MSD matrix, etc.).
    config:
        SCRP configuration (SoC thresholds, slot duration, …).

    Returns
    -------
    BatchDeconflictResult
        One ``DroneDeconflictEntry`` per input FI, in the original input order.
    """
    if not flight_intentions:
        return BatchDeconflictResult(
            entries=[],
            approved_count=0,
            rejected_count=0,
            total_drones=0,
        )

    # --- Sort by (priority ASC, t_des ASC) to determine processing order ---
    # Lower priority number = higher priority (1 = most urgent).
    indexed: List[Tuple[int, FlightIntention]] = list(enumerate(flight_intentions))
    sorted_fis = sorted(indexed, key=lambda x: (x[1].priority, x[1].t_des))

    # Running list of approved plans: starts with pre-existing system plans
    running_approved: List[ApprovedPlan] = list(system_state.approved_plans)

    # Collect results keyed by original index
    results_by_idx: Dict[int, Tuple[SCRPResult, int]] = {}  # orig_idx -> (result, rank)

    for rank, (orig_idx, fi) in enumerate(sorted_fis):
        result = resolve_conflict(fi, running_approved, system_state, config)
        results_by_idx[orig_idx] = (result, rank)

        if isinstance(result, ApproveResult):
            plan = _approved_plan_from_result(fi, result)
            running_approved.append(plan)

    # --- Assemble output in original input order ---
    entries: List[DroneDeconflictEntry] = []
    approved_count = 0
    rejected_count = 0

    for orig_idx, fi in enumerate(flight_intentions):
        result, rank = results_by_idx[orig_idx]
        entries.append(
            DroneDeconflictEntry(
                drone_id=fi.drone_id,
                result=result,
                processing_rank=rank,
            )
        )
        if isinstance(result, ApproveResult):
            approved_count += 1
        else:
            rejected_count += 1

    return BatchDeconflictResult(
        entries=entries,
        approved_count=approved_count,
        rejected_count=rejected_count,
        total_drones=len(flight_intentions),
    )
