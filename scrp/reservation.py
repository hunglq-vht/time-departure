"""Reservation lifecycle management — all side-effecting operations."""
from __future__ import annotations

import copy
from typing import List, Optional, Tuple

from .models import ApproveResult, ApprovedPlan, Lane, SystemState, Waypoint


def create_soft_reservation(
    result: ApproveResult,
    fi_lane: Lane,
    fi_drone_id: str,
    fi_uav_type: str,
    state: SystemState,
    t_response_window: float,
) -> SystemState:
    """Create a SOFT_RESERVED plan and mark the slot. Returns mutated state copy."""
    new_state = copy.deepcopy(state)

    # Build waypoint_times list of (Waypoint, float)
    wp_map = {w.id: w for w in fi_lane.waypoints}
    waypoint_times: List[Tuple[Waypoint, float]] = [
        (wp_map[wp_id], t) for wp_id, t in result.waypoint_times if wp_id in wp_map
    ]

    plan = ApprovedPlan(
        drone_id=fi_drone_id,
        uav_type=fi_uav_type,
        lane=fi_lane,
        waypoint_times=waypoint_times,
        t_land=result.t_land_assigned,
        pad_id=result.pad_id,
        slot_index=result.slot_index,
        status='SOFT_RESERVED',
        expires_at=result.expires_at,
    )
    new_state.approved_plans.append(plan)

    key = (result.pad_id, result.slot_index)
    new_state.vertiport_state.slots[key] = fi_drone_id
    new_state.vertiport_state.slot_status[key] = 'SOFT'

    return new_state


def commit_reservation(drone_id: str, state: SystemState) -> SystemState:
    """Transition a SOFT_RESERVED plan to COMMITTED."""
    new_state = copy.deepcopy(state)
    for plan in new_state.approved_plans:
        if plan.drone_id == drone_id and plan.status == 'SOFT_RESERVED':
            plan.status = 'COMMITTED'
            plan.expires_at = None
            key = (plan.pad_id, plan.slot_index)
            new_state.vertiport_state.slot_status[key] = 'COMMITTED'
            break
    return new_state


def release_reservation(drone_id: str, state: SystemState) -> SystemState:
    """Remove a plan and free its slot."""
    new_state = copy.deepcopy(state)
    for i, plan in enumerate(new_state.approved_plans):
        if plan.drone_id == drone_id:
            key = (plan.pad_id, plan.slot_index)
            new_state.vertiport_state.slots[key] = None
            new_state.vertiport_state.slot_status[key] = 'FREE'
            new_state.approved_plans.pop(i)
            break
    return new_state


def expire_soft_reservations(state: SystemState, t_now: float) -> SystemState:
    """Release all SOFT_RESERVED plans whose expires_at has passed."""
    new_state = copy.deepcopy(state)
    expired = [
        p for p in new_state.approved_plans
        if p.status == 'SOFT_RESERVED' and p.expires_at is not None and p.expires_at < t_now
    ]
    for plan in expired:
        key = (plan.pad_id, plan.slot_index)
        new_state.vertiport_state.slots[key] = None
        new_state.vertiport_state.slot_status[key] = 'FREE'
        new_state.approved_plans.remove(plan)
    return new_state


def cascade_notify(rejected_drone_id: str, state: SystemState) -> List[str]:
    """Return list of drone_ids whose plans depended on the rejected plan.

    In this implementation, dependency is implicit: any SOFT_RESERVED plan
    on a lane that shares waypoints with the rejected plan's lane.
    Callers can use this list to notify affected operators.
    """
    rejected = next(
        (p for p in state.approved_plans if p.drone_id == rejected_drone_id), None
    )
    if rejected is None:
        return []

    rejected_wp_ids = {w.id for w in rejected.lane.waypoints}
    affected: List[str] = []
    for plan in state.approved_plans:
        if plan.drone_id == rejected_drone_id:
            continue
        if plan.status != 'SOFT_RESERVED':
            continue
        plan_wp_ids = {w.id for w in plan.lane.waypoints}
        if plan_wp_ids & rejected_wp_ids:
            affected.append(plan.drone_id)
    return affected
