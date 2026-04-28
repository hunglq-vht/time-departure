"""Junction conflict detection and C2 delay computation."""
from __future__ import annotations

from typing import Dict, List, Set

from .geometry import t_arrival_at_waypoint
from .models import ApprovedPlan, FlightIntention, Lane, SCRPConfig, SystemState, Waypoint


def detect_junctions(lane: Lane, all_lanes: List[Lane]) -> List[Waypoint]:
    """Return waypoints in `lane` that also appear in at least one other lane."""
    other_ids: Set[str] = set()
    for other in all_lanes:
        if other.id == lane.id:
            continue
        for wp in other.waypoints:
            other_ids.add(wp.id)

    return [wp for wp in lane.waypoints if wp.id in other_ids]


def compute_delta_C2(
    fi: FlightIntention,
    approved_plans: List[ApprovedPlan],
    t_des: float,
    state: SystemState,
    config: SCRPConfig,
) -> float:
    """Return maximum additional delay needed to avoid junction conflicts."""
    if not approved_plans:
        return 0.0

    all_lanes = [fi.lane] + [p.lane for p in approved_plans]
    junctions = detect_junctions(fi.lane, all_lanes)
    if not junctions:
        return 0.0

    delta_t_J = config.JUNCTION_DIAMETER_M / fi.lane.segments[0].v_min if fi.lane.segments else \
        config.JUNCTION_DIAMETER_M / 5.0

    delta = 0.0

    for junction in junctions:
        # Find index of junction in the new FI's lane
        idx_new = next((i for i, w in enumerate(fi.lane.waypoints) if w.id == junction.id), -1)
        if idx_new == -1:
            continue

        t_arr_new = t_arrival_at_waypoint(fi, idx_new, t_des)

        for plan in approved_plans:
            plan_times: Dict[str, float] = {w.id: t for w, t in plan.waypoint_times}
            tau_J = plan_times.get(junction.id)
            if tau_J is None:
                continue

            gap = t_arr_new - tau_J
            if abs(gap) < delta_t_J:
                if t_arr_new >= tau_J:
                    # New drone arrives after existing plan — must wait
                    needed = tau_J + delta_t_J - t_arr_new
                    if needed > delta:
                        delta = needed
                # If new drone arrives before, no delay needed

    return max(0.0, delta)
