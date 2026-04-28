"""Pure geometry helpers — no side effects."""
from __future__ import annotations

import math
from typing import Dict, List, Tuple

from .models import ApprovedPlan, FlightIntention, Lane, Waypoint


def segment_travel_times(v_waypoints: List[float], segments_lengths: List[float]) -> List[float]:
    """Return travel time for each segment."""
    return [L / v for L, v in zip(segments_lengths, v_waypoints)]


def waypoint_arrival_times(fi: FlightIntention, t_dep: float) -> List[float]:
    """Absolute arrival time at each waypoint given departure time t_dep."""
    times: List[float] = [t_dep]
    cumulative = t_dep
    for seg, v in zip(fi.lane.segments, fi.v_waypoints):
        cumulative += seg.length / v
        times.append(cumulative)
    return times


def t_arrival_at_waypoint(fi: FlightIntention, waypoint_index: int, t_dep: float) -> float:
    """Arrival time at waypoints[waypoint_index] given t_dep."""
    t = t_dep
    for k in range(waypoint_index):
        seg = fi.lane.segments[k]
        t += seg.length / fi.v_waypoints[k]
    return t


def t_land(fi: FlightIntention, t_dep: float) -> float:
    """Total flight time added to t_dep gives landing time."""
    total = t_dep
    for seg, v in zip(fi.lane.segments, fi.v_waypoints):
        total += seg.length / v
    return total


def waypoint_index_in_lane(wp: Waypoint, lane: Lane) -> int:
    """Return index of waypoint in lane, or -1 if not found."""
    for i, w in enumerate(lane.waypoints):
        if w.id == wp.id:
            return i
    return -1


def common_waypoints(lane_a: Lane, lane_b: Lane) -> List[Tuple[int, int]]:
    """Return list of (index_in_a, index_in_b) for shared waypoints."""
    ids_b: Dict[str, int] = {w.id: i for i, w in enumerate(lane_b.waypoints)}
    result: List[Tuple[int, int]] = []
    for i, w in enumerate(lane_a.waypoints):
        if w.id in ids_b:
            result.append((i, ids_b[w.id]))
    return result


def plan_waypoint_times_dict(plan: ApprovedPlan) -> Dict[str, float]:
    """Return {waypoint_id: arrival_time} for an approved plan.

    Uses pre-computed waypoint_times if available; otherwise derives them
    from plan.t_dep and plan.v_waypoints (lazy computation path).
    """
    if plan.waypoint_times:
        return {w.id: t for w, t in plan.waypoint_times}
    if plan.t_dep is None:
        return {}
    result: Dict[str, float] = {}
    cumulative = plan.t_dep
    for i, wp in enumerate(plan.lane.waypoints):
        result[wp.id] = cumulative
        if i < len(plan.lane.segments):
            v = plan.v_waypoints[i] if i < len(plan.v_waypoints) else plan.lane.segments[i].v_min
            cumulative += plan.lane.segments[i].length / v
    return result


def euclidean_distance(a: Waypoint, b: Waypoint) -> float:
    dx = b.position[0] - a.position[0]
    dy = b.position[1] - a.position[1]
    dz = b.position[2] - a.position[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)
