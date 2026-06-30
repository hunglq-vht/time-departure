"""Flight route recommendation based on the scrp_simple conflict resolver.

Algorithm
---------
Given a movement demand (takeoff/landing vertiport, desired departure time,
max speed, takeoff mass, takeoff/landing durations) and a list of planned
flight routes, this module finds which routes are operationally feasible and
at what departure time.

Processing pipeline
-------------------
For each planned route:

1. **Vertiport match** — the route's takeoff/landing vertiport IDs must match
   the demand.  Routes with a mismatch are immediately rejected.

2. **Weight limit** — if the route specifies a maximum takeoff mass, the
   demand's takeoff_mass_kg must not exceed it.

3. **Conflict resolution** — a NewPlanRequest is built from the demand and
   the route, then passed to ConflictResolver.  The resolver iterates four
   interdependent constraints (lane separation, takeoff/landing airspace
   serialisation, pad availability) until the required departure delay
   converges.  If the resulting delay exceeds the demand's max_wait_time the
   route is marked infeasible.

4. **Travel time estimation** — for feasible routes the expected total journey
   time is computed as:
       t_takeoff + lane_time + t_land
   where lane_time uses the effective per-segment speed.

Effective segment speed
-----------------------
Each segment is flown at:
    v_seg = min(demand.max_speed_ms, route.max_speed_ms)

This ensures neither the drone's capability nor the route's design limit is
exceeded.

Output
------
The returned RecommendationResult contains:
  • recommended_routes — feasible routes sorted by (feasible_departure_time,
    estimated_travel_time_s).  The operator is shown the soonest options first.
  • infeasible_routes — routes that failed with the reason token.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .models import ApprovedPlan, FlightPath, NewPlanRequest, Point3D, Vertiport
from .resolver import ConflictResolver

# ---------------------------------------------------------------------------
# Rejection reason tokens
# ---------------------------------------------------------------------------
INFEASIBLE_VERTIPORT_MISMATCH = "vertiport_mismatch"
INFEASIBLE_WEIGHT_EXCEEDED = "weight_limit_exceeded"
INFEASIBLE_CONFLICT_UNRESOLVABLE = "conflict_unresolvable"


# ---------------------------------------------------------------------------
# Data models
# ---------------------------------------------------------------------------

@dataclass
class PlannedFlightRoute:
    """A pre-planned, approved flight corridor.

    ``direction`` is a free-form label that identifies which direction within a
    corridor this route serves (e.g., "VP_A→VP_B", "NORTHBOUND").  It is
    informational only; directional filtering is enforced through the
    takeoff/landing vertiport IDs.

    ``waypoints`` are the *intermediate* points along the horizontal lane, not
    including the vertiport locations themselves (same convention as
    scrp_simple.models.FlightPath.waypoints).

    ``max_speed_ms`` is the highest speed permitted anywhere on this route by
    its design (airspace speed limit, obstacle clearance margin, etc.).

    ``max_takeoff_mass_kg`` is the heaviest drone this route is certified to
    carry.  Set to ``float('inf')`` (the default) to disable the weight check.
    """

    route_id: str
    takeoff_vertiport_id: str
    landing_vertiport_id: str
    direction: str
    waypoints: List[Point3D]
    max_speed_ms: float
    max_takeoff_mass_kg: float = float("inf")


@dataclass
class MovementDemand:
    """A single travel request submitted for route recommendation.

    Fields
    ------
    takeoff_vertiport_id / landing_vertiport_id
        Origin and destination vertiports.
    desired_departure_time
        Earliest acceptable departure time in Unix seconds.
    t_takeoff
        Duration of the vertical takeoff ascent phase [s].
    t_land
        Duration of the vertical landing descent phase [s].
    max_speed_ms
        Maximum horizontal cruise speed the drone can sustain [m/s].
    takeoff_mass_kg
        Actual takeoff mass of the drone (used to check route weight limits) [kg].
    max_wait_time
        Maximum acceptable departure delay beyond *desired_departure_time* [s].
        Defaults to 30 minutes.
    priority
        Scheduling priority (1 = highest).  Passed through to the conflict
        resolver so that high-priority demands take precedence in the airspace.
    drone_type
        Opaque identifier used in conflict resolution logging (e.g., 'A', 'B').
    """

    takeoff_vertiport_id: str
    landing_vertiport_id: str
    desired_departure_time: float
    t_takeoff: float
    t_land: float
    max_speed_ms: float
    takeoff_mass_kg: float
    max_wait_time: float = 1800.0
    priority: int = 5
    drone_type: str = ""


@dataclass
class RouteRecommendation:
    """A feasible route paired with its scheduling outcome.

    Fields
    ------
    route
        The planned route that can be flown.
    feasible_departure_time
        Earliest conflict-free departure time in Unix seconds.
    delay_seconds
        Seconds added beyond the demand's desired_departure_time.
    cruise_time_s
        Time spent flying the horizontal lane (waypoint-to-waypoint) [s].
    estimated_travel_time_s
        Total journey time from pad to pad: t_takeoff + cruise_time + t_land [s].
    segment_speeds
        Effective speed [m/s] used on each lane segment (len = waypoints-1).
    assigned_pad_id
        Landing pad assigned by the resolver at the arrival vertiport.
    conflict_details
        Human-readable list of constraints that pushed the departure time.
    """

    route: PlannedFlightRoute
    feasible_departure_time: float
    delay_seconds: float
    cruise_time_s: float
    estimated_travel_time_s: float
    segment_speeds: List[float]
    assigned_pad_id: Optional[str]
    conflict_details: List[str] = field(default_factory=list)


@dataclass
class RecommendationResult:
    """Output of recommend_routes()."""

    demand: MovementDemand
    recommended_routes: List[RouteRecommendation]
    infeasible_routes: List[Tuple[PlannedFlightRoute, str]]


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------

def recommend_routes(
    demand: MovementDemand,
    planned_routes: List[PlannedFlightRoute],
    vertiports: Dict[str, Vertiport],
    approved_plans: List[ApprovedPlan],
) -> RecommendationResult:
    """Find all feasible routes and their earliest departure times.

    Parameters
    ----------
    demand:
        The movement demand describing the drone's travel need.
    planned_routes:
        All pre-planned routes in the airspace.  Routes with mismatching
        vertiports or excessive weight are filtered out before conflict
        resolution runs.
    vertiports:
        Mapping from vertiport_id to Vertiport (with pad definitions).
        Required by ConflictResolver for pad-availability checks.
    approved_plans:
        Flight plans already in the system.  These are treated as fixed
        constraints when resolving conflicts for the new demand.

    Returns
    -------
    RecommendationResult
        recommended_routes sorted by (feasible_departure_time,
        estimated_travel_time_s) — earliest and fastest options first.
    """
    recommended: List[RouteRecommendation] = []
    infeasible: List[Tuple[PlannedFlightRoute, str]] = []

    for route in planned_routes:
        # --- Stage 1: vertiport match ---
        if (
            route.takeoff_vertiport_id != demand.takeoff_vertiport_id
            or route.landing_vertiport_id != demand.landing_vertiport_id
        ):
            infeasible.append((route, INFEASIBLE_VERTIPORT_MISMATCH))
            continue

        # --- Stage 2: weight limit ---
        if demand.takeoff_mass_kg > route.max_takeoff_mass_kg:
            infeasible.append((route, INFEASIBLE_WEIGHT_EXCEEDED))
            continue

        # --- Stage 3: compute effective segment speeds ---
        effective_speed = min(demand.max_speed_ms, route.max_speed_ms)
        num_segments = len(route.waypoints) - 1 if len(route.waypoints) >= 2 else 0
        segment_speeds = [effective_speed] * num_segments

        # --- Stage 4: build NewPlanRequest ---
        flight_path = FlightPath(
            takeoff_vertiport_id=route.takeoff_vertiport_id,
            landing_vertiport_id=route.landing_vertiport_id,
            waypoints=list(route.waypoints),
        )
        request = NewPlanRequest(
            flight_path=flight_path,
            drone_type=demand.drone_type,
            segment_speeds=segment_speeds,
            desired_start_time=demand.desired_departure_time,
            max_wait_time=demand.max_wait_time,
            t_takeoff=demand.t_takeoff,
            t_land=demand.t_land,
            priority=demand.priority,
        )

        # --- Stage 5: conflict resolution ---
        resolver = ConflictResolver(vertiports, approved_plans)
        result = resolver.resolve(request)

        if not result.approved:
            infeasible.append((route, INFEASIBLE_CONFLICT_UNRESOLVABLE))
            continue

        # --- Stage 6: travel time ---
        cruise_time = _cruise_time(route.waypoints, segment_speeds)
        total_travel_time = demand.t_takeoff + cruise_time + demand.t_land

        recommended.append(
            RouteRecommendation(
                route=route,
                feasible_departure_time=result.actual_start_time,  # type: ignore[arg-type]
                delay_seconds=result.delay,
                cruise_time_s=cruise_time,
                estimated_travel_time_s=total_travel_time,
                segment_speeds=segment_speeds,
                assigned_pad_id=result.assigned_pad_id,
                conflict_details=result.conflict_details,
            )
        )

    # Sort: soonest departure first, then shortest journey
    recommended.sort(
        key=lambda r: (r.feasible_departure_time, r.estimated_travel_time_s)
    )

    return RecommendationResult(
        demand=demand,
        recommended_routes=recommended,
        infeasible_routes=infeasible,
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _cruise_time(waypoints: List[Point3D], segment_speeds: List[float]) -> float:
    """Total horizontal lane time given waypoints and per-segment speeds."""
    if len(waypoints) < 2 or not segment_speeds:
        return 0.0
    return sum(
        waypoints[i].distance_to(waypoints[i + 1]) / segment_speeds[i]
        for i in range(len(waypoints) - 1)
    )
