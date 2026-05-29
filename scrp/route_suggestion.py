"""Flight route suggestion: filter available routes and estimate flight metrics.

This module is a pure function with no side effects.  The caller is responsible
for fetching routes from the external route service (see flight_route_client.py)
before calling suggest_routes().

Filtering pipeline
------------------
Each route passes through three stages in order:

1. **Hard eligibility** — objective mismatches the drone cannot overcome:
   vertiport endpoints, wind envelope, weight limit, size compatibility,
   and service ceiling.

2. **Internal safety viability** — the route's externally-published
   safety_score is compared against a minimum threshold that the system
   computes adaptively.  The threshold rises as the route's average wind
   speed approaches the drone's wind resistance limit, ensuring that
   challenging conditions require correspondingly safer corridors.  Callers
   do not set this threshold; it is a system-level policy.

3. **Energy viability** — the physics-based estimate (see power_model.py)
   must leave SoC_remaining >= drone.soc_min on arrival.

Routes that fail are collected with the first failing reason.
"""
from __future__ import annotations

from typing import List, Optional

from .models import (
    DroneProfile,
    FlightRoute,
    RouteRejection,
    RouteSuggestionRequest,
    RouteSuggestionResult,
    SuggestedRoute,
)
from .power_model import estimate_flight

# ---------------------------------------------------------------------------
# Rejection reason tokens
# ---------------------------------------------------------------------------
VERTIPORT_MISMATCH = "vertiport_mismatch"
WIND_TOO_HIGH = "wind_speed_too_high"
WEIGHT_EXCEEDED = "weight_limit_exceeded"
SIZE_INCOMPATIBLE = "drone_size_incompatible"
ALTITUDE_EXCEEDED = "altitude_exceeded"
SAFETY_SCORE_INSUFFICIENT = "safety_score_insufficient"
INSUFFICIENT_BATTERY = "insufficient_battery"

# ---------------------------------------------------------------------------
# Internal safety policy (not exposed to API callers)
# ---------------------------------------------------------------------------
# Minimum route safety_score at calm conditions (no wind penalty)
_SAFETY_FLOOR = 0.50
# Required safety_score when route wind equals the drone's maximum wind tolerance
_SAFETY_CEILING = 0.90


def suggest_routes(
    request: RouteSuggestionRequest,
    available_routes: List[FlightRoute],
) -> RouteSuggestionResult:
    """Return routes compatible with *request*, each paired with a flight estimate.

    Routes that fail any check are placed in *rejected_routes* with the first
    failing reason.  Compatible routes are in *suggested_routes*.
    """
    suggested: List[SuggestedRoute] = []
    rejected: List[RouteRejection] = []

    for route in available_routes:
        # Stage 1 — hard eligibility
        reason = _eligibility_rejection(route, request)
        if reason is not None:
            rejected.append(RouteRejection(route=route, reason=reason))
            continue

        # Stage 2 — internal safety viability
        min_safety = _min_required_safety_score(route, request.drone)
        if route.safety_score < min_safety:
            rejected.append(RouteRejection(route=route, reason=SAFETY_SCORE_INSUFFICIENT))
            continue

        # Stage 3 — energy viability
        flight_est = estimate_flight(request.drone, route)
        if flight_est.SoC_remaining < request.drone.soc_min:
            rejected.append(RouteRejection(route=route, reason=INSUFFICIENT_BATTERY))
            continue

        suggested.append(SuggestedRoute(route=route, estimate=flight_est))

    return RouteSuggestionResult(suggested_routes=suggested, rejected_routes=rejected)


# ---------------------------------------------------------------------------
# Stage 1 — hard eligibility
# ---------------------------------------------------------------------------

def _eligibility_rejection(
    route: FlightRoute,
    request: RouteSuggestionRequest,
) -> Optional[str]:
    """Return the first hard eligibility rejection reason, or None if eligible."""
    if (
        route.take_off_vertiport_id != request.take_off_vertiport_id
        or route.landing_vertiport_id != request.landing_vertiport_id
    ):
        return VERTIPORT_MISMATCH

    drone = request.drone

    if route.average_wind_speed > drone.max_wind_resistance_ms:
        return WIND_TOO_HIGH

    if drone.mtow_kg > route.max_drone_weight_kg:
        return WEIGHT_EXCEEDED

    if drone.drone_size not in route.compatible_drone_sizes:
        return SIZE_INCOMPATIBLE

    # Altitude check: no waypoint may exceed the drone's service ceiling
    if drone.service_ceiling_m > 0 and route.waypoints:
        max_alt = max(wp.position[2] for wp in route.waypoints)
        if max_alt > drone.service_ceiling_m:
            return ALTITUDE_EXCEEDED

    return None


# ---------------------------------------------------------------------------
# Stage 2 — internal safety policy
# ---------------------------------------------------------------------------

def _min_required_safety_score(route: FlightRoute, drone: DroneProfile) -> float:
    """Compute the minimum route safety_score required for this combination.

    The closer the route wind is to the drone's wind resistance limit, the
    higher the required safety score.  This ensures that challenging conditions
    are served only by well-rated corridors.

    Returns a value in [_SAFETY_FLOOR, _SAFETY_CEILING].
    """
    if drone.max_wind_resistance_ms <= 0:
        return _SAFETY_CEILING
    wind_fraction = min(1.0, route.average_wind_speed / drone.max_wind_resistance_ms)
    return _SAFETY_FLOOR + (_SAFETY_CEILING - _SAFETY_FLOOR) * wind_fraction
