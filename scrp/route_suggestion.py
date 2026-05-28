"""Flight route suggestion: filter available routes for a given drone and vertiport pair.

This module is a pure function with no side effects.  The caller is responsible
for fetching the list of available routes from the external route service (see
flight_route_client.py) before invoking suggest_routes().
"""
from __future__ import annotations

from typing import List, Optional

from .models import (
    DroneProfile,
    FlightRoute,
    RouteRejection,
    RouteSuggestionRequest,
    RouteSuggestionResult,
)

# Human-readable rejection reason tokens
VERTIPORT_MISMATCH = "vertiport_mismatch"
WIND_TOO_HIGH = "wind_speed_too_high"
WEIGHT_EXCEEDED = "weight_limit_exceeded"
SIZE_INCOMPATIBLE = "drone_size_incompatible"
SAFETY_TOO_LOW = "safety_below_threshold"


def suggest_routes(
    request: RouteSuggestionRequest,
    available_routes: List[FlightRoute],
) -> RouteSuggestionResult:
    """Return routes from *available_routes* that are compatible with *request*.

    Each route is checked in order against four criteria:
    1. Vertiport endpoints must match exactly.
    2. Route average wind speed must not exceed the drone's wind resistance.
    3. Drone weight must not exceed the route's weight limit.
    4. Drone size must be listed as compatible by the route.
    5. Route safety score must meet the caller's minimum threshold.

    Routes that fail any check are collected in *rejected_routes* with the
    first failing reason; compatible routes are in *compatible_routes*.
    """
    compatible: List[FlightRoute] = []
    rejected: List[RouteRejection] = []

    for route in available_routes:
        reason = _rejection_reason(route, request)
        if reason is None:
            compatible.append(route)
        else:
            rejected.append(RouteRejection(route=route, reason=reason))

    return RouteSuggestionResult(
        compatible_routes=compatible,
        rejected_routes=rejected,
    )


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _rejection_reason(
    route: FlightRoute,
    request: RouteSuggestionRequest,
) -> Optional[str]:
    """Return the first rejection reason for *route*, or None if compatible."""
    if (
        route.take_off_vertiport_id != request.take_off_vertiport_id
        or route.landing_vertiport_id != request.landing_vertiport_id
    ):
        return VERTIPORT_MISMATCH

    drone: DroneProfile = request.drone

    if drone.max_wind_resistance < route.average_wind_speed:
        return WIND_TOO_HIGH

    if drone.weight_kg > route.max_drone_weight_kg:
        return WEIGHT_EXCEEDED

    if drone.drone_size not in route.compatible_drone_sizes:
        return SIZE_INCOMPATIBLE

    if route.safety_score < request.min_safety_score:
        return SAFETY_TOO_LOW

    return None
