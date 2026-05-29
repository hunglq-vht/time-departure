"""Flight route suggestion: filter available routes and estimate power consumption.

This module is a pure function with no side effects.  The caller is responsible
for fetching the list of available routes from the external route service (see
flight_route_client.py) before invoking suggest_routes().

Energy model
------------
Cruise power is modelled as:

    P_cruise = P_hover * cruise_power_factor * (1 + wind_penalty)

where:

    wind_penalty = (average_wind_speed / cruise_speed) ** 2

This follows from aerodynamic drag scaling with airspeed squared: flying against
a headwind equal to the cruise speed requires roughly double the cruise power.
For a two-way trip the average headwind/tailwind effect collapses to this
quadratic penalty on the mean wind magnitude.

Each flight phase burns energy at its own power level:

    E_takeoff  = P_hover   * t_takeoff_s  / 3600   [Wh]
    E_cruise   = P_cruise  * cruise_time_s / 3600  [Wh]   (sum across segments)
    E_landing  = P_hover   * t_landing_s  / 3600   [Wh]

    SoC_remaining = SoC_0 - (E_takeoff + E_cruise + E_landing) / C_bat

A route is rejected as 'insufficient_battery' when SoC_remaining < drone.SoC_min.
"""
from __future__ import annotations

from typing import List, Optional

from .models import (
    DroneProfile,
    FlightRoute,
    RouteFlightEstimate,
    RouteRejection,
    RouteSuggestionRequest,
    RouteSuggestionResult,
    SuggestedRoute,
)

# Rejection reason tokens
VERTIPORT_MISMATCH = "vertiport_mismatch"
WIND_TOO_HIGH = "wind_speed_too_high"
WEIGHT_EXCEEDED = "weight_limit_exceeded"
SIZE_INCOMPATIBLE = "drone_size_incompatible"
SAFETY_TOO_LOW = "safety_below_threshold"
INSUFFICIENT_BATTERY = "insufficient_battery"


def suggest_routes(
    request: RouteSuggestionRequest,
    available_routes: List[FlightRoute],
) -> RouteSuggestionResult:
    """Return routes compatible with *request*, each paired with flight estimates.

    Each route is checked in order:
    1. Vertiport endpoints must match exactly.
    2. Route average wind speed must not exceed the drone's wind resistance.
    3. Drone weight must not exceed the route's weight limit.
    4. Drone size must appear in the route's compatible_drone_sizes list.
    5. Route safety score must meet the caller's minimum threshold.
    6. Estimated SoC on arrival must be >= drone.SoC_min.

    Compatible routes are returned in *suggested_routes*, each bundled with a
    RouteFlightEstimate containing cruise_time_s, total_time_s, energy_consumed_wh,
    and SoC_remaining.  Incompatible routes are in *rejected_routes* with the
    first failing reason.
    """
    suggested: List[SuggestedRoute] = []
    rejected: List[RouteRejection] = []

    for route in available_routes:
        eligibility_reason = _eligibility_rejection(route, request)
        if eligibility_reason is not None:
            rejected.append(RouteRejection(route=route, reason=eligibility_reason))
            continue

        estimate = _estimate_flight(route, request.drone)
        if estimate.SoC_remaining < request.drone.SoC_min:
            rejected.append(RouteRejection(route=route, reason=INSUFFICIENT_BATTERY))
            continue

        suggested.append(SuggestedRoute(route=route, estimate=estimate))

    return RouteSuggestionResult(suggested_routes=suggested, rejected_routes=rejected)


# ---------------------------------------------------------------------------
# Eligibility filter (no energy involved)
# ---------------------------------------------------------------------------

def _eligibility_rejection(
    route: FlightRoute,
    request: RouteSuggestionRequest,
) -> Optional[str]:
    """Return the first non-energy rejection reason, or None if eligible."""
    if (
        route.take_off_vertiport_id != request.take_off_vertiport_id
        or route.landing_vertiport_id != request.landing_vertiport_id
    ):
        return VERTIPORT_MISMATCH

    drone = request.drone

    if drone.max_wind_resistance < route.average_wind_speed:
        return WIND_TOO_HIGH

    if drone.weight_kg > route.max_drone_weight_kg:
        return WEIGHT_EXCEEDED

    if drone.drone_size not in route.compatible_drone_sizes:
        return SIZE_INCOMPATIBLE

    if route.safety_score < request.min_safety_score:
        return SAFETY_TOO_LOW

    return None


# ---------------------------------------------------------------------------
# Power / time estimation
# ---------------------------------------------------------------------------

def _estimate_flight(route: FlightRoute, drone: DroneProfile) -> RouteFlightEstimate:
    """Compute time and energy estimates for *drone* flying *route*.

    Cruise speed per segment is clamped to the segment's [v_min, v_max] range
    so we never compute infeasibly fast or slow travel times.

    Wind increases effective cruise power via the quadratic drag penalty:
        wind_penalty = (average_wind_speed / cruise_speed) ** 2
    When average_wind_speed is zero the formula reduces to the standard
        P_cruise = P_hover * cruise_power_factor.
    """
    wind_penalty = (route.average_wind_speed / drone.cruise_speed) ** 2
    P_cruise = drone.P_hover * drone.cruise_power_factor * (1.0 + wind_penalty)

    cruise_time_s = 0.0
    E_cruise = 0.0

    for seg in route.segments:
        v = _clamp(drone.cruise_speed, seg.v_min, seg.v_max)
        t_seg = seg.length / v
        cruise_time_s += t_seg
        E_cruise += P_cruise * t_seg / 3600.0

    E_takeoff = drone.P_hover * drone.t_takeoff_s / 3600.0
    E_landing = drone.P_hover * drone.t_landing_s / 3600.0
    energy_consumed_wh = E_takeoff + E_cruise + E_landing

    total_time_s = drone.t_takeoff_s + cruise_time_s + drone.t_landing_s
    SoC_remaining = drone.SoC_0 - energy_consumed_wh / drone.C_bat

    return RouteFlightEstimate(
        cruise_time_s=cruise_time_s,
        total_time_s=total_time_s,
        energy_consumed_wh=energy_consumed_wh,
        SoC_remaining=SoC_remaining,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
