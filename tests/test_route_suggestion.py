"""Tests for the flight route suggestion feature."""
from __future__ import annotations

import pytest

from scrp.models import (
    DroneProfile,
    FlightRoute,
    RouteRejection,
    RouteSuggestionRequest,
    RouteSuggestionResult,
    Waypoint,
)
from scrp.route_suggestion import (
    SIZE_INCOMPATIBLE,
    SAFETY_TOO_LOW,
    VERTIPORT_MISMATCH,
    WEIGHT_EXCEEDED,
    WIND_TOO_HIGH,
    suggest_routes,
)

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

def _make_route(
    route_id: str = "r1",
    take_off: str = "vp_a",
    landing: str = "vp_b",
    safety: float = 0.9,
    wind: float = 5.0,
    max_weight: float = 25.0,
    sizes: list[str] | None = None,
) -> FlightRoute:
    if sizes is None:
        sizes = ["A", "B", "C"]
    wp1 = Waypoint(id=f"{route_id}_w1", position=(0.0, 0.0, 50.0))
    wp2 = Waypoint(id=f"{route_id}_w2", position=(100.0, 0.0, 50.0))
    return FlightRoute(
        route_id=route_id,
        waypoints=[wp1, wp2],
        take_off_vertiport_id=take_off,
        landing_vertiport_id=landing,
        safety_score=safety,
        average_wind_speed=wind,
        max_drone_weight_kg=max_weight,
        compatible_drone_sizes=sizes,
    )


def _make_drone(
    weight: float = 10.0,
    wind_resistance: float = 10.0,
    size: str = "A",
) -> DroneProfile:
    return DroneProfile(
        weight_kg=weight,
        max_wind_resistance=wind_resistance,
        drone_size=size,
    )


def _make_request(
    take_off: str = "vp_a",
    landing: str = "vp_b",
    drone: DroneProfile | None = None,
    min_safety: float = 0.0,
) -> RouteSuggestionRequest:
    return RouteSuggestionRequest(
        take_off_vertiport_id=take_off,
        landing_vertiport_id=landing,
        drone=drone or _make_drone(),
        min_safety_score=min_safety,
    )


# ---------------------------------------------------------------------------
# Empty route list
# ---------------------------------------------------------------------------

def test_no_routes_returns_empty_result():
    result = suggest_routes(_make_request(), [])
    assert result.compatible_routes == []
    assert result.rejected_routes == []


# ---------------------------------------------------------------------------
# Happy path
# ---------------------------------------------------------------------------

def test_single_compatible_route():
    route = _make_route()
    result = suggest_routes(_make_request(), [route])
    assert len(result.compatible_routes) == 1
    assert result.compatible_routes[0].route_id == "r1"
    assert result.rejected_routes == []


def test_multiple_compatible_routes():
    routes = [_make_route(route_id=f"r{i}") for i in range(3)]
    result = suggest_routes(_make_request(), routes)
    assert len(result.compatible_routes) == 3
    assert result.rejected_routes == []


# ---------------------------------------------------------------------------
# Vertiport mismatch
# ---------------------------------------------------------------------------

def test_wrong_take_off_vertiport_rejected():
    route = _make_route(take_off="vp_x")
    result = suggest_routes(_make_request(take_off="vp_a"), [route])
    assert result.compatible_routes == []
    assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH


def test_wrong_landing_vertiport_rejected():
    route = _make_route(landing="vp_z")
    result = suggest_routes(_make_request(landing="vp_b"), [route])
    assert result.compatible_routes == []
    assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH


# ---------------------------------------------------------------------------
# Wind speed
# ---------------------------------------------------------------------------

def test_route_wind_exceeds_drone_resistance_rejected():
    route = _make_route(wind=15.0)
    drone = _make_drone(wind_resistance=10.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.compatible_routes == []
    assert result.rejected_routes[0].reason == WIND_TOO_HIGH


def test_route_wind_exactly_at_limit_accepted():
    route = _make_route(wind=10.0)
    drone = _make_drone(wind_resistance=10.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.compatible_routes) == 1


def test_route_wind_below_limit_accepted():
    route = _make_route(wind=5.0)
    drone = _make_drone(wind_resistance=10.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.compatible_routes) == 1


# ---------------------------------------------------------------------------
# Drone weight
# ---------------------------------------------------------------------------

def test_drone_overweight_rejected():
    route = _make_route(max_weight=10.0)
    drone = _make_drone(weight=15.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.compatible_routes == []
    assert result.rejected_routes[0].reason == WEIGHT_EXCEEDED


def test_drone_weight_at_limit_accepted():
    route = _make_route(max_weight=10.0)
    drone = _make_drone(weight=10.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.compatible_routes) == 1


def test_drone_weight_below_limit_accepted():
    route = _make_route(max_weight=25.0)
    drone = _make_drone(weight=10.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.compatible_routes) == 1


# ---------------------------------------------------------------------------
# Drone size
# ---------------------------------------------------------------------------

def test_incompatible_drone_size_rejected():
    route = _make_route(sizes=["B", "C"])
    drone = _make_drone(size="A")
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.compatible_routes == []
    assert result.rejected_routes[0].reason == SIZE_INCOMPATIBLE


def test_compatible_drone_size_accepted():
    route = _make_route(sizes=["A", "B"])
    drone = _make_drone(size="A")
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.compatible_routes) == 1


# ---------------------------------------------------------------------------
# Safety score threshold
# ---------------------------------------------------------------------------

def test_route_below_min_safety_rejected():
    route = _make_route(safety=0.5)
    result = suggest_routes(_make_request(min_safety=0.8), [route])
    assert result.compatible_routes == []
    assert result.rejected_routes[0].reason == SAFETY_TOO_LOW


def test_route_at_min_safety_accepted():
    route = _make_route(safety=0.8)
    result = suggest_routes(_make_request(min_safety=0.8), [route])
    assert len(result.compatible_routes) == 1


def test_route_above_min_safety_accepted():
    route = _make_route(safety=0.95)
    result = suggest_routes(_make_request(min_safety=0.8), [route])
    assert len(result.compatible_routes) == 1


# ---------------------------------------------------------------------------
# Priority of rejection reasons (vertiport mismatch checked first)
# ---------------------------------------------------------------------------

def test_vertiport_mismatch_takes_priority_over_other_failures():
    # Route fails everything, but vertiport mismatch should be the reported reason
    route = _make_route(
        take_off="vp_wrong",
        wind=999.0,
        max_weight=0.1,
        sizes=["C"],
        safety=0.0,
    )
    drone = _make_drone(size="A", weight=10.0, wind_resistance=5.0)
    result = suggest_routes(_make_request(drone=drone, min_safety=1.0), [route])
    assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH


# ---------------------------------------------------------------------------
# Mixed compatible and rejected routes
# ---------------------------------------------------------------------------

def test_mixed_routes_split_correctly():
    drone = _make_drone(weight=10.0, wind_resistance=8.0, size="B")
    routes = [
        _make_route(route_id="ok1"),                       # compatible
        _make_route(route_id="wind_fail", wind=20.0),      # wind too high
        _make_route(route_id="ok2"),                       # compatible
        _make_route(route_id="weight_fail", max_weight=5.0),  # overweight
        _make_route(route_id="ok3"),                       # compatible
    ]
    result = suggest_routes(_make_request(drone=drone), routes)
    assert len(result.compatible_routes) == 3
    assert len(result.rejected_routes) == 2
    compatible_ids = {r.route_id for r in result.compatible_routes}
    assert compatible_ids == {"ok1", "ok2", "ok3"}
    rejected_reasons = {rj.route.route_id: rj.reason for rj in result.rejected_routes}
    assert rejected_reasons["wind_fail"] == WIND_TOO_HIGH
    assert rejected_reasons["weight_fail"] == WEIGHT_EXCEEDED


# ---------------------------------------------------------------------------
# Routes sharing the same vertiport pair
# ---------------------------------------------------------------------------

def test_multiple_routes_same_vertiport_pair():
    """Vertiports can have many routes between them; all valid ones returned."""
    drone = _make_drone()
    routes = [
        _make_route(route_id="scenic", wind=3.0, safety=0.95),
        _make_route(route_id="express", wind=4.0, safety=0.85),
        _make_route(route_id="risky", wind=4.0, safety=0.4),
    ]
    result = suggest_routes(_make_request(drone=drone, min_safety=0.8), routes)
    compatible_ids = {r.route_id for r in result.compatible_routes}
    assert compatible_ids == {"scenic", "express"}
    assert result.rejected_routes[0].route.route_id == "risky"
    assert result.rejected_routes[0].reason == SAFETY_TOO_LOW


# ---------------------------------------------------------------------------
# Route auto-builds segments from waypoints
# ---------------------------------------------------------------------------

def test_flight_route_segments_auto_built():
    wp1 = Waypoint(id="w1", position=(0.0, 0.0, 50.0))
    wp2 = Waypoint(id="w2", position=(300.0, 400.0, 50.0))
    route = FlightRoute(
        route_id="seg_test",
        waypoints=[wp1, wp2],
        take_off_vertiport_id="vp_a",
        landing_vertiport_id="vp_b",
        safety_score=1.0,
        average_wind_speed=0.0,
        max_drone_weight_kg=50.0,
        compatible_drone_sizes=["A"],
    )
    assert len(route.segments) == 1
    assert abs(route.segments[0].length - 500.0) < 0.01  # 3-4-5 triangle * 100
