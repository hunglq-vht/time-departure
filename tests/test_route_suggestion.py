"""Tests for the flight route suggestion feature, including power estimation."""
from __future__ import annotations

import math

import pytest

from scrp.models import (
    DroneProfile,
    FlightRoute,
    RouteFlightEstimate,
    RouteSuggestionRequest,
    SuggestedRoute,
    Waypoint,
)
from scrp.route_suggestion import (
    INSUFFICIENT_BATTERY,
    SIZE_INCOMPATIBLE,
    SAFETY_TOO_LOW,
    VERTIPORT_MISMATCH,
    WEIGHT_EXCEEDED,
    WIND_TOO_HIGH,
    suggest_routes,
    _estimate_flight,
    _clamp,
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
    segment_length_m: float = 1000.0,
) -> FlightRoute:
    if sizes is None:
        sizes = ["A", "B", "C"]
    wp1 = Waypoint(id=f"{route_id}_w1", position=(0.0, 0.0, 50.0))
    wp2 = Waypoint(id=f"{route_id}_w2", position=(segment_length_m, 0.0, 50.0))
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
    wind_resistance: float = 15.0,
    size: str = "A",
    cruise_speed: float = 10.0,
    P_hover: float = 500.0,
    C_bat: float = 1000.0,
    SoC_0: float = 1.0,
    cruise_power_factor: float = 1.2,
    t_takeoff_s: float = 30.0,
    t_landing_s: float = 30.0,
    SoC_min: float = 0.20,
) -> DroneProfile:
    return DroneProfile(
        weight_kg=weight,
        max_wind_resistance=wind_resistance,
        drone_size=size,
        cruise_speed=cruise_speed,
        P_hover=P_hover,
        C_bat=C_bat,
        SoC_0=SoC_0,
        cruise_power_factor=cruise_power_factor,
        t_takeoff_s=t_takeoff_s,
        t_landing_s=t_landing_s,
        SoC_min=SoC_min,
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
# _clamp helper
# ---------------------------------------------------------------------------

def test_clamp_below_range():
    assert _clamp(3.0, 5.0, 20.0) == 5.0


def test_clamp_above_range():
    assert _clamp(25.0, 5.0, 20.0) == 20.0


def test_clamp_within_range():
    assert _clamp(10.0, 5.0, 20.0) == 10.0


# ---------------------------------------------------------------------------
# _estimate_flight — zero wind baseline
# ---------------------------------------------------------------------------

def test_estimate_zero_wind_cruise_time():
    # 1000 m at 10 m/s → 100 s cruise
    route = _make_route(wind=0.0, segment_length_m=1000.0)
    drone = _make_drone(cruise_speed=10.0, t_takeoff_s=0.0, t_landing_s=0.0)
    est = _estimate_flight(route, drone)
    assert est.cruise_time_s == pytest.approx(100.0)


def test_estimate_zero_wind_total_time_includes_phases():
    route = _make_route(wind=0.0, segment_length_m=1000.0)
    drone = _make_drone(cruise_speed=10.0, t_takeoff_s=30.0, t_landing_s=20.0)
    est = _estimate_flight(route, drone)
    # 100 s cruise + 30 takeoff + 20 landing = 150 s
    assert est.total_time_s == pytest.approx(150.0)


def test_estimate_zero_wind_energy():
    # P_cruise = 500 * 1.2 = 600 W, 100 s cruise → 600 * 100 / 3600 = 16.667 Wh
    # E_takeoff = 500 * 0 / 3600 = 0, E_landing = 0
    route = _make_route(wind=0.0, segment_length_m=1000.0)
    drone = _make_drone(
        P_hover=500.0, cruise_power_factor=1.2, cruise_speed=10.0,
        t_takeoff_s=0.0, t_landing_s=0.0,
    )
    est = _estimate_flight(route, drone)
    expected = 600.0 * 100.0 / 3600.0
    assert est.energy_consumed_wh == pytest.approx(expected, rel=1e-6)


def test_estimate_zero_wind_soc_remaining():
    route = _make_route(wind=0.0, segment_length_m=1000.0)
    drone = _make_drone(
        P_hover=500.0, C_bat=1000.0, SoC_0=1.0,
        cruise_power_factor=1.2, cruise_speed=10.0,
        t_takeoff_s=0.0, t_landing_s=0.0,
    )
    est = _estimate_flight(route, drone)
    expected_e = 600.0 * 100.0 / 3600.0  # ~16.667 Wh
    expected_soc = 1.0 - expected_e / 1000.0
    assert est.SoC_remaining == pytest.approx(expected_soc, rel=1e-6)


# ---------------------------------------------------------------------------
# _estimate_flight — wind penalty
# ---------------------------------------------------------------------------

def test_estimate_wind_penalty_increases_energy():
    route_no_wind = _make_route(wind=0.0)
    route_with_wind = _make_route(wind=5.0)
    drone = _make_drone(cruise_speed=10.0, t_takeoff_s=0.0, t_landing_s=0.0)
    est_no_wind = _estimate_flight(route_no_wind, drone)
    est_with_wind = _estimate_flight(route_with_wind, drone)
    assert est_with_wind.energy_consumed_wh > est_no_wind.energy_consumed_wh


def test_estimate_wind_equal_to_cruise_speed_doubles_cruise_factor():
    # wind = cruise_speed → wind_penalty = 1.0
    # P_cruise_effective = P_hover * cpf * 2
    route = _make_route(wind=10.0, segment_length_m=1000.0)
    drone = _make_drone(
        cruise_speed=10.0, P_hover=500.0, cruise_power_factor=1.2,
        t_takeoff_s=0.0, t_landing_s=0.0,
    )
    est = _estimate_flight(route, drone)
    # P_effective = 500 * 1.2 * (1 + 1) = 1200 W, 100 s → 1200*100/3600 Wh
    expected = 1200.0 * 100.0 / 3600.0
    assert est.energy_consumed_wh == pytest.approx(expected, rel=1e-6)


def test_estimate_wind_does_not_change_cruise_time():
    # Wind increases power but not travel time (airspeed stays at cruise_speed)
    route_no_wind = _make_route(wind=0.0)
    route_with_wind = _make_route(wind=8.0)
    drone = _make_drone(cruise_speed=10.0, t_takeoff_s=0.0, t_landing_s=0.0)
    est_no_wind = _estimate_flight(route_no_wind, drone)
    est_with_wind = _estimate_flight(route_with_wind, drone)
    assert est_with_wind.cruise_time_s == pytest.approx(est_no_wind.cruise_time_s)


# ---------------------------------------------------------------------------
# _estimate_flight — takeoff/landing energy
# ---------------------------------------------------------------------------

def test_estimate_phase_energy_at_hover_power():
    # Only phases, no segments
    route = _make_route(wind=0.0, segment_length_m=0.0)
    # Force segments to be empty
    route.segments = []
    route.waypoints = []
    drone = _make_drone(P_hover=400.0, t_takeoff_s=60.0, t_landing_s=30.0)
    est = _estimate_flight(route, drone)
    expected_e = 400.0 * (60.0 + 30.0) / 3600.0
    assert est.energy_consumed_wh == pytest.approx(expected_e, rel=1e-6)
    assert est.total_time_s == pytest.approx(90.0)
    assert est.cruise_time_s == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# _estimate_flight — speed clamping
# ---------------------------------------------------------------------------

def test_estimate_cruise_speed_clamped_to_segment_v_max():
    # drone cruise_speed > segment v_max → actual speed = v_max
    route = _make_route(wind=0.0, segment_length_m=1000.0)
    # Override segment bounds (auto-generated segments have v_min=5, v_max=20)
    route.segments[0].v_max = 8.0
    drone = _make_drone(cruise_speed=15.0, t_takeoff_s=0.0, t_landing_s=0.0)
    est = _estimate_flight(route, drone)
    # 1000 m at clamped 8 m/s → 125 s
    assert est.cruise_time_s == pytest.approx(125.0)


def test_estimate_cruise_speed_clamped_to_segment_v_min():
    route = _make_route(wind=0.0, segment_length_m=1000.0)
    route.segments[0].v_min = 12.0
    drone = _make_drone(cruise_speed=5.0, t_takeoff_s=0.0, t_landing_s=0.0)
    est = _estimate_flight(route, drone)
    # 1000 m at clamped 12 m/s → ~83.33 s
    assert est.cruise_time_s == pytest.approx(1000.0 / 12.0, rel=1e-6)


# ---------------------------------------------------------------------------
# Empty route list
# ---------------------------------------------------------------------------

def test_no_routes_returns_empty_result():
    result = suggest_routes(_make_request(), [])
    assert result.suggested_routes == []
    assert result.rejected_routes == []


# ---------------------------------------------------------------------------
# Happy path — compatible routes returned as SuggestedRoute
# ---------------------------------------------------------------------------

def test_single_compatible_route_returns_suggested_route():
    route = _make_route()
    result = suggest_routes(_make_request(), [route])
    assert len(result.suggested_routes) == 1
    assert isinstance(result.suggested_routes[0], SuggestedRoute)
    assert result.suggested_routes[0].route.route_id == "r1"
    assert result.suggested_routes[0].estimate is not None
    assert result.rejected_routes == []


def test_estimate_attached_to_suggested_route():
    route = _make_route(wind=0.0, segment_length_m=2000.0)
    drone = _make_drone(cruise_speed=10.0, t_takeoff_s=30.0, t_landing_s=30.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    est = result.suggested_routes[0].estimate
    # 2000 m / 10 m/s = 200 s cruise; total = 200 + 30 + 30 = 260 s
    assert est.cruise_time_s == pytest.approx(200.0)
    assert est.total_time_s == pytest.approx(260.0)
    assert est.energy_consumed_wh > 0
    assert 0.0 < est.SoC_remaining < 1.0


def test_multiple_compatible_routes():
    routes = [_make_route(route_id=f"r{i}") for i in range(3)]
    result = suggest_routes(_make_request(), routes)
    assert len(result.suggested_routes) == 3
    assert result.rejected_routes == []


# ---------------------------------------------------------------------------
# Vertiport mismatch
# ---------------------------------------------------------------------------

def test_wrong_take_off_vertiport_rejected():
    route = _make_route(take_off="vp_x")
    result = suggest_routes(_make_request(take_off="vp_a"), [route])
    assert result.suggested_routes == []
    assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH


def test_wrong_landing_vertiport_rejected():
    route = _make_route(landing="vp_z")
    result = suggest_routes(_make_request(landing="vp_b"), [route])
    assert result.suggested_routes == []
    assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH


# ---------------------------------------------------------------------------
# Wind resistance
# ---------------------------------------------------------------------------

def test_route_wind_exceeds_drone_resistance_rejected():
    route = _make_route(wind=20.0)
    drone = _make_drone(wind_resistance=15.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.suggested_routes == []
    assert result.rejected_routes[0].reason == WIND_TOO_HIGH


def test_route_wind_exactly_at_limit_accepted():
    route = _make_route(wind=10.0)
    drone = _make_drone(wind_resistance=10.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.suggested_routes) == 1


# ---------------------------------------------------------------------------
# Weight limit
# ---------------------------------------------------------------------------

def test_drone_overweight_rejected():
    route = _make_route(max_weight=10.0)
    drone = _make_drone(weight=15.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.suggested_routes == []
    assert result.rejected_routes[0].reason == WEIGHT_EXCEEDED


def test_drone_weight_at_limit_accepted():
    route = _make_route(max_weight=10.0)
    drone = _make_drone(weight=10.0)
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.suggested_routes) == 1


# ---------------------------------------------------------------------------
# Drone size
# ---------------------------------------------------------------------------

def test_incompatible_drone_size_rejected():
    route = _make_route(sizes=["B", "C"])
    drone = _make_drone(size="A")
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.suggested_routes == []
    assert result.rejected_routes[0].reason == SIZE_INCOMPATIBLE


def test_compatible_drone_size_accepted():
    route = _make_route(sizes=["A", "B"])
    drone = _make_drone(size="A")
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.suggested_routes) == 1


# ---------------------------------------------------------------------------
# Safety score threshold
# ---------------------------------------------------------------------------

def test_route_below_min_safety_rejected():
    route = _make_route(safety=0.5)
    result = suggest_routes(_make_request(min_safety=0.8), [route])
    assert result.suggested_routes == []
    assert result.rejected_routes[0].reason == SAFETY_TOO_LOW


def test_route_at_min_safety_accepted():
    route = _make_route(safety=0.8)
    result = suggest_routes(_make_request(min_safety=0.8), [route])
    assert len(result.suggested_routes) == 1


# ---------------------------------------------------------------------------
# Insufficient battery
# ---------------------------------------------------------------------------

def test_insufficient_battery_rejected():
    # Very long route that drains the battery below SoC_min
    route = _make_route(wind=0.0, segment_length_m=500_000.0)  # 500 km
    drone = _make_drone(
        P_hover=500.0, C_bat=10.0,  # tiny battery
        SoC_0=1.0, cruise_speed=10.0,
        SoC_min=0.20,
    )
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.suggested_routes == []
    assert result.rejected_routes[0].reason == INSUFFICIENT_BATTERY


def test_sufficient_battery_just_above_min_accepted():
    # Construct a route whose energy leaves SoC_remaining just above SoC_min.
    # E = P_cruise * (L / cruise_speed) / 3600  →  L = E * 3600 * cruise_speed / P_cruise
    C_bat = 100.0
    P_hover = 500.0
    cpf = 1.2
    cruise_speed = 10.0
    P_cruise = P_hover * cpf          # 600 W
    # Target: consume 79 Wh → SoC_remaining = 0.21 (well above SoC_min=0.20)
    E_consumed_target = 79.0
    L = E_consumed_target * 3600.0 * cruise_speed / P_cruise
    route = _make_route(wind=0.0, segment_length_m=L)
    drone = _make_drone(
        P_hover=P_hover, C_bat=C_bat, SoC_0=1.0,
        cruise_power_factor=cpf, cruise_speed=cruise_speed,
        t_takeoff_s=0.0, t_landing_s=0.0,
        SoC_min=0.20,
    )
    result = suggest_routes(_make_request(drone=drone), [route])
    assert len(result.suggested_routes) == 1
    assert result.suggested_routes[0].estimate.SoC_remaining == pytest.approx(0.21, rel=1e-6)


# ---------------------------------------------------------------------------
# Priority: vertiport mismatch checked before battery
# ---------------------------------------------------------------------------

def test_vertiport_mismatch_takes_priority_over_battery():
    route = _make_route(take_off="vp_wrong", segment_length_m=999_999.0)
    drone = _make_drone(C_bat=0.001)  # would definitely fail battery
    result = suggest_routes(_make_request(drone=drone), [route])
    assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH


# ---------------------------------------------------------------------------
# Mixed compatible and rejected routes
# ---------------------------------------------------------------------------

def test_mixed_routes_split_correctly():
    drone = _make_drone(weight=10.0, wind_resistance=8.0, size="B")
    routes = [
        _make_route(route_id="ok1"),
        _make_route(route_id="wind_fail", wind=20.0),
        _make_route(route_id="ok2"),
        _make_route(route_id="weight_fail", max_weight=5.0),
        _make_route(route_id="ok3"),
    ]
    result = suggest_routes(_make_request(drone=drone), routes)
    assert len(result.suggested_routes) == 3
    assert len(result.rejected_routes) == 2
    compatible_ids = {sr.route.route_id for sr in result.suggested_routes}
    assert compatible_ids == {"ok1", "ok2", "ok3"}
    rejected_reasons = {rj.route.route_id: rj.reason for rj in result.rejected_routes}
    assert rejected_reasons["wind_fail"] == WIND_TOO_HIGH
    assert rejected_reasons["weight_fail"] == WEIGHT_EXCEEDED


# ---------------------------------------------------------------------------
# Routes sharing the same vertiport pair
# ---------------------------------------------------------------------------

def test_multiple_routes_same_vertiport_pair_different_safety():
    drone = _make_drone()
    routes = [
        _make_route(route_id="scenic", wind=3.0, safety=0.95),
        _make_route(route_id="express", wind=4.0, safety=0.85),
        _make_route(route_id="risky", wind=4.0, safety=0.4),
    ]
    result = suggest_routes(_make_request(drone=drone, min_safety=0.8), routes)
    compatible_ids = {sr.route.route_id for sr in result.suggested_routes}
    assert compatible_ids == {"scenic", "express"}
    assert result.rejected_routes[0].route.route_id == "risky"
    assert result.rejected_routes[0].reason == SAFETY_TOO_LOW


# ---------------------------------------------------------------------------
# FlightRoute auto-builds segments
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
    assert abs(route.segments[0].length - 500.0) < 0.01  # 3-4-5 scaled by 100


# ---------------------------------------------------------------------------
# Multi-segment route energy sums correctly
# ---------------------------------------------------------------------------

def test_multi_segment_energy_sums_over_all_segments():
    wp1 = Waypoint(id="w1", position=(0.0, 0.0, 50.0))
    wp2 = Waypoint(id="w2", position=(1000.0, 0.0, 50.0))
    wp3 = Waypoint(id="w3", position=(2000.0, 0.0, 50.0))
    route = FlightRoute(
        route_id="multi",
        waypoints=[wp1, wp2, wp3],
        take_off_vertiport_id="vp_a",
        landing_vertiport_id="vp_b",
        safety_score=1.0,
        average_wind_speed=0.0,
        max_drone_weight_kg=50.0,
        compatible_drone_sizes=["A"],
    )
    drone = _make_drone(cruise_speed=10.0, P_hover=500.0, cruise_power_factor=1.2,
                        t_takeoff_s=0.0, t_landing_s=0.0)
    est = _estimate_flight(route, drone)
    # 2 segments × 1000 m / 10 m/s = 200 s total cruise
    assert est.cruise_time_s == pytest.approx(200.0)
    expected_e = 600.0 * 200.0 / 3600.0
    assert est.energy_consumed_wh == pytest.approx(expected_e, rel=1e-6)
