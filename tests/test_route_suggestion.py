"""Tests for flight route suggestion and physics-based power model.

DroneProfile now maps 1:1 to datasheet fields, so test fixtures use
realistic values derived from a notional medium-class UAV.
"""
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
from scrp.power_model import (
    AIR_DENSITY_SL,
    GRAVITY,
    FIGURE_OF_MERIT,
    MOMENTUM_THEORY_EFFICIENCY,
    CLIMB_POWER_FACTOR,
    DESCENT_POWER_FACTOR,
    cruise_power_w,
    hover_induced_velocity,
    hover_power_from_endurance,
    hover_power_from_momentum_theory,
    parasite_drag_area,
    resolve_hover_power,
    estimate_flight,
    _clamp,
)
from scrp.route_suggestion import (
    ALTITUDE_EXCEEDED,
    INSUFFICIENT_BATTERY,
    SAFETY_SCORE_INSUFFICIENT,
    SIZE_INCOMPATIBLE,
    VERTIPORT_MISMATCH,
    WEIGHT_EXCEEDED,
    WIND_TOO_HIGH,
    _min_required_safety_score,
    suggest_routes,
)

# ---------------------------------------------------------------------------
# Shared drone fixture — realistic medium-class UAV datasheet values
#   6-rotor, 0.38 m props, MTOW 9 kg, max speed 20 m/s, hover ~1500 W
# ---------------------------------------------------------------------------

def _make_drone(
    drone_size: str = "B",
    mtow_kg: float = 9.0,
    num_rotors: int = 6,
    propeller_diameter_m: float = 0.38,
    max_tilt_angle_deg: float = 30.0,
    max_speed_ms: float = 20.0,
    max_ascent_speed_ms: float = 5.0,
    max_descent_speed_ms: float = 3.0,
    max_wind_resistance_ms: float = 12.0,
    service_ceiling_m: float = 500.0,
    hover_power_w: float = 1500.0,   # explicit; set to 0.0 to test derivation
    battery_energy_wh: float = 300.0,
    flight_time_min: float = 0.0,
    soc_0: float = 1.0,
    soc_min: float = 0.20,
    cruise_speed_ms: float = 0.0,   # 0 → 75% of max_speed_ms = 15 m/s
    takeoff_height_m: float = 50.0,
    landing_height_m: float = 50.0,
) -> DroneProfile:
    return DroneProfile(
        drone_size=drone_size,
        mtow_kg=mtow_kg,
        num_rotors=num_rotors,
        propeller_diameter_m=propeller_diameter_m,
        max_tilt_angle_deg=max_tilt_angle_deg,
        max_speed_ms=max_speed_ms,
        max_ascent_speed_ms=max_ascent_speed_ms,
        max_descent_speed_ms=max_descent_speed_ms,
        max_wind_resistance_ms=max_wind_resistance_ms,
        service_ceiling_m=service_ceiling_m,
        battery_energy_wh=battery_energy_wh,
        hover_power_w=hover_power_w,
        flight_time_min=flight_time_min,
        soc_0=soc_0,
        soc_min=soc_min,
        cruise_speed_ms=cruise_speed_ms,
        takeoff_height_m=takeoff_height_m,
        landing_height_m=landing_height_m,
    )


def _make_route(
    route_id: str = "r1",
    take_off: str = "vp_a",
    landing: str = "vp_b",
    safety: float = 0.85,
    wind: float = 3.0,
    max_weight: float = 15.0,
    sizes: list[str] | None = None,
    segment_length_m: float = 5000.0,   # 5 km segment
    max_altitude_m: float = 80.0,
) -> FlightRoute:
    if sizes is None:
        sizes = ["A", "B", "C"]
    wp1 = Waypoint(id=f"{route_id}_w1", position=(0.0, 0.0, max_altitude_m))
    wp2 = Waypoint(id=f"{route_id}_w2", position=(segment_length_m, 0.0, max_altitude_m))
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


def _make_request(
    take_off: str = "vp_a",
    landing: str = "vp_b",
    drone: DroneProfile | None = None,
) -> RouteSuggestionRequest:
    return RouteSuggestionRequest(
        take_off_vertiport_id=take_off,
        landing_vertiport_id=landing,
        drone=drone or _make_drone(),
    )


# ===========================================================================
# power_model — unit tests
# ===========================================================================

class TestClamp:
    def test_below_range(self):
        assert _clamp(3.0, 5.0, 20.0) == 5.0

    def test_above_range(self):
        assert _clamp(25.0, 5.0, 20.0) == 20.0

    def test_within_range(self):
        assert _clamp(10.0, 5.0, 20.0) == 10.0


class TestHoverInducedVelocity:
    def test_formula(self):
        # v_i0 = sqrt(MTOW*g / (2*rho*N*pi*(D/2)^2))
        mtow, N, D = 9.0, 6, 0.38
        r = D / 2
        A = N * math.pi * r * r
        expected = math.sqrt(mtow * GRAVITY / (2 * AIR_DENSITY_SL * A))
        assert hover_induced_velocity(mtow, N, D) == pytest.approx(expected, rel=1e-9)

    def test_larger_disk_gives_lower_induced_velocity(self):
        v_small = hover_induced_velocity(9.0, 4, 0.20)
        v_large = hover_induced_velocity(9.0, 4, 0.50)
        assert v_large < v_small

    def test_heavier_drone_gives_higher_induced_velocity(self):
        v_light = hover_induced_velocity(3.0, 6, 0.38)
        v_heavy = hover_induced_velocity(15.0, 6, 0.38)
        assert v_heavy > v_light


class TestParasiteDragArea:
    def test_formula(self):
        F_drag = 9.0 * GRAVITY * math.tan(math.radians(30.0))
        expected = F_drag / (0.5 * AIR_DENSITY_SL * 20.0 ** 2)
        assert parasite_drag_area(9.0, 30.0, 20.0) == pytest.approx(expected, rel=1e-9)

    def test_larger_tilt_gives_larger_area(self):
        assert parasite_drag_area(9.0, 40.0, 20.0) > parasite_drag_area(9.0, 20.0, 20.0)

    def test_higher_max_speed_gives_smaller_area(self):
        # Same drag force, but drag is normalised by higher dynamic pressure
        assert parasite_drag_area(9.0, 30.0, 30.0) < parasite_drag_area(9.0, 30.0, 20.0)


class TestCruisePower:
    def setup_method(self):
        self.P_hover = 1500.0
        self.v_i0 = hover_induced_velocity(9.0, 6, 0.38)
        self.Cd_A = parasite_drag_area(9.0, 30.0, 20.0)

    def test_at_zero_speed_equals_hover_power(self):
        # P_induced + P_profile = FM*P + (1-FM)*P = P_hover
        P = cruise_power_w(self.P_hover, self.v_i0, self.Cd_A, 0.0)
        assert P == pytest.approx(self.P_hover, rel=1e-9)

    def test_power_minimum_is_below_hover(self):
        # Induced power falls faster than parasite rises at moderate speed
        powers = [cruise_power_w(self.P_hover, self.v_i0, self.Cd_A, v) for v in range(1, 15)]
        assert min(powers) < self.P_hover

    def test_high_speed_power_exceeds_hover(self):
        # At max speed parasite drag dominates
        P_fast = cruise_power_w(self.P_hover, self.v_i0, self.Cd_A, 20.0)
        assert P_fast > self.P_hover

    def test_power_increases_at_high_speed(self):
        # Must see P increasing in the high-speed regime
        P_15 = cruise_power_w(self.P_hover, self.v_i0, self.Cd_A, 15.0)
        P_20 = cruise_power_w(self.P_hover, self.v_i0, self.Cd_A, 20.0)
        assert P_20 > P_15

    def test_wind_increases_power(self):
        # v_eff = sqrt(v^2 + wind^2) > v → higher power
        v = 12.0
        P_no_wind = cruise_power_w(self.P_hover, self.v_i0, self.Cd_A,
                                   math.sqrt(v ** 2 + 0.0 ** 2))
        P_wind = cruise_power_w(self.P_hover, self.v_i0, self.Cd_A,
                                math.sqrt(v ** 2 + 5.0 ** 2))
        assert P_wind > P_no_wind


class TestResolveHoverPower:
    """hover_power_w is optional — the model derives it when absent."""

    def test_explicit_value_returned_directly(self):
        drone = _make_drone(hover_power_w=2000.0)
        assert resolve_hover_power(drone) == pytest.approx(2000.0)

    def test_flight_time_path(self):
        # P = battery_energy / (flight_time_min / 60)
        drone = _make_drone(hover_power_w=0.0, battery_energy_wh=300.0, flight_time_min=60.0)
        assert resolve_hover_power(drone) == pytest.approx(300.0 / 1.0)

    def test_flight_time_path_thirty_minutes(self):
        drone = _make_drone(hover_power_w=0.0, battery_energy_wh=150.0, flight_time_min=30.0)
        # 150 Wh / 0.5 h = 300 W
        assert resolve_hover_power(drone) == pytest.approx(300.0)

    def test_explicit_beats_flight_time(self):
        # Explicit value takes priority even when flight_time_min is also set
        drone = _make_drone(hover_power_w=1234.0, battery_energy_wh=300.0, flight_time_min=60.0)
        assert resolve_hover_power(drone) == pytest.approx(1234.0)

    def test_momentum_theory_fallback_formula(self):
        # When neither hover_power_w nor flight_time_min is provided,
        # momentum theory is used.
        drone = _make_drone(hover_power_w=0.0, flight_time_min=0.0,
                            mtow_kg=9.0, num_rotors=6, propeller_diameter_m=0.38)
        p_resolved = resolve_hover_power(drone)
        p_expected = hover_power_from_momentum_theory(9.0, 6, 0.38)
        assert p_resolved == pytest.approx(p_expected)

    def test_momentum_theory_physically_reasonable(self):
        # A 9 kg hexacopter should hover somewhere in the 500-4000 W range
        drone = _make_drone(hover_power_w=0.0, flight_time_min=0.0,
                            mtow_kg=9.0, num_rotors=6, propeller_diameter_m=0.38)
        p = resolve_hover_power(drone)
        assert 500 < p < 4000

    def test_momentum_theory_scales_with_weight(self):
        light = _make_drone(hover_power_w=0.0, flight_time_min=0.0, mtow_kg=3.0)
        heavy = _make_drone(hover_power_w=0.0, flight_time_min=0.0, mtow_kg=15.0)
        assert resolve_hover_power(heavy) > resolve_hover_power(light)

    def test_momentum_theory_scales_with_prop_area(self):
        # Larger props → more disk area → lower induced velocity → less power
        small_prop = _make_drone(hover_power_w=0.0, flight_time_min=0.0,
                                 propeller_diameter_m=0.20)
        large_prop = _make_drone(hover_power_w=0.0, flight_time_min=0.0,
                                 propeller_diameter_m=0.50)
        assert resolve_hover_power(large_prop) < resolve_hover_power(small_prop)

    def test_hover_power_from_endurance_formula(self):
        assert hover_power_from_endurance(300.0, 60.0) == pytest.approx(300.0)
        assert hover_power_from_endurance(150.0, 30.0) == pytest.approx(300.0)

    def test_hover_power_from_momentum_theory_formula(self):
        mtow, N, D = 9.0, 6, 0.38
        r = D / 2
        A = N * math.pi * r * r
        T = mtow * GRAVITY
        P_ideal = T * math.sqrt(T / (2 * AIR_DENSITY_SL * A))
        expected = P_ideal / MOMENTUM_THEORY_EFFICIENCY
        assert hover_power_from_momentum_theory(mtow, N, D) == pytest.approx(expected, rel=1e-9)

    def test_estimate_flight_uses_derived_hover_power(self):
        # estimate_flight with hover_power_w=0 and flight_time_min=60 should give
        # the same result as setting hover_power_w explicitly to the derived value
        bat = 300.0
        ft = 60.0
        derived_p = hover_power_from_endurance(bat, ft)

        drone_implicit = _make_drone(hover_power_w=0.0, flight_time_min=ft,
                                     battery_energy_wh=bat,
                                     takeoff_height_m=0.0, landing_height_m=0.0)
        drone_explicit = _make_drone(hover_power_w=derived_p, battery_energy_wh=bat,
                                     takeoff_height_m=0.0, landing_height_m=0.0)

        route = _make_route(wind=0.0, segment_length_m=5000.0)
        est_implicit = estimate_flight(drone_implicit, route)
        est_explicit = estimate_flight(drone_explicit, route)

        assert est_implicit.energy_consumed_wh == pytest.approx(est_explicit.energy_consumed_wh)
        assert est_implicit.SoC_remaining == pytest.approx(est_explicit.SoC_remaining)


class TestEstimeFlight:
    def test_zero_wind_cruise_time(self):
        # 5000 m at 15 m/s (75% of 20 m/s) → 333.3 s
        drone = _make_drone(max_speed_ms=20.0, cruise_speed_ms=0.0,
                            takeoff_height_m=0.0, landing_height_m=0.0)
        route = _make_route(wind=0.0, segment_length_m=5000.0)
        est = estimate_flight(drone, route)
        expected_t = 5000.0 / (20.0 * 0.75)
        assert est.cruise_time_s == pytest.approx(expected_t, rel=1e-6)

    def test_explicit_cruise_speed_used(self):
        drone = _make_drone(cruise_speed_ms=10.0,
                            takeoff_height_m=0.0, landing_height_m=0.0)
        route = _make_route(wind=0.0, segment_length_m=1000.0)
        est = estimate_flight(drone, route)
        assert est.cruise_time_s == pytest.approx(100.0, rel=1e-6)

    def test_total_time_includes_vertical_phases(self):
        drone = _make_drone(cruise_speed_ms=15.0,
                            max_ascent_speed_ms=5.0, takeoff_height_m=50.0,
                            max_descent_speed_ms=3.0, landing_height_m=30.0)
        route = _make_route(wind=0.0, segment_length_m=0.0)
        route.segments = []
        route.waypoints = []
        est = estimate_flight(drone, route)
        t_up = 50.0 / 5.0    # 10 s
        t_down = 30.0 / 3.0  # 10 s
        assert est.total_time_s == pytest.approx(t_up + t_down, rel=1e-6)
        assert est.cruise_time_s == pytest.approx(0.0)

    def test_takeoff_energy_uses_climb_factor(self):
        drone = _make_drone(hover_power_w=1000.0,
                            max_ascent_speed_ms=5.0, takeoff_height_m=50.0,
                            max_descent_speed_ms=5.0, landing_height_m=0.0,
                            cruise_speed_ms=15.0)
        route = _make_route(wind=0.0, segment_length_m=0.0)
        route.segments = []
        route.waypoints = []
        est = estimate_flight(drone, route)
        t_up = 50.0 / 5.0
        expected_e = CLIMB_POWER_FACTOR * 1000.0 * t_up / 3600.0
        assert est.energy_consumed_wh == pytest.approx(expected_e, rel=1e-6)

    def test_landing_energy_uses_descent_factor(self):
        drone = _make_drone(hover_power_w=1000.0,
                            max_ascent_speed_ms=5.0, takeoff_height_m=0.0,
                            max_descent_speed_ms=4.0, landing_height_m=40.0,
                            cruise_speed_ms=15.0)
        route = _make_route(wind=0.0, segment_length_m=0.0)
        route.segments = []
        route.waypoints = []
        est = estimate_flight(drone, route)
        t_down = 40.0 / 4.0
        expected_e = DESCENT_POWER_FACTOR * 1000.0 * t_down / 3600.0
        assert est.energy_consumed_wh == pytest.approx(expected_e, rel=1e-6)

    def test_wind_increases_energy_not_time(self):
        drone = _make_drone(takeoff_height_m=0.0, landing_height_m=0.0)
        route_calm = _make_route(wind=0.0, segment_length_m=5000.0)
        route_windy = _make_route(wind=8.0, segment_length_m=5000.0)
        est_calm = estimate_flight(drone, route_calm)
        est_windy = estimate_flight(drone, route_windy)
        assert est_windy.energy_consumed_wh > est_calm.energy_consumed_wh
        assert est_windy.cruise_time_s == pytest.approx(est_calm.cruise_time_s)

    def test_soc_remaining_decreases_with_distance(self):
        drone = _make_drone(takeoff_height_m=0.0, landing_height_m=0.0)
        est_short = estimate_flight(drone, _make_route(segment_length_m=1000.0))
        est_long = estimate_flight(drone, _make_route(segment_length_m=20_000.0))
        assert est_long.SoC_remaining < est_short.SoC_remaining

    def test_speed_clamped_to_segment_v_max(self):
        drone = _make_drone(cruise_speed_ms=25.0,  # above segment v_max=20
                            takeoff_height_m=0.0, landing_height_m=0.0)
        route = _make_route(wind=0.0, segment_length_m=2000.0)
        route.segments[0].v_max = 10.0
        est = estimate_flight(drone, route)
        assert est.cruise_time_s == pytest.approx(2000.0 / 10.0, rel=1e-6)

    def test_speed_clamped_to_segment_v_min(self):
        drone = _make_drone(cruise_speed_ms=2.0,  # below segment v_min=5
                            takeoff_height_m=0.0, landing_height_m=0.0)
        route = _make_route(wind=0.0, segment_length_m=1000.0)
        route.segments[0].v_min = 8.0
        est = estimate_flight(drone, route)
        assert est.cruise_time_s == pytest.approx(1000.0 / 8.0, rel=1e-6)


# ===========================================================================
# route_suggestion — internal safety score threshold
# ===========================================================================

class TestMinRequiredSafetyScore:
    def test_no_wind_returns_floor(self):
        drone = _make_drone(max_wind_resistance_ms=12.0)
        route = _make_route(wind=0.0)
        score = _min_required_safety_score(route, drone)
        assert score == pytest.approx(0.50)

    def test_full_wind_returns_ceiling(self):
        drone = _make_drone(max_wind_resistance_ms=12.0)
        route = _make_route(wind=12.0)
        score = _min_required_safety_score(route, drone)
        assert score == pytest.approx(0.90)

    def test_half_wind_is_midpoint(self):
        drone = _make_drone(max_wind_resistance_ms=12.0)
        route = _make_route(wind=6.0)
        score = _min_required_safety_score(route, drone)
        assert score == pytest.approx(0.70)

    def test_zero_wind_resistance_returns_ceiling(self):
        drone = _make_drone(max_wind_resistance_ms=0.0)
        route = _make_route(wind=0.0)
        assert _min_required_safety_score(route, drone) == pytest.approx(0.90)


# ===========================================================================
# suggest_routes — end-to-end filtering
# ===========================================================================

class TestSuggestRoutesEmpty:
    def test_empty_route_list(self):
        result = suggest_routes(_make_request(), [])
        assert result.suggested_routes == []
        assert result.rejected_routes == []


class TestSuggestRoutesHappyPath:
    def test_compatible_route_returns_suggested_route(self):
        route = _make_route()
        result = suggest_routes(_make_request(), [route])
        assert len(result.suggested_routes) == 1
        assert isinstance(result.suggested_routes[0], SuggestedRoute)
        assert result.suggested_routes[0].route.route_id == "r1"
        assert result.rejected_routes == []

    def test_estimate_is_attached_and_populated(self):
        route = _make_route()
        result = suggest_routes(_make_request(), [route])
        est = result.suggested_routes[0].estimate
        assert est.cruise_time_s > 0
        assert est.total_time_s >= est.cruise_time_s
        assert est.energy_consumed_wh > 0
        assert 0.0 < est.SoC_remaining < 1.0

    def test_multiple_compatible_routes(self):
        routes = [_make_route(route_id=f"r{i}") for i in range(4)]
        result = suggest_routes(_make_request(), routes)
        assert len(result.suggested_routes) == 4
        assert result.rejected_routes == []


class TestVertiportFilter:
    def test_wrong_takeoff_vertiport(self):
        route = _make_route(take_off="vp_x")
        result = suggest_routes(_make_request(take_off="vp_a"), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH

    def test_wrong_landing_vertiport(self):
        route = _make_route(landing="vp_z")
        result = suggest_routes(_make_request(landing="vp_b"), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH


class TestWindFilter:
    def test_route_wind_exceeds_max_wind_resistance(self):
        drone = _make_drone(max_wind_resistance_ms=10.0)
        route = _make_route(wind=11.0)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == WIND_TOO_HIGH

    def test_route_wind_equals_limit_is_accepted(self):
        drone = _make_drone(max_wind_resistance_ms=10.0)
        route = _make_route(wind=10.0, safety=0.92)  # wind=10/max=10 → min_safety=0.90
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1


class TestWeightFilter:
    def test_mtow_exceeds_route_limit(self):
        drone = _make_drone(mtow_kg=20.0)
        route = _make_route(max_weight=15.0)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == WEIGHT_EXCEEDED

    def test_mtow_at_limit_is_accepted(self):
        drone = _make_drone(mtow_kg=15.0)
        route = _make_route(max_weight=15.0)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1


class TestSizeFilter:
    def test_incompatible_size_rejected(self):
        drone = _make_drone(drone_size="C")
        route = _make_route(sizes=["A", "B"])
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == SIZE_INCOMPATIBLE

    def test_compatible_size_accepted(self):
        drone = _make_drone(drone_size="B")
        route = _make_route(sizes=["A", "B", "C"])
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1


class TestAltitudeFilter:
    def test_route_above_service_ceiling_rejected(self):
        drone = _make_drone(service_ceiling_m=100.0)
        route = _make_route(max_altitude_m=150.0)  # waypoint z = 150 m
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == ALTITUDE_EXCEEDED

    def test_route_at_service_ceiling_accepted(self):
        drone = _make_drone(service_ceiling_m=100.0)
        route = _make_route(max_altitude_m=100.0)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1

    def test_zero_service_ceiling_skips_altitude_check(self):
        # service_ceiling_m=0 means "not specified"; altitude check skipped
        drone = _make_drone(service_ceiling_m=0.0)
        route = _make_route(max_altitude_m=99999.0)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1


class TestSafetyFilter:
    def test_low_safety_score_rejected_at_high_wind(self):
        # wind=6, max_wind=12 → wind_fraction=0.5 → min_safety=0.70
        drone = _make_drone(max_wind_resistance_ms=12.0)
        route = _make_route(wind=6.0, safety=0.60)  # 0.60 < 0.70
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == SAFETY_SCORE_INSUFFICIENT

    def test_sufficient_safety_score_accepted(self):
        drone = _make_drone(max_wind_resistance_ms=12.0)
        route = _make_route(wind=6.0, safety=0.75)  # 0.75 >= 0.70
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1

    def test_calm_wind_requires_only_floor_safety(self):
        drone = _make_drone(max_wind_resistance_ms=12.0)
        route = _make_route(wind=0.0, safety=0.55)  # min_safety = 0.50
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1

    def test_below_floor_always_rejected(self):
        drone = _make_drone(max_wind_resistance_ms=12.0)
        route = _make_route(wind=0.0, safety=0.40)  # below 0.50 floor
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.rejected_routes[0].reason == SAFETY_SCORE_INSUFFICIENT


class TestBatteryFilter:
    def test_very_long_route_drained_battery(self):
        drone = _make_drone(battery_energy_wh=50.0, soc_min=0.20)
        route = _make_route(segment_length_m=500_000.0)   # 500 km
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.suggested_routes == []
        assert result.rejected_routes[0].reason == INSUFFICIENT_BATTERY

    def test_short_route_sufficient_battery(self):
        drone = _make_drone(battery_energy_wh=300.0)
        route = _make_route(segment_length_m=2000.0)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert len(result.suggested_routes) == 1


class TestRejectionPriority:
    def test_vertiport_mismatch_beats_battery(self):
        drone = _make_drone(battery_energy_wh=0.001)
        route = _make_route(take_off="wrong_vp", segment_length_m=999_999.0)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.rejected_routes[0].reason == VERTIPORT_MISMATCH

    def test_wind_beats_safety(self):
        # Route wind exceeds drone limit AND has low safety — wind is checked first
        drone = _make_drone(max_wind_resistance_ms=5.0)
        route = _make_route(wind=10.0, safety=0.10)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.rejected_routes[0].reason == WIND_TOO_HIGH

    def test_eligibility_beats_safety(self):
        # Altitude exceeded + low safety → altitude is the hard-eligibility reason
        drone = _make_drone(service_ceiling_m=50.0)
        route = _make_route(max_altitude_m=100.0, safety=0.10)
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.rejected_routes[0].reason == ALTITUDE_EXCEEDED

    def test_safety_beats_battery(self):
        # Safety fails AND battery would fail — safety is checked first
        drone = _make_drone(max_wind_resistance_ms=12.0,
                            battery_energy_wh=0.001, soc_min=0.20)
        route = _make_route(wind=6.0, safety=0.50, segment_length_m=999_999.0)
        # wind=6, max=12 → min_safety=0.70; route safety=0.50 < 0.70
        result = suggest_routes(_make_request(drone=drone), [route])
        assert result.rejected_routes[0].reason == SAFETY_SCORE_INSUFFICIENT


class TestMixedRoutes:
    def test_split_across_all_rejection_reasons(self):
        drone = _make_drone(
            mtow_kg=10.0,
            max_wind_resistance_ms=12.0,
            service_ceiling_m=200.0,
            battery_energy_wh=300.0,
        )
        routes = [
            _make_route(route_id="ok"),
            _make_route(route_id="wrong_vp", take_off="vp_z"),
            _make_route(route_id="wind_fail", wind=15.0),
            _make_route(route_id="weight_fail", max_weight=5.0),
            _make_route(route_id="size_fail", sizes=["A"]),
            _make_route(route_id="alt_fail", max_altitude_m=250.0),
            _make_route(route_id="safety_fail", wind=6.0, safety=0.55),
        ]
        result = suggest_routes(_make_request(drone=drone), routes)
        assert len(result.suggested_routes) == 1
        assert result.suggested_routes[0].route.route_id == "ok"

        reasons = {rj.route.route_id: rj.reason for rj in result.rejected_routes}
        assert reasons["wrong_vp"] == VERTIPORT_MISMATCH
        assert reasons["wind_fail"] == WIND_TOO_HIGH
        assert reasons["weight_fail"] == WEIGHT_EXCEEDED
        assert reasons["size_fail"] == SIZE_INCOMPATIBLE
        assert reasons["alt_fail"] == ALTITUDE_EXCEEDED
        assert reasons["safety_fail"] == SAFETY_SCORE_INSUFFICIENT


class TestMultiSegmentRoute:
    def test_energy_sums_over_all_segments(self):
        drone = _make_drone(takeoff_height_m=0.0, landing_height_m=0.0)
        # Build a 3-waypoint route manually
        wp1 = Waypoint(id="w1", position=(0.0, 0.0, 50.0))
        wp2 = Waypoint(id="w2", position=(3000.0, 0.0, 50.0))
        wp3 = Waypoint(id="w3", position=(6000.0, 0.0, 50.0))
        route = FlightRoute(
            route_id="multi",
            waypoints=[wp1, wp2, wp3],
            take_off_vertiport_id="vp_a",
            landing_vertiport_id="vp_b",
            safety_score=0.85,
            average_wind_speed=0.0,
            max_drone_weight_kg=20.0,
            compatible_drone_sizes=["B"],
        )
        est = estimate_flight(drone, route)
        # Each 3000 m segment at 15 m/s → 200 s each; total cruise = 400 s
        assert est.cruise_time_s == pytest.approx(400.0, rel=1e-3)

    def test_flight_route_auto_builds_segments(self):
        wp1 = Waypoint(id="w1", position=(0.0, 0.0, 50.0))
        wp2 = Waypoint(id="w2", position=(300.0, 400.0, 50.0))  # 500 m (3-4-5 triangle)
        route = FlightRoute(
            route_id="seg_test",
            waypoints=[wp1, wp2],
            take_off_vertiport_id="vp_a",
            landing_vertiport_id="vp_b",
            safety_score=1.0,
            average_wind_speed=0.0,
            max_drone_weight_kg=50.0,
            compatible_drone_sizes=["B"],
        )
        assert len(route.segments) == 1
        assert route.segments[0].length == pytest.approx(500.0, rel=1e-6)
