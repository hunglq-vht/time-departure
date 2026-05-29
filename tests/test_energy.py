"""Tests for energy and SoC computation."""
from __future__ import annotations

import math

import pytest

from scrp.energy import _resolve_fi_hover_power, compute_E_cruise, compute_SoC_remaining
from scrp.models import FlightIntention, SCRPConfig
from scrp.power_model import (
    cruise_power_w,
    hover_induced_velocity,
    hover_power_from_momentum_theory,
    parasite_drag_area,
)
from tests.conftest import make_fi, make_lane, make_waypoint


class TestComputeECruise:
    def test_single_segment(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, P_hover=500.0)
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)
        E = compute_E_cruise(fi, config)
        # L=100m, v=10m/s → travel_time=10s → travel_time_h=10/3600
        # P_cruise = 500 * 1.2 = 600W
        # E = 600 * (10/3600) = 1.6667 Wh
        expected = 600.0 * (10.0 / 3600.0)
        assert E == pytest.approx(expected, rel=1e-6)

    def test_multi_segment(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0), make_waypoint("C", 300, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, P_hover=500.0)
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)
        E = compute_E_cruise(fi, config)
        P_cruise = 600.0
        E_expected = P_cruise * (10.0 / 3600.0) + P_cruise * (20.0 / 3600.0)
        assert E == pytest.approx(E_expected, rel=1e-6)


class TestComputeSoCRemaining:
    def test_no_hover_wait(self):
        """Drone lands exactly at slot start → no hover wait."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0, P_hover=500.0)
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)
        E_cruise = compute_E_cruise(fi, config)

        t_dep_star = 0.0
        # t_land = 10s, slot_start = 10s → no wait
        soc = compute_SoC_remaining(fi, t_dep_star, t_slot_start=10.0, E_cruise=E_cruise)
        expected = 1.0 - E_cruise / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)

    def test_hover_wait_reduces_soc(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0, P_hover=500.0)
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)
        E_cruise = compute_E_cruise(fi, config)

        t_dep_star = 0.0
        # t_land = 10s, slot_start = 40s → 30s hover wait
        soc = compute_SoC_remaining(fi, t_dep_star, t_slot_start=40.0, E_cruise=E_cruise)
        E_hover = 500.0 * (30.0 / 3600.0)
        expected = 1.0 - (E_cruise + E_hover) / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)

    def test_hover_wait_excludes_landing_duration(self):
        """Hover wait must be measured before the landing phase, not after it."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0, P_hover=500.0)
        fi.t_land_estimated = 5.0  # 5 s landing phase
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)
        E_cruise = compute_E_cruise(fi, config)

        t_dep_star = 0.0
        # cruise ends at 10 s; slot_start = 40 s → hover wait = 40 - 10 = 30 s
        # (the 5 s landing phase happens after the slot opens, not during hover)
        soc = compute_SoC_remaining(fi, t_dep_star, t_slot_start=40.0, E_cruise=E_cruise)
        E_hover = 500.0 * (30.0 / 3600.0)
        expected = 1.0 - (E_cruise + E_hover) / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)

    def test_takeoff_duration_shifts_dest_arrival(self):
        """t_takeoff delays when the drone arrives at the destination, reducing hover wait."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0, P_hover=500.0)
        fi.t_takeoff = 20.0  # 20 s takeoff → dest arrival at 30 s
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)
        E_cruise = compute_E_cruise(fi, config)

        t_dep_star = 0.0
        # dest arrival = 0 + 20 (takeoff) + 10 (cruise) = 30 s; slot_start = 40 s → 10 s hover
        soc = compute_SoC_remaining(fi, t_dep_star, t_slot_start=40.0, E_cruise=E_cruise)
        E_hover = 500.0 * (10.0 / 3600.0)
        expected = 1.0 - (E_cruise + E_hover) / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)


class TestResolveHoverPower:
    def test_explicit_p_hover_has_priority(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        fi = make_fi(make_lane("l", wps), P_hover=400.0)
        fi.flight_time_min = 30.0  # should be ignored
        assert _resolve_fi_hover_power(fi) == pytest.approx(400.0)

    def test_endurance_path(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        fi = make_fi(make_lane("l", wps), P_hover=0.0, C_bat=200.0)
        fi.flight_time_min = 30.0  # 200 Wh / 0.5 h = 400 W
        assert _resolve_fi_hover_power(fi) == pytest.approx(400.0)

    def test_momentum_theory_path(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        fi = make_fi(make_lane("l", wps), P_hover=0.0)
        fi.mtow_kg = 5.0
        fi.num_rotors = 4
        fi.propeller_diameter_m = 0.3
        expected = hover_power_from_momentum_theory(5.0, 4, 0.3)
        assert _resolve_fi_hover_power(fi) == pytest.approx(expected)

    def test_no_source_raises(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        fi = make_fi(make_lane("l", wps), P_hover=0.0)
        with pytest.raises(ValueError, match="hover power cannot be determined"):
            _resolve_fi_hover_power(fi)


class TestPhysicsBasedCruiseEnergy:
    """compute_E_cruise uses the physics model when propulsion geometry is provided."""

    def _make_fi_physics(self, lane, v=10.0):
        fi = make_fi(lane, v=v, P_hover=0.0, C_bat=500.0)
        fi.mtow_kg = 5.0
        fi.num_rotors = 4
        fi.propeller_diameter_m = 0.3
        fi.max_tilt_angle_deg = 30.0
        fi.max_speed_ms = 20.0
        return fi

    def test_physics_path_produces_finite_positive_energy(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 1000, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        fi = self._make_fi_physics(lane)
        config = SCRPConfig()
        E = compute_E_cruise(fi, config)
        assert E > 0.0
        assert math.isfinite(E)

    def test_physics_path_matches_manual_calculation(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 1000, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        fi = self._make_fi_physics(lane, v=10.0)
        config = SCRPConfig()

        v_i0 = hover_induced_velocity(5.0, 4, 0.3)
        Cd_A = parasite_drag_area(5.0, 30.0, 20.0)
        P_hover = hover_power_from_momentum_theory(5.0, 4, 0.3)
        t_seg = 1000.0 / 10.0
        P_seg = cruise_power_w(P_hover, v_i0, Cd_A, 10.0)
        expected = P_seg * t_seg / 3600.0

        E = compute_E_cruise(fi, config)
        assert E == pytest.approx(expected, rel=1e-6)

    def test_physics_path_differs_from_legacy_path(self):
        """Physics model and legacy fixed-factor model should give different results."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 1000, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)

        fi_physics = self._make_fi_physics(lane)
        fi_legacy = make_fi(lane, P_hover=hover_power_from_momentum_theory(5.0, 4, 0.3))

        E_physics = compute_E_cruise(fi_physics, config)
        E_legacy = compute_E_cruise(fi_legacy, config)
        # They use the same P_hover but different cruise power models — results differ
        assert E_physics != pytest.approx(E_legacy, rel=1e-3)

    def test_no_geometry_falls_back_to_legacy(self):
        """Without propulsion geometry, legacy P_hover * CRUISE_POWER_FACTOR is used."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, P_hover=500.0)
        config = SCRPConfig(CRUISE_POWER_FACTOR=1.2)

        E = compute_E_cruise(fi, config)
        expected = 600.0 * (10.0 / 3600.0)
        assert E == pytest.approx(expected, rel=1e-6)
