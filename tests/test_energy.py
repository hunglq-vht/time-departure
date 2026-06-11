"""Tests for energy and SoC computation."""
from __future__ import annotations

import math

import pytest

from scrp.energy import _resolve_fi_hover_power, compute_E_cruise, compute_SoC_remaining
from scrp.models import SCRPConfig
from scrp.power_model import (
    cruise_power_w,
    hover_induced_velocity,
    hover_power_from_momentum_theory,
    parasite_drag_area,
)
from tests.conftest import make_fi, make_lane, make_waypoint


# ---------------------------------------------------------------------------
# Shared helper: fi with full propulsion geometry
# ---------------------------------------------------------------------------

def _make_physics_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0):
    fi = make_fi(lane, v=v, SoC_0=SoC_0, C_bat=C_bat, P_hover=0.0)
    fi.mtow_kg = 5.0
    fi.num_rotors = 4
    fi.propeller_diameter_m = 0.3
    fi.max_tilt_angle_deg = 30.0
    fi.max_speed_ms = 20.0
    return fi


# ---------------------------------------------------------------------------
# _resolve_fi_hover_power
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# compute_E_cruise
# ---------------------------------------------------------------------------

class TestComputeECruise:
    def test_returns_none_without_geometry(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        fi = make_fi(make_lane("l", wps), P_hover=500.0)  # only P_hover, no geometry
        assert compute_E_cruise(fi) is None

    def test_single_segment_matches_manual(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 1000, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        fi = _make_physics_fi(lane, v=10.0)

        v_i0 = hover_induced_velocity(5.0, 4, 0.3)
        Cd_A = parasite_drag_area(5.0, 30.0, 20.0)
        P_hover = hover_power_from_momentum_theory(5.0, 4, 0.3)
        t_seg = 1000.0 / 10.0
        expected = cruise_power_w(P_hover, v_i0, Cd_A, 10.0) * t_seg / 3600.0

        assert compute_E_cruise(fi) == pytest.approx(expected, rel=1e-6)

    def test_multi_segment_sums_correctly(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 1000, 0), make_waypoint("C", 3000, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        fi = _make_physics_fi(lane, v=10.0)

        v_i0 = hover_induced_velocity(5.0, 4, 0.3)
        Cd_A = parasite_drag_area(5.0, 30.0, 20.0)
        P_hover = hover_power_from_momentum_theory(5.0, 4, 0.3)
        P_seg = cruise_power_w(P_hover, v_i0, Cd_A, 10.0)
        expected = P_seg * (1000.0 / 10.0) / 3600.0 + P_seg * (2000.0 / 10.0) / 3600.0

        assert compute_E_cruise(fi) == pytest.approx(expected, rel=1e-6)

    def test_result_is_finite_and_positive(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 5000, 0)]
        fi = _make_physics_fi(make_lane("l", wps, v_min=5.0, v_max=20.0))
        E = compute_E_cruise(fi)
        assert E is not None
        assert E > 0.0
        assert math.isfinite(E)

    def test_no_parasite_drag_when_tilt_absent(self):
        """Without max_tilt_angle_deg, Cd_A = 0; result must still be positive."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 1000, 0)]
        fi = _make_physics_fi(make_lane("l", wps, v_min=5.0, v_max=20.0))
        fi.max_tilt_angle_deg = 0.0  # disable parasite drag
        E = compute_E_cruise(fi)
        assert E is not None and E > 0.0


# ---------------------------------------------------------------------------
# compute_SoC_remaining
# ---------------------------------------------------------------------------

class TestComputeSoCRemaining:
    def test_returns_none_when_e_cruise_is_none(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        fi = _make_physics_fi(make_lane("l", wps))
        soc = compute_SoC_remaining(fi, t_dep_star=0.0, t_slot_start=10.0, E_cruise=None)
        assert soc is None

    def test_no_hover_wait(self):
        """Drone lands exactly when slot opens → no hover energy."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = _make_physics_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0)
        E_cruise = compute_E_cruise(fi)
        assert E_cruise is not None

        # t_dep=0, cruise=10s → t_land=10s; slot_start=10s → no wait
        soc = compute_SoC_remaining(fi, t_dep_star=0.0, t_slot_start=10.0, E_cruise=E_cruise)
        expected = 1.0 - E_cruise / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)

    def test_hover_wait_reduces_soc(self):
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = _make_physics_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0)
        E_cruise = compute_E_cruise(fi)
        assert E_cruise is not None

        P_hover = _resolve_fi_hover_power(fi)
        # cruise=10s; slot_start=40s → 30s hover wait
        soc = compute_SoC_remaining(fi, t_dep_star=0.0, t_slot_start=40.0, E_cruise=E_cruise)
        E_hover = P_hover * (30.0 / 3600.0)
        expected = 1.0 - (E_cruise + E_hover) / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)

    def test_hover_wait_excludes_landing_duration(self):
        """Landing phase duration is excluded from hover wait calculation."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = _make_physics_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0)
        fi.t_land_estimated = 5.0

        E_cruise = compute_E_cruise(fi)
        assert E_cruise is not None
        P_hover = _resolve_fi_hover_power(fi)

        # cruise ends at 10s; slot_start=40s → hover wait = 40 - 10 = 30s
        soc = compute_SoC_remaining(fi, t_dep_star=0.0, t_slot_start=40.0, E_cruise=E_cruise)
        E_hover = P_hover * (30.0 / 3600.0)
        expected = 1.0 - (E_cruise + E_hover) / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)

    def test_takeoff_duration_shifts_dest_arrival(self):
        """t_takeoff delays arrival at destination, reducing hover wait."""
        wps = [make_waypoint("A", 0, 0), make_waypoint("B", 100, 0)]
        lane = make_lane("l", wps)
        fi = _make_physics_fi(lane, v=10.0, SoC_0=1.0, C_bat=1000.0)
        fi.t_takeoff = 20.0  # arrival = 0 + 20 + 10 = 30s

        E_cruise = compute_E_cruise(fi)
        assert E_cruise is not None
        P_hover = _resolve_fi_hover_power(fi)

        # slot_start=40s → 10s hover wait
        soc = compute_SoC_remaining(fi, t_dep_star=0.0, t_slot_start=40.0, E_cruise=E_cruise)
        E_hover = P_hover * (10.0 / 3600.0)
        expected = 1.0 - (E_cruise + E_hover) / 1000.0
        assert soc == pytest.approx(expected, rel=1e-6)
