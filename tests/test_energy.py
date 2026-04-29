"""Tests for energy and SoC computation."""
from __future__ import annotations

import pytest

from scrp.energy import compute_E_cruise, compute_SoC_remaining
from scrp.models import SCRPConfig
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
