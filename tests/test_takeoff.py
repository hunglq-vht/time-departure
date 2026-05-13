"""Unit tests for the C0 (takeoff separation) constraint."""
from __future__ import annotations

import pytest

from scrp.models import ApprovedPlan, FlightIntention, SCRPConfig, SystemState
from scrp.takeoff import compute_delta_C0

from tests.conftest import (
    make_fi,
    make_lane,
    make_system_state,
    make_takeoff_pad,
    make_vertiport,
    make_waypoint,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_plan_with_takeoff(
    drone_id: str,
    t_dep: float,
    t_takeoff: float,
    origin_vertiport_id: str,
    takeoff_pad_id: str,
    lane,
    uav_type: str = "A",
) -> ApprovedPlan:
    return ApprovedPlan(
        drone_id=drone_id,
        uav_type=uav_type,
        lane=lane,
        v_waypoints=[10.0] * len(lane.waypoints),
        t_land=t_dep + 100.0,
        pad_id="lpad1",
        slot_index=0,
        status="SOFT_RESERVED",
        t_dep=t_dep,
        waypoint_times=[],
        origin_vertiport_id=origin_vertiport_id,
        takeoff_pad_id=takeoff_pad_id,
        t_takeoff=t_takeoff,
    )


# ---------------------------------------------------------------------------
# Tests — no constraint when takeoff context absent
# ---------------------------------------------------------------------------

class TestC0NullCases:
    def test_no_origin_vertiport_returns_zero(self):
        wps = [make_waypoint("A"), make_waypoint("B", 100)]
        lane = make_lane("L1", wps, destination_vertiport_id="vp_dest")
        fi = make_fi(lane, t_des=1000.0)  # no origin_vertiport_id
        state = make_system_state()
        assert compute_delta_C0(fi, [], 1000.0, 1000.0, state) == 0.0

    def test_no_approved_plans_returns_zero(self):
        wps = [make_waypoint("A"), make_waypoint("B", 100)]
        lane = make_lane("L1", wps, destination_vertiport_id="vp_dest")
        tp = make_takeoff_pad("tp1", x=0, y=0)
        vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=[tp])
        vp_dest = make_vertiport(vertiport_id="vp_dest")
        state = make_system_state(vertiports={"vp_origin": vp_origin, "vp_dest": vp_dest})
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        assert compute_delta_C0(fi, [], 1000.0, 1000.0, state) == 0.0

    def test_plan_from_different_vertiport_ignored(self):
        wps = [make_waypoint("A"), make_waypoint("B", 100)]
        lane = make_lane("L1", wps, destination_vertiport_id="vp_dest")
        tp1 = make_takeoff_pad("tp1", x=0, y=0)
        tp2 = make_takeoff_pad("tp2", x=5, y=0)  # within MSD=50 m
        vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=[tp1, tp2])
        vp_dest = make_vertiport(vertiport_id="vp_dest")
        state = make_system_state(vertiports={"vp_origin": vp_origin, "vp_dest": vp_dest})
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")

        # Plan departs from a DIFFERENT origin vertiport — must be ignored
        plan = _make_plan_with_takeoff(
            "drone_other", t_dep=990.0, t_takeoff=30.0,
            origin_vertiport_id="vp_other", takeoff_pad_id="tp2", lane=lane,
        )
        assert compute_delta_C0(fi, [plan], 1000.0, 1000.0, state) == 0.0


# ---------------------------------------------------------------------------
# Tests — same pad (exclusive use)
# ---------------------------------------------------------------------------

class TestC0SamePad:
    def _setup(self):
        wps = [make_waypoint("A"), make_waypoint("B", 100)]
        lane = make_lane("L1", wps, destination_vertiport_id="vp_dest")
        tp = make_takeoff_pad("tp1", x=0, y=0)
        vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=[tp])
        vp_dest = make_vertiport(vertiport_id="vp_dest")
        state = make_system_state(vertiports={"vp_origin": vp_origin, "vp_dest": vp_dest})
        return lane, state

    def test_same_pad_plan_clears_before_t_des_no_delay(self):
        lane, state = self._setup()
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0)
        # Plan departs at 950, takeoff=30 → clears at 980 ≤ t_des=1000
        plan = _make_plan_with_takeoff("j", 950.0, 30.0, "vp_origin", "tp1", lane)
        assert compute_delta_C0(fi, [plan], 1000.0, 1000.0, state) == 0.0

    def test_same_pad_plan_overlaps_requires_delay(self):
        lane, state = self._setup()
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0)
        # Plan departs at 990, takeoff=30 → clears at 1020 > t_des=1000 → wait 20s
        plan = _make_plan_with_takeoff("j", 990.0, 30.0, "vp_origin", "tp1", lane)
        delta = compute_delta_C0(fi, [plan], 1000.0, 1000.0, state)
        assert abs(delta - 20.0) < 1e-9

    def test_same_pad_two_plans_takes_max_delay(self):
        lane, state = self._setup()
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0)
        plan_a = _make_plan_with_takeoff("j_a", 985.0, 30.0, "vp_origin", "tp1", lane)  # clears 1015 → need 15
        plan_b = _make_plan_with_takeoff("j_b", 995.0, 50.0, "vp_origin", "tp1", lane)  # clears 1045 → need 45
        delta = compute_delta_C0(fi, [plan_a, plan_b], 1000.0, 1000.0, state)
        assert abs(delta - 45.0) < 1e-9

    def test_fi_clears_before_j_starts_no_delay(self):
        """fi finishes its climb before j's approved departure: fi can go first."""
        lane, state = self._setup()
        # fi t_takeoff=5 → clears at 1005; j departs at 1010 → fi clears before j starts
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=5.0)
        plan = _make_plan_with_takeoff("j", 1010.0, 30.0, "vp_origin", "tp1", lane)
        assert compute_delta_C0(fi, [plan], 1000.0, 1000.0, state) == 0.0

    def test_fi_and_j_overlap_fi_must_wait(self):
        """fi's climb zone overlaps j's climb zone: fi must wait for j to clear."""
        lane, state = self._setup()
        # fi t_takeoff=30 → would clear at 1030; j departs 1010, clears 1040
        # 1000+30=1030 > 1010 (fi can't clear before j starts) AND 1040 > 1000 (j not done)
        # → overlap → fi waits until 1040 → delta=40
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0)
        plan = _make_plan_with_takeoff("j", 1010.0, 30.0, "vp_origin", "tp1", lane)
        delta = compute_delta_C0(fi, [plan], 1000.0, 1000.0, state)
        assert delta == pytest.approx(40.0)


# ---------------------------------------------------------------------------
# Tests — different pads, distance-based separation
# ---------------------------------------------------------------------------

class TestC0DifferentPads:
    MSD = 50.0  # msd_matrix[("A","A")] in make_system_state

    def _setup(self, pad_distance: float):
        wps = [make_waypoint("A"), make_waypoint("B", 100)]
        lane = make_lane("L1", wps, destination_vertiport_id="vp_dest")
        tp1 = make_takeoff_pad("tp1", x=0, y=0)
        tp2 = make_takeoff_pad("tp2", x=pad_distance, y=0)
        vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=[tp1, tp2])
        vp_dest = make_vertiport(vertiport_id="vp_dest")
        state = make_system_state(vertiports={"vp_origin": vp_origin, "vp_dest": vp_dest})
        return lane, state

    def test_far_pads_no_constraint(self):
        """Pads separated by > MSD: concurrent departure allowed."""
        lane, state = self._setup(pad_distance=60.0)  # > MSD=50
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        plan = _make_plan_with_takeoff("j", 995.0, 30.0, "vp_origin", "tp2", lane)
        assert compute_delta_C0(fi, [plan], 1000.0, 1000.0, state) == 0.0

    def test_close_pads_within_msd_require_delay(self):
        """Pads separated by < MSD: sequential departure required."""
        lane, state = self._setup(pad_distance=30.0)  # < MSD=50
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        # Plan departs 995, takeoff=30 → clears 1025 → wait 25s
        plan = _make_plan_with_takeoff("j", 995.0, 30.0, "vp_origin", "tp2", lane)
        delta = compute_delta_C0(fi, [plan], 1000.0, 1000.0, state)
        assert abs(delta - 25.0) < 1e-9

    def test_pads_at_exactly_msd_distance_no_constraint(self):
        """Pads at exactly MSD distance: no constraint (d >= MSD)."""
        lane, state = self._setup(pad_distance=self.MSD)  # == MSD=50
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        plan = _make_plan_with_takeoff("j", 995.0, 30.0, "vp_origin", "tp2", lane)
        assert compute_delta_C0(fi, [plan], 1000.0, 1000.0, state) == 0.0

    def test_plan_already_cleared_before_t_des(self):
        """Plan clears zone before t_des even though pads are close."""
        lane, state = self._setup(pad_distance=30.0)
        fi = make_fi(lane, t_des=1000.0, origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        plan = _make_plan_with_takeoff("j", 960.0, 30.0, "vp_origin", "tp2", lane)  # clears 990 < 1000
        assert compute_delta_C0(fi, [plan], 1000.0, 1000.0, state) == 0.0
