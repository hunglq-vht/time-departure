"""Tests for headway (C1) computation."""
from __future__ import annotations

import pytest

from scrp.headway import compute_delta_C1, h_min_full
from scrp.models import ApprovedPlan
from tests.conftest import make_fi, make_lane, make_system_state, make_waypoint


def make_plan_on_lane(lane, t_start: float, v: float = 10.0, drone_id: str = "d_existing") -> ApprovedPlan:
    """Create an approved plan for a drone traveling `lane` starting at t_start."""
    cumulative = t_start
    waypoint_times = []
    for i, wp in enumerate(lane.waypoints):
        waypoint_times.append((wp, cumulative))
        if i < len(lane.segments):
            cumulative += lane.segments[i].length / v
    return ApprovedPlan(
        drone_id=drone_id,
        uav_type="A",
        lane=lane,
        waypoint_times=waypoint_times,
        v_waypoints=[v] * len(lane.waypoints),
        t_land=cumulative,
        pad_id="pad1",
        slot_index=0,
        status="COMMITTED",
        algorithm='SCRP',
    )


class TestHMinFull:
    def test_same_velocity_no_correction(self):
        msd, body, v_i, v_j, L, v_min = 50.0, 1.0, 10.0, 10.0, 100.0, 5.0
        result = h_min_full(msd, body, v_i, v_j, L, v_min)
        expected = (msd + body) / max(v_i, v_j)
        assert result == pytest.approx(expected)

    def test_different_velocity_correction_positive(self):
        msd, body = 50.0, 1.0
        v_i, v_j = 15.0, 10.0  # new drone faster
        L, v_min = 100.0, 5.0
        result = h_min_full(msd, body, v_i, v_j, L, v_min)
        v_fast = 15.0
        base = (msd + body) / v_fast
        correction = abs(v_i - v_j) * L / (v_fast ** 2)
        assert result == pytest.approx(base + correction)
        assert result > base  # correction adds to base

    def test_slower_new_drone(self):
        msd, body = 50.0, 1.0
        v_i, v_j = 8.0, 12.0  # new drone slower
        L, v_min = 100.0, 5.0
        result = h_min_full(msd, body, v_i, v_j, L, v_min)
        v_fast = 12.0
        base = (msd + body) / v_fast
        correction = abs(v_i - v_j) * L / (v_fast ** 2)
        assert result == pytest.approx(base + correction)


class TestDeltaC1:
    def test_no_plans_zero_delta(self):
        wps = [make_waypoint("P1", 0, 0), make_waypoint("P2", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, t_des=1000.0)
        state = make_system_state()
        delta = compute_delta_C1(fi, [], 1000.0, state)
        assert delta == 0.0

    def test_headway_single_plan_same_lane(self):
        """Existing drone departs 10s before new drone; h_min_full > 10 → delay needed."""
        wps = [make_waypoint("P1", 0, 0), make_waypoint("P2", 200, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        fi = make_fi(lane, v=10.0, t_des=1010.0)
        existing_plan = make_plan_on_lane(lane, t_start=1000.0, v=10.0)

        state = make_system_state()
        delta = compute_delta_C1(fi, [existing_plan], 1010.0, state)

        # h_min_full with same velocity = (50 + 1) / 10 = 5.1s
        # existing at P1 is t=1000, new at P1 is t=1010 → gap=10s > h=5.1s → no delay
        assert delta == pytest.approx(0.0, abs=1e-9)

    def test_headway_violation_triggers_delay(self):
        """Existing drone 2s before new drone; h_min_full=5.1s → need 3.1s delay."""
        wps = [make_waypoint("P1", 0, 0), make_waypoint("P2", 200, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        fi = make_fi(lane, v=10.0, t_des=1002.0)
        existing_plan = make_plan_on_lane(lane, t_start=1000.0, v=10.0)

        state = make_system_state()
        delta = compute_delta_C1(fi, [existing_plan], 1002.0, state)

        msd = 50.0
        body = 1.0
        h = h_min_full(msd, body, 10.0, 10.0, 200.0, 5.0)
        expected = max(0.0, 1000.0 + h - 1002.0)
        assert delta == pytest.approx(expected, rel=1e-6)

    def test_soft_reserved_treated_as_committed(self):
        """SOFT_RESERVED plan must be treated same as COMMITTED."""
        wps = [make_waypoint("P1", 0, 0), make_waypoint("P2", 200, 0)]
        lane = make_lane("l", wps, v_min=5.0, v_max=20.0)
        fi = make_fi(lane, v=10.0, t_des=1002.0)
        soft_plan = make_plan_on_lane(lane, t_start=1000.0, v=10.0)
        soft_plan.status = "SOFT_RESERVED"
        soft_plan.expires_at = 1300.0

        state = make_system_state()
        delta_soft = compute_delta_C1(fi, [soft_plan], 1002.0, state)
        committed_plan = make_plan_on_lane(lane, t_start=1000.0, v=10.0)
        delta_committed = compute_delta_C1(fi, [committed_plan], 1002.0, state)

        assert delta_soft == pytest.approx(delta_committed)
