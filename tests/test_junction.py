"""Tests for junction (C2) computation."""
from __future__ import annotations

import pytest

from scrp.junction import compute_delta_C2, detect_junctions
from scrp.models import ApprovedPlan, SCRPConfig
from tests.conftest import make_fi, make_lane, make_system_state, make_waypoint


def make_plan_with_junction(lane, junction_wp, t_junction: float, drone_id: str = "d2") -> ApprovedPlan:
    """Build a plan where the drone reaches the junction waypoint at t_junction."""
    wt = [(wp, t_junction if wp.id == junction_wp.id else 0.0) for wp in lane.waypoints]
    return ApprovedPlan(
        drone_id=drone_id,
        uav_type="A",
        lane=lane,
        waypoint_times=wt,
        v_waypoints=[10.0] * len(lane.waypoints),
        t_land=t_junction + 10.0,
        pad_id="pad1",
        slot_index=1,
        status="COMMITTED",
        algorithm='SCRP',
    )


class TestDetectJunctions:
    def test_no_junction_separate_lanes(self):
        wps_a = [make_waypoint("A1"), make_waypoint("A2")]
        wps_b = [make_waypoint("B1"), make_waypoint("B2")]
        lane_a = make_lane("a", wps_a)
        lane_b = make_lane("b", wps_b)
        junctions = detect_junctions(lane_a, [lane_a, lane_b])
        assert junctions == []

    def test_junction_detected(self):
        shared = make_waypoint("J", 50, 0)
        wps_a = [make_waypoint("A1"), shared, make_waypoint("A2")]
        wps_b = [make_waypoint("B1"), shared, make_waypoint("B2")]
        lane_a = make_lane("a", wps_a)
        lane_b = make_lane("b", wps_b)
        junctions = detect_junctions(lane_a, [lane_a, lane_b])
        assert len(junctions) == 1
        assert junctions[0].id == "J"


class TestDeltaC2:
    def test_no_junction_no_delay(self):
        wps = [make_waypoint("P1", 0, 0), make_waypoint("P2", 100, 0)]
        lane = make_lane("l", wps)
        fi = make_fi(lane, v=10.0, t_des=1000.0)
        state = make_system_state()
        config = SCRPConfig()
        delta = compute_delta_C2(fi, [], 1000.0, state, config)
        assert delta == 0.0

    def test_junction_conflict_simultaneous(self):
        """Two drones arrive at junction at same time → delta_C2 = Delta_t_J."""
        junction = make_waypoint("J", 50, 0)
        wps_new = [make_waypoint("S1", 0, 0), junction, make_waypoint("E1", 100, 0)]
        wps_existing = [make_waypoint("S2", 50, 100), junction, make_waypoint("E2", 50, -100)]
        lane_new = make_lane("ln", wps_new, v_min=5.0, v_max=20.0)
        lane_existing = make_lane("le", wps_existing, v_min=5.0, v_max=20.0)

        t_des = 1000.0
        v = 10.0
        fi = make_fi(lane_new, v=v, t_des=t_des)
        # new drone arrives at J at t_des + 50/v = 1005

        # existing drone arrives at J at same time 1005
        wt = [
            (lane_existing.waypoints[0], 995.0),
            (junction, 1005.0),
            (lane_existing.waypoints[2], 1015.0),
        ]
        existing_plan = ApprovedPlan(
            drone_id="d2",
            uav_type="A",
            lane=lane_existing,
            waypoint_times=wt,
            v_waypoints=[10.0] * len(lane_existing.waypoints),
            t_land=1015.0,
            pad_id="pad1",
            slot_index=1,
            status="COMMITTED",
            algorithm='SCRP',
        )

        state = make_system_state()
        config = SCRPConfig(JUNCTION_DIAMETER_M=20.0)
        delta = compute_delta_C2(fi, [existing_plan], t_des, state, config)

        delta_t_J = 20.0 / 5.0  # diameter / v_min
        # New arrives at same time as existing → gap = 0 < delta_t_J → needs delta_t_J delay
        assert delta == pytest.approx(delta_t_J, rel=1e-6)

    def test_junction_new_arrives_before_no_delay(self):
        """New drone arrives well before existing drone → no delay needed."""
        junction = make_waypoint("J", 50, 0)
        wps_new = [make_waypoint("S1", 0, 0), junction, make_waypoint("E1", 100, 0)]
        wps_existing = [make_waypoint("S2", 50, 100), junction, make_waypoint("E2", 50, -100)]
        lane_new = make_lane("ln", wps_new, v_min=5.0, v_max=20.0)
        lane_existing = make_lane("le", wps_existing, v_min=5.0, v_max=20.0)

        fi = make_fi(lane_new, v=10.0, t_des=1000.0)
        # new drone at J: t=1005
        # existing at J: t=1100 → gap = 1005 - 1100 = -95 → new is before, no delay
        wt = [
            (lane_existing.waypoints[0], 1090.0),
            (junction, 1100.0),
            (lane_existing.waypoints[2], 1110.0),
        ]
        existing_plan = ApprovedPlan(
            drone_id="d2",
            uav_type="A",
            lane=lane_existing,
            waypoint_times=wt,
            v_waypoints=[10.0] * len(lane_existing.waypoints),
            t_land=1110.0,
            pad_id="pad1",
            slot_index=2,
            status="COMMITTED",
            algorithm='SCRP',
        )

        state = make_system_state()
        config = SCRPConfig()
        delta = compute_delta_C2(fi, [existing_plan], 1000.0, state, config)
        assert delta == pytest.approx(0.0, abs=1e-9)
