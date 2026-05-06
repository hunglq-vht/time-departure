"""Tests for reservation lifecycle management."""
from __future__ import annotations

import pytest

from scrp.reservation import (
    cascade_notify,
    commit_reservation,
    expire_soft_reservations,
    release_reservation,
)
from scrp.models import ApprovedPlan
from tests.conftest import make_lane, make_system_state, make_vertiport, make_waypoint


def _make_soft_plan(drone_id: str, lane, expires_at: float, vertiport_id: str = "vp1") -> ApprovedPlan:
    wt = [(wp, float(i * 10)) for i, wp in enumerate(lane.waypoints)]
    return ApprovedPlan(
        drone_id=drone_id,
        uav_type="A",
        lane=lane,
        waypoint_times=wt,
        v_waypoints=[10.0] * len(lane.waypoints),
        t_land=100.0,
        pad_id="pad1",
        slot_index=5,
        status="SOFT_RESERVED",
        algorithm='SCRP',
        expires_at=expires_at,
        vertiport_id=vertiport_id,
    )


class TestCommitReservation:
    def test_commit_changes_status(self):
        wps = [make_waypoint("P1"), make_waypoint("P2")]
        lane = make_lane("l", wps)
        plan = _make_soft_plan("d1", lane, 9999.0)
        vp = make_vertiport()
        vp.slot_status[("pad1", 5)] = "SOFT"
        state = make_system_state([plan], vp)

        new_state = commit_reservation("d1", state)
        committed = next(p for p in new_state.approved_plans if p.drone_id == "d1")
        assert committed.status == "COMMITTED"
        assert committed.expires_at is None
        assert new_state.vertiports["vp1"].slot_status[("pad1", 5)] == "COMMITTED"

    def test_commit_does_not_mutate_original(self):
        wps = [make_waypoint("P1"), make_waypoint("P2")]
        lane = make_lane("l", wps)
        plan = _make_soft_plan("d1", lane, 9999.0)
        state = make_system_state([plan])
        commit_reservation("d1", state)
        assert state.approved_plans[0].status == "SOFT_RESERVED"


class TestReleaseReservation:
    def test_release_removes_plan_and_frees_slot(self):
        wps = [make_waypoint("P1"), make_waypoint("P2")]
        lane = make_lane("l", wps)
        plan = _make_soft_plan("d1", lane, 9999.0)
        vp = make_vertiport()
        vp.slot_status[("pad1", 5)] = "SOFT"
        vp.slots[("pad1", 5)] = "d1"
        state = make_system_state([plan], vp)

        new_state = release_reservation("d1", state)
        assert not any(p.drone_id == "d1" for p in new_state.approved_plans)
        assert new_state.vertiports["vp1"].slot_status[("pad1", 5)] == "FREE"
        assert new_state.vertiports["vp1"].slots[("pad1", 5)] is None


class TestExpireSoftReservations:
    def test_expired_plan_removed(self):
        wps = [make_waypoint("P1"), make_waypoint("P2")]
        lane = make_lane("l", wps)
        plan = _make_soft_plan("d1", lane, expires_at=500.0)
        vp = make_vertiport()
        vp.slot_status[("pad1", 5)] = "SOFT"
        vp.slots[("pad1", 5)] = "d1"
        state = make_system_state([plan], vp, t_now=600.0)

        new_state = expire_soft_reservations(new_state := make_system_state([plan], vp, t_now=900.0), 900.0)
        assert not any(p.drone_id == "d1" for p in new_state.approved_plans)
        assert new_state.vertiports["vp1"].slot_status[("pad1", 5)] == "FREE"

    def test_non_expired_plan_kept(self):
        wps = [make_waypoint("P1"), make_waypoint("P2")]
        lane = make_lane("l", wps)
        plan = _make_soft_plan("d1", lane, expires_at=2000.0)
        state = make_system_state([plan], t_now=900.0)

        new_state = expire_soft_reservations(state, 900.0)
        assert any(p.drone_id == "d1" for p in new_state.approved_plans)


class TestCascadeNotify:
    def test_cascade_finds_dependent_plans(self):
        shared_wp = make_waypoint("J", 50, 0)
        wps_a = [make_waypoint("A1"), shared_wp]
        wps_b = [shared_wp, make_waypoint("B2")]
        lane_a = make_lane("la", wps_a)
        lane_b = make_lane("lb", wps_b)

        plan_a = _make_soft_plan("rejected_drone", lane_a, 9999.0)
        plan_b = _make_soft_plan("dependent_drone", lane_b, 9999.0)
        plan_b.slot_index = 6

        state = make_system_state([plan_a, plan_b])
        affected = cascade_notify("rejected_drone", state)
        assert "dependent_drone" in affected

    def test_cascade_empty_when_no_shared_waypoints(self):
        wps_a = [make_waypoint("A1"), make_waypoint("A2")]
        wps_b = [make_waypoint("B1"), make_waypoint("B2")]
        lane_a = make_lane("la", wps_a)
        lane_b = make_lane("lb", wps_b)

        plan_a = _make_soft_plan("rejected_drone", lane_a, 9999.0)
        plan_b = _make_soft_plan("other_drone", lane_b, 9999.0)
        plan_b.slot_index = 6

        state = make_system_state([plan_a, plan_b])
        affected = cascade_notify("rejected_drone", state)
        assert affected == []
