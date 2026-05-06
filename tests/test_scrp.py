"""Integration tests for resolve_conflict — all 10 required test cases."""
from __future__ import annotations

import math
import time

import pytest

from scrp.headway import h_min_full
from scrp.models import ApprovedPlan, SCRPConfig, ApproveResult, RejectResult
from scrp.scrp import resolve_conflict
from tests.conftest import (
    make_fi,
    make_lane,
    make_system_state,
    make_vertiport,
    make_waypoint,
)


def _committed_plan(
    lane,
    t_start: float,
    v: float = 10.0,
    drone_id: str = "d_existing",
    status: str = "COMMITTED",
    vertiport_id: str = "vp1",
) -> ApprovedPlan:
    cumulative = t_start
    wt = []
    for i, wp in enumerate(lane.waypoints):
        wt.append((wp, cumulative))
        if i < len(lane.segments):
            cumulative += lane.segments[i].length / v
    return ApprovedPlan(
        drone_id=drone_id,
        uav_type="A",
        lane=lane,
        waypoint_times=wt,
        v_waypoints=[v] * len(lane.waypoints),
        t_land=cumulative,
        pad_id="pad1",
        slot_index=0,
        status=status,
        algorithm='SCRP',
        vertiport_id=vertiport_id,
    )


# ── test_no_conflict ───────────────────────────────────────────────────────────

class TestNoConflict:
    def test_no_approved_plans(self, simple_lane, system_state, config):
        fi = make_fi(simple_lane, v=10.0, t_des=1000.0)
        result = resolve_conflict(fi, [], system_state, config)
        assert isinstance(result, ApproveResult)
        assert result.t_dep_star == pytest.approx(1000.0)
        assert result.delay_seconds == pytest.approx(0.0, abs=1e-9)
        assert result.delay_source == 'none'


# ── test_headway_single ────────────────────────────────────────────────────────

class TestHeadwaySingle:
    def test_delay_when_gap_less_than_h_min(self, simple_lane, system_state, config):
        """Existing drone departs 2s before; h_min > 2s → delay needed."""
        t_des = 1002.0
        existing = _committed_plan(simple_lane, t_start=1000.0, v=10.0)
        fi = make_fi(simple_lane, v=10.0, t_des=t_des)

        result = resolve_conflict(fi, [existing], system_state, config)
        assert isinstance(result, ApproveResult)

        h = h_min_full(50.0, 1.0, 10.0, 10.0, 100.0, 5.0)
        if h > 2.0:
            assert result.delay_seconds > 0.0


# ── test_headway_same_velocity ─────────────────────────────────────────────────

class TestHeadwaySameVelocity:
    def test_correction_term_zero(self):
        """When v_i == v_j, correction term == 0."""
        h = h_min_full(msd=50.0, body_len_j=1.0, v_i=10.0, v_j=10.0, seg_length=200.0, v_min_seg=5.0)
        h_basic = (50.0 + 1.0) / 10.0
        assert h == pytest.approx(h_basic, rel=1e-9)


# ── test_headway_different_velocity ───────────────────────────────────────────

class TestHeadwayDifferentVelocity:
    def test_faster_new_drone_adds_correction(self):
        """v_i > v_j → correction term > 0 → h_min_full > h_min_basic."""
        v_i, v_j = 15.0, 10.0
        h_full = h_min_full(50.0, 1.0, v_i, v_j, 200.0, 5.0)
        v_fast = max(v_i, v_j)
        h_basic = (50.0 + 1.0) / v_fast
        assert h_full > h_basic

    def test_slower_new_drone_adds_correction(self):
        v_i, v_j = 8.0, 12.0
        h_full = h_min_full(50.0, 1.0, v_i, v_j, 200.0, 5.0)
        h_basic = (50.0 + 1.0) / max(v_i, v_j)
        assert h_full > h_basic


# ── test_junction_conflict ─────────────────────────────────────────────────────

class TestJunctionConflict:
    def test_junction_causes_delay(self, system_state, config):
        junction = make_waypoint("J", 50, 0)
        wps_new = [make_waypoint("S1", 0, 0), junction, make_waypoint("E1", 100, 0)]
        wps_ex = [make_waypoint("S2", 0, 100), junction, make_waypoint("E2", 100, 100)]
        lane_new = make_lane("ln", wps_new, v_min=5.0, v_max=20.0)
        lane_ex = make_lane("le", wps_ex, v_min=5.0, v_max=20.0)

        fi = make_fi(lane_new, v=10.0, t_des=1000.0)
        # new arrives at J at 1005; existing at J also at 1005
        wt = [
            (lane_ex.waypoints[0], 995.0),
            (junction, 1005.0),
            (lane_ex.waypoints[2], 1015.0),
        ]
        existing = ApprovedPlan(
            drone_id="d2", uav_type="A", lane=lane_ex,
            waypoint_times=wt, v_waypoints=[10.0] * len(lane_ex.waypoints),
            t_land=1015.0, pad_id="pad1",
            slot_index=1, status="COMMITTED", algorithm='SCRP',
        )

        result = resolve_conflict(fi, [existing], system_state, config)
        assert isinstance(result, ApproveResult)
        assert result.delay_seconds > 0.0


# ── test_slot_occupied ─────────────────────────────────────────────────────────

class TestSlotOccupied:
    def test_all_early_slots_committed_forces_later_departure(self, simple_lane, config):
        vp = make_vertiport(n_slots=20, slot_duration=30.0)
        # Occupy first 5 slots
        for i in range(5):
            vp.slot_status[("pad1", i)] = "COMMITTED"
            vp.slots[("pad1", i)] = f"drone_{i}"
        state = make_system_state(vertiport=vp)
        fi = make_fi(simple_lane, v=10.0, t_des=0.0)

        result = resolve_conflict(fi, [], state, config)
        assert isinstance(result, ApproveResult)
        assert result.slot_index >= 5


# ── test_SoC_insufficient ─────────────────────────────────────────────────────

class TestSoCInsufficient:
    def test_low_battery_rejected(self, config):
        # Very long route with tiny battery
        wps = [make_waypoint(f"P{i}", i * 1000, 0) for i in range(5)]
        lane = make_lane("long", wps, v_min=5.0, v_max=20.0)
        # SoC_0=0.21 (just above min), tiny C_bat → depleted quickly
        fi = make_fi(lane, v=10.0, t_des=1000.0, SoC_0=0.21, C_bat=1.0, P_hover=500.0)
        vp = make_vertiport()
        state = make_system_state(vertiport=vp)

        result = resolve_conflict(fi, [], state, config)
        assert isinstance(result, RejectResult)
        assert result.reason == 'SoC_insufficient'


# ── test_soft_reservation_treated_as_committed ────────────────────────────────

class TestSoftReservedTreatedAsCommitted:
    def test_soft_plan_causes_same_delay_as_committed(self, simple_lane, system_state, config):
        t_des = 1002.0

        committed_plan = _committed_plan(simple_lane, t_start=1000.0, v=10.0, status="COMMITTED")
        soft_plan = _committed_plan(simple_lane, t_start=1000.0, v=10.0, status="SOFT_RESERVED", drone_id="d_soft")
        soft_plan.expires_at = 9999.0

        fi = make_fi(simple_lane, v=10.0, t_des=t_des)

        r1 = resolve_conflict(fi, [committed_plan], system_state, config)
        r2 = resolve_conflict(fi, [soft_plan], system_state, config)

        assert isinstance(r1, ApproveResult)
        assert isinstance(r2, ApproveResult)
        assert r1.t_dep_star == pytest.approx(r2.t_dep_star, rel=1e-9)


# ── test_soft_reservation_timeout ─────────────────────────────────────────────

class TestSoftReservationTimeout:
    def test_expired_soft_excluded_from_conflict(self, simple_lane, config):
        """After expiring a SOFT plan, C1 headway delay must be zero.

        Any remaining delay comes only from C3 (slot alignment) — not from the
        purged plan, confirming it was correctly removed.
        """
        from scrp.reservation import expire_soft_reservations

        expired_plan = _committed_plan(simple_lane, t_start=1000.0, v=10.0, status="SOFT_RESERVED", drone_id="d_expired")
        expired_plan.expires_at = 500.0  # already expired at t_now=900

        vp = make_vertiport(n_slots=50)  # plenty of slots
        state = make_system_state([expired_plan], vp, t_now=900.0)

        # Caller is expected to expire reservations before calling resolve_conflict
        cleaned_state = expire_soft_reservations(state, state.t_now)
        assert cleaned_state.approved_plans == []  # expired plan was removed

        fi = make_fi(simple_lane, v=10.0, t_des=1002.0)
        result = resolve_conflict(fi, cleaned_state.approved_plans, cleaned_state, config)
        assert isinstance(result, ApproveResult)
        # No C1 headway delay because expired plan was removed
        assert result.delay_source != 'C1_headway'


# ── test_closed_form_correctness ──────────────────────────────────────────────

class TestClosedFormCorrectness:
    def test_t_dep_star_satisfies_all_constraints(self, simple_lane, config):
        """Verify that after t_dep*, the new plan respects headway against the existing plan."""
        from scrp.geometry import t_arrival_at_waypoint
        from scrp.headway import h_min_full

        t_des = 1002.0
        existing = _committed_plan(simple_lane, t_start=1000.0, v=10.0)
        fi = make_fi(simple_lane, v=10.0, t_des=t_des)
        vp = make_vertiport()
        state = make_system_state(vertiport=vp)

        result = resolve_conflict(fi, [existing], state, config)
        assert isinstance(result, ApproveResult)

        # Rebuild fi with adjusted t_dep* to verify no C1 violation
        from scrp.models import FlightIntention
        fi_adjusted = FlightIntention(
            drone_id=fi.drone_id, uav_type=fi.uav_type, lane=fi.lane,
            v_waypoints=fi.v_waypoints, t_des=result.t_dep_star,
            SoC_0=fi.SoC_0, C_bat=fi.C_bat, P_hover=fi.P_hover,
            priority=fi.priority, operator_id=fi.operator_id,
        )

        plan_times = {w.id: t for w, t in existing.waypoint_times}
        for idx_new in range(len(fi_adjusted.lane.segments)):
            wp_id = fi_adjusted.lane.waypoints[idx_new].id
            tau_k = plan_times.get(wp_id)
            if tau_k is None:
                continue
            t_new_arr = t_arrival_at_waypoint(fi_adjusted, idx_new, result.t_dep_star)
            if t_new_arr < tau_k:
                continue
            seg = fi_adjusted.lane.segments[idx_new]
            h = h_min_full(50.0, 1.0, 10.0, 10.0, seg.length, seg.v_min)
            gap = t_new_arr - tau_k
            assert gap >= h - 1e-6, f"Headway violated at waypoint {idx_new}: gap={gap:.4f} < h={h:.4f}"


# ── Performance test ───────────────────────────────────────────────────────────

class TestPerformance:
    def test_resolve_under_200ms_with_500_plans(self, config):
        """resolve_conflict must complete in < 200ms with 500 approved plans."""
        # Build a lane with 20 waypoints
        wps = [make_waypoint(f"P{i}", i * 50, 0) for i in range(20)]
        lane = make_lane("perf_lane", wps, v_min=5.0, v_max=20.0)
        fi = make_fi(lane, v=10.0, t_des=5000.0, SoC_0=1.0, C_bat=100000.0)
        vp = make_vertiport(n_slots=600, slot_duration=30.0)
        state = make_system_state(vertiport=vp, t_now=4000.0)

        # 500 approved plans on the same lane (worst case for C1)
        plans: list[ApprovedPlan] = []
        for i in range(500):
            t_start = 1000.0 + i * 20.0
            plan = _committed_plan(lane, t_start=t_start, v=10.0, drone_id=f"drone_{i}")
            plan.slot_index = i % 580
            plans.append(plan)

        start = time.perf_counter()
        result = resolve_conflict(fi, plans, state, config)
        elapsed_ms = (time.perf_counter() - start) * 1000.0

        assert elapsed_ms < 200.0, f"resolve_conflict took {elapsed_ms:.1f}ms (limit: 200ms)"

    def test_resolve_under_200ms_20_waypoints(self, config):
        wps = [make_waypoint(f"P{i}", i * 100, 0) for i in range(20)]
        lane = make_lane("long_lane", wps, v_min=5.0, v_max=20.0)
        fi = make_fi(lane, v=10.0, t_des=5000.0, SoC_0=1.0, C_bat=100000.0)
        vp = make_vertiport(n_slots=300, slot_duration=30.0)
        state = make_system_state(vertiport=vp)

        plans: list[ApprovedPlan] = []
        for i in range(500):
            plan = _committed_plan(lane, t_start=1000.0 + i * 25.0, drone_id=f"d{i}")
            plan.slot_index = i % 280
            plans.append(plan)

        start = time.perf_counter()
        resolve_conflict(fi, plans, state, config)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        assert elapsed_ms < 200.0, f"resolve_conflict took {elapsed_ms:.1f}ms"


# ── Validation tests ───────────────────────────────────────────────────────────

class TestValidation:
    def test_invalid_fi_wrong_v_waypoints_length(self, simple_lane, system_state, config):
        fi = make_fi(simple_lane, v=10.0)
        fi.v_waypoints = [10.0]  # wrong length
        result = resolve_conflict(fi, [], system_state, config)
        assert isinstance(result, RejectResult)
        assert result.reason == 'invalid_fi'

    def test_invalid_fi_soc_out_of_range(self, simple_lane, system_state, config):
        fi = make_fi(simple_lane, v=10.0, SoC_0=1.5)
        result = resolve_conflict(fi, [], system_state, config)
        assert isinstance(result, RejectResult)
        assert result.reason == 'invalid_fi'

    def test_no_slot_available_rejected(self, simple_lane, config):
        # Commit all known slots so no free slot remains
        vp = make_vertiport(n_slots=200, slot_duration=30.0)
        for i in range(200):
            vp.slot_status[("pad1", i)] = "COMMITTED"
            vp.slots[("pad1", i)] = f"d{i}"
        state = make_system_state(vertiport=vp)
        fi = make_fi(simple_lane, v=10.0, t_des=0.0)
        result = resolve_conflict(fi, [], state, config)
        assert isinstance(result, RejectResult)
        assert result.reason == 'no_slot_available'
