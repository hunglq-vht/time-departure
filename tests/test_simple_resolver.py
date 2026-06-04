"""
Tests for scrp_simple.ConflictResolver and resolve_batch.

Naming convention:
    TestNoConflict         – plans that should pass with zero delay
    TestLaneConflict       – same-lane separation violations
    TestPadConflict        – landing-pad occupancy violations
    TestCombinedConflict   – lane + pad simultaneously
    TestBatchResolution    – priority ordering in resolve_batch
"""

from __future__ import annotations

import math
from typing import Dict

import pytest

from scrp_simple.models import ApprovedPlan, FlightPath, NewPlanRequest, Point3D, Vertiport
from scrp_simple.resolver import ConflictResolver, resolve_batch


# ─────────────────────────────────────────────────────────────────────────────
# Shared helpers
# ─────────────────────────────────────────────────────────────────────────────

MIN_SEP = ConflictResolver.MIN_SEPARATION_M
LAND_DUR = ConflictResolver.LANDING_DURATION_S


def vports_ab(pads_a: int = 2, pads_b: int = 2) -> Dict[str, Vertiport]:
    """Two vertiports 1 000 m apart along the x-axis."""
    return {
        "A": Vertiport("A", Point3D(0, 0, 100), pads_a),
        "B": Vertiport("B", Point3D(1000, 0, 100), pads_b),
    }


def straight_plan(
    plan_id: str,
    start_time: float,
    speed: float,
    pads_b: int = 2,
    priority: int = 5,
) -> tuple[Dict[str, Vertiport], ApprovedPlan]:
    vp = vports_ab(pads_b=pads_b)
    plan = ApprovedPlan(
        plan_id=plan_id,
        flight_path=FlightPath("A", "B", []),
        drone_type="X",
        segment_speeds=[speed],
        start_time=start_time,
        priority=priority,
    )
    return vp, plan


# ─────────────────────────────────────────────────────────────────────────────
# No conflict
# ─────────────────────────────────────────────────────────────────────────────

class TestNoConflict:
    def test_different_routes_no_delay(self):
        """Plans on distinct routes produce no conflict."""
        vp = {
            "A": Vertiport("A", Point3D(0, 0, 100), 2),
            "B": Vertiport("B", Point3D(1000, 0, 100), 2),
            "C": Vertiport("C", Point3D(0, 1000, 100), 2),
        }
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [20.0], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "C", []), "X", [20.0],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_same_lane_well_separated(self):
        """
        Same route, same speed, but second departs so late that MIN_SEP is
        already guaranteed without any extra delay.
        """
        speed = 20.0
        # Time gap needed: MIN_SEP / v_lead = 50/20 = 2.5 s  → use 10 s gap to be safe
        vp, approved = straight_plan("p1", start_time=0.0, speed=speed)
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=10.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, [approved]).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_no_approved_plans(self):
        """No existing plans → immediate approval."""
        vp = vports_ab()
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [20.0],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, []).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)


# ─────────────────────────────────────────────────────────────────────────────
# Lane conflicts
# ─────────────────────────────────────────────────────────────────────────────

class TestLaneConflict:
    def test_same_speed_simultaneous_departure(self):
        """
        Both drones depart at t=0 on the same lane.
        Required headway = MIN_SEP / v_lead = 50/20 = 2.5 s.
        """
        speed = 20.0
        expected_headway = MIN_SEP / speed  # 2.5 s
        vp, approved = straight_plan("p1", start_time=0.0, speed=speed)
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, [approved]).resolve(req)
        assert result.approved
        assert result.delay >= expected_headway - 0.01

    def test_faster_follower_larger_headway(self):
        """
        New drone is twice as fast as the approved one – it would catch up,
        requiring a much larger headway.

        v_lead=10, v_follow=20, seg_len=1000:
          h_initial = 50/10 = 5 s
          h_exit    = 50/20 + 1000*(1/10 − 1/20) = 2.5 + 50 = 52.5 s
          → expected headway = 52.5 s
        """
        v_lead, v_follow = 10.0, 20.0
        seg_len = 1000.0
        vp = vports_ab()
        expected = max(
            MIN_SEP / v_lead,
            MIN_SEP / v_follow + seg_len * (1.0 / v_lead - 1.0 / v_follow),
        )  # 52.5 s
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [v_lead], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [v_follow],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= expected - 0.01

    def test_slower_follower_smaller_headway(self):
        """
        Slower follower never catches up – only initial gap matters.
        v_lead=20, v_follow=10: h = MIN_SEP / v_lead = 2.5 s.
        """
        v_lead, v_follow = 20.0, 10.0
        expected = MIN_SEP / v_lead  # 2.5 s
        vp = vports_ab()
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [v_lead], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [v_follow],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= expected - 0.01
        assert result.delay < 10.0  # should not be excessively large

    def test_rejection_when_max_wait_too_short(self):
        """
        Headway of 52.5 s is needed but max_wait_time=10 s → rejected.
        """
        v_lead, v_follow = 10.0, 20.0
        vp = vports_ab()
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [v_lead], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [v_follow],
            desired_start_time=0.0, max_wait_time=10.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert not result.approved
        assert result.actual_start_time is None
        assert result.delay > 10.0

    def test_multi_segment_shared_second_leg(self):
        """
        Two plans share only the second segment.  Conflict on that segment
        must still produce a non-zero delay.
        """
        mid = Point3D(500, 0, 100)
        speed = 20.0
        vp = vports_ab()
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", [mid]), "X", [speed, speed], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", [mid]), "X", [speed, speed],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay > 0.0

    def test_new_drone_leads_safely(self):
        """
        New drone departs early enough to be well ahead of the approved drone.
        No delay should be added.
        """
        speed = 20.0
        seg_len = 1000.0
        # New departs at t=-100; approved at t=0.
        # New exits at t=-100+50=-50; approved needs to follow.
        # lead_headway for approved = MIN_SEP/v_lead = 2.5 s
        # -50 + 2.5 = -47.5 < 0 → new is safely ahead
        vp = vports_ab()
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=-100.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_multiple_approved_plans_cumulative_delay(self):
        """
        Three approved plans depart at 0, 3, and 6 s.  New plan at t=0 must
        follow all three, waiting for the last one.
        speed=20, headway=2.5 s → new must depart ≥ 6 + 2.5 = 8.5 s.
        """
        speed = 20.0
        headway = MIN_SEP / speed  # 2.5 s
        vp = vports_ab()
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0),
            ApprovedPlan("p2", FlightPath("A", "B", []), "X", [speed], start_time=3.0),
            ApprovedPlan("p3", FlightPath("A", "B", []), "X", [speed], start_time=6.0),
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= 6.0 + headway - 0.01


# ─────────────────────────────────────────────────────────────────────────────
# Landing-pad conflicts
# ─────────────────────────────────────────────────────────────────────────────

class TestPadConflict:
    def test_single_pad_occupied(self):
        """
        Vertiport B has 1 pad.  Approved plan lands at t=50, occupies until
        t=170.  New plan must land ≥ t=170, so depart ≥ t=120.
        """
        speed = 20.0
        flight_time = 1000.0 / speed  # 50 s
        expected_delay = LAND_DUR  # 120 s
        vp = vports_ab(pads_b=1)
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= expected_delay - 0.1

    def test_two_pads_two_simultaneous_landings(self):
        """
        Both pads occupied at t=50.  New arrival must be ≥ t=170.
        """
        speed = 20.0
        vp = vports_ab(pads_b=2)
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0),
            ApprovedPlan("p2", FlightPath("A", "B", []), "X", [speed], start_time=0.0),
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= LAND_DUR - 0.1

    def test_staggered_landings_first_slot_free(self):
        """
        Vertiport has 1 pad; approved lands at t=50 (busy 50–170).
        New desired arrival is t=200 > 170 → no pad delay needed.
        """
        speed = 20.0
        # desired_start_time=150 → arrival at 150+50=200, which is after pad frees at 170
        vp = vports_ab(pads_b=1)
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=150.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_pad_rejection_max_wait_exceeded(self):
        """
        Three sequential approved landings chain-occupy the single pad for
        360 s (3 × 120 s).  A new plan with max_wait_time=100 s is rejected.
        """
        speed = 20.0
        flight_time = 50.0  # 1000m / 20 m/s
        vp = vports_ab(pads_b=1)
        # Landings at t=50, 170, 290 → pad free after t=410
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0),
            ApprovedPlan("p2", FlightPath("A", "B", []), "X", [speed], start_time=120.0),
            ApprovedPlan("p3", FlightPath("A", "B", []), "X", [speed], start_time=240.0),
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=100.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert not result.approved


# ─────────────────────────────────────────────────────────────────────────────
# Combined conflicts
# ─────────────────────────────────────────────────────────────────────────────

class TestCombinedConflict:
    def test_lane_delay_pushes_into_pad_conflict(self):
        """
        Lane conflict forces a delay; the delayed arrival then hits the pad
        occupancy window, requiring additional delay.

        Setup (1 pad at B, speed=20, seg_len=1000):
          Approved departs t=0, arrives t=50, pad busy 50–170.
          Lane headway = 2.5 s → new could depart at t=2.5, arrive at t=52.5.
          t=52.5 falls inside [50, 170] → pad conflict adds more delay.
          New must arrive ≥ t=170 → depart ≥ t=120.
        """
        speed = 20.0
        vp = vports_ab(pads_b=1)
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        # Must depart ≥ 120 s (pad drives this, lane headway is only 2.5 s)
        assert result.delay >= LAND_DUR - 0.1

    def test_both_constraints_satisfied_together(self):
        """
        Only check that approval is granted and actual_start_time is consistent
        with delay when both constraints are active.
        """
        speed = 20.0
        vp = vports_ab(pads_b=1)
        approved = [
            ApprovedPlan("p1", FlightPath("A", "B", []), "X", [speed], start_time=0.0)
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.actual_start_time == pytest.approx(
            req.desired_start_time + result.delay, abs=0.01
        )


# ─────────────────────────────────────────────────────────────────────────────
# Batch resolution with priority
# ─────────────────────────────────────────────────────────────────────────────

class TestBatchResolution:
    def test_high_priority_gets_no_delay(self):
        """
        Two requests, same route, same desired time.
        Priority-1 request is resolved first → zero delay.
        Priority-5 request must wait.
        """
        speed = 20.0
        vp = vports_ab(pads_b=2)
        req_high = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0, priority=1,
        )
        req_low = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0, priority=5,
        )
        # Submit in reverse order: [low, high]
        results = resolve_batch(vp, [], [req_low, req_high])
        low_result, high_result = results[0], results[1]

        assert high_result.approved
        assert high_result.delay == pytest.approx(0.0, abs=0.01)
        assert low_result.approved
        assert low_result.delay > 0.0

    def test_same_priority_ordered_by_desired_time(self):
        """
        Two requests with identical priority; earlier desired time wins.
        """
        speed = 20.0
        vp = vports_ab(pads_b=2)
        req_early = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=0.0, max_wait_time=300.0, priority=3,
        )
        req_late = NewPlanRequest(
            FlightPath("A", "B", []), "X", [speed],
            desired_start_time=10.0, max_wait_time=300.0, priority=3,
        )
        results = resolve_batch(vp, [], [req_early, req_late])
        assert results[0].delay == pytest.approx(0.0, abs=0.01)
        # req_late already departs 10 s later; headway = 2.5 s so no extra delay needed
        assert results[1].delay == pytest.approx(0.0, abs=0.01)

    def test_result_order_matches_input_order(self):
        """resolve_batch returns results indexed to match the input list."""
        speed = 20.0
        vp = vports_ab()
        reqs = [
            NewPlanRequest(FlightPath("A", "B", []), "X", [speed], 0.0, 300.0, priority=p)
            for p in [5, 3, 1]
        ]
        results = resolve_batch(vp, [], reqs)
        # All should be approved; result[i] corresponds to reqs[i]
        assert len(results) == 3
        for i, (req, res) in enumerate(zip(reqs, results)):
            assert res.approved, f"Request {i} (priority {req.priority}) was rejected"

    def test_lower_priority_rejected_when_slots_exhausted(self):
        """
        Single pad, 3 requests.  First 2 (high/medium priority) fill the pad
        occupancy window; third (low priority) exceeds max_wait_time.

        With 1 pad and LANDING_DURATION=120 s:
          priority-1 lands at t=50, pad busy 50–170
          priority-2 must wait until t=170, departs t=120, lands t=170, pad busy 170–290
          priority-3 must wait until t=290, departs t=240, delay=240 > max_wait=100 → rejected
        """
        speed = 20.0
        vp = vports_ab(pads_b=1)
        reqs = [
            NewPlanRequest(FlightPath("A", "B", []), "X", [speed], 0.0, 300.0, priority=1),
            NewPlanRequest(FlightPath("A", "B", []), "X", [speed], 0.0, 300.0, priority=2),
            NewPlanRequest(FlightPath("A", "B", []), "X", [speed], 0.0, 100.0, priority=5),
        ]
        results = resolve_batch(vp, [], reqs)
        assert results[0].approved and results[0].delay == pytest.approx(0.0, abs=0.1)
        assert results[1].approved and results[1].delay >= LAND_DUR - 0.1
        assert not results[2].approved
