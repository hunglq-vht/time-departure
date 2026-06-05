"""
Tests for scrp_simple – ConflictResolver and resolve_batch.

Geometry used throughout
------------------------
Vertiport A: (0, 0, 0)       Vertiport B: (1000, 0, 0)
Default lane waypoints (above the vertiports at altitude 100 m):
    wp_A = Point3D(0,    0, 100)   ← wp that sits above A
    wp_B = Point3D(1000, 0, 100)   ← wp that sits above B

For a plan A→B with [wp_A, wp_B]:
    lane segment: wp_A → wp_B, length = 1000 m
    segment_speeds = [v]  → lane_time = 1000/v seconds

Full journey timing (using t_tk=takeoff, t_ld=land):
    departure at t=start
    reach wp_A at t=start + t_tk
    reach wp_B at t=start + t_tk + lane_time
    touchdown B at t=start + t_tk + lane_time + t_ld
"""

from __future__ import annotations

import pytest

from scrp_simple.models import (
    ApprovedPlan,
    FlightPath,
    NewPlanRequest,
    Pad,
    Point3D,
    Vertiport,
)
from scrp_simple.resolver import ConflictResolver, resolve_batch

# ──────────────────────────────────────────────────────────────────────────────
# Constants from the resolver (avoid magic numbers in assertions)
# ──────────────────────────────────────────────────────────────────────────────
MIN_SEP = ConflictResolver.MIN_SEPARATION_M  # 50 m


# ──────────────────────────────────────────────────────────────────────────────
# Shared geometry
# ──────────────────────────────────────────────────────────────────────────────

WP_ABOVE_A = Point3D(0, 0, 100)
WP_ABOVE_B = Point3D(1000, 0, 100)
LANE_LEN = WP_ABOVE_A.distance_to(WP_ABOVE_B)  # 1000 m


def make_vports(
    pads_a: list[tuple[str, float]] | None = None,
    pads_b: list[tuple[str, float]] | None = None,
) -> dict[str, Vertiport]:
    """Build a two-vertiport dict.  Default: 2 pads at each, 120 s duration."""
    if pads_a is None:
        pads_a = [("PA1", 120.0), ("PA2", 120.0)]
    if pads_b is None:
        pads_b = [("PB1", 120.0), ("PB2", 120.0)]
    return {
        "A": Vertiport("A", Point3D(0, 0, 0), [Pad(i, d) for i, d in pads_a]),
        "B": Vertiport("B", Point3D(1000, 0, 0), [Pad(i, d) for i, d in pads_b]),
    }


def approved_a_to_b(
    plan_id: str,
    start_time: float,
    speed: float = 20.0,
    t_tk: float = 10.0,
    t_ld: float = 10.0,
    assigned_pad: str = "PB1",
    priority: int = 5,
) -> ApprovedPlan:
    """Pre-built approved plan from A to B via [WP_ABOVE_A, WP_ABOVE_B]."""
    return ApprovedPlan(
        plan_id=plan_id,
        flight_path=FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
        drone_type="X",
        segment_speeds=[speed],
        start_time=start_time,
        t_takeoff=t_tk,
        t_land=t_ld,
        priority=priority,
        assigned_pad_id=assigned_pad,
    )


def request_a_to_b(
    desired: float = 0.0,
    max_wait: float = 600.0,
    speed: float = 20.0,
    t_tk: float = 10.0,
    t_ld: float = 10.0,
    priority: int = 5,
) -> NewPlanRequest:
    """Pre-built new-plan request from A to B via [WP_ABOVE_A, WP_ABOVE_B]."""
    return NewPlanRequest(
        flight_path=FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
        drone_type="X",
        segment_speeds=[speed],
        desired_start_time=desired,
        max_wait_time=max_wait,
        t_takeoff=t_tk,
        t_land=t_ld,
        priority=priority,
    )


# ──────────────────────────────────────────────────────────────────────────────
# C1 – Lane separation
# ──────────────────────────────────────────────────────────────────────────────

class TestLaneConflict:

    def test_no_conflict_different_routes(self):
        """Plans on entirely different routes – no lane conflict.

        Approved A→B uses [WP_ABOVE_A, WP_ABOVE_B].
        New A→C uses a completely different waypoint above C.
        Both depart from A, but the new plan departs *after* the approved
        takeoff window ends (desired=15 > t_takeoff=10) so there is also
        no departure-airspace conflict.
        """
        vp = {
            "A": Vertiport("A", Point3D(0, 0, 0), [Pad("P1", 120.0)]),
            "B": Vertiport("B", Point3D(1000, 0, 0), [Pad("P1", 120.0)]),
            "C": Vertiport("C", Point3D(0, 1000, 0), [Pad("P1", 120.0)]),
        }
        wp_above_c = Point3D(0, 1000, 100)
        approved = [
            ApprovedPlan(
                "p1", FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
                "X", [20.0], 0.0, 10.0, 10.0, assigned_pad_id="P1",
            )
        ]
        # desired=15 > approved t_takeoff=10, so departure airspace at A is clear
        req = NewPlanRequest(
            FlightPath("A", "C", [WP_ABOVE_A, wp_above_c]),
            "X", [20.0], 15.0, 600.0, 10.0, 10.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_no_conflict_well_separated(self):
        """Same lane but departure gap already exceeds the minimum headway."""
        vp = make_vports()
        speed = 20.0
        # headway = MIN_SEP/v_lead = 50/20 = 2.5 s → gap of 20 s is fine
        approved = [approved_a_to_b("p1", start_time=0.0, speed=speed)]
        req = request_a_to_b(desired=20.0, speed=speed)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_same_speed_requires_headway(self):
        """Same lane, same speed, depart simultaneously → headway delay needed.

        v_lead = v_follow = 20 m/s
        h_min_full = MIN_SEP / v_lead = 50/20 = 2.5 s
        """
        vp = make_vports()
        speed = 20.0
        expected_headway = MIN_SEP / speed  # 2.5 s
        approved = [approved_a_to_b("p1", start_time=0.0, speed=speed)]
        req = request_a_to_b(desired=0.0, speed=speed)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= expected_headway - 0.01

    def test_faster_follower_larger_headway(self):
        """Faster follower catches up – h_min_full correction term applies.

        v_lead=10, v_follow=20, seg_len=1000:
          base      = MIN_SEP / v_lead              = 50/10 = 5 s
          correction= (v_follow - v_lead)*L/(v_follow*v_lead)
                    = 10*1000/(20*10) = 5 s
          → h_min   = 5 + 5 = 10 s
        """
        v_lead, v_follow = 10.0, 20.0
        expected_h = MIN_SEP / v_lead + (v_follow - v_lead) * LANE_LEN / (v_follow * v_lead)
        vp = make_vports()
        approved = [approved_a_to_b("p1", start_time=0.0, speed=v_lead)]
        req = request_a_to_b(desired=0.0, speed=v_follow)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= expected_h - 0.01

    def test_slower_follower_only_base_headway(self):
        """Slower follower never closes the gap – only the base term of h_min_full applies.

        v_lead=20, v_follow=10: correction = max(0, v_follow-v_lead)*L/... = 0
        → h_min = MIN_SEP / v_lead = 50/20 = 2.5 s

        t_takeoff=0 on both plans so the departure-airspace constraint at A
        is trivially satisfied (zero-duration window), letting us isolate the
        lane-headway formula.
        """
        v_lead, v_follow = 20.0, 10.0
        expected_h = MIN_SEP / v_lead  # 2.5 s, no correction term
        vp = make_vports()
        # t_tk=0: takeoff window has zero duration → no departure airspace delay
        approved = [approved_a_to_b("p1", start_time=0.0, speed=v_lead, t_tk=0.0)]
        req = request_a_to_b(desired=0.0, speed=v_follow, t_tk=0.0)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= expected_h - 0.01
        assert result.delay < 5.0  # must not apply the faster-follower correction

    def test_new_drone_leads_safely(self):
        """New drone departs early enough to be well ahead – no delay."""
        vp = make_vports()
        speed = 20.0
        # Approved departs at t=0; new departs at t=-100 → new exits lane at -50 s
        # lead_headway = MIN_SEP/speed = 2.5 s → new_exit + 2.5 = -47.5 < 0 ✓
        approved = [approved_a_to_b("p1", start_time=0.0, speed=speed)]
        req = request_a_to_b(desired=-100.0, speed=speed)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_t_takeoff_offsets_lane_entry(self):
        """t_takeoff correctly shifts the lane-entry time.

        Both drones have t_takeoff=30 and depart at t=0.
        They enter the lane at t=30.  headway = 50/20 = 2.5 s.
        Required delay must push new drone's lane entry to t=30 + 2.5 = 32.5,
        i.e. departure at t=2.5.
        """
        vp = make_vports()
        speed = 20.0
        t_tk = 30.0
        expected_h = MIN_SEP / speed  # 2.5 s
        approved = [approved_a_to_b("p1", start_time=0.0, speed=speed, t_tk=t_tk)]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= expected_h - 0.01

    def test_direct_hop_no_lane_conflict(self):
        """Plans with no waypoints have no lane segments – never a lane conflict."""
        vp = make_vports()
        # No waypoints → direct hop, segment_speeds is empty
        approved = [
            ApprovedPlan(
                "p1", FlightPath("A", "B", []), "X", [], 0.0, 10.0, 10.0,
                assigned_pad_id="PB1",
            )
        ]
        req = NewPlanRequest(
            FlightPath("A", "B", []), "X", [], 0.0, 600.0, 10.0, 10.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        # Lane delay must be 0; only pad and airspace can add delay
        # (both are absent here given separate vertiports and enough pads)
        assert result.approved

    def test_max_wait_exceeded_rejected(self):
        """Required headway > max_wait_time → rejection."""
        v_lead, v_follow = 10.0, 20.0
        # h_min = 10 s; max_wait = 5 s → rejected
        vp = make_vports()
        approved = [approved_a_to_b("p1", start_time=0.0, speed=v_lead)]
        req = request_a_to_b(desired=0.0, speed=v_follow, max_wait=5.0)
        result = ConflictResolver(vp, approved).resolve(req)
        assert not result.approved
        assert result.actual_start_time is None

    def test_multiple_approved_cumulative_delay(self):
        """Three approved plans at t=0, 3, 6 s; new plan must follow the last.

        headway = 50/20 = 2.5 s → new must depart ≥ 6 + 2.5 = 8.5 s.
        """
        vp = make_vports()
        speed = 20.0
        headway = MIN_SEP / speed
        approved = [
            approved_a_to_b("p1", start_time=0.0, speed=speed, assigned_pad="PB1"),
            approved_a_to_b("p2", start_time=3.0, speed=speed, assigned_pad="PB2"),
            approved_a_to_b("p3", start_time=6.0, speed=speed, assigned_pad="PB1"),
        ]
        req = request_a_to_b(desired=0.0, speed=speed)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= 6.0 + headway - 0.01


# ──────────────────────────────────────────────────────────────────────────────
# C2 – Departure airspace serialisation
# ──────────────────────────────────────────────────────────────────────────────

class TestDepartureAirspace:

    def test_two_departures_serialised(self):
        """Two drones from A at t=0 – second must wait for first to clear.

        Approved: t_takeoff=30 → airspace A [0, 30]
        New: desired t=0, t_takeoff=20 → window [0, 20] overlaps
        → new must depart ≥ t=30 → delay ≥ 30 s
        """
        vp = make_vports()
        t_tk_approved = 30.0
        t_tk_new = 20.0
        approved = [approved_a_to_b("p1", start_time=0.0, t_tk=t_tk_approved)]
        req = request_a_to_b(desired=0.0, t_tk=t_tk_new)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        # New must start after approved's takeoff window ends
        assert result.delay >= t_tk_approved - 0.01

    def test_no_conflict_departure_after_window(self):
        """New departure starts after the approved takeoff window – no delay."""
        vp = make_vports()
        t_tk = 30.0
        approved = [approved_a_to_b("p1", start_time=0.0, t_tk=t_tk)]
        # New departs at t=40 > 30 → no overlap
        req = request_a_to_b(desired=40.0, t_tk=t_tk)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_three_departures_chain(self):
        """Approved drones at t=0 (t_tk=20) and t=20 (t_tk=20) – new must wait until t=40."""
        vp = make_vports()
        t_tk = 20.0
        approved = [
            approved_a_to_b("p1", start_time=0.0, t_tk=t_tk, assigned_pad="PB1"),
            approved_a_to_b("p2", start_time=20.0, t_tk=t_tk, assigned_pad="PB2"),
        ]
        req = request_a_to_b(desired=0.0, t_tk=t_tk)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= 40.0 - 0.01

    def test_different_departure_vertiports_no_conflict(self):
        """Approved plan departs from B; new plan departs from A – different airspace."""
        vp = make_vports()
        # Approved B→A (reversed route)
        approved = [
            ApprovedPlan(
                "p1",
                FlightPath("B", "A", [WP_ABOVE_B, WP_ABOVE_A]),
                "X", [20.0], 0.0, 30.0, 10.0,
                assigned_pad_id="PA1",
            )
        ]
        req = request_a_to_b(desired=0.0, t_tk=30.0)  # departs from A
        result = ConflictResolver(vp, approved).resolve(req)
        # No departure airspace conflict (different vertiports)
        assert result.approved
        # delay should only come from pad/arrival constraints, not departure
        assert result.delay < 30.0


# ──────────────────────────────────────────────────────────────────────────────
# C3 – Arrival airspace serialisation
# ──────────────────────────────────────────────────────────────────────────────

class TestArrivalAirspace:

    def test_two_landings_serialised(self):
        """Two drones landing at B at the same time – second must wait.

        Approved: start=0, t_tk=10, lane=50s (1000m/20), t_ld=20
          → t_arrive_wpN = 60, descent window [60, 80]
        New: same timing, desired t=0 → t_arrive_wpN = 60 → overlap
          → new must descend after t=80 → additional delay ≥ 20 s
        """
        vp = make_vports()
        speed = 20.0
        t_tk, t_ld = 10.0, 20.0
        lane_time = LANE_LEN / speed  # 50 s
        approved = [approved_a_to_b("p1", start_time=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        # New arrival must start at t=80; departure delay = 80-10-50-0 = 20 s
        assert result.delay >= t_ld - 0.01

    def test_no_conflict_staggered_arrivals(self):
        """Arrivals separated enough – no extra delay."""
        vp = make_vports()
        speed, t_tk, t_ld = 20.0, 10.0, 20.0
        lane_time = LANE_LEN / speed  # 50 s
        # Approved arrives at wpN at t=60, descent [60, 80]
        # New desired: depart at t=20 → t_arrive_wpN = 80 → descent [80, 100] – no overlap
        approved = [approved_a_to_b("p1", start_time=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)]
        req = request_a_to_b(desired=20.0, speed=speed, t_tk=t_tk, t_ld=t_ld)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay == pytest.approx(0.0, abs=0.01)


# ──────────────────────────────────────────────────────────────────────────────
# C2 ∩ C3 – Takeoff vs landing at the SAME vertiport
# ──────────────────────────────────────────────────────────────────────────────

class TestTakeoffVsLandingSameVertiport:
    """Shared airspace prevents simultaneous takeoff and landing at the same vertiport."""

    def test_landing_blocked_by_ongoing_takeoff(self):
        """An approved drone is taking off from B; new drone wants to land at B.

        Approved B→A: start=100, t_takeoff=30 → airspace at B: [100, 130]
        New A→B: desired t=0, t_tk=10, lane_time=100 (speed=10 over 1000m)
          → desired t_arrive_wpN = 0+10+100 = 110 → descent [110, 130] OVERLAPS
          → must wait until t_arrive_wpN ≥ 130
          → required delay = 130 - 10 - 100 - 0 = 20 s
        """
        vp = make_vports()
        # Approved plan departs FROM B (takeoff window at B = [100, 130])
        approved = [
            ApprovedPlan(
                "depart_B",
                FlightPath("B", "A", [WP_ABOVE_B, WP_ABOVE_A]),
                "X", [20.0], 100.0, 30.0, 10.0,
                assigned_pad_id="PA1",
            )
        ]
        # New plan lands AT B with speed=10 → lane_time=100 s
        req = NewPlanRequest(
            FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
            "X", [10.0], 0.0, 600.0, 10.0, 20.0,
        )
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        # t_arrive_wpN must be ≥ 130 → delay ≥ 130 - 10 - 100 = 20 s
        assert result.delay >= 20.0 - 0.1

    def test_takeoff_blocked_by_ongoing_landing(self):
        """An approved drone is landing at A; new drone wants to take off from A.

        Approved B→A: lane=50s (speed=20), t_tk=10, t_ld=20
          → t_arrive_wpN at A = start + t_tk + lane = 0+10+50 = 60
          → descent window at A = [60, 80]
        New A→B: desired t=0, t_takeoff=25
          → takeoff window [0, 25] – no overlap with [60, 80]
          → no delay needed from airspace
        """
        vp = make_vports()
        approved = [
            ApprovedPlan(
                "land_A",
                FlightPath("B", "A", [WP_ABOVE_B, WP_ABOVE_A]),
                "X", [20.0], 0.0, 10.0, 20.0,
                assigned_pad_id="PA1",
            )
        ]
        req = request_a_to_b(desired=0.0, t_tk=25.0)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        # takeoff window [0, 25] doesn't overlap landing window [60, 80]
        assert result.delay == pytest.approx(0.0, abs=0.01)

    def test_takeoff_blocked_by_overlapping_landing(self):
        """Approved landing at A overlaps with new takeoff from A.

        Approved B→A: t_arrive_wpN = 5, descent [5, 25]
        New A→B: desired t=0, t_takeoff=30 → window [0, 30] OVERLAPS [5, 25]
          → new takeoff must start ≥ 25 → delay ≥ 25 s
        """
        vp = make_vports()
        # Approved lands at A: start=0-45 so that t_arrive_wpN=5
        # lane_time = LANE_LEN/20 = 50s; t_arrive_wpN = start + 10 + 50 = start + 60
        # For t_arrive_wpN=5: start = 5-60 = -55
        approved = [
            ApprovedPlan(
                "land_A",
                FlightPath("B", "A", [WP_ABOVE_B, WP_ABOVE_A]),
                "X", [20.0], -55.0, 10.0, 20.0,
                assigned_pad_id="PA1",
            )
        ]
        # Descent window at A: [5, 25]
        req = request_a_to_b(desired=0.0, t_tk=30.0)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        # Takeoff window [0,30] overlaps [5,25]; new must start ≥ 25 → delay ≥ 25
        assert result.delay >= 25.0 - 0.1


# ──────────────────────────────────────────────────────────────────────────────
# C4 – Pad availability
# ──────────────────────────────────────────────────────────────────────────────

class TestPadAvailability:

    def test_single_pad_busy_forces_delay(self):
        """One pad at B, already occupied; new drone must wait.

        Approved: start=0, t_tk=10, lane=50s, t_ld=10 → touchdown=70
          → pad PB1 busy [70, 70+120] = [70, 190]
        New: desired=0 → touchdown=70 → pad busy → must touchdown ≥ 190
          → delay = 190 - 70 = 120 s
        """
        vp = make_vports(pads_b=[("PB1", 120.0)])  # 1 pad only
        speed, t_tk, t_ld = 20.0, 10.0, 10.0
        lane_time = LANE_LEN / speed  # 50 s
        touchdown_0 = 0.0 + t_tk + lane_time + t_ld  # = 70 s
        approved = [approved_a_to_b("p1", start_time=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.assigned_pad_id == "PB1"
        # New touchdown must be ≥ 190; delay = 190 - 70 = 120 s
        assert result.delay >= 120.0 - 0.1

    def test_two_pads_second_free(self):
        """Two pads; first busy, second free – no delay (use second pad)."""
        vp = make_vports(pads_b=[("PB1", 120.0), ("PB2", 120.0)])
        speed, t_tk, t_ld = 20.0, 10.0, 10.0
        approved = [
            approved_a_to_b("p1", start_time=0.0, speed=speed, t_tk=t_tk,
                             t_ld=t_ld, assigned_pad="PB1"),
        ]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.assigned_pad_id == "PB2"
        # PB2 is free → no pad delay needed (airspace might force a small delay)

    def test_choose_pad_with_shorter_wait(self):
        """Two pads with different occupation durations; resolver picks the one free sooner.

        PB1: occ=300 s, occupied [70, 370]
        PB2: occ=60 s,  occupied [70, 130]
        New arrives at t=70 → PB1 free at 370, PB2 free at 130.
        Optimal: PB2, delay = 130 - 70 = 60 s (back-calculated to departure).
        """
        vp = make_vports(pads_b=[("PB1", 300.0), ("PB2", 60.0)])
        speed, t_tk, t_ld = 20.0, 10.0, 10.0
        lane_time = LANE_LEN / speed  # 50 s
        approved = [
            approved_a_to_b("p1", 0.0, speed, t_tk, t_ld, assigned_pad="PB1"),
            approved_a_to_b("p2", 0.0, speed, t_tk, t_ld, assigned_pad="PB2"),
        ]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.assigned_pad_id == "PB2"
        # PB2 free at t=130; delay = (130 - t_ld - lane - t_tk) - desired
        #                          = (130 - 10 - 50 - 10) - 0 = 60 s
        assert result.delay >= 60.0 - 0.1
        assert result.delay < 300.0 - 0.1  # definitely did not choose PB1

    def test_pad_assignment_returned_in_result(self):
        """assigned_pad_id is populated in the approval result."""
        vp = make_vports(pads_b=[("PB1", 60.0)])
        result = ConflictResolver(vp, []).resolve(request_a_to_b())
        assert result.approved
        assert result.assigned_pad_id == "PB1"

    def test_rejection_no_pads_within_wait(self):
        """Pad chain forces delay > max_wait_time → rejection.

        Three sequential drones fill PB1 (only pad) for 3×120=360 s.
        Landings at t=70, 190, 310 → free at t=430.
        New plan: max_wait=100 → delay would be 360 s → rejected.
        """
        vp = make_vports(pads_b=[("PB1", 120.0)])
        speed, t_tk, t_ld = 20.0, 10.0, 10.0
        # Stagger departures so pads chain: land at 70, 190, 310
        # touchdown = start + 10 + 50 + 10 = start + 70
        approved = [
            approved_a_to_b("p1", 0.0,   speed, t_tk, t_ld, "PB1"),
            approved_a_to_b("p2", 120.0, speed, t_tk, t_ld, "PB1"),
            approved_a_to_b("p3", 240.0, speed, t_tk, t_ld, "PB1"),
        ]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld, max_wait=100.0)
        result = ConflictResolver(vp, approved).resolve(req)
        assert not result.approved


# ──────────────────────────────────────────────────────────────────────────────
# Combined constraints
# ──────────────────────────────────────────────────────────────────────────────

class TestCombined:

    def test_lane_delay_triggers_pad_delay(self):
        """Lane conflict adds small delay; resulting touchdown falls in pad window; pad adds more.

        Setup (1 pad PB1, occ=120s, speed=20):
          Approved: start=0, t_tk=10, lane=50, t_ld=10 → touchdown=70, pad busy [70,190]
          Lane headway = 50/20 = 2.5 s → new departs at 2.5, touchdown at 72.5 → still in [70,190]
          Pad then forces touchdown ≥ 190 → delay = 190-70 = 120 s
        """
        vp = make_vports(pads_b=[("PB1", 120.0)])
        speed, t_tk, t_ld = 20.0, 10.0, 10.0
        approved = [approved_a_to_b("p1", 0.0, speed, t_tk, t_ld, "PB1")]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.delay >= 120.0 - 0.1

    def test_actual_start_time_consistent_with_delay(self):
        """actual_start_time == desired_start_time + delay."""
        vp = make_vports(pads_b=[("PB1", 120.0)])
        approved = [approved_a_to_b("p1", 0.0)]
        req = request_a_to_b(desired=0.0)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.actual_start_time == pytest.approx(
            req.desired_start_time + result.delay, abs=0.01
        )

    def test_airspace_and_pad_combined(self):
        """Airspace conflict at arrival and pad conflict at same vertiport.

        Both constraints active: result must satisfy both.
        The resolved actual_start_time must be consistent with delay.
        """
        vp = make_vports(pads_b=[("PB1", 120.0)])
        speed, t_tk, t_ld = 20.0, 10.0, 20.0
        approved = [approved_a_to_b("p1", 0.0, speed, t_tk, t_ld, "PB1")]
        req = request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld)
        result = ConflictResolver(vp, approved).resolve(req)
        assert result.approved
        assert result.assigned_pad_id is not None
        assert result.actual_start_time == pytest.approx(
            req.desired_start_time + result.delay, abs=0.01
        )


# ──────────────────────────────────────────────────────────────────────────────
# Batch resolution with priority ordering
# ──────────────────────────────────────────────────────────────────────────────

class TestBatchResolution:

    def test_high_priority_gets_no_delay(self):
        """Priority-1 request resolved first → zero delay; priority-5 must wait."""
        vp = make_vports(pads_b=[("PB1", 120.0), ("PB2", 120.0)])
        speed = 20.0
        req_high = request_a_to_b(desired=0.0, speed=speed, priority=1)
        req_low = request_a_to_b(desired=0.0, speed=speed, priority=5)
        results = resolve_batch(vp, [], [req_low, req_high])
        assert results[1].approved and results[1].delay == pytest.approx(0.0, abs=0.01)
        assert results[0].approved and results[0].delay > 0.0

    def test_result_order_matches_input_order(self):
        """resolve_batch preserves input list ordering in the output."""
        vp = make_vports()
        reqs = [request_a_to_b(desired=float(i * 60), priority=p)
                for i, p in enumerate([5, 3, 1])]
        results = resolve_batch(vp, [], reqs)
        assert len(results) == 3
        for i, res in enumerate(results):
            assert res.approved, f"Request {i} was unexpectedly rejected"

    def test_approved_batch_plan_propagates_to_pool(self):
        """Approved batch plan's airspace and pad reservation blocks next request.

        Both requests target B at t=0; only 1 pad PB1, t_takeoff=20.
        Priority-1 approved with no delay; priority-5 must wait for:
          - departure airspace at A (t_takeoff=20)
          - arrival airspace at B
          - pad PB1
        """
        vp = make_vports(pads_b=[("PB1", 120.0)])
        req1 = request_a_to_b(desired=0.0, t_tk=20.0, t_ld=10.0, priority=1)
        req2 = request_a_to_b(desired=0.0, t_tk=20.0, t_ld=10.0, priority=5)
        results = resolve_batch(vp, [], [req2, req1])
        r1 = results[1]  # priority-1 at index 1
        r2 = results[0]  # priority-5 at index 0
        assert r1.approved and r1.delay == pytest.approx(0.0, abs=0.01)
        assert r2.approved and r2.delay > 0.0

    def test_lower_priority_rejected_pad_exhausted(self):
        """Three sequential requests to 1 pad; third (low priority) is rejected.

        p1 (priority=1) → no delay, lands at t=70, pad busy [70,190]
        p2 (priority=2) → waits for pad, lands at t=190, pad busy [190,310]
        p3 (priority=5, max_wait=100) → pad free at 310, delay=240 > 100 → rejected
        """
        vp = make_vports(pads_b=[("PB1", 120.0)])
        speed, t_tk, t_ld = 20.0, 10.0, 10.0
        reqs = [
            request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld,
                            max_wait=600.0, priority=1),
            request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld,
                            max_wait=600.0, priority=2),
            request_a_to_b(desired=0.0, speed=speed, t_tk=t_tk, t_ld=t_ld,
                            max_wait=100.0, priority=5),
        ]
        results = resolve_batch(vp, [], reqs)
        assert results[0].approved and results[0].delay == pytest.approx(0.0, abs=0.1)
        assert results[1].approved and results[1].delay >= 120.0 - 0.1
        assert not results[2].approved
