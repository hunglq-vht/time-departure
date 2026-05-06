"""Full-flow integration tests.

Two complementary test suites:

1. TestWaypointTimesAreTimestamps — confirms that every waypoint_time value in
   ApproveResult is an *absolute* timestamp (i.e. anchored to t_dep_star, not a
   travel-time delta starting at 0).

2. TestFullFlowWithOperatorLifecycle — end-to-end scenario that simulates a
   10-second window of flight-intention submissions, verifies that t_dep* is
   computed with proper waypoint timestamps, then walks through a mixed
   accept/reject operator response cycle and confirms that freed soft-reserved
   slots are reused by replacement flight intentions.

Scenario layout (T_BASE = 10_000.0):

    Lane A   A0(0,0) ── A1(200,0) ── A2(400,0)   2 × 200 m, v = 10 m/s → 40 s flight
             destination: vertiport vp_a

    Lane B   B0(0,500)─ B1(200,500)─ B2(400,500)  identical geometry, no shared waypoints
             destination: vertiport vp_b

    Each vertiport is independent; lane routing is by destination_vertiport_id,
    not by UAV type or pad name.

    Submission window (10 s):

        +0 s  drone_1 (lane_a)  t_des = T_BASE+120   → t_dep* = T_BASE+130  slot N
        +2 s  drone_2 (lane_a)  t_des = T_BASE+131   → t_dep* > drone_1     slot N+1
        +5 s  drone_3 (lane_a)  t_des = T_BASE+156   → t_dep* > drone_2     slot N+2
        +7 s  drone_4 (lane_b)  t_des = T_BASE+220   → t_dep* = T_BASE+220  slot M
        +9 s  drone_5 (lane_b)  t_des = T_BASE+221   → t_dep* > drone_4     slot M+1

    Operator responses:
        drone_1, drone_2, drone_3  →  COMMITTED
        drone_4, drone_5           →  rejected / released  (slots freed in vp_b)

    Replacement round:
        drone_6 (lane_b)  t_des = drone_4.t_dep*  → reuses freed slot M
        drone_7 (lane_b)  t_des = drone_5.t_dep*  → reuses freed slot M+1
"""
from __future__ import annotations

import pytest

from scrp.models import ApproveResult, RejectResult
from scrp.reservation import (
    commit_reservation,
    create_soft_reservation,
    release_reservation,
)
from scrp.scrp import resolve_conflict
from tests.conftest import make_fi, make_lane, make_system_state, make_vertiport, make_waypoint

# ---------------------------------------------------------------------------
# Shared constants
# ---------------------------------------------------------------------------

# Non-zero base so waypoint timestamps are clearly absolute (not deltas from 0).
T_BASE = 10_000.0
V = 10.0       # m/s on all segments
SEG_LEN = 200.0  # metres
FLIGHT_TIME = 2 * SEG_LEN / V  # 40 s (two segments at 10 m/s)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _build_lanes():
    """Two parallel lanes, each routed to its own vertiport."""
    wps_a = [
        make_waypoint("A0", 0.0, 0.0),
        make_waypoint("A1", SEG_LEN, 0.0),
        make_waypoint("A2", 2 * SEG_LEN, 0.0),
    ]
    wps_b = [
        make_waypoint("B0", 0.0, 500.0),
        make_waypoint("B1", SEG_LEN, 500.0),
        make_waypoint("B2", 2 * SEG_LEN, 500.0),
    ]
    lane_a = make_lane("lane_a", wps_a, destination_vertiport_id="vp_a")
    lane_b = make_lane("lane_b", wps_b, destination_vertiport_id="vp_b")
    return lane_a, lane_b


def _build_vertiports():
    """One vertiport per lane, each with 500 slots."""
    vp_a = make_vertiport(n_slots=500, slot_duration=30.0, vertiport_id="vp_a")
    vp_b = make_vertiport(n_slots=500, slot_duration=30.0, vertiport_id="vp_b")
    return vp_a, vp_b


# ---------------------------------------------------------------------------
# Suite 1 — Waypoint times are absolute timestamps
# ---------------------------------------------------------------------------

class TestWaypointTimesAreTimestamps:
    """Verify ApproveResult.waypoint_times carries absolute timestamps, not offsets."""

    def test_first_waypoint_time_equals_t_dep_star(self, config):
        """The first waypoint timestamp must equal t_dep_star (departure moment)."""
        lane_a, _ = _build_lanes()
        vp_a, _ = _build_vertiports()
        fi = make_fi(lane_a, v=V, t_des=T_BASE + 120.0)
        state = make_system_state(vertiports={"vp_a": vp_a}, t_now=T_BASE)

        result = resolve_conflict(fi, [], state, config)

        assert isinstance(result, ApproveResult)
        first_wp_id, first_wp_time = result.waypoint_times[0]
        assert first_wp_id == lane_a.waypoints[0].id
        assert first_wp_time == pytest.approx(result.t_dep_star), (
            f"First waypoint time {first_wp_time} must equal t_dep_star {result.t_dep_star}"
        )

    def test_waypoint_times_are_monotonically_increasing(self, config):
        """Each successive waypoint must have a strictly later timestamp."""
        lane_a, _ = _build_lanes()
        vp_a, _ = _build_vertiports()
        fi = make_fi(lane_a, v=V, t_des=T_BASE + 120.0)
        state = make_system_state(vertiports={"vp_a": vp_a}, t_now=T_BASE)

        result = resolve_conflict(fi, [], state, config)

        assert isinstance(result, ApproveResult)
        times = [t for _, t in result.waypoint_times]
        for i in range(1, len(times)):
            assert times[i] > times[i - 1], (
                f"waypoint_times[{i}]={times[i]} not > waypoint_times[{i-1}]={times[i-1]}"
            )

    def test_waypoint_times_match_cumulative_travel_time(self, config):
        """waypoint_times[i] == t_dep_star + sum of travel times for segments 0..i-1."""
        lane_a, _ = _build_lanes()
        vp_a, _ = _build_vertiports()
        fi = make_fi(lane_a, v=V, t_des=T_BASE + 120.0)
        state = make_system_state(vertiports={"vp_a": vp_a}, t_now=T_BASE)

        result = resolve_conflict(fi, [], state, config)

        assert isinstance(result, ApproveResult)
        t_dep = result.t_dep_star
        expected = t_dep
        for i, (wp_id, t_wp) in enumerate(result.waypoint_times):
            assert t_wp == pytest.approx(expected, abs=1e-9), (
                f"Waypoint {i} ({wp_id}): expected timestamp {expected}, got {t_wp}"
            )
            if i < len(lane_a.segments):
                expected += lane_a.segments[i].length / fi.v_waypoints[i]

    def test_waypoint_times_are_not_offset_from_zero(self, config):
        """All waypoint timestamps must exceed T_BASE — not small offset-from-zero values."""
        lane_a, _ = _build_lanes()
        vp_a, _ = _build_vertiports()
        fi = make_fi(lane_a, v=V, t_des=T_BASE + 120.0)
        state = make_system_state(vertiports={"vp_a": vp_a}, t_now=T_BASE)

        result = resolve_conflict(fi, [], state, config)

        assert isinstance(result, ApproveResult)
        for wp_id, t_wp in result.waypoint_times:
            assert t_wp > T_BASE, (
                f"waypoint_time {t_wp} for '{wp_id}' should be > T_BASE ({T_BASE}), "
                f"proving it is an absolute timestamp rather than a travel-time delta"
            )


# ---------------------------------------------------------------------------
# Suite 2 — Full flow: 10-second submission window → accept/reject → slot reuse
# ---------------------------------------------------------------------------

class TestFullFlowWithOperatorLifecycle:
    """End-to-end scenario: 5 FIs within 10 s, mixed operator responses, freed-slot reuse."""

    def test_full_10s_window_with_operator_responses(self, config):
        lane_a, lane_b = _build_lanes()
        vp_a, vp_b = _build_vertiports()

        # Two independent vertiports — lane A lands at vp_a, lane B lands at vp_b.
        state = make_system_state(
            vertiports={"vp_a": vp_a, "vp_b": vp_b},
            t_now=T_BASE,
        )

        # ── Phase 1: submit 5 FIs within a 10-second window ─────────────────
        #
        # Lane-A drones (drone_1/2/3): each t_des is just 1 s after the previous
        # drone's actual t_dep_star, forcing a C1 headway violation that pushes
        # successive departures progressively later.
        #
        # Lane-B drones (drone_4/5): same pattern but on an independent lane
        # routed to a completely separate vertiport (vp_b). No slot contention
        # with lane-A drones at all.
        #
        # Submission offsets within the 10-second window are noted in comments.
        fis = [
            make_fi(lane_a, v=V,   t_des=T_BASE + 120.0, drone_id="drone_1", operator_id="op1"),  # +0 s
            make_fi(lane_a, v=V+2, t_des=T_BASE + 131.0, drone_id="drone_2", operator_id="op2"),  # +2 s
            make_fi(lane_a, v=V+1, t_des=T_BASE + 156.0, drone_id="drone_3", operator_id="op3"),  # +5 s
            make_fi(lane_b, v=V,   t_des=T_BASE + 220.0, drone_id="drone_4", operator_id="op4"),  # +7 s
            make_fi(lane_b, v=V,   t_des=T_BASE + 221.0, drone_id="drone_5", operator_id="op5"),  # +9 s
        ]

        results: dict[str, ApproveResult] = {}

        for fi in fis:
            result = resolve_conflict(
                fi, state.approved_plans, state, config
            )
            assert isinstance(result, ApproveResult), (
                f"{fi.drone_id} should be approved; got {result}"
            )

            # ── waypoint timestamp assertions (inline) ───────────────────────
            # First waypoint timestamp == t_dep_star (moment of departure).
            first_wp_id, first_wp_time = result.waypoint_times[0]
            assert first_wp_time == pytest.approx(result.t_dep_star), (
                f"{fi.drone_id}: first waypoint_time != t_dep_star"
            )

            # All timestamps must be absolute (> T_BASE), not travel-time deltas.
            for wp_id, t_wp in result.waypoint_times:
                assert t_wp > T_BASE, (
                    f"{fi.drone_id}: waypoint_time {t_wp} for '{wp_id}' "
                    f"is not an absolute timestamp (expected > T_BASE={T_BASE})"
                )

            # Timestamps must increase strictly along the route.
            wp_times = [t for _, t in result.waypoint_times]
            assert wp_times == sorted(wp_times), (
                f"{fi.drone_id}: waypoint_times are not monotonically increasing"
            )

            # t_dep_star must not be earlier than t_des (no negative delay).
            assert result.t_dep_star >= fi.t_des - 1e-9, (
                f"{fi.drone_id}: t_dep_star {result.t_dep_star} < t_des {fi.t_des}"
            )

            # ApproveResult must name the correct destination vertiport.
            assert result.vertiport_id == fi.lane.destination_vertiport_id, (
                f"{fi.drone_id}: result.vertiport_id={result.vertiport_id!r} "
                f"!= lane.destination_vertiport_id={fi.lane.destination_vertiport_id!r}"
            )

            results[fi.drone_id] = result

            # Materialise the soft reservation so subsequent FIs see this plan.
            state = create_soft_reservation(
                result,
                fi.lane,
                fi.drone_id,
                fi.uav_type,
                fi.v_waypoints,
                state,
                config.T_RESPONSE_WINDOW_SEC,
            )

        # ── Phase 1 structural assertions ────────────────────────────────────

        # Five soft reservations were created.
        soft_plans = [p for p in state.approved_plans if p.status == "SOFT_RESERVED"]
        assert len(soft_plans) == 5

        # Lane-A drones must depart in strictly increasing order because each
        # t_des is chosen to be just 1 s after the previous drone's t_dep_star,
        # which triggers a C1 headway violation and pushes the new departure
        # further into the future.
        t_dep_a = [results[f"drone_{i}"].t_dep_star for i in range(1, 4)]
        assert t_dep_a[0] < t_dep_a[1] < t_dep_a[2], (
            f"Lane-A drones must depart in strictly increasing order: {t_dep_a}"
        )

        # Lane-B drones must also depart in order (same C1 logic applies).
        t_dep_b = [results[f"drone_{i}"].t_dep_star for i in range(4, 6)]
        assert t_dep_b[0] < t_dep_b[1], (
            f"Lane-B drones must depart in strictly increasing order: {t_dep_b}"
        )

        # drone_1 gets no C1 delay (no prior plans on lane_a); its only delay
        # comes from C3 slot alignment, so t_dep_star >= t_des.
        assert results["drone_1"].t_dep_star >= T_BASE + 120.0

        # drone_2 (v=V+2) is faster than drone_1 (v=V): C1 headway correction applies
        # and C3 slot alignment pushes t_dep* further out.
        assert results["drone_2"].delay_seconds > 0, (
            "drone_2 must be delayed (C1 headway from drone_1 + C3 slot alignment)"
        )
        assert results["drone_3"].delay_seconds > 0, (
            "drone_3 must be delayed (C3 slot alignment — slot held by drone_2)"
        )

        # Lane-A and lane-B land at different vertiports — verify routing.
        for i in range(1, 4):
            assert results[f"drone_{i}"].vertiport_id == "vp_a"
        for i in range(4, 6):
            assert results[f"drone_{i}"].vertiport_id == "vp_b"

        # Slots within each vertiport must be distinct.
        lane_a_slots = [
            (results[f"drone_{i}"].pad_id, results[f"drone_{i}"].slot_index)
            for i in range(1, 4)
        ]
        lane_b_slots = [
            (results[f"drone_{i}"].pad_id, results[f"drone_{i}"].slot_index)
            for i in range(4, 6)
        ]
        assert len(lane_a_slots) == len(set(lane_a_slots)), (
            f"Lane-A drones must each hold a unique slot in vp_a: {lane_a_slots}"
        )
        assert len(lane_b_slots) == len(set(lane_b_slots)), (
            f"Lane-B drones must each hold a unique slot in vp_b: {lane_b_slots}"
        )

        # ── Phase 2: operator responses ──────────────────────────────────────

        # Operators 1, 2, 3 accept → COMMITTED.
        for drone_id in ("drone_1", "drone_2", "drone_3"):
            state = commit_reservation(drone_id, state)

        # Record which slots drone_4 and drone_5 held before release.
        freed_slot_4 = (results["drone_4"].pad_id, results["drone_4"].slot_index)
        freed_slot_5 = (results["drone_5"].pad_id, results["drone_5"].slot_index)

        # Operators 4 and 5 do not respond (reject) → release their reservations.
        for drone_id in ("drone_4", "drone_5"):
            state = release_reservation(drone_id, state)

        # Three committed plans remain; no soft plans.
        committed = [p for p in state.approved_plans if p.status == "COMMITTED"]
        remaining_soft = [p for p in state.approved_plans if p.status == "SOFT_RESERVED"]
        assert len(committed) == 3, f"Expected 3 committed plans, got {len(committed)}"
        assert len(remaining_soft) == 0, f"Expected 0 soft plans, got {len(remaining_soft)}"

        # Freed slots must be back to FREE in vp_b.
        assert state.vertiports["vp_b"].slot_status[freed_slot_4] == "FREE", (
            f"Slot {freed_slot_4} in vp_b should be FREE after drone_4 release"
        )
        assert state.vertiports["vp_b"].slot_status[freed_slot_5] == "FREE", (
            f"Slot {freed_slot_5} in vp_b should be FREE after drone_5 release"
        )

        # ── Phase 3: replacement FIs reclaim the freed slots ─────────────────
        #
        # New operators submit intentions targeting the same departure times as
        # the rejected drones.  Because those slots are now FREE in vp_b, the
        # SCRP must assign them (and only them, since no earlier free slot
        # exists near those landing times).

        fi_6 = make_fi(
            lane_b, v=V,
            t_des=results["drone_4"].t_dep_star,
            drone_id="drone_6",
            operator_id="op6",
        )
        result_6 = resolve_conflict(
            fi_6, state.approved_plans, state, config
        )
        assert isinstance(result_6, ApproveResult), (
            f"drone_6 should be approved after slot freed; got {result_6}"
        )
        state = create_soft_reservation(
            result_6, fi_6.lane, fi_6.drone_id, fi_6.uav_type, fi_6.v_waypoints,
            state, config.T_RESPONSE_WINDOW_SEC,
        )

        fi_7 = make_fi(
            lane_b, v=V,
            t_des=results["drone_5"].t_dep_star,
            drone_id="drone_7",
            operator_id="op7",
        )
        result_7 = resolve_conflict(
            fi_7, state.approved_plans, state, config
        )
        assert isinstance(result_7, ApproveResult), (
            f"drone_7 should be approved after slot freed; got {result_7}"
        )
        state = create_soft_reservation(
            result_7, fi_7.lane, fi_7.drone_id, fi_7.uav_type, fi_7.v_waypoints,
            state, config.T_RESPONSE_WINDOW_SEC,
        )

        # At least one replacement drone must land in a previously freed slot,
        # proving that released SOFT reservations genuinely open capacity.
        replacement_slots = {
            (result_6.pad_id, result_6.slot_index),
            (result_7.pad_id, result_7.slot_index),
        }
        freed_slot_keys = {freed_slot_4, freed_slot_5}
        assert replacement_slots & freed_slot_keys, (
            f"Replacement FIs should reuse freed slots. "
            f"Replacement slots: {replacement_slots}, freed: {freed_slot_keys}"
        )

        # Replacement FIs also carry valid absolute waypoint timestamps and
        # are routed to vp_b (lane_b's destination).
        for fi, res in [(fi_6, result_6), (fi_7, result_7)]:
            assert res.vertiport_id == "vp_b"
            for wp_id, t_wp in res.waypoint_times:
                assert t_wp > T_BASE, (
                    f"{fi.drone_id}: replacement waypoint_time {t_wp} not an absolute timestamp"
                )

        # Final system state: 3 COMMITTED (lane_a / vp_a) + 2 SOFT (lane_b / vp_b).
        final_committed = [p for p in state.approved_plans if p.status == "COMMITTED"]
        final_soft = [p for p in state.approved_plans if p.status == "SOFT_RESERVED"]
        assert len(final_committed) == 3
        assert len(final_soft) == 2
        committed_ids = {p.drone_id for p in final_committed}
        soft_ids = {p.drone_id for p in final_soft}
        assert committed_ids == {"drone_1", "drone_2", "drone_3"}
        assert soft_ids == {"drone_6", "drone_7"}
