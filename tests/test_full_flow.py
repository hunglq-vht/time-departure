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


# ---------------------------------------------------------------------------
# Suite 3 — Priority sorting, t_takeoff/t_land_estimated, and lane isolation
# ---------------------------------------------------------------------------

# Per-UAV-type kinematic profile.  Each entry is
#   (cruise_speed_m_s, takeoff_phase_s, landing_phase_s)
# Values are chosen so every type has a distinctly different total block time.
UAV_PROFILES: dict[str, tuple[float, float, float]] = {
    #  type   v (m/s)  t_takeoff (s)  t_land_est (s)
    "A": (10.0,  5.0,  8.0),   # lane time 40 s, block 53 s
    "B": (12.0,  7.0, 12.0),   # lane time ~33.3 s, block ~52.3 s
    "C": ( 8.0,  4.0,  6.0),   # lane time 50 s, block 60 s
}


def _fi_block_time(fi) -> float:
    """Total time from t_dep_star until wheels-down: t_takeoff + lane + t_land_est."""
    lane_time = sum(seg.length / v for seg, v in zip(fi.lane.segments, fi.v_waypoints))
    return (fi.t_takeoff or 0.0) + lane_time + (fi.t_land_estimated or 0.0)


class TestPriorityBatchSortingAndLaneIsolation:
    """Verify that:

    1. Flight intentions carry per-UAV-type t_takeoff, t_land_estimated, and
       velocity, and both timing fields are reflected in waypoint_times and
       t_land_assigned.
    2. A batch of mixed-priority FIs is sorted by priority descending
       (emergency first) before processing.
    3. Each lane has at least one emergency (priority 3) FI.
    4. A priority FI in lane A does NOT alter t_dep_star for lane B FIs —
       only same-lane approved plans and the destination vertiport's slot
       calendar influence a FI's departure time.

    UAV profiles (v m/s | t_takeoff s | t_land_est s | block time s):
        Type A : 10 |  5 |  8 | 53 s
        Type B : 12 |  7 | 12 | ~52.3 s
        Type C :  8 |  4 |  6 | 60 s

    Batch (unsorted):
        drone_a_emerg : lane_a, type B, priority=3, t_des = T_BASE+100  ← emergency
        drone_b_emerg : lane_b, type C, priority=3, t_des = T_BASE+200  ← emergency
        drone_a_norm  : lane_a, type A, priority=1, t_des = T_BASE+160
        drone_b_norm  : lane_b, type B, priority=1, t_des = T_BASE+280

    After priority sort (descending priority, then ascending t_des):
        1. drone_a_emerg  (priority 3, lane_a, type B)
        2. drone_b_emerg  (priority 3, lane_b, type C)
        3. drone_a_norm   (priority 1, lane_a, type A)
        4. drone_b_norm   (priority 1, lane_b, type B)
    """

    EMERGENCY = 3
    NORMAL = 1

    def _build_batch(self, lane_a, lane_b):
        """Return an *unsorted* list of four FIs, each with its UAV-type profile."""
        def _kw(uav_type):
            v, t_to, t_lo = UAV_PROFILES[uav_type]
            return dict(v=v, uav_type=uav_type, t_takeoff=t_to, t_land_estimated=t_lo)

        return [
            make_fi(lane_a, t_des=T_BASE + 100.0, drone_id="drone_a_emerg",
                    operator_id="op_ae", priority=self.EMERGENCY, **_kw("B")),
            make_fi(lane_b, t_des=T_BASE + 200.0, drone_id="drone_b_emerg",
                    operator_id="op_be", priority=self.EMERGENCY, **_kw("C")),
            make_fi(lane_a, t_des=T_BASE + 160.0, drone_id="drone_a_norm",
                    operator_id="op_an", priority=self.NORMAL, **_kw("A")),
            make_fi(lane_b, t_des=T_BASE + 280.0, drone_id="drone_b_norm",
                    operator_id="op_bn", priority=self.NORMAL, **_kw("B")),
        ]

    def test_priority_sorting_and_lane_isolation(self, config):
        lane_a, lane_b = _build_lanes()
        vp_a, vp_b = _build_vertiports()

        batch = self._build_batch(lane_a, lane_b)

        # ── Priority sort: descending priority, then ascending t_des ─────────
        sorted_batch = sorted(batch, key=lambda fi: (-fi.priority, fi.t_des))

        # Emergency FIs must lead the sorted batch.
        assert sorted_batch[0].priority == self.EMERGENCY, (
            "First FI in sorted batch must be emergency"
        )
        assert sorted_batch[1].priority == self.EMERGENCY, (
            "Second FI in sorted batch must be emergency"
        )
        assert sorted_batch[2].priority == self.NORMAL
        assert sorted_batch[3].priority == self.NORMAL

        # Each lane must have at least one emergency FI.
        emergency_lane_ids = {fi.lane.id for fi in batch if fi.priority == self.EMERGENCY}
        assert "lane_a" in emergency_lane_ids, "Lane A must have at least one emergency FI"
        assert "lane_b" in emergency_lane_ids, "Lane B must have at least one emergency FI"

        # Each UAV type must carry its own distinct kinematic profile.
        for fi in batch:
            v_exp, to_exp, lo_exp = UAV_PROFILES[fi.uav_type]
            assert fi.v_waypoints[0] == pytest.approx(v_exp), (
                f"{fi.drone_id}: v_waypoints should be {v_exp} for type {fi.uav_type}"
            )
            assert fi.t_takeoff == pytest.approx(to_exp), (
                f"{fi.drone_id}: t_takeoff should be {to_exp} for type {fi.uav_type}"
            )
            assert fi.t_land_estimated == pytest.approx(lo_exp), (
                f"{fi.drone_id}: t_land_estimated should be {lo_exp} for type {fi.uav_type}"
            )

        # ── Process sorted batch ─────────────────────────────────────────────
        state = make_system_state(
            vertiports={"vp_a": vp_a, "vp_b": vp_b},
            t_now=T_BASE,
        )
        results: dict[str, ApproveResult] = {}
        for fi in sorted_batch:
            result = resolve_conflict(fi, state.approved_plans, state, config)
            assert isinstance(result, ApproveResult), (
                f"{fi.drone_id} (type={fi.uav_type}, priority={fi.priority}) "
                f"must be approved; got {result}"
            )
            results[fi.drone_id] = result
            state = create_soft_reservation(
                result, fi.lane, fi.drone_id, fi.uav_type, fi.v_waypoints,
                state, config.T_RESPONSE_WINDOW_SEC,
            )

        # ── t_takeoff reflected in waypoint_times (per-FI value) ─────────────
        # First waypoint arrival = t_dep_star + fi.t_takeoff (varies by UAV type).
        for fi in batch:
            result = results[fi.drone_id]
            _, first_wp_time = result.waypoint_times[0]
            assert first_wp_time == pytest.approx(result.t_dep_star + fi.t_takeoff), (
                f"{fi.drone_id} (type {fi.uav_type}): waypoint_times[0] should be "
                f"t_dep_star + t_takeoff ({result.t_dep_star} + {fi.t_takeoff}), "
                f"got {first_wp_time}"
            )

        # ── t_land_estimated reflected in t_land_assigned (per-FI value) ─────
        # t_land_assigned = t_dep_star + block_time (unique per UAV type).
        for fi in batch:
            result = results[fi.drone_id]
            expected_land = result.t_dep_star + _fi_block_time(fi)
            assert result.t_land_assigned == pytest.approx(expected_land), (
                f"{fi.drone_id} (type {fi.uav_type}): t_land_assigned "
                f"{result.t_land_assigned} != t_dep_star + block_time ({expected_land})"
            )

        # ── Block times must differ across UAV types ──────────────────────────
        block_times = {uav: _fi_block_time(next(fi for fi in batch if fi.uav_type == uav))
                       for uav in {"A", "B", "C"}}
        assert block_times["A"] != pytest.approx(block_times["B"]), (
            "Type A and B must have different block times"
        )
        assert block_times["A"] != pytest.approx(block_times["C"]), (
            "Type A and C must have different block times"
        )
        assert block_times["B"] != pytest.approx(block_times["C"]), (
            "Type B and C must have different block times"
        )

        # ── t_dep_star must not be earlier than t_des ────────────────────────
        for fi in batch:
            assert results[fi.drone_id].t_dep_star >= fi.t_des - 1e-9, (
                f"{fi.drone_id}: t_dep_star < t_des"
            )

        # ── Lane isolation: lane-A FIs must not affect lane-B t_dep_star ─────
        #
        # Re-run lane-B FIs in a pristine state (no lane-A plans present).
        # Because lanes share no waypoints and route to separate vertiports,
        # the t_dep_star values must be identical to the full-batch results
        # regardless of what UAV types were used in lane A.
        vp_a_iso = make_vertiport(n_slots=500, slot_duration=30.0, vertiport_id="vp_a")
        vp_b_iso = make_vertiport(n_slots=500, slot_duration=30.0, vertiport_id="vp_b")
        state_iso = make_system_state(
            vertiports={"vp_a": vp_a_iso, "vp_b": vp_b_iso},
            t_now=T_BASE,
        )

        fi_b_emerg = next(fi for fi in batch if fi.drone_id == "drone_b_emerg")
        fi_b_norm = next(fi for fi in batch if fi.drone_id == "drone_b_norm")

        # Process lane-B emergency then lane-B normal in isolation.
        res_b_emerg_iso = resolve_conflict(
            fi_b_emerg, state_iso.approved_plans, state_iso, config
        )
        assert isinstance(res_b_emerg_iso, ApproveResult)
        state_iso = create_soft_reservation(
            res_b_emerg_iso, fi_b_emerg.lane, fi_b_emerg.drone_id,
            fi_b_emerg.uav_type, fi_b_emerg.v_waypoints,
            state_iso, config.T_RESPONSE_WINDOW_SEC,
        )

        res_b_norm_iso = resolve_conflict(
            fi_b_norm, state_iso.approved_plans, state_iso, config
        )
        assert isinstance(res_b_norm_iso, ApproveResult)

        # Lane-A FIs (different UAV types and priorities) must not shift
        # lane-B t_dep_star, because there are no shared waypoints or slots.
        assert results["drone_b_emerg"].t_dep_star == pytest.approx(
            res_b_emerg_iso.t_dep_star
        ), (
            f"Lane-A FIs altered drone_b_emerg (type C) t_dep_star: "
            f"full={results['drone_b_emerg'].t_dep_star}, "
            f"isolated={res_b_emerg_iso.t_dep_star}"
        )
        assert results["drone_b_norm"].t_dep_star == pytest.approx(
            res_b_norm_iso.t_dep_star
        ), (
            f"Lane-A FIs altered drone_b_norm (type B) t_dep_star: "
            f"full={results['drone_b_norm'].t_dep_star}, "
            f"isolated={res_b_norm_iso.t_dep_star}"
        )
