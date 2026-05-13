"""Integration tests for multi-drone takeoff deconfliction."""
from __future__ import annotations

import pytest

from scrp.models import ApproveResult, BatchDeconflictResult, RejectResult, SCRPConfig
from scrp.multi_deconflict import resolve_multi_drone_deconfliction

from tests.conftest import (
    make_fi,
    make_lane,
    make_system_state,
    make_takeoff_pad,
    make_vertiport,
    make_waypoint,
)


# ---------------------------------------------------------------------------
# Scene builders
# ---------------------------------------------------------------------------

def _make_scene(n_takeoff_pads: int = 2, pad_spacing: float = 30.0):
    """Build a simple scene: one takeoff vertiport, one destination vertiport.

    Takeoff pads are placed along the X axis with ``pad_spacing`` metres apart.
    pad_spacing < MSD(50m) → adjacent pads conflict; pad_spacing >= 50 → no conflict.
    """
    # Two diverging lanes (different waypoint ids → no shared junctions, no headway)
    wps_a = [make_waypoint("A1", 0, 0), make_waypoint("A2", 1000, 100)]
    wps_b = [make_waypoint("B1", 0, 0), make_waypoint("B2", 1000, -100)]
    lane_a = make_lane("lane_a", wps_a, destination_vertiport_id="vp_dest")
    lane_b = make_lane("lane_b", wps_b, destination_vertiport_id="vp_dest")

    takeoff_pads = [
        make_takeoff_pad(f"tp{i+1}", x=i * pad_spacing, y=0)
        for i in range(n_takeoff_pads)
    ]
    vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=takeoff_pads)
    vp_dest = make_vertiport(vertiport_id="vp_dest")
    state = make_system_state(vertiports={"vp_origin": vp_origin, "vp_dest": vp_dest})
    config = SCRPConfig()
    return lane_a, lane_b, state, config


def _dep(result) -> float:
    assert isinstance(result, ApproveResult)
    return result.t_dep_star


# ---------------------------------------------------------------------------
# Basic functionality
# ---------------------------------------------------------------------------

class TestBatchBasics:
    def test_empty_batch_returns_empty_result(self):
        _, _, state, config = _make_scene()
        result = resolve_multi_drone_deconfliction([], state, config)
        assert result.total_drones == 0
        assert result.approved_count == 0
        assert result.rejected_count == 0
        assert result.entries == []

    def test_single_drone_is_approved_without_c0_delay(self):
        """A lone drone has no peer to conflict with — C0 contributes 0."""
        lane_a, _, state, config = _make_scene()
        fi = make_fi(
            lane_a, drone_id="d1", t_des=1000.0,
            origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0,
        )
        result = resolve_multi_drone_deconfliction([fi], state, config)
        assert result.total_drones == 1
        assert result.approved_count == 1
        entry = result.entries[0]
        assert entry.drone_id == "d1"
        assert isinstance(entry.result, ApproveResult)
        # Delay may be non-zero due to landing slot alignment (C3), but NOT from C0
        assert entry.result.delay_source != "C0_takeoff"

    def test_results_returned_in_input_order(self):
        lane_a, lane_b, state, config = _make_scene(pad_spacing=60.0)  # far pads, no C0
        fi1 = make_fi(lane_a, drone_id="d1", t_des=1000.0, priority=2,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        fi2 = make_fi(lane_b, drone_id="d2", t_des=1000.0, priority=1,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp2")
        result = resolve_multi_drone_deconfliction([fi1, fi2], state, config)
        assert result.entries[0].drone_id == "d1"
        assert result.entries[1].drone_id == "d2"


# ---------------------------------------------------------------------------
# Priority ordering
# ---------------------------------------------------------------------------

class TestPriorityOrdering:
    def test_higher_priority_processed_first_gets_no_c0_delay(self):
        """Priority-1 drone is processed first and incurs no C0 penalty."""
        lane_a, lane_b, state, config = _make_scene(pad_spacing=30.0)
        fi_high = make_fi(
            lane_a, drone_id="high", t_des=1000.0, priority=1,
            origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0,
        )
        fi_low = make_fi(
            lane_b, drone_id="low", t_des=1000.0, priority=2,
            origin_vertiport_id="vp_origin", takeoff_pad_id="tp2", t_takeoff=30.0,
        )
        result = resolve_multi_drone_deconfliction([fi_high, fi_low], state, config)

        high_res = result.entries[0].result
        low_res = result.entries[1].result
        assert isinstance(high_res, ApproveResult)
        assert isinstance(low_res, ApproveResult)

        # High priority has no C0 delay
        assert high_res.delay_source != "C0_takeoff"

        # Low priority departs after high clears (high.t_dep_star + takeoff ≤ low.t_dep_star)
        assert _dep(low_res) >= _dep(high_res) + 30.0 - 1e-6

    def test_equal_priority_earlier_t_des_processed_first(self):
        """When priorities are equal, earliest t_des wins the first slot."""
        lane_a, lane_b, state, config = _make_scene(pad_spacing=30.0)
        fi_early = make_fi(
            lane_a, drone_id="early", t_des=1000.0, priority=2,
            origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0,
        )
        fi_late = make_fi(
            lane_b, drone_id="late", t_des=1005.0, priority=2,
            origin_vertiport_id="vp_origin", takeoff_pad_id="tp2", t_takeoff=30.0,
        )
        # Feed in reversed order to test sorting
        result = resolve_multi_drone_deconfliction([fi_late, fi_early], state, config)

        early_res = next(e.result for e in result.entries if e.drone_id == "early")
        late_res = next(e.result for e in result.entries if e.drone_id == "late")
        assert isinstance(early_res, ApproveResult)
        assert isinstance(late_res, ApproveResult)

        # early was processed first → no C0 delay for it
        assert early_res.delay_source != "C0_takeoff"

        # late departs only after early has cleared its climb zone
        assert _dep(late_res) >= _dep(early_res) + 30.0 - 1e-6

    def test_processing_rank_reflects_priority(self):
        """Lower processing_rank means the drone was handled earlier (= higher priority)."""
        lane_a, lane_b, state, config = _make_scene(pad_spacing=60.0)
        fi1 = make_fi(lane_a, drone_id="d_prio1", t_des=1000.0, priority=1,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        fi2 = make_fi(lane_b, drone_id="d_prio2", t_des=1000.0, priority=2,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp2")
        result = resolve_multi_drone_deconfliction([fi2, fi1], state, config)  # reversed input

        rank_prio1 = next(e.processing_rank for e in result.entries if e.drone_id == "d_prio1")
        rank_prio2 = next(e.processing_rank for e in result.entries if e.drone_id == "d_prio2")
        assert rank_prio1 < rank_prio2  # prio 1 was handled first


# ---------------------------------------------------------------------------
# C0 constraint in batch: close vs far pads
# ---------------------------------------------------------------------------

class TestC0InBatch:
    def test_close_pads_cascade_delays(self):
        """With 3 close-pad drones, each waits for the previous to clear."""
        wps_a = [make_waypoint("A1", 0, 0), make_waypoint("A2", 1000, 0)]
        wps_b = [make_waypoint("B1", 0, 0), make_waypoint("B2", 1000, 50)]
        wps_c = [make_waypoint("C1", 0, 0), make_waypoint("C2", 1000, -50)]
        lane_a = make_lane("La", wps_a, destination_vertiport_id="vp_dest")
        lane_b = make_lane("Lb", wps_b, destination_vertiport_id="vp_dest")
        lane_c = make_lane("Lc", wps_c, destination_vertiport_id="vp_dest")

        tp1 = make_takeoff_pad("tp1", x=0)
        tp2 = make_takeoff_pad("tp2", x=20)   # 20m < MSD=50
        tp3 = make_takeoff_pad("tp3", x=40)   # 40m < MSD=50
        vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=[tp1, tp2, tp3])
        vp_dest = make_vertiport(vertiport_id="vp_dest")
        state = make_system_state(vertiports={"vp_origin": vp_origin, "vp_dest": vp_dest})
        config = SCRPConfig()

        takeoff_dur = 30.0
        fi1 = make_fi(lane_a, drone_id="d1", t_des=1000.0, priority=1,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=takeoff_dur)
        fi2 = make_fi(lane_b, drone_id="d2", t_des=1000.0, priority=2,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp2", t_takeoff=takeoff_dur)
        fi3 = make_fi(lane_c, drone_id="d3", t_des=1000.0, priority=3,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp3", t_takeoff=takeoff_dur)

        result = resolve_multi_drone_deconfliction([fi1, fi2, fi3], state, config)

        dep1 = _dep(result.entries[0].result)
        dep2 = _dep(result.entries[1].result)
        dep3 = _dep(result.entries[2].result)

        # Each subsequent drone departs only after the previous clears its climb zone
        assert dep2 >= dep1 + takeoff_dur - 1e-6
        assert dep3 >= dep2 + takeoff_dur - 1e-6
        # Staggering is strictly increasing
        assert dep2 > dep1
        assert dep3 > dep2

    def test_far_pads_no_c0_penalty(self):
        """Drones on pads >= MSD apart can depart at the same time."""
        lane_a, lane_b, state, config = _make_scene(pad_spacing=60.0)  # > MSD=50
        fi1 = make_fi(lane_a, drone_id="d1", t_des=1000.0, priority=1,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0)
        fi2 = make_fi(lane_b, drone_id="d2", t_des=1000.0, priority=2,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp2", t_takeoff=30.0)
        result = resolve_multi_drone_deconfliction([fi1, fi2], state, config)
        dep1 = _dep(result.entries[0].result)
        dep2 = _dep(result.entries[1].result)
        # With no C0, both drones get the same C3-only delay → identical t_dep_star
        assert dep1 == pytest.approx(dep2)

    def test_no_origin_vertiport_id_no_c0(self):
        """FIs without origin_vertiport_id set generate no C0 constraint."""
        lane_a, lane_b, state, config = _make_scene(pad_spacing=10.0)
        # No origin_vertiport_id → C0 always returns 0
        fi1 = make_fi(lane_a, drone_id="d1", t_des=1000.0, priority=1)
        fi2 = make_fi(lane_b, drone_id="d2", t_des=1000.0, priority=2)
        result = resolve_multi_drone_deconfliction([fi1, fi2], state, config)
        for entry in result.entries:
            assert isinstance(entry.result, ApproveResult)
            assert entry.result.delay_source != "C0_takeoff"


# ---------------------------------------------------------------------------
# Mixed destinations
# ---------------------------------------------------------------------------

class TestMixedDestinations:
    def test_drones_fly_to_different_destinations(self):
        """Drones at the same takeoff vertiport heading to different destinations."""
        wps_a = [make_waypoint("A1", 0, 0), make_waypoint("A2", 1000, 0)]
        wps_b = [make_waypoint("B1", 0, 0), make_waypoint("B2", 0, 1000)]
        lane_to_east = make_lane("to_east", wps_a, destination_vertiport_id="vp_east")
        lane_to_north = make_lane("to_north", wps_b, destination_vertiport_id="vp_north")

        tp1 = make_takeoff_pad("tp1", x=0)
        tp2 = make_takeoff_pad("tp2", x=20)  # 20m < MSD=50 → C0 applies
        vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=[tp1, tp2])
        vp_east = make_vertiport(vertiport_id="vp_east")
        vp_north = make_vertiport(vertiport_id="vp_north")
        state = make_system_state(
            vertiports={"vp_origin": vp_origin, "vp_east": vp_east, "vp_north": vp_north}
        )
        config = SCRPConfig()

        fi1 = make_fi(lane_to_east, drone_id="east_drone", t_des=1000.0, priority=1,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=30.0)
        fi2 = make_fi(lane_to_north, drone_id="north_drone", t_des=1000.0, priority=2,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp2", t_takeoff=30.0)

        result = resolve_multi_drone_deconfliction([fi1, fi2], state, config)
        assert result.approved_count == 2

        east_res = result.entries[0].result
        north_res = result.entries[1].result
        assert isinstance(east_res, ApproveResult)
        assert isinstance(north_res, ApproveResult)

        # east drone (priority 1) incurs no C0 penalty
        assert east_res.delay_source != "C0_takeoff"

        # north drone departs after east clears
        assert _dep(north_res) >= _dep(east_res) + 30.0 - 1e-6

        # Each drone lands at its own destination
        assert east_res.vertiport_id == "vp_east"
        assert north_res.vertiport_id == "vp_north"


# ---------------------------------------------------------------------------
# Interaction with C1/C2/C3 constraints
# ---------------------------------------------------------------------------

class TestC0InteractionWithOtherConstraints:
    def test_c0_delay_source_when_c0_dominates(self):
        """When C0 produces the largest delay, delay_source == 'C0_takeoff'."""
        wps_a = [make_waypoint("P1", 0, 0), make_waypoint("P2", 1000, 100)]
        wps_b = [make_waypoint("Q1", 0, 0), make_waypoint("Q2", 1000, -100)]
        lane_a = make_lane("La", wps_a, destination_vertiport_id="vp_dest")
        lane_b = make_lane("Lb", wps_b, destination_vertiport_id="vp_dest")

        # Large t_takeoff (120s) so C0 >> C3 (slot alignment ~10s)
        large_takeoff = 120.0

        tp1 = make_takeoff_pad("tp1", x=0)
        tp2 = make_takeoff_pad("tp2", x=10)  # 10m < MSD=50 → C0 applies
        vp_origin = make_vertiport(vertiport_id="vp_origin", takeoff_pads=[tp1, tp2])
        vp_dest = make_vertiport(vertiport_id="vp_dest")
        state = make_system_state(vertiports={"vp_origin": vp_origin, "vp_dest": vp_dest})
        config = SCRPConfig()

        fi1 = make_fi(lane_a, drone_id="d1", t_des=1000.0, priority=1,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp1", t_takeoff=large_takeoff)
        fi2 = make_fi(lane_b, drone_id="d2", t_des=1000.0, priority=2,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp2", t_takeoff=large_takeoff)

        result = resolve_multi_drone_deconfliction([fi1, fi2], state, config)
        d2_res = result.entries[1].result
        assert isinstance(d2_res, ApproveResult)
        # C0 dominates: d1 departs ~1000+C3_d1, clears at ~1000+C3_d1+120; C0 >> any C3
        assert d2_res.delay_source == "C0_takeoff"
        # d2's departure is at least 120s after d1's
        d1_dep = _dep(result.entries[0].result)
        assert _dep(d2_res) >= d1_dep + large_takeoff - 1e-6

    def test_approved_count_and_rejected_count(self):
        """Aggregate counts are consistent with individual results."""
        lane_a, lane_b, state, config = _make_scene()
        fi1 = make_fi(lane_a, drone_id="ok", t_des=1000.0, SoC_0=1.0,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp1")
        # Drone with tiny battery that will be rejected by SoC check
        fi2 = make_fi(lane_b, drone_id="bad_soc", t_des=1000.0, SoC_0=0.01,
                      C_bat=10.0, P_hover=5000.0,
                      origin_vertiport_id="vp_origin", takeoff_pad_id="tp2")
        result = resolve_multi_drone_deconfliction([fi1, fi2], state, config)
        assert result.total_drones == 2
        ok_res = next(e.result for e in result.entries if e.drone_id == "ok")
        bad_res = next(e.result for e in result.entries if e.drone_id == "bad_soc")
        assert isinstance(ok_res, ApproveResult)
        assert isinstance(bad_res, RejectResult)
        assert result.approved_count == 1
        assert result.rejected_count == 1
