"""Tests for slot allocation (C3)."""
from __future__ import annotations

import math

import pytest

from scrp.slot import find_earliest_slot, slot_start_time
from scrp.models import Pad, VertiportState


def make_vertiport(n_slots: int = 200, slot_duration: float = 30.0) -> VertiportState:
    pad = Pad(id="pad1", compatible_types=["A", "B", "C"])
    slots = {("pad1", i): None for i in range(n_slots)}
    slot_status = {("pad1", i): "FREE" for i in range(n_slots)}
    return VertiportState(
        vertiport_id="vp1",
        pads=[pad],
        slot_duration=slot_duration,
        slots=slots,
        slot_status=slot_status,
    )


class TestSlotStartTime:
    def test_slot_zero(self):
        assert slot_start_time(0, 0.0, 30.0) == 0.0

    def test_slot_five(self):
        assert slot_start_time(5, 0.0, 30.0) == 150.0


class TestFindEarliestSlot:
    def test_no_conflict_returns_zero_delta(self):
        # t_land_desired=0.0 aligns exactly with slot 0 start → delta=0
        vp = make_vertiport()
        key, delta = find_earliest_slot(0.0, vp, "A", t_epoch=0.0)
        assert delta == pytest.approx(0.0, abs=1e-9)
        assert key is not None

    def test_land_mid_slot_delta_is_remaining(self):
        # t_land_desired=50.0: slot 0=[0-30), slot 1=[30-60), slot 2=[60+)
        # first slot with start >= 50 is slot 2 (start=60) → delta=10
        vp = make_vertiport()
        key, delta = find_earliest_slot(50.0, vp, "A", t_epoch=0.0)
        assert delta == pytest.approx(10.0, rel=1e-6)
        assert key is not None

    def test_all_committed_returns_inf(self):
        """When all known slots are COMMITTED, should return inf."""
        vp = make_vertiport(n_slots=200)
        for i in range(200):
            vp.slot_status[("pad1", i)] = "COMMITTED"
            vp.slots[("pad1", i)] = "somedroned"
        key, delta = find_earliest_slot(0.0, vp, "A", t_epoch=0.0)
        assert math.isinf(delta)
        assert key is None

    def test_first_slots_committed_delta_positive(self):
        """First 3 slots COMMITTED → must use slot 3 → delta > 0."""
        vp = make_vertiport(n_slots=10, slot_duration=30.0)
        for i in range(3):
            vp.slot_status[("pad1", i)] = "COMMITTED"
            vp.slots[("pad1", i)] = "d_existing"

        t_desired = 0.0
        key, delta = find_earliest_slot(t_desired, vp, "A", t_epoch=0.0)
        assert key == ("pad1", 3)
        assert delta == pytest.approx(3 * 30.0 - t_desired)

    def test_soft_slot_usable(self):
        """SOFT slots should be usable (treated like FREE for planning)."""
        vp = make_vertiport(n_slots=5, slot_duration=30.0)
        vp.slot_status[("pad1", 0)] = "SOFT"
        key, delta = find_earliest_slot(0.0, vp, "A", t_epoch=0.0)
        assert key is not None

    def test_incompatible_type_returns_inf(self):
        """No pad compatible with uav_type 'X'."""
        vp = make_vertiport()
        key, delta = find_earliest_slot(0.0, vp, "X", t_epoch=0.0)
        assert math.isinf(delta)
        assert key is None
