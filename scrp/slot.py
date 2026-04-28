"""Landing slot allocation (C3 constraint)."""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

from .models import SCRPConfig, VertiportState


def slot_start_time(slot_index: int, t_epoch: float, slot_duration: float) -> float:
    """Absolute start time of slot given a reference epoch and slot duration."""
    return t_epoch + slot_index * slot_duration


def slot_index_for_time(t: float, t_epoch: float, slot_duration: float) -> int:
    """Return the slot index that covers time t (floor division)."""
    return int((t - t_epoch) // slot_duration)


def find_earliest_slot(
    t_land_desired: float,
    vertiport_state: VertiportState,
    uav_type: str,
    t_epoch: float = 0.0,
    config: Optional[SCRPConfig] = None,
) -> Tuple[Optional[Tuple[str, int]], float]:
    """Find the earliest FREE slot at or after t_land_desired for uav_type.

    Returns (slot_key, delta_C3) where slot_key = (pad_id, slot_index).
    If no slot available, returns (None, inf).
    """
    slot_dur = vertiport_state.slot_duration

    # Find compatible pads
    compatible_pads = [p for p in vertiport_state.pads if uav_type in p.compatible_types]
    if not compatible_pads:
        return None, math.inf

    # Earliest slot index we could use
    min_slot = slot_index_for_time(t_land_desired, t_epoch, slot_dur)
    if slot_start_time(min_slot, t_epoch, slot_dur) < t_land_desired:
        min_slot += 1

    # Search within a reasonable window (up to 200 slots ~ ~100 min at 30s)
    max_search = min_slot + 200

    best: Optional[Tuple[str, int]] = None
    best_t: float = math.inf

    for pad in compatible_pads:
        # Collect known slot indices for this pad, sorted ascending
        pad_slots = sorted(
            si for (pid, si) in vertiport_state.slot_status if pid == pad.id
        )
        for si in pad_slots:
            if si < min_slot:
                continue
            if si >= max_search:
                break
            key = (pad.id, si)
            status = vertiport_state.slot_status.get(key, 'COMMITTED')
            if status in ('FREE', 'SOFT'):
                t_start = slot_start_time(si, t_epoch, slot_dur)
                if t_start < best_t:
                    best_t = t_start
                    best = key
                break  # found earliest for this pad

    if best is None:
        return None, math.inf

    delta_C3 = max(0.0, best_t - t_land_desired)
    return best, delta_C3


def compute_delta_C3(
    t_land_desired: float,
    vertiport_state: VertiportState,
    uav_type: str,
    t_epoch: float = 0.0,
) -> float:
    """Return C3 delay scalar (without slot key)."""
    _, delta = find_earliest_slot(t_land_desired, vertiport_state, uav_type, t_epoch)
    return delta
