"""Energy and SoC calculations."""
from __future__ import annotations

from .models import FlightIntention, SCRPConfig


def compute_E_cruise(fi: FlightIntention, config: SCRPConfig) -> float:
    """Estimate cruise energy in Wh.

    Uses P_hover * CRUISE_POWER_FACTOR as cruise power proxy.
    E = P_cruise * (L / v) / 3600  summed over all segments.
    """
    P_cruise = fi.P_hover * config.CRUISE_POWER_FACTOR
    E = 0.0
    for seg, v in zip(fi.lane.segments, fi.v_waypoints):
        travel_time_h = (seg.length / v) / 3600.0
        E += P_cruise * travel_time_h
    return E


def compute_SoC_remaining(
    fi: FlightIntention,
    t_dep_star: float,
    t_slot_start: float,
    E_cruise: float,
) -> float:
    """Compute SoC remaining after flight and hover wait.

    t_slot_start: absolute start time of the assigned landing slot.
    Returns SoC as a fraction [0..1].
    """
    from .geometry import t_land as compute_t_land

    t_land_val = compute_t_land(fi, t_dep_star)
    t_hover_wait = max(0.0, t_slot_start - t_land_val)
    E_hover = fi.P_hover * (t_hover_wait / 3600.0)

    E_total = E_cruise + E_hover
    soc = fi.SoC_0 - E_total / fi.C_bat
    return soc
