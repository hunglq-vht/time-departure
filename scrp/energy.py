"""Energy and SoC calculations."""
from __future__ import annotations

import math

from .models import FlightIntention, SCRPConfig
from .power_model import (
    cruise_power_w,
    hover_induced_velocity,
    hover_power_from_endurance,
    hover_power_from_momentum_theory,
    parasite_drag_area,
)


def _resolve_fi_hover_power(fi: FlightIntention) -> float:
    """Resolve effective hover power [W] from available FlightIntention fields.

    Resolution priority:
    1. fi.P_hover > 0  →  use directly.
    2. fi.flight_time_min > 0  →  derive from battery capacity and endurance.
    3. Propulsion geometry (mtow_kg, num_rotors, propeller_diameter_m)  →  momentum theory.
    """
    if fi.P_hover > 0.0:
        return fi.P_hover
    if fi.flight_time_min > 0.0:
        return hover_power_from_endurance(fi.C_bat, fi.flight_time_min)
    if fi.mtow_kg > 0.0 and fi.num_rotors > 0 and fi.propeller_diameter_m > 0.0:
        return hover_power_from_momentum_theory(fi.mtow_kg, fi.num_rotors, fi.propeller_diameter_m)
    raise ValueError(
        "hover power cannot be determined: provide P_hover, flight_time_min, "
        "or [mtow_kg + num_rotors + propeller_diameter_m]"
    )


def _has_physics_geometry(fi: FlightIntention) -> bool:
    return fi.num_rotors > 0 and fi.propeller_diameter_m > 0.0 and fi.mtow_kg > 0.0


def compute_E_cruise(fi: FlightIntention, config: SCRPConfig) -> float:
    """Estimate cruise energy in Wh.

    When propulsion geometry (num_rotors, propeller_diameter_m, mtow_kg) is
    available, uses the physics-based cruise power model from power_model.py.
    Otherwise falls back to the legacy P_hover * CRUISE_POWER_FACTOR proxy.
    """
    P_hover = _resolve_fi_hover_power(fi)

    if _has_physics_geometry(fi):
        v_i0 = hover_induced_velocity(fi.mtow_kg, fi.num_rotors, fi.propeller_diameter_m)
        Cd_A = (
            parasite_drag_area(fi.mtow_kg, fi.max_tilt_angle_deg, fi.max_speed_ms)
            if fi.max_tilt_angle_deg > 0.0 and fi.max_speed_ms > 0.0
            else 0.0
        )
        E = 0.0
        for seg, v in zip(fi.lane.segments, fi.v_waypoints):
            t_seg = seg.length / v
            P_seg = cruise_power_w(P_hover, v_i0, Cd_A, v)
            E += P_seg * t_seg / 3600.0
        return E

    # Legacy fallback: P_hover * CRUISE_POWER_FACTOR
    P_cruise = P_hover * config.CRUISE_POWER_FACTOR
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

    Hover wait is measured from when the drone reaches the destination
    airspace (after lane traversal, before the landing phase) to when its
    slot opens.  The landing-phase duration is stripped from t_land so the
    drone is not modelled as hovering while already on the ground.
    """
    from .geometry import t_land as compute_t_land

    P_hover = _resolve_fi_hover_power(fi)
    land_dur = fi.t_land_estimated if fi.t_land_estimated is not None else 0.0
    t_arrive_at_dest = compute_t_land(fi, t_dep_star) - land_dur
    t_hover_wait = max(0.0, t_slot_start - t_arrive_at_dest)
    E_hover = P_hover * (t_hover_wait / 3600.0)

    E_total = E_cruise + E_hover
    soc = fi.SoC_0 - E_total / fi.C_bat
    return soc
