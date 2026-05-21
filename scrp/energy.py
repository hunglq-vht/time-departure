"""Energy and State-of-Charge calculations for UAV flight planning.

Physical basis
--------------
Energy (Wh) = Power (W) × Time (h).  Battery capacity is specified in Wh,
so every energy term is kept in Wh throughout.

A multirotor in cruise is tilted forward, so it must generate both lift (to
oppose gravity) and thrust (to overcome aerodynamic drag).  Empirically this
costs more power than pure hover.  Because we usually lack full aerodynamic
data at planning time, cruise power is approximated as:

    P_cruise = P_hover × CRUISE_POWER_FACTOR

with CRUISE_POWER_FACTOR = 1.2 (i.e. cruise draws ~20 % more than hover).

State-of-Charge (SoC) is a dimensionless fraction in [0, 1] representing the
fraction of usable battery energy remaining.  Depleting E Wh from a battery
of capacity C_bat Wh reduces SoC by E / C_bat.
"""
from __future__ import annotations

from .models import FlightIntention, SCRPConfig


def compute_E_cruise(fi: FlightIntention, config: SCRPConfig) -> float:
    """Estimate cruise energy consumed while traversing all lane segments [Wh].

    Variables
    ---------
    P_hover             : float [W]   — Hover Power; power the drone consumes
                                        to maintain stationary flight (no forward
                                        motion).  Supplied by the operator as part
                                        of the drone specification.
    CRUISE_POWER_FACTOR : float [—]   — Cruise Power Factor; dimensionless
                                        multiplier that converts hover power to an
                                        approximate cruise power.  Default 1.2
                                        (cruise draws 20 % more than hover).
    P_cruise            : float [W]   — Cruise Power; estimated power during
                                        forward flight.
                                        P_cruise = P_hover × CRUISE_POWER_FACTOR
    L_k                 : float [m]   — Length of lane segment k; Euclidean
                                        distance between consecutive waypoints.
    v_k                 : float [m/s] — Cruise Velocity on segment k; submitted
                                        by the operator as v_waypoints[k].
    t_k                 : float [s]   — Travel Time on segment k.
                                        t_k = L_k / v_k
    E_cruise            : float [Wh]  — Cruise Energy; total electrical energy
                                        consumed while flying from take-off
                                        waypoint to the destination airspace entry.

    Formula (derived from E = P × t, converted to hours)
    -----------------------------------------------------
        E_cruise = Σ_k  P_cruise × (L_k / v_k) / 3600

    The division by 3600 converts seconds → hours so the result is in Wh.

    Usage
    -----
    E_cruise is later combined with E_hover_wait (energy spent hovering while
    waiting for the landing slot) to check whether SoC_remaining stays above
    the minimum threshold SOC_MIN (constraint C4).
    """
    # Cruise Power (W): hover power scaled by the cruise power factor.
    P_cruise = fi.P_hover * config.CRUISE_POWER_FACTOR
    E = 0.0
    for seg, v in zip(fi.lane.segments, fi.v_waypoints):
        # Travel time on this segment in hours: t_k [s] / 3600
        travel_time_h = (seg.length / v) / 3600.0
        E += P_cruise * travel_time_h
    return E


def compute_SoC_remaining(
    fi: FlightIntention,
    t_dep_star: float,
    t_slot_start: float,
    E_cruise: float,
) -> float:
    """Compute State-of-Charge remaining after the full flight mission [0..1].

    The mission has two energy-consuming phases after take-off:

    1. **Cruise phase** — flying through all lane segments (energy already
       computed externally as E_cruise).
    2. **Hover-wait phase** — hovering at the destination airspace entry while
       waiting for the assigned landing slot to open.  Only occurs when the
       drone arrives before its slot start time.

    Variables
    ---------
    t_dep_star          : float [s Unix] — Approved Departure Time; the
                                           adjusted take-off time returned by
                                           the conflict-resolution algorithm.
    t_slot_start        : float [s Unix] — Landing Slot Start Time; absolute
                                           Unix time at which the reserved
                                           landing pad slot opens.
    t_arrive_at_dest    : float [s Unix] — Arrival Time at Destination Airspace;
                                           time when the drone finishes lane
                                           traversal and enters the vertiport
                                           approach zone (excludes the landing-
                                           phase duration t_land_estimated).
    t_hover_wait        : float [s]      — Hover Wait Duration; time the drone
                                           spends hovering before its slot opens.
                                           t_hover_wait = max(0, t_slot_start
                                                               − t_arrive_at_dest)
    P_hover             : float [W]      — Hover Power (same as in E_cruise).
    E_cruise            : float [Wh]     — Cruise Energy from compute_E_cruise().
    E_hover_wait        : float [Wh]     — Hover-Wait Energy; energy consumed
                                           during the hover-wait phase.
                                           E_hover_wait = P_hover
                                                          × (t_hover_wait / 3600)
    E_total             : float [Wh]     — Total Mission Energy.
                                           E_total = E_cruise + E_hover_wait
    SoC_0               : float [0..1]   — Initial State of Charge; fraction of
                                           usable battery capacity at departure.
    C_bat               : float [Wh]     — Battery Capacity; total usable energy
                                           stored in the battery.
    SoC_remaining       : float [0..1]   — Remaining State of Charge after the
                                           full mission.

    Formula
    -------
        SoC_remaining = SoC_0 − E_total / C_bat
                      = SoC_0 − (E_cruise + E_hover_wait) / C_bat

    The result is compared against SOC_MIN (constraint C4).  If
    SoC_remaining < SOC_MIN the flight intention is rejected.

    Note: the hover-wait window begins when the drone reaches the destination
    airspace (t_arrive_at_dest), not when it touches down.  t_land_estimated
    (the landing-phase duration) is subtracted from t_land so the drone is not
    modelled as hovering while already executing the landing manoeuvre.
    """
    from .geometry import t_land as compute_t_land

    # t_arrive_at_dest: time drone enters destination airspace, before landing phase
    land_dur = fi.t_land_estimated if fi.t_land_estimated is not None else 0.0
    t_arrive_at_dest = compute_t_land(fi, t_dep_star) - land_dur

    # Hover-wait duration [s]: zero if drone arrives after slot has opened
    t_hover_wait = max(0.0, t_slot_start - t_arrive_at_dest)

    # E_hover_wait [Wh]: P_hover [W] × time [h]
    E_hover = fi.P_hover * (t_hover_wait / 3600.0)

    E_total = E_cruise + E_hover
    soc = fi.SoC_0 - E_total / fi.C_bat
    return soc
