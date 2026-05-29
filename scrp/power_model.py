"""Physics-based power consumption model for multirotor drones.

All quantities needed to compute cruise power are derived directly from
standard datasheet values — no abstract `cruise_power_factor` is required.

Theory
------
**Hover induced velocity** (Rankine-Froude momentum theory):

    v_i0 = sqrt(T / (2 · ρ · A_disk))

where T = MTOW · g and A_disk = N_rotors · π · (D_prop/2)².

**Power decomposition**

The measured hover power already includes both induced and profile drag:

    P_hover = P_induced_hover + P_profile
            = FM · P_hover  +  (1 − FM) · P_hover

Using a figure of merit FM = 0.75 (typical for well-designed multirotors) to
separate the two components.

**Induced power in forward flight** (Glauert momentum-theory approximation):

    P_induced_cruise(v) = FM · P_hover / sqrt(1 + (v / v_i0)²)

This falls from FM · P_hover at hover toward zero at high speed.

**Parasite drag power**

At maximum horizontal speed the horizontal thrust component is the only force
counteracting aerodynamic drag, so at max tilt angle θ:

    F_drag_max = MTOW · g · tan(θ)
    C_d·A      = F_drag_max / (0.5 · ρ · v_max²)
    P_parasite = 0.5 · ρ · (C_d·A) · v³

**Effective airspeed with wind**

Wind increases the airspeed seen by the rotor system.  For an arbitrary wind
direction relative to heading, the average drag effect is modelled using the
root-mean-square airspeed:

    v_eff = sqrt(v_cruise² + v_wind²)

**Total cruise power**

    P_total(v_eff) = P_induced_cruise(v_eff) + P_profile + P_parasite(v_eff)

At v_eff = 0: P_total = FM · P_hover + (1−FM) · P_hover = P_hover  ✓

**Vertical phase energy**

Takeoff (climb) and landing (descent) are short vertical transitions.  They
are modelled using fixed multiples of hover power applied over the phase
duration derived from ascent/descent speeds:

    t_takeoff = takeoff_height_m / max_ascent_speed_ms
    t_landing = landing_height_m / max_descent_speed_ms
    E_takeoff  = CLIMB_POWER_FACTOR   · P_hover · t_takeoff / 3600
    E_landing  = DESCENT_POWER_FACTOR · P_hover · t_landing / 3600
"""
from __future__ import annotations

import math
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .models import DroneProfile, FlightRoute, RouteFlightEstimate

# Physical constants
AIR_DENSITY_SL = 1.225   # kg/m³, ISA sea-level
GRAVITY = 9.81            # m/s²

# Aerodynamic model parameters
FIGURE_OF_MERIT = 0.75    # FM = P_induced_ideal / P_hover; 0.70–0.80 for typical multirotors

# Power factors for vertical flight phases (fractions of hover power)
CLIMB_POWER_FACTOR = 1.10    # slightly above hover to gain altitude
DESCENT_POWER_FACTOR = 0.50  # motor braking: roughly half hover power


# ---------------------------------------------------------------------------
# Aerodynamic parameter derivations
# ---------------------------------------------------------------------------

def hover_induced_velocity(mtow_kg: float, num_rotors: int, prop_diameter_m: float) -> float:
    """Ideal induced velocity at hover [m/s] from momentum theory.

    v_i0 = sqrt(MTOW · g / (2 · ρ · A_disk))
    """
    r = prop_diameter_m / 2.0
    A_disk = num_rotors * math.pi * r * r
    return math.sqrt(mtow_kg * GRAVITY / (2.0 * AIR_DENSITY_SL * A_disk))


def parasite_drag_area(mtow_kg: float, max_tilt_angle_deg: float, max_speed_ms: float) -> float:
    """Estimate equivalent parasite drag area C_d · A_frontal [m²].

    At max speed the horizontal thrust (MTOW · g · tan(tilt)) equals drag:
        C_d · A = F_drag / (0.5 · ρ · v_max²)
    """
    F_drag = mtow_kg * GRAVITY * math.tan(math.radians(max_tilt_angle_deg))
    return F_drag / (0.5 * AIR_DENSITY_SL * max_speed_ms ** 2)


# ---------------------------------------------------------------------------
# Power model
# ---------------------------------------------------------------------------

def cruise_power_w(
    hover_power_w: float,
    v_i0: float,
    Cd_A: float,
    airspeed_ms: float,
) -> float:
    """Total cruise power [W] at the given effective airspeed [m/s].

    Combines three terms:
    - Induced power (decreases with speed, Glauert approximation)
    - Profile drag power (constant fraction of hover power)
    - Parasite drag power (increases with v³)
    """
    P_induced_cruise = (FIGURE_OF_MERIT * hover_power_w
                        / math.sqrt(1.0 + (airspeed_ms / v_i0) ** 2))
    P_profile = (1.0 - FIGURE_OF_MERIT) * hover_power_w
    P_parasite = 0.5 * AIR_DENSITY_SL * Cd_A * airspeed_ms ** 3
    return P_induced_cruise + P_profile + P_parasite


# ---------------------------------------------------------------------------
# Full-route estimate
# ---------------------------------------------------------------------------

def estimate_flight(drone: "DroneProfile", route: "FlightRoute") -> "RouteFlightEstimate":
    """Compute cruise time, total time, and energy for *drone* on *route*.

    Per-segment cruise speed is clamped to [v_min, v_max].
    Wind adds to effective airspeed via the RMS combination:
        v_eff = sqrt(v_cruise² + v_wind²)

    Takeoff and landing altitudes are taken from the drone profile's
    takeoff_height_m / landing_height_m mission parameters.
    """
    from .models import RouteFlightEstimate

    v_plan = drone.cruise_speed_ms if drone.cruise_speed_ms > 0.0 else drone.max_speed_ms * 0.75

    # Pre-compute aerodynamic constants once (they depend only on drone specs)
    v_i0 = hover_induced_velocity(drone.mtow_kg, drone.num_rotors, drone.propeller_diameter_m)
    Cd_A = parasite_drag_area(drone.mtow_kg, drone.max_tilt_angle_deg, drone.max_speed_ms)

    cruise_time_s = 0.0
    E_cruise_wh = 0.0

    for seg in route.segments:
        v_seg = _clamp(v_plan, seg.v_min, seg.v_max)
        v_eff = math.sqrt(v_seg ** 2 + route.average_wind_speed ** 2)
        t_seg = seg.length / v_seg
        P_seg = cruise_power_w(drone.hover_power_w, v_i0, Cd_A, v_eff)
        E_cruise_wh += P_seg * t_seg / 3600.0
        cruise_time_s += t_seg

    # Vertical transition phases
    t_takeoff = drone.takeoff_height_m / drone.max_ascent_speed_ms
    E_takeoff_wh = CLIMB_POWER_FACTOR * drone.hover_power_w * t_takeoff / 3600.0

    t_landing = drone.landing_height_m / drone.max_descent_speed_ms
    E_landing_wh = DESCENT_POWER_FACTOR * drone.hover_power_w * t_landing / 3600.0

    energy_consumed_wh = E_takeoff_wh + E_cruise_wh + E_landing_wh
    total_time_s = t_takeoff + cruise_time_s + t_landing
    SoC_remaining = drone.soc_0 - energy_consumed_wh / drone.battery_energy_wh

    return RouteFlightEstimate(
        cruise_time_s=cruise_time_s,
        total_time_s=total_time_s,
        energy_consumed_wh=energy_consumed_wh,
        SoC_remaining=SoC_remaining,
    )


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
