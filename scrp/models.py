from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional, Tuple


@dataclass
class Waypoint:
    id: str
    position: Tuple[float, float, float]  # (x, y, z) in metres


@dataclass
class Segment:
    p_start: Waypoint
    p_end: Waypoint
    length: float  # metres
    v_min: float   # m/s
    v_max: float   # m/s


@dataclass
class Lane:
    id: str
    waypoints: List[Waypoint]
    segments: List[Segment] = field(default_factory=list)
    destination_vertiport_id: str = ""  # vertiport where this lane terminates

    def __post_init__(self) -> None:
        if not self.segments and len(self.waypoints) >= 2:
            import math
            segs: List[Segment] = []
            for i in range(len(self.waypoints) - 1):
                a, b = self.waypoints[i], self.waypoints[i + 1]
                dx = b.position[0] - a.position[0]
                dy = b.position[1] - a.position[1]
                dz = b.position[2] - a.position[2]
                length = math.sqrt(dx * dx + dy * dy + dz * dz)
                segs.append(Segment(p_start=a, p_end=b, length=length, v_min=5.0, v_max=20.0))
            self.segments = segs


@dataclass
class OperatorFlightRequest:
    """What an operator submits: identity, flight path, schedule, and drone spec.

    The operator supplies all drone hardware properties themselves.
    The only information NOT present here is the MSD matrix, which the
    algorithm maintains independently for all known UAV types.
    """
    operator_id: str
    drone_id: str
    # Flight path
    lane: Lane
    v_waypoints: List[float]          # velocity entering each segment
    destination_vertiport_id: str
    # Schedule
    t_des: float                      # desired departure time [s Unix]
    priority: int                     # 1 | 2 | 3  (1 = highest)
    # Drone specification submitted by the operator
    uav_type: str                     # 'A' | 'B' | 'C'
    SoC_0: float                      # state of charge [0..1]
    C_bat: float                      # battery capacity [Wh]
    # Hover power and propulsion geometry — at least one resolution path must be provided.
    # Resolution priority: P_hover > 0 → flight_time_min > 0 → (mtow_kg + num_rotors + propeller_diameter_m)
    P_hover: float = 0.0              # hover power [W]; 0.0 = derive automatically
    num_rotors: int = 0               # number of rotors (for momentum-theory derivation)
    propeller_diameter_m: float = 0.0 # propeller diameter [m]
    mtow_kg: float = 0.0              # max takeoff weight [kg]
    max_speed_ms: float = 0.0         # max horizontal speed [m/s] (for parasite drag)
    max_tilt_angle_deg: float = 0.0   # max tilt angle [°] (for parasite drag)
    flight_time_min: float = 0.0      # manufacturer endurance [min] (for endurance derivation)
    t_takeoff: Optional[float] = None        # takeoff phase duration [s]
    t_land_estimated: Optional[float] = None # landing phase duration [s]


@dataclass
class FlightIntention:
    drone_id: str
    uav_type: str           # 'A' | 'B' | 'C'
    lane: Lane
    v_waypoints: List[float]  # velocity on each segment; len == len(lane.waypoints)
    t_des: float              # desired departure time [s Unix]
    SoC_0: float              # state of charge [0..1]
    C_bat: float              # battery capacity [Wh]
    priority: int             # 1 | 2 | 3
    operator_id: str
    # Hover power and propulsion geometry — at least one resolution path must be provided.
    # Resolution priority: P_hover > 0 → flight_time_min > 0 → (mtow_kg + num_rotors + propeller_diameter_m)
    P_hover: float = 0.0              # hover power [W]; 0.0 = derive automatically
    num_rotors: int = 0               # number of rotors (for momentum-theory derivation)
    propeller_diameter_m: float = 0.0 # propeller diameter [m]
    mtow_kg: float = 0.0              # max takeoff weight [kg]
    max_speed_ms: float = 0.0         # max horizontal speed [m/s] (for parasite drag)
    max_tilt_angle_deg: float = 0.0   # max tilt angle [°] (for parasite drag)
    flight_time_min: float = 0.0      # manufacturer endurance [min] (for endurance derivation)
    t_takeoff: Optional[float] = None       # takeoff phase duration [s]; added before entering the lane
    t_land_estimated: Optional[float] = None  # landing phase duration [s]; added after exiting the lane


@dataclass
class ApprovedPlan:
    drone_id: str
    uav_type: str
    lane: Lane
    v_waypoints: List[float]                       # velocity entering each segment; len == len(lane.waypoints)
    t_land: float
    pad_id: str
    slot_index: int
    status: str   # 'SOFT_RESERVED' | 'COMMITTED'
    t_dep: Optional[float] = None                  # approved departure time; used to compute waypoint_times lazily
    waypoint_times: List[Tuple[Waypoint, float]] = field(default_factory=list)  # [(waypoint, abs_time), ...]
    algorithm: str = 'SCRP'                        # algorithm that produced this plan
    expires_at: Optional[float] = None
    vertiport_id: str = ""                         # destination vertiport for slot management


@dataclass
class Pad:
    id: str
    compatible_types: List[str]


@dataclass
class VertiportState:
    vertiport_id: str
    pads: List[Pad]
    slot_duration: float  # seconds per slot, default 30
    slots: Dict[Tuple[str, int], Optional[str]]        # (pad_id, slot_index) -> drone_id | None
    slot_status: Dict[Tuple[str, int], str]            # (pad_id, slot_index) -> 'FREE'|'SOFT'|'COMMITTED'


@dataclass
class SystemState:
    approved_plans: List[ApprovedPlan]
    vertiports: Dict[str, VertiportState]     # vertiport_id -> VertiportState
    t_now: float
    msd_matrix: Dict[Tuple[str, str], float]  # (uav_type_i, uav_type_j) -> MSD [m]
    body_length: Dict[str, float]             # uav_type -> body length [m]


@dataclass
class SCRPConfig:
    SOC_MIN: float = 0.20
    SLOT_DURATION_SEC: float = 30.0
    T_RESPONSE_WINDOW_SEC: float = 120.0
    JUNCTION_DIAMETER_M: float = 20.0
    DEFAULT_TIMEOUT_BEHAVIOR: str = 'reject'   # 'reject' | 'accept'
    CRUISE_POWER_FACTOR: float = 1.2
    MAX_ACCEPTABLE_DELAY_SEC: float = 1800.0


@dataclass
class ApproveResult:
    status: Literal['SOFT_RESERVED']
    t_dep_star: float
    t_land_assigned: float
    pad_id: str
    slot_index: int
    vertiport_id: str                        # destination vertiport for this approval
    waypoint_times: List[Tuple[str, float]]  # [(waypoint_id, time), ...]
    expires_at: float
    delay_seconds: float
    delay_source: str  # 'C1_headway' | 'C2_junction' | 'C3_slot' | 'none'
    energy_estimate: float   # E_cruise [Wh]
    SoC_remaining: float


@dataclass
class RejectResult:
    status: Literal['REJECTED']
    reason: str   # 'SoC_insufficient' | 'no_slot_available' | 'no_feasible_t_dep' | 'invalid_fi'
    earliest_possible: Optional[float]
    detail: str


SCRPResult = ApproveResult | RejectResult


@dataclass
class PlanDependency:
    plan_id: str
    depends_on: str


# ---------------------------------------------------------------------------
# Flight route suggestion models
# ---------------------------------------------------------------------------

@dataclass
class FlightRoute:
    """A pre-planned route published by the external route service."""
    route_id: str
    waypoints: List[Waypoint]
    take_off_vertiport_id: str
    landing_vertiport_id: str
    safety_score: float           # 0.0–1.0; higher is safer
    average_wind_speed: float     # m/s average wind speed along this route
    max_drone_weight_kg: float    # maximum drone weight this route supports
    compatible_drone_sizes: List[str]  # subset of {'A', 'B', 'C'}
    segments: List[Segment] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.segments and len(self.waypoints) >= 2:
            import math
            segs: List[Segment] = []
            for i in range(len(self.waypoints) - 1):
                a, b = self.waypoints[i], self.waypoints[i + 1]
                dx = b.position[0] - a.position[0]
                dy = b.position[1] - a.position[1]
                dz = b.position[2] - a.position[2]
                length = math.sqrt(dx * dx + dy * dy + dz * dz)
                segs.append(Segment(p_start=a, p_end=b, length=length, v_min=5.0, v_max=20.0))
            self.segments = segs


@dataclass
class DroneProfile:
    """Drone characteristics drawn directly from the manufacturer's datasheet.

    Fields mirror standard datasheet sections: physical, performance, propulsion,
    power/battery.  Mission-specific parameters (SoC, cruise speed, takeoff/landing
    height) are added as optional fields because they change per flight while the
    datasheet values remain constant.
    """

    # --- Size class (for route compatibility) ---
    drone_size: str                   # 'A' | 'B' | 'C'

    # --- Physical & structural ---
    mtow_kg: float                    # Max Takeoff Weight [kg]
    num_rotors: int                   # Number of rotors
    propeller_diameter_m: float       # Propeller diameter [m]
    max_tilt_angle_deg: float         # Max tilt angle [°]; used to derive drag area

    # --- Performance envelope ---
    max_speed_ms: float               # Max horizontal speed [m/s]
    max_ascent_speed_ms: float        # Max ascent speed [m/s]
    max_descent_speed_ms: float       # Max descent speed [m/s]
    max_wind_resistance_ms: float     # Max tolerated wind speed [m/s]
    service_ceiling_m: float          # Max operating altitude [m ASL]

    # --- Power / battery (from datasheet) ---
    battery_energy_wh: float          # Battery energy [Wh]

    # hover_power_w: Hover power consumption [W].
    # Set to 0.0 (the default) to let the model derive it automatically:
    #   1. From battery_energy_wh + flight_time_min (if flight_time_min > 0)
    #   2. From momentum theory using mtow_kg, num_rotors, propeller_diameter_m
    hover_power_w: float = 0.0

    # flight_time_min: manufacturer's stated endurance [minutes].
    # Used only to derive hover_power_w when that field is absent.
    # Should be the hover-only endurance figure if the datasheet distinguishes it
    # from the forward-flight range figure.
    flight_time_min: float = 0.0

    # --- Mission-specific parameters (not on the datasheet) ---
    soc_0: float = 1.0                # Initial state of charge [0..1]
    soc_min: float = 0.20             # Minimum acceptable SoC on arrival
    cruise_speed_ms: float = 0.0      # Planned cruise speed; 0 → 75 % of max_speed_ms
    takeoff_height_m: float = 50.0    # Altitude to climb during takeoff phase [m]
    landing_height_m: float = 50.0    # Altitude to descend during landing phase [m]


@dataclass
class RouteFlightEstimate:
    """Per-route power and time estimates produced by the suggestion algorithm."""
    cruise_time_s: float        # s; time spent flying the route segments
    total_time_s: float         # s; cruise_time + takeoff + landing phases
    energy_consumed_wh: float   # Wh; total energy across all flight phases
    SoC_remaining: float        # [0..1]; estimated battery fraction on arrival


@dataclass
class SuggestedRoute:
    """A compatible route paired with its estimated flight metrics."""
    route: FlightRoute
    estimate: RouteFlightEstimate


@dataclass
class RouteSuggestionRequest:
    """What a client submits to ask which routes are flyable.

    The safety viability of each route is evaluated internally using the
    route's safety_score from the external route service.  Callers do not
    set a safety threshold — the system adapts the minimum required score
    to the drone's operating conditions (primarily wind margin).
    """
    take_off_vertiport_id: str
    landing_vertiport_id: str
    drone: DroneProfile


@dataclass
class RouteRejection:
    route: FlightRoute
    reason: str  # see rejection reason constants in route_suggestion.py


@dataclass
class RouteSuggestionResult:
    suggested_routes: List[SuggestedRoute]
    rejected_routes: List[RouteRejection]
