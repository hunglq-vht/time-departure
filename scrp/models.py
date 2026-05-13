from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional, Tuple, Union


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
    P_hover: float                    # hover power [W]
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
    P_hover: float            # hover power [W]
    priority: int             # 1 | 2 | 3
    operator_id: str
    t_takeoff: Optional[float] = None       # takeoff phase duration [s]; added before entering the lane
    t_land_estimated: Optional[float] = None  # landing phase duration [s]; added after exiting the lane
    origin_vertiport_id: str = ""           # vertiport where this drone departs from
    takeoff_pad_id: str = ""               # pad at origin vertiport used for departure


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
    origin_vertiport_id: str = ""                  # origin vertiport this plan departs from
    takeoff_pad_id: str = ""                       # pad at origin vertiport used for departure
    t_takeoff: Optional[float] = None              # takeoff phase duration [s]


@dataclass
class Pad:
    id: str
    compatible_types: List[str]


@dataclass
class TakeoffPad:
    """A pad at the origin (takeoff) vertiport from which drones depart."""
    id: str
    position: Tuple[float, float, float]  # (x, y, z) absolute metres
    compatible_types: List[str]


@dataclass
class VertiportState:
    vertiport_id: str
    pads: List[Pad]
    slot_duration: float  # seconds per slot, default 30
    slots: Dict[Tuple[str, int], Optional[str]]        # (pad_id, slot_index) -> drone_id | None
    slot_status: Dict[Tuple[str, int], str]            # (pad_id, slot_index) -> 'FREE'|'SOFT'|'COMMITTED'
    takeoff_pads: List[TakeoffPad] = field(default_factory=list)  # pads used for departure


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


SCRPResult = Union[ApproveResult, RejectResult]


@dataclass
class PlanDependency:
    plan_id: str
    depends_on: str


@dataclass
class DroneDeconflictEntry:
    """Single entry in a multi-drone batch deconfliction result."""
    drone_id: str
    result: SCRPResult
    processing_rank: int  # order in which this drone was processed (0 = highest priority)


@dataclass
class BatchDeconflictResult:
    """Aggregated result from resolving a group of drones at the same takeoff vertiport."""
    entries: List[DroneDeconflictEntry]           # one per input FI, in original input order
    approved_count: int
    rejected_count: int
    total_drones: int
