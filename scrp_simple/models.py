"""Data models for the simplified conflict resolver."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class Point3D:
    """3-D coordinate in metres."""

    x: float
    y: float
    z: float

    def distance_to(self, other: Point3D) -> float:
        return math.sqrt(
            (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2
        )


@dataclass
class Vertiport:
    id: str
    location: Point3D
    total_pads: int  # number of simultaneous landings supported


@dataclass
class FlightPath:
    """
    Route descriptor.

    ``waypoints`` are the *intermediate* points only – the takeoff and landing
    vertiport locations are **not** included here; the resolver looks them up
    from the vertiport registry.

    Full geometric path:
        [takeoff_location] + waypoints + [landing_location]

    ``segment_speeds`` (on ApprovedPlan / NewPlanRequest) must contain one
    entry for every consecutive pair in the full path, i.e.
    ``len(waypoints) + 1`` speeds.
    """

    takeoff_vertiport_id: str
    landing_vertiport_id: str
    waypoints: List[Point3D]


@dataclass
class ApprovedPlan:
    plan_id: str
    flight_path: FlightPath
    drone_type: str
    segment_speeds: List[float]  # m/s, one per segment of the full path
    start_time: float            # absolute departure time in seconds
    priority: int = 5            # 1 = highest, 10 = lowest


@dataclass
class NewPlanRequest:
    flight_path: FlightPath
    drone_type: str
    segment_speeds: List[float]
    desired_start_time: float
    max_wait_time: float  # maximum tolerable delay in seconds
    priority: int = 5    # 1 = highest, 10 = lowest


@dataclass
class ResolveResult:
    approved: bool
    actual_start_time: Optional[float]  # None if rejected
    delay: float                        # seconds of delay applied (0 if no conflict)
    reason: str
    conflict_details: List[str] = field(default_factory=list)
