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
            (self.x - other.x) ** 2
            + (self.y - other.y) ** 2
            + (self.z - other.z) ** 2
        )


@dataclass
class Pad:
    """A single landing/takeoff pad at a vertiport.

    ``occupation_duration`` is how long the pad stays physically occupied
    *after* a drone touches down (unloading, charging, clearing, etc.).
    Different pads can have different durations (e.g., a fast-turnaround pad
    vs a charging pad that keeps the drone for longer).
    """

    id: str
    occupation_duration: float  # seconds the pad is busy after touchdown


@dataclass
class Vertiport:
    """A vertiport with one or more landing/takeoff pads.

    Each pad is an independent physical platform; multiple pads can be
    occupied simultaneously once their respective drones have touched down.
    However, the *shared airspace* above the vertiport (used during takeoff
    ascent and landing descent) can only be occupied by one drone at a time.
    """

    id: str
    location: Point3D
    pads: List[Pad]  # at least one pad expected


@dataclass
class FlightPath:
    """Route descriptor.

    ``waypoints`` are the *intermediate* lane points only – the takeoff and
    landing vertiport locations are NOT included here.

    Full journey timeline
    ---------------------
    ::

        [start_time]                  drone on pad, about to take off
        [start_time + t_takeoff]      drone reaches waypoints[0]  ← lane begins
        ...lane segments (wp→wp)...
        [start_time + t_takeoff
         + lane_time]                 drone reaches waypoints[-1]  ← lane ends
        [start_time + t_takeoff
         + lane_time + t_land]        drone touches down on landing pad

    ``segment_speeds`` (on ApprovedPlan / NewPlanRequest) covers only the
    *N-1* lane segments between N waypoints.  For a direct hop with no
    intermediate waypoints, ``segment_speeds`` is empty and ``lane_time`` is 0.

    ``t_takeoff`` and ``t_land`` are stored on the plan (not the path) because
    they depend on the specific drone's climb/descent performance.
    """

    takeoff_vertiport_id: str
    landing_vertiport_id: str
    waypoints: List[Point3D]  # intermediate only; excludes vertiport locations


@dataclass
class ApprovedPlan:
    """A flight plan that has already been approved and is in the system.

    ``assigned_pad_id`` records which pad at the landing vertiport was
    reserved for this plan.  It is needed to calculate per-pad occupation
    windows when resolving conflicts for future requests.
    """

    plan_id: str
    flight_path: FlightPath
    drone_type: str
    segment_speeds: List[float]  # m/s; len == len(waypoints) - 1
    start_time: float            # absolute departure time in seconds
    t_takeoff: float             # ascent duration: pad → waypoints[0]
    t_land: float                # descent duration: waypoints[-1] → pad
    priority: int = 5            # 1 = highest priority
    assigned_pad_id: Optional[str] = None  # set at approval time


@dataclass
class NewPlanRequest:
    """A flight plan submitted for approval.

    ``desired_start_time`` is the earliest the operator wants to depart.
    The resolver may approve a later time (up to ``max_wait_time`` seconds
    of added delay); beyond that the request is rejected.
    """

    flight_path: FlightPath
    drone_type: str
    segment_speeds: List[float]
    desired_start_time: float
    max_wait_time: float   # maximum tolerable delay in seconds
    t_takeoff: float       # ascent duration: pad → waypoints[0]
    t_land: float          # descent duration: waypoints[-1] → pad
    priority: int = 5      # 1 = highest priority


@dataclass
class ResolveResult:
    """Outcome of a conflict-resolution run."""

    approved: bool
    actual_start_time: Optional[float]  # None when rejected
    delay: float                        # seconds added beyond desired_start_time
    reason: str
    assigned_pad_id: Optional[str] = None  # None when rejected
    conflict_details: List[str] = field(default_factory=list)
