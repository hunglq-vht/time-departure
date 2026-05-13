"""Shared fixtures for all SCRP tests."""
from __future__ import annotations

from typing import Optional

import pytest

from scrp.models import (
    ApprovedPlan,
    FlightIntention,
    Lane,
    Pad,
    SCRPConfig,
    Segment,
    SystemState,
    TakeoffPad,
    VertiportState,
    Waypoint,
)


def make_waypoint(id: str, x: float = 0.0, y: float = 0.0, z: float = 50.0) -> Waypoint:
    return Waypoint(id=id, position=(x, y, z))


def make_lane(
    id: str,
    waypoints: list[Waypoint],
    v_min: float = 5.0,
    v_max: float = 20.0,
    destination_vertiport_id: str = "vp1",
) -> Lane:
    import math
    segments = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        dx = b.position[0] - a.position[0]
        dy = b.position[1] - a.position[1]
        dz = b.position[2] - a.position[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        segments.append(Segment(p_start=a, p_end=b, length=length, v_min=v_min, v_max=v_max))
    lane = Lane(
        id=id,
        waypoints=waypoints,
        segments=segments,
        destination_vertiport_id=destination_vertiport_id,
    )
    return lane


def make_fi(
    lane: Lane,
    v: float = 10.0,
    t_des: float = 1000.0,
    SoC_0: float = 1.0,
    C_bat: float = 1000.0,
    P_hover: float = 500.0,
    drone_id: str = "drone_new",
    uav_type: str = "A",
    operator_id: str = "op1",
    priority: int = 1,
    t_takeoff: Optional[float] = None,
    t_land_estimated: Optional[float] = None,
    origin_vertiport_id: str = "",
    takeoff_pad_id: str = "",
) -> FlightIntention:
    v_waypoints = [v] * len(lane.waypoints)
    return FlightIntention(
        drone_id=drone_id,
        uav_type=uav_type,
        lane=lane,
        v_waypoints=v_waypoints,
        t_des=t_des,
        SoC_0=SoC_0,
        C_bat=C_bat,
        P_hover=P_hover,
        priority=priority,
        operator_id=operator_id,
        t_takeoff=t_takeoff,
        t_land_estimated=t_land_estimated,
        origin_vertiport_id=origin_vertiport_id,
        takeoff_pad_id=takeoff_pad_id,
    )


def make_takeoff_pad(
    id: str,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    compatible_types: list[str] | None = None,
) -> TakeoffPad:
    if compatible_types is None:
        compatible_types = ["A", "B", "C"]
    return TakeoffPad(id=id, position=(x, y, z), compatible_types=compatible_types)


def make_vertiport(
    slot_duration: float = 30.0,
    n_slots: int = 200,
    vertiport_id: str = "vp1",
    pad_configs: list[tuple[str, list[str]]] | None = None,
    takeoff_pads: list[TakeoffPad] | None = None,
) -> VertiportState:
    if pad_configs is None:
        pad_configs = [("pad1", ["A", "B", "C"])]
    pads = [Pad(id=pid, compatible_types=types) for pid, types in pad_configs]
    slots: dict = {}
    slot_status: dict = {}
    for pid, _ in pad_configs:
        for i in range(n_slots):
            slots[(pid, i)] = None
            slot_status[(pid, i)] = "FREE"
    return VertiportState(
        vertiport_id=vertiport_id,
        pads=pads,
        slot_duration=slot_duration,
        slots=slots,
        slot_status=slot_status,
        takeoff_pads=takeoff_pads or [],
    )


def make_system_state(
    approved_plans: list[ApprovedPlan] | None = None,
    vertiport: VertiportState | None = None,
    t_now: float = 900.0,
    vertiports: dict[str, VertiportState] | None = None,
) -> SystemState:
    """Build a SystemState.

    Pass ``vertiports`` for multi-vertiport setups.  For single-vertiport
    tests the legacy ``vertiport`` kwarg is still accepted; it is stored
    under its own ``vertiport_id``.
    """
    if vertiports is None:
        vp = vertiport or make_vertiport()
        vertiports = {vp.vertiport_id: vp}
    return SystemState(
        approved_plans=approved_plans or [],
        vertiports=vertiports,
        t_now=t_now,
        msd_matrix={
            ("A", "A"): 50.0,
            ("A", "B"): 60.0,
            ("B", "A"): 60.0,
            ("B", "B"): 55.0,
            ("A", "C"): 65.0,
            ("C", "A"): 65.0,
            ("C", "C"): 70.0,
            ("B", "C"): 62.0,
            ("C", "B"): 62.0,
        },
        body_length={"A": 1.0, "B": 1.5, "C": 2.0},
    )


@pytest.fixture
def config() -> SCRPConfig:
    return SCRPConfig()


@pytest.fixture
def simple_lane() -> Lane:
    wps = [
        make_waypoint("P1", 0, 0),
        make_waypoint("P2", 100, 0),
        make_waypoint("P3", 200, 0),
    ]
    return make_lane("lane1", wps, destination_vertiport_id="vp1")


@pytest.fixture
def vertiport() -> VertiportState:
    return make_vertiport()


@pytest.fixture
def system_state(vertiport) -> SystemState:
    return make_system_state(vertiport=vertiport)
