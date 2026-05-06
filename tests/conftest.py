"""Shared fixtures for all SCRP tests."""
from __future__ import annotations

import pytest

from scrp.models import (
    ApprovedPlan,
    FlightIntention,
    Lane,
    Pad,
    SCRPConfig,
    Segment,
    SystemState,
    VertiportState,
    Waypoint,
)


def make_waypoint(id: str, x: float = 0.0, y: float = 0.0, z: float = 50.0) -> Waypoint:
    return Waypoint(id=id, position=(x, y, z))


def make_lane(id: str, waypoints: list[Waypoint], v_min: float = 5.0, v_max: float = 20.0) -> Lane:
    import math
    segments = []
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        dx = b.position[0] - a.position[0]
        dy = b.position[1] - a.position[1]
        dz = b.position[2] - a.position[2]
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        segments.append(Segment(p_start=a, p_end=b, length=length, v_min=v_min, v_max=v_max))
    lane = Lane(id=id, waypoints=waypoints, segments=segments)
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
        priority=1,
        operator_id=operator_id,
    )


def make_vertiport(
    slot_duration: float = 30.0,
    n_slots: int = 200,
    pad_configs: list[tuple[str, list[str]]] | None = None,
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
        vertiport_id="vp1",
        pads=pads,
        slot_duration=slot_duration,
        slots=slots,
        slot_status=slot_status,
    )


def make_system_state(
    approved_plans: list[ApprovedPlan] | None = None,
    vertiport: VertiportState | None = None,
    t_now: float = 900.0,
) -> SystemState:
    return SystemState(
        approved_plans=approved_plans or [],
        vertiport_state=vertiport or make_vertiport(),
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
    return make_lane("lane1", wps)


@pytest.fixture
def vertiport() -> VertiportState:
    return make_vertiport()


@pytest.fixture
def system_state(vertiport) -> SystemState:
    return make_system_state(vertiport=vertiport)
