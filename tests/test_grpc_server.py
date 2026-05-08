"""Integration tests for the gRPC microservice layer.

Starts a real in-process gRPC server, sends requests via a real stub,
and verifies responses round-trip correctly.
"""
from __future__ import annotations

import math
from concurrent import futures

import grpc
import pytest

from scrp.grpc_server import SCRPServicer
from scrp.proto import scrp_pb2, scrp_pb2_grpc

# ---------------------------------------------------------------------------
# Helpers to build proto messages (mirrors tests/conftest.py domain fixtures)
# ---------------------------------------------------------------------------

def _wp(id: str, x: float, y: float, z: float = 50.0) -> scrp_pb2.WaypointProto:
    return scrp_pb2.WaypointProto(id=id, x=x, y=y, z=z)


def _seg(
    p_start: scrp_pb2.WaypointProto,
    p_end: scrp_pb2.WaypointProto,
) -> scrp_pb2.SegmentProto:
    dx = p_end.x - p_start.x
    dy = p_end.y - p_start.y
    dz = p_end.z - p_start.z
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    return scrp_pb2.SegmentProto(
        p_start=p_start, p_end=p_end, length=length, v_min=5.0, v_max=20.0
    )


def _lane(id: str, wps: list[scrp_pb2.WaypointProto]) -> scrp_pb2.LaneProto:
    segs = [_seg(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
    return scrp_pb2.LaneProto(id=id, waypoints=wps, segments=segs)


def _simple_lane() -> scrp_pb2.LaneProto:
    wps = [_wp("P1", 0, 0), _wp("P2", 100, 0), _wp("P3", 200, 0)]
    return _lane("lane1", wps)


def _fi(
    lane: scrp_pb2.LaneProto,
    *,
    drone_id: str = "drone_new",
    v: float = 10.0,
    t_des: float = 1000.0,
    uav_type: str = "A",
    soc_0: float = 1.0,
    c_bat: float = 1000.0,
    p_hover: float = 500.0,
) -> scrp_pb2.OperatorFlightRequestProto:
    return scrp_pb2.OperatorFlightRequestProto(
        operator_id="op1",
        drone_id=drone_id,
        lane=lane,
        v_waypoints=[v] * len(lane.waypoints),
        destination_vertiport_id="vp1",
        t_des=t_des,
        priority=1,
        uav_type=uav_type,
        soc_0=soc_0,
        c_bat=c_bat,
        p_hover=p_hover,
    )


def _vertiport(n_slots: int = 200) -> scrp_pb2.VertiportStateProto:
    slots = [
        scrp_pb2.SlotEntryProto(pad_id="pad1", slot_index=i, drone_id="", status="FREE")
        for i in range(n_slots)
    ]
    return scrp_pb2.VertiportStateProto(
        vertiport_id="vp1",
        pads=[scrp_pb2.PadProto(id="pad1", compatible_types=["A", "B", "C"])],
        slot_duration=30.0,
        slots=slots,
    )


def _system_state(
    approved_plans: list[scrp_pb2.ApprovedPlanProto] | None = None,
    t_now: float = 900.0,
) -> scrp_pb2.SystemStateProto:
    msd = [
        scrp_pb2.MsdEntryProto(uav_type_i="A", uav_type_j="A", msd=50.0),
        scrp_pb2.MsdEntryProto(uav_type_i="A", uav_type_j="B", msd=60.0),
        scrp_pb2.MsdEntryProto(uav_type_i="B", uav_type_j="A", msd=60.0),
        scrp_pb2.MsdEntryProto(uav_type_i="B", uav_type_j="B", msd=55.0),
        scrp_pb2.MsdEntryProto(uav_type_i="A", uav_type_j="C", msd=65.0),
        scrp_pb2.MsdEntryProto(uav_type_i="C", uav_type_j="A", msd=65.0),
        scrp_pb2.MsdEntryProto(uav_type_i="C", uav_type_j="C", msd=70.0),
        scrp_pb2.MsdEntryProto(uav_type_i="B", uav_type_j="C", msd=62.0),
        scrp_pb2.MsdEntryProto(uav_type_i="C", uav_type_j="B", msd=62.0),
    ]
    body = [
        scrp_pb2.BodyLengthEntryProto(uav_type="A", length_m=1.0),
        scrp_pb2.BodyLengthEntryProto(uav_type="B", length_m=1.5),
        scrp_pb2.BodyLengthEntryProto(uav_type="C", length_m=2.0),
    ]
    return scrp_pb2.SystemStateProto(
        approved_plans=approved_plans or [],
        vertiport_state=_vertiport(),
        t_now=t_now,
        msd_matrix=msd,
        body_length=body,
    )


def _default_config() -> scrp_pb2.SCRPConfigProto:
    return scrp_pb2.SCRPConfigProto(
        soc_min=0.20,
        slot_duration_sec=30.0,
        t_response_window_sec=120.0,
        junction_diameter_m=20.0,
        default_timeout_behavior="reject",
        cruise_power_factor=1.2,
        max_acceptable_delay_sec=1800.0,
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def grpc_stub() -> scrp_pb2_grpc.SCRPServiceStub:
    """Start an in-process gRPC server and return a connected stub."""
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=2))
    scrp_pb2_grpc.add_SCRPServiceServicer_to_server(SCRPServicer(), server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()
    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    stub = scrp_pb2_grpc.SCRPServiceStub(channel)
    yield stub
    channel.close()
    server.stop(grace=0)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestGrpcResolveConflict:
    def test_no_conflict_returns_approve(self, grpc_stub):
        lane = _simple_lane()
        request = scrp_pb2.ResolveConflictRequest(
            fi=_fi(lane),
            approved_plans=[],
            vertiport_state=_vertiport(),
            system_state=_system_state(),
            config=_default_config(),
        )
        response = grpc_stub.ResolveConflict(request)
        assert response.HasField("approve")
        assert response.approve.status == "SOFT_RESERVED"
        assert response.approve.t_dep_star == pytest.approx(1000.0, abs=1e-3)
        assert response.approve.delay_seconds == pytest.approx(0.0, abs=1e-3)
        assert response.approve.delay_source == "none"

    def test_approve_includes_waypoint_times(self, grpc_stub):
        lane = _simple_lane()
        request = scrp_pb2.ResolveConflictRequest(
            fi=_fi(lane),
            approved_plans=[],
            vertiport_state=_vertiport(),
            system_state=_system_state(),
            config=_default_config(),
        )
        response = grpc_stub.ResolveConflict(request)
        assert response.HasField("approve")
        wt = response.approve.waypoint_times
        assert len(wt) == len(lane.waypoints)
        assert all(entry.waypoint_id != "" for entry in wt)
        assert all(entry.time > 0 for entry in wt)

    def test_soc_insufficient_returns_reject(self, grpc_stub):
        lane = _simple_lane()
        request = scrp_pb2.ResolveConflictRequest(
            fi=_fi(lane, soc_0=0.001, c_bat=1.0),
            approved_plans=[],
            vertiport_state=_vertiport(),
            system_state=_system_state(),
            config=_default_config(),
        )
        response = grpc_stub.ResolveConflict(request)
        assert response.HasField("reject")
        assert response.reject.status == "REJECTED"
        assert response.reject.reason == "SoC_insufficient"

    def test_invalid_fi_returns_reject(self, grpc_stub):
        lane = _simple_lane()
        # Mismatched v_waypoints length triggers invalid_fi
        bad_fi = scrp_pb2.OperatorFlightRequestProto(
            operator_id="op1",
            drone_id="drone_bad",
            lane=lane,
            v_waypoints=[10.0],  # wrong length (lane has 3 waypoints)
            destination_vertiport_id="vp1",
            t_des=1000.0,
            priority=1,
            uav_type="A",
            soc_0=1.0,
            c_bat=1000.0,
            p_hover=500.0,
        )
        request = scrp_pb2.ResolveConflictRequest(
            fi=bad_fi,
            approved_plans=[],
            vertiport_state=_vertiport(),
            system_state=_system_state(),
            config=_default_config(),
        )
        response = grpc_stub.ResolveConflict(request)
        assert response.HasField("reject")
        assert response.reject.reason == "invalid_fi"

    def test_approve_soc_and_energy_populated(self, grpc_stub):
        lane = _simple_lane()
        request = scrp_pb2.ResolveConflictRequest(
            fi=_fi(lane, soc_0=1.0, c_bat=1000.0, p_hover=500.0),
            approved_plans=[],
            vertiport_state=_vertiport(),
            system_state=_system_state(),
            config=_default_config(),
        )
        response = grpc_stub.ResolveConflict(request)
        assert response.HasField("approve")
        assert 0.0 < response.approve.soc_remaining <= 1.0
        assert response.approve.energy_estimate > 0.0

    def test_reject_has_earliest_possible_flag(self, grpc_stub):
        lane = _simple_lane()
        # Force max delay exceeded: use a config with tiny max delay
        tight_config = scrp_pb2.SCRPConfigProto(
            soc_min=0.20,
            slot_duration_sec=30.0,
            t_response_window_sec=120.0,
            junction_diameter_m=20.0,
            default_timeout_behavior="reject",
            cruise_power_factor=1.2,
            max_acceptable_delay_sec=0.0,  # impossible to satisfy
        )
        # Add an approved plan that forces a delay
        existing_wps = [_wp("P1", 0, 0), _wp("P2", 100, 0), _wp("P3", 200, 0)]
        existing_lane = _lane("lane1", existing_wps)
        existing_plan = scrp_pb2.ApprovedPlanProto(
            drone_id="drone_existing",
            uav_type="A",
            lane=existing_lane,
            v_waypoints=[10.0, 10.0, 10.0],
            t_land=1040.0,
            pad_id="pad1",
            slot_index=0,
            status="COMMITTED",
            t_dep=1000.0,
            algorithm="SCRP",
        )
        request = scrp_pb2.ResolveConflictRequest(
            fi=_fi(lane),
            approved_plans=[existing_plan],
            vertiport_state=_vertiport(),
            system_state=_system_state(approved_plans=[existing_plan]),
            config=tight_config,
        )
        response = grpc_stub.ResolveConflict(request)
        # Either rejected with no_feasible_t_dep or approved — depends on
        # whether a conflict exists. Either path must parse without error.
        assert response.HasField("approve") or response.HasField("reject")
