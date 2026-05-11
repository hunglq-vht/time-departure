"""
Example: sending a ResolveConflict request to the SCRP gRPC service.

Usage:
    python client_example.py --host localhost --port 50051

Requirements:
    pip install grpcio protobuf
"""

from __future__ import annotations

import argparse
import math
import time

import grpc

from scrp.proto import scrp_pb2, scrp_pb2_grpc


# ---------------------------------------------------------------------------
# Helpers for building proto messages
# ---------------------------------------------------------------------------

def make_waypoint(id: str, x: float, y: float, z: float = 50.0) -> scrp_pb2.WaypointProto:
    return scrp_pb2.WaypointProto(id=id, x=x, y=y, z=z)


def make_segment(
    p_start: scrp_pb2.WaypointProto,
    p_end: scrp_pb2.WaypointProto,
    v_min: float = 5.0,
    v_max: float = 20.0,
) -> scrp_pb2.SegmentProto:
    dx, dy, dz = p_end.x - p_start.x, p_end.y - p_start.y, p_end.z - p_start.z
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    return scrp_pb2.SegmentProto(p_start=p_start, p_end=p_end, length=length, v_min=v_min, v_max=v_max)


def make_lane(id: str, waypoints: list[scrp_pb2.WaypointProto]) -> scrp_pb2.LaneProto:
    segments = [make_segment(waypoints[i], waypoints[i + 1]) for i in range(len(waypoints) - 1)]
    return scrp_pb2.LaneProto(id=id, waypoints=waypoints, segments=segments)


def make_vertiport(vertiport_id: str = "vp1", n_slots: int = 10) -> scrp_pb2.VertiportStateProto:
    slots = [
        scrp_pb2.SlotEntryProto(pad_id="pad1", slot_index=i, drone_id="", status="FREE")
        for i in range(n_slots)
    ]
    return scrp_pb2.VertiportStateProto(
        vertiport_id=vertiport_id,
        pads=[scrp_pb2.PadProto(id="pad1", compatible_types=["A", "B", "C"])],
        slot_duration=30.0,
        slots=slots,
    )


def make_system_state(
    vertiport: scrp_pb2.VertiportStateProto,
    approved_plans: list[scrp_pb2.ApprovedPlanProto] | None = None,
    t_now: float | None = None,
) -> scrp_pb2.SystemStateProto:
    msd_matrix = [
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
    body_lengths = [
        scrp_pb2.BodyLengthEntryProto(uav_type="A", length_m=1.0),
        scrp_pb2.BodyLengthEntryProto(uav_type="B", length_m=1.5),
        scrp_pb2.BodyLengthEntryProto(uav_type="C", length_m=2.0),
    ]
    return scrp_pb2.SystemStateProto(
        approved_plans=approved_plans or [],
        vertiport_state=vertiport,
        t_now=t_now if t_now is not None else time.time(),
        msd_matrix=msd_matrix,
        body_length=body_lengths,
    )


def make_config() -> scrp_pb2.SCRPConfigProto:
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
# Sending the request
# ---------------------------------------------------------------------------

def resolve_conflict(host: str, port: int) -> None:
    # Build the flight path: three waypoints forming a straight 200 m corridor
    waypoints = [
        make_waypoint("P1", x=0.0, y=0.0),
        make_waypoint("P2", x=100.0, y=0.0),
        make_waypoint("P3", x=200.0, y=0.0),
    ]
    lane = make_lane("lane1", waypoints)

    # Desired departure 60 seconds from now
    t_now = time.time()
    t_desired_departure = t_now + 60.0

    flight_intention = scrp_pb2.OperatorFlightRequestProto(
        operator_id="op1",
        drone_id="drone_alpha",
        lane=lane,
        v_waypoints=[10.0, 10.0, 10.0],    # 10 m/s on every segment
        destination_vertiport_id="vp1",
        t_des=t_desired_departure,
        priority=1,                          # 1 = highest priority
        uav_type="A",
        soc_0=1.0,                           # fully charged
        c_bat=1000.0,                        # 1000 Wh battery
        p_hover=500.0,                       # 500 W hover power
    )

    vertiport = make_vertiport()
    system_state = make_system_state(vertiport=vertiport, t_now=t_now)

    request = scrp_pb2.ResolveConflictRequest(
        fi=flight_intention,
        approved_plans=[],       # no existing bookings in this example
        vertiport_state=vertiport,
        system_state=system_state,
        config=make_config(),
    )

    # Open an insecure channel (no TLS). For TLS use grpc.secure_channel().
    target = f"{host}:{port}"
    print(f"Connecting to SCRP service at {target} ...")
    with grpc.insecure_channel(target) as channel:
        stub = scrp_pb2_grpc.SCRPServiceStub(channel)
        response = stub.ResolveConflict(request, timeout=10.0)

    # Handle the response — exactly one of "approve" or "reject" is set
    if response.HasField("approve"):
        a = response.approve
        print("APPROVED")
        print(f"  Status          : {a.status}")
        print(f"  t_dep*          : {a.t_dep_star:.3f}  (Unix seconds)")
        print(f"  Delay           : {a.delay_seconds:.1f} s  [{a.delay_source}]")
        print(f"  Landing time    : {a.t_land_assigned:.3f}")
        print(f"  Pad / slot      : {a.pad_id} / slot {a.slot_index}")
        print(f"  Energy estimate : {a.energy_estimate:.2f} Wh")
        print(f"  SoC remaining   : {a.soc_remaining * 100:.1f} %")
        print(f"  Reservation expires at: {a.expires_at:.3f}")
        print("  Waypoint schedule:")
        for wt in a.waypoint_times:
            print(f"    {wt.waypoint_id:6s}  t = {wt.time:.3f}")
    else:
        r = response.reject
        print("REJECTED")
        print(f"  Status  : {r.status}")
        print(f"  Reason  : {r.reason}")
        print(f"  Detail  : {r.detail}")
        if r.has_earliest_possible:
            print(f"  Earliest feasible departure: {r.earliest_possible:.3f}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="SCRP gRPC client example")
    parser.add_argument("--host", default="localhost", help="Service host (default: localhost)")
    parser.add_argument("--port", type=int, default=50051, help="Service port (default: 50051)")
    args = parser.parse_args()

    resolve_conflict(args.host, args.port)
