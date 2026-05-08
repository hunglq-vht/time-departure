"""
Batch gRPC integration test — multiple operators concurrently send flight
intentions to a gRPC server; the server queues them in a BatchListener
and processes all pending requests together every batch_interval seconds
(10 s in production, configurable for testing).

Scenario
--------
* 5 operators each with one drone.
* Operators A1, A2, A3 fly the **same shared corridor** (headway conflicts
  expected when an existing committed plan is present on that lane).
* Operators B1, C1 fly **exclusive lanes** (no shared segments, no conflict).
* One pre-existing COMMITTED plan occupies the shared lane, forcing at least
  the lowest-priority new request to be delayed.

All five gRPC calls are issued simultaneously from separate threads and block
until the BatchListener fires and returns each result.
"""
from __future__ import annotations

import math
import threading
import time
from concurrent import futures
from typing import Dict, List, Tuple

import grpc
import pytest

from scrp.batch_listener import BatchListener
from scrp.grpc_server import (
    _approve_result_to_proto,
    _operator_fi_from_proto,
    _reject_result_to_proto,
    _to_flight_intention,
)
from scrp.models import (
    ApproveResult,
    ApprovedPlan,
    Lane,
    Pad,
    RejectResult,
    SCRPConfig,
    SCRPResult,
    Segment,
    SystemState,
    VertiportState,
    Waypoint,
)
from scrp.proto import scrp_pb2, scrp_pb2_grpc

# ---------------------------------------------------------------------------
# Proto-message helpers
# ---------------------------------------------------------------------------

def _wp(id: str, x: float, y: float, z: float = 50.0) -> scrp_pb2.WaypointProto:
    return scrp_pb2.WaypointProto(id=id, x=x, y=y, z=z)


def _seg(p_start: scrp_pb2.WaypointProto, p_end: scrp_pb2.WaypointProto) -> scrp_pb2.SegmentProto:
    dx, dy, dz = p_end.x - p_start.x, p_end.y - p_start.y, p_end.z - p_start.z
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    return scrp_pb2.SegmentProto(p_start=p_start, p_end=p_end, length=length, v_min=5.0, v_max=20.0)


def _lane_proto(id: str, wps: List[scrp_pb2.WaypointProto]) -> scrp_pb2.LaneProto:
    segs = [_seg(wps[i], wps[i + 1]) for i in range(len(wps) - 1)]
    return scrp_pb2.LaneProto(id=id, waypoints=wps, segments=segs)


def _shared_lane_proto() -> scrp_pb2.LaneProto:
    """Shared corridor: three collinear waypoints along the X axis."""
    wps = [_wp("SH_P1", 0, 0), _wp("SH_P2", 200, 0), _wp("SH_P3", 400, 0)]
    return _lane_proto("shared_lane", wps)


def _exclusive_lane_proto(operator_idx: int) -> scrp_pb2.LaneProto:
    """Lane offset along Y so it shares no segments with any other lane."""
    y = operator_idx * 1000.0
    prefix = f"OP{operator_idx}"
    wps = [_wp(f"{prefix}_P1", 0, y), _wp(f"{prefix}_P2", 200, y), _wp(f"{prefix}_P3", 400, y)]
    return _lane_proto(f"lane_op{operator_idx}", wps)


def _fi_proto(
    drone_id: str,
    operator_id: str,
    lane: scrp_pb2.LaneProto,
    *,
    t_des: float = 1000.0,
    priority: int = 1,
    uav_type: str = "A",
    v: float = 10.0,
    soc_0: float = 1.0,
    c_bat: float = 1000.0,
    p_hover: float = 500.0,
) -> scrp_pb2.OperatorFlightRequestProto:
    return scrp_pb2.OperatorFlightRequestProto(
        operator_id=operator_id,
        drone_id=drone_id,
        lane=lane,
        v_waypoints=[v] * len(lane.waypoints),
        destination_vertiport_id="vp1",
        t_des=t_des,
        priority=priority,
        uav_type=uav_type,
        soc_0=soc_0,
        c_bat=c_bat,
        p_hover=p_hover,
    )


# ---------------------------------------------------------------------------
# Domain-model helpers (for system-state construction)
# ---------------------------------------------------------------------------

def _build_system_state() -> SystemState:
    """Build initial SystemState with one committed plan on the shared lane."""
    # Vertiport with plenty of free slots
    pad = Pad(id="pad1", compatible_types=["A", "B", "C"])
    slots: Dict[Tuple[str, int], str | None] = {}
    slot_status: Dict[Tuple[str, int], str] = {}
    for i in range(300):
        slots[("pad1", i)] = None
        slot_status[("pad1", i)] = "FREE"
    vertiport = VertiportState(
        vertiport_id="vp1",
        pads=[pad],
        slot_duration=30.0,
        slots=slots,
        slot_status=slot_status,
    )

    # Pre-existing COMMITTED plan on the shared lane (drone_ex at t_dep=990)
    sh_wps = [
        Waypoint(id="SH_P1", position=(0.0, 0.0, 50.0)),
        Waypoint(id="SH_P2", position=(200.0, 0.0, 50.0)),
        Waypoint(id="SH_P3", position=(400.0, 0.0, 50.0)),
    ]
    sh_lane = Lane(id="shared_lane", waypoints=sh_wps, destination_vertiport_id="vp1")
    v_ex = 10.0
    existing_plan = ApprovedPlan(
        drone_id="drone_ex",
        uav_type="A",
        lane=sh_lane,
        v_waypoints=[v_ex] * len(sh_wps),
        t_land=990.0 + 40.0,  # 40 s flight time at 10 m/s over 400 m
        pad_id="pad1",
        slot_index=2,
        status="COMMITTED",
        t_dep=990.0,
        waypoint_times=[
            (sh_wps[0], 990.0),
            (sh_wps[1], 990.0 + 20.0),
            (sh_wps[2], 990.0 + 40.0),
        ],
        vertiport_id="vp1",
    )
    # Mark its slot as COMMITTED
    slot_status[("pad1", 2)] = "COMMITTED"
    slots[("pad1", 2)] = "drone_ex"

    return SystemState(
        approved_plans=[existing_plan],
        vertiports={"vp1": vertiport},
        t_now=900.0,
        msd_matrix={
            ("A", "A"): 50.0, ("A", "B"): 60.0, ("B", "A"): 60.0,
            ("B", "B"): 55.0, ("A", "C"): 65.0, ("C", "A"): 65.0,
            ("C", "C"): 70.0, ("B", "C"): 62.0, ("C", "B"): 62.0,
        },
        body_length={"A": 1.0, "B": 1.5, "C": 2.0},
    )


# ---------------------------------------------------------------------------
# Batched gRPC servicer
# ---------------------------------------------------------------------------

class BatchedSCRPServicer(scrp_pb2_grpc.SCRPServiceServicer):
    """
    gRPC servicer that feeds all incoming flight intentions into a BatchListener.

    Each ResolveConflict RPC call blocks until the batch fires and the result
    for that drone becomes available.  The default batch_interval is 10 s (as
    specified); tests may pass a shorter value to keep the suite fast.
    """

    def __init__(
        self,
        system_state: SystemState,
        config: SCRPConfig,
        batch_interval: float = 10.0,
    ) -> None:
        self._system_state = system_state
        self._events: Dict[str, threading.Event] = {}
        self._results: Dict[str, SCRPResult] = {}
        self._lock = threading.Lock()

        self.listener = BatchListener(
            system_state=system_state,
            config=config,
            on_result=self._on_batch_result,
            batch_interval=batch_interval,
        )
        self.listener.start()

    # -- internal callback invoked by BatchListener after each FI is resolved --

    def _on_batch_result(self, drone_id: str, result: SCRPResult) -> None:
        with self._lock:
            self._results[drone_id] = result
            ev = self._events.get(drone_id)
        if ev:
            ev.set()

    # -- gRPC handler --

    def ResolveConflict(
        self,
        request: scrp_pb2.ResolveConflictRequest,
        context: grpc.ServicerContext,
    ) -> scrp_pb2.ResolveConflictResponse:
        try:
            fi = _to_flight_intention(_operator_fi_from_proto(request.fi))

            # Set destination vertiport when operator did not specify one explicitly
            if not fi.lane.destination_vertiport_id and self._system_state.vertiports:
                fi.lane.destination_vertiport_id = next(iter(self._system_state.vertiports))

            event = threading.Event()
            with self._lock:
                self._events[fi.drone_id] = event

            print(
                f"  [SERVER] Queued   drone={fi.drone_id:<14s} "
                f"operator={fi.operator_id:<6s} priority={fi.priority}  "
                f"t_des={fi.t_des:.0f}s"
            )
            self.listener.submit(fi)

            # Block until the batch fires (up to 30 s)
            if not event.wait(timeout=30.0):
                context.set_code(grpc.StatusCode.DEADLINE_EXCEEDED)
                context.set_details(f"Batch timeout for {fi.drone_id}")
                return scrp_pb2.ResolveConflictResponse(
                    reject=scrp_pb2.RejectResultProto(
                        status="REJECTED",
                        reason="batch_timeout",
                        has_earliest_possible=False,
                        detail="No batch result within 30 s timeout",
                    )
                )

            with self._lock:
                result = self._results.pop(fi.drone_id, None)

            if isinstance(result, ApproveResult):
                return scrp_pb2.ResolveConflictResponse(approve=_approve_result_to_proto(result))
            return scrp_pb2.ResolveConflictResponse(reject=_reject_result_to_proto(result))

        except Exception as exc:  # noqa: BLE001
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(exc))
            return scrp_pb2.ResolveConflictResponse(
                reject=scrp_pb2.RejectResultProto(
                    status="REJECTED",
                    reason="invalid_fi",
                    has_earliest_possible=False,
                    detail=f"Internal error: {exc}",
                )
            )

    def stop(self) -> None:
        self.listener.stop()


# ---------------------------------------------------------------------------
# Result printer
# ---------------------------------------------------------------------------

def _print_batch_results(
    responses: Dict[str, scrp_pb2.ResolveConflictResponse],
    errors: Dict[str, str],
    elapsed: float,
) -> None:
    total = len(responses) + len(errors)
    approved = [d for d, r in responses.items() if r.HasField("approve")]
    rejected = [d for d, r in responses.items() if r.HasField("reject")]

    print()
    print("=" * 72)
    print(f"{'BATCH PROCESSING RESULTS':^72}")
    print("=" * 72)
    print(f"  Total processed  : {total}")
    print(f"  Approved         : {len(approved)}")
    print(f"  Rejected         : {len(rejected) + len(errors)}")
    print(f"  Elapsed (wall)   : {elapsed:.2f} s  (batch interval = 10 s)")
    print("-" * 72)

    for drone_id in sorted(responses):
        resp = responses[drone_id]
        if resp.HasField("approve"):
            a = resp.approve
            delay_tag = f"{a.delay_source}" if a.delay_seconds > 0 else "none"
            print(
                f"  [APPROVED]  {drone_id:<16s}"
                f"  t_dep* = {a.t_dep_star:>8.1f} s"
                f"  delay = {a.delay_seconds:>6.1f} s  ({delay_tag})"
                f"  SoC_rem = {a.soc_remaining:.4f}"
            )
        else:
            r = resp.reject
            ep = (
                f"  earliest = {r.earliest_possible:.1f} s"
                if r.has_earliest_possible
                else ""
            )
            print(f"  [REJECTED]  {drone_id:<16s}  reason = {r.reason}{ep}")

    for drone_id, err in sorted(errors.items()):
        print(f"  [ERROR   ]  {drone_id:<16s}  {err}")

    print("=" * 72)
    print()


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

BATCH_INTERVAL = 10.0  # seconds — matches the production default


@pytest.fixture(scope="module")
def batch_server():
    """
    Start an in-process gRPC server backed by a real BatchListener and return
    a (stub, servicer) pair.  The batch_interval is set to BATCH_INTERVAL so
    that requests submitted via the stub are held for ~10 s before processing.
    """
    state = _build_system_state()
    config = SCRPConfig()

    servicer = BatchedSCRPServicer(
        system_state=state,
        config=config,
        batch_interval=BATCH_INTERVAL,
    )

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    scrp_pb2_grpc.add_SCRPServiceServicer_to_server(servicer, server)
    port = server.add_insecure_port("127.0.0.1:0")
    server.start()

    channel = grpc.insecure_channel(f"127.0.0.1:{port}")
    stub = scrp_pb2_grpc.SCRPServiceStub(channel)

    yield stub, servicer

    channel.close()
    server.stop(grace=0)
    servicer.stop()


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestMultiOperatorBatch:
    """
    Simulate five operators submitting flight intentions simultaneously via gRPC.
    All requests are queued in the BatchListener and processed after ~10 seconds.
    """

    def test_all_operators_receive_results(self, batch_server):
        """
        All five operators get a definitive response (approve or reject) after
        the batch window closes.  No request is lost or times out.
        """
        stub, _ = batch_server

        operators = [
            # (drone_id, operator_id, lane_fn, priority, uav_type, t_des)
            ("drone_A1", "op-1", _shared_lane_proto(),     1, "A", 1000.0),
            ("drone_A2", "op-2", _shared_lane_proto(),     2, "A", 1000.0),
            ("drone_A3", "op-3", _shared_lane_proto(),     3, "A", 1000.0),
            ("drone_B1", "op-4", _exclusive_lane_proto(4), 1, "B", 1000.0),
            ("drone_C1", "op-5", _exclusive_lane_proto(5), 1, "C", 1000.0),
        ]

        responses: Dict[str, scrp_pb2.ResolveConflictResponse] = {}
        errors: Dict[str, str] = {}
        lock = threading.Lock()

        print()
        print("=" * 72)
        print(f"{'MULTI-OPERATOR BATCH gRPC TEST':^72}")
        print("=" * 72)
        print(f"  Operators      : {len(operators)}")
        print(f"  Batch interval : {BATCH_INTERVAL:.0f} seconds")
        print(f"  Scenario       : drone_A1/A2/A3 share the same lane")
        print(f"                   (existing committed plan at t_dep=990 s)")
        print(f"                   drone_B1/C1 fly independent exclusive lanes")
        print("-" * 72)

        def send(drone_id, operator_id, lane, priority, uav_type, t_des):
            fi = _fi_proto(
                drone_id=drone_id,
                operator_id=operator_id,
                lane=lane,
                t_des=t_des,
                priority=priority,
                uav_type=uav_type,
            )
            req = scrp_pb2.ResolveConflictRequest(fi=fi)
            try:
                resp = stub.ResolveConflict(req, timeout=35.0)
                with lock:
                    responses[drone_id] = resp
            except grpc.RpcError as exc:
                with lock:
                    errors[drone_id] = exc.details()

        t_start = time.time()

        # Launch all operator threads simultaneously
        threads = [
            threading.Thread(target=send, args=op, daemon=True)
            for op in operators
        ]
        for th in threads:
            th.start()

        print(f"\n  [t+0.00 s] {len(operators)} operators submitted simultaneously")
        print(f"  [waiting ~{BATCH_INTERVAL:.0f} s for the batch window to close ...]\n")

        for th in threads:
            th.join(timeout=35.0)

        elapsed = time.time() - t_start

        _print_batch_results(responses, errors, elapsed)

        # --- assertions ---
        assert not errors, f"RPC errors: {errors}"
        assert set(responses.keys()) == {op[0] for op in operators}

    def test_exclusive_lane_operators_approved_without_delay(self, batch_server):
        """
        Operators flying independent lanes with no shared segments are approved
        at their desired departure time with zero delay.
        """
        stub, _ = batch_server

        operators = [
            ("drone_excl_B", "op-excl-b", _exclusive_lane_proto(10), 1, "B", 2000.0),
            ("drone_excl_C", "op-excl-c", _exclusive_lane_proto(11), 1, "C", 2000.0),
        ]

        responses: Dict[str, scrp_pb2.ResolveConflictResponse] = {}
        errors: Dict[str, str] = {}
        lock = threading.Lock()

        def send(drone_id, operator_id, lane, priority, uav_type, t_des):
            fi = _fi_proto(
                drone_id=drone_id,
                operator_id=operator_id,
                lane=lane,
                t_des=t_des,
                priority=priority,
                uav_type=uav_type,
            )
            req = scrp_pb2.ResolveConflictRequest(fi=fi)
            try:
                resp = stub.ResolveConflict(req, timeout=35.0)
                with lock:
                    responses[drone_id] = resp
            except grpc.RpcError as exc:
                with lock:
                    errors[drone_id] = exc.details()

        threads = [threading.Thread(target=send, args=op, daemon=True) for op in operators]
        for th in threads:
            th.start()
        for th in threads:
            th.join(timeout=35.0)

        assert not errors, f"RPC errors: {errors}"
        for drone_id in ("drone_excl_B", "drone_excl_C"):
            assert responses[drone_id].HasField("approve"), (
                f"{drone_id} should be approved on an exclusive lane"
            )
            assert responses[drone_id].approve.delay_seconds == pytest.approx(0.0, abs=1e-3), (
                f"{drone_id} should have zero delay on an exclusive lane"
            )

    def test_priority_ordering_respected_in_batch(self, batch_server):
        """
        Within a batch, FIs are resolved in ascending priority order (1 = highest).
        The highest-priority shared-lane operator therefore encounters no headway
        conflict from lower-priority peers (they were not yet approved when it ran).
        """
        stub, _ = batch_server

        # Use a fresh t_des window far enough from earlier tests to avoid
        # slot collisions (t_des=3000 is 2000 s beyond the initial committed plan).
        operators = [
            ("drone_prio_low",  "op-pl", _shared_lane_proto(), 3, "A", 3000.0),
            ("drone_prio_high", "op-ph", _shared_lane_proto(), 1, "A", 3000.0),
            ("drone_prio_mid",  "op-pm", _shared_lane_proto(), 2, "A", 3000.0),
        ]

        responses: Dict[str, scrp_pb2.ResolveConflictResponse] = {}
        errors: Dict[str, str] = {}
        lock = threading.Lock()

        def send(drone_id, operator_id, lane, priority, uav_type, t_des):
            fi = _fi_proto(
                drone_id=drone_id,
                operator_id=operator_id,
                lane=lane,
                t_des=t_des,
                priority=priority,
                uav_type=uav_type,
            )
            req = scrp_pb2.ResolveConflictRequest(fi=fi)
            try:
                resp = stub.ResolveConflict(req, timeout=35.0)
                with lock:
                    responses[drone_id] = resp
            except grpc.RpcError as exc:
                with lock:
                    errors[drone_id] = exc.details()

        threads = [threading.Thread(target=send, args=op, daemon=True) for op in operators]
        for th in threads:
            th.start()
        for th in threads:
            th.join(timeout=35.0)

        assert not errors, f"RPC errors: {errors}"
        # Priority-1 operator has no peers ahead of it in the batch, so no
        # additional intra-batch delay beyond the existing committed plan.
        assert responses["drone_prio_high"].HasField("approve"), (
            "Highest-priority operator must be approved"
        )
        # All operators must receive a definitive result.
        for drone_id, *_ in operators:
            assert drone_id in responses, f"No result for {drone_id}"
