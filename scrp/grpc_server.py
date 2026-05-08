"""gRPC servicer for the SCRP algorithm.

Converts proto messages to/from Python domain models and delegates to
the pure resolve_conflict() function.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import grpc

from scrp.proto import scrp_pb2, scrp_pb2_grpc
from .models import (
    ApprovedPlan,
    ApproveResult,
    DroneSpec,
    FlightIntention,
    Lane,
    OperatorFlightRequest,
    Pad,
    RejectResult,
    SCRPConfig,
    Segment,
    SystemState,
    VertiportState,
    Waypoint,
)
from .scrp import resolve_conflict


# ---------------------------------------------------------------------------
# Proto → Python helpers
# ---------------------------------------------------------------------------

def _wp_from_proto(p: scrp_pb2.WaypointProto) -> Waypoint:
    return Waypoint(id=p.id, position=(p.x, p.y, p.z))


def _seg_from_proto(p: scrp_pb2.SegmentProto) -> Segment:
    return Segment(
        p_start=_wp_from_proto(p.p_start),
        p_end=_wp_from_proto(p.p_end),
        length=p.length,
        v_min=p.v_min,
        v_max=p.v_max,
    )


def _lane_from_proto(p: scrp_pb2.LaneProto) -> Lane:
    waypoints = [_wp_from_proto(w) for w in p.waypoints]
    segments = [_seg_from_proto(s) for s in p.segments]
    lane = Lane(id=p.id, waypoints=waypoints, segments=segments)
    return lane


def _operator_fi_from_proto(p: scrp_pb2.OperatorFlightRequestProto) -> OperatorFlightRequest:
    lane = _lane_from_proto(p.lane)
    lane.destination_vertiport_id = p.destination_vertiport_id
    return OperatorFlightRequest(
        operator_id=p.operator_id,
        drone_id=p.drone_id,
        lane=lane,
        v_waypoints=list(p.v_waypoints),
        destination_vertiport_id=p.destination_vertiport_id,
        t_des=p.t_des,
        priority=p.priority,
    )


def _drone_spec_from_proto(p: scrp_pb2.DroneSpecProto) -> DroneSpec:
    return DroneSpec(
        drone_id=p.drone_id,
        uav_type=p.uav_type,
        SoC_0=p.soc_0,
        C_bat=p.c_bat,
        P_hover=p.p_hover,
        t_takeoff=p.t_takeoff if p.t_takeoff != 0.0 else None,
        t_land_estimated=p.t_land_estimated if p.t_land_estimated != 0.0 else None,
    )


def _merge_to_flight_intention(
    fi_req: OperatorFlightRequest,
    drone_spec: DroneSpec,
) -> FlightIntention:
    return FlightIntention(
        drone_id=fi_req.drone_id,
        uav_type=drone_spec.uav_type,
        lane=fi_req.lane,
        v_waypoints=fi_req.v_waypoints,
        t_des=fi_req.t_des,
        SoC_0=drone_spec.SoC_0,
        C_bat=drone_spec.C_bat,
        P_hover=drone_spec.P_hover,
        priority=fi_req.priority,
        operator_id=fi_req.operator_id,
        t_takeoff=drone_spec.t_takeoff,
        t_land_estimated=drone_spec.t_land_estimated,
    )


def _approved_plan_from_proto(p: scrp_pb2.ApprovedPlanProto) -> ApprovedPlan:
    wpt: List[Tuple[Waypoint, float]] = [
        (_wp_from_proto(e.waypoint), e.time) for e in p.waypoint_times
    ]
    return ApprovedPlan(
        drone_id=p.drone_id,
        uav_type=p.uav_type,
        lane=_lane_from_proto(p.lane),
        v_waypoints=list(p.v_waypoints),
        t_land=p.t_land,
        pad_id=p.pad_id,
        slot_index=p.slot_index,
        status=p.status,
        t_dep=p.t_dep if p.t_dep != 0.0 else None,
        waypoint_times=wpt,
        algorithm=p.algorithm or 'SCRP',
        expires_at=p.expires_at if p.expires_at != 0.0 else None,
    )


def _vertiport_from_proto(p: scrp_pb2.VertiportStateProto) -> VertiportState:
    pads = [Pad(id=pad.id, compatible_types=list(pad.compatible_types)) for pad in p.pads]
    slots: Dict[Tuple[str, int], Optional[str]] = {}
    slot_status: Dict[Tuple[str, int], str] = {}
    for entry in p.slots:
        key = (entry.pad_id, entry.slot_index)
        slots[key] = entry.drone_id if entry.drone_id else None
        slot_status[key] = entry.status
    return VertiportState(
        vertiport_id=p.vertiport_id,
        pads=pads,
        slot_duration=p.slot_duration,
        slots=slots,
        slot_status=slot_status,
    )


def _system_state_from_proto(p: scrp_pb2.SystemStateProto) -> SystemState:
    approved_plans = [_approved_plan_from_proto(ap) for ap in p.approved_plans]
    vertiport_state = _vertiport_from_proto(p.vertiport_state)
    msd_matrix: Dict[Tuple[str, str], float] = {
        (e.uav_type_i, e.uav_type_j): e.msd for e in p.msd_matrix
    }
    body_length: Dict[str, float] = {e.uav_type: e.length_m for e in p.body_length}
    vertiports = (
        {vertiport_state.vertiport_id: vertiport_state}
        if vertiport_state.vertiport_id
        else {}
    )
    return SystemState(
        approved_plans=approved_plans,
        vertiports=vertiports,
        t_now=p.t_now,
        msd_matrix=msd_matrix,
        body_length=body_length,
    )


def _config_from_proto(p: scrp_pb2.SCRPConfigProto) -> SCRPConfig:
    cfg = SCRPConfig()
    if p.soc_min != 0.0:
        cfg.SOC_MIN = p.soc_min
    if p.slot_duration_sec != 0.0:
        cfg.SLOT_DURATION_SEC = p.slot_duration_sec
    if p.t_response_window_sec != 0.0:
        cfg.T_RESPONSE_WINDOW_SEC = p.t_response_window_sec
    if p.junction_diameter_m != 0.0:
        cfg.JUNCTION_DIAMETER_M = p.junction_diameter_m
    if p.default_timeout_behavior:
        cfg.DEFAULT_TIMEOUT_BEHAVIOR = p.default_timeout_behavior
    if p.cruise_power_factor != 0.0:
        cfg.CRUISE_POWER_FACTOR = p.cruise_power_factor
    if p.max_acceptable_delay_sec != 0.0:
        cfg.MAX_ACCEPTABLE_DELAY_SEC = p.max_acceptable_delay_sec
    return cfg


# ---------------------------------------------------------------------------
# Python → Proto helpers
# ---------------------------------------------------------------------------

def _approve_result_to_proto(r: ApproveResult) -> scrp_pb2.ApproveResultProto:
    wpt_times = [
        scrp_pb2.WaypointTimeResultProto(waypoint_id=wp_id, time=t)
        for wp_id, t in r.waypoint_times
    ]
    return scrp_pb2.ApproveResultProto(
        status=r.status,
        t_dep_star=r.t_dep_star,
        t_land_assigned=r.t_land_assigned,
        pad_id=r.pad_id,
        slot_index=r.slot_index,
        waypoint_times=wpt_times,
        expires_at=r.expires_at,
        delay_seconds=r.delay_seconds,
        delay_source=r.delay_source,
        energy_estimate=r.energy_estimate,
        soc_remaining=r.SoC_remaining,
    )


def _reject_result_to_proto(r: RejectResult) -> scrp_pb2.RejectResultProto:
    has_ep = r.earliest_possible is not None
    return scrp_pb2.RejectResultProto(
        status=r.status,
        reason=r.reason,
        earliest_possible=r.earliest_possible if has_ep else 0.0,
        has_earliest_possible=has_ep,
        detail=r.detail,
    )


# ---------------------------------------------------------------------------
# Servicer
# ---------------------------------------------------------------------------

class SCRPServicer(scrp_pb2_grpc.SCRPServiceServicer):
    """gRPC servicer that wraps the pure resolve_conflict() function."""

    def ResolveConflict(
        self,
        request: scrp_pb2.ResolveConflictRequest,
        context: grpc.ServicerContext,
    ) -> scrp_pb2.ResolveConflictResponse:
        try:
            fi_req = _operator_fi_from_proto(request.fi)
            drone_spec = _drone_spec_from_proto(request.drone_spec)
            fi = _merge_to_flight_intention(fi_req, drone_spec)
            system_state = _system_state_from_proto(request.system_state)
            config = _config_from_proto(request.config)

            # Infer destination vertiport when operator did not specify one explicitly
            if not fi.lane.destination_vertiport_id and system_state.vertiports:
                fi.lane.destination_vertiport_id = next(iter(system_state.vertiports))

            result = resolve_conflict(fi, system_state.approved_plans, system_state, config)

            if isinstance(result, ApproveResult):
                return scrp_pb2.ResolveConflictResponse(
                    approve=_approve_result_to_proto(result)
                )
            else:
                return scrp_pb2.ResolveConflictResponse(
                    reject=_reject_result_to_proto(result)
                )
        except Exception as exc:  # noqa: BLE001
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(exc))
            return scrp_pb2.ResolveConflictResponse(
                reject=scrp_pb2.RejectResultProto(
                    status='REJECTED',
                    reason='invalid_fi',
                    has_earliest_possible=False,
                    detail=f'Internal error: {exc}',
                )
            )
