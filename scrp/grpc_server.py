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
    DroneProfile,
    FlightIntention,
    FlightRoute,
    Lane,
    OperatorFlightRequest,
    Pad,
    RejectResult,
    RouteRejection,
    RouteSuggestionRequest,
    RouteSuggestionResult,
    SCRPConfig,
    Segment,
    SystemState,
    VertiportState,
    Waypoint,
)
from .scrp import resolve_conflict
from .route_suggestion import suggest_routes


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
        uav_type=p.uav_type,
        SoC_0=p.soc_0,
        C_bat=p.c_bat,
        P_hover=p.p_hover,
        t_takeoff=p.t_takeoff if p.t_takeoff != 0.0 else None,
        t_land_estimated=p.t_land_estimated if p.t_land_estimated != 0.0 else None,
    )


def _to_flight_intention(req: OperatorFlightRequest) -> FlightIntention:
    return FlightIntention(
        drone_id=req.drone_id,
        uav_type=req.uav_type,
        lane=req.lane,
        v_waypoints=req.v_waypoints,
        t_des=req.t_des,
        SoC_0=req.SoC_0,
        C_bat=req.C_bat,
        P_hover=req.P_hover,
        priority=req.priority,
        operator_id=req.operator_id,
        t_takeoff=req.t_takeoff,
        t_land_estimated=req.t_land_estimated,
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


def _drone_profile_from_proto(p: scrp_pb2.DroneProfileProto) -> DroneProfile:
    return DroneProfile(
        weight_kg=p.weight_kg,
        max_wind_resistance=p.max_wind_resistance,
        drone_size=p.drone_size,
    )


def _flight_route_from_proto(p: scrp_pb2.FlightRouteProto) -> FlightRoute:
    waypoints = [_wp_from_proto(w) for w in p.waypoints]
    segments = [_seg_from_proto(s) for s in p.segments]
    route = FlightRoute(
        route_id=p.route_id,
        waypoints=waypoints,
        take_off_vertiport_id=p.take_off_vertiport_id,
        landing_vertiport_id=p.landing_vertiport_id,
        safety_score=p.safety_score,
        average_wind_speed=p.average_wind_speed,
        max_drone_weight_kg=p.max_drone_weight_kg,
        compatible_drone_sizes=list(p.compatible_drone_sizes),
        segments=segments,
    )
    return route


def _flight_route_to_proto(r: FlightRoute) -> scrp_pb2.FlightRouteProto:
    wps = [
        scrp_pb2.WaypointProto(id=w.id, x=w.position[0], y=w.position[1], z=w.position[2])
        for w in r.waypoints
    ]
    segs = [
        scrp_pb2.SegmentProto(
            p_start=scrp_pb2.WaypointProto(
                id=s.p_start.id,
                x=s.p_start.position[0],
                y=s.p_start.position[1],
                z=s.p_start.position[2],
            ),
            p_end=scrp_pb2.WaypointProto(
                id=s.p_end.id,
                x=s.p_end.position[0],
                y=s.p_end.position[1],
                z=s.p_end.position[2],
            ),
            length=s.length,
            v_min=s.v_min,
            v_max=s.v_max,
        )
        for s in r.segments
    ]
    return scrp_pb2.FlightRouteProto(
        route_id=r.route_id,
        waypoints=wps,
        segments=segs,
        take_off_vertiport_id=r.take_off_vertiport_id,
        landing_vertiport_id=r.landing_vertiport_id,
        safety_score=r.safety_score,
        average_wind_speed=r.average_wind_speed,
        max_drone_weight_kg=r.max_drone_weight_kg,
        compatible_drone_sizes=r.compatible_drone_sizes,
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
            fi = _to_flight_intention(_operator_fi_from_proto(request.fi))
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

    def SuggestRoutes(
        self,
        request: scrp_pb2.RouteSuggestionRequest,
        context: grpc.ServicerContext,
    ) -> scrp_pb2.RouteSuggestionResponse:
        try:
            drone = _drone_profile_from_proto(request.drone)
            available_routes = [_flight_route_from_proto(r) for r in request.available_routes]

            suggestion_request = RouteSuggestionRequest(
                take_off_vertiport_id=request.take_off_vertiport_id,
                landing_vertiport_id=request.landing_vertiport_id,
                drone=drone,
                min_safety_score=request.min_safety_score,
            )

            result = suggest_routes(suggestion_request, available_routes)

            compatible_protos = [_flight_route_to_proto(r) for r in result.compatible_routes]
            rejected_protos = [
                scrp_pb2.RouteRejectionProto(
                    route=_flight_route_to_proto(rj.route),
                    reason=rj.reason,
                )
                for rj in result.rejected_routes
            ]
            return scrp_pb2.RouteSuggestionResponse(
                compatible_routes=compatible_protos,
                rejected_routes=rejected_protos,
            )
        except Exception as exc:  # noqa: BLE001
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(exc))
            return scrp_pb2.RouteSuggestionResponse()
