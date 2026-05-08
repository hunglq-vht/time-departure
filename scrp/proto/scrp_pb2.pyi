from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf.internal import containers as _containers
from typing import Optional, Iterable

DESCRIPTOR: _descriptor.FileDescriptor

class WaypointProto(_message.Message):
    id: str
    x: float
    y: float
    z: float
    def __init__(
        self,
        *,
        id: str = ...,
        x: float = ...,
        y: float = ...,
        z: float = ...,
    ) -> None: ...

class SegmentProto(_message.Message):
    p_start: WaypointProto
    p_end: WaypointProto
    length: float
    v_min: float
    v_max: float
    def __init__(
        self,
        *,
        p_start: Optional[WaypointProto] = ...,
        p_end: Optional[WaypointProto] = ...,
        length: float = ...,
        v_min: float = ...,
        v_max: float = ...,
    ) -> None: ...

class LaneProto(_message.Message):
    id: str
    waypoints: _containers.RepeatedCompositeFieldContainer[WaypointProto]
    segments: _containers.RepeatedCompositeFieldContainer[SegmentProto]
    def __init__(
        self,
        *,
        id: str = ...,
        waypoints: Optional[Iterable[WaypointProto]] = ...,
        segments: Optional[Iterable[SegmentProto]] = ...,
    ) -> None: ...

class FlightIntentionProto(_message.Message):
    drone_id: str
    uav_type: str
    lane: LaneProto
    v_waypoints: _containers.RepeatedScalarFieldContainer[float]
    t_des: float
    soc_0: float
    c_bat: float
    p_hover: float
    priority: int
    operator_id: str
    t_takeoff: float
    t_land_estimated: float
    def __init__(
        self,
        *,
        drone_id: str = ...,
        uav_type: str = ...,
        lane: Optional[LaneProto] = ...,
        v_waypoints: Optional[Iterable[float]] = ...,
        t_des: float = ...,
        soc_0: float = ...,
        c_bat: float = ...,
        p_hover: float = ...,
        priority: int = ...,
        operator_id: str = ...,
        t_takeoff: float = ...,
        t_land_estimated: float = ...,
    ) -> None: ...

class WaypointTimeEntryProto(_message.Message):
    waypoint: WaypointProto
    time: float
    def __init__(
        self,
        *,
        waypoint: Optional[WaypointProto] = ...,
        time: float = ...,
    ) -> None: ...

class ApprovedPlanProto(_message.Message):
    drone_id: str
    uav_type: str
    lane: LaneProto
    v_waypoints: _containers.RepeatedScalarFieldContainer[float]
    t_land: float
    pad_id: str
    slot_index: int
    status: str
    t_dep: float
    waypoint_times: _containers.RepeatedCompositeFieldContainer[WaypointTimeEntryProto]
    algorithm: str
    expires_at: float
    def __init__(
        self,
        *,
        drone_id: str = ...,
        uav_type: str = ...,
        lane: Optional[LaneProto] = ...,
        v_waypoints: Optional[Iterable[float]] = ...,
        t_land: float = ...,
        pad_id: str = ...,
        slot_index: int = ...,
        status: str = ...,
        t_dep: float = ...,
        waypoint_times: Optional[Iterable[WaypointTimeEntryProto]] = ...,
        algorithm: str = ...,
        expires_at: float = ...,
    ) -> None: ...

class PadProto(_message.Message):
    id: str
    compatible_types: _containers.RepeatedScalarFieldContainer[str]
    def __init__(
        self,
        *,
        id: str = ...,
        compatible_types: Optional[Iterable[str]] = ...,
    ) -> None: ...

class SlotEntryProto(_message.Message):
    pad_id: str
    slot_index: int
    drone_id: str
    status: str
    def __init__(
        self,
        *,
        pad_id: str = ...,
        slot_index: int = ...,
        drone_id: str = ...,
        status: str = ...,
    ) -> None: ...

class VertiportStateProto(_message.Message):
    vertiport_id: str
    pads: _containers.RepeatedCompositeFieldContainer[PadProto]
    slot_duration: float
    slots: _containers.RepeatedCompositeFieldContainer[SlotEntryProto]
    def __init__(
        self,
        *,
        vertiport_id: str = ...,
        pads: Optional[Iterable[PadProto]] = ...,
        slot_duration: float = ...,
        slots: Optional[Iterable[SlotEntryProto]] = ...,
    ) -> None: ...

class MsdEntryProto(_message.Message):
    uav_type_i: str
    uav_type_j: str
    msd: float
    def __init__(
        self,
        *,
        uav_type_i: str = ...,
        uav_type_j: str = ...,
        msd: float = ...,
    ) -> None: ...

class BodyLengthEntryProto(_message.Message):
    uav_type: str
    length_m: float
    def __init__(
        self,
        *,
        uav_type: str = ...,
        length_m: float = ...,
    ) -> None: ...

class SystemStateProto(_message.Message):
    approved_plans: _containers.RepeatedCompositeFieldContainer[ApprovedPlanProto]
    vertiport_state: VertiportStateProto
    t_now: float
    msd_matrix: _containers.RepeatedCompositeFieldContainer[MsdEntryProto]
    body_length: _containers.RepeatedCompositeFieldContainer[BodyLengthEntryProto]
    def __init__(
        self,
        *,
        approved_plans: Optional[Iterable[ApprovedPlanProto]] = ...,
        vertiport_state: Optional[VertiportStateProto] = ...,
        t_now: float = ...,
        msd_matrix: Optional[Iterable[MsdEntryProto]] = ...,
        body_length: Optional[Iterable[BodyLengthEntryProto]] = ...,
    ) -> None: ...

class SCRPConfigProto(_message.Message):
    soc_min: float
    slot_duration_sec: float
    t_response_window_sec: float
    junction_diameter_m: float
    default_timeout_behavior: str
    cruise_power_factor: float
    max_acceptable_delay_sec: float
    def __init__(
        self,
        *,
        soc_min: float = ...,
        slot_duration_sec: float = ...,
        t_response_window_sec: float = ...,
        junction_diameter_m: float = ...,
        default_timeout_behavior: str = ...,
        cruise_power_factor: float = ...,
        max_acceptable_delay_sec: float = ...,
    ) -> None: ...

class WaypointTimeResultProto(_message.Message):
    waypoint_id: str
    time: float
    def __init__(
        self,
        *,
        waypoint_id: str = ...,
        time: float = ...,
    ) -> None: ...

class ApproveResultProto(_message.Message):
    status: str
    t_dep_star: float
    t_land_assigned: float
    pad_id: str
    slot_index: int
    waypoint_times: _containers.RepeatedCompositeFieldContainer[WaypointTimeResultProto]
    expires_at: float
    delay_seconds: float
    delay_source: str
    energy_estimate: float
    soc_remaining: float
    def __init__(
        self,
        *,
        status: str = ...,
        t_dep_star: float = ...,
        t_land_assigned: float = ...,
        pad_id: str = ...,
        slot_index: int = ...,
        waypoint_times: Optional[Iterable[WaypointTimeResultProto]] = ...,
        expires_at: float = ...,
        delay_seconds: float = ...,
        delay_source: str = ...,
        energy_estimate: float = ...,
        soc_remaining: float = ...,
    ) -> None: ...

class RejectResultProto(_message.Message):
    status: str
    reason: str
    earliest_possible: float
    has_earliest_possible: bool
    detail: str
    def __init__(
        self,
        *,
        status: str = ...,
        reason: str = ...,
        earliest_possible: float = ...,
        has_earliest_possible: bool = ...,
        detail: str = ...,
    ) -> None: ...

class ResolveConflictRequest(_message.Message):
    fi: FlightIntentionProto
    approved_plans: _containers.RepeatedCompositeFieldContainer[ApprovedPlanProto]
    vertiport_state: VertiportStateProto
    system_state: SystemStateProto
    config: SCRPConfigProto
    def __init__(
        self,
        *,
        fi: Optional[FlightIntentionProto] = ...,
        approved_plans: Optional[Iterable[ApprovedPlanProto]] = ...,
        vertiport_state: Optional[VertiportStateProto] = ...,
        system_state: Optional[SystemStateProto] = ...,
        config: Optional[SCRPConfigProto] = ...,
    ) -> None: ...

class ResolveConflictResponse(_message.Message):
    approve: ApproveResultProto
    reject: RejectResultProto
    def __init__(
        self,
        *,
        approve: Optional[ApproveResultProto] = ...,
        reject: Optional[RejectResultProto] = ...,
    ) -> None: ...
    def HasField(self, field_name: str) -> bool: ...
    def WhichOneof(self, oneof_group: str) -> Optional[str]: ...
