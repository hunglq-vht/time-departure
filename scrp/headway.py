"""Headway constraint (C1) computation."""
from __future__ import annotations

from typing import List

from .geometry import common_waypoints, plan_waypoint_times_dict, t_arrival_at_waypoint
from .models import ApprovedPlan, FlightIntention, Segment, SystemState


def h_min_full(
    msd: float,
    body_len_j: float,
    v_i: float,
    v_j: float,
    seg_length: float,
    v_min_seg: float,
) -> float:
    """Full headway requirement accounting for velocity difference on a segment.

    h_min_full = (MSD + L_body_j) / v_fast
                 + |v_i - v_j| * L_k / v_fast^2
    """
    v_fast = max(v_i, v_j)
    base = (msd + body_len_j) / v_fast
    correction = abs(v_i - v_j) * seg_length / (v_fast * v_fast)
    return base + correction


def compute_delta_C1(
    fi: FlightIntention,
    approved_plans: List[ApprovedPlan],
    t_des: float,
    state: SystemState,
) -> float:
    """Return maximum additional delay needed to satisfy headway on all shared segments."""
    delta = 0.0

    for plan in approved_plans:
        shared = common_waypoints(fi.lane, plan.lane)
        if not shared:
            continue

        # Build lookup: waypoint_id -> arrival time (computed lazily if waypoint_times absent)
        plan_times = plan_waypoint_times_dict(plan)

        for idx_new, idx_plan in shared:
            # Only consider segments (not the last waypoint which has no outgoing segment)
            if idx_new >= len(fi.lane.segments):
                continue
            if idx_plan >= len(plan.lane.segments):
                continue

            wp_id = fi.lane.waypoints[idx_new].id
            tau_k = plan_times.get(wp_id)
            if tau_k is None:
                continue

            t_arr_new = t_arrival_at_waypoint(fi, idx_new, t_des)

            # Only constrain when drone new arrives after the existing plan
            if t_arr_new < tau_k:
                continue

            seg = fi.lane.segments[idx_new]
            v_i = fi.v_waypoints[idx_new]

            # Use stored velocity from ApprovedPlan when available; fall back to
            # deriving from consecutive waypoint_times for legacy plans without v_waypoints.
            if plan.v_waypoints and idx_plan < len(plan.v_waypoints):
                v_j = plan.v_waypoints[idx_plan]
            else:
                plan_seg = plan.lane.segments[idx_plan]
                t_next_wp_id = plan.lane.waypoints[idx_plan + 1].id
                t_next = plan_times.get(t_next_wp_id)
                if t_next is not None and t_next > tau_k:
                    v_j = plan_seg.length / (t_next - tau_k)
                else:
                    v_j = plan_seg.v_min

            msd = state.msd_matrix.get((fi.uav_type, plan.uav_type), 50.0)
            body_j = state.body_length.get(plan.uav_type, 1.0)

            h = h_min_full(
                msd=msd,
                body_len_j=body_j,
                v_i=v_i,
                v_j=v_j,
                seg_length=seg.length,
                v_min_seg=seg.v_min,
            )

            needed = tau_k + h - t_arr_new
            if needed > delta:
                delta = needed

    return max(0.0, delta)
