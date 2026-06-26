"""
Mixed-Integer Programming conflict resolver for UAM flight scheduling.

Solves the same four-constraint problem as ConflictResolver (resolver.py)
but formulates it as a single MIP and hands it to CBC (via PuLP):

  Minimise  d  (departure delay, continuous ≥ 0)

  Subject to:
    C1 – Lane separation
         For each approved plan j sharing segment s with the new request,
         binary variable x_{j,s} ∈ {0,1} selects the safe ordering:
           x=0  →  new drone leads j  (exit + lead-headway ≤ j's entry)
           x=1  →  new drone follows j (new entry ≥ j's entry + follow-headway)

    C2 – Takeoff airspace exclusion at departure vertiport
         For each existing airspace window k at the departure vertiport,
         binary a_k ∈ {0,1} selects:
           a=0  →  new takeoff window ends before window k starts
           a=1  →  new takeoff window starts after window k ends

    C3 – Landing airspace exclusion at arrival vertiport
         Identical structure to C2 but for the descent window.
         Binary b_k per existing arrival-airspace window.

    C4 – Pad occupancy
         Binary u_p selects exactly one pad at the landing vertiport.
         For each pad p and each occupation window k on that pad,
         binary c_{p,k} selects:
           c=0  →  new touchdown arrives before window k starts
           c=1  →  new touchdown arrives after window k ends

Big-M value: 86 400 s (one full day) – large enough to never bind on
real scenarios but small enough to avoid numerical issues with CBC.

The MIP reduces to the same optimal delay d* as the fixed-point algorithm
because both find the minimum feasible departure delay satisfying all
four constraints exactly.  The MIP formulation is intentionally verbose
(one binary per window per constraint) to make the comparison fair and
educational.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import pulp

from .models import (
    ApprovedPlan,
    NewPlanRequest,
    Pad,
    Point3D,
    ResolveResult,
    Vertiport,
)

_BIG_M = 86_400.0  # 24 h in seconds
_EPS = 1e-4        # small value for strict-inequality linearisation


def _h_min(msd: float, v_i: float, v_j: float, seg_len: float) -> float:
    """Minimum time headway: follower v_i behind lead v_j on segment of length seg_len."""
    base = msd / v_j
    correction = max(0.0, v_i - v_j) * seg_len / (v_i * v_j) if v_i > v_j else 0.0
    return base + correction


class MIPConflictResolver:
    """Resolve a single new-plan request via Mixed-Integer Programming.

    Interface mirrors ConflictResolver.resolve() so both can be called
    interchangeably in benchmarks.
    """

    MIN_SEPARATION_M: float = 50.0
    POINT_TOL_M: float = 1.0

    def __init__(
        self,
        vertiports: Dict[str, Vertiport],
        approved_plans: List[ApprovedPlan],
        solver_msg: bool = False,
    ) -> None:
        self.vertiports = vertiports
        self.approved_plans = approved_plans
        self._solver_msg = solver_msg

    # ──────────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────────

    def resolve(self, request: NewPlanRequest) -> ResolveResult:
        """Return the minimum-delay approval (or rejection) for *request*."""
        prob = pulp.LpProblem("UAM_conflict_resolution", pulp.LpMinimize)

        # Primary decision variable
        d = pulp.LpVariable("d", lowBound=0.0, cat="Continuous")
        prob += d  # objective: minimise delay

        # Convenience: touchdown time expression (linear in d)
        lane_time = self._request_lane_time(request)
        t_des = request.desired_start_time
        # t_dep    = t_des + d
        # t_wpN    = t_des + d + t_takeoff + lane_time
        # t_touch  = t_des + d + t_takeoff + lane_time + t_land
        t_takeoff_offset = request.t_takeoff
        t_land_offset = request.t_land

        # ── C1: lane separation ───────────────────────────────────────────────
        new_wps = list(request.flight_path.waypoints)
        if len(new_wps) >= 2:
            for appr in self.approved_plans:
                appr_wps = list(appr.flight_path.waypoints)
                if len(appr_wps) < 2:
                    continue
                for ni in range(len(new_wps) - 1):
                    for ai in range(len(appr_wps) - 1):
                        if not self._same_segment(
                            new_wps[ni], new_wps[ni + 1],
                            appr_wps[ai], appr_wps[ai + 1],
                        ):
                            continue

                        seg_len = new_wps[ni].distance_to(new_wps[ni + 1])
                        v_new = request.segment_speeds[ni]
                        v_appr = appr.segment_speeds[ai]

                        offset_new = t_takeoff_offset + sum(
                            new_wps[k].distance_to(new_wps[k + 1]) / request.segment_speeds[k]
                            for k in range(ni)
                        )
                        offset_appr = appr.t_takeoff + sum(
                            appr_wps[k].distance_to(appr_wps[k + 1]) / appr.segment_speeds[k]
                            for k in range(ai)
                        )

                        t_j_entry = appr.start_time + offset_appr  # constant
                        # t_new_entry = t_des + d + offset_new
                        # t_new_exit  = t_des + d + offset_new + seg_len/v_new

                        h_follow = _h_min(self.MIN_SEPARATION_M, v_new, v_appr, seg_len)
                        h_lead   = _h_min(self.MIN_SEPARATION_M, v_appr, v_new, seg_len)

                        # Threshold delays (may be negative → constraint trivially satisfied)
                        # New follows j: t_des + d + offset_new ≥ t_j_entry + h_follow
                        d_follow = t_j_entry + h_follow - offset_new - t_des
                        # New leads j:  t_des + d + offset_new + seg_len/v_new + h_lead ≤ t_j_entry
                        d_lead = t_j_entry - offset_new - seg_len / v_new - h_lead - t_des

                        x = pulp.LpVariable(
                            f"x_{appr.plan_id}_s{ni}_a{ai}", cat="Binary"
                        )
                        # x=1 → new follows:  d ≥ d_follow   (enforced, other relaxed)
                        # x=0 → new leads:    d ≤ d_lead      (enforced, other relaxed)
                        prob += d >= d_follow - _BIG_M * (1 - x)
                        prob += d <= d_lead   + _BIG_M * x

        # ── C2: takeoff airspace at departure vertiport ───────────────────────
        dep_id = request.flight_path.takeoff_vertiport_id
        dep_windows = self._all_airspace_windows(dep_id)
        for idx, (s, e) in enumerate(dep_windows):
            a = pulp.LpVariable(f"a_{idx}", cat="Binary")
            # a=0: new takeoff window ends before s  →  t_des + d + t_takeoff ≤ s
            prob += t_des + d + t_takeoff_offset <= s + _BIG_M * a
            # a=1: new takeoff window starts after e →  t_des + d ≥ e
            prob += t_des + d >= e - _BIG_M * (1 - a)

        # ── C3: landing airspace at arrival vertiport ─────────────────────────
        arr_id = request.flight_path.landing_vertiport_id
        arr_windows = self._all_airspace_windows(arr_id)
        for idx, (s, e) in enumerate(arr_windows):
            b = pulp.LpVariable(f"b_{idx}", cat="Binary")
            # b=0: landing window ends before s  →  t_des + d + t_takeoff + lane + t_land ≤ s
            prob += t_des + d + t_takeoff_offset + lane_time + t_land_offset <= s + _BIG_M * b
            # b=1: landing window starts after e →  t_des + d + t_takeoff + lane ≥ e
            prob += t_des + d + t_takeoff_offset + lane_time >= e - _BIG_M * (1 - b)

        # ── C4: pad occupancy ─────────────────────────────────────────────────
        vertiport = self.vertiports[arr_id]
        pads = vertiport.pads
        u_vars = []
        for pad in pads:
            u = pulp.LpVariable(f"u_{pad.id}", cat="Binary")
            u_vars.append(u)

            occ_windows = self._pad_occupation_windows(arr_id, pad.id, pad)
            for kidx, (s, e) in enumerate(occ_windows):
                c = pulp.LpVariable(f"c_{pad.id}_{kidx}", cat="Binary")
                # c=0: touchdown before window  →  t_touch ≤ s (- eps for strict <)
                # Only enforced when this pad is selected (u=1)
                prob += (
                    t_des + d + t_takeoff_offset + lane_time + t_land_offset
                    <= s - _EPS + _BIG_M * c + _BIG_M * (1 - u)
                )
                # c=1: touchdown after window   →  t_touch ≥ e
                prob += (
                    t_des + d + t_takeoff_offset + lane_time + t_land_offset
                    >= e - _BIG_M * (1 - c) - _BIG_M * (1 - u)
                )

        # Exactly one pad selected
        if u_vars:
            prob += pulp.lpSum(u_vars) == 1
        else:
            return ResolveResult(
                approved=False,
                actual_start_time=None,
                delay=0.0,
                reason=f"No pads configured at vertiport {arr_id!r}",
            )

        # ── Solve ─────────────────────────────────────────────────────────────
        solver = pulp.PULP_CBC_CMD(msg=self._solver_msg)
        status = prob.solve(solver)

        if status != pulp.LpStatusOptimal:
            return ResolveResult(
                approved=False,
                actual_start_time=None,
                delay=float("inf"),
                reason=f"MIP infeasible or unbounded (status={pulp.LpStatus[status]})",
            )

        delay_val = max(0.0, pulp.value(d))

        # Identify assigned pad
        assigned_pad_id: Optional[str] = None
        for pad, u in zip(pads, u_vars):
            if pulp.value(u) is not None and pulp.value(u) > 0.5:
                assigned_pad_id = pad.id
                break

        if delay_val > request.max_wait_time:
            return ResolveResult(
                approved=False,
                actual_start_time=None,
                delay=delay_val,
                reason=(
                    f"Required delay {delay_val:.1f}s exceeds "
                    f"max wait {request.max_wait_time:.1f}s"
                ),
                assigned_pad_id=None,
            )

        return ResolveResult(
            approved=True,
            actual_start_time=t_des + delay_val,
            delay=delay_val,
            reason="Approved" if delay_val < 0.01 else f"Approved with {delay_val:.1f}s delay",
            assigned_pad_id=assigned_pad_id,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers (mirrors resolver.py geometry)
    # ──────────────────────────────────────────────────────────────────────────

    def _request_lane_time(self, request: NewPlanRequest) -> float:
        wps = request.flight_path.waypoints
        if len(wps) < 2:
            return 0.0
        return sum(
            wps[i].distance_to(wps[i + 1]) / request.segment_speeds[i]
            for i in range(len(wps) - 1)
        )

    def _plan_t_arrive_wpN(self, plan: ApprovedPlan) -> float:
        wps = plan.flight_path.waypoints
        lane = (
            sum(wps[i].distance_to(wps[i + 1]) / plan.segment_speeds[i]
                for i in range(len(wps) - 1))
            if len(wps) >= 2 else 0.0
        )
        return plan.start_time + plan.t_takeoff + lane

    def _plan_touchdown(self, plan: ApprovedPlan) -> float:
        return self._plan_t_arrive_wpN(plan) + plan.t_land

    def _same_segment(self, a: Point3D, b: Point3D, c: Point3D, d: Point3D) -> bool:
        tol = self.POINT_TOL_M
        return a.distance_to(c) < tol and b.distance_to(d) < tol

    def _all_airspace_windows(self, vertiport_id: str) -> List[Tuple[float, float]]:
        windows: List[Tuple[float, float]] = []
        for plan in self.approved_plans:
            if plan.flight_path.takeoff_vertiport_id == vertiport_id:
                windows.append((plan.start_time, plan.start_time + plan.t_takeoff))
            if plan.flight_path.landing_vertiport_id == vertiport_id:
                t_wpN = self._plan_t_arrive_wpN(plan)
                windows.append((t_wpN, t_wpN + plan.t_land))
        return windows

    def _pad_occupation_windows(
        self, vertiport_id: str, pad_id: str, pad: Pad
    ) -> List[Tuple[float, float]]:
        windows: List[Tuple[float, float]] = []
        for plan in self.approved_plans:
            if (
                plan.flight_path.landing_vertiport_id == vertiport_id
                and plan.assigned_pad_id == pad_id
            ):
                t_td = self._plan_touchdown(plan)
                windows.append((t_td, t_td + pad.occupation_duration))
        return windows


# ──────────────────────────────────────────────────────────────────────────────
# Batch resolver (same priority-ordering logic as resolve_batch in resolver.py)
# ──────────────────────────────────────────────────────────────────────────────

def resolve_batch_mip(
    vertiports: Dict[str, Vertiport],
    approved_plans: List[ApprovedPlan],
    requests: List[NewPlanRequest],
    solver_msg: bool = False,
) -> List[ResolveResult]:
    """Resolve a batch of requests in priority order using the MIP solver."""
    indexed = sorted(
        enumerate(requests),
        key=lambda x: (x[1].priority, x[1].desired_start_time),
    )
    pool = list(approved_plans)
    results: List[Optional[ResolveResult]] = [None] * len(requests)

    for orig_idx, req in indexed:
        result = MIPConflictResolver(vertiports, pool, solver_msg=solver_msg).resolve(req)
        results[orig_idx] = result

        if result.approved:
            pool.append(
                ApprovedPlan(
                    plan_id=f"_mip_batch_{orig_idx}",
                    flight_path=req.flight_path,
                    drone_type=req.drone_type,
                    segment_speeds=req.segment_speeds,
                    start_time=result.actual_start_time,
                    t_takeoff=req.t_takeoff,
                    t_land=req.t_land,
                    priority=req.priority,
                    assigned_pad_id=result.assigned_pad_id,
                )
            )

    return results  # type: ignore[return-value]
