"""
Simplified conflict resolver.

Two conflict sources are checked:
  1. Same-lane separation  – two drones on the same directed segment must
     maintain at least MIN_SEPARATION_M at all times.
  2. Landing-pad availability – a vertiport pad is occupied for
     LANDING_DURATION_S seconds after a drone touches down.

The resolver finds the *minimum* departure delay for the new plan that
satisfies both constraints.  If that delay exceeds ``max_wait_time`` the
request is rejected.

Priority is honoured at the *batch* level via ``resolve_batch``: requests
are processed in priority order so higher-priority plans lock in their
slots first.

Headway formula
---------------
Reuses ``scrp.headway.h_min_full`` directly.  Parameter mapping:
    v_i (headway.py)  = v_follow  (follower / new drone)
    v_j (headway.py)  = v_lead    (lead / approved drone)
    body_len_j        = 0.0       (body length excluded in the simple model)
    v_min_seg         = unused    (not needed by the formula itself)

Landing-pad model
-----------------
``slot.py`` was *not* reused because it operates on a discrete slot grid
with per-pad UAV-type compatibility (``VertiportState`` / ``PadState``).
The simplified model collapses pad state to a single integer (``total_pads``)
and tracks occupancy as continuous time intervals, which makes ``find_earliest_slot``
incompatible without pulling in the full slot machinery.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from scrp.headway import h_min_full

from .models import ApprovedPlan, FlightPath, NewPlanRequest, Point3D, ResolveResult, Vertiport


class ConflictResolver:
    """Resolve a single new-plan request against a fixed set of approved plans."""

    MIN_SEPARATION_M: float = 50.0     # minimum distance between drones on the same lane
    LANDING_DURATION_S: float = 120.0  # seconds a pad is occupied per landing
    POINT_TOL_M: float = 1.0           # tolerance for "same waypoint" comparisons
    MAX_ITER: int = 10                 # convergence iterations between the two passes

    def __init__(
        self,
        vertiports: Dict[str, Vertiport],
        approved_plans: List[ApprovedPlan],
    ) -> None:
        self.vertiports = vertiports
        self.approved_plans = approved_plans

    # ──────────────────────────────────────────────────────────────────────────
    # Public API
    # ──────────────────────────────────────────────────────────────────────────

    def resolve(self, request: NewPlanRequest) -> ResolveResult:
        """
        Compute the minimum departure delay for *request* and return an
        approval or rejection.
        """
        delay = 0.0
        details: List[str] = []

        for _ in range(self.MAX_ITER):
            prev = delay

            delay, lane_details = self._resolve_lane_conflicts(request, delay)
            details += [d for d in lane_details if d not in details]

            delay, pad_details = self._resolve_pad_conflict(request, delay)
            details += [d for d in pad_details if d not in details]

            if abs(delay - prev) < 0.01:
                break

        if delay > request.max_wait_time:
            return ResolveResult(
                approved=False,
                actual_start_time=None,
                delay=delay,
                reason=(
                    f"Required delay {delay:.1f}s exceeds max wait {request.max_wait_time:.1f}s"
                ),
                conflict_details=details,
            )

        return ResolveResult(
            approved=True,
            actual_start_time=request.desired_start_time + delay,
            delay=delay,
            reason="Approved" if delay < 0.01 else f"Approved with {delay:.1f}s delay",
            conflict_details=details,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Geometry helpers
    # ──────────────────────────────────────────────────────────────────────────

    def _full_path(self, fp: FlightPath) -> List[Point3D]:
        """Expand a FlightPath to the full list of 3-D points."""
        return [
            self.vertiports[fp.takeoff_vertiport_id].location,
            *fp.waypoints,
            self.vertiports[fp.landing_vertiport_id].location,
        ]

    @staticmethod
    def _flight_time(path: List[Point3D], speeds: List[float]) -> float:
        return sum(
            path[i].distance_to(path[i + 1]) / speeds[i] for i in range(len(path) - 1)
        )

    @staticmethod
    def _segment_times(
        path: List[Point3D], speeds: List[float], start_time: float
    ) -> List[Tuple[float, float]]:
        """Return ``(entry_time, exit_time)`` for each segment."""
        result: List[Tuple[float, float]] = []
        t = start_time
        for i in range(len(path) - 1):
            dur = path[i].distance_to(path[i + 1]) / speeds[i]
            result.append((t, t + dur))
            t += dur
        return result

    def _same_segment(
        self, a: Point3D, b: Point3D, c: Point3D, d: Point3D
    ) -> bool:
        """True when the directed segment a→b is the same as c→d (within tolerance)."""
        tol = self.POINT_TOL_M
        return a.distance_to(c) < tol and b.distance_to(d) < tol

    def _min_follow_headway(
        self, v_lead: float, v_follow: float, seg_len: float
    ) -> float:
        """
        Thin wrapper around ``scrp.headway.h_min_full``.

        headway.py convention:  v_i = follower,  v_j = lead.
        body_len_j = 0 (body length not modelled in the simple variant).
        """
        return h_min_full(
            msd=self.MIN_SEPARATION_M,
            body_len_j=0.0,
            v_i=v_follow,
            v_j=v_lead,
            seg_length=seg_len,
            v_min_seg=min(v_lead, v_follow),
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Constraint 1: same-lane separation
    # ──────────────────────────────────────────────────────────────────────────

    def _resolve_lane_conflicts(
        self, request: NewPlanRequest, current_delay: float
    ) -> Tuple[float, List[str]]:
        new_path = self._full_path(request.flight_path)
        required_delay = current_delay
        details: List[str] = []

        for approved in self.approved_plans:
            appr_path = self._full_path(approved.flight_path)
            appr_times = self._segment_times(appr_path, approved.segment_speeds, approved.start_time)

            for ni in range(len(new_path) - 1):
                for ai in range(len(appr_path) - 1):
                    if not self._same_segment(
                        new_path[ni], new_path[ni + 1],
                        appr_path[ai], appr_path[ai + 1],
                    ):
                        continue

                    seg_len = new_path[ni].distance_to(new_path[ni + 1])
                    v_new = request.segment_speeds[ni]
                    v_appr = approved.segment_speeds[ai]
                    appr_entry, _ = appr_times[ai]

                    # Seconds from plan start to reach the beginning of segment ni
                    time_before_seg = sum(
                        new_path[k].distance_to(new_path[k + 1]) / request.segment_speeds[k]
                        for k in range(ni)
                    )
                    new_entry = request.desired_start_time + required_delay + time_before_seg
                    new_exit = new_entry + seg_len / v_new

                    # Check whether new drone can safely lead
                    lead_headway = self._min_follow_headway(v_new, v_appr, seg_len)
                    if new_exit + lead_headway <= appr_entry:
                        continue  # new leads – no conflict

                    # Check whether new drone safely follows
                    follow_headway = self._min_follow_headway(v_appr, v_new, seg_len)
                    if new_entry >= appr_entry + follow_headway:
                        continue  # new follows at safe distance

                    # Conflict: push new drone to follow safely
                    required_entry = appr_entry + follow_headway
                    needed_delay = required_entry - time_before_seg - request.desired_start_time
                    if needed_delay > required_delay:
                        details.append(
                            f"Lane conflict with {approved.plan_id!r} "
                            f"(segment {ni}→{ni + 1}): delay ≥ {needed_delay:.1f}s"
                        )
                        required_delay = needed_delay

        return required_delay, details

    # ──────────────────────────────────────────────────────────────────────────
    # Constraint 2: landing-pad availability
    # ──────────────────────────────────────────────────────────────────────────

    def _resolve_pad_conflict(
        self, request: NewPlanRequest, current_delay: float
    ) -> Tuple[float, List[str]]:
        landing_id = request.flight_path.landing_vertiport_id
        vertiport = self.vertiports[landing_id]
        details: List[str] = []

        new_path = self._full_path(request.flight_path)
        total_flight = self._flight_time(new_path, request.segment_speeds)
        arrival = request.desired_start_time + current_delay + total_flight

        occupancies = self._landing_occupancies(landing_id)

        # Advance arrival time until a pad is free
        for _ in range(500):
            busy = [(s, e) for s, e in occupancies if s <= arrival < e]
            if len(busy) < vertiport.total_pads:
                break
            arrival = min(e for _, e in busy)
        else:
            details.append(f"No landing pad available at {landing_id!r}")
            return float("inf"), details

        required_delay = arrival - total_flight - request.desired_start_time
        if required_delay > current_delay:
            details.append(
                f"Landing pad at {landing_id!r} busy: delay ≥ {required_delay:.1f}s"
            )

        return max(current_delay, required_delay), details

    def _landing_occupancies(self, vertiport_id: str) -> List[Tuple[float, float]]:
        result = []
        for plan in self.approved_plans:
            if plan.flight_path.landing_vertiport_id != vertiport_id:
                continue
            path = self._full_path(plan.flight_path)
            arrival = plan.start_time + self._flight_time(path, plan.segment_speeds)
            result.append((arrival, arrival + self.LANDING_DURATION_S))
        return result


# ──────────────────────────────────────────────────────────────────────────────
# Batch resolver
# ──────────────────────────────────────────────────────────────────────────────

def resolve_batch(
    vertiports: Dict[str, Vertiport],
    approved_plans: List[ApprovedPlan],
    requests: List[NewPlanRequest],
) -> List[ResolveResult]:
    """
    Resolve a batch of new-plan requests in priority order.

    Requests with a lower ``priority`` number (= higher priority) are resolved
    first.  Ties are broken by ``desired_start_time``.  Each approved result is
    added to the working pool before lower-priority requests are evaluated, so
    higher-priority plans effectively "lock in" their slots first.

    Returns results in the *same order* as the input ``requests`` list.
    """
    indexed = sorted(
        enumerate(requests),
        key=lambda x: (x[1].priority, x[1].desired_start_time),
    )
    pool = list(approved_plans)
    results: List[Optional[ResolveResult]] = [None] * len(requests)

    for orig_idx, req in indexed:
        result = ConflictResolver(vertiports, pool).resolve(req)
        results[orig_idx] = result

        if result.approved:
            pool.append(
                ApprovedPlan(
                    plan_id=f"_batch_{orig_idx}",
                    flight_path=req.flight_path,
                    drone_type=req.drone_type,
                    segment_speeds=req.segment_speeds,
                    start_time=result.actual_start_time,  # type: ignore[arg-type]
                    priority=req.priority,
                )
            )

    return results  # type: ignore[return-value]
