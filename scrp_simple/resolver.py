"""
Simplified conflict resolver.

Four constraints are checked in order inside a convergence loop:

  C1 – Lane separation
       Two drones on the same directed segment (waypoint → waypoint) must
       maintain at least MIN_SEPARATION_M at all times.
       Uses ``_h_min_full`` (defined in this module) for the minimum time-headway formula.

  C2 – Vertiport airspace at *departure* (takeoff serialisation)
       Only one drone may use the shared airspace above a vertiport at a
       time.  A drone occupies the airspace during its takeoff ascent:
           window = [start_time, start_time + t_takeoff]
       This window must not overlap with any other airspace reservation at
       the same vertiport (from either a takeoff or a landing).

  C3 – Vertiport airspace at *arrival* (landing serialisation)
       Same shared-airspace constraint, but for the landing descent:
           window = [t_arrive_wpN, t_arrive_wpN + t_land]
       where t_arrive_wpN = start_time + t_takeoff + lane_time.
       Must not overlap with any other airspace reservation at the landing
       vertiport (C2 and C3 share the same airspace-window pool, so
       takeoff ↔ landing conflicts at the same vertiport are caught
       automatically).

  C4 – Pad availability
       After the drone descends and touches down at
           t_touchdown = t_arrive_wpN + t_land
       its assigned pad is occupied for ``pad.occupation_duration`` seconds.
       The resolver finds the pad that becomes available soonest and assigns
       it to the new plan.  Pads are per-vertiport and have independent
       occupation timelines.

Convergence
-----------
Each constraint pass may push ``delay`` further.  A later arrival can
conflict with additional airspace windows that were clear at the previous
delay value, so all four passes run inside a loop until ``delay`` stabilises
(changes < 0.01 s between iterations).

Headway formula
---------------
Uses ``_h_min_full`` (local to this module):
    v_i = v_follow  (following / new drone)
    v_j = v_lead    (lead / approved drone)
    body_len_j = 0.0  (body length not modelled here)

Pad model vs slot.py
--------------------
``slot.py`` was *not* reused because it works on a discrete slot grid with
per-pad UAV-type compatibility (VertiportState / PadState).  This module
uses continuous time intervals and a simpler Pad model, which is
intentionally incompatible.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from .models import (
    ApprovedPlan,
    FlightPath,
    NewPlanRequest,
    Pad,
    Point3D,
    ResolveResult,
    Vertiport,
)


def _h_min_full(
    msd: float,
    body_len_j: float,
    v_i: float,
    v_j: float,
    seg_length: float,
    v_min_seg: float,  # noqa: ARG001 — kept for API symmetry
) -> float:
    """Minimum time headway between a follower (v_i) and a lead drone (v_j).

    h = (MSD + L_body_j) / v_j
        + max(0, v_i - v_j) * seg_length / (v_i * v_j)
    """
    base = (msd + body_len_j) / v_j
    correction = max(0.0, v_i - v_j) * seg_length / (v_i * v_j) if v_i > v_j else 0.0
    return base + correction


class ConflictResolver:
    """Resolve a single new-plan request against a fixed set of approved plans."""

    MIN_SEPARATION_M: float = 50.0  # minimum drone-to-drone distance on a shared lane
    POINT_TOL_M: float = 1.0        # tolerance for "same waypoint" comparisons
    MAX_ITER: int = 15              # maximum convergence iterations

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
        """Return the minimum-delay approval (or rejection) for *request*.

        Iterates all four constraint passes until the required delay
        converges, then compares against ``max_wait_time``.
        """
        delay = 0.0
        assigned_pad_id: Optional[str] = None
        details: List[str] = []

        for _ in range(self.MAX_ITER):
            prev = delay

            # C1: keep safe distance from other drones on shared lane segments
            delay, new_d = self._resolve_lane_conflicts(request, delay)
            details += [x for x in new_d if x not in details]

            # C2: serialise takeoffs at the departure vertiport
            delay, new_d = self._resolve_vertiport_airspace(request, delay, "departure")
            details += [x for x in new_d if x not in details]

            # C3: serialise landings (and prevent takeoff/landing overlap) at arrival
            delay, new_d = self._resolve_vertiport_airspace(request, delay, "arrival")
            details += [x for x in new_d if x not in details]

            # C4: assign a free pad and check occupation timeline
            delay, assigned_pad_id, new_d = self._resolve_pad_conflict(request, delay)
            details += [x for x in new_d if x not in details]

            if abs(delay - prev) < 0.01:
                break

        if delay > request.max_wait_time:
            return ResolveResult(
                approved=False,
                actual_start_time=None,
                delay=delay,
                reason=(
                    f"Required delay {delay:.1f}s exceeds "
                    f"max wait {request.max_wait_time:.1f}s"
                ),
                assigned_pad_id=None,
                conflict_details=details,
            )

        return ResolveResult(
            approved=True,
            actual_start_time=request.desired_start_time + delay,
            delay=delay,
            reason="Approved" if delay < 0.01 else f"Approved with {delay:.1f}s delay",
            assigned_pad_id=assigned_pad_id,
            conflict_details=details,
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Geometry & timing helpers
    # ──────────────────────────────────────────────────────────────────────────

    @staticmethod
    def _lane_path(fp: FlightPath) -> List[Point3D]:
        """Intermediate waypoints only – the actual horizontal lane.

        Vertiport locations are excluded because takeoff and landing phases
        are vertical and handled separately via the airspace constraint.
        """
        return list(fp.waypoints)

    @staticmethod
    def _lane_time_from_wps(waypoints: List[Point3D], speeds: List[float]) -> float:
        """Total flight time through all lane segments (wp→wp)."""
        if len(waypoints) < 2:
            return 0.0  # no lane segments for a direct hop
        return sum(
            waypoints[i].distance_to(waypoints[i + 1]) / speeds[i]
            for i in range(len(waypoints) - 1)
        )

    def _plan_lane_time(self, plan: ApprovedPlan) -> float:
        return self._lane_time_from_wps(plan.flight_path.waypoints, plan.segment_speeds)

    def _request_lane_time(self, request: NewPlanRequest) -> float:
        return self._lane_time_from_wps(request.flight_path.waypoints, request.segment_speeds)

    def _plan_t_arrive_wpN(self, plan: ApprovedPlan) -> float:
        """Absolute time when the approved drone reaches its last waypoint.

        This is when the descent begins, i.e. the start of the drone's
        airspace reservation at the landing vertiport.
        """
        return plan.start_time + plan.t_takeoff + self._plan_lane_time(plan)

    def _plan_touchdown(self, plan: ApprovedPlan) -> float:
        """Absolute time when the approved drone touches down on its pad."""
        return self._plan_t_arrive_wpN(plan) + plan.t_land

    def _same_segment(
        self, a: Point3D, b: Point3D, c: Point3D, d: Point3D
    ) -> bool:
        """True when directed segment a→b coincides with c→d (within tolerance)."""
        tol = self.POINT_TOL_M
        return a.distance_to(c) < tol and b.distance_to(d) < tol

    def _min_follow_headway(
        self, v_lead: float, v_follow: float, seg_len: float
    ) -> float:
        """Minimum time gap (s) before the follower may enter a segment after the lead.

        Delegates to ``_h_min_full``:
          - v_i = v_follow
          - v_j = v_lead
          - body_len_j = 0  (body length not modelled in this simplified version)
        """
        return _h_min_full(
            msd=self.MIN_SEPARATION_M,
            body_len_j=0.0,
            v_i=v_follow,
            v_j=v_lead,
            seg_length=seg_len,
            v_min_seg=min(v_lead, v_follow),
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Airspace helpers (shared across C2 and C3)
    # ──────────────────────────────────────────────────────────────────────────

    def _all_airspace_windows(self, vertiport_id: str) -> List[Tuple[float, float]]:
        """All currently reserved airspace windows at *vertiport_id*.

        Includes:
          • takeoff windows  [start_time, start_time + t_takeoff]
            for every approved plan *departing from* vertiport_id.
          • descent windows  [t_arrive_wpN, t_arrive_wpN + t_land]
            for every approved plan *landing at* vertiport_id.

        Both window types share the same physical airspace above the
        vertiport (vertical corridor used for both ascent and descent), so
        they form a single pool of mutual-exclusion windows.  This ensures
        that a simultaneous takeoff and landing at the same vertiport is
        also prevented.
        """
        windows: List[Tuple[float, float]] = []
        for plan in self.approved_plans:
            if plan.flight_path.takeoff_vertiport_id == vertiport_id:
                # Drone ascends: airspace busy from departure until wp1 reached
                windows.append((plan.start_time, plan.start_time + plan.t_takeoff))
            if plan.flight_path.landing_vertiport_id == vertiport_id:
                # Drone descends: airspace busy from wpN until touchdown
                t_wpN = self._plan_t_arrive_wpN(plan)
                windows.append((t_wpN, t_wpN + plan.t_land))
        return windows

    @staticmethod
    def _advance_past_conflicts(
        t_start: float,
        window_dur: float,
        all_windows: List[Tuple[float, float]],
    ) -> float:
        """Find the earliest t_start ≥ current t_start such that
        [t_start, t_start + window_dur] does not overlap any window in
        all_windows.

        Each iteration jumps t_start to the furthest end-time of all
        currently-conflicting windows (greedy forward scan).  With a finite
        set of windows the loop always terminates.
        """
        for _ in range(len(all_windows) + 1):
            t_end = t_start + window_dur
            # A window (s, e) conflicts iff s < t_end AND e > t_start
            conflicts = [(s, e) for s, e in all_windows if s < t_end and e > t_start]
            if not conflicts:
                return t_start
            # Jump past the furthest conflicting window
            t_start = max(e for _, e in conflicts)
        return t_start  # all windows exhausted, current position is safe

    # ──────────────────────────────────────────────────────────────────────────
    # C1 – Lane separation
    # ──────────────────────────────────────────────────────────────────────────

    def _resolve_lane_conflicts(
        self, request: NewPlanRequest, current_delay: float
    ) -> Tuple[float, List[str]]:
        """Push departure delay until MIN_SEPARATION_M is maintained on every
        shared lane segment.

        Only waypoint-to-waypoint segments are checked.  The takeoff ascent
        and landing descent are vertical phases handled by the airspace
        constraint; they are never in the same lane as another drone's
        horizontal segment.

        Time to reach segment i from plan start:
            t_takeoff  +  sum(lane_seg_times[0 .. i-1])
        """
        new_wps = self._lane_path(request.flight_path)
        required_delay = current_delay
        details: List[str] = []

        if len(new_wps) < 2:
            # No lane segments – a direct hop has no horizontal conflict
            return required_delay, details

        for approved in self.approved_plans:
            appr_wps = self._lane_path(approved.flight_path)
            if len(appr_wps) < 2:
                continue  # approved plan also has no lane – skip

            for ni in range(len(new_wps) - 1):
                for ai in range(len(appr_wps) - 1):
                    if not self._same_segment(
                        new_wps[ni], new_wps[ni + 1],
                        appr_wps[ai], appr_wps[ai + 1],
                    ):
                        continue  # different segments, no conflict possible

                    seg_len = new_wps[ni].distance_to(new_wps[ni + 1])
                    v_new = request.segment_speeds[ni]
                    v_appr = approved.segment_speeds[ai]

                    # Seconds from departure until new drone reaches segment ni
                    # = takeoff duration + all preceding lane segments
                    time_before_seg_new = request.t_takeoff + sum(
                        new_wps[k].distance_to(new_wps[k + 1]) / request.segment_speeds[k]
                        for k in range(ni)
                    )

                    # Same for the approved drone
                    time_before_seg_appr = approved.t_takeoff + sum(
                        appr_wps[k].distance_to(appr_wps[k + 1]) / approved.segment_speeds[k]
                        for k in range(ai)
                    )

                    appr_entry = approved.start_time + time_before_seg_appr
                    new_entry = (
                        request.desired_start_time + required_delay + time_before_seg_new
                    )
                    new_exit = new_entry + seg_len / v_new

                    # Case A: new drone leads – exits before approved enters with safe gap
                    lead_headway = self._min_follow_headway(v_new, v_appr, seg_len)
                    if new_exit + lead_headway <= appr_entry:
                        continue  # safe: new is ahead

                    # Case B: new drone follows – enters after approved with safe headway
                    follow_headway = self._min_follow_headway(v_appr, v_new, seg_len)
                    if new_entry >= appr_entry + follow_headway:
                        continue  # safe: new is behind

                    # Conflict: delay new plan until it can safely follow
                    required_entry = appr_entry + follow_headway
                    needed_delay = (
                        required_entry - time_before_seg_new - request.desired_start_time
                    )
                    if needed_delay > required_delay:
                        details.append(
                            f"Lane conflict with {approved.plan_id!r} "
                            f"(segment {ni}→{ni + 1}): delay ≥ {needed_delay:.1f}s"
                        )
                        required_delay = needed_delay

        return required_delay, details

    # ──────────────────────────────────────────────────────────────────────────
    # C2 / C3 – Vertiport airspace serialisation
    # ──────────────────────────────────────────────────────────────────────────

    def _resolve_vertiport_airspace(
        self,
        request: NewPlanRequest,
        current_delay: float,
        which: str,  # "departure" or "arrival"
    ) -> Tuple[float, List[str]]:
        """Ensure the new plan's airspace window does not overlap any existing
        reservation at the relevant vertiport.

        ``which="departure"``
            Checks the *takeoff* window [t_dep, t_dep + t_takeoff] at the
            departure vertiport.  Pushing this window forward is equivalent
            to increasing the departure delay directly.

        ``which="arrival"``
            Checks the *descent* window [t_wpN, t_wpN + t_land] at the
            arrival vertiport.  t_wpN depends on delay, so a conflict here
            translates into additional departure delay.

        Both "departure" and "arrival" query the *same* airspace-window pool
        (which includes both takeoff and descent windows of all approved
        plans).  This means a new departure is blocked by an ongoing landing
        at the same vertiport, and vice versa – matching the shared-airspace
        physics described in the design.
        """
        details: List[str] = []

        if which == "departure":
            vertiport_id = request.flight_path.takeoff_vertiport_id
            all_windows = self._all_airspace_windows(vertiport_id)

            # New plan's takeoff window starts at departure time
            t_dep = request.desired_start_time + current_delay
            t_dep_free = self._advance_past_conflicts(
                t_dep, request.t_takeoff, all_windows
            )

            required_delay = max(current_delay, t_dep_free - request.desired_start_time)
            if required_delay > current_delay:
                details.append(
                    f"Airspace conflict at departure {vertiport_id!r}: "
                    f"delay ≥ {required_delay:.1f}s"
                )

        else:  # "arrival"
            vertiport_id = request.flight_path.landing_vertiport_id
            all_windows = self._all_airspace_windows(vertiport_id)

            lane_time = self._request_lane_time(request)
            # t_arrive_wpN = desired_start + delay + t_takeoff + lane_time
            t_wpN = (
                request.desired_start_time
                + current_delay
                + request.t_takeoff
                + lane_time
            )
            t_wpN_free = self._advance_past_conflicts(
                t_wpN, request.t_land, all_windows
            )

            # Back-calculate departure delay from the required t_wpN
            required_start = t_wpN_free - request.t_takeoff - lane_time
            required_delay = max(
                current_delay, required_start - request.desired_start_time
            )
            if required_delay > current_delay:
                details.append(
                    f"Airspace conflict at arrival {vertiport_id!r}: "
                    f"delay ≥ {required_delay:.1f}s"
                )

        return required_delay, details

    # ──────────────────────────────────────────────────────────────────────────
    # C4 – Pad availability
    # ──────────────────────────────────────────────────────────────────────────

    def _resolve_pad_conflict(
        self,
        request: NewPlanRequest,
        current_delay: float,
    ) -> Tuple[float, Optional[str], List[str]]:
        """Assign a landing pad and ensure it is free at touchdown.

        For each pad at the landing vertiport, compute the earliest touchdown
        time at which that pad is free.  Select the pad that gives the
        smallest required delay, then back-calculate the departure delay.

        Note: the airspace constraint (C3) has already been checked before
        this method runs, so the descent window is guaranteed conflict-free.
        This method only needs to worry about pad-level occupation after
        touchdown.

        Returns ``(required_delay, assigned_pad_id, details)``.
        """
        landing_id = request.flight_path.landing_vertiport_id
        vertiport = self.vertiports[landing_id]
        details: List[str] = []

        lane_time = self._request_lane_time(request)
        # Touchdown = departure + delay + climb + lane + descent
        t_touchdown = (
            request.desired_start_time
            + current_delay
            + request.t_takeoff
            + lane_time
            + request.t_land
        )

        best_delay = float("inf")
        best_pad_id: Optional[str] = None

        for pad in vertiport.pads:
            occ_windows = self._pad_occupation_windows(landing_id, pad.id, pad)

            # Find the earliest time this specific pad is free
            t_td = t_touchdown
            for _ in range(len(occ_windows) + 1):
                busy = [(s, e) for s, e in occ_windows if s <= t_td < e]
                if not busy:
                    break
                # Advance to when the next overlapping occupation ends
                t_td = min(e for _, e in busy)

            # Convert the free touchdown time back to a departure delay
            required_start = (
                t_td - request.t_land - lane_time - request.t_takeoff
            )
            pad_delay = max(current_delay, required_start - request.desired_start_time)

            if pad_delay < best_delay:
                best_delay = pad_delay
                best_pad_id = pad.id

        if best_pad_id is None:
            details.append(f"No landing pad available at {landing_id!r}")
            return float("inf"), None, details

        if best_delay > current_delay:
            details.append(
                f"Pad {best_pad_id!r} at {landing_id!r}: delay ≥ {best_delay:.1f}s"
            )

        return best_delay, best_pad_id, details

    def _pad_occupation_windows(
        self,
        vertiport_id: str,
        pad_id: str,
        pad: Pad,
    ) -> List[Tuple[float, float]]:
        """All occupation windows for *pad_id* at *vertiport_id*.

        A pad is occupied from touchdown until touchdown + pad.occupation_duration.
        Only approved plans whose ``assigned_pad_id`` matches are counted.
        """
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
# Batch resolver
# ──────────────────────────────────────────────────────────────────────────────

def resolve_batch(
    vertiports: Dict[str, Vertiport],
    approved_plans: List[ApprovedPlan],
    requests: List[NewPlanRequest],
) -> List[ResolveResult]:
    """Resolve a batch of new-plan requests in priority order.

    Requests with a lower ``priority`` number (= higher priority) are resolved
    first.  Ties broken by ``desired_start_time``.

    Each approved result is immediately added to the working pool so that
    higher-priority plans "lock in" their airspace and pad slots before
    lower-priority requests are evaluated.

    The returned list is indexed to match the input ``requests`` list.
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
            # Add the newly approved plan to the pool so subsequent
            # (lower-priority) requests treat it as a conflict source.
            pool.append(
                ApprovedPlan(
                    plan_id=f"_batch_{orig_idx}",
                    flight_path=req.flight_path,
                    drone_type=req.drone_type,
                    segment_speeds=req.segment_speeds,
                    start_time=result.actual_start_time,  # type: ignore[arg-type]
                    t_takeoff=req.t_takeoff,
                    t_land=req.t_land,
                    priority=req.priority,
                    assigned_pad_id=result.assigned_pad_id,
                )
            )

    return results  # type: ignore[return-value]
