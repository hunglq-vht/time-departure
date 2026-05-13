"""Takeoff separation constraint (C0) computation.

Two drones departing from the same origin vertiport must maintain minimum
separation during their climb-out phase.  The constraint applies when:

  * Both drones share the same takeoff pad (exclusive pad use), OR
  * Their pads are within MSD of each other (adjacent pads, shared climb zone).

In either case the new drone must wait until the earlier-departing drone has
fully completed its takeoff phase before the new drone begins its own.
"""
from __future__ import annotations

import math
from typing import List

from .models import ApprovedPlan, FlightIntention, SystemState, TakeoffPad


def _pad_distance(a: TakeoffPad, b: TakeoffPad) -> float:
    dx = b.position[0] - a.position[0]
    dy = b.position[1] - a.position[1]
    dz = b.position[2] - a.position[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def compute_delta_C0(
    fi: FlightIntention,
    approved_plans: List[ApprovedPlan],
    t_dep_current: float,
    t_des: float,
    system_state: SystemState,
) -> float:
    """Return the additional delay (relative to *t_des*) for takeoff separation.

    Parameters
    ----------
    t_dep_current:
        Current estimate of fi's departure time, accounting for delays already
        accumulated from other constraints.  Used to determine whether fi's
        climb zone overlaps with an approved plan's climb zone.  The caller
        should iterate this function, updating *t_dep_current* on each round,
        until the returned delta stabilises (see ``resolve_conflict``).
    t_des:
        fi's originally desired departure time.  Returned delta is relative
        to this value.

    Returns 0.0 when *fi* has no origin_vertiport_id or takeoff_pad_id set
    (backward-compatible: existing flight intentions without takeoff context
    are unaffected).
    """
    if not fi.origin_vertiport_id or not fi.takeoff_pad_id:
        return 0.0

    origin_vp = system_state.vertiports.get(fi.origin_vertiport_id)
    if origin_vp is None:
        return 0.0

    pad_i = next(
        (p for p in origin_vp.takeoff_pads if p.id == fi.takeoff_pad_id), None
    )
    if pad_i is None:
        return 0.0

    t_takeoff_i = fi.t_takeoff if fi.t_takeoff is not None else 0.0

    # delta starts at the current delay already accumulated (relative to t_des),
    # so convergence in the caller's iteration is monotone non-decreasing.
    delta = max(0.0, t_dep_current - t_des)

    for plan in approved_plans:
        if plan.origin_vertiport_id != fi.origin_vertiport_id:
            continue
        if plan.t_dep is None:
            continue

        t_dep_j = plan.t_dep
        t_takeoff_j = plan.t_takeoff if plan.t_takeoff is not None else 0.0

        # Zone-clear time for plan j: when its climb-out phase ends
        t_clear_j = t_dep_j + t_takeoff_j

        # No temporal overlap: fi (at t_dep_current) finishes climbing before j starts
        if t_dep_current + t_takeoff_i <= t_dep_j:
            continue

        # No temporal overlap: j has fully cleared before fi's current estimated departure
        if t_clear_j <= t_dep_current:
            continue

        # Climb zones overlap — check spatial proximity of pads
        if plan.takeoff_pad_id == fi.takeoff_pad_id:
            # Same pad: exclusive-use rule always applies
            conflicts = True
        elif plan.takeoff_pad_id:
            pad_j = next(
                (p for p in origin_vp.takeoff_pads if p.id == plan.takeoff_pad_id),
                None,
            )
            if pad_j is None:
                continue
            d = _pad_distance(pad_i, pad_j)
            msd = system_state.msd_matrix.get((fi.uav_type, plan.uav_type), 50.0)
            conflicts = d < msd
        else:
            # Plan has no takeoff_pad_id — treat conservatively as conflicting
            conflicts = True

        if conflicts:
            needed = t_clear_j - t_des
            if needed > delta:
                delta = needed

    return max(0.0, delta)
