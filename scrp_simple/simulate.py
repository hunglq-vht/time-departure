"""Simulation demo for scrp_simple.

Builds a small scenario: a set of already-approved flight plans, then feeds
a handful of new flight-plan requests into the ``ConflictResolver`` one at a
time, in arrival order (not sorted by priority like ``resolve_batch``) —
mirroring how requests actually arrive at an ATC-style service.

Each newly *approved* plan is appended to the working pool before the next
request is resolved, so later requests see earlier ones as committed traffic.

For every request, the per-iteration delay trace (C1→C2→C3→C4, repeated
until the fixed point) is printed to show how the required departure delay
climbs and converges, per the fixed-point argument in DESIGN.md §6.

Run with:  python -m scrp_simple.simulate
"""

from __future__ import annotations

from typing import Dict, List

from .models import ApprovedPlan, FlightPath, NewPlanRequest, Pad, Point3D, Vertiport
from .resolver import ConflictResolver

WP_ABOVE_A = Point3D(0, 0, 100)
WP_ABOVE_B = Point3D(1000, 0, 100)


def build_vertiports() -> Dict[str, Vertiport]:
    return {
        "A": Vertiport("A", Point3D(0, 0, 0), [Pad("PA1", 20.0), Pad("PA2", 20.0)]),
        "B": Vertiport("B", Point3D(1000, 0, 0), [Pad("PB1", 20.0), Pad("PB2", 20.0)]),
    }


def build_approved_plans() -> List[ApprovedPlan]:
    """Three plans already in the system, forming a chain on lane A→B.

    Departures at t=0, 10, 20 (t_takeoff=10 each) fully occupy the A airspace
    until t=30.  Landings arrive at wpN at t=60, 70, 80 (t_land=10 each),
    occupying the B airspace until t=90, and alternate PB1/PB2 (20s each).
    """
    common = dict(
        flight_path=FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
        drone_type="X",
        segment_speeds=[20.0],
        t_takeoff=10.0,
        t_land=10.0,
    )
    return [
        ApprovedPlan(plan_id="AP1", start_time=0.0, assigned_pad_id="PB1", **common),
        ApprovedPlan(plan_id="AP2", start_time=10.0, assigned_pad_id="PB2", **common),
        ApprovedPlan(plan_id="AP3", start_time=20.0, assigned_pad_id="PB1", **common),
    ]


def build_pending_requests() -> List[NewPlanRequest]:
    """Four new requests submitted one after another, awaiting approval."""
    return [
        # R1: wants to squeeze in among AP1-AP3 -> expect lane/airspace/pad cascade.
        NewPlanRequest(
            flight_path=FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
            drone_type="X", segment_speeds=[20.0],
            desired_start_time=5.0, max_wait_time=300.0,
            t_takeoff=10.0, t_land=10.0,
        ),
        # R2: faster drone (25 m/s) on the same lane -> triggers the h_min_full
        # correction term on top of whatever R1 leaves behind.
        NewPlanRequest(
            flight_path=FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
            drone_type="Y", segment_speeds=[25.0],
            desired_start_time=5.0, max_wait_time=300.0,
            t_takeoff=10.0, t_land=10.0,
        ),
        # R3: departs well after the initial congestion clears -> near-zero delay.
        NewPlanRequest(
            flight_path=FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
            drone_type="X", segment_speeds=[20.0],
            desired_start_time=200.0, max_wait_time=300.0,
            t_takeoff=10.0, t_land=10.0,
        ),
        # R4: same congested window as R1 but a tight max_wait -> expect rejection.
        NewPlanRequest(
            flight_path=FlightPath("A", "B", [WP_ABOVE_A, WP_ABOVE_B]),
            drone_type="X", segment_speeds=[20.0],
            desired_start_time=5.0, max_wait_time=5.0,
            t_takeoff=10.0, t_land=10.0,
        ),
    ]


def _print_plan_table(title: str, plans: List[ApprovedPlan]) -> None:
    print(f"\n{title}")
    print(f"{'plan_id':<10}{'start':>8}{'speed':>8}{'pad':>6}")
    for p in sorted(plans, key=lambda x: x.start_time):
        print(f"{p.plan_id:<10}{p.start_time:>8.1f}{p.segment_speeds[0]:>8.1f}{p.assigned_pad_id or '-':>6}")


def _print_trace(trace: List[dict]) -> None:
    print(f"    {'iter':<6}{'step':<24}{'delay (s)':>10}")
    prev_delay = None
    for row in trace:
        marker = "" if prev_delay is None or abs(row["delay"] - prev_delay) > 0.01 else "  (unchanged)"
        print(f"    {row['iteration']:<6}{row['step']:<24}{row['delay']:>10.2f}{marker}")
        prev_delay = row["delay"]


def run_simulation() -> None:
    vertiports = build_vertiports()
    pool = build_approved_plans()
    _print_plan_table("Cac ke hoach bay da duoc phe duyet (truoc mo phong):", pool)

    requests = build_pending_requests()
    for i, req in enumerate(requests, start=1):
        print(f"\n{'=' * 70}")
        print(
            f"Request R{i}: desired_start={req.desired_start_time:.1f}s, "
            f"speed={req.segment_speeds[0]:.1f} m/s, max_wait={req.max_wait_time:.1f}s"
        )
        trace: List[dict] = []
        result = ConflictResolver(vertiports, pool).resolve(req, trace=trace)
        _print_trace(trace)

        if result.approved:
            print(
                f"  -> APPROVED: actual_start={result.actual_start_time:.2f}s "
                f"(delay={result.delay:.2f}s), pad={result.assigned_pad_id}, "
                f"converged in {trace[-1]['iteration']} iteration(s)"
            )
            pool.append(
                ApprovedPlan(
                    plan_id=f"R{i}",
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
        else:
            print(f"  -> REJECTED: {result.reason}")
        if result.conflict_details:
            print("  Conflict details:")
            for d in result.conflict_details:
                print(f"    - {d}")

    _print_plan_table("\nLich bay cuoi cung (da duyet, sau khi xu ly tat ca request):", pool)


if __name__ == "__main__":
    run_simulation()
