"""
Benchmark: Fixed-Point (simple) vs Mixed-Integer Programming conflict resolver.

Scenarios
---------
Two benchmark families are run:

  1. Single-request  – one new request resolved against N pre-approved plans.
     N ∈ {5, 10, 20, 40, 80} with 10 independent repeats each.

  2. Batch-request   – a batch of B new requests resolved in priority order,
     each against N=20 pre-approved plans.
     B ∈ {2, 5, 10, 20} with 5 repeats each.

Each scenario is constructed so that genuine conflicts exist and both
solvers must compute a non-zero delay, exercising all four constraints.

The script prints a table of mean ± std wall-clock times (ms) for each
scenario and a final summary comparing the two approaches.
"""

from __future__ import annotations

import math
import random
import statistics
import time
from typing import Dict, List, Tuple

from .models import (
    ApprovedPlan,
    FlightPath,
    NewPlanRequest,
    Pad,
    Point3D,
    Vertiport,
)
from .resolver import ConflictResolver, resolve_batch
from .mip_resolver import MIPConflictResolver, resolve_batch_mip

# ──────────────────────────────────────────────────────────────────────────────
# Deterministic scenario builder
# ──────────────────────────────────────────────────────────────────────────────

_SHARED_WP_A = Point3D(0.0, 0.0, 100.0)
_SHARED_WP_B = Point3D(5_000.0, 0.0, 100.0)
_SHARED_WP_C = Point3D(10_000.0, 0.0, 100.0)


def _make_vertiports(n_pads: int = 2) -> Dict[str, Vertiport]:
    pads = [Pad(id=f"P{i}", occupation_duration=60.0) for i in range(n_pads)]
    return {
        "VA": Vertiport(id="VA", location=Point3D(-500, 0, 0), pads=[Pad("PA0", 60.0)]),
        "VB": Vertiport(id="VB", location=Point3D(10_500, 0, 0), pads=list(pads)),
    }


def _make_approved_plans(n: int, rng: random.Random) -> List[ApprovedPlan]:
    """Create N approved plans flying VA→VB on the shared lane, spaced 30–60 s apart."""
    plans: List[ApprovedPlan] = []
    t = 0.0
    for i in range(n):
        t += rng.uniform(30.0, 60.0)
        speed = rng.uniform(20.0, 30.0)
        plans.append(
            ApprovedPlan(
                plan_id=f"AP{i}",
                flight_path=FlightPath(
                    takeoff_vertiport_id="VA",
                    landing_vertiport_id="VB",
                    waypoints=[_SHARED_WP_A, _SHARED_WP_B, _SHARED_WP_C],
                ),
                drone_type="generic",
                segment_speeds=[speed, speed],
                start_time=t,
                t_takeoff=10.0,
                t_land=10.0,
                priority=5,
                assigned_pad_id=f"P{i % 2}",
            )
        )
    return plans


def _make_request(
    desired_start: float,
    speed: float = 25.0,
    max_wait: float = 9_999.0,
) -> NewPlanRequest:
    """New request on the same VA→VB corridor, almost certain to conflict."""
    return NewPlanRequest(
        flight_path=FlightPath(
            takeoff_vertiport_id="VA",
            landing_vertiport_id="VB",
            waypoints=[_SHARED_WP_A, _SHARED_WP_B, _SHARED_WP_C],
        ),
        drone_type="generic",
        segment_speeds=[speed, speed],
        desired_start_time=desired_start,
        max_wait_time=max_wait,
        t_takeoff=10.0,
        t_land=10.0,
        priority=5,
    )


# ──────────────────────────────────────────────────────────────────────────────
# Timing helpers
# ──────────────────────────────────────────────────────────────────────────────

def _time_ms(fn) -> float:
    """Return elapsed wall-clock time in milliseconds for fn()."""
    t0 = time.perf_counter()
    fn()
    return (time.perf_counter() - t0) * 1_000.0


def _stats(times: List[float]) -> Tuple[float, float]:
    mean = statistics.mean(times)
    std = statistics.stdev(times) if len(times) > 1 else 0.0
    return mean, std


# ──────────────────────────────────────────────────────────────────────────────
# Benchmark 1: single-request, vary N approved plans
# ──────────────────────────────────────────────────────────────────────────────

def bench_single(
    plan_counts: List[int],
    repeats: int = 10,
    seed: int = 42,
) -> List[dict]:
    rows = []
    rng = random.Random(seed)

    for n in plan_counts:
        fp_times, mip_times = [], []

        for _ in range(repeats):
            vports = _make_vertiports(n_pads=2)
            approved = _make_approved_plans(n, rng)
            # Request desired start at t=0 — guaranteed to conflict with early plans
            req = _make_request(desired_start=0.0, speed=28.0)

            fp_times.append(
                _time_ms(lambda: ConflictResolver(vports, approved).resolve(req))
            )
            mip_times.append(
                _time_ms(lambda: MIPConflictResolver(vports, approved).resolve(req))
            )

        fp_mean, fp_std = _stats(fp_times)
        mip_mean, mip_std = _stats(mip_times)
        rows.append(
            dict(
                n_plans=n,
                fp_mean=fp_mean,
                fp_std=fp_std,
                mip_mean=mip_mean,
                mip_std=mip_std,
                speedup=mip_mean / fp_mean if fp_mean > 0 else float("inf"),
            )
        )
    return rows


# ──────────────────────────────────────────────────────────────────────────────
# Benchmark 2: batch requests, vary batch size
# ──────────────────────────────────────────────────────────────────────────────

def bench_batch(
    batch_sizes: List[int],
    n_approved: int = 20,
    repeats: int = 5,
    seed: int = 99,
) -> List[dict]:
    rows = []
    rng = random.Random(seed)

    for b in batch_sizes:
        fp_times, mip_times = [], []

        for _ in range(repeats):
            vports = _make_vertiports(n_pads=3)
            approved = _make_approved_plans(n_approved, rng)
            reqs = [
                _make_request(
                    desired_start=rng.uniform(0.0, 30.0),
                    speed=rng.uniform(20.0, 30.0),
                )
                for _ in range(b)
            ]

            fp_times.append(
                _time_ms(lambda: resolve_batch(vports, approved, reqs))
            )
            mip_times.append(
                _time_ms(lambda: resolve_batch_mip(vports, approved, reqs))
            )

        fp_mean, fp_std = _stats(fp_times)
        mip_mean, mip_std = _stats(mip_times)
        rows.append(
            dict(
                batch_size=b,
                fp_mean=fp_mean,
                fp_std=fp_std,
                mip_mean=mip_mean,
                mip_std=mip_std,
                speedup=mip_mean / fp_mean if fp_mean > 0 else float("inf"),
            )
        )
    return rows


# ──────────────────────────────────────────────────────────────────────────────
# Correctness check: both algorithms must agree on delay value
# ──────────────────────────────────────────────────────────────────────────────

def check_correctness(seed: int = 7) -> None:
    rng = random.Random(seed)
    vports = _make_vertiports(n_pads=2)
    approved = _make_approved_plans(15, rng)
    req = _make_request(desired_start=0.0, speed=27.0)

    fp_result  = ConflictResolver(vports, approved).resolve(req)
    mip_result = MIPConflictResolver(vports, approved).resolve(req)

    tol = 1.0  # 1-second tolerance between the two solvers
    match = (
        fp_result.approved == mip_result.approved
        and abs(fp_result.delay - mip_result.delay) <= tol
    )
    status = "PASS" if match else "FAIL"
    print(
        f"Correctness check [{status}]  "
        f"FP delay={fp_result.delay:.2f}s  "
        f"MIP delay={mip_result.delay:.2f}s  "
        f"approved: FP={fp_result.approved} MIP={mip_result.approved}"
    )
    if not match:
        print(f"  FP:  {fp_result}")
        print(f"  MIP: {mip_result}")


# ──────────────────────────────────────────────────────────────────────────────
# Pretty-print helpers
# ──────────────────────────────────────────────────────────────────────────────

def _print_table_single(rows: List[dict]) -> None:
    hdr = f"{'N approved':>12} | {'FP mean (ms)':>14} {'FP std':>10} | {'MIP mean (ms)':>14} {'MIP std':>10} | {'MIP/FP ratio':>13}"
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(
            f"{r['n_plans']:>12} | "
            f"{r['fp_mean']:>12.2f}   {r['fp_std']:>8.2f}   | "
            f"{r['mip_mean']:>12.2f}   {r['mip_std']:>8.2f}   | "
            f"{r['speedup']:>11.1f}×"
        )


def _print_table_batch(rows: List[dict]) -> None:
    hdr = f"{'Batch size':>12} | {'FP mean (ms)':>14} {'FP std':>10} | {'MIP mean (ms)':>14} {'MIP std':>10} | {'MIP/FP ratio':>13}"
    print(hdr)
    print("-" * len(hdr))
    for r in rows:
        print(
            f"{r['batch_size']:>12} | "
            f"{r['fp_mean']:>12.2f}   {r['fp_std']:>8.2f}   | "
            f"{r['mip_mean']:>12.2f}   {r['mip_std']:>8.2f}   | "
            f"{r['speedup']:>11.1f}×"
        )


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def main() -> None:
    print("=" * 72)
    print("UAM Conflict Resolution — Fixed-Point vs MIP Benchmark")
    print("=" * 72)

    print("\n[Correctness verification]")
    check_correctness()

    print("\n[Benchmark 1] Single request — varying number of approved plans")
    print("(10 repeats each, new request at t=0 on same corridor)\n")
    single_rows = bench_single(plan_counts=[5, 10, 20, 40, 80], repeats=10)
    _print_table_single(single_rows)

    print("\n[Benchmark 2] Batch requests — varying batch size (N_approved=20)")
    print("(5 repeats each)\n")
    batch_rows = bench_batch(batch_sizes=[2, 5, 10, 20], repeats=5)
    _print_table_batch(batch_rows)

    # Summary
    overall_fp  = statistics.mean([r["fp_mean"]  for r in single_rows + batch_rows])
    overall_mip = statistics.mean([r["mip_mean"] for r in single_rows + batch_rows])
    print(f"\n[Summary]")
    print(f"  Fixed-Point overall mean : {overall_fp:8.2f} ms")
    print(f"  MIP         overall mean : {overall_mip:8.2f} ms")
    print(f"  MIP / FP ratio           : {overall_mip/overall_fp:.1f}×  "
          f"({'MIP slower' if overall_mip > overall_fp else 'MIP faster'})")
    print()


if __name__ == "__main__":
    main()
