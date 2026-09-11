#!/usr/bin/env python3
"""Run large offline Reset Monte Carlo campaigns by sharding the C++ plant model."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any, Dict, List, Optional, Tuple

CASE_PREFIX = "XGC_RESET_MC_CASE "
SUMMARY_PREFIX = "XGC_RESET_MC_SUMMARY "


def find_binary(explicit: Optional[str]) -> str:
    if explicit:
        return explicit
    env_binary = os.environ.get("XGC_RESET_MONTE_CARLO_BIN")
    if env_binary:
        return env_binary
    candidate = shutil.which("reset_monte_carlo")
    if candidate:
        return candidate
    for path in (
        Path("devel/lib/ugv_reset_safety/reset_monte_carlo"),
        Path("build/ugv_reset_safety/reset_monte_carlo"),
    ):
        if path.is_file() and os.access(path, os.X_OK):
            return str(path)
    raise SystemExit(
        "reset_monte_carlo not found; build the catkin workspace or pass --binary"
    )


def distribute(total: int, shards: int) -> List[int]:
    base, remainder = divmod(total, shards)
    return [base + (1 if index < remainder else 0) for index in range(shards)]


def run_shard(
    binary: str,
    mode: str,
    cases: int,
    seed: int,
    robots_min: int,
    robots_max: int,
    obstacles_min: int,
    obstacles_max: int,
    max_time: float,
) -> Dict[str, Any]:
    if cases == 0:
        return {"summary": None, "failures": [], "stdout": "", "returncode": 0}
    env = os.environ.copy()
    env.update(
        {
            "XGC_RESET_MC_MODE": mode,
            "XGC_RESET_MC_CASES": str(cases),
            "XGC_RESET_MC_SEED": str(seed),
            "XGC_RESET_MC_ROBOTS_MIN": str(robots_min),
            "XGC_RESET_MC_ROBOTS_MAX": str(robots_max),
            "XGC_RESET_MC_OBSTACLES_MIN": str(obstacles_min),
            "XGC_RESET_MC_OBSTACLES_MAX": str(obstacles_max),
            "XGC_RESET_MC_MAX_TIME": str(max_time),
        }
    )
    completed = subprocess.run(
        [binary, "--gtest_filter=ResetMonteCarlo.Campaign", "--gtest_color=no"],
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    failures: List[Dict[str, Any]] = []
    summary: Optional[Dict[str, Any]] = None
    for line in completed.stdout.splitlines():
        if line.startswith(CASE_PREFIX):
            failures.append(json.loads(line[len(CASE_PREFIX) :]))
        elif line.startswith(SUMMARY_PREFIX):
            summary = json.loads(line[len(SUMMARY_PREFIX) :])
    if summary is None:
        raise RuntimeError(
            f"shard mode={mode} seed={seed} produced no summary\n{completed.stdout}"
        )
    return {
        "summary": summary,
        "failures": failures,
        "stdout": completed.stdout,
        "returncode": completed.returncode,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--binary")
    parser.add_argument("--cases-per-mode", type=int, default=1000)
    parser.add_argument("--shards", type=int, default=max(1, min(8, os.cpu_count() or 1)))
    parser.add_argument("--seed", type=int, default=20260912)
    parser.add_argument("--modes", default="scout,mecanum,mixed")
    parser.add_argument("--robots-min", type=int, default=1)
    parser.add_argument("--robots-max", type=int, default=6)
    parser.add_argument("--obstacles-min", type=int, default=0)
    parser.add_argument("--obstacles-max", type=int, default=5)
    parser.add_argument("--max-time", type=float, default=240.0)
    parser.add_argument("--output-prefix", default="reset_monte_carlo")
    args = parser.parse_args()

    if args.cases_per_mode <= 0 or args.shards <= 0:
        parser.error("--cases-per-mode and --shards must be positive")
    if not (1 <= args.robots_min <= args.robots_max <= 8):
        parser.error("robot range must satisfy 1 <= min <= max <= 8")
    if not (0 <= args.obstacles_min <= args.obstacles_max <= 8):
        parser.error("obstacle range must satisfy 0 <= min <= max <= 8")

    binary = find_binary(args.binary)
    modes = [mode.strip() for mode in args.modes.split(",") if mode.strip()]
    for mode in modes:
        if mode not in {"scout", "mecanum", "mixed"}:
            parser.error(f"unsupported mode: {mode}")

    jobs: List[Tuple[str, int, int]] = []
    seed_cursor = args.seed
    for mode_index, mode in enumerate(modes):
        shard_sizes = distribute(args.cases_per_mode, args.shards)
        mode_seed = args.seed + mode_index * 100_000_000
        offset = 0
        for shard_size in shard_sizes:
            jobs.append((mode, shard_size, mode_seed + offset))
            offset += shard_size
        seed_cursor = max(seed_cursor, mode_seed + offset)

    results: List[Dict[str, Any]] = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.shards) as pool:
        futures = [
            pool.submit(
                run_shard,
                binary,
                mode,
                cases,
                seed,
                args.robots_min,
                args.robots_max,
                args.obstacles_min,
                args.obstacles_max,
                args.max_time,
            )
            for mode, cases, seed in jobs
            if cases > 0
        ]
        for future in concurrent.futures.as_completed(futures):
            result = future.result()
            results.append(result)
            summary = result["summary"]
            print(json.dumps(summary, sort_keys=True))

    totals = {
        "cases": 0,
        "completed": 0,
        "no_route": 0,
        "timeout": 0,
        "filter_failure": 0,
        "collision": 0,
        "contract_failure": 0,
    }
    minimum_clearance = float("inf")
    maximum_elapsed = 0.0
    failures: List[Dict[str, Any]] = []
    bad_process = False
    per_mode: Dict[str, Dict[str, int]] = {}
    for result in results:
        summary = result["summary"]
        mode = summary["mode"]
        mode_counts = per_mode.setdefault(mode, {key: 0 for key in totals})
        for key in totals:
            value = int(summary[key])
            totals[key] += value
            mode_counts[key] += value
        minimum_clearance = min(minimum_clearance, float(summary["min_clearance"]))
        maximum_elapsed = max(maximum_elapsed, float(summary["max_elapsed"]))
        failures.extend(result["failures"])
        bad_process = bad_process or result["returncode"] != 0

    aggregate = {
        **totals,
        "completion_rate": totals["completed"] / totals["cases"] if totals["cases"] else 0.0,
        "min_clearance": minimum_clearance,
        "max_elapsed": maximum_elapsed,
        "per_mode": per_mode,
        "seed_begin": args.seed,
        "seed_end_exclusive": seed_cursor,
        "robots": [args.robots_min, args.robots_max],
        "obstacles": [args.obstacles_min, args.obstacles_max],
        "max_time": args.max_time,
    }

    prefix = Path(args.output_prefix)
    summary_path = prefix.with_suffix(".json")
    failure_path = prefix.with_name(prefix.name + "_failures.jsonl")
    summary_path.write_text(json.dumps(aggregate, indent=2, sort_keys=True) + "\n")
    with failure_path.open("w") as stream:
        for failure in sorted(failures, key=lambda item: (item["mode"], item["seed"])):
            stream.write(json.dumps(failure, sort_keys=True) + "\n")

    print(json.dumps(aggregate, indent=2, sort_keys=True))
    print(f"wrote {summary_path} and {failure_path}")
    # Collision or controller/plant contract violations make the C++ shard fail.
    # Timeouts, no-route and QP infeasibility remain measured campaign outcomes.
    return 1 if bad_process else 0


if __name__ == "__main__":
    sys.exit(main())
