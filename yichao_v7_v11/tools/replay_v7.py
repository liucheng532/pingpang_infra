#!/usr/bin/env python3
"""Deterministically replay V7 feature records without ROS or publishers."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from yichao_v7_v11 import PLANNER_IDENTITY  # noqa: E402
from yichao_v7_v11.actor import OnnxActor, OnnxBarrier, sha256  # noqa: E402
from yichao_v7_v11.features import build_vectors  # noqa: E402
from yichao_v7_v11.guard import project_v7_pair  # noqa: E402


def _load_rows(path: Path) -> list[dict]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line]
    if not rows:
        raise ValueError("replay input has no records")
    return rows


def replay(source: Path, output: Path, summary_path: Path, *, benchmark_iterations: int = 35) -> dict:
    """Run each shot once, latch duplicate-shot results, and write audit JSONL."""

    actor_path = ROOT / "models/v7_planner_actor.onnx"
    barrier_path = ROOT / "models/frozen_v7_barrier_risk.onnx"
    contract_path = ROOT / "models/model_contract.json"
    actor = OnnxActor(actor_path, contract_path)
    barrier = OnnxBarrier(barrier_path, contract_path)
    rows = _load_rows(Path(source))
    latched: dict[str | int, dict] = {}
    outputs = []
    inference_latencies = []
    duplicates = 0
    for sequence, row in enumerate(rows, start=1):
        shot_id = row.get("shot_id")
        if shot_id is None:
            raise ValueError("replay record is missing shot_id")
        executed = shot_id not in latched
        if executed:
            features = build_vectors(
                row["histories"], row["normalized_joints"], row["ball"],
                row["hitter"], ball_age_s=float(row.get("ball_age_s", 0.0)),
            )
            started = time.perf_counter()
            raw = actor(features.actor)[0]
            decision = project_v7_pair(raw, features.safe, barrier, row["hitter"])
            inference_latencies.append((time.perf_counter() - started) * 1000.0)
            latched[shot_id] = {
                "actor_names": list(features.actor_names),
                "actor_observation": features.actor.tolist(),
                "safe_names": list(features.safe_names),
                "safe_observation": features.safe.tolist(),
                "decision": decision,
                "first_sequence": sequence,
            }
        else:
            duplicates += 1
        record = latched[shot_id]
        outputs.append({
            "schema": "v7-deterministic-replay-v1",
            "planner_identity": PLANNER_IDENTITY,
            "sequence": sequence,
            "shot_id": shot_id,
            "hitter": row["hitter"],
            "inference_executed": executed,
            "latched_from_sequence": record["first_sequence"],
            "actor_names": record["actor_names"],
            "actor_observation": record["actor_observation"],
            "safe_names": record["safe_names"],
            "safe_observation": record["safe_observation"],
            "decision": record["decision"],
            "commands_published": 0,
        })

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "".join(json.dumps(item, separators=(",", ":"), allow_nan=False) + "\n" for item in outputs),
        encoding="utf-8",
    )

    # Benchmark the actual full 2603-candidate branch, not a nominal bypass.
    safe = np.zeros(87, dtype=np.float32)
    timings = []
    candidate_counts = []
    for _ in range(max(benchmark_iterations, 6)):
        started = time.perf_counter()
        measured = project_v7_pair(np.zeros(2, dtype=np.float32), safe, barrier, "left")
        timings.append((time.perf_counter() - started) * 1000.0)
        candidate_counts.append(measured["cbf_candidate_count"])
    measured_timings = timings[5:]
    p95 = float(np.percentile(measured_timings, 95))
    maximum = float(max(measured_timings))
    summary = {
        "schema": "v7-deterministic-replay-summary-v1",
        "planner_identity": PLANNER_IDENTITY,
        "source": (
            Path(source).resolve().relative_to(ROOT).as_posix()
            if Path(source).resolve().is_relative_to(ROOT)
            else str(Path(source).resolve())
        ),
        "source_sha256": sha256(source),
        "actor_sha256": sha256(actor_path),
        "barrier_sha256": sha256(barrier_path),
        "input_events": len(rows),
        "distinct_shots": len(latched),
        "decisions_executed": len(latched),
        "duplicate_shot_events": duplicates,
        "commands_published": 0,
        "latched_targets_stable": all(
            item["decision"] == outputs[item["latched_from_sequence"] - 1]["decision"]
            for item in outputs
        ),
        "decision_latency_ms": {
            "maximum": max(inference_latencies),
            "p95": float(np.percentile(inference_latencies, 95)),
        },
        "benchmark": {
            "candidate_count": min(candidate_counts),
            "iterations_measured": len(measured_timings),
            "p95_ms": p95,
            "maximum_ms": maximum,
            "budget_p95_ms": 20.0,
            "budget_maximum_ms": 50.0,
            "passed": min(candidate_counts) == 2603 and p95 <= 20.0 and maximum <= 50.0,
        },
    }
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--benchmark-iterations", type=int, default=35)
    args = parser.parse_args()
    print(json.dumps(replay(args.source, args.output, args.summary, benchmark_iterations=args.benchmark_iterations), indent=2))


if __name__ == "__main__":
    main()
