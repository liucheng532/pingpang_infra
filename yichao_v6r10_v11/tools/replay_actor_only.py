#!/usr/bin/env python3
"""Replay frozen V3 source records through V6R10 feature/actor/guard only.

This tool never imports ROS and never creates a command publisher.  A pair is
treated as accepted only when the source row records both legacy robot IDs in
``relay.accepted``; that convention is used solely to reconstruct the next
shot's previous-target and interval features.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from yichao_v6r10_v11 import HOME_PAIR, MODEL_ORDER, PLANNER_IDENTITY
from yichao_v6r10_v11.actor import OnnxActor
from yichao_v6r10_v11.features import JointNormalizer, build_vectors, interval_metadata
from yichao_v6r10_v11.guard import project_actor_pair


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--benchmark-iterations", type=int, default=2000)
    return parser.parse_args()


def source_record(row):
    record = row.get("source_record", row)
    if not isinstance(record, dict) or not isinstance(record.get("robots"), dict):
        raise ValueError("row has no replayable source_record")
    return record


def model_robots(record):
    by_role = {item.get("role"): item for item in record["robots"].values()}
    if set(by_role) != {"table_left", "table_right"}:
        raise ValueError("source robot roles do not match table_left/table_right")
    return (by_role["table_left"], by_role["table_right"])


def selected_hitter(row):
    value = (row.get("relay") or {}).get("hitter")
    if value in {"198", "table_left", "left"}:
        return "left"
    if value in {"66", "table_right", "right"}:
        return "right"
    raise ValueError("source row has no explicit Fixed-selected hitter")


def make_features(row, previous, previous_time):
    record = source_record(row)
    left, right = model_robots(record)
    for item in (left, right):
        if tuple(item.get("joint_order", ())) != JointNormalizer.JOINT_NAMES:
            raise ValueError("source joint order is not the frozen LAB 29D order")
    ball_source = record["ball"]
    ball = {
        "position": ball_source["position"],
        "velocity": ball_source["velocity"],
        "predicted_strike_position": ball_source["strike_position"],
        "racket_velocity": ball_source["racket_velocity"],
        "time_to_strike_s": ball_source["time_to_strike_s"],
        "shot_id": record["shot_id"],
        "valid": ball_source["valid"],
    }
    received = float(record["now"])
    interval = interval_metadata(
        1.65 if previous_time is None else received - previous_time,
        first=previous_time is None,
    )
    normalizer = JointNormalizer()
    features = build_vectors(
        [left["base_history"], right["base_history"]],
        [normalizer.normalize(left["q"]), normalizer.normalize(right["q"])],
        ball,
        previous,
        selected_hitter(row),
        interval,
        acceleration=ball_source.get("acceleration", (0.0, 0.0, -9.81)),
        current_bases=[left["base_position"], right["base_position"]],
    )
    return features, received


def decide(actor, row, previous, previous_time):
    started = time.perf_counter_ns()
    features, received = make_features(row, previous, previous_time)
    raw = actor(features.actor)[0]
    guarded = project_actor_pair(raw, selected_hitter(row)).to_dict()
    latency_ms = (time.perf_counter_ns() - started) / 1.0e6
    return features, guarded, received, latency_ms


def benchmark(actor, row, previous, previous_time, iterations):
    if iterations <= 0:
        raise ValueError("benchmark iterations must be positive")
    for _ in range(50):
        decide(actor, row, previous, previous_time)
    timings = np.empty(iterations, dtype=np.float64)
    for index in range(iterations):
        timings[index] = decide(actor, row, previous, previous_time)[-1]
    return {
        "iterations": iterations,
        "scope": "36D_and_87D_feature_build_plus_onnx_actor_plus_hard_guard",
        "p50_ms": float(np.quantile(timings, 0.50)),
        "p95_ms": float(np.quantile(timings, 0.95)),
        "p99_ms": float(np.quantile(timings, 0.99)),
        "max_ms": float(np.max(timings)),
        "threshold_p95_ms": 20.0,
        "passed": bool(np.quantile(timings, 0.95) < 20.0),
    }


def replay(input_path, output_path, summary_path, benchmark_iterations=2000):
    actor = OnnxActor(ROOT / "models/model630_actor.onnx", ROOT / "models/model630_actor.onnx.json")
    rows = [json.loads(line) for line in input_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if not rows:
        raise ValueError("empty replay input")
    selected = []
    seen = set()
    for row in rows:
        shot_id = source_record(row).get("shot_id")
        if shot_id is not None and shot_id not in seen:
            seen.add(shot_id)
            selected.append(row)
    previous = np.asarray(HOME_PAIR, dtype=np.float32)
    previous_time = None
    records = []
    latencies = []
    for index, row in enumerate(selected):
        features, guarded, received, latency_ms = decide(actor, row, previous, previous_time)
        accepted = set((row.get("relay") or {}).get("accepted", ())) >= {"198", "66"}
        if accepted and guarded["valid"]:
            previous = np.asarray(guarded["applied_pair_m"], dtype=np.float32)
            previous_time = received
        source = source_record(row)
        records.append(
            {
                "schema": "v6r10-actor-only-offline-replay-v1",
                "record_index": index,
                "source_row_index": row.get("record_index"),
                "source_provenance": source.get("provenance"),
                "shot_id": source["shot_id"],
                "fixed_selected_hitter": selected_hitter(row),
                "source_pair_fully_accepted": accepted,
                "planner_identity": PLANNER_IDENTITY,
                "model_order": list(MODEL_ORDER),
                "filter_mode": "off",
                "barrier_risk": "not_evaluated",
                "ensemble_risk": "not_evaluated",
                "actor_observation": features.actor.tolist(),
                "audit_observation": features.audit.tolist(),
                "actor_names": list(features.actor_names),
                "audit_names": list(features.audit_names),
                "feature_record": features.record,
                "decision": guarded,
                "processing_ms": latency_ms,
            }
        )
        latencies.append(latency_ms)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        "".join(json.dumps(item, separators=(",", ":"), allow_nan=False) + "\n" for item in records),
        encoding="utf-8",
    )
    perf = benchmark(actor, selected[0], HOME_PAIR, None, benchmark_iterations)
    summary = {
        "schema": "v6r10-actor-only-offline-replay-summary-v1",
        "planner_identity": PLANNER_IDENTITY,
        "input": str(input_path),
        "input_sha256": sha256(input_path),
        "source_rows": len(rows),
        "distinct_shots": len(records),
        "valid_decisions": sum(item["decision"]["valid"] for item in records),
        "guard_interventions": sum(item["decision"]["guard_intervened"] for item in records),
        "fully_accepted_source_pairs": sum(item["source_pair_fully_accepted"] for item in records),
        "filter_mode": "off",
        "barrier_risk": "not_evaluated",
        "ensemble_risk": "not_evaluated",
        "replay_processing_p95_ms": float(np.quantile(latencies, 0.95)),
        "benchmark": perf,
        "output": str(output_path),
        "output_sha256": sha256(output_path),
        "commands_published": 0,
        "ros_imported": False,
    }
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def main():
    args = arguments()
    summary = replay(args.input, args.output, args.summary, args.benchmark_iterations)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary["benchmark"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
