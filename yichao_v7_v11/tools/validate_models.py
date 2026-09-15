#!/usr/bin/env python3
"""Validate V7 PT/JIT/ONNX parity, ABI, and full-candidate CPU latency."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np
import onnxruntime as ort
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from yichao_v7_v11 import PLANNER_IDENTITY  # noqa: E402
from yichao_v7_v11.actor import (  # noqa: E402
    OnnxActor,
    OnnxBarrier,
    rebuild_torch_actor,
    rebuild_torch_barrier,
)
from yichao_v7_v11.guard import project_v7_pair  # noqa: E402


def validate(iterations: int = 35) -> dict:
    contract = ROOT / "models/model_contract.json"
    actor = OnnxActor(ROOT / "models/v7_planner_actor.onnx", contract)
    barrier = OnnxBarrier(ROOT / "models/frozen_v7_barrier_risk.onnx", contract)
    actor_pt, actor_checkpoint = rebuild_torch_actor(ROOT / "models/model_190.pt")
    actor_jit = torch.jit.load(
        str(ROOT / "models/v7_planner_actor_jit.pt"), map_location="cpu"
    ).eval()
    barrier_pt, cbf_checkpoint = rebuild_torch_barrier(ROOT / "models/frozen_cbf_v7.pt")

    generator = np.random.default_rng(190)
    maximum = {
        "actor_pt_vs_jit": 0.0,
        "actor_pt_vs_onnx": 0.0,
        "cbf_head_probabilities": 0.0,
        "cbf_member_risk": 0.0,
        "cbf_conservative_risk": 0.0,
    }
    batches = []
    for batch in (1, 7, 32):
        observation = generator.normal(size=(batch, 29)).astype(np.float32)
        safe = generator.normal(size=(batch, 87)).astype(np.float32)
        action = generator.uniform(-1.0, 1.0, size=(batch, 2)).astype(np.float32)
        with torch.no_grad():
            pt_actor = actor_pt(torch.from_numpy(observation)).numpy()
            jit_actor = actor_jit(torch.from_numpy(observation)).numpy()
            pt_cbf = tuple(
                value.numpy()
                for value in barrier_pt(torch.from_numpy(safe), torch.from_numpy(action))
            )
        onnx_actor = actor(observation)
        onnx_cbf = barrier(safe, action)
        errors = {
            "actor_pt_vs_jit": float(np.max(np.abs(pt_actor - jit_actor))),
            "actor_pt_vs_onnx": float(np.max(np.abs(pt_actor - onnx_actor))),
        }
        for reference, name in zip(
            pt_cbf,
            ("cbf_head_probabilities", "cbf_member_risk", "cbf_conservative_risk"),
        ):
            output_name = name.removeprefix("cbf_")
            errors[name] = float(np.max(np.abs(reference - onnx_cbf[output_name])))
        for name, value in errors.items():
            maximum[name] = max(maximum[name], value)
        batches.append({"batch": batch, "maximum_absolute_errors": errors})

    timings = []
    counts = []
    safe = np.zeros(87, dtype=np.float32)
    for _ in range(max(6, int(iterations))):
        started = time.perf_counter()
        decision = project_v7_pair(np.zeros(2, dtype=np.float32), safe, barrier, "left")
        timings.append((time.perf_counter() - started) * 1000.0)
        counts.append(decision["cbf_candidate_count"])
    measured = timings[5:]
    p95 = float(np.percentile(measured, 95))
    maximum_latency = float(max(measured))
    tolerance = 2.0e-5
    return {
        "schema": "v7-model-validation-v1",
        "planner_identity": PLANNER_IDENTITY,
        "platform": platform.platform(),
        "python": platform.python_version(),
        "numpy": np.__version__,
        "torch": torch.__version__,
        "onnxruntime": ort.__version__,
        "actor_update": actor_checkpoint["iter"],
        "cbf_threshold": cbf_checkpoint["calibrated_threshold"],
        "abi": {
            "actor": {"input": ["batch", 29], "output": ["batch", 2]},
            "cbf": {
                "inputs": [["batch", 87], ["batch", 2]],
                "outputs": [["batch", 3, 3], ["batch", 3], ["batch"]],
            },
        },
        "parity": {
            "absolute_tolerance": tolerance,
            "maximum_absolute_errors": maximum,
            "batches": batches,
            "passed": max(maximum.values()) <= tolerance,
        },
        "full_candidate_latency": {
            "candidate_count": min(counts),
            "iterations_measured": len(measured),
            "p50_ms": float(np.percentile(measured, 50)),
            "p95_ms": p95,
            "maximum_ms": maximum_latency,
            "budget_p95_ms": 20.0,
            "budget_maximum_ms": 50.0,
            "passed": min(counts) == 2603 and p95 <= 20.0 and maximum_latency <= 50.0,
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=35)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = validate(args.iterations)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["parity"]["passed"] and report["full_candidate_latency"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
