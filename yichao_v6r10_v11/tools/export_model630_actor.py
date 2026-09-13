#!/usr/bin/env python3
"""Export only the deterministic model630 actor mean to a batched ONNX."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np
import onnx
import onnxruntime as ort
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from yichao_v6r10_v11 import MODEL_ORDER, MODEL_TO_ROS, PLANNER_IDENTITY
from yichao_v6r10_v11.actor import rebuild_torch_actor, sha256
from yichao_v6r10_v11.features import ACTOR_NAMES, AUDIT_NAMES

EXPECTED_CHECKPOINT_SHA256 = "81501373c08f6f4aa6bae3fdaccdd073939e5346d2b25a8e419ab35c6657ed0c"
EXPECTED_SCHEMA_SHA256 = "f2bf9c51e37f675cdb6e063cab325c0bf0eb569bb46c0fb104c075c3e6ed7cee"


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--schema", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=ROOT / "models/model630_actor.onnx")
    parser.add_argument("--sidecar", type=Path, default=ROOT / "models/model630_actor.onnx.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if sha256(args.checkpoint) != EXPECTED_CHECKPOINT_SHA256:
        raise ValueError("model630 checkpoint SHA256 mismatch")
    if sha256(args.schema) != EXPECTED_SCHEMA_SHA256:
        raise ValueError("model630 schema SHA256 mismatch")
    schema = json.loads(args.schema.read_text(encoding="utf-8"))
    if tuple(schema.get("observation_names", ())) != ACTOR_NAMES:
        raise ValueError("handoff 36D observation schema mismatch")
    if tuple(schema.get("cbf_input_names", ())) != AUDIT_NAMES:
        raise ValueError("handoff 87D audit schema mismatch")
    if schema.get("action_names") != ["left_base_target_y", "right_base_target_y"]:
        raise ValueError("handoff action mapping mismatch")
    if schema.get("action_bounds_m") != [-0.9, 0.9]:
        raise ValueError("handoff workspace mismatch")

    actor, checkpoint = rebuild_torch_actor(args.checkpoint)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.onnx.export(
        actor,
        (torch.zeros(1, 36, dtype=torch.float32),),
        args.output,
        export_params=True,
        opset_version=18,
        input_names=["observation"],
        output_names=["action"],
        dynamic_axes={"observation": {0: "batch"}, "action": {0: "batch"}},
        dynamo=False,
    )
    model = onnx.load(args.output)
    onnx.checker.check_model(model)
    session = ort.InferenceSession(str(args.output), providers=["CPUExecutionProvider"])
    generator = np.random.default_rng(630)
    cases = [
        np.zeros((1, 36), dtype=np.float32),
        np.ones((3, 36), dtype=np.float32) * 3.0,
        np.ones((3, 36), dtype=np.float32) * -3.0,
        generator.normal(0.0, 0.8, size=(64, 36)).astype(np.float32),
        generator.uniform(-3.0, 3.0, size=(17, 36)).astype(np.float32),
    ]
    maximum_error = 0.0
    parity = []
    with torch.no_grad():
        for values in cases:
            expected = actor(torch.from_numpy(values)).numpy()
            actual = session.run(["action"], {"observation": values})[0]
            error = float(np.max(np.abs(expected - actual)))
            maximum_error = max(maximum_error, error)
            parity.append({"batch": len(values), "max_abs_error": error})
    if maximum_error > 2.0e-5:
        raise RuntimeError(f"PT/ONNX parity failed: {maximum_error}")

    sidecar = {
        "format": "v6r10-model630-actor-onnx-v1",
        "planner_identity": PLANNER_IDENTITY,
        "filter_mode": "off",
        "barrier_dependency": None,
        "checkpoint_file": args.checkpoint.name,
        "checkpoint_sha256": EXPECTED_CHECKPOINT_SHA256,
        "checkpoint_iteration": int(checkpoint.get("iter", -1)),
        "schema_file": args.schema.name,
        "schema_sha256": EXPECTED_SCHEMA_SHA256,
        "observation_version": schema["observation_version"],
        "observation_schema_hash": schema["observation_schema_hash"],
        "observation_names": list(ACTOR_NAMES),
        "action_names": schema["action_names"],
        "action_bounds_m": schema["action_bounds_m"],
        "model_order": list(MODEL_ORDER),
        "ros_mapping": dict(MODEL_TO_ROS),
        "input_name": "observation",
        "output_name": "action",
        "input_dim": 36,
        "output_dim": 2,
        "audit_dim": 87,
        "audit_version": schema["cbf_input_version"],
        "audit_schema_hash": schema["cbf_input_schema_hash"],
        "audit_names": list(AUDIT_NAMES),
        "audit_active_model_input": False,
        "batch_dynamic": True,
        "network": {"hidden_dims": [256, 256], "activation": "elu"},
        "observation_normalization": {
            "frozen": True,
            "formula": "(observation - mean) / (std + 0.01)",
        },
        "onnx_opset": 18,
        "onnx_sha256": sha256(args.output),
        "parity_threshold": 2.0e-5,
        "parity_max_abs_error": maximum_error,
        "parity_cases": parity,
    }
    args.sidecar.write_text(json.dumps(sidecar, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(sidecar, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
