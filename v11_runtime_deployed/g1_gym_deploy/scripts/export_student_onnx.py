#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn


STUDENT_OBS_DIM = 1666
ACTION_DIM = 29
PHASE_ORDER = ("HIT", "POST_DELAY", "OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD")
OUTPUT_BASENAME = "student_m73000_hold6_1666_model23000.onnx"
EXPECTED_CHECKPOINT_SHA256 = "f2d31b802a37c014971bf83a0b9d1a61e2b8727b42ffbed3dd153b7e4d67127d"
EXPECTED_HIT_TEACHER_SHA256 = "5a4e650813f227f704ae2040df73ece8da11dae1746bfc8b6e982c68e9efc94c"
EXPECTED_MOVE_TEACHER_SHA256 = "9255177c1ae6eced0a88547ea0d362cced22b3fbda2bd044221f43846345f260"
EXPECTED_HIT_MANIFEST_SHA256 = "157bfe81e828f42661e3414a720791ef1968b8488841d5d354dcdc40d7726f03"
EXPECTED_MOVE_MANIFEST_SHA256 = "2684d5cfb1a4b9d10a4eb8df3b4a945b20662fcc06bdde599bd32da393a4e90d"


class StudentPolicy(nn.Module):
    def __init__(self):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(STUDENT_OBS_DIM, 768),
            nn.ELU(),
            nn.Linear(768, 384),
            nn.ELU(),
            nn.Linear(384, 192),
            nn.ELU(),
            nn.Linear(192, ACTION_DIM),
        )

    def forward(self, observation):
        return self.actor(observation)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def export_student(
    checkpoint_path: Path,
    output_dir: Path,
    force=False,
    expected_checkpoint_sha256=EXPECTED_CHECKPOINT_SHA256,
    parity_observations: Path | None = None,
):
    checkpoint_path = checkpoint_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Student checkpoint not found: {checkpoint_path}")
    output_dir.mkdir(parents=True, exist_ok=True)
    output_onnx = output_dir / OUTPUT_BASENAME
    output_sidecar = output_dir / f"{OUTPUT_BASENAME}.json"
    if not force and (output_onnx.exists() or output_sidecar.exists()):
        raise FileExistsError(f"Deployment output already exists under {output_dir}; pass --force to replace it.")

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    checkpoint_sha256 = sha256_file(checkpoint_path)
    if checkpoint_sha256 != expected_checkpoint_sha256:
        raise RuntimeError(
            f"Checkpoint SHA256 is {checkpoint_sha256}, expected {expected_checkpoint_sha256}."
        )
    if "student_state_dict" not in checkpoint:
        raise RuntimeError("Checkpoint is not a pure distillation student: missing student_state_dict.")
    checkpoint_obs_dim = checkpoint.get("student_obs_dim", STUDENT_OBS_DIM)
    if int(checkpoint_obs_dim) != STUDENT_OBS_DIM:
        raise RuntimeError(f"Checkpoint student_obs_dim is {checkpoint_obs_dim}, expected {STUDENT_OBS_DIM}.")
    if int(checkpoint.get("iteration", -1)) != 23000:
        raise RuntimeError(f"Checkpoint iteration is {checkpoint.get('iteration')}, expected 23000.")
    if tuple(checkpoint.get("phase_order", ())) != PHASE_ORDER:
        raise RuntimeError(f"Checkpoint phase_order is {checkpoint.get('phase_order')}, expected {PHASE_ORDER}.")
    if checkpoint.get("hit_teacher_sha256") != EXPECTED_HIT_TEACHER_SHA256:
        raise RuntimeError("Checkpoint hit-teacher SHA256 does not match the frozen V00 teacher.")
    if checkpoint.get("move_teacher_sha256") != EXPECTED_MOVE_TEACHER_SHA256:
        raise RuntimeError("Checkpoint move-teacher SHA256 does not match model_73000.")

    student = StudentPolicy().eval()
    student.load_state_dict(checkpoint["student_state_dict"], strict=True)
    student.requires_grad_(False)
    example = torch.zeros(1, STUDENT_OBS_DIM, dtype=torch.float32)
    torch.onnx.export(
        student,
        example,
        output_onnx,
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=17,
    )

    session = ort.InferenceSession(str(output_onnx), providers=["CPUExecutionProvider"])
    if parity_observations is None:
        generator = np.random.default_rng(0)
        test_obs = 0.5 * generator.standard_normal((8, STUDENT_OBS_DIM), dtype=np.float32)
        parity_source = "bounded_synthetic"
    else:
        parity_path = parity_observations.expanduser().resolve()
        test_obs = np.asarray(np.load(parity_path), dtype=np.float32)
        if test_obs.ndim != 2 or test_obs.shape[1] != STUDENT_OBS_DIM:
            raise ValueError(
                f"Parity observations must have shape [N, {STUDENT_OBS_DIM}], got {test_obs.shape}."
            )
        if not np.isfinite(test_obs).all():
            raise ValueError("Parity observations contain non-finite values.")
        parity_source = str(parity_path)
    with torch.no_grad():
        torch_actions = student(torch.from_numpy(test_obs)).numpy()
    onnx_actions = session.run(None, {session.get_inputs()[0].name: test_obs})[0]
    parity_error = float(np.max(np.abs(torch_actions - onnx_actions)))
    if parity_error > 1.0e-5:
        raise RuntimeError(f"PyTorch/ONNX action parity failed: max error={parity_error:.3e}.")

    frozen_checkpoint = output_dir / f"student_iteration_{int(checkpoint.get('iteration', 0)):06d}.pt"
    if force or not frozen_checkpoint.exists():
        shutil.copy2(checkpoint_path, frozen_checkpoint)
    metadata = {
        "format": "doubles_student_onnx_v1",
        "input_dim": STUDENT_OBS_DIM,
        "output_dim": ACTION_DIM,
        "history_length": 10,
        "history_order": "oldest_to_newest_term_major",
        "phase_order": list(PHASE_ORDER),
        "iteration": int(checkpoint.get("iteration", 0)),
        "checkpoint_file": frozen_checkpoint.name,
        "checkpoint_sha256": sha256_file(frozen_checkpoint),
        "onnx_sha256": sha256_file(output_onnx),
        "hit_teacher_sha256": checkpoint["hit_teacher_sha256"],
        "move_teacher_sha256": checkpoint["move_teacher_sha256"],
        "hit_motion_manifest_sha256": EXPECTED_HIT_MANIFEST_SHA256,
        "move_motion_manifest_sha256": EXPECTED_MOVE_MANIFEST_SHA256,
        "student_terms": {
            "strike_time": 1,
            "target_vel": 3,
            "racket_target_hit": 3,
            "target_base_position": 2,
            "task_anchor_base": 3,
            "robot_orientation": 6,
            "base_ang_vel": 3,
            "joint_pos": 29,
            "joint_vel": 29,
            "actions": 29,
            "command": 58,
            "phase_current": 6,
        },
        "parity_observation_source": parity_source,
        "parity_observation_count": int(test_obs.shape[0]),
        "pytorch_onnx_max_action_error": parity_error,
    }
    output_sidecar.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return metadata


def parse_args():
    parser = argparse.ArgumentParser(description="Export a frozen pure-distillation student as 1666->29 ONNX.")
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--parity-observations", type=Path)
    parser.add_argument("--force", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print(
        json.dumps(
            export_student(
                args.checkpoint,
                args.output_dir,
                force=args.force,
                parity_observations=args.parity_observations,
            ),
            indent=2,
        )
    )
