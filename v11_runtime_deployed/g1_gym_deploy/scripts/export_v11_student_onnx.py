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
OUTPUT_BASENAME = "student_v11_r2i299_commonhold_1666_resume42500.onnx"
EXPECTED_CHECKPOINT_SHA256 = "04461633b8ad75a2b173ceacf34257fae437cbf935f6effe8b279464586bea12"
EXPECTED_HIT_TEACHER_SHA256 = "5a4e650813f227f704ae2040df73ece8da11dae1746bfc8b6e982c68e9efc94c"
EXPECTED_MOVE_TEACHER_SHA256 = "5bcf39a8a27b4dd831068711b5a7b823867494087dedb6f1312d37da8e7ae2af"
EXPECTED_MOVE_TEACHER_CONTRACT = "v11_commonhold_fkupright_rawwrist_r2_model299"
EXPECTED_STUDENT_SOURCE_COMMIT = "2efd2c28ee1a021b574bc604aa20d5d76bba24f7"
EXPECTED_MOVE_TEACHER_SOURCE_COMMIT = "190ec6bd1d56de123924a1ffcdab1e6a8a10c79a"
EXPECTED_RESUME_SOURCE_SHA256 = "10f5ba7fb24d354256d2952b283180eed89c7baf42f31bc96f1359098f1cd76d"
EXPECTED_COMMON_HOLD_SHA256 = "9565a8ed1ba22cfc0d759d0a8327dc7dd37989c242d4f4337dcc79c796ea3a8f"
EXPECTED_HIT_MANIFEST_SHA256 = "157bfe81e828f42661e3414a720791ef1968b8488841d5d354dcdc40d7726f03"
EXPECTED_MOVE_MANIFEST_SHA256 = "2684d5cfb1a4b9d10a4eb8df3b4a945b20662fcc06bdde599bd32da393a4e90d"
EXPECTED_MOVE_EXCLUSION_SHA256 = "82045bd01817022444bb2cde0f391123c840d197f8d15ae7b6e35793dbb17005"
EXCLUDED_MOVE_SOURCE_IDS = [41, 47, 69, 78, 118, 119, 138, 139]


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


def _validate_checkpoint(checkpoint: dict, checkpoint_sha256: str) -> None:
    expected = {
        "iteration": 42500,
        "format": "doubles_distillation_v3_hold6",
        "student_obs_dim": STUDENT_OBS_DIM,
        "hit_teacher_obs_dim": 1670,
        "move_teacher_obs_dim": 1720,
        "phase_order": list(PHASE_ORDER),
        "hit_teacher_sha256": EXPECTED_HIT_TEACHER_SHA256,
        "move_teacher_sha256": EXPECTED_MOVE_TEACHER_SHA256,
        "move_teacher_contract": EXPECTED_MOVE_TEACHER_CONTRACT,
        "code_commit": EXPECTED_STUDENT_SOURCE_COMMIT,
        "teacher_transition_contract": "reference_end_outward_return_v1",
        "distill_transition_contract": "v9_stable_or_reference_end_v1",
        "teacher_hold_x_contract": "latch_phase_entry_then_feedback",
        "distill_hold_x_contract": "latch_phase_entry_then_feedback",
    }
    mismatches = {
        key: {"expected": value, "actual": checkpoint.get(key)}
        for key, value in expected.items()
        if checkpoint.get(key) != value
    }
    if checkpoint_sha256 != EXPECTED_CHECKPOINT_SHA256:
        mismatches["checkpoint_sha256"] = {
            "expected": EXPECTED_CHECKPOINT_SHA256,
            "actual": checkpoint_sha256,
        }
    action_contract = checkpoint.get("action_contract") or {}
    if action_contract != {
        "clip": 10.0,
        "teacher_target": "clipped_before_behavior_cloning",
        "student_prediction": "raw_with_loss_gradient",
        "rollout": "clipped_after_teacher_student_blend",
    }:
        mismatches["action_contract"] = {"actual": action_contract}
    resume_source = checkpoint.get("resume_source") or {}
    if (
        resume_source.get("sha256") != EXPECTED_RESUME_SOURCE_SHA256
        or resume_source.get("iteration") != 8000
        or resume_source.get("mode") != "full_student_optimizer_absolute_iteration"
    ):
        mismatches["resume_source"] = {"actual": resume_source}
    move_teacher = checkpoint.get("move_teacher_checkpoint_contract") or {}
    if (
        move_teacher.get("code_commit") != EXPECTED_MOVE_TEACHER_SOURCE_COMMIT
        or move_teacher.get("move_teacher_contract")
        != "v10_xrecovery_refend_commonhold_fkupright_rawwrist_r2"
    ):
        mismatches["move_teacher_checkpoint_contract"] = {"actual": move_teacher}
    common_hold = checkpoint.get("distill_common_hold_contract") or {}
    if common_hold.get("asset_sha256") != EXPECTED_COMMON_HOLD_SHA256:
        mismatches["distill_common_hold_contract"] = {"actual": common_hold}
    move_pool = checkpoint.get("move_pool_contract") or {}
    if (
        move_pool.get("manifest_sha256") != EXPECTED_MOVE_MANIFEST_SHA256
        or move_pool.get("exclusion_sha256") != EXPECTED_MOVE_EXCLUSION_SHA256
        or move_pool.get("excluded_source_ids") != EXCLUDED_MOVE_SOURCE_IDS
        or move_pool.get("motion_count") != 147
    ):
        mismatches["move_pool_contract"] = {"actual": move_pool}
    if mismatches:
        raise RuntimeError(f"V11 deployment checkpoint contract mismatch: {mismatches}")


def export_student(
    checkpoint_path: Path,
    common_hold_path: Path,
    output_dir: Path,
    parity_observations: Path | None = None,
    force: bool = False,
) -> dict:
    checkpoint_path = checkpoint_path.expanduser().resolve()
    common_hold_path = common_hold_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()
    checkpoint_sha256 = sha256_file(checkpoint_path)
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
    _validate_checkpoint(checkpoint, checkpoint_sha256)
    if sha256_file(common_hold_path) != EXPECTED_COMMON_HOLD_SHA256:
        raise RuntimeError("V11 common-HOLD asset SHA256 mismatch.")

    output_dir.mkdir(parents=True, exist_ok=True)
    output_onnx = output_dir / OUTPUT_BASENAME
    output_sidecar = output_dir / f"{OUTPUT_BASENAME}.json"
    frozen_checkpoint = output_dir / "student_iteration_042500.pt"
    frozen_common_hold = output_dir / common_hold_path.name
    if not force and any(path.exists() for path in (output_onnx, output_sidecar, frozen_checkpoint)):
        raise FileExistsError(f"V11 deployment output already exists under {output_dir}.")

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
        parity = 0.5 * generator.standard_normal((8, STUDENT_OBS_DIM), dtype=np.float32)
        parity_source = "bounded_synthetic_seed0"
    else:
        parity_path = parity_observations.expanduser().resolve()
        parity = np.asarray(np.load(parity_path), dtype=np.float32)
        if parity.ndim != 2 or parity.shape[1] != STUDENT_OBS_DIM or not np.isfinite(parity).all():
            raise ValueError(f"Invalid V11 parity observations: {parity.shape}")
        parity_source = str(parity_path)
    with torch.no_grad():
        expected = student(torch.from_numpy(parity)).numpy()
    actual = session.run(None, {session.get_inputs()[0].name: parity})[0]
    parity_error = float(np.max(np.abs(expected - actual)))
    if parity_error >= 1.0e-5:
        raise RuntimeError(f"V11 PT/ONNX parity failed: {parity_error:.8g}")

    shutil.copy2(checkpoint_path, frozen_checkpoint)
    shutil.copy2(common_hold_path, frozen_common_hold)
    move_pool = dict(checkpoint["move_pool_contract"])
    move_pool["active_count"] = int(move_pool["motion_count"])
    metadata = {
        "format": "doubles_student_onnx_v11_commonhold_v1",
        "input_dim": STUDENT_OBS_DIM,
        "output_dim": ACTION_DIM,
        "history_length": 10,
        "history_order": "oldest_to_newest_term_major",
        "phase_order": list(PHASE_ORDER),
        "iteration": 42500,
        "checkpoint_file": frozen_checkpoint.name,
        "checkpoint_sha256": sha256_file(frozen_checkpoint),
        "onnx_sha256": sha256_file(output_onnx),
        "common_hold_pose_file": frozen_common_hold.name,
        "common_hold_pose_sha256": sha256_file(frozen_common_hold),
        "student_source_commit": checkpoint["code_commit"],
        "resume_source": checkpoint["resume_source"],
        "hit_teacher_sha256": checkpoint["hit_teacher_sha256"],
        "move_teacher_sha256": checkpoint["move_teacher_sha256"],
        "move_teacher_contract": checkpoint["move_teacher_contract"],
        "move_teacher_source_commit": checkpoint["move_teacher_checkpoint_contract"]["code_commit"],
        "teacher_transition_contract": checkpoint["teacher_transition_contract"],
        "distill_transition_contract": checkpoint["distill_transition_contract"],
        "teacher_hold_x_contract": checkpoint["teacher_hold_x_contract"],
        "distill_hold_x_contract": checkpoint["distill_hold_x_contract"],
        "action_contract": checkpoint["action_contract"],
        "numerical_isolation_contract": checkpoint["numerical_isolation_contract"],
        "x_recovery_contract": checkpoint["x_recovery_contract"],
        "move_pool_contract": move_pool,
        "distill_common_hold_contract": checkpoint["distill_common_hold_contract"],
        "hit_motion_manifest_sha256": EXPECTED_HIT_MANIFEST_SHA256,
        "move_motion_manifest_sha256": EXPECTED_MOVE_MANIFEST_SHA256,
        "deployment_x_contract": {
            "kind": "startup_relative_safe_band_v1",
            "canonical_center_m": 0.18,
            "relative_half_width_m": 0.04,
            "target_base_x_clip_m": 0.04,
        },
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
        "parity_observation_count": int(parity.shape[0]),
        "pytorch_onnx_max_action_error": parity_error,
    }
    output_sidecar.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
    return metadata


def parse_args():
    parser = argparse.ArgumentParser(description="Export the frozen V11 resume-i42500 Student.")
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--common-hold-pose", required=True, type=Path)
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
                args.common_hold_pose,
                args.output_dir,
                parity_observations=args.parity_observations,
                force=args.force,
            ),
            indent=2,
        )
    )
