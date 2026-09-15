from __future__ import annotations

import hashlib
import importlib.util
import inspect
import re
import sys
from pathlib import Path

import numpy as np
import torch


V01_RUNTIME_SHA256 = "e199d57c3e90d54c01cfc9bb810da9e03ad53330d80eda03f57f57e46c247240"
V01_RESIDUAL_SHA256 = "e3ef9aa8adcbbb317e010b4175be76199d71203bb55f18d2bc4e978e9c0f9045"
V01_CHECKPOINT_SHA256 = "ed348b0923f8b4d618553eb734d0b57e70d880870d84d1c1942854d881f3d2ed"
TEACHER_SHA256 = "5a4e650813f227f704ae2040df73ece8da11dae1746bfc8b6e982c68e9efc94c"
ARM7_INDICES = (12, 16, 20, 22, 24, 26, 28)
ARM7_NAMES = (
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require_hash(path, expected, label):
    path = Path(path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(path)
    actual = sha256_file(path)
    if actual != expected:
        raise RuntimeError(f"{label} SHA256 is {actual}, expected {expected}")
    return path


def validate_i9500_checkpoint(path):
    checkpoint = torch.load(path, map_location="cpu")
    expected = {
        "format": "v01_ball_residual_ppo_v2",
        "reward_profile": "v01_ball_outcome_target_v2",
        "iteration": 9500,
        "mode": "arm7",
        "teacher_sha256": TEACHER_SHA256,
        "code_commit": "262d2027efbd37593fa99b34ac500bcb7e3fbef6",
        "selected_joint_names": list(ARM7_NAMES),
        "selected_joint_indices": list(ARM7_INDICES),
    }
    mismatches = {
        key: {"expected": value, "actual": checkpoint.get(key)}
        for key, value in expected.items()
        if checkpoint.get(key) != value
    }
    if checkpoint.get("ppo_config", {}).get("residual_scale") != 0.15:
        mismatches["ppo_config.residual_scale"] = {
            "expected": 0.15,
            "actual": checkpoint.get("ppo_config", {}).get("residual_scale"),
        }
    mean = np.asarray(checkpoint.get("ball_observation_mean", []))
    std = np.asarray(checkpoint.get("ball_observation_std", []))
    if (
        mean.shape != (6,)
        or std.shape != (6,)
        or not np.isfinite(mean).all()
        or not np.isfinite(std).all()
        or not np.all(std > 0)
    ):
        mismatches["ball_normalization"] = {"expected": "finite [6] mean and positive std"}
    if mismatches:
        raise RuntimeError(f"V01 i9500 checkpoint mismatch: {mismatches}")


def load_runtime_module(path, expected_sha256=V01_RUNTIME_SHA256):
    path = require_hash(path, expected_sha256, "V01 runtime")
    name = "locked_v01_hit_residual_runtime_" + re.sub(r"[^0-9a-zA-Z_]", "_", expected_sha256[:12])
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load V01 runtime: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class MirroredBallSource:
    """Expose the physical table-right ball stream in canonical coordinates."""

    def __init__(self, source):
        self.source = source

    def sample(self):
        observation, valid, age_s, reason = self.source.sample()
        canonical = np.asarray(observation, dtype=np.float32).reshape(6).copy()
        canonical[[1, 4]] *= -1.0
        return canonical, valid, age_s, reason

    def reset(self):
        self.source.reset()

    def metadata(self):
        metadata = dict(self.source.metadata()) if hasattr(self.source, "metadata") else {}
        metadata["ball_mirror_y"] = True
        return metadata


class V01HitResidualAdapter:
    """Locked i9500 Teacher+Arm7 runtime used only during HIT/POST_DELAY."""

    def __init__(
        self,
        runtime_module_path,
        teacher_path,
        residual_path,
        checkpoint_path,
        mode="off",
        residual_scale=1.0,
        ball_topic="/residual/mocap_ball_state",
        ball_max_age_s=0.06,
        diagnostics_path=None,
        ball_source=None,
        mirror_ball_y=False,
        expected_runtime_sha256=V01_RUNTIME_SHA256,
    ):
        teacher_path = require_hash(teacher_path, TEACHER_SHA256, "HIT Teacher")
        residual_path = require_hash(residual_path, V01_RESIDUAL_SHA256, "V01 residual")
        checkpoint_path = require_hash(checkpoint_path, V01_CHECKPOINT_SHA256, "V01 checkpoint")
        validate_i9500_checkpoint(checkpoint_path)
        module = load_runtime_module(runtime_module_path, expected_runtime_sha256)
        if ball_source is None:
            ball_source = module.MocapBallStateSource(
                topic=ball_topic,
                control_dt=0.02,
                max_age_s=float(ball_max_age_s),
            )
        if mirror_ball_y:
            ball_source = MirroredBallSource(ball_source)
        parameters = inspect.signature(module.Arm7ResidualPolicy).parameters
        kwargs = {
            "mode": mode,
            "residual_scale": float(residual_scale),
            "output_device": "cpu",
            "diagnostics_path": diagnostics_path,
        }
        positional = [teacher_path, residual_path]
        if "checkpoint_path" in parameters:
            positional.append(checkpoint_path)
        positional.append(ball_source)
        self.runtime = module.Arm7ResidualPolicy(*positional, **kwargs)
        self.mode = mode
        self.residual_scale = float(residual_scale)
        self.mirror_ball_y = bool(mirror_ball_y)
        self.providers = self.runtime.providers
        self.model_metadata = {
            **self.runtime.model_metadata,
            "checkpoint_iteration": 9500,
            "checkpoint_sha256": V01_CHECKPOINT_SHA256,
            "runtime_sha256": expected_runtime_sha256,
            "ball_mirror_y": self.mirror_ball_y,
        }

    @property
    def last_diagnostics(self):
        return self.runtime.last_diagnostics

    def __call__(self, observation):
        return self.runtime(observation)

    def reset(self):
        self.runtime.reset()

    def close(self):
        self.runtime.close()

    def record_applied_command(self, *args, **kwargs):
        return self.runtime.record_applied_command(*args, **kwargs)
