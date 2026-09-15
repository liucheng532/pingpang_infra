from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch

from utils.v01_hit_residual_adapter import (
    ARM7_INDICES,
    ARM7_NAMES,
    TEACHER_SHA256,
    V01_RUNTIME_SHA256,
    MirroredBallSource,
    load_runtime_module,
    require_hash,
)


CHECKPOINT_SHA256 = "f08fffa370e47ce783087a53534b4e69a293e60133e6b8ad3ca36b817fd4ee9a"
V11_CHECKPOINT_SHA256 = "04461633b8ad75a2b173ceacf34257fae437cbf935f6effe8b279464586bea12"
CHECKPOINT_FORMAT = "v11_teacher_ball_residual_ppo_v1"
DEPLOY_FORMAT = "v11_teacher_arm7_hard1_onnx_v1"
ONNX_BASENAME = "v11_teacher_arm7_i15000_hard1.onnx"
CHECKPOINT_BASENAME = "model_15000.pt"


def validate_checkpoint(path):
    path = require_hash(path, CHECKPOINT_SHA256, "V11 Teacher i15000 checkpoint")
    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    expected = {
        "format": CHECKPOINT_FORMAT,
        "base_kind": "teacher",
        "iteration": 15000,
        "v11_checkpoint_sha256": V11_CHECKPOINT_SHA256,
        "teacher_sha256": TEACHER_SHA256,
        "base_observation_dim": 1670,
        "ball_actor_obs_dim": 6,
        "action_dim": 29,
        "selected_joint_indices": list(ARM7_INDICES),
        "selected_joint_names": list(ARM7_NAMES),
    }
    for key, value in expected.items():
        if checkpoint.get(key) != value:
            raise RuntimeError(f"V11 residual checkpoint mismatch: {key}")
    if checkpoint["ppo_config"]["residual_scale"] != 0.15:
        raise RuntimeError("V11 residual scale must be 0.15")
    mean = np.asarray(checkpoint["ball_observation_mean"])
    std = np.asarray(checkpoint["ball_observation_std"])
    if (mean.shape != (6,) or std.shape != (6,) or not np.isfinite(mean).all()
            or not np.isfinite(std).all() or np.any(std <= 0)):
        raise RuntimeError("Invalid V11 ball normalization")
    return checkpoint


class V11TeacherResidualAdapter:
    """i15000 with gate=1; the outer router restricts calls to HIT/POST.

    Ball estimation and asynchronous diagnostics reuse the locked V01 runtime.
    The V01 model loader and TTS gate are not used.
    """

    def __init__(
        self, runtime_module_path, teacher_path, residual_path, checkpoint_path,
        mode="off", residual_scale=1.0, ball_topic="/residual/mocap_ball_state",
        ball_max_age_s=0.06, diagnostics_path=None, ball_source=None,
        mirror_ball_y=False,
    ):
        if mode not in ("off", "shadow", "active"):
            raise ValueError("mode must be off, shadow, or active")
        if not 0.0 <= float(residual_scale) <= 1.0:
            raise ValueError("residual_scale must be within [0, 1]")
        if not np.isfinite(ball_max_age_s) or ball_max_age_s <= 0:
            raise ValueError("ball_max_age_s must be positive and finite")
        teacher_path = require_hash(teacher_path, TEACHER_SHA256, "HIT Teacher")
        validate_checkpoint(checkpoint_path)
        residual_path = Path(residual_path).expanduser().resolve()
        metadata = json.loads(Path(str(residual_path) + ".json").read_text())
        expected = {
            "format": DEPLOY_FORMAT,
            "checkpoint_sha256": CHECKPOINT_SHA256,
            "teacher_sha256": TEACHER_SHA256,
            "v11_checkpoint_sha256": V11_CHECKPOINT_SHA256,
            "gate": 1.0,
            "residual_scale": 0.15,
            "selected_joint_indices": list(ARM7_INDICES),
        }
        for key, value in expected.items():
            if metadata.get(key) != value:
                raise RuntimeError(f"V11 residual ONNX metadata mismatch: {key}")
        require_hash(residual_path, metadata["onnx_sha256"], "V11 residual ONNX")
        runtime = load_runtime_module(runtime_module_path)
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        self.providers = ("CPUExecutionProvider",)
        self.teacher = ort.InferenceSession(str(teacher_path), options, providers=list(self.providers))
        self.residual = ort.InferenceSession(str(residual_path), options, providers=list(self.providers))
        inputs = self.residual.get_inputs()
        if {x.name: x.shape[-1] for x in inputs} != {"teacher_obs": 1670, "ball_obs": 6}:
            raise RuntimeError("V11 hard1 ONNX requires teacher_obs[1670] and ball_obs[6]")
        if any(len(x.shape) != 2 or x.type != "tensor(float)" for x in inputs):
            raise RuntimeError("V11 ONNX inputs must be float32 matrices")
        outputs = self.residual.get_outputs()
        if len(outputs) != 1 or len(outputs[0].shape) != 2 or outputs[0].shape[-1] != 29:
            raise RuntimeError("V11 residual ONNX must output actions[29]")
        if ball_source is None:
            ball_source = runtime.MocapBallStateSource(
                topic=ball_topic, control_dt=0.02, max_age_s=float(ball_max_age_s)
            )
        self.ball_source = MirroredBallSource(ball_source) if mirror_ball_y else ball_source
        self.writer = runtime.ResidualDiagnosticsWriter(diagnostics_path)
        self.mode = mode
        self.residual_scale = float(residual_scale)
        self.ball_max_age_s = float(ball_max_age_s)
        self.last_diagnostics = {}
        self.calls = 0
        self.model_metadata = {
            **metadata,
            "policy_version": "v11_teacher_arm7_i15000_hard1",
            "checkpoint_iteration": 15000,
            "runtime_sha256": V01_RUNTIME_SHA256,
            "ball_mirror_y": bool(mirror_ball_y),
            "runtime_gate": "hit_post_one_valid",
        }

    def __call__(self, observation):
        started = time.perf_counter_ns()
        if torch.is_tensor(observation):
            observation = observation.detach().cpu().numpy()
        teacher_obs = np.asarray(observation, dtype=np.float32)
        if teacher_obs.shape == (1670,):
            teacher_obs = teacher_obs[None, :]
        if teacher_obs.ndim != 2 or teacher_obs.shape[1] != 1670 or teacher_obs.shape[0] == 0:
            raise RuntimeError("Expected teacher observation [batch,1670]")
        if not np.isfinite(teacher_obs).all():
            raise RuntimeError("Teacher observation contains NaN/Inf")
        teacher = self.teacher.run(None, {self.teacher.get_inputs()[0].name: teacher_obs})[0]
        if teacher.shape != (teacher_obs.shape[0], 29) or not np.isfinite(teacher).all():
            raise RuntimeError("Invalid teacher action")
        combined = teacher.copy()
        output = teacher.copy()
        ball = np.zeros(6, dtype=np.float32)
        valid, age_s, reason, backend = False, None, "mode_off", "teacher"
        if self.mode != "off":
            ball, valid, age_s, reason = self.ball_source.sample()
            ball = np.asarray(ball, dtype=np.float32).reshape(6)
            if not np.isfinite(ball).all():
                valid, reason = False, "nonfinite_observation"
            elif age_s is None or not np.isfinite(age_s) or not 0 <= age_s <= self.ball_max_age_s:
                valid, reason = False, "stale_or_missing_timestamp"
            if valid:
                combined = self.residual.run(None, {
                    "teacher_obs": teacher_obs,
                    "ball_obs": np.repeat(ball[None, :], teacher_obs.shape[0], axis=0),
                })[0]
                if combined.shape != teacher.shape or not np.isfinite(combined).all():
                    raise RuntimeError("Invalid residual action")
                delta = combined - teacher
                inactive = [i for i in range(29) if i not in ARM7_INDICES]
                if np.max(np.abs(delta)) > 0.1501 or np.max(np.abs(delta[:, inactive])) > 1.0e-5:
                    raise RuntimeError("V11 residual violated Arm7/bound contract")
                # Separate ONNX graphs may round the frozen teacher differently.
                # Preserve the standalone teacher bit-for-bit outside Arm7.
                combined[:, inactive] = teacher[:, inactive]
                backend = "shadow"
                if self.mode == "active":
                    output[:, ARM7_INDICES] += self.residual_scale * delta[:, ARM7_INDICES]
                    backend = "active"
            else:
                backend = "fallback"
        self.last_diagnostics = {
            **self.model_metadata,
            **(self.ball_source.metadata() if hasattr(self.ball_source, "metadata") else {}),
            "sequence": self.calls, "wall_time_ns": time.time_ns(),
            "monotonic_s": time.monotonic(), "mode": self.mode, "backend": backend,
            "residual_scale": self.residual_scale,
            "ball_valid": bool(valid), "ball_reason": reason,
            "ball_age_ms": None if age_s is None else 1000.0 * age_s,
            "ball_observation": ball.tolist(), "strike_time_s": float(teacher_obs[0, 9]),
            "phase_gate": float(valid),
            "teacher_action": teacher[0].tolist(), "combined_action": combined[0].tolist(),
            "residual_action": (combined[0] - teacher[0]).tolist(),
            "output_action": output[0].tolist(),
            "inference_ms": (time.perf_counter_ns() - started) * 1.0e-6,
        }
        self.calls += 1
        return torch.from_numpy(output)

    def record_applied_command(
        self, action_gym, q_des, command_publish_monotonic_ns, loop_period_s,
        observe_s=None, policy_call_s=None, apply_publish_s=None,
    ):
        if torch.is_tensor(action_gym):
            action_gym = action_gym.detach().cpu().numpy()
        action_gym = np.asarray(action_gym).reshape(29)
        q_des = np.asarray(q_des).reshape(29)
        if not np.isfinite(action_gym).all() or not np.isfinite(q_des).all():
            raise RuntimeError("Nonfinite applied action diagnostics")
        self.last_diagnostics.update({
            "action_gym": action_gym.tolist(), "q_des": q_des.tolist(),
            "command_publish_monotonic_ns": int(command_publish_monotonic_ns),
            "loop_period_s": float(loop_period_s),
            "observe_ms": None if observe_s is None else 1000.0 * observe_s,
            "policy_call_ms": None if policy_call_s is None else 1000.0 * policy_call_s,
            "apply_publish_ms": None if apply_publish_s is None else 1000.0 * apply_publish_s,
        })
        self.writer.enqueue(dict(self.last_diagnostics))

    def reset(self):
        self.ball_source.reset()
        self.last_diagnostics = {}

    def close(self):
        self.writer.close()
