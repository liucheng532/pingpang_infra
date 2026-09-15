from __future__ import annotations

import numpy as np
import torch


STUDENT_OBS_DIM = 1666
TEACHER_OBS_DIM = 1670
ACTION_DIM = 29


def _numpy(value, width):
    if torch.is_tensor(value):
        value = value.detach().reshape(-1).cpu().numpy()
    return np.asarray(value, dtype=np.float32).reshape(width).copy()


class TeacherHitOverridePolicy:
    """Use Teacher/V01 for HIT/POST and Student for all locomotion phases."""

    def __init__(self, student_policy, teacher_policy, hit_policy_source="student"):
        if hit_policy_source not in ("student", "teacher"):
            raise ValueError("hit_policy_source must be 'student' or 'teacher'.")
        self.student_policy = student_policy
        self.teacher_policy = teacher_policy
        self.hit_policy_source = hit_policy_source
        self.sidecar = student_policy.sidecar
        self.sequence = 0
        self.last_sequence = 0
        self.last_source = "student"
        self.last_policy_observation = None
        self.last_teacher_policy_observation = None
        self.last_canonical_action = None
        self.last_physical_action = None

    def infer_control(self, control_obs):
        hit_active = int(control_obs["phase_id"]) <= 1
        use_teacher = hit_active and self.hit_policy_source == "teacher"
        if use_teacher:
            action = self.teacher_policy(control_obs["teacher_obs_history"])
            self.last_source = "teacher"
            self.last_teacher_policy_observation = getattr(
                self.teacher_policy,
                "last_policy_observation",
                control_obs["teacher_obs_history"],
            )
            if hasattr(self.student_policy, "canonical_observation"):
                self.last_policy_observation = self.student_policy.canonical_observation(
                    control_obs["obs_history"]
                )
            else:
                self.last_policy_observation = control_obs["obs_history"]
            self.last_canonical_action = getattr(
                self.teacher_policy, "last_canonical_action", action
            )
        else:
            action = self.student_policy(control_obs["obs_history"])
            self.last_source = "student"
            self.last_policy_observation = getattr(
                self.student_policy, "last_policy_observation", control_obs["obs_history"]
            )
            self.last_canonical_action = getattr(
                self.student_policy, "last_canonical_action", action
            )
        self.last_physical_action = action
        self.last_sequence = self.sequence
        self.sequence += 1
        return action

    def _physical_action(self, canonical_action):
        if hasattr(self.teacher_policy, "mirror_action"):
            return _numpy(self.teacher_policy.mirror_action(canonical_action), ACTION_DIM)
        return _numpy(canonical_action, ACTION_DIM)

    def record_snapshot_fields(self):
        fields = {
            "control_policy_sequence": self.last_sequence,
            "active_policy_source": 1 if self.last_source == "teacher" else 0,
        }
        if self.last_teacher_policy_observation is not None:
            fields["teacher_policy_obs_history"] = _numpy(
                self.last_teacher_policy_observation, TEACHER_OBS_DIM
            )
        diagnostics = getattr(self.teacher_policy, "last_diagnostics", {})
        if self.last_source != "teacher" or not diagnostics:
            return fields

        canonical_teacher = np.asarray(
            diagnostics.get("teacher_action", np.zeros(ACTION_DIM)), dtype=np.float32
        ).reshape(ACTION_DIM)
        canonical_combined = np.asarray(
            diagnostics.get("combined_action", canonical_teacher), dtype=np.float32
        ).reshape(ACTION_DIM)
        canonical_output = np.asarray(
            diagnostics.get("output_action", canonical_teacher), dtype=np.float32
        ).reshape(ACTION_DIM)
        physical_teacher = self._physical_action(canonical_teacher)
        physical_combined = self._physical_action(canonical_combined)
        physical_output = self._physical_action(canonical_output)
        physical_residual = physical_output - physical_teacher
        backend_codes = {"teacher": 0, "fallback": 1, "shadow": 2, "active": 3}
        mode_codes = {"off": 0, "shadow": 1, "active": 2}
        fields.update(
            {
                "hit_residual_mode": mode_codes.get(diagnostics.get("mode"), 0),
                "hit_residual_backend": backend_codes.get(diagnostics.get("backend"), 0),
                "hit_residual_ball_valid": int(bool(diagnostics.get("ball_valid", False))),
                "hit_residual_ball_age_ms": (
                    np.nan if diagnostics.get("ball_age_ms") is None else diagnostics["ball_age_ms"]
                ),
                "hit_residual_ball_reason": str(diagnostics.get("ball_reason", ""))[:24].encode(),
                "hit_residual_phase_gate": float(diagnostics.get("phase_gate", 0.0)),
                "hit_teacher_canonical_action": canonical_teacher,
                "hit_combined_canonical_action": canonical_combined,
                "hit_output_canonical_action": canonical_output,
                "hit_teacher_physical_action": physical_teacher,
                "hit_combined_physical_action": physical_combined,
                "hit_output_physical_action": physical_output,
                "hit_residual_applied_physical": physical_residual,
                "hit_residual_abs_max": float(np.max(np.abs(physical_residual))),
                "hit_residual_rms": float(np.sqrt(np.mean(np.square(physical_residual)))),
                "hit_residual_inference_ms": float(diagnostics.get("inference_ms", 0.0)),
            }
        )
        return fields

    def record_applied_command(self, *args, **kwargs):
        if self.last_source == "teacher" and hasattr(self.teacher_policy, "record_applied_command"):
            return self.teacher_policy.record_applied_command(*args, **kwargs)

    def reset(self):
        for policy in (self.student_policy, self.teacher_policy):
            if hasattr(policy, "reset"):
                policy.reset()

    def close(self):
        if hasattr(self.teacher_policy, "close"):
            self.teacher_policy.close()
