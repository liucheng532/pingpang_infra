from __future__ import annotations

import numpy as np
import torch

from utils.hit_teacher_override_policy import TeacherHitOverridePolicy
from utils.joint_mapping import LAB_JOINT_NAMES
from utils.right_side_right_hand import RightSideRightHandStudentPolicy


class _Policy:
    def __init__(self, value, width):
        self.value = float(value)
        self.width = int(width)
        self.calls = 0
        self.resets = 0
        self.sidecar = {"policy": value}

    def __call__(self, observation):
        assert tuple(observation.shape) == (1, self.width)
        self.calls += 1
        return torch.full((1, 29), self.value)

    def reset(self):
        self.resets += 1


def _control_obs(phase):
    return {
        "phase_id": phase,
        "obs": torch.zeros(1, 172),
        "obs_history": torch.zeros(1, 1666),
        "teacher_obs_history": torch.zeros(1, 1670),
    }


def test_teacher_routes_only_hit_and_post_delay():
    student = _Policy(1.0, 1666)
    teacher = _Policy(2.0, 1670)
    router = TeacherHitOverridePolicy(student, teacher, hit_policy_source="teacher")
    for phase in range(6):
        action = router.infer_control(_control_obs(phase))
        expected = 2.0 if phase <= 1 else 1.0
        assert torch.all(action == expected)
        assert router.last_source == ("teacher" if phase <= 1 else "student")
    assert teacher.calls == 2
    assert student.calls == 4
    router.reset()
    assert teacher.resets == 1 and student.resets == 1


def test_student_source_preserves_legacy_output_in_all_phases():
    student = _Policy(1.0, 1666)
    teacher = _Policy(2.0, 1670)
    router = TeacherHitOverridePolicy(student, teacher, hit_policy_source="student")
    outputs = [router.infer_control(_control_obs(phase)).numpy() for phase in range(6)]
    assert all(np.array_equal(output, np.ones((1, 29), dtype=np.float32)) for output in outputs)
    assert teacher.calls == 0
    assert student.calls == 6


def test_right_side_student_adapter_never_transforms_right_hand_teacher_hit():
    base_student = _Policy(1.0, 1666)
    student = RightSideRightHandStudentPolicy(
        base_student,
        LAB_JOINT_NAMES,
        np.zeros(29, dtype=np.float32),
        np.ones(29, dtype=np.float32),
    )
    expected_action = torch.arange(29, dtype=torch.float32).reshape(1, 29)

    class Teacher:
        def __call__(self, observation):
            self.observation = observation.clone()
            return expected_action.clone()

    teacher = Teacher()
    router = TeacherHitOverridePolicy(student, teacher, hit_policy_source="teacher")
    control = _control_obs(0)
    control["teacher_obs_history"] = torch.arange(1670, dtype=torch.float32).reshape(1, 1670)
    action = router.infer_control(control)

    assert torch.equal(teacher.observation, control["teacher_obs_history"])
    assert torch.equal(action, expected_action)
    assert torch.equal(action[:, [12, 16, 20, 22, 24, 26, 28]], expected_action[:, [12, 16, 20, 22, 24, 26, 28]])
