from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import torch


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_ROOT))

from utils.data_utils import MotionCommand, MoveMotionBank
from utils.doubles_reference_scheduler import DoublesPhase, DoublesReferenceScheduler
from utils.joint_mapping import LAB_JOINT_NAMES
from utils.left_right_mirror import (
    MirroredStudentPolicy,
    MirroredTeacherPolicy,
    joint_mirror_spec,
    mirror_axial_vectors,
    mirror_joint_values,
    mirror_planar_vectors,
    mirror_polar_vectors,
    mirror_policy_action,
    mirror_rpy,
    mirror_student_observation,
    mirror_teacher_observation,
)
from utils.mirrored_reference_scheduler import MirroredReferenceScheduler
from utils.right_side_right_hand import (
    RightSideRightHandStudentPolicy,
    arm_joint_indices,
    canonicalize_right_side_student_observation,
    reflect_right_side_action,
    reflect_right_side_body,
)
from utils.v01_hit_residual_adapter import MirroredBallSource


def _write_mirror_hit_bank(root: Path, frames: int = 80) -> None:
    root.mkdir()
    lines = ["index backhand target\n"]
    for motion_number, (index, target_y) in enumerate((("0368", -0.3), ("0001", 0.3))):
        folder = root / f"{root.name}-{index}:v0"
        folder.mkdir()
        time_axis = np.arange(frames, dtype=np.float32)[:, None]
        joint_axis = np.arange(29, dtype=np.float32)[None, :] * 0.001
        q = motion_number + time_axis * 0.01 + joint_axis
        qd = np.full_like(q, 0.1 + motion_number)
        body_pos = np.zeros((frames, 10, 3), dtype=np.float32)
        body_quat = np.zeros((frames, 10, 4), dtype=np.float32)
        body_quat[..., 0] = 1.0
        np.savez(
            folder / "motion.npz",
            fps=np.array(50.0, dtype=np.float32),
            joint_pos=q,
            joint_vel=qd,
            body_pos_w=body_pos,
            body_quat_w=body_quat,
        )
        lines.append(f"{index} {motion_number % 2} 0.45, {target_y}, 1.0\n")
    (root / "dataindex.csv").write_text("".join(lines), encoding="utf-8")


def _write_mirror_move_bank(root: Path, frames: int = 100) -> None:
    root.mkdir()
    rows = (
        ("0000", 0, -0.45),
        ("0001", 0, -0.60),
        ("0002", 1, 0.45),
        ("0003", 1, 0.60),
    )
    lines = ["index posthitlabel target\n"]
    for motion_number, (index, label, target_y) in enumerate(rows):
        folder = root / f"{root.name}-{index}:v0"
        folder.mkdir()
        time_axis = np.arange(frames, dtype=np.float32)[:, None]
        q = motion_number + time_axis * 0.01 + np.zeros((frames, 29), dtype=np.float32)
        qd = np.full_like(q, target_y)
        np.savez(
            folder / "motion.npz",
            fps=np.array(80.0, dtype=np.float32),
            joint_pos=q,
            joint_vel=qd,
        )
        lines.append(f"{index} {label} {target_y}\n")
    (root / "dataindex.csv").write_text("".join(lines), encoding="utf-8")


@pytest.fixture()
def mirrored_motion_banks(tmp_path):
    hit_root = tmp_path / "0302_combined"
    move_root = tmp_path / "0718-move-160-80hz"
    _write_mirror_hit_bank(hit_root)
    _write_mirror_move_bank(move_root)
    return (
        MotionCommand(str(hit_root), device="cpu"),
        MoveMotionBank(str(move_root), expected_count=4, expected_fps=80.0),
    )


def _action_parameters():
    offset = np.linspace(-0.4, 0.4, 29, dtype=np.float32)
    scale = np.linspace(0.2, 0.9, 29, dtype=np.float32)
    return offset, scale


def test_joint_mapping_requires_exact_g1_set_and_is_involutive():
    spec = joint_mirror_spec(LAB_JOINT_NAMES)
    assert len(spec.permutation) == 29
    values = np.arange(58, dtype=np.float32).reshape(2, 29)
    assert np.array_equal(
        mirror_joint_values(mirror_joint_values(values, LAB_JOINT_NAMES), LAB_JOINT_NAMES),
        values,
    )
    with pytest.raises(ValueError, match="exact 29"):
        joint_mirror_spec(LAB_JOINT_NAMES[:-1] + ("not_a_g1_joint",))


def test_right_side_body_reflects_legs_and_waist_but_preserves_arms():
    values = np.linspace(-1.0, 1.0, 58, dtype=np.float32).reshape(2, 29)
    full_mirror = mirror_joint_values(values, LAB_JOINT_NAMES)
    reflected = reflect_right_side_body(values, LAB_JOINT_NAMES)
    arms = set(arm_joint_indices(LAB_JOINT_NAMES))
    non_arms = [index for index in range(29) if index not in arms]

    assert np.array_equal(reflected[..., list(arms)], values[..., list(arms)])
    assert np.array_equal(reflected[..., non_arms], full_mirror[..., non_arms])
    assert np.array_equal(
        reflect_right_side_body(reflected, LAB_JOINT_NAMES), values
    )


def test_right_side_student_adapter_preserves_all_arm_histories_and_actions():
    observation = np.arange(1666, dtype=np.float32).reshape(1, 1666) * 0.001
    offset, scale = _action_parameters()
    canonical = canonicalize_right_side_student_observation(
        observation, LAB_JOINT_NAMES, offset, scale
    )
    arms = list(arm_joint_indices(LAB_JOINT_NAMES))

    for term_slice in (slice(210, 500), slice(500, 790), slice(790, 1080)):
        source_term = observation[:, term_slice].reshape(1, 10, 29)
        canonical_term = canonical[:, term_slice].reshape(1, 10, 29)
        assert np.array_equal(canonical_term[..., arms], source_term[..., arms])

    source_command = observation[:, 1080:1660].reshape(1, 10, 2, 29)
    canonical_command = canonical[:, 1080:1660].reshape(1, 10, 2, 29)
    assert np.array_equal(canonical_command[..., arms], source_command[..., arms])
    assert np.array_equal(canonical[:, 70:90:2], observation[:, 70:90:2])
    assert np.array_equal(canonical[:, 71:90:2], -observation[:, 71:90:2])

    action = np.linspace(-0.8, 0.8, 29, dtype=np.float32).reshape(1, 29)

    class Policy:
        sidecar = {"iteration": 21500}

        def __call__(self, value):
            self.observation = np.asarray(value).copy()
            return action.copy()

    policy = Policy()
    wrapper = RightSideRightHandStudentPolicy(
        policy, LAB_JOINT_NAMES, offset, scale
    )
    physical_action = wrapper(observation)
    full_action_mirror = mirror_policy_action(
        action, LAB_JOINT_NAMES, offset, scale, offset, scale
    )
    non_arms = [index for index in range(29) if index not in arms]
    assert np.array_equal(policy.observation, canonical)
    assert np.array_equal(physical_action[..., arms], action[..., arms])
    assert np.array_equal(
        physical_action[..., non_arms], full_action_mirror[..., non_arms]
    )
    assert np.allclose(
        reflect_right_side_action(
            physical_action, LAB_JOINT_NAMES, offset, scale
        ),
        action,
        atol=1.0e-6,
    )


def test_right_side_scheduler_uses_same_canonical_move_and_physical_negative_y(
    mirrored_motion_banks,
):
    left = DoublesReferenceScheduler(
        *mirrored_motion_banks, external_control=True, move_side_sign=1
    )
    right = DoublesReferenceScheduler(
        *mirrored_motion_banks, external_control=True, move_side_sign=-1
    )
    left_pose = np.array([0.20, 0.20, 0.75], dtype=np.float32)
    right_pose = np.array([0.20, -0.20, 0.75], dtype=np.float32)
    left.reset(left_pose, left_pose)
    right.reset(right_pose, right_pose)

    left.set_external_base_target(
        np.array([0.20, 0.80], dtype=np.float32), return_target_y=0.20
    )
    right.set_external_base_target(
        np.array([0.20, -0.80], dtype=np.float32), return_target_y=-0.20
    )
    left_reference = left.output()
    right_reference = right.output()

    assert left.state == right.state == DoublesPhase.OUTWARD
    assert left_reference.move_motion_index == right_reference.move_motion_index
    assert np.isclose(left_reference.target_base[1], 0.60)
    assert np.isclose(right_reference.target_base[1], -0.60)
    arms = list(arm_joint_indices(LAB_JOINT_NAMES))
    assert np.array_equal(
        right_reference.joint_pos[arms], left_reference.joint_pos[arms]
    )
    assert np.array_equal(
        right_reference.joint_pos,
        reflect_right_side_body(left_reference.joint_pos, LAB_JOINT_NAMES),
    )

    left._start_outward_hold("stable")
    right._start_outward_hold("stable")
    left._pelvis_position = np.array([0.10, 0.80, 0.75], dtype=np.float32)
    right._pelvis_position = np.array([0.10, -0.80, 0.75], dtype=np.float32)
    left.set_external_base_target(np.array([0.10, 0.20], dtype=np.float32))
    right.set_external_base_target(np.array([0.10, -0.20], dtype=np.float32))
    left_return = left.output()
    right_return = right.output()
    assert left.state == right.state == DoublesPhase.RETURN
    assert left_return.move_motion_index == right_return.move_motion_index
    assert np.isclose(left_return.target_base[0], 0.10)
    assert np.isclose(right_return.target_base[0], 0.10)
    assert np.isclose(left_return.target_base[1], -0.60)
    assert np.isclose(right_return.target_base[1], 0.60)


@pytest.mark.parametrize("value", (0, 2, -2, True, 1.0))
def test_move_side_sign_requires_integer_reflection(value, mirrored_motion_banks):
    with pytest.raises(ValueError, match="move_side_sign"):
        DoublesReferenceScheduler(*mirrored_motion_banks, move_side_sign=value)


def test_native_no_mirror_uses_physical_motion_direction_and_semantic_phase(
    mirrored_motion_banks,
):
    scheduler = DoublesReferenceScheduler(
        *mirrored_motion_banks,
        external_control=True,
        transition_s=0.0,
        episode_start_x=0.20,
        move_side_sign=-1,
        native_no_mirror=True,
    )
    pose = np.array([0.20, -0.20, 0.75], dtype=np.float32)
    scheduler.reset(pose, pose)

    scheduler.set_external_base_target(
        np.array([0.20, -0.80], dtype=np.float32),
        return_target_y=-0.20,
        semantic_phase="OUTWARD",
    )
    outward = scheduler.output()
    assert outward.state == DoublesPhase.OUTWARD
    assert scheduler.move_motions.labels[outward.move_motion_index] == 0
    assert np.isclose(outward.target_base[1], -0.60)
    expected_q, expected_qd, _ = scheduler.move_motions.frame(
        outward.move_motion_index, 0
    )
    assert np.array_equal(outward.joint_pos, expected_q)
    assert np.array_equal(outward.joint_vel, expected_qd)

    scheduler._start_outward_hold("stable")
    scheduler._pelvis_position = np.array([0.10, -0.80, 0.75], dtype=np.float32)
    scheduler.set_external_base_target(
        np.array([0.10, -0.20], dtype=np.float32),
        return_target_y=-0.20,
        semantic_phase="RETURN",
    )
    returning = scheduler.output()
    assert returning.state == DoublesPhase.RETURN
    assert scheduler.move_motions.labels[returning.move_motion_index] == 1
    assert np.isclose(returning.target_base[0], 0.10)
    assert np.isclose(returning.target_base[1], 0.60)
    expected_q, expected_qd, _ = scheduler.move_motions.frame(
        returning.move_motion_index, 0
    )
    assert np.array_equal(returning.joint_pos, expected_q)
    assert np.array_equal(returning.joint_vel, expected_qd)


def test_native_no_mirror_does_not_change_hit_reference(mirrored_motion_banks):
    baseline = DoublesReferenceScheduler(*mirrored_motion_banks, transition_s=0.0)
    native = DoublesReferenceScheduler(
        *mirrored_motion_banks,
        transition_s=0.0,
        move_side_sign=-1,
        native_no_mirror=True,
    )
    pose = np.array([0.20, -0.20, 0.75], dtype=np.float32)
    target = np.array([0.45, -0.25, 1.0], dtype=np.float32)
    velocity = np.array([3.0, -0.1, 0.5], dtype=np.float32)
    for scheduler in (baseline, native):
        scheduler.reset(pose, pose)
        scheduler.set_external_hit(target, velocity, 0.40)
    assert np.array_equal(baseline.output().command, native.output().command)
    assert np.array_equal(baseline.output().racket_target, native.output().racket_target)


def test_move_side_sign_does_not_change_right_hand_hit_selection_or_reference(
    mirrored_motion_banks,
):
    left = DoublesReferenceScheduler(*mirrored_motion_banks, transition_s=0.0)
    right = DoublesReferenceScheduler(
        *mirrored_motion_banks, transition_s=0.0, move_side_sign=-1
    )
    pose = np.array([0.20, -0.20, 0.75], dtype=np.float32)
    target = np.array([0.45, -0.25, 1.0], dtype=np.float32)
    velocity = np.array([3.0, -0.1, 0.5], dtype=np.float32)
    for scheduler in (left, right):
        scheduler.reset(pose, pose)
        scheduler.set_external_hit(target, velocity, 0.40)
    left_reference = left.output()
    right_reference = right.output()

    assert left_reference.state == right_reference.state == DoublesPhase.HIT
    assert left_reference.hit_motion_index == right_reference.hit_motion_index
    assert np.array_equal(left_reference.racket_target, right_reference.racket_target)
    assert np.array_equal(left_reference.target_velocity, right_reference.target_velocity)
    assert np.array_equal(left_reference.joint_pos, right_reference.joint_pos)


def test_1666_observation_and_absolute_action_round_trip():
    rng = np.random.default_rng(20260820)
    observation = rng.normal(size=(3, 1666)).astype(np.float32)
    offset, scale = _action_parameters()
    mirrored = mirror_student_observation(
        observation,
        LAB_JOINT_NAMES,
        offset,
        scale,
        offset,
        scale,
    )
    restored = mirror_student_observation(
        mirrored,
        LAB_JOINT_NAMES,
        offset,
        scale,
        offset,
        scale,
    )
    assert np.max(np.abs(restored - observation)) <= 1.0e-6
    assert np.array_equal(mirrored[:, 0:10], observation[:, 0:10])
    assert np.array_equal(mirrored[:, 1660:1666], observation[:, 1660:1666])

    action = rng.normal(size=(3, 29)).astype(np.float32)
    action_round_trip = mirror_policy_action(
        mirror_policy_action(action, LAB_JOINT_NAMES, offset, scale, offset, scale),
        LAB_JOINT_NAMES,
        offset,
        scale,
        offset,
        scale,
    )
    assert np.max(np.abs(action_round_trip - action)) <= 1.0e-6


def test_1670_teacher_observation_round_trip_and_term_mirror():
    rng = np.random.default_rng(20260826)
    observation = rng.normal(size=(3, 1670)).astype(np.float32)
    offset, scale = _action_parameters()
    mirrored = mirror_teacher_observation(
        observation, LAB_JOINT_NAMES, offset, scale, offset, scale
    )
    restored = mirror_teacher_observation(
        mirrored, LAB_JOINT_NAMES, offset, scale, offset, scale
    )
    assert np.max(np.abs(restored - observation)) <= 2.0e-6
    assert np.array_equal(mirrored[:, 0:10], observation[:, 0:10])
    target_velocity = observation[:, 10:40].reshape(3, 10, 3)
    mirrored_velocity = mirrored[:, 10:40].reshape(3, 10, 3)
    assert np.array_equal(mirrored_velocity[..., 0], target_velocity[..., 0])
    assert np.array_equal(mirrored_velocity[..., 1], -target_velocity[..., 1])
    assert np.array_equal(mirrored_velocity[..., 2], target_velocity[..., 2])


def test_mirrored_teacher_policy_returns_left_hand_physical_action():
    offset, scale = _action_parameters()
    physical_observation = torch.arange(1670, dtype=torch.float32).reshape(1, 1670)
    canonical_action = torch.linspace(-0.5, 0.5, 29).reshape(1, 29)
    def teacher_policy(observation):
        assert torch.is_tensor(observation)
        assert observation.device.type == "cpu"
        return canonical_action.clone()

    wrapper = MirroredTeacherPolicy(
        teacher_policy,
        LAB_JOINT_NAMES,
        offset,
        scale,
        offset,
        scale,
    )
    physical_action = wrapper(physical_observation)
    assert np.array_equal(
        wrapper.last_policy_observation,
        mirror_teacher_observation(
            physical_observation, LAB_JOINT_NAMES, offset, scale, offset, scale
        ).numpy(),
    )
    assert torch.equal(
        physical_action,
        mirror_policy_action(
            canonical_action, LAB_JOINT_NAMES, offset, scale, offset, scale
        ),
    )


def test_mirrored_ball_source_flips_lateral_position_and_velocity():
    class Source:
        def sample(self):
            return np.arange(1, 7, dtype=np.float32), True, 0.01, "ok"

        def reset(self):
            self.reset_called = True

        def metadata(self):
            return {"ball_sample_count": 10}

    source = Source()
    mirrored = MirroredBallSource(source)
    observation, valid, age_s, reason = mirrored.sample()
    assert np.array_equal(observation, [1.0, -2.0, 3.0, 4.0, -5.0, 6.0])
    assert valid and age_s == 0.01 and reason == "ok"
    assert mirrored.metadata() == {"ball_sample_count": 10, "ball_mirror_y": True}


def test_canonical_right_arm_residual_maps_only_to_physical_left_arm():
    offset = np.zeros(29, dtype=np.float32)
    scale = np.ones(29, dtype=np.float32)
    canonical_teacher = np.zeros((1, 29), dtype=np.float32)
    canonical_combined = canonical_teacher.copy()
    canonical_right_arm = (12, 16, 20, 22, 24, 26, 28)
    physical_left_arm = (11, 15, 19, 21, 23, 25, 27)
    canonical_combined[:, list(canonical_right_arm)] = 0.14
    physical_teacher = mirror_policy_action(
        canonical_teacher, LAB_JOINT_NAMES, offset, scale, offset, scale
    )
    physical_combined = mirror_policy_action(
        canonical_combined, LAB_JOINT_NAMES, offset, scale, offset, scale
    )
    delta = np.asarray(physical_combined - physical_teacher)
    inactive = np.ones(29, dtype=bool)
    inactive[list(physical_left_arm)] = False
    assert np.max(np.abs(delta[:, inactive])) == 0.0
    assert np.allclose(np.abs(delta[:, list(physical_left_arm)]), 0.14)

    class ActivePolicy:
        def __init__(self):
            self.last_diagnostics = {
                "mode": "active",
                "teacher_action": canonical_teacher[0].tolist(),
            }

        def __call__(self, _observation):
            return torch.from_numpy(canonical_combined.copy())

    wrapper = MirroredTeacherPolicy(
        ActivePolicy(), LAB_JOINT_NAMES, offset, scale, offset, scale
    )
    guarded_output = wrapper(torch.zeros(1, 1670))
    assert np.array_equal(guarded_output, physical_combined)


def test_policy_wrapper_exposes_canonical_trace_and_returns_physical_action():
    offset, scale = _action_parameters()
    physical_observation = torch.arange(1666, dtype=torch.float32).reshape(1, 1666)
    canonical_action = torch.linspace(-0.5, 0.5, 29).reshape(1, 29)
    seen = []

    def policy(observation):
        assert isinstance(observation, np.ndarray)
        seen.append(observation.copy())
        return canonical_action.clone()

    wrapper = MirroredStudentPolicy(
        policy,
        LAB_JOINT_NAMES,
        offset,
        scale,
        offset,
        scale,
    )
    physical_action = wrapper(physical_observation)
    expected_observation = mirror_student_observation(
        physical_observation,
        LAB_JOINT_NAMES,
        offset,
        scale,
        offset,
        scale,
    )
    expected_action = mirror_policy_action(
        canonical_action,
        LAB_JOINT_NAMES,
        offset,
        scale,
        offset,
        scale,
    )
    assert np.array_equal(seen[0], expected_observation.numpy())
    assert np.array_equal(wrapper.last_policy_observation, expected_observation.numpy())
    assert torch.equal(wrapper.last_canonical_action, canonical_action)
    assert torch.equal(physical_action, expected_action)


def _assert_reference_mirror(canonical, physical):
    assert canonical.state == physical.state
    assert canonical.hit_motion_index == physical.hit_motion_index
    assert canonical.move_motion_index == physical.move_motion_index
    assert canonical.reference_step == physical.reference_step
    assert canonical.strike_time == physical.strike_time
    assert np.array_equal(canonical.phase, physical.phase)
    assert np.allclose(physical.target_velocity, mirror_polar_vectors(canonical.target_velocity), atol=1.0e-6)
    assert np.allclose(physical.racket_target, mirror_polar_vectors(canonical.racket_target), atol=1.0e-6)
    assert np.allclose(physical.target_base, mirror_planar_vectors(canonical.target_base), atol=1.0e-6)
    assert np.allclose(
        physical.joint_pos,
        mirror_joint_values(canonical.joint_pos, LAB_JOINT_NAMES),
        atol=1.0e-6,
    )
    assert np.allclose(
        physical.joint_vel,
        mirror_joint_values(canonical.joint_vel, LAB_JOINT_NAMES),
        atol=1.0e-6,
    )
    assert np.allclose(
        physical.hit_joint_pos,
        mirror_joint_values(canonical.hit_joint_pos, LAB_JOINT_NAMES),
        atol=1.0e-6,
    )
    assert np.allclose(
        physical.hit_joint_vel,
        mirror_joint_values(canonical.hit_joint_vel, LAB_JOINT_NAMES),
        atol=1.0e-6,
    )
    assert np.allclose(
        physical.hit_anchor_pos_w,
        mirror_polar_vectors(canonical.hit_anchor_pos_w),
        atol=1.0e-6,
    )


def test_reference_wrapper_matches_canonical_scheduler_through_full_cycle(mirrored_motion_banks):
    hit, move = mirrored_motion_banks
    canonical_scheduler = DoublesReferenceScheduler(hit, move)
    physical_scheduler = MirroredReferenceScheduler(DoublesReferenceScheduler(hit, move))

    canonical_pelvis = np.array([0.0, 0.35, 0.75], dtype=np.float32)
    canonical_torso = np.array([0.0, 0.35, 0.82], dtype=np.float32)
    canonical_target = np.array([0.45, 0.30, 1.0], dtype=np.float32)
    canonical_velocity = np.array([3.0, 0.2, 0.5], dtype=np.float32)
    canonical_rpy = np.array([0.03, -0.04, 0.05], dtype=np.float32)
    canonical_angular = np.array([0.06, -0.07, 0.08], dtype=np.float32)

    canonical = canonical_scheduler.reset(canonical_pelvis, canonical_torso)
    physical = physical_scheduler.reset(
        mirror_polar_vectors(canonical_pelvis),
        mirror_polar_vectors(canonical_torso),
    )
    _assert_reference_mirror(canonical, physical)
    assert physical_scheduler.outward_target_y == -canonical_scheduler.outward_target_y
    assert physical_scheduler.home_y == -canonical_scheduler.home_y

    def update_pair(tts):
        canonical_reference = canonical_scheduler.update(
            tts,
            canonical_target,
            canonical_velocity,
            canonical_pelvis,
            canonical_torso,
            canonical_rpy,
            canonical_angular,
            dt=0.02,
        )
        physical_reference = physical_scheduler.update(
            tts,
            mirror_polar_vectors(canonical_target),
            mirror_polar_vectors(canonical_velocity),
            mirror_polar_vectors(canonical_pelvis),
            mirror_polar_vectors(canonical_torso),
            mirror_rpy(canonical_rpy),
            mirror_axial_vectors(canonical_angular),
            dt=0.02,
        )
        _assert_reference_mirror(canonical_reference, physical_reference)
        return canonical_reference

    for tts in (0.40, 0.30, 0.20, 0.10, 0.0):
        update_pair(tts)
    for _ in range(12):
        update_pair(-0.1)

    canonical_pelvis[1] = canonical_scheduler.outward_target_y
    canonical_torso[1] = canonical_scheduler.outward_target_y
    for _ in range(20):
        reference = update_pair(-0.5)
        if reference.state.name == "OUTWARD_HOLD":
            break
    assert reference.state.name == "OUTWARD_HOLD"

    for _ in range(150):
        reference = update_pair(-0.5)
    assert reference.state.name == "RETURN"

    canonical_pelvis[1] = canonical_scheduler.home_y
    canonical_torso[1] = canonical_scheduler.home_y
    for _ in range(20):
        reference = update_pair(-0.5)
        if reference.state.name == "HOME_HOLD":
            break
    assert reference.state.name == "HOME_HOLD"


def test_mirrored_external_hit_and_outward_target_use_canonical_coordinates(
    mirrored_motion_banks,
):
    hit, move = mirrored_motion_banks
    canonical = DoublesReferenceScheduler(hit, move, external_control=True)
    physical = MirroredReferenceScheduler(canonical)
    physical_pelvis = np.array([0.0, -0.35, 0.75], dtype=np.float32)
    physical_torso = np.array([0.0, -0.35, 0.82], dtype=np.float32)
    physical.reset(physical_pelvis, physical_torso)

    physical.set_external_outward_target(-0.80)
    assert canonical.pending_outward_target_valid
    assert np.isclose(canonical.pending_outward_target_y, 0.80)
    physical.set_external_home_target(-0.35)
    assert np.isclose(canonical.home_y, 0.35)
    assert np.isclose(physical.home_y, -0.35)

    target = np.array([0.45, -0.25, 1.0], dtype=np.float32)
    velocity = np.array([3.0, -0.2, 0.5], dtype=np.float32)
    ball_velocity = np.array([-3.0, 0.1, -0.5], dtype=np.float32)
    physical.set_external_hit(target, velocity, 0.40, ball_velocity)
    assert canonical.state.name == "HIT"
    assert np.allclose(canonical.racket_target, mirror_polar_vectors(target))
    assert np.allclose(canonical.target_velocity, mirror_polar_vectors(velocity))


def test_mirrored_external_base_target_starts_physical_outward_and_return(
    mirrored_motion_banks,
):
    hit, move = mirrored_motion_banks
    canonical = DoublesReferenceScheduler(hit, move, external_control=True)
    physical = MirroredReferenceScheduler(canonical)
    physical_pelvis = np.array([0.0, -0.35, 0.75], dtype=np.float32)
    physical.reset(physical_pelvis, physical_pelvis)

    physical.set_external_base_target(
        np.array([0.0, -0.80], dtype=np.float32),
        return_target_y=-0.35,
    )
    assert canonical.state.name == "OUTWARD"
    assert np.isclose(canonical.outward_target_y, 0.80)
    assert np.isclose(physical.outward_target_y, -0.80)
    outward = physical.output()
    assert np.isclose(outward.target_base[1], -0.45)

    physical_pelvis[1] = -0.80
    physical.update(
        -0.5,
        np.zeros(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
        physical_pelvis,
        physical_pelvis,
        np.zeros(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
    )
    physical.set_external_base_target(np.array([0.0, -0.35], dtype=np.float32))
    assert canonical.state.name == "RETURN"
    assert np.isclose(canonical.home_y, 0.35)
    assert np.isclose(physical.home_y, -0.35)


def test_mirrored_external_return_keeps_physical_x_anchor_unchanged(
    mirrored_motion_banks,
):
    canonical = DoublesReferenceScheduler(
        *mirrored_motion_banks,
        external_control=True,
        episode_start_x=0.10,
    )
    physical = MirroredReferenceScheduler(canonical)
    pelvis = np.array([0.04, -0.35, 0.75], dtype=np.float32)
    physical.reset(pelvis, pelvis)
    physical.set_external_base_target(
        np.array([0.04, -0.80], dtype=np.float32),
        return_target_y=-0.35,
    )

    pelvis = np.array([0.02, -0.80, 0.75], dtype=np.float32)
    physical.update(
        -0.5,
        np.zeros(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
        pelvis,
        pelvis,
        np.zeros(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
    )
    physical.set_external_base_target(np.array([0.02, -0.35], dtype=np.float32))

    assert np.isclose(canonical.episode_start_x, 0.10)
    assert np.isclose(physical.episode_start_x, 0.10)
    assert np.isclose(canonical.output().target_base[0], 0.08)
    assert np.isclose(physical.output().target_base[0], 0.08)


def test_external_control_disables_autonomous_tts_trigger(mirrored_motion_banks):
    scheduler = DoublesReferenceScheduler(*mirrored_motion_banks, external_control=True)
    base = np.array([0.0, 0.35, 0.75], dtype=np.float32)
    scheduler.reset(base, base)
    output = scheduler.update(
        0.40,
        np.array([0.45, 0.2, 1.0], dtype=np.float32),
        np.array([3.0, 0.0, 0.5], dtype=np.float32),
        base,
        base,
        np.zeros(3, dtype=np.float32),
        np.zeros(3, dtype=np.float32),
    )
    assert output.state.name == "HOME_HOLD"


def test_mirrored_stationary_hit_home_preserves_physical_home_and_phase_set(
    mirrored_motion_banks,
):
    canonical = DoublesReferenceScheduler(
        *mirrored_motion_banks,
        external_control=True,
        stationary_hit_test="hit_home",
    )
    physical = MirroredReferenceScheduler(canonical)
    pelvis = np.array([0.18, -0.20, 0.75], dtype=np.float32)
    target = np.array([0.45, -0.30, 1.0], dtype=np.float32)
    velocity = np.array([3.0, -0.1, 0.5], dtype=np.float32)
    output = physical.reset(pelvis, pelvis)
    seen = {output.state}

    physical.set_external_hit(target, velocity, 0.40)
    for tts in (0.30, 0.20, 0.10, 0.0, *([0.40] * 9)):
        output = physical.update(
            tts,
            target,
            velocity,
            pelvis,
            pelvis,
            np.zeros(3, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
        )
        seen.add(output.state)

    assert output.state == DoublesPhase.HOME_HOLD
    assert seen == {DoublesPhase.HIT, DoublesPhase.HOME_HOLD}
    assert np.isclose(physical.home_y, pelvis[1])
    assert np.allclose(output.target_base, np.zeros(2, dtype=np.float32))


def test_mirrored_bootstrap_holds_outward_until_planner_return(mirrored_motion_banks):
    canonical = DoublesReferenceScheduler(
        *mirrored_motion_banks, external_control=True
    )
    physical = MirroredReferenceScheduler(canonical)
    pelvis = np.array([0.0, -0.9125, 0.75], dtype=np.float32)
    physical.reset(pelvis, pelvis)
    physical.bootstrap_outward_hold(home_y=-0.20, outward_y=-0.9125)
    assert physical.state.name == "OUTWARD_HOLD"
    assert np.isclose(physical.home_y, -0.20)
    assert np.isclose(physical.outward_target_y, -0.9125)

    for _ in range(250):
        output = physical.update(
            -0.5,
            np.zeros(3, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
            pelvis,
            pelvis,
            np.zeros(3, dtype=np.float32),
            np.zeros(3, dtype=np.float32),
        )
    assert output.state.name == "OUTWARD_HOLD"

    physical.set_external_base_target(np.array([0.0, -0.20], dtype=np.float32))
    assert physical.state.name == "RETURN"


def test_v10_relative_x_is_unchanged_by_full_left_mirror(mirrored_motion_banks):
    canonical = DoublesReferenceScheduler(
        *mirrored_motion_banks,
        v10_relative_x=True,
        reference_end_forces_hold=True,
        latch_hold_x_on_entry=True,
    )
    physical = MirroredReferenceScheduler(canonical)
    pelvis = np.array([-0.161, 0.10, 0.75], dtype=np.float32)
    physical.reset(pelvis, pelvis)
    assert np.isclose(physical.canonical_pelvis_x(), 0.18)
    assert np.isclose(physical.outward_target_y, -0.9125)

    canonical._start_outward()
    output = physical.output()
    assert output.target_base[1] < 0.0
    assert np.isclose(output.target_base[0], 0.0)

    canonical._pelvis_position = np.array([-0.190, 0.80, 0.75], dtype=np.float32)
    canonical._start_outward_hold("reference_end")
    canonical._pelvis_position[0] = -0.180
    assert np.isclose(physical.output().target_base[0], -0.010, atol=1.0e-6)


def test_v11_common_hold_uses_existing_full_left_mirror(mirrored_motion_banks):
    asset = DEPLOY_ROOT / "assets/common_hold/source29_0368_upright_fk_v1.npz"
    common_q = np.load(asset)["joint_pos"][0]
    canonical = DoublesReferenceScheduler(
        *mirrored_motion_banks,
        transition_s=0.0,
        v10_relative_x=True,
        reference_end_forces_hold=True,
        latch_hold_x_on_entry=True,
        common_hold_pose_path=str(asset),
        common_hold_pose_sha256=(
            "9565a8ed1ba22cfc0d759d0a8327dc7dd37989c242d4f4337dcc79c796ea3a8f"
        ),
    )
    physical = MirroredReferenceScheduler(canonical)
    pelvis = np.array([-0.161, -0.20, 0.75], dtype=np.float32)
    physical.reset(pelvis, pelvis)
    canonical._start_outward()
    canonical.transition_remaining_s = 0.0
    canonical.reference_transition_remaining_s = 0.0
    canonical._start_outward_hold("reference_end")
    canonical.reference_transition_remaining_s = 0.0
    output = physical.output()
    assert np.array_equal(output.joint_pos, mirror_joint_values(common_q, LAB_JOINT_NAMES))
    assert np.isclose(output.target_base[0], 0.0)
    assert physical.outward_target_y < 0.0


def test_runtime_motion_config_uses_physical_y_through_left_mirror(mirrored_motion_banks):
    canonical = DoublesReferenceScheduler(
        *mirrored_motion_banks,
        v10_relative_x=True,
        reference_end_forces_hold=True,
        latch_hold_x_on_entry=True,
    )
    physical = MirroredReferenceScheduler(canonical)
    pelvis = np.array([0.18, -0.10, 0.75], dtype=np.float32)
    physical.reset(pelvis, pelvis)
    physical.stage_runtime_motion_config(
        command_id="mirror-config",
        goal_x=0.20,
        outward_y=-0.90,
        home_y=-0.10,
        outward_hold_s=4.0,
    )
    assert np.isclose(canonical._pending_motion_config["outward_y"], 0.90)
    assert np.isclose(canonical._pending_motion_config["home_y"], 0.10)
    status = physical.runtime_motion_config_status()
    assert np.isclose(status["pending"]["outward_y"], -0.90)
    assert np.isclose(status["pending"]["home_y"], -0.10)
