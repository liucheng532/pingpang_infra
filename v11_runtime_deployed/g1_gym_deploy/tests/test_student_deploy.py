from __future__ import annotations

import json
import queue
import sys
import time
from pathlib import Path

import numpy as np
import pytest
import torch


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_ROOT))

from envs.history_wrapper import (
    FRAME_DIM,
    FRAME_TERMS,
    HistoryWrapper,
    TEACHER_HISTORY_DIM,
    TEACHER_TASK_TERMS,
    TEACHER_TERM_DIMS,
)
from scripts.export_student_onnx import (
    EXPECTED_HIT_TEACHER_SHA256,
    EXPECTED_MOVE_TEACHER_SHA256,
    OUTPUT_BASENAME,
    StudentPolicy,
    export_student,
    sha256_file,
)
from utils.async_deploy_recorder import AsyncDeployRecorder, RECORD_DTYPE, parse_planner_monitor_payload
from utils.cheetah_state_estimator import StateEstimator, _LATENCY_COMMAND_TRACE
from utils.data_utils import MotionCommand, MoveMotionBank
from utils.doubles_reference_scheduler import DoublesPhase, DoublesReferenceScheduler
from utils.g1_pelvis_pose import G1PelvisPoseEstimator
from utils.joint_mapping import FROM_GYM_TO_LAB, FROM_LAB_TO_GYM, LAB_JOINT_NAMES
from utils.math import matrix_from_quat_wxyz, quat_from_rpy_wxyz, rotmat_to_quat_xyzw


V11_COMMON_HOLD_ASSET = DEPLOY_ROOT / "assets/common_hold/source29_0368_upright_fk_v1.npz"
V11_COMMON_HOLD_SHA256 = "9565a8ed1ba22cfc0d759d0a8327dc7dd37989c242d4f4337dcc79c796ea3a8f"


def _write_hit_bank(root: Path, motion_count=2, frames=80):
    root.mkdir()
    lines = ["index backhand target\n"]
    targets = ((0.45, -0.3, 1.0), (0.45, 0.3, 1.0))
    for index in range(motion_count):
        manifest_index = "0368" if index == 0 else f"{index:04d}"
        folder = root / f"{root.name}-{manifest_index}:v0"
        folder.mkdir()
        time_axis = np.arange(frames, dtype=np.float32)[:, None]
        joint_axis = np.arange(29, dtype=np.float32)[None, :] * 0.001
        q = index + time_axis * 0.01 + joint_axis
        qd = np.full_like(q, 0.1 + index)
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
        tx, ty, tz = targets[index]
        lines.append(f"{manifest_index} {index % 2} {tx}, {ty}, {tz}\n")
    (root / "dataindex.csv").write_text("".join(lines), encoding="utf-8")


def _write_move_bank(root: Path, frames=100):
    root.mkdir()
    rows = [
        ("0000", 0, -0.45),
        ("0001", 0, -0.60),
        ("0002", 1, 0.45),
        ("0003", 1, 0.60),
    ]
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
def motion_banks(tmp_path):
    hit_root = tmp_path / "0302_combined"
    move_root = tmp_path / "0718-move-160-80hz"
    _write_hit_bank(hit_root)
    _write_move_bank(move_root)
    hit = MotionCommand(str(hit_root), device="cpu")
    move = MoveMotionBank(str(move_root), expected_count=4, expected_fps=80.0)
    return hit, move


def _update_scheduler(scheduler, tts, target, velocity, pelvis, torso, **kwargs):
    pelvis_rpy = kwargs.pop("pelvis_rpy", np.zeros(3, dtype=np.float32))
    pelvis_angular_velocity_w = kwargs.pop(
        "pelvis_angular_velocity_w", np.zeros(3, dtype=np.float32)
    )
    return scheduler.update(
        tts,
        target,
        velocity,
        pelvis,
        torso,
        pelvis_rpy,
        pelvis_angular_velocity_w,
        **kwargs,
    )


def test_move_manifest_and_nearest_label(motion_banks):
    _, move = motion_banks
    assert move.num_motions == 4
    assert set(move.labels.tolist()) == {0, 1}
    assert all(q.shape[1] == 29 for q in move.joint_pos)
    assert np.allclose(move.fps, 80.0)
    assert move.labels[move.nearest_index(0.52, label=1)] == 1
    assert move.labels[move.nearest_index(-0.52, label=0)] == 0


def test_v11_move_pool_filters_source_ids_without_changing_legacy_default(tmp_path):
    root = tmp_path / "0718-move-160-80hz"
    _write_move_bank(root)
    legacy = MoveMotionBank(str(root), expected_count=4, expected_fps=80.0)
    v11 = MoveMotionBank(
        str(root),
        expected_count=4,
        expected_fps=80.0,
        excluded_source_ids=[1, 3],
        expected_active_count=2,
    )
    assert legacy.active_count == 4
    assert v11.active_count == 2
    assert v11.source_ids[v11.nearest_index(-0.52, label=0)] == 0
    assert v11.source_ids[v11.nearest_index(0.52, label=1)] == 2


def test_move_nearest_index_skips_disabled_motion(motion_banks):
    _, move = motion_banks
    assert MoveMotionBank.DISABLED_INDICES == frozenset({41, 47})
    move.DISABLED_INDICES = frozenset({1})
    assert move.nearest_index(-0.60, label=0) == 0


def test_hit_reference_lead_is_applied_once_and_remains_fixed(motion_banks):
    hit, move = motion_banks
    base = DoublesReferenceScheduler(hit, move, external_control=True, hit_reference_lead_steps=0)
    lead = DoublesReferenceScheduler(hit, move, external_control=True, hit_reference_lead_steps=1)
    pelvis = np.array([0.0, 0.1, 0.8], dtype=np.float32)
    torso = np.array([0.0, 0.1, 0.85], dtype=np.float32)
    target = np.array([0.45, -0.3, 1.0], dtype=np.float32)
    velocity = np.array([2.0, 0.0, 0.5], dtype=np.float32)
    for scheduler in (base, lead):
        scheduler.reset(pelvis, torso)
        scheduler.set_external_hit(target, velocity, 0.40)
    assert lead.hit_step == base.hit_step + 1

    base_out = _update_scheduler(base, 0.38, target, velocity, pelvis, torso)
    lead_out = _update_scheduler(lead, 0.38, target, velocity, pelvis, torso)
    assert lead_out.reference_step == base_out.reference_step + 1


def test_external_hit_start_matches_local_first_hit_frame(motion_banks):
    hit, move = motion_banks
    local = DoublesReferenceScheduler(
        hit, move, external_control=False, hit_reference_lead_steps=1
    )
    external = DoublesReferenceScheduler(
        hit, move, external_control=True, hit_reference_lead_steps=1
    )
    pelvis = np.array([0.0, 0.1, 0.8], dtype=np.float32)
    torso = np.array([0.0, 0.1, 0.85], dtype=np.float32)
    target = np.array([0.45, -0.3, 1.0], dtype=np.float32)
    velocity = np.array([2.0, 0.0, 0.5], dtype=np.float32)
    local.reset(pelvis, torso)
    external.reset(pelvis, torso)

    local_output = _update_scheduler(local, 0.40, target, velocity, pelvis, torso)
    external_output = _update_scheduler(
        external,
        0.40,
        target,
        velocity,
        pelvis,
        torso,
        start_external_hit=True,
    )

    assert external_output.state == local_output.state == DoublesPhase.HIT
    assert external_output.reference_step == local_output.reference_step
    assert external_output.hit_motion_index == local_output.hit_motion_index
    assert np.isclose(external_output.strike_time, local_output.strike_time)
    assert np.array_equal(external_output.racket_target, local_output.racket_target)
    assert np.array_equal(external_output.target_velocity, local_output.target_velocity)
    assert np.array_equal(external_output.hit_anchor_pos_w, local_output.hit_anchor_pos_w)
    assert np.array_equal(external_output.joint_pos, local_output.joint_pos)
    assert np.array_equal(external_output.joint_vel, local_output.joint_vel)


def test_hit_reference_lead_clips_at_motion_end(motion_banks):
    hit, move = motion_banks
    scheduler = DoublesReferenceScheduler(
        hit, move, external_control=True, hit_reference_lead_steps=2
    )
    pelvis = np.array([0.0, 0.1, 0.8], dtype=np.float32)
    torso = np.array([0.0, 0.1, 0.85], dtype=np.float32)
    scheduler.reset(pelvis, torso)
    scheduler._hit_lengths[:] = 40
    scheduler.set_external_hit(
        np.array([0.45, -0.3, 1.0], dtype=np.float32),
        np.array([2.0, 0.0, 0.5], dtype=np.float32),
        -0.5,
    )
    assert scheduler.hit_step == 39


class _FakeObsEnv:
    num_history_length = 10
    num_obs = 172
    num_envs = 1
    device = "cpu"

    def __init__(self, obs):
        self.obs = obs.clone()
        self.events = []

    def reset(self):
        return self.obs.clone()

    def get_obs(self):
        return self.obs.clone()

    def apply_action(self, action, hard_reset=False):
        self.events.append("apply")
        self.obs[:, FRAME_TERMS["actions"]] = action

    def observe(self):
        self.events.append("observe")
        return self.obs.clone()

    def step(self, action):
        self.apply_action(action)
        return self.observe()


class _Reference:
    state = DoublesPhase.HOME_HOLD


class _FakeTeacherObsEnv(_FakeObsEnv):
    def __init__(self, obs):
        super().__init__(obs)
        self.reference = _Reference()
        self.teacher_frame = torch.arange(167, dtype=torch.float32).reshape(1, -1)

    def get_teacher_frame(self, _obs):
        return self.teacher_frame.clone()


def _sentinel_obs(offset=0.0):
    obs = torch.zeros(1, 172)
    for value, term_slice in enumerate(FRAME_TERMS.values(), start=1):
        obs[:, term_slice] = float(value) + offset
    obs[:, FRAME_DIM:] = torch.arange(6, dtype=torch.float32) + 50.0 + offset
    return obs


def test_history_wrapper_term_major_abi_reset_and_shift():
    env = _FakeObsEnv(_sentinel_obs())
    wrapper = HistoryWrapper(env)
    result = wrapper.reset()
    history = result["obs_history"]
    assert history.shape == (1, 1666)

    offset = 0
    for name, term_slice in FRAME_TERMS.items():
        width = term_slice.stop - term_slice.start
        block = history[:, offset : offset + width * 10].reshape(1, 10, width)
        expected = 0.0 if name == "actions" else float(list(FRAME_TERMS).index(name) + 1)
        assert torch.all(block == expected)
        offset += width * 10
    assert offset == 1660
    assert torch.equal(history[0, 1660:1666], torch.arange(6, dtype=torch.float32) + 50.0)

    env.obs = _sentinel_obs(offset=100.0)
    shifted = wrapper.get_obs()["obs_history"]
    offset = 0
    for name, term_slice in FRAME_TERMS.items():
        width = term_slice.stop - term_slice.start
        block = shifted[:, offset : offset + width * 10].reshape(1, 10, width)
        old = 0.0 if name == "actions" else float(list(FRAME_TERMS).index(name) + 1)
        new = float(list(FRAME_TERMS).index(name) + 101)
        assert torch.all(block[:, :9] == old)
        assert torch.all(block[:, 9:] == new)
        offset += width * 10
    assert torch.equal(shifted[0, 1660:1666], torch.arange(6, dtype=torch.float32) + 150.0)


def test_history_apply_does_not_shift_and_observe_appends_previous_action():
    env = _FakeObsEnv(_sentinel_obs())
    wrapper = HistoryWrapper(env)
    wrapper.reset()
    before_apply = wrapper.obs_history.clone()
    action = torch.arange(1, 30, dtype=torch.float32).reshape(1, -1)

    wrapper.apply_action(action)
    assert torch.equal(wrapper.obs_history, before_apply)

    observed = wrapper.observe()
    assert env.events == ["apply", "observe"]
    action_offset = 0
    for name, term_slice in FRAME_TERMS.items():
        if name == "actions":
            break
        action_offset += (term_slice.stop - term_slice.start) * 10
    action_history = observed["obs_history"][:, action_offset : action_offset + 290].reshape(1, 10, 29)
    assert torch.equal(action_history[:, -1], action)


def test_teacher_task_history_freezes_through_move_and_resets_on_next_hit():
    env = _FakeTeacherObsEnv(_sentinel_obs())
    wrapper = HistoryWrapper(env, enable_teacher_history=True, teacher_history_mode="distill-reset")
    assert wrapper.reset()["teacher_obs_history"].shape == (1, TEACHER_HISTORY_DIM)

    env.teacher_frame += 1000.0
    env.reference.state = DoublesPhase.HIT
    first_hit = wrapper.get_obs()["teacher_obs_history"]
    env.teacher_frame += 1000.0
    env.reference.state = DoublesPhase.OUTWARD
    move = wrapper.get_obs()["teacher_obs_history"]
    env.teacher_frame += 1000.0
    env.reference.state = DoublesPhase.HIT
    second_hit = wrapper.get_obs()["teacher_obs_history"]

    offset = 0
    frame_offset = 0
    for name, width in TEACHER_TERM_DIMS.items():
        first_block = first_hit[:, offset : offset + width * 10].reshape(1, 10, width)
        move_block = move[:, offset : offset + width * 10].reshape(1, 10, width)
        second_block = second_hit[:, offset : offset + width * 10].reshape(1, 10, width)
        if name in TEACHER_TASK_TERMS:
            assert torch.equal(move_block, first_block)
            expected = torch.arange(frame_offset, frame_offset + width) + 3000.0
            assert torch.equal(second_block[0], expected.repeat(10, 1))
        offset += width * 10
        frame_offset += width


class _FakeLCM:
    def subscribe(self, channel, callback):
        return channel


def test_joint_mapping_is_unique_and_invertible():
    estimator = StateEstimator(_FakeLCM())
    estimator.joint_pos = np.arange(29, dtype=np.float32)
    assert len(set(estimator.joint_idxs)) == 29
    assert estimator.get_dof_pos()[21] == 21

    gym = np.arange(29)
    lab = gym[list(FROM_GYM_TO_LAB)]
    reconstructed_gym = lab[list(FROM_LAB_TO_GYM)]
    assert np.array_equal(reconstructed_gym, gym)


def test_pelvis_fk_zero_waist_fixed_translation():
    estimator = G1PelvisPoseEstimator()
    torso_pos = np.array([0.4, 0.6, 0.8], dtype=np.float64)
    pelvis_pos, pelvis_quat = estimator.estimate(
        torso_pos,
        np.array([1.0, 0.0, 0.0, 0.0]),
        np.zeros(29),
    )
    assert np.allclose(pelvis_pos, [0.4039635, 0.6, 0.756], atol=1.0e-7)
    assert np.allclose(pelvis_quat, [1.0, 0.0, 0.0, 0.0], atol=1.0e-7)


def test_pelvis_fk_round_trip_random_waist_angles():
    estimator = G1PelvisPoseEstimator()
    rng = np.random.default_rng(42)
    offset = estimator.WAIST_ROLL_OFFSET_M
    for _ in range(100):
        pelvis_pos_expected = rng.uniform([-0.1, 0.3, 0.7], [0.1, 0.9, 0.9])
        pelvis_quat_expected = quat_from_rpy_wxyz(*rng.uniform(-0.4, 0.4, size=3))
        waist = rng.uniform([-1.0, -0.3, -0.3], [1.0, 0.3, 0.3])
        yaw, roll, pitch = waist
        rotation_origin_pelvis = matrix_from_quat_wxyz(pelvis_quat_expected)
        rotation_pelvis_torso = (
            estimator._rot_z(yaw) @ estimator._rot_x(roll) @ estimator._rot_y(pitch)
        )
        position_pelvis_torso = estimator._rot_z(yaw) @ offset
        torso_pos = pelvis_pos_expected + rotation_origin_pelvis @ position_pelvis_torso
        rotation_origin_torso = rotation_origin_pelvis @ rotation_pelvis_torso
        torso_quat_xyzw = rotmat_to_quat_xyzw(rotation_origin_torso)
        torso_quat_wxyz = torso_quat_xyzw[[3, 0, 1, 2]]
        joints = np.zeros(29)
        joints[12:15] = waist
        pelvis_pos, pelvis_quat = estimator.estimate(torso_pos, torso_quat_wxyz, joints)
        assert np.allclose(pelvis_pos, pelvis_pos_expected, atol=1.0e-6)
        assert np.allclose(
            matrix_from_quat_wxyz(pelvis_quat), rotation_origin_pelvis, atol=1.0e-6
        )


def test_pelvis_fk_rejects_invalid_inputs():
    estimator = G1PelvisPoseEstimator()
    joints = np.zeros(29)
    with pytest.raises(ValueError, match="near-zero norm"):
        estimator.estimate(np.zeros(3), np.zeros(4), joints)
    joints[13] = 0.7
    with pytest.raises(ValueError, match="exceeds URDF limits"):
        estimator.estimate(np.zeros(3), [1.0, 0.0, 0.0, 0.0], joints)


def test_pelvis_fk_accepts_small_measured_waist_limit_excursion():
    estimator = G1PelvisPoseEstimator()
    joints = np.zeros(29)
    joints[13] = -0.536406397819519
    pelvis_pos, pelvis_quat = estimator.estimate(
        np.array([0.2, 0.3, 0.8]),
        np.array([1.0, 0.0, 0.0, 0.0]),
        joints,
    )
    assert np.isfinite(pelvis_pos).all()
    assert np.isfinite(pelvis_quat).all()


@pytest.mark.parametrize(
    ("torso_pos", "torso_quat", "waist", "expected_pelvis"),
    [
        (
            [0.34932402, 0.60361385, 0.7560627],
            [0.98890096, 0.11979602, 0.08332529, 0.02794165],
            [0.00145972, -0.01059593, -0.01442676],
            [0.3450301, 0.61454815, 0.7134750],
        ),
        (
            [0.09398303, 0.59038925, 0.8148392],
            [0.98935914, 0.08867301, 0.00857402, -0.11503080],
            [-0.09824295, -0.00619471, -0.03647030],
            [0.09649085, 0.5979448, 0.77138424],
        ),
    ],
)
def test_pelvis_fk_recorded_robot_golden(torso_pos, torso_quat, waist, expected_pelvis):
    joints = np.zeros(29, dtype=np.float64)
    joints[12:15] = waist
    pelvis_pos, _ = G1PelvisPoseEstimator().estimate(torso_pos, torso_quat, joints)
    assert np.allclose(pelvis_pos, expected_pelvis, atol=1.0e-6)


def _advance_to_home(scheduler, target, velocity):
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    _update_scheduler(scheduler, 0.40, target, velocity, base, base)
    for tts in (0.30, 0.20, 0.10, 0.0):
        _update_scheduler(scheduler, tts, target, velocity, base, base)
    assert scheduler.state == DoublesPhase.POST_DELAY

    for _ in range(9):
        _update_scheduler(scheduler, -0.1, target, velocity, base, base)
        assert scheduler.state == DoublesPhase.POST_DELAY
    for _ in range(5):
        _update_scheduler(scheduler, -0.1, target, velocity, base, base)
    assert scheduler.state == DoublesPhase.OUTWARD
    assert scheduler.move_motions.labels[scheduler.move_motion_index] == 1

    base[1] = scheduler.outward_target_y
    for _ in range(20):
        _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        if scheduler.state == DoublesPhase.OUTWARD_HOLD:
            break
    assert scheduler.state == DoublesPhase.OUTWARD_HOLD
    for _ in range(150):
        _update_scheduler(scheduler, -0.5, target, velocity, base, base)
    assert scheduler.state == DoublesPhase.RETURN
    assert scheduler.move_motions.labels[scheduler.move_motion_index] == 0

    base[1] = scheduler.home_y
    for _ in range(20):
        _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        if scheduler.state == DoublesPhase.HOME_HOLD:
            break
    assert scheduler.state == DoublesPhase.HOME_HOLD
    return base


def test_scheduler_state_machine_blend_and_no_duplicate_hit(motion_banks):
    hit, move = motion_banks
    scheduler = DoublesReferenceScheduler(hit, move)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.30, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    home = scheduler.reset(base, base)
    assert home.state == DoublesPhase.HOME_HOLD
    assert move.labels[home.move_motion_index] == 0
    assert np.allclose(home.phase, [0, 0, 0, 0, 0, 1])

    first_hit = _update_scheduler(scheduler, 0.40, target, velocity, base, base)
    assert first_hit.state == DoublesPhase.HIT
    assert np.allclose(first_hit.phase, [0, 0, 0, 0, 0, 1])
    assert np.isclose(first_hit.phase.sum(), 1.0)
    for tts in (0.38, 0.36, 0.34, 0.32, 0.30):
        blended = _update_scheduler(scheduler, tts, target, velocity, base, base)
        assert np.isclose(blended.phase.sum(), 1.0)
    assert np.allclose(blended.phase, [1, 0, 0, 0, 0, 0], atol=1.0e-6)

    scheduler.reset(base, base)
    base = _advance_to_home(scheduler, target, velocity)
    for _ in range(500):
        held = _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        assert held.state == DoublesPhase.HOME_HOLD
        assert np.isfinite(held.command).all()
        assert np.isfinite(held.phase).all()
    assert _update_scheduler(scheduler, 0.40, target, velocity, base, base).state == DoublesPhase.HIT


def test_stationary_hit_home_external_cycle_never_enters_locomotion(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        external_control=True,
        stationary_hit_test="hit_home",
    )
    base = np.array([0.18, 0.20, 0.75], dtype=np.float32)
    target = np.array([0.45, 0.30, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    output = scheduler.reset(base, base)
    seen = {output.state}

    output = _update_scheduler(
        scheduler,
        0.40,
        target,
        velocity,
        base,
        base,
        start_external_hit=True,
    )
    seen.add(output.state)
    for tts in (0.30, 0.20, 0.10, 0.0):
        output = _update_scheduler(scheduler, tts, target, velocity, base, base)
        seen.add(output.state)
    assert output.state == DoublesPhase.HIT
    assert scheduler.stationary_hit_crossed

    strike_time = output.strike_time
    output = _update_scheduler(scheduler, 0.40, target, velocity, base, base)
    assert output.state == DoublesPhase.HIT
    assert output.strike_time < strike_time
    for _ in range(8):
        output = _update_scheduler(scheduler, 0.40, target, velocity, base, base)
        seen.add(output.state)
    assert output.state == DoublesPhase.HOME_HOLD
    assert seen == {DoublesPhase.HIT, DoublesPhase.HOME_HOLD}
    assert output.strike_time == -0.5
    assert np.array_equal(output.racket_target, np.zeros(3, dtype=np.float32))
    assert np.array_equal(output.target_velocity, np.zeros(3, dtype=np.float32))

    output = _update_scheduler(
        scheduler,
        0.40,
        target,
        velocity,
        base,
        base,
        start_external_hit=True,
    )
    assert output.state == DoublesPhase.HIT


def test_stationary_hit_home_ignores_external_motion_commands(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        external_control=True,
        stationary_hit_test="hit_home",
        v10_relative_x=True,
    )
    base = np.array([0.18, 0.20, 0.75], dtype=np.float32)
    scheduler.reset(base, base)
    initial = scheduler.runtime_motion_config_status()

    scheduler.stage_runtime_motion_config(
        command_id="ignored",
        goal_x=0.30,
        outward_y=0.90,
        home_y=-0.20,
        outward_hold_s=8.0,
    )
    scheduler.set_external_outward_target(0.90)
    scheduler.set_external_home_target(-0.20)
    scheduler.set_external_base_target(
        np.array([0.18, 0.90], dtype=np.float32),
        return_target_y=-0.20,
    )

    output = scheduler.output()
    status = scheduler.runtime_motion_config_status()
    assert output.state == DoublesPhase.HOME_HOLD
    assert np.allclose(output.target_base, np.zeros(2, dtype=np.float32))
    assert status["active"] == initial["active"]
    assert status["pending"] is None
    assert not scheduler.pending_outward_target_valid
    with pytest.raises(RuntimeError, match="disabled by stationary hit_home"):
        scheduler.bootstrap_outward_hold(0.20, outward_y=0.90)


def test_scheduler_rejects_unknown_stationary_hit_mode(motion_banks):
    with pytest.raises(ValueError, match="stationary_hit_test"):
        DoublesReferenceScheduler(*motion_banks, stationary_hit_test="hit")


@pytest.mark.parametrize(
    "state",
    [DoublesPhase.OUTWARD, DoublesPhase.OUTWARD_HOLD, DoublesPhase.RETURN],
)
def test_new_hit_preempts_locomotion_state(motion_banks, state):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._set_state(state)
    scheduler.transition_remaining_s = 0.0

    _update_scheduler(scheduler, -0.5, target, velocity, base, base)
    hit = _update_scheduler(scheduler, 0.40, target, velocity, base, base)
    assert hit.state == DoublesPhase.HIT
    assert np.isclose(hit.phase[int(state)], 1.0)
    assert scheduler.hit_armed is False

    duplicate = _update_scheduler(scheduler, 0.38, target, velocity, base, base)
    assert duplicate.state == DoublesPhase.HIT


@pytest.mark.parametrize("state", [DoublesPhase.HIT, DoublesPhase.POST_DELAY])
def test_new_hit_does_not_preempt_current_hit_sequence(motion_banks, state):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._set_state(state)
    scheduler.hit_armed = True

    output = _update_scheduler(scheduler, 0.40, target, velocity, base, base)
    assert output.state == state


def test_scheduler_uses_torso_for_hit_and_pelvis_for_base_command(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    pelvis = np.array([0.10, 0.00, 0.75], dtype=np.float32)
    torso = np.array([0.10, 0.60, 0.80], dtype=np.float32)
    racket_target = np.array([0.55, 0.30, 1.80], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(pelvis, torso)
    hit = _update_scheduler(scheduler, 0.40, racket_target, velocity, pelvis, torso)
    # torso-relative target y=-0.30 selects motion 0; pelvis-relative y=+0.30 selects motion 1.
    assert hit.hit_motion_index == 0

    scheduler.state = DoublesPhase.OUTWARD
    pelvis_moved = pelvis.copy()
    pelvis_moved[:2] += [0.03, 0.20]
    torso_unrelated = torso.copy()
    torso_unrelated[:2] += [-0.15, -0.25]
    scheduler._pelvis_position = pelvis_moved
    scheduler._torso_position = torso_unrelated
    output = scheduler.output()
    assert np.allclose(output.target_base, [-0.03, 0.7125], atol=1.0e-6)


def test_post_delay_starts_outward_after_fixed_delay(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(base, base)
    _update_scheduler(scheduler, 0.40, target, velocity, base, base)
    for tts in (0.30, 0.20, 0.10, 0.0):
        _update_scheduler(scheduler, tts, target, velocity, base, base)
    assert scheduler.state == DoublesPhase.POST_DELAY

    tilted = np.array([0.20, 0.0, 0.0], dtype=np.float32)
    moving_angular_velocity = np.array([1.0, 1.0, 0.0], dtype=np.float32)
    for _ in range(9):
        _update_scheduler(
            scheduler,
            -0.1,
            target,
            velocity,
            base,
            base,
            pelvis_rpy=tilted,
            pelvis_angular_velocity_w=moving_angular_velocity,
        )
        assert scheduler.state == DoublesPhase.POST_DELAY

    _update_scheduler(
        scheduler,
        -0.1,
        target,
        velocity,
        base,
        base,
        pelvis_rpy=tilted,
        pelvis_angular_velocity_w=moving_angular_velocity,
    )
    assert scheduler.state == DoublesPhase.OUTWARD


def test_outward_relocks_x_at_handoff(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._pelvis_position = np.array([0.25, 0.35, 1.0], dtype=np.float32)
    scheduler._start_outward()
    assert np.isclose(scheduler.episode_start_x, 0.25)
    assert np.isclose(scheduler.output().target_base[0], 0.0)


def test_v10_relative_x_uses_outward_entry_and_canonical_anchor(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks, v10_relative_x=True)
    startup = np.array([-0.161, 0.35, 1.0], dtype=np.float32)
    scheduler.reset(startup, startup)
    assert np.isclose(scheduler.canonical_pelvis_x(), 0.18)

    scheduler._pelvis_position = np.array([-0.251, 0.35, 1.0], dtype=np.float32)
    scheduler._start_outward()
    assert np.isclose(scheduler.goal_x, -0.251)
    assert np.isclose(scheduler.output().target_base[0], 0.0)

    duration = (scheduler.move_motions.lengths[scheduler.move_motion_index] - 1) / scheduler.move_motions.fps[
        scheduler.move_motion_index
    ]
    scheduler.segment_elapsed_s = float(duration / 2.0)
    assert np.isclose(scheduler.output().target_base[0], 0.0, atol=1.0e-6)


def test_v10_hold_x_latches_actual_entry_position(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        v10_relative_x=True,
        reference_end_forces_hold=True,
        latch_hold_x_on_entry=True,
    )
    pelvis = np.array([-0.161, 0.35, 1.0], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler._start_outward()
    scheduler._pelvis_position = np.array([-0.193, 0.80, 1.0], dtype=np.float32)
    scheduler._start_outward_hold("reference_end")
    assert np.isclose(scheduler.target_hold_x, -0.193)
    scheduler._pelvis_position[0] = -0.183
    assert np.isclose(scheduler.output().target_base[0], -0.010, atol=1.0e-6)

    scheduler._start_return()
    scheduler._pelvis_position = np.array([-0.188, scheduler.home_y, 1.0], dtype=np.float32)
    scheduler._start_home_hold()
    assert np.isclose(scheduler.home_hold_x, -0.188)
    scheduler._pelvis_position[0] = -0.180
    assert np.isclose(scheduler.output().target_base[0], -0.008, atol=1.0e-6)


def test_external_return_preserves_outward_x_anchor_after_drift(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks, external_control=True)
    pelvis = np.array([0.10, 0.20, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler._start_outward()
    scheduler._start_outward_hold("stable")

    scheduler._pelvis_position = np.array([0.05, 0.90, 0.75], dtype=np.float32)
    scheduler.set_external_base_target(np.array([0.05, 0.20], dtype=np.float32))

    output = scheduler.output()
    assert output.state == DoublesPhase.RETURN
    assert np.isclose(scheduler.episode_start_x, 0.10)
    assert np.isclose(output.target_base[0], 0.05)


def test_external_and_automatic_return_share_outward_x_anchor(motion_banks):
    automatic = DoublesReferenceScheduler(*motion_banks)
    external = DoublesReferenceScheduler(*motion_banks, external_control=True)
    pelvis = np.array([0.10, 0.20, 0.75], dtype=np.float32)
    for scheduler in (automatic, external):
        scheduler.reset(pelvis, pelvis)
        scheduler._start_outward()
        scheduler._start_outward_hold("stable")
        scheduler._pelvis_position = np.array([0.05, 0.90, 0.75], dtype=np.float32)

    automatic._start_return()
    external.set_external_base_target(np.array([0.05, 0.20], dtype=np.float32))

    automatic_output = automatic.output()
    external_output = external.output()
    assert np.isclose(automatic.episode_start_x, external.episode_start_x)
    assert np.array_equal(automatic_output.target_base, external_output.target_base)
    assert automatic_output.move_motion_index == external_output.move_motion_index
    assert np.array_equal(automatic_output.joint_pos, external_output.joint_pos)
    assert np.array_equal(automatic_output.joint_vel, external_output.joint_vel)


def test_fixed_episode_start_x_survives_multiple_outward_return_cycles(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        external_control=True,
        episode_start_x=0.10,
    )
    pelvis = np.array([-0.20, 0.20, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)

    for outward_x, return_x in ((0.04, 0.02), (-0.03, -0.05)):
        scheduler._pelvis_position = np.array([outward_x, 0.20, 0.75], dtype=np.float32)
        scheduler._start_outward()
        scheduler._start_outward_hold("stable")
        scheduler._pelvis_position = np.array([return_x, 0.90, 0.75], dtype=np.float32)
        scheduler.set_external_base_target(np.array([return_x, 0.20], dtype=np.float32))
        assert np.isclose(scheduler.episode_start_x, 0.10)
        scheduler._start_home_hold()


def test_direct_home_to_return_initializes_missing_x_anchor(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks, external_control=True)
    pelvis = np.array([0.30, 0.35, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler._pelvis_position = np.array([0.25, 0.35, 0.75], dtype=np.float32)

    scheduler.set_external_base_target(np.array([0.25, -0.20], dtype=np.float32))

    assert scheduler.state == DoublesPhase.RETURN
    assert np.isclose(scheduler.episode_start_x, 0.25)
    assert np.isclose(scheduler.output().target_base[0], 0.0)


def test_outward_reference_end_enters_stationary_hold_then_returns(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._start_outward()

    # Training enters OUTWARD_HOLD when the outbound reference ends, even if the
    # robot has not reached the target. The hold reference then remains fixed.
    for _ in range(100):
        output = _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        if scheduler.state == DoublesPhase.OUTWARD_HOLD:
            break
    assert scheduler.state == DoublesPhase.OUTWARD_HOLD
    assert output.outward_hold_entry_reason == "reference_end"
    assert output.reference_step == scheduler.move_motions.lengths[output.move_motion_index] - 1
    assert np.array_equal(output.joint_vel, np.zeros(29, dtype=np.float32))

    for _ in range(149):
        output = _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        assert output.state == DoublesPhase.OUTWARD_HOLD
    output = _update_scheduler(scheduler, -0.5, target, velocity, base, base)
    assert np.isclose(output.outward_hold_duration_s, 3.0)
    assert scheduler.state == DoublesPhase.RETURN


def test_outward_stability_enters_hold_with_training_thresholds(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._start_outward()
    base[1] = scheduler.outward_target_y
    for _ in range(12):
        output = _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        if output.state == DoublesPhase.OUTWARD_HOLD:
            break
    assert output.state == DoublesPhase.OUTWARD_HOLD
    assert output.outward_hold_entry_reason == "stable"


def test_v9_outward_hold_uses_nominal_upper_ready_legs_neutral_waist(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks, transition_s=0.0)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._start_outward()
    scheduler.transition_remaining_s = 0.0
    scheduler.reference_transition_remaining_s = 0.0
    scheduler._start_outward_hold("stable")
    scheduler.reference_transition_remaining_s = 0.0
    output = scheduler.output()

    for index, name in enumerate(LAB_JOINT_NAMES):
        if "shoulder" in name or "elbow" in name or "wrist" in name:
            assert output.joint_pos[index] == scheduler._hold_nominal_joint_pos[index]
        elif "hip" in name or "knee" in name or "ankle" in name:
            assert output.joint_pos[index] == scheduler._hold_ready_joint_pos[index]
        elif name in ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"):
            assert output.joint_pos[index] == 0.0
    assert np.array_equal(output.joint_vel, np.zeros(29, dtype=np.float32))


def test_v11_target_and_home_hold_share_the_fk_pose(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        transition_s=0.0,
        common_hold_pose_path=str(V11_COMMON_HOLD_ASSET),
        common_hold_pose_sha256=V11_COMMON_HOLD_SHA256,
    )
    expected = np.load(V11_COMMON_HOLD_ASSET)["joint_pos"][0]
    base = np.array([0.18, 0.35, 1.0], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._start_outward()
    scheduler.transition_remaining_s = 0.0
    scheduler.reference_transition_remaining_s = 0.0
    scheduler._start_outward_hold("reference_end")
    scheduler.reference_transition_remaining_s = 0.0
    target = scheduler.output()
    scheduler._start_return()
    scheduler.transition_remaining_s = 0.0
    scheduler.reference_transition_remaining_s = 0.0
    scheduler._start_home_hold()
    scheduler.reference_transition_remaining_s = 0.0
    home = scheduler.output()
    assert np.array_equal(target.joint_pos, expected)
    assert np.array_equal(home.joint_pos, expected)
    assert np.array_equal(target.joint_pos, home.joint_pos)
    assert np.array_equal(target.joint_vel, np.zeros(29, dtype=np.float32))
    assert np.array_equal(home.joint_vel, np.zeros(29, dtype=np.float32))


def test_v9_outward_hold_reference_uses_smoothstep_but_phase_stays_linear(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._start_outward()
    scheduler.transition_remaining_s = 0.0
    scheduler.reference_transition_remaining_s = 0.0
    scheduler._refresh_move_reference()
    scheduler._start_outward_hold("stable")
    previous_q = scheduler._previous_q.copy()
    previous_qd = scheduler._previous_qd.copy()
    current_q = scheduler._current_q.copy()

    scheduler.transition_remaining_s = 0.05
    scheduler.reference_transition_remaining_s = 0.15
    output = scheduler.output()

    assert np.allclose(output.phase, [0, 0, 0.5, 0.5, 0, 0], atol=1.0e-6)
    assert np.allclose(output.joint_pos, 0.5 * (previous_q + current_q), atol=1.0e-6)
    assert np.allclose(
        output.joint_vel,
        0.5 * previous_qd + 5.0 * (current_q - previous_q),
        atol=1.0e-6,
    )


def test_return_waits_for_stability_after_reference_end(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(base, base)
    base[1] = scheduler.outward_target_y
    scheduler._pelvis_position = base.copy()
    scheduler._start_return()

    for _ in range(100):
        output = _update_scheduler(scheduler, -0.5, target, velocity, base, base)
    assert scheduler.state == DoublesPhase.RETURN
    assert output.reference_step == scheduler.move_motions.lengths[output.move_motion_index] - 1
    assert np.array_equal(output.joint_vel, np.zeros(29, dtype=np.float32))

    base[1] = scheduler.home_y
    for _ in range(10):
        _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        if scheduler.state == DoublesPhase.HOME_HOLD:
            break
    assert scheduler.state == DoublesPhase.HOME_HOLD


def test_v10_return_reference_end_forces_home_hold(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        v10_relative_x=True,
        reference_end_forces_hold=True,
        latch_hold_x_on_entry=True,
    )
    base = np.array([0.0, 0.35, 1.0], dtype=np.float32)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    scheduler.reset(base, base)
    scheduler._start_return()
    for _ in range(100):
        output = _update_scheduler(scheduler, -0.5, target, velocity, base, base)
        if output.state == DoublesPhase.HOME_HOLD:
            break
    assert output.state == DoublesPhase.HOME_HOLD


def test_scheduler_masks_hit_inputs_after_post_delay(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks)
    target = np.array([0.45, 0.0, 1.0], dtype=np.float32)
    velocity = np.array([3.0, 0.0, 0.5], dtype=np.float32)
    _advance_to_home(scheduler, target, velocity)
    output = scheduler.output()
    assert output.strike_time == -0.5
    assert np.array_equal(output.racket_target, np.zeros(3, dtype=np.float32))
    assert np.array_equal(output.target_velocity, np.zeros(3, dtype=np.float32))
    assert np.isclose(output.target_base[1], 0.0, atol=1.0e-6)


def test_frozen_student_onnx_export_contract(tmp_path):
    checkpoint = tmp_path / "model_latest.pt"
    torch.save(
        {
            "student_state_dict": StudentPolicy().state_dict(),
            "student_obs_dim": 1666,
            "iteration": 23000,
            "phase_order": ["HIT", "POST_DELAY", "OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD"],
            "hit_teacher_sha256": EXPECTED_HIT_TEACHER_SHA256,
            "move_teacher_sha256": EXPECTED_MOVE_TEACHER_SHA256,
        },
        checkpoint,
    )
    metadata = export_student(
        checkpoint,
        tmp_path / "bundle",
        expected_checkpoint_sha256=sha256_file(checkpoint),
    )
    assert metadata["input_dim"] == 1666
    assert metadata["output_dim"] == 29
    assert metadata["history_length"] == 10
    assert metadata["phase_order"] == ["HIT", "POST_DELAY", "OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD"]
    assert metadata["pytorch_onnx_max_action_error"] <= 1.0e-5
    assert (tmp_path / "bundle" / OUTPUT_BASENAME).is_file()
    assert (tmp_path / "bundle" / f"{OUTPUT_BASENAME}.json").is_file()
    assert (tmp_path / "bundle" / "student_iteration_023000.pt").is_file()


def test_planner_monitor_payload_extracts_tracker_freshness():
    payload = {
        "planner": {
            "state": "waiting",
            "time_to_strike": -0.52,
            "cross_detected": False,
            "planning_armed": True,
        },
        "mocap": {
            "tracker_sensor": 3,
            "ball_sensor": 30301,
            "callback_stats": {"3": {"count": 123}, "30301": {"count": 456}},
            "tracker_status": "tracker sensor=3 ok age_ms=87.5 pos_world=[-0.6  0.5  1.2]",
        },
    }
    parsed = parse_planner_monitor_payload(payload)
    assert parsed["monitor_tracker_sensor"] == 3
    assert parsed["monitor_tracker_callback_count"] == 123
    assert parsed["monitor_ball_callback_count"] == 456
    assert np.isclose(parsed["monitor_tracker_age_ms"], 87.5)
    assert np.allclose(parsed["monitor_tracker_pos_world"], [-0.6, 0.5, 1.2])
    assert parsed["monitor_planner_state"] == b"waiting"
    assert parsed["monitor_planning_armed"] == 1


def test_latency_command_trace_decodes_network_order():
    estimator = StateEstimator.__new__(StateEstimator)
    payload = _LATENCY_COMMAND_TRACE.pack(17, 101, 202, 303)
    estimator._latency_trace_cb("latency_command_trace", payload)
    assert estimator.trace_command_seq == 17
    assert estimator.command_lcm_receive_monotonic_raw_ns == 101
    assert estimator.command_control_monotonic_raw_ns == 202
    assert estimator.command_dds_write_monotonic_raw_ns == 303

    estimator._latency_trace_cb("latency_command_trace", b"invalid")
    assert estimator.trace_command_seq == 17


def test_async_recorder_writes_fixed_schema_without_ros(tmp_path):
    recorder = AsyncDeployRecorder(
        tmp_path,
        duration_s=0.2,
        metadata={"test": True},
        subscribe_planner_monitor=False,
    )
    for sequence in range(3):
        assert recorder.enqueue(
            {
                "wall_time_ns": time.time_ns(),
                "monotonic_time_ns": time.monotonic_ns(),
                "command_seq": 100 + sequence,
                "command_publish_monotonic_raw_ns": 1000 + sequence,
                "trace_command_seq": 99 + sequence,
                "command_lcm_receive_monotonic_raw_ns": 2000 + sequence,
                "command_control_monotonic_raw_ns": 3000 + sequence,
                "command_dds_write_monotonic_raw_ns": 4000 + sequence,
                "body_source_monotonic_raw_ns": 5000 + sequence,
                "body_receive_monotonic_raw_ns": 6000 + sequence,
                "imu_source_monotonic_raw_ns": 7000 + sequence,
                "imu_receive_monotonic_raw_ns": 8000 + sequence,
                "phase_id": sequence,
                "phase": np.eye(6, dtype=np.float32)[sequence],
                "outward_hold_elapsed_s": sequence * 0.02,
                "outward_hold_duration_s": 1.0,
                "outward_hold_entry_reason": b"stable",
                "torso_pos": np.full(3, sequence + 0.4, dtype=np.float32),
                "pelvis_pos": np.full(3, sequence + 0.5, dtype=np.float32),
                "torso_minus_pelvis": np.full(3, -0.1, dtype=np.float32),
                "waist_joint_pos": np.full(3, sequence + 0.6, dtype=np.float32),
                "obs": np.full(172, sequence, dtype=np.float32),
                "obs_history": np.full(1666, sequence, dtype=np.float32),
                "action_lab": np.full(29, sequence + 0.1, dtype=np.float32),
                "action_gym": np.full(29, sequence + 0.2, dtype=np.float32),
                "q_des": np.full(29, sequence + 0.3, dtype=np.float32),
                "racket_hand": 0,
                "right_side_canonicalization": 1,
                "arm_transform_applied": 0,
                "table_right_move_mode": 2,
                "observation_transform_applied": 0,
                "action_transform_applied": 0,
                "physical_target_base": np.array([0.2, -0.7], dtype=np.float32),
                "policy_target_base": np.array([0.2, 0.7], dtype=np.float32),
                "move_motion_source_label": 0,
                "move_semantic_phase": 2,
                "episode_start_x_override": 0.1,
                "episode_start_x": 0.1,
                "target_base_x": 0.01 * sequence,
                "external_hit_command_mode": 1,
                "planner_command_sequence": 200 + sequence,
                "planner_commit_token": b"relay-stream",
                "planner_command_age_ms": 4.0 + sequence,
                "planner_raw_tts": 0.4 - sequence * 0.02,
                "planner_streamed_tts": 0.39 - sequence * 0.02,
                "planner_stream_update": 1,
                "planner_prediction_source_timestamp_s": 100.0 + sequence,
                "planner_prediction_age_ms": 8.0 + sequence,
                "raw_ball_valid": 1,
                "raw_ball_reason": b"ok",
                "raw_ball_source_time_s": 200.0 + sequence,
                "raw_ball_frame": 300 + sequence,
                "raw_ball_rigid_body_id": 30301,
                "raw_ball_position": np.full(3, sequence + 0.7, dtype=np.float32),
                "raw_ball_velocity": np.full(3, sequence + 0.8, dtype=np.float32),
                "raw_ball_age_ms": 3.0 + sequence,
                "raw_ball_receive_monotonic_ns": 9000 + sequence,
                "raw_ball_sample_count": 6 + sequence,
            }
        )
    session_dir = recorder.session_dir
    recorder.close(timeout_s=5.0)

    metadata = json.loads((session_dir / "metadata.json").read_text(encoding="utf-8"))
    frames = np.load(session_dir / "frames.npy", mmap_mode="r")
    assert frames.dtype == RECORD_DTYPE
    assert metadata["status"] == "complete"
    assert metadata["valid_frames"] == 3
    assert np.array_equal(frames[:3]["sequence"], [0, 1, 2])
    assert np.all(frames[2]["obs_history"] == 2.0)
    assert np.allclose(frames[1]["q_des"], 1.3)
    assert np.allclose(frames[2]["pelvis_pos"], 2.5)
    assert np.allclose(frames[1]["torso_minus_pelvis"], -0.1)
    assert np.allclose(frames[0]["waist_joint_pos"], 0.6)
    assert frames[2]["command_seq"] == 102
    assert frames[2]["trace_command_seq"] == 101
    assert frames[2]["command_dds_write_monotonic_raw_ns"] == 4002
    assert frames[2]["external_hit_command_mode"] == 1
    assert frames[2]["planner_command_sequence"] == 202
    assert frames[2]["planner_commit_token"] == b"relay-stream"
    assert np.isclose(frames[2]["planner_command_age_ms"], 6.0)
    assert np.isclose(frames[2]["planner_prediction_source_timestamp_s"], 102.0)
    assert np.isclose(frames[2]["planner_prediction_age_ms"], 10.0)
    assert frames[2]["raw_ball_valid"] == 1
    assert frames[2]["raw_ball_reason"] == b"ok"
    assert np.isclose(frames[2]["raw_ball_source_time_s"], 202.0)
    assert frames[2]["raw_ball_frame"] == 302
    assert frames[2]["raw_ball_rigid_body_id"] == 30301
    assert np.allclose(frames[2]["raw_ball_position"], 2.7)
    assert np.allclose(frames[2]["raw_ball_velocity"], 2.8)
    assert np.isclose(frames[2]["raw_ball_age_ms"], 5.0)
    assert frames[2]["raw_ball_sample_count"] == 8
    assert frames[2]["planner_stream_update"] == 1
    assert np.isclose(frames[2]["episode_start_x_override"], 0.1)
    assert np.isclose(frames[2]["episode_start_x"], 0.1)
    assert np.isclose(frames[2]["target_base_x"], 0.02)
    assert frames[2]["racket_hand"] == 0
    assert frames[2]["right_side_canonicalization"] == 1
    assert frames[2]["arm_transform_applied"] == 0
    assert frames[2]["table_right_move_mode"] == 2
    assert frames[2]["observation_transform_applied"] == 0
    assert frames[2]["action_transform_applied"] == 0
    assert np.allclose(frames[2]["physical_target_base"], [0.2, -0.7])
    assert np.allclose(frames[2]["policy_target_base"], [0.2, 0.7])
    assert frames[2]["move_motion_source_label"] == 0
    assert frames[2]["move_semantic_phase"] == 2


def test_triggered_recorder_wraps_and_saves_hit_window(tmp_path):
    recorder = AsyncDeployRecorder(
        tmp_path,
        duration_s=0.2,
        mode="triggered",
        pre_trigger_s=0.04,
        post_trigger_s=0.04,
        subscribe_planner_monitor=False,
    )
    phases = [5, 5, 5, 5, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    for phase in phases:
        assert recorder.enqueue(
            {
                "wall_time_ns": time.time_ns(),
                "monotonic_time_ns": time.monotonic_ns(),
                "phase_id": phase,
                "phase": np.eye(6, dtype=np.float32)[phase],
            }
        )
    session_dir = recorder.session_dir
    recorder.close(timeout_s=5.0)

    metadata = json.loads((session_dir / "metadata.json").read_text(encoding="utf-8"))
    frames = np.load(session_dir / "frames.npy", mmap_mode="r")
    clip = np.load(session_dir / "hit_00000004.npy", mmap_mode="r")
    events = (session_dir / "events.jsonl").read_text(encoding="utf-8")
    assert metadata["record_mode"] == "triggered"
    assert metadata["total_frames"] == len(phases)
    assert metadata["valid_frames"] == 10
    assert set(frames["sequence"].tolist()) == set(range(5, 15))
    assert np.array_equal(clip["sequence"], np.arange(2, 7))
    assert np.array_equal(clip["phase_id"], [5, 5, 0, 0, 1])
    assert "hit_clip_saved" in events
    assert "capacity_reached" not in events


def test_async_recorder_queue_full_never_waits():
    class _AliveProcess:
        @staticmethod
        def is_alive():
            return True

    class _UnsetEvent:
        @staticmethod
        def is_set():
            return False

    class _FullQueue:
        @staticmethod
        def qsize():
            return 256

        @staticmethod
        def put_nowait(snapshot):
            raise queue.Full

    recorder = AsyncDeployRecorder.__new__(AsyncDeployRecorder)
    recorder.failed = False
    recorder._capacity_event = _UnsetEvent()
    recorder._process = _AliveProcess()
    recorder._queue = _FullQueue()
    recorder.sequence = 0
    recorder.dropped_frames = 0
    recorder.last_enqueue_s = 0.0

    started = time.perf_counter()
    assert not recorder.enqueue({})
    elapsed = time.perf_counter() - started
    assert elapsed < 0.01
    assert recorder.dropped_frames == 1
    assert recorder.sequence == 0


def test_runtime_motion_config_applies_at_next_relevant_segments(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        v10_relative_x=True,
        reference_end_forces_hold=True,
        latch_hold_x_on_entry=True,
    )
    pelvis = np.array([0.18, 0.0, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler.stage_runtime_motion_config(
        command_id="cmd-1",
        goal_x=0.20,
        outward_y=0.60,
        home_y=-0.10,
        outward_hold_s=4.0,
    )
    status = scheduler.runtime_motion_config_status()
    assert status["pending"]["awaiting_outward"]
    assert status["pending"]["awaiting_home"]
    assert np.isclose(scheduler.outward_target_y, 0.9125)
    assert np.isclose(scheduler.home_y, 0.0)

    scheduler._start_outward()
    assert np.isclose(scheduler.goal_x, 0.20)
    assert np.isclose(scheduler.outward_target_y, 0.60)
    assert np.isclose(scheduler.outward_hold_s, 4.0)
    assert np.isclose(scheduler.home_y, 0.0)
    status = scheduler.runtime_motion_config_status()
    assert not status["pending"]["awaiting_outward"]
    assert status["pending"]["awaiting_home"]
    assert status["outward_applied_command_id"] == "cmd-1"

    scheduler._pelvis_position[:] = [0.20, 0.60, 0.75]
    scheduler._start_outward_hold("stable")
    assert np.isclose(scheduler.outward_hold_duration_s, 4.0)
    scheduler._start_return()
    assert np.isclose(scheduler.goal_x, 0.20)
    assert np.isclose(scheduler._desired_x(), 0.20)
    assert np.isclose(scheduler.home_y, -0.10)
    assert scheduler.runtime_motion_config_status()["pending"] is None
    assert scheduler.runtime_motion_config_status()["home_applied_command_id"] == "cmd-1"


def test_runtime_motion_config_does_not_change_current_hold(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        outward_hold_s=3.0,
        v10_relative_x=True,
        reference_end_forces_hold=True,
        latch_hold_x_on_entry=True,
    )
    pelvis = np.array([0.18, 0.0, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler._start_outward()
    scheduler._start_outward_hold("stable")
    assert np.isclose(scheduler.outward_hold_duration_s, 3.0)
    scheduler.stage_runtime_motion_config(
        command_id="cmd-2",
        goal_x=0.19,
        outward_y=0.55,
        home_y=-0.05,
        outward_hold_s=8.0,
    )
    assert np.isclose(scheduler.outward_hold_duration_s, 3.0)
    assert np.isclose(scheduler.outward_hold_s, 3.0)


def test_external_return_applies_pending_home_config(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        v10_relative_x=True,
        external_control=True,
    )
    pelvis = np.array([0.18, 0.60, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler._outward_x_anchor_valid = True
    scheduler.state = DoublesPhase.OUTWARD_HOLD
    scheduler.stage_runtime_motion_config(
        command_id="external-return",
        goal_x=0.22,
        outward_y=0.90,
        home_y=-0.10,
        outward_hold_s=5.0,
    )
    scheduler.set_external_base_target(
        np.array([0.18, -0.10], dtype=np.float32),
        return_target_y=-0.10,
        semantic_phase="RETURN",
    )
    status = scheduler.runtime_motion_config_status()
    assert status["home_applied_command_id"] == "external-return"
    assert status["pending"]["awaiting_outward"]
    assert not status["pending"]["awaiting_home"]


def test_runtime_motion_config_accepts_x_outside_legacy_session_band(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks, v10_relative_x=True)
    pelvis = np.array([0.18, 0.0, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler.stage_runtime_motion_config(
        command_id="unbounded-x",
        goal_x=0.50,
        outward_y=0.60,
        home_y=0.0,
        outward_hold_s=3.0,
    )
    status = scheduler.runtime_motion_config_status()
    assert np.isclose(status["pending"]["goal_x"], 0.50)
    assert status["x_safe_min"] is None
    assert status["x_safe_max"] is None
    scheduler._start_outward()
    assert np.isclose(scheduler.goal_x, 0.50)


def test_v10_target_base_x_runtime_clip_is_point_zero_four(motion_banks):
    scheduler = DoublesReferenceScheduler(*motion_banks, v10_relative_x=True)
    pelvis = np.array([0.18, 0.0, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler.goal_x = 0.22
    scheduler.state = DoublesPhase.RETURN
    scheduler._pelvis_position[0] = 0.0
    assert np.isclose(scheduler.output().target_base[0], 0.04)
    assert np.isclose(
        scheduler.runtime_motion_config_status()["active"]["target_base_x_clip"],
        0.04,
    )


def test_v11_target_base_x_clip_can_be_explicitly_disabled(motion_banks):
    scheduler = DoublesReferenceScheduler(
        *motion_banks,
        v10_relative_x=True,
        target_base_x_clip=None,
    )
    pelvis = np.array([0.18, 0.0, 0.75], dtype=np.float32)
    scheduler.reset(pelvis, pelvis)
    scheduler.goal_x = 0.50
    scheduler.state = DoublesPhase.RETURN
    scheduler._pelvis_position[0] = 0.0

    assert np.isclose(scheduler.output().target_base[0], 0.50)
    assert scheduler.runtime_motion_config_status()["active"]["target_base_x_clip"] is None
