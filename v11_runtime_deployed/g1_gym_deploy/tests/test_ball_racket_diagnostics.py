from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_ROOT))

from utils.async_deploy_recorder import RECORD_DTYPE
from utils.ball_racket_diagnostics import analyze_hit_frames, canonical_racket_pose
from utils.joint_mapping import FROM_GYM_TO_LAB, FROM_LAB_TO_GYM, LAB_JOINT_NAMES
from utils.left_right_mirror import mirror_joint_values


def frame_fixture(count=5, mirrored=False):
    frames = np.zeros(count, dtype=RECORD_DTYPE)
    frames["phase_id"] = 0
    frames["mirror_left_hand"] = int(mirrored)
    frames["raw_ball_valid"] = 1
    frames["raw_ball_receive_monotonic_ns"] = np.arange(count) * 20_000_000 + 1_000_000_000
    frames["monotonic_time_ns"] = frames["raw_ball_receive_monotonic_ns"]
    frames["corrected_tts"] = np.linspace(0.04, -0.04, count)
    frames["raw_ball_velocity"][:] = [-3.0, 0.0, 0.0]
    orientation = np.eye(3, dtype=np.float32)[:, :2].reshape(-1)
    for frame in frames:
        frame["teacher_policy_obs_history"][127:130] = [0.0, 0.0, 0.8]
        frame["teacher_policy_obs_history"][184:190] = orientation
        frame["obs"][4:7] = [0.45, 0.0, 1.0]
    return frames


def test_contact_geometry_accepts_ball_at_reconstructed_racket_center():
    frames = frame_fixture()
    center, _ = canonical_racket_pose(frames[2])
    frames["raw_ball_position"][:] = center
    result = analyze_hit_frames(frames)
    assert result["classification"] == "geometric_contact"
    assert result["geometric_contact"]
    assert result["center_distance_m"] <= 1.0e-6
    assert result["ellipse_value"] <= 1.0


def test_mirrored_joint_state_reconstructs_same_canonical_racket_pose():
    canonical = frame_fixture(count=1, mirrored=False)
    canonical_q_lab = np.linspace(-0.3, 0.4, 29, dtype=np.float32)
    canonical[0]["joint_pos"] = canonical_q_lab[list(FROM_LAB_TO_GYM)]

    mirrored = frame_fixture(count=1, mirrored=True)
    physical_q_lab = mirror_joint_values(canonical_q_lab, LAB_JOINT_NAMES)
    mirrored[0]["joint_pos"] = np.asarray(physical_q_lab)[list(FROM_LAB_TO_GYM)]

    canonical_position, canonical_rotation = canonical_racket_pose(canonical[0])
    mirrored_position, mirrored_rotation = canonical_racket_pose(mirrored[0])
    assert np.allclose(mirrored_position, canonical_position, atol=1.0e-6)
    assert np.allclose(mirrored_rotation, canonical_rotation, atol=1.0e-6)


def test_missing_raw_ball_is_reported_without_fk_guess():
    frames = frame_fixture()
    frames["raw_ball_valid"] = 0
    result = analyze_hit_frames(frames)
    assert result == {"classification": "raw_ball_missing", "valid_fraction": 0.0}


def test_swept_contact_classifies_ball_outside_face_as_lateral_miss():
    frames = frame_fixture()
    center, rotation = canonical_racket_pose(frames[2])
    offsets = np.linspace(0.05, -0.05, len(frames))
    for index, normal_offset in enumerate(offsets):
        frames[index]["raw_ball_position"] = center + rotation @ np.array(
            [0.10, normal_offset, 0.0]
        )
    result = analyze_hit_frames(frames)
    assert result["classification"] == "lateral_miss"
    assert not result["inside_face"]
    assert abs(result["plane_local_delta_m"][0]) > 0.085
