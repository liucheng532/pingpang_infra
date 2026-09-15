import json

import numpy as np
import pytest

from yichao_v7_v11 import ROOT
from yichao_v7_v11.features import (
    ACTOR_NAMES,
    SAFE_NAMES,
    FeatureTracker,
    JointNormalizer,
    build_vectors,
)


def ball(tts=0.5):
    return {
        "position": [1.2, 0.15, 1.0],
        "velocity": [-3.0, 0.2, -0.1],
        "predicted_strike_position": [0.45, 0.12, 1.05],
        "racket_velocity": [2.0, -0.3, 0.7],
        "time_to_strike_s": tts,
        "shot_id": "s1",
        "valid": True,
    }


def test_exact_v7_schema_order_dimensions_and_scaling():
    assert len(ACTOR_NAMES) == 29 and len(SAFE_NAMES) == 87
    schema = json.loads((ROOT / "models/handoff_schema.json").read_text())
    assert list(ACTOR_NAMES) == schema["observation_names"]
    assert list(SAFE_NAMES) == schema["cbf_input_names"]
    assert schema["observation_schema_hash"] == "6f635a3cc33ef827b820f40e27087868d8af60f081ab2f835c4a94a9a26a1cfc"
    assert schema["cbf_input_schema_hash"] == "29bcbcd34b8b41c9720381488d9b70615bce63a9e284dcdacba5786c4652cc2d"

    histories = np.zeros((2, 5, 3), np.float32)
    histories[0, :, :] = [0.3, 0.45, 0.75]
    histories[1, :, :] = [0.6, -0.55, 0.78]
    joints = np.vstack((np.linspace(-1, 1, 29), np.linspace(1, -1, 29))).astype(np.float32)
    result = build_vectors(histories, joints, ball(), "left", ball_age_s=0.02)

    assert result.actor.shape == (29,) and result.safe.shape == (87,)
    np.testing.assert_array_equal(result.actor[:2], [1.0, 0.0])
    assert result.actor[2] == pytest.approx(0.48)
    np.testing.assert_allclose(result.actor[3:6], [0.5, -0.075, 0.175])
    np.testing.assert_allclose(result.actor[6:9], [0.15, 0.08, 0.7])
    np.testing.assert_allclose(result.actor[9:13], [0.1, 0.45, 0.1, 0.45])
    np.testing.assert_allclose(result.actor[19:23], [0.2, -0.55, 0.2, -0.55])
    np.testing.assert_array_equal(result.safe[20:49], joints[0])
    np.testing.assert_array_equal(result.safe[49:78], joints[1])
    np.testing.assert_allclose(result.safe[78:85], [0.15, 0.08, 0.7, 0.5, -0.075, 0.175, 0.48])
    np.testing.assert_array_equal(result.safe[85:], [1.0, 0.0])


def test_right_hitter_and_tts_correction_clips_to_v7_range():
    histories = np.zeros((2, 5, 3), np.float32)
    high = build_vectors(histories, np.zeros((2, 29)), ball(3.2), "right")
    low = build_vectors(histories, np.zeros((2, 29)), ball(0.01), "right", ball_age_s=0.03)
    np.testing.assert_array_equal(high.actor[:2], [0.0, 1.0])
    np.testing.assert_array_equal(high.safe[-2:], [0.0, 1.0])
    assert high.actor[2] == pytest.approx(0.54)
    assert low.actor[2] == 0.0


def test_fully_acked_targets_update_only_after_valid_pair():
    tracker = FeatureTracker()
    with pytest.raises(ValueError, match="gap"):
        tracker.commit_fully_acked("bad", 1.0, [0.1, -0.1])
    tracker.commit_fully_acked("shot-1", 1.0, [0.4, -0.3])
    tracker.commit_fully_acked("shot-1", 1.1, [0.5, -0.4])
    assert tracker.previous_targets.tolist() == pytest.approx([0.4, -0.3])
    tracker.reset_session()
    assert tracker.previous_targets.tolist() == pytest.approx([0.35, -0.35])


def test_joint_normalizer_matches_frozen_soft_limit_contract():
    normalizer = JointNormalizer()
    assert len(normalizer.JOINT_NAMES) == 29
    assert normalizer.JOINT_NAMES[:3] == (
        "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint"
    )
    hard = normalizer.HARD_LIMITS
    assert hard.shape == (29, 2) and np.all(hard[:, 0] < hard[:, 1])
    assert np.allclose(normalizer.normalize(normalizer.midpoint), 0.0, atol=1e-7)
    assert np.allclose(normalizer.normalize(hard[:, 0]), -1.0, atol=1e-7)
    assert np.allclose(normalizer.normalize(hard[:, 1]), 1.0, atol=1e-7)
