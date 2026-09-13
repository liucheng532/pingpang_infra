import numpy as np
import pytest

from yichao_v6r10_v11 import ROOT
from yichao_v6r10_v11.features import (
    ACTOR_NAMES,
    AUDIT_NAMES,
    FeatureTracker,
    build_vectors,
    interval_metadata,
    JointNormalizer,
)


def ball():
    return {
        "position": [1.2, 0.15, 1.0],
        "velocity": [-3.0, 0.2, -0.1],
        "predicted_strike_position": [0.45, 0.12, 1.05],
        "racket_velocity": [2.0, -0.3, 0.7],
        "time_to_strike_s": 0.5,
        "shot_id": "s1",
        "valid": True,
    }


def test_exact_schema_order_dimensions_and_non_symmetric_slots():
    assert len(ACTOR_NAMES) == 36 and len(AUDIT_NAMES) == 87
    assert ACTOR_NAMES[:6] == (
        "ball_history_0_relative_left_x", "ball_history_0_relative_left_y",
        "ball_history_0_relative_left_z", "ball_history_0_relative_right_x",
        "ball_history_0_relative_right_y", "ball_history_0_relative_right_z",
    )
    histories = np.zeros((2, 5, 3), np.float32)
    histories[0, :, :] = [0.1, 0.45, 0.75]
    histories[1, :, :] = [0.3, -0.55, 0.78]
    joints = np.vstack((np.linspace(-1, 1, 29), np.linspace(1, -1, 29))).astype(np.float32)
    result = build_vectors(
        histories, joints, ball(), [0.35, -0.35], "left",
        interval_metadata(1.65, first=True), current_bases=histories[:, -1],
    )
    assert result.actor.shape == (36,) and result.audit.shape == (87,)
    assert result.actor[30] == pytest.approx(0.1 / 3.0)
    assert result.actor[31] == pytest.approx(0.45)  # Y divisor is exactly 1.0.
    assert result.actor[32] == pytest.approx(0.3 / 3.0)
    assert result.actor[33] == pytest.approx(-0.55)
    assert result.actor[34] == pytest.approx(0.35 / 0.9)
    assert result.actor[35] == pytest.approx(-0.35 / 0.9)
    assert result.audit[-2] == -1.0
    assert result.audit[-1] == pytest.approx((1.65 - 0.8) / 0.8)
    assert not np.array_equal(result.actor[0:3], result.actor[3:6])
    handoff = ROOT / "tests/fixtures/handoff_schema.json"
    schema = __import__("json").loads(handoff.read_text())
    assert list(ACTOR_NAMES) == schema["observation_names"]
    assert list(AUDIT_NAMES) == schema["cbf_input_names"]
    assert schema["cbf_input_schema_hash"] == "1eb855b26c4c7661d2385b34dc91d53a830b6ebfd01087d7cca2e2a53c7e82b2"


def test_right_hitter_sign_and_interval_clipping():
    histories = np.zeros((2, 5, 3), np.float32)
    result = build_vectors(
        histories, np.zeros((2, 29)), ball(), [0.35, -0.35], "right",
        interval_metadata(3.2, first=False),
    )
    assert result.audit[-2] == 1.0
    assert result.record["interval"]["clipped_s"] == 1.8
    assert result.audit[-1] == pytest.approx((1.8 - 0.8) / 0.8)


def test_interval_updates_only_after_distinct_full_pair_ack_and_session_reset():
    tracker = FeatureTracker()
    first = tracker.interval_for("shot-1", 10.0)
    assert first["source"] == "first_shot_default" and first["clipped_s"] == 1.65
    # A proposal or one-side ACK has no API that can mutate the history.
    assert tracker.last_fully_acked_shot_id is None
    tracker.commit_fully_acked("shot-1", 10.0, [0.4, -0.3])
    with pytest.raises(ValueError, match="duplicate"):
        tracker.interval_for("shot-1", 10.1)
    second = tracker.interval_for("shot-2", 11.61)
    assert second["raw_s"] == pytest.approx(1.61)
    tracker.reset_session()
    assert tracker.previous_targets.tolist() == pytest.approx([0.35, -0.35])
    assert tracker.interval_for("shot-3", 20.0)["clipped_s"] == 1.65


def test_invalid_or_insufficient_pair_cannot_commit_previous_targets():
    tracker = FeatureTracker()
    with pytest.raises(ValueError, match="gap"):
        tracker.commit_fully_acked("bad", 1.0, [0.1, -0.1])
    with pytest.raises(ValueError):
        tracker.commit_fully_acked("nan", 1.0, [np.nan, -0.5])


def test_joint_normalizer_matches_frozen_v3_soft_limit_contract():
    profile = __import__("json").loads(
        (ROOT / "tests/fixtures/joint_normalization.json").read_text()
    )
    normalizer = JointNormalizer()
    assert list(normalizer.JOINT_NAMES) == profile["joint_names"]
    hard = np.asarray(profile["hard_joint_limits_rad"], dtype=np.float32)
    assert np.array_equal(normalizer.HARD_LIMITS, hard)
    assert np.allclose(normalizer.normalize(normalizer.midpoint), 0.0, atol=1e-7)
    assert np.allclose(normalizer.normalize(hard[:, 0]), -1.0, atol=1e-7)
    assert np.allclose(normalizer.normalize(hard[:, 1]), 1.0, atol=1e-7)
