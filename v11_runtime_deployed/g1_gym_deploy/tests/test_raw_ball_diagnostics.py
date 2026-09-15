from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_ROOT))

from utils.raw_ball_diagnostics import BALL_RIGID_BODY_ID, RawBallStateBuffer


def payload(source_time, frame, position, rigid_body_id=BALL_RIGID_BODY_ID):
    return [source_time, frame, rigid_body_id, *position]


def test_raw_ball_buffer_estimates_velocity_and_age():
    value = RawBallStateBuffer(max_age_s=0.06, velocity_window_s=0.06)
    velocity = np.array([-3.0, 0.4, -0.2])
    for frame in range(8):
        source_time = 100.0 + frame * 0.01
        position = np.array([0.8, -0.1, 1.0]) + velocity * frame * 0.01
        assert value.ingest_payload(
            payload(source_time, frame, position),
            receive_monotonic_ns=1_000_000_000 + frame * 10_000_000,
        )

    sample = value.sample(now_monotonic_ns=1_075_000_000)
    assert sample["valid"]
    assert sample["reason"] == "ok"
    assert sample["frame"] == 7
    assert sample["rigid_body_id"] == BALL_RIGID_BODY_ID
    assert np.allclose(sample["velocity"], velocity, atol=1.0e-4)
    assert np.isclose(sample["age_ms"], 5.0)
    assert sample["sample_count"] >= 6


def test_raw_ball_buffer_rejects_wrong_rigid_and_stale_data():
    value = RawBallStateBuffer(max_age_s=0.02)
    assert not value.ingest_payload(payload(1.0, 1, [0.5, 0.0, 1.0], rigid_body_id=10))
    assert value.sample()["reason"] == "wrong_rigid"

    assert value.ingest_payload(
        payload(2.0, 2, [0.5, 0.0, 1.0]), receive_monotonic_ns=1_000_000_000
    )
    assert value.ingest_payload(
        payload(2.01, 3, [0.47, 0.0, 1.0]), receive_monotonic_ns=1_010_000_000
    )
    sample = value.sample(now_monotonic_ns=1_040_000_000)
    assert not sample["valid"]
    assert sample["reason"] == "stale"


def test_raw_ball_buffer_rejects_duplicate_frame_without_destroying_latest():
    value = RawBallStateBuffer()
    assert value.ingest_payload(payload(1.0, 10, [0.5, 0.0, 1.0]), 1_000_000_000)
    assert not value.ingest_payload(payload(1.01, 10, [0.4, 0.0, 1.0]), 1_010_000_000)
    sample = value.sample(now_monotonic_ns=1_011_000_000)
    assert not sample["valid"]
    assert sample["reason"] == "out_of_order"
    assert np.allclose(sample["position"], [0.5, 0.0, 1.0])
