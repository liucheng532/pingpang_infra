from __future__ import annotations

import hashlib

import pytest

import evaluate_isaac_doubles_ppo as evaluation


def test_parser_exposes_rollout_controls_without_launching_isaac(tmp_path):
    args = evaluation.build_parser().parse_args(
        [
            "--policy",
            str(tmp_path / "policy.pt"),
            "--output-dir",
            str(tmp_path / "video"),
            "--steps",
            "17",
            "--low-level-steps",
            "3",
            "--warmup-steps",
            "5",
            "--disable-video",
        ]
    )

    assert args.policy == tmp_path / "policy.pt"
    assert args.output_dir == tmp_path / "video"
    assert args.steps == 17
    assert args.low_level_steps == 3
    assert args.warmup_steps == 5
    assert args.disable_video


def test_sha256_streams_policy_file(tmp_path):
    policy = tmp_path / "policy.pt"
    payload = b"centralized-policy" * 100
    policy.write_bytes(payload)

    assert evaluation._sha256(policy) == hashlib.sha256(payload).hexdigest()


def test_video_length_must_cover_low_level_frames(monkeypatch):
    monkeypatch.setattr(
        evaluation,
        "_validate_checkpoint_controller_contract",
        lambda *_: (_ for _ in ()).throw(AssertionError("validation should not run")),
    )

    with pytest.raises(ValueError, match="cover every low-level"):
        evaluation.main(["--steps", "10", "--low-level-steps", "3", "--video-length", "29"])
