from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

import run_isaac_doubles_relay as runner


def _write_controller(
    root: Path,
    *,
    methods: tuple[str, ...] = (
        "set_external_hit",
        "set_external_base_target",
        "set_external_outward_target",
    ),
    cfg_fields: tuple[str, ...] = ("contact_sensor_name",),
) -> None:
    play_script = root / "scripts/rsl_rl/play_distill.py"
    command_source = root / runner.CONTROLLER_COMMAND_SOURCE
    play_script.parent.mkdir(parents=True)
    command_source.parent.mkdir(parents=True)
    play_script.write_text("", encoding="utf-8")
    signatures = {
        "set_external_hit": (
            "self, env_ids, racket_target, target_velocity, time_to_strike_s, "
            "ball_velocity=None"
        ),
        "set_external_base_target": (
            "self, env_ids, target_xy, return_target_y=None, safety_override=False"
        ),
        "set_external_outward_target": "self, env_ids, target_y",
    }
    method_source = "\n".join(
        f"    def {method}({signatures.get(method, 'self')}):\n        pass"
        for method in methods
    ) or "    pass"
    field_source = "\n".join(
        f"    {field}: str = ''" for field in cfg_fields
    ) or "    pass"
    command_source.write_text(
        f"class DoublesDistillCommand:\n{method_source}\n\n"
        f"class DoublesDistillCommandCfg:\n{field_source}\n",
        encoding="utf-8",
    )


def test_controller_abi_accepts_required_hooks_and_config(tmp_path: Path) -> None:
    _write_controller(tmp_path)

    assert runner._controller_abi_errors(tmp_path) == ()
    assert runner._controller_root(tmp_path) == tmp_path.resolve()


def test_controller_abi_reports_missing_runtime_contract(tmp_path: Path) -> None:
    _write_controller(
        tmp_path,
        methods=("set_external_hit",),
        cfg_fields=(),
    )

    assert runner._controller_abi_errors(tmp_path) == (
        "DoublesDistillCommand missing set_external_base_target()",
        "DoublesDistillCommand missing set_external_outward_target()",
        "DoublesDistillCommandCfg missing contact_sensor_name",
    )


def test_controller_root_fails_closed_without_fallback(tmp_path: Path) -> None:
    _write_controller(tmp_path, methods=("set_external_hit",), cfg_fields=())

    with pytest.raises(FileNotFoundError, match="Requested controller"):
        runner._controller_root(tmp_path)


def test_v9_contract_selects_timed_handoff_task() -> None:
    assert runner._task_for_checkpoint_compatibility(
        {"controller_contract": {"contract_version": "v9-timed-handoff-v1"}}
    ) == runner.V9_TASK
    assert runner._task_for_checkpoint_compatibility({}) == runner.TASK


def test_v9_runtime_uses_deployment_scheduler_values() -> None:
    class CommandCfg:
        robust_handoff_readiness = True
        post_delay_range_s = (0.20, 0.50)
        outward_hold_range_s = (0.20, 0.50)
        outward_medium_hold_range_s = (0.50, 1.00)
        outward_long_hold_range_s = (1.00, 5.00)
        target_hold_reference_transition_s = 0.10

    cfg = CommandCfg()
    runner._configure_task_runtime(cfg, runner.V9_TASK)

    assert not cfg.robust_handoff_readiness
    assert cfg.post_delay_range_s == (0.20, 0.20)
    assert cfg.outward_hold_range_s == (3.0, 3.0)
    assert cfg.outward_medium_hold_range_s == (3.0, 3.0)
    assert cfg.outward_long_hold_range_s == (3.0, 3.0)
    assert cfg.target_hold_reference_transition_s == 0.30


def test_unitree_asset_link_is_shared_with_controller_checkout(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = tmp_path / "v9_controller"
    fallback = (
        tmp_path
        / "pingpang_controller"
        / "source"
        / "whole_body_tracking"
        / "whole_body_tracking"
        / "assets"
        / "unitree_description"
    )
    fallback.mkdir(parents=True)
    monkeypatch.setattr(runner, "WORKSPACE_ROOT", tmp_path)

    linked = runner._ensure_unitree_asset(controller)

    assert linked.is_symlink()
    assert linked.resolve() == fallback.resolve()
    assert runner._ensure_unitree_asset(controller) == linked


def test_controller_abi_rejects_incomplete_hook_signature(tmp_path: Path) -> None:
    _write_controller(tmp_path)
    command_source = tmp_path / runner.CONTROLLER_COMMAND_SOURCE
    command_source.write_text(
        command_source.read_text(encoding="utf-8").replace(
            "self, env_ids, target_xy, return_target_y=None, safety_override=False",
            "self, env_ids, target_xy",
        ),
        encoding="utf-8",
    )

    assert runner._controller_abi_errors(tmp_path) == (
        "DoublesDistillCommand.set_external_base_target() missing keyword parameters: "
        "return_target_y, safety_override",
    )


def test_run_video_selection_ignores_preexisting_artifacts(tmp_path: Path) -> None:
    (tmp_path / "mens_doubles_relay.mp4").write_bytes(b"stale-final")
    (tmp_path / "another-run-episode-0.mp4").write_bytes(b"stale-run")
    current = tmp_path / "unique-run-episode-0.mp4"
    current.write_bytes(b"current-run")

    assert runner._select_run_video(tmp_path, "unique-run") == current


def test_run_video_selection_fails_without_current_run_video(tmp_path: Path) -> None:
    (tmp_path / "mens_doubles_relay.mp4").write_bytes(b"stale-final")

    with pytest.raises(RuntimeError, match="produced no non-empty MP4"):
        runner._select_run_video(tmp_path, "unique-run")


def test_summary_paths_are_portable_from_repository_root(tmp_path: Path) -> None:
    rendered = runner._portable_summary_path(tmp_path / "artifact.json")

    assert not Path(rendered).is_absolute()
    assert (runner.REPO_ROOT / rendered).resolve() == (tmp_path / "artifact.json").resolve()


def test_rear_camera_is_behind_players_and_front_camera_is_across_table() -> None:
    rear_eye, rear_lookat = runner._camera_pose("rear")
    front_eye, front_lookat = runner._camera_pose("front")

    assert rear_eye[0] < rear_lookat[0]
    assert rear_eye[0] < 0.0 < rear_lookat[0]
    assert rear_eye[2] > rear_lookat[2]
    assert front_eye[0] > front_lookat[0]
    assert rear_eye[1] > 0.0
    assert front_eye[1] > 0.0


def test_unknown_camera_view_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown camera view"):
        runner._camera_pose("side")


def test_segment_marker_pose_aligns_unit_cylinder_to_y() -> None:
    midpoint, quaternion, length = runner._segment_marker_pose(
        [0.0, -0.3, 0.1],
        [0.0, 0.5, 0.1],
    )
    np.testing.assert_allclose(midpoint, [0.0, 0.1, 0.1], atol=1.0e-12)
    np.testing.assert_allclose(
        quaternion,
        [2**-0.5, -(2**-0.5), 0.0, 0.0],
        atol=1.0e-12,
    )
    assert length == pytest.approx(0.8)


def test_strike_zero_interpolation_uses_bracketing_controller_frames() -> None:
    before = runner.StrikeSnapshot(0.015, 0.05, 0.7, 0.08)
    after = runner.StrikeSnapshot(-0.005, 0.03, 0.5, 0.04)

    result = runner._interpolate_strike_zero(before, after)

    assert result is not None
    assert result.time_offset_s == 0.0
    assert result.position_error_m == pytest.approx(0.035)
    assert result.velocity_error_mps == pytest.approx(0.55)
    assert result.orientation_error_rad == pytest.approx(0.05)


def test_release_summary_requires_every_strike_metric_to_pass() -> None:
    summary = {
        "shots": [
            {"shot": 0, "strike_success": True},
            {"shot": 1, "strike_success": False},
        ]
    }

    assert runner._failed_strike_metric_shots(summary["shots"]) == (1,)


def test_termination_failure_message_keeps_state_for_report_only_artifacts() -> None:
    message = runner._termination_failure_message(
        366,
        {"robust_doubles_timeout_left": 1},
        {"left": {"phase": "RETURN", "home_target_y": -0.63}},
    )

    assert "relay step 366" in message
    assert "robust_doubles_timeout_left" in message
    assert "'phase': 'RETURN'" in message
    assert "'home_target_y': -0.63" in message


@pytest.mark.parametrize(
    ("fallbacks", "allow_tail", "expected"),
    [
        (("shot_not_armed",), True, ()),
        (("shot_not_armed",), False, ("shot_not_armed",)),
        (("left_feedback_stale",), True, ("left_feedback_stale",)),
        (
            ("shot_not_armed", "right_feedback_last_feedback"),
            True,
            ("right_feedback_last_feedback",),
        ),
        (
            ("ordered_goal_projection", "rl_goal_projection", "mpc_goal_projection"),
            False,
            (),
        ),
    ],
)
def test_unexpected_fallback_classification(
    fallbacks: tuple[str, ...],
    allow_tail: bool,
    expected: tuple[str, ...],
) -> None:
    assert runner._unexpected_fallbacks(
        fallbacks,
        allow_shot_not_armed_tail=allow_tail,
    ) == expected
