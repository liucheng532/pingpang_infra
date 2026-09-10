from __future__ import annotations

from pathlib import Path
import subprocess


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "start_v11_dual_robots.sh"
)


def run_launcher(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(SCRIPT), *args],
        check=False,
        capture_output=True,
        text=True,
    )


def test_default_dry_run_starts_both_robots_in_shadow() -> None:
    result = run_launcher("dry-run")

    assert result.returncode == 0, result.stderr
    assert "mode=shadow" in result.stdout
    assert "192.168.124.164" in result.stdout
    assert "start_v9_g1_control.sh table_left" in result.stdout
    assert "start_v11_resume42500_hitteacher_198.sh shadow" in result.stdout
    assert "192.168.123.164" in result.stdout
    assert "start_v9_g1_control.sh table_right" in result.stdout
    assert "start_v11_resume42500_hitteacher_leftmirror_66.sh shadow" in result.stdout
    for forbidden in (
        "run_predictor.py",
        "run_v9_real_fixed_relay.py",
        "ros_web_monitor.server",
    ):
        assert forbidden not in result.stdout


def test_active_mode_must_be_explicit() -> None:
    result = run_launcher("dry-run", "active")

    assert result.returncode == 0, result.stderr
    assert "mode=active" in result.stdout
    assert "start_v11_resume42500_hitteacher_198.sh active" in result.stdout
    assert "start_v11_resume42500_hitteacher_leftmirror_66.sh active" in result.stdout


def test_invalid_mode_is_rejected_without_remote_calls() -> None:
    result = run_launcher("dry-run", "invalid")

    assert result.returncode == 64
    assert "Mode must be shadow or active" in result.stderr


def test_help_states_that_only_robot_processes_are_started() -> None:
    result = run_launcher("--help")

    assert result.returncode == 0
    assert "starts only the two robots" in result.stdout


def test_v11_preflight_locks_runtime_and_all_policy_assets() -> None:
    script = SCRIPT.read_text(encoding="utf-8")
    for value in (
        "/home/unitree/haoran/doubles-v11-r2i299-resume42500-dual-runtime",
        "d7be604c882307f6951c4649607dd6a07e6dbbd31bd3d80a481920ff5f762eaa",
        "0603032faa7b0f5c2a81823a1d8db16f4689c399b5950a536cd811905acd7674",
        "04461633b8ad75a2b173ceacf34257fae437cbf935f6effe8b279464586bea12",
        "9565a8ed1ba22cfc0d759d0a8327dc7dd37989c242d4f4337dcc79c796ea3a8f",
    ):
        assert value in script


def test_launcher_waits_for_hardware_and_policy_readiness() -> None:
    script = SCRIPT.read_text(encoding="utf-8")

    assert "wait_for_window_output 198-g1 g1-198 'G1 type:'" in script
    assert "wait_for_window_output 66-g1 g1-66 'G1 type:'" in script
    assert "policy_ready_marker='About to calibrate'" in script
    assert "policy_ready_marker='[DEPLOY]'" in script
    assert "wait_for_window_output 198-policy policy-198" in script
    assert "wait_for_window_output 66-policy policy-66" in script
    assert "require_ros_topic /doubles/table_left/state" in script
    assert "require_ros_topic /doubles/table_right/state" in script
    assert (
        script.index("create_window policy-198")
        < script.index("wait_for_window_output 198-policy")
        < script.index("create_window policy-66")
    )


def test_failed_policy_panes_are_retained_and_logged_before_cleanup() -> None:
    script = SCRIPT.read_text(encoding="utf-8")

    assert "remain-on-exit on" in script
    assert "capture_startup_logs; stop_session" in script
    assert "Saved failed startup logs:" in script
