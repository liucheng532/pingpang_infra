from __future__ import annotations

import os
from pathlib import Path
import subprocess
import ast


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "start_v9_canonical_left_test.sh"
NORMAL_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "start_v9_policy.sh"
NATIVE_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "start_66_native_no_mirror.sh"
V10_LEFTMIRROR_SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "start_v10_i25000_hitteacher_leftmirror_66.sh"
)
V10_198_SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "start_v10_i25000_hitteacher_198.sh"
)
V11_LEFTMIRROR_SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "start_v11_resume42500_hitteacher_leftmirror_66.sh"
)
V11_198_SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "start_v11_resume42500_hitteacher_198.sh"
)


def _dry_run(mode: str):
    environment = os.environ.copy()
    environment["V9_CANONICAL_TEST_DRY_RUN"] = "1"
    return subprocess.run(
        ["bash", str(SCRIPT), mode],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def _normal_dry_run(robot_id: str, mode: str):
    environment = os.environ.copy()
    environment["DUAL_RIGHT_HAND_DRY_RUN"] = "1"
    return subprocess.run(
        ["bash", str(NORMAL_SCRIPT), robot_id, mode],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def _native_dry_run(mode: str):
    environment = os.environ.copy()
    environment["NATIVE_NO_MIRROR_DRY_RUN"] = "1"
    return subprocess.run(
        ["bash", str(NATIVE_SCRIPT), mode],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def _v10_leftmirror_dry_run(
    mode: str,
    profile: str = "normal",
    residual_mode: str = "shadow",
    residual_scale: str = "1.0",
):
    environment = os.environ.copy()
    environment["V10_I25000_LEFTMIRROR_DRY_RUN"] = "1"
    return subprocess.run(
        [
            "bash",
            str(V10_LEFTMIRROR_SCRIPT),
            mode,
            profile,
            residual_mode,
            residual_scale,
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def _v10_198_dry_run(
    mode: str,
    profile: str = "normal",
    residual_mode: str = "shadow",
    residual_scale: str = "1.0",
):
    environment = os.environ.copy()
    environment["V10_I25000_198_DRY_RUN"] = "1"
    return subprocess.run(
        [
            "bash",
            str(V10_198_SCRIPT),
            mode,
            profile,
            residual_mode,
            residual_scale,
        ],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def _v11_dry_run(script: Path, mode: str = "shadow"):
    environment = os.environ.copy()
    environment[
        "V11_RESUME42500_66_DRY_RUN"
        if script == V11_LEFTMIRROR_SCRIPT
        else "V11_RESUME42500_198_DRY_RUN"
    ] = "1"
    return subprocess.run(
        ["bash", str(script), mode, "normal", "shadow", "1.0"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def test_canonical_66_launcher_uses_positive_side_policy_without_mirror():
    result = _dry_run("shadow")
    assert result.returncode == 0, result.stderr
    output = result.stdout
    assert "ROS_MASTER_URI=http://192.168.123.165:11311" in output
    assert "ROS_IP=192.168.123.164" in output
    assert "239.255.76.66:7667" in output
    assert "/doubles/table_right/torso_pose_origin" in output
    assert "table_right_canonical" in output
    assert "--startup-home-current" in output
    assert "--record-mode triggered" in output
    assert "--shadow" in output
    for forbidden in ("--mirror-left-hand", "--external-planner", "--robot-id"):
        assert forbidden not in output


def test_canonical_66_active_launcher_omits_shadow():
    result = _dry_run("active")
    assert result.returncode == 0, result.stderr
    assert "--shadow" not in result.stdout


def test_canonical_66_launcher_rejects_unknown_mode():
    result = _dry_run("invalid")
    assert result.returncode == 2


def test_normal_robot_launcher_uses_wired_ros_master():
    source = NORMAL_SCRIPT.read_text(encoding="utf-8")
    assert "ROS_MASTER_URI=http://192.168.123.165:11311" in source
    assert "ROS_MASTER_URI=http://172.16.4.184:11311" not in source
    for flag in (
        "--policy \"${deploy_root}/policy/v10move_i21500/student_v10move18500_xrecovery_1666_model21500.onnx\"",
        "--racket-hand right",
        "--external-hit-command-mode stream",
        "--hit-policy-source teacher",
        "--hit-teacher-history distill-reset",
        "--hit-reference-lead-steps 1",
        "--hit-arm7-residual-mode off",
        "--hit-arm7-residual-scale 1.0",
        "--episode-start-x 0.2",
    ):
        assert flag in source
    assert "mirror_args=(--mirror-left-hand)" not in source


def test_dual_right_hand_launchers_share_v10_policy_without_arm_mirror():
    left = _normal_dry_run("table_left", "shadow")
    right = _normal_dry_run("table_right", "shadow")
    assert left.returncode == right.returncode == 0
    for result in (left, right):
        assert "student_v10move18500_xrecovery_1666_model21500.onnx" in result.stdout
        assert "--racket-hand right" in result.stdout
        assert "--episode-start-x 0.2" in result.stdout
        assert "--hit-policy-source teacher" in result.stdout
        assert "--mirror-left-hand" not in result.stdout
        assert "--shadow" in result.stdout
    assert "ROS_IP=192.168.124.164" in left.stdout
    assert "239.255.76.198:7667" in left.stdout
    assert "--bootstrap-outward-hold" in left.stdout
    assert "ROS_IP=192.168.123.164" in right.stdout
    assert "239.255.76.66:7667" in right.stdout
    assert "--startup-home-current" in right.stdout


def test_66_native_no_mirror_launcher_locks_v9_and_raw_move_mode():
    result = _native_dry_run("active")
    assert result.returncode == 0, result.stderr
    output = result.stdout
    assert "student_v9_m14500_timedhandoff_1666_model19000.onnx" in output
    assert "--robot-id table_right" in output
    assert "--racket-hand right" in output
    assert "--table-right-move-mode native-no-mirror" in output
    assert "--episode-start-x 0.2" in output
    assert "--hit-policy-source teacher" in output
    assert "--mirror-left-hand" not in output
    assert "--shadow" not in output


def test_v10_i25000_leftmirror_66_launcher_uses_doubles_planner():
    result = _v10_leftmirror_dry_run("active")
    assert result.returncode == 0, result.stderr
    output = result.stdout
    for required in (
        "student_v10_refend_clip10_1666_model25000.onnx",
        "--robot-id table_right",
        "--racket-hand left",
        "--mirror-left-hand",
        "--external-planner",
        "--torso-topic /doubles/table_right/torso_pose_origin",
        "--planner-home-y -0.20",
        "--external-hit-command-mode stream",
        "--startup-home-current",
        "--v10-relative-x",
        "--hit-policy-source teacher",
        "--hit-teacher-history distill-reset",
        "--hit-reference-lead-steps 1",
        "--hit-arm7-residual-mode shadow",
        "--hit-arm7-residual-scale 1.0",
        "--hit-arm7-diagnostics-jsonl",
        "239.255.76.66:7667",
    ):
        assert required in output
    for forbidden in (
        "--bootstrap-outward-hold",
        "--table-right-move-mode native-no-mirror",
        "--episode-start-x",
        "--shadow",
    ):
        assert forbidden not in output


def test_v10_i25000_198_launcher_is_canonical_and_planner_controlled():
    result = _v10_198_dry_run("shadow")
    assert result.returncode == 0, result.stderr
    output = result.stdout
    for required in (
        "student_v10_refend_clip10_1666_model25000.onnx",
        "--external-planner",
        "--robot-id table_left",
        "--racket-hand right",
        "--torso-topic /doubles/table_left/torso_pose_origin",
        "--planner-home-y 0.20",
        "--bootstrap-outward-hold",
        "--external-hit-command-mode stream",
        "--v10-relative-x",
        "--hit-policy-source teacher",
        "--hit-teacher-history distill-reset",
        "--hit-reference-lead-steps 1",
        "--hit-arm7-residual-mode shadow",
        "--hit-arm7-residual-scale 1.0",
        "--hit-arm7-diagnostics-jsonl",
        "ROS_IP=192.168.124.164",
        "239.255.76.198:7667",
        "--shadow",
    ):
        assert required in output
    for forbidden in (
        "--mirror-left-hand",
        "--table-right-move-mode",
        "--episode-start-x",
    ):
        assert forbidden not in output


def test_v10_i25000_dual_launchers_share_policy_and_isolate_networks():
    right = _v10_leftmirror_dry_run("shadow")
    left = _v10_198_dry_run("shadow")
    assert right.returncode == left.returncode == 0
    policy = "student_v10_refend_clip10_1666_model25000.onnx"
    assert policy in right.stdout
    assert policy in left.stdout
    assert "ROS_IP=192.168.123.164" in right.stdout
    assert "239.255.76.66:7667" in right.stdout
    assert "ROS_IP=192.168.124.164" in left.stdout
    assert "239.255.76.198:7667" in left.stdout


def test_v10_i25000_dual_launchers_reject_unknown_mode():
    assert _v10_leftmirror_dry_run("invalid").returncode == 2
    assert _v10_198_dry_run("invalid").returncode == 2


def test_v10_i25000_dual_launchers_forward_active_residual_scale():
    right = _v10_leftmirror_dry_run("active", residual_mode="active", residual_scale="0.25")
    left = _v10_198_dry_run("active", residual_mode="active", residual_scale="0.25")
    assert right.returncode == left.returncode == 0
    for result in (right, left):
        assert "--hit-arm7-residual-mode active" in result.stdout
        assert "--hit-arm7-residual-scale 0.25" in result.stdout


def test_v10_i25000_dual_launchers_allow_off_and_reject_invalid_residual_settings():
    assert "--hit-arm7-residual-mode off" in _v10_198_dry_run(
        "shadow", residual_mode="off"
    ).stdout
    assert _v10_leftmirror_dry_run("shadow", residual_mode="invalid").returncode == 2
    assert _v10_198_dry_run("shadow", residual_scale="1.1").returncode == 2


def test_v10_i25000_dual_launchers_offer_stationary_hit_home_profile():
    right = _v10_leftmirror_dry_run("active", "hit_home")
    left = _v10_198_dry_run("shadow", "hit_home")
    assert right.returncode == left.returncode == 0
    for result in (right, left):
        assert "--stationary-hit-test hit_home" in result.stdout
        assert "--hit-policy-source teacher" in result.stdout
        assert "--external-planner" in result.stdout
        assert "_hit_home" in result.stdout
    assert "--startup-home-current" in right.stdout
    assert "--startup-home-current" in left.stdout
    assert "--bootstrap-outward-hold" not in left.stdout
    assert "--mirror-left-hand" in right.stdout
    assert "--mirror-left-hand" not in left.stdout
    assert "--shadow" not in right.stdout
    assert "--shadow" in left.stdout


def test_v10_i25000_dual_launchers_keep_normal_profile_unchanged():
    right = _v10_leftmirror_dry_run("shadow")
    left = _v10_198_dry_run("shadow")
    for result in (right, left):
        assert result.returncode == 0
        assert "--stationary-hit-test" not in result.stdout
        assert "_hit_home" not in result.stdout
    assert "--startup-home-current" in right.stdout
    assert "--bootstrap-outward-hold" in left.stdout
    assert "--startup-home-current" not in left.stdout


def test_v10_i25000_dual_launchers_reject_unknown_profile():
    assert _v10_leftmirror_dry_run("shadow", "invalid").returncode == 2
    assert _v10_198_dry_run("shadow", "invalid").returncode == 2


def test_v11_dual_launchers_use_common_policy_teacher_hit_and_shadow_default():
    right = _v11_dry_run(V11_LEFTMIRROR_SCRIPT)
    left = _v11_dry_run(V11_198_SCRIPT)
    assert right.returncode == left.returncode == 0
    for result in (right, left):
        assert "student_v11_r2i299_commonhold_1666_resume42500.onnx" in result.stdout
        assert "--startup-relative-x" in result.stdout
        assert "--record-mode circular" in result.stdout
        assert "--hit-policy-source teacher" in result.stdout
        assert "--hit-teacher-history distill-reset" in result.stdout
        assert "--shadow" in result.stdout
    assert "--robot-id table_right" in right.stdout
    assert "--racket-hand left" in right.stdout
    assert "--mirror-left-hand" in right.stdout
    assert "239.255.76.66:7667" in right.stdout
    assert "--robot-id table_left" in left.stdout
    assert "--racket-hand right" in left.stdout
    assert "--mirror-left-hand" not in left.stdout
    assert "239.255.76.198:7667" in left.stdout


def test_v11_runtime_recorder_defaults_to_circular():
    deploy_script = Path(__file__).resolve().parents[1] / "scripts" / "deploy_policy.py"
    tree = ast.parse(deploy_script.read_text(encoding="utf-8"))
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "add_argument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "--record-mode"
    ]
    assert len(calls) == 1
    defaults = [keyword.value for keyword in calls[0].keywords if keyword.arg == "default"]
    assert len(defaults) == 1
    assert isinstance(defaults[0], ast.Constant) and defaults[0].value == "circular"


def test_v10_i25000_dual_launchers_only_reject_python_policy_processes():
    precise_probe = "pgrep -f '[p]ython3 .*scripts/deploy_policy.py'"
    for script in (V10_LEFTMIRROR_SCRIPT, V10_198_SCRIPT):
        source = script.read_text(encoding="utf-8")
        assert precise_probe in source
        assert "pgrep -f '[d]eploy_policy.py'" not in source


def test_episode_start_x_cli_defaults_to_none_and_deploy_remains_v9():
    deploy_script = Path(__file__).resolve().parents[1] / "scripts" / "deploy_policy.py"
    source = deploy_script.read_text(encoding="utf-8")
    tree = ast.parse(source)
    episode_calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "add_argument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "--episode-start-x"
    ]
    assert len(episode_calls) == 1
    defaults = [
        keyword.value
        for keyword in episode_calls[0].keywords
        if keyword.arg == "default"
    ]
    assert len(defaults) == 1
    assert isinstance(defaults[0], ast.Constant) and defaults[0].value is None
    assert "policy/v9_model19000/" in source
    assert "V10_CHECKPOINT_SHA256" in source
