from __future__ import annotations

import numpy as np

from doubles_planner.real_ros_runtime import (
    BALL_SCHEMA,
    PLANNER_COMMAND_SCHEMA,
    ROBOT_ORDER,
    ROBOT_STATE_SCHEMA,
    V9RealCbfRuntime,
    V9RealFixedRelayRuntime,
    v9_real_planner_config,
)
from doubles_planner.runtime_motion_config import RUNTIME_MOTION_CONFIG_SCHEMA


def robot_state(robot: str, y: float, sequence: int = 1, *, x: float = 0.0):
    return {
        "schema_version": ROBOT_STATE_SCHEMA,
        "robot": robot,
        "sequence": sequence,
        "source_monotonic_ns": sequence * 20_000_000,
        "q": np.zeros(29).tolist(),
        "dq": np.zeros(29).tolist(),
        "imu_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        "gyro_xyz": [0.0, 0.0, 0.0],
        "base_position_xyz": [x, y, 0.75],
        "base_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "base_linear_velocity_xyz": [0.0, 0.0, 0.0],
        "base_angular_velocity_xyz": [0.0, 0.0, 0.0],
        "phase": "HOME_HOLD",
        "ready": True,
        "state_elapsed_s": 1.0,
        "stable_elapsed_s": 1.0,
        "target_base_y": y,
        "home_y": y,
        "time_to_strike_s": -0.5,
        "valid": True,
        "emergency_stop": False,
    }


def ball_prediction(sequence: int = 1):
    return {
        "schema_version": BALL_SCHEMA,
        "sequence": sequence,
        "position": [0.0, 0.1, 1.1],
        "velocity": [-3.0, 0.0, -0.5],
        "predicted_strike_position": [0.45, 0.1, 1.0],
        "predicted_strike_velocity": [-3.0, 0.0, -0.5],
        "time_to_strike_s": 0.5,
        "source_timestamp": 10.0,
        "racket_normal": [0.93, 0.0, 0.36],
        "racket_velocity": [2.0, 0.0, 0.7],
        "shot_id": "shot-1",
        "valid": True,
    }


def motion_config(robot: str, command_id: str = "cfg-1", **overrides):
    defaults = {
        "table_right": {
            "goal_x": 0.30,
            "outward_y": -0.90,
            "home_y": -0.20,
            "outward_hold_s": 4.0,
        },
        "table_left": {
            "goal_x": 0.25,
            "outward_y": 0.90,
            "home_y": 0.20,
            "outward_hold_s": 5.0,
        },
    }
    return {
        "schema": RUNTIME_MOTION_CONFIG_SCHEMA,
        "command_id": command_id,
        "published_at": 123.0,
        "robot_id": robot,
        **defaults[robot],
        **overrides,
    }


def test_valid_ball_prediction_requires_explicit_shot_id():
    runtime = V9RealCbfRuntime(shadow=True)
    payload = ball_prediction()
    payload["shot_id"] = None
    with np.testing.assert_raises_regex(ValueError, "shot_id"):
        runtime.update_ball(payload, 1.0)


def test_valid_ball_prediction_requires_source_timestamp():
    runtime = V9RealCbfRuntime(shadow=True)
    payload = ball_prediction()
    payload["source_timestamp"] = None
    with np.testing.assert_raises_regex(ValueError, "source_timestamp"):
        runtime.update_ball(payload, 1.0)


def load_inputs(runtime, timestamp=1.0):
    runtime.update_robot_state(
        "table_right", robot_state("table_right", -0.35), timestamp
    )
    runtime.update_robot_state(
        "table_left", robot_state("table_left", 0.35), timestamp
    )
    runtime.update_ball(ball_prediction(), timestamp)


def test_physical_robot_order_matches_mirrored_and_canonical_slots():
    config = v9_real_planner_config()
    assert config.robots == ("table_right", "table_left")
    assert config.initial_hitter == "table_right"
    assert config.home_y == (-0.20, 0.20)
    assert config.outward_y == (-0.70, 0.70)


def test_missing_input_is_fail_closed_for_both_robots():
    outputs = V9RealCbfRuntime(shadow=True).tick(now=1.0)
    assert set(outputs) == set(ROBOT_ORDER)
    assert all(not payload["valid"] for payload in outputs.values())
    assert all(not payload["command"]["active"] for payload in outputs.values())
    assert all(payload["schema_version"] == PLANNER_COMMAND_SCHEMA for payload in outputs.values())


def test_shadow_keeps_planned_hit_visible_but_never_activates_output():
    runtime = V9RealCbfRuntime(shadow=True, session_id="test-session")
    load_inputs(runtime)
    outputs = runtime.tick(now=1.0)
    planned = [name for name, payload in outputs.items() if payload["planned_active"]]
    active = [name for name, payload in outputs.items() if payload["command"]["active"]]
    assert len(planned) == 1
    assert active == []
    assert all(payload["planned_valid"] for payload in outputs.values())
    assert all(not payload["valid"] for payload in outputs.values())
    assert all(payload["session_id"] == "test-session" for payload in outputs.values())
    assert outputs["table_right"]["return_target_y"] == -0.20
    assert outputs["table_left"]["return_target_y"] == 0.20
    assert outputs["table_right"]["post_hit_outward_y"] == -0.70
    assert outputs["table_left"]["post_hit_outward_y"] == 0.70


def test_active_runtime_exposes_exactly_one_hitter():
    runtime = V9RealCbfRuntime(shadow=False)
    load_inputs(runtime)
    outputs = runtime.tick(now=1.0)
    active = [name for name, payload in outputs.items() if payload["command"]["active"]]
    assert len(active) == 1
    assert outputs[active[0]]["command"]["role"] == "hit"
    assert all(payload["valid"] for payload in outputs.values())
    assert all(
        payload["prediction_source_timestamp_s"] == 10.0
        for payload in outputs.values()
    )


def test_duplicate_state_does_not_refresh_and_becomes_stale():
    runtime = V9RealCbfRuntime(shadow=True)
    load_inputs(runtime)
    assert not runtime.update_robot_state(
        "table_left", robot_state("table_left", 0.35, sequence=1), 1.2
    )
    outputs = runtime.tick(now=1.26)
    assert all(not payload["valid"] for payload in outputs.values())


def test_robot_state_sequence_restart_is_accepted_with_newer_source_clock():
    runtime = V9RealCbfRuntime(shadow=True)
    initial = robot_state("table_left", 0.35, sequence=100)
    initial["source_monotonic_ns"] = 2_000_000_000
    assert runtime.update_robot_state("table_left", initial, 1.0)

    restarted = robot_state("table_left", 0.36, sequence=1)
    restarted["source_monotonic_ns"] = 3_000_000_000
    assert runtime.update_robot_state("table_left", restarted, 1.1)
    assert runtime._state_sequences["table_left"] == 1
    assert np.isclose(runtime._states["table_left"]["base_position_xyz"][1], 0.36)


def test_robot_outside_feedback_workspace_forces_two_robot_safe_hold():
    runtime = V9RealCbfRuntime(shadow=False)
    runtime.update_robot_state(
        "table_right", robot_state("table_right", -0.35), 1.0
    )
    runtime.update_robot_state(
        "table_left", robot_state("table_left", 1.20), 1.0
    )
    runtime.update_ball(ball_prediction(), 1.0)
    outputs = runtime.tick(now=1.0)
    assert all(not payload["valid"] for payload in outputs.values())
    assert all(not payload["command"]["active"] for payload in outputs.values())


def test_startup_feedback_can_be_slightly_outside_goal_workspace():
    runtime = V9RealCbfRuntime(shadow=True)
    runtime.update_robot_state(
        "table_right", robot_state("table_right", -1.03), 1.0
    )
    runtime.update_robot_state(
        "table_left", robot_state("table_left", 0.20), 1.0
    )
    runtime.update_ball(ball_prediction(), 1.0)
    outputs = runtime.tick(now=1.0)
    assert all(payload["planned_valid"] for payload in outputs.values())
    assert all(not payload["valid"] for payload in outputs.values())


def test_first_hit_is_fixed_left_with_peer_stable_in_outward_hold():
    runtime = V9RealCbfRuntime(shadow=True)
    peer = robot_state("table_right", -0.70)
    peer["phase"] = "OUTWARD_HOLD"
    peer["ready"] = True
    hitter = robot_state("table_left", 0.20)
    runtime.update_robot_state("table_right", peer, 1.0)
    runtime.update_robot_state("table_left", hitter, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    outputs = runtime.tick(now=1.0)
    assert outputs["table_left"]["planned_active"]
    assert outputs["table_left"]["command"]["role"] == "hit"
    assert not outputs["table_right"]["planned_active"]


def test_restart_recovers_home_robot_as_next_hitter():
    runtime = V9RealCbfRuntime(shadow=True)
    right = robot_state("table_right", -0.20)
    left = robot_state("table_left", 0.70)
    left["phase"] = "OUTWARD_HOLD"
    left["ready"] = True
    runtime.update_robot_state("table_right", right, 1.0)
    runtime.update_robot_state("table_left", left, 1.0)
    payload = ball_prediction()
    payload["shot_id"] = "restart-shot"
    runtime.update_ball(payload, 1.0)
    outputs = runtime.tick(now=1.0)
    assert outputs["table_right"]["planned_active"]
    assert outputs["table_right"]["command"]["role"] == "hit"
    assert not outputs["table_left"]["planned_active"]


def test_new_hit_preempts_released_hitter_phases():
    for phase in ("OUTWARD", "OUTWARD_HOLD", "RETURN"):
        runtime = V9RealCbfRuntime(shadow=True)
        peer = robot_state("table_left", 0.70)
        peer["phase"] = "OUTWARD_HOLD"
        peer["ready"] = True
        hitter = robot_state("table_right", -0.20)
        hitter["phase"] = phase
        hitter["ready"] = False
        runtime.update_robot_state("table_left", peer, 1.0)
        runtime.update_robot_state("table_right", hitter, 1.0)
        runtime.update_ball(ball_prediction(), 1.0)

        outputs = runtime.tick(now=1.0)

        assert outputs["table_right"]["planned_active"], phase
        assert outputs["table_right"]["command"]["role"] == "hit", phase
        assert not outputs["table_left"]["planned_active"], phase


def test_new_hit_does_not_preempt_hit_locked_phases():
    for phase in ("HIT", "POST_DELAY"):
        runtime = V9RealCbfRuntime(shadow=True)
        peer = robot_state("table_right", -0.70)
        peer["phase"] = "OUTWARD_HOLD"
        peer["ready"] = True
        hitter = robot_state("table_left", 0.20)
        hitter["phase"] = phase
        hitter["ready"] = True
        runtime.update_robot_state("table_right", peer, 1.0)
        runtime.update_robot_state("table_left", hitter, 1.0)
        runtime.update_ball(ball_prediction(), 1.0)

        outputs = runtime.tick(now=1.0)

        assert not outputs["table_left"]["planned_active"], phase
        assert outputs["table_left"]["command"]["role"] != "hit", phase


def test_fixed_relay_first_hit_holds_peer_and_disables_cbf():
    runtime = V9RealFixedRelayRuntime(shadow=True, session_id="fixed-session")
    peer = robot_state("table_left", 0.70)
    peer["phase"] = "OUTWARD_HOLD"
    hitter = robot_state("table_right", -0.20)
    runtime.update_robot_state("table_left", peer, 1.0)
    runtime.update_robot_state("table_right", hitter, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)

    outputs = runtime.tick(now=1.0)

    assert outputs["table_right"]["planner_mode"] == "fixed_relay"
    assert outputs["table_right"]["planned_active"]
    assert outputs["table_right"]["command"]["role"] == "hit"
    assert outputs["table_right"]["post_hit_outward_y"] == -0.70
    assert outputs["table_right"]["return_target_y"] == -0.20
    assert outputs["table_left"]["command"]["role"] == "hold"
    assert not outputs["table_left"]["safety_active"]
    assert not outputs["table_right"]["safety_active"]


def test_fixed_relay_releases_commit_that_never_reaches_hitter():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    runtime.update_robot_state("table_right", robot_state("table_right", -0.20), 1.0)
    peer = robot_state("table_left", 0.70)
    peer["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_left", peer, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    first = runtime.tick(now=1.0)
    assert first["table_right"]["planned_active"]
    assert runtime._swap_commit_token is not None
    assert not runtime._swap_observed_hit_phase

    invalid = ball_prediction(sequence=2)
    invalid["valid"] = False
    invalid["shot_id"] = None
    invalid["time_to_strike_s"] = -0.52
    runtime.update_ball(invalid, 1.1)
    runtime.tick(now=1.1 + runtime.config.commit_handoff_timeout)

    assert runtime._swap_hitter is None
    assert runtime._swap_commit_token is None
    assert runtime._swap_committed_at is None


def test_fixed_relay_returns_peer_only_after_hitter_starts_outward():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    right = robot_state("table_right", -0.70)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.20)
    runtime.update_robot_state("table_right", right, 1.0)
    runtime.update_robot_state("table_left", left, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    runtime.tick(now=1.0)

    right = robot_state("table_right", -0.70, sequence=2)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.20, sequence=2)
    left["phase"] = "HIT"
    runtime.update_robot_state("table_right", right, 1.1)
    runtime.update_robot_state("table_left", left, 1.1)
    payload = ball_prediction(sequence=2)
    payload["time_to_strike_s"] = 0.1
    runtime.update_ball(payload, 1.1)
    before_outward = runtime.tick(now=1.1)
    assert before_outward["table_right"]["command"]["role"] == "hold"

    right = robot_state("table_right", -0.70, sequence=3)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.30, sequence=3)
    left["phase"] = "OUTWARD"
    runtime.update_robot_state("table_right", right, 1.2)
    runtime.update_robot_state("table_left", left, 1.2)
    payload = ball_prediction(sequence=3)
    payload["time_to_strike_s"] = -0.1
    runtime.update_ball(payload, 1.2)
    outputs = runtime.tick(now=1.2)

    command = outputs["table_right"]["command"]
    assert command["role"] == "return"
    assert not command["active"]
    assert command["desired_base_position"][1] == -0.20
    assert command["trajectory_base_position"][1] == -0.20
    assert outputs["table_left"]["command"]["role"] == "hold"

    right = robot_state("table_right", -0.70, sequence=4)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.40, sequence=4)
    left["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.3)
    runtime.update_robot_state("table_left", left, 1.3)
    invalid = ball_prediction(sequence=4)
    invalid["valid"] = False
    invalid["shot_id"] = None
    invalid["time_to_strike_s"] = -0.52
    runtime.update_ball(invalid, 1.3)
    repeated = runtime.tick(now=1.3)
    assert repeated["table_right"]["command"]["role"] == "return"

    right = robot_state("table_right", -0.70, sequence=5)
    right["phase"] = "RETURN"
    right["ready"] = False
    left = robot_state("table_left", 0.50, sequence=5)
    left["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.4)
    runtime.update_robot_state("table_left", left, 1.4)
    invalid = ball_prediction(sequence=5)
    invalid["valid"] = False
    invalid["shot_id"] = None
    invalid["time_to_strike_s"] = -0.52
    runtime.update_ball(invalid, 1.4)
    acknowledged = runtime.tick(now=1.4)
    assert acknowledged["table_right"]["command"]["role"] == "hold"
    assert runtime._return_dispatched


def test_fixed_relay_completes_two_strictly_alternating_cycles():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    right = robot_state("table_right", -0.70)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.20)
    runtime.update_robot_state("table_right", right, 1.0)
    runtime.update_robot_state("table_left", left, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    first = runtime.tick(now=1.0)
    assert first["table_left"]["planned_active"]

    right = robot_state("table_right", -0.70, sequence=2)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.20, sequence=2)
    left["phase"] = "HIT"
    runtime.update_robot_state("table_right", right, 1.05)
    runtime.update_robot_state("table_left", left, 1.05)
    payload = ball_prediction(sequence=2)
    payload["time_to_strike_s"] = 0.1
    runtime.update_ball(payload, 1.05)
    runtime.tick(now=1.05)

    right = robot_state("table_right", -0.70, sequence=3)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.40, sequence=3)
    left["phase"] = "OUTWARD"
    runtime.update_robot_state("table_right", right, 1.10)
    runtime.update_robot_state("table_left", left, 1.10)
    payload = ball_prediction(sequence=3)
    payload["time_to_strike_s"] = -0.1
    runtime.update_ball(payload, 1.10)
    runtime.tick(now=1.10)

    right = robot_state("table_right", -0.20, sequence=4)
    left = robot_state("table_left", 0.70, sequence=4)
    left["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.15)
    runtime.update_robot_state("table_left", left, 1.15)
    payload = ball_prediction(sequence=4)
    payload["shot_id"] = "shot-2"
    runtime.update_ball(payload, 1.15)
    second = runtime.tick(now=1.15)
    assert second["table_right"]["planned_active"]
    assert second["table_right"]["command"]["role"] == "hit"
    assert second["table_left"]["command"]["role"] == "hold"

    right = robot_state("table_right", -0.30, sequence=5)
    right["phase"] = "HIT"
    left = robot_state("table_left", 0.70, sequence=5)
    left["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.2)
    runtime.update_robot_state("table_left", left, 1.2)
    payload = ball_prediction(sequence=5)
    payload["shot_id"] = "shot-2"
    payload["time_to_strike_s"] = 0.1
    runtime.update_ball(payload, 1.2)
    runtime.tick(now=1.2)

    right = robot_state("table_right", -0.40, sequence=6)
    right["phase"] = "OUTWARD"
    left = robot_state("table_left", 0.70, sequence=6)
    left["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.3)
    runtime.update_robot_state("table_left", left, 1.3)
    payload = ball_prediction(sequence=6)
    payload["shot_id"] = "shot-2"
    payload["time_to_strike_s"] = -0.1
    runtime.update_ball(payload, 1.3)
    returned = runtime.tick(now=1.3)
    assert returned["table_left"]["command"]["role"] == "return"
    assert returned["table_left"]["return_target_y"] == 0.20


def test_fixed_relay_preempts_all_released_hitter_phases():
    for phase in ("OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD"):
        runtime = V9RealFixedRelayRuntime(shadow=True)
        peer = robot_state("table_right", -0.70)
        peer["phase"] = "OUTWARD_HOLD"
        hitter = robot_state("table_left", 0.20)
        hitter["phase"] = phase
        hitter["ready"] = phase in {"OUTWARD_HOLD", "HOME_HOLD"}
        if phase == "OUTWARD_HOLD":
            hitter["state_elapsed_s"] = 3.0
        runtime.update_robot_state("table_right", peer, 1.0)
        runtime.update_robot_state("table_left", hitter, 1.0)
        runtime.update_ball(ball_prediction(), 1.0)
        outputs = runtime.tick(now=1.0)
        assert outputs["table_left"]["planned_active"], phase
        assert outputs["table_right"]["command"]["role"] == "hold", phase


def test_fixed_relay_does_not_preempt_hit_locked_phases():
    for phase in ("HIT", "POST_DELAY"):
        runtime = V9RealFixedRelayRuntime(shadow=True)
        peer = robot_state("table_right", -0.70)
        peer["phase"] = "OUTWARD_HOLD"
        hitter = robot_state("table_left", 0.20)
        hitter["phase"] = phase
        runtime.update_robot_state("table_right", peer, 1.0)
        runtime.update_robot_state("table_left", hitter, 1.0)
        runtime.update_ball(ball_prediction(), 1.0)
        outputs = runtime.tick(now=1.0)
        assert all(not payload["planned_active"] for payload in outputs.values()), phase


def test_fixed_relay_allows_peer_home_hold():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    runtime.update_robot_state("table_right", robot_state("table_right", -0.20), 1.0)
    runtime.update_robot_state("table_left", robot_state("table_left", 0.20), 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    outputs = runtime.tick(now=1.0)
    assert outputs["table_right"]["planned_active"]


def test_fixed_relay_rejects_peer_hit_locked_phase():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    peer = robot_state("table_right", -0.20)
    peer["phase"] = "HIT"
    runtime.update_robot_state("table_right", peer, 1.0)
    runtime.update_robot_state("table_left", robot_state("table_left", 0.20), 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    outputs = runtime.tick(now=1.0)
    assert all(not payload["planned_active"] for payload in outputs.values())
    assert "peer_hit_locked" in outputs["table_left"]["admission_reasons"]


def test_fixed_relay_invalid_prediction_clears_uncommitted_pending_shot():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    peer = robot_state("table_right", -0.20)
    peer["phase"] = "POST_DELAY"
    runtime.update_robot_state("table_right", peer, 1.0)
    runtime.update_robot_state("table_left", robot_state("table_left", 0.20), 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    reserved = runtime.tick(now=1.0)
    assert reserved["table_left"]["pending_shot_id"] == "shot-1"
    assert "peer_hit_locked" in reserved["table_left"]["admission_reasons"]

    invalid = ball_prediction(sequence=2)
    invalid["valid"] = False
    invalid["shot_id"] = None
    invalid["time_to_strike_s"] = -0.52
    runtime.update_ball(invalid, 1.1)
    cleared = runtime.tick(now=1.1)
    assert cleared["table_left"]["pending_shot_id"] is None
    assert cleared["table_left"]["commit_token"] is None

    right = robot_state("table_right", -0.70, sequence=2)
    right["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.2)
    runtime.update_robot_state("table_left", robot_state("table_left", 0.20, 2), 1.2)
    next_ball = ball_prediction(sequence=3)
    next_ball["shot_id"] = "shot-2"
    runtime.update_ball(next_ball, 1.2)
    committed = runtime.tick(now=1.2)
    assert committed["table_left"]["planned_active"]
    assert committed["table_left"]["commit_token"].endswith(":shot-2")


def test_fixed_relay_invalid_prediction_still_advances_committed_handoff():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    right = robot_state("table_right", -0.70)
    right["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.0)
    runtime.update_robot_state("table_left", robot_state("table_left", 0.20), 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    first = runtime.tick(now=1.0)
    assert first["table_left"]["planned_active"]

    right = robot_state("table_right", -0.70, sequence=2)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.20, sequence=2)
    left["phase"] = "HIT"
    runtime.update_robot_state("table_right", right, 1.1)
    runtime.update_robot_state("table_left", left, 1.1)
    invalid = ball_prediction(sequence=2)
    invalid["valid"] = False
    invalid["shot_id"] = None
    invalid["time_to_strike_s"] = -0.52
    runtime.update_ball(invalid, 1.1)
    runtime.tick(now=1.1)

    right = robot_state("table_right", -0.70, sequence=3)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.50, sequence=3)
    left["phase"] = "OUTWARD"
    runtime.update_robot_state("table_right", right, 1.2)
    runtime.update_robot_state("table_left", left, 1.2)
    invalid = ball_prediction(sequence=3)
    invalid["valid"] = False
    invalid["shot_id"] = None
    invalid["time_to_strike_s"] = -0.52
    runtime.update_ball(invalid, 1.2)
    returned = runtime.tick(now=1.2)
    assert returned["table_right"]["command"]["role"] == "return"

    right = robot_state("table_right", -0.20, sequence=4)
    left = robot_state("table_left", 0.70, sequence=4)
    left["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.3)
    runtime.update_robot_state("table_left", left, 1.3)
    next_ball = ball_prediction(sequence=4)
    next_ball["shot_id"] = "shot-2"
    runtime.update_ball(next_ball, 1.3)
    second = runtime.tick(now=1.3)
    assert second["table_right"]["planned_active"]


def test_fixed_relay_long_gap_selects_robot_closer_to_table_center():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    right = robot_state("table_right", -0.70)
    right["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 1.0)
    runtime.update_robot_state("table_left", robot_state("table_left", 0.20), 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    assert runtime.tick(now=1.0)["table_left"]["planned_active"]

    right = robot_state("table_right", -0.70, sequence=2)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.20, sequence=2)
    left["phase"] = "HIT"
    runtime.update_robot_state("table_right", right, 1.1)
    runtime.update_robot_state("table_left", left, 1.1)
    payload = ball_prediction(sequence=2)
    payload["time_to_strike_s"] = 0.1
    runtime.update_ball(payload, 1.1)
    runtime.tick(now=1.1)

    right = robot_state("table_right", -0.70, sequence=3)
    right["phase"] = "OUTWARD_HOLD"
    left = robot_state("table_left", 0.50, sequence=3)
    left["phase"] = "OUTWARD"
    runtime.update_robot_state("table_right", right, 1.2)
    runtime.update_robot_state("table_left", left, 1.2)
    payload = ball_prediction(sequence=3)
    payload["time_to_strike_s"] = -0.1
    runtime.update_ball(payload, 1.2)
    runtime.tick(now=1.2)

    right = robot_state("table_right", -0.40, sequence=4)
    right["phase"] = "RETURN"
    right["ready"] = False
    left = robot_state("table_left", 0.90, sequence=4)
    left["phase"] = "OUTWARD_HOLD"
    runtime.update_robot_state("table_right", right, 12.3)
    runtime.update_robot_state("table_left", left, 12.3)
    next_ball = ball_prediction(sequence=4)
    next_ball["shot_id"] = "shot-after-gap"
    runtime.update_ball(next_ball, 12.3)
    selected = runtime.tick(now=12.3)

    assert selected["table_right"]["planned_active"]
    assert selected["table_right"]["hitter"] == "table_right"
    assert selected["table_left"]["command"]["role"] == "hold"


def test_fixed_relay_center_distance_tie_prefers_initial_right():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    right = robot_state("table_right", -0.40)
    right["phase"] = "OUTWARD_HOLD"
    right["state_elapsed_s"] = 3.0
    left = robot_state("table_left", 0.40)
    left["phase"] = "RETURN"
    left["ready"] = False
    runtime.update_robot_state("table_right", right, 1.0)
    runtime.update_robot_state("table_left", left, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    outputs = runtime.tick(now=1.0)
    assert outputs["table_right"]["planned_active"]
    assert outputs["table_right"]["hitter"] == "table_right"


def test_fixed_relay_honors_runtime_outward_hold_duration():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    right = robot_state("table_right", -0.40)
    right["phase"] = "OUTWARD_HOLD"
    right["state_elapsed_s"] = 2.9
    left = robot_state("table_left", 0.40)
    left["phase"] = "RETURN"
    left["ready"] = False
    runtime.update_robot_state("table_right", right, 1.0)
    runtime.update_robot_state("table_left", left, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    assert not runtime.tick(now=1.0)["table_right"]["planned_active"]

    right = robot_state("table_right", -0.40, sequence=2)
    right["phase"] = "OUTWARD_HOLD"
    right["state_elapsed_s"] = 3.0
    runtime.update_robot_state("table_right", right, 1.1)
    payload = ball_prediction(sequence=2)
    payload["shot_id"] = "shot-2"
    runtime.update_ball(payload, 1.1)
    assert runtime.tick(now=1.1)["table_right"]["planned_active"]


def test_both_robots_at_narrow_home_gap_cannot_commit():
    runtime = V9RealCbfRuntime(shadow=True)
    runtime.update_robot_state(
        "table_right", robot_state("table_right", -0.20), 1.0
    )
    runtime.update_robot_state(
        "table_left", robot_state("table_left", 0.20), 1.0
    )
    runtime.update_ball(ball_prediction(), 1.0)
    outputs = runtime.tick(now=1.0)
    assert all(not payload["planned_active"] for payload in outputs.values())


def test_committed_hitter_outward_commands_peer_return_home():
    runtime = V9RealCbfRuntime(shadow=True)
    peer = robot_state("table_right", -0.70)
    peer["phase"] = "OUTWARD_HOLD"
    hitter = robot_state("table_left", 0.20)
    runtime.update_robot_state("table_right", peer, 1.0)
    runtime.update_robot_state("table_left", hitter, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    first = runtime.tick(now=1.0)
    assert first["table_left"]["planned_active"]

    peer = robot_state("table_right", -0.70, sequence=2)
    peer["phase"] = "OUTWARD_HOLD"
    hitter = robot_state("table_left", 0.50, sequence=2)
    hitter["phase"] = "OUTWARD"
    runtime.update_robot_state("table_right", peer, 1.1)
    runtime.update_robot_state("table_left", hitter, 1.1)
    outputs = runtime.tick(now=1.1)
    command = outputs["table_right"]["command"]
    assert outputs["table_right"]["planned_valid"]
    assert not outputs["table_right"]["valid"]
    assert command["role"] == "stage"
    assert command["trajectory_base_position"][1] == -0.20


def test_peer_waits_outward_until_hitter_crosses_clearance_gate():
    runtime = V9RealCbfRuntime(shadow=True)
    peer = robot_state("table_right", -0.70)
    peer["phase"] = "OUTWARD_HOLD"
    hitter = robot_state("table_left", 0.20)
    runtime.update_robot_state("table_right", peer, 1.0)
    runtime.update_robot_state("table_left", hitter, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    runtime.tick(now=1.0)

    peer = robot_state("table_right", -0.70, sequence=2)
    peer["phase"] = "OUTWARD_HOLD"
    hitter = robot_state("table_left", 0.30, sequence=2)
    hitter["phase"] = "OUTWARD"
    runtime.update_robot_state("table_right", peer, 1.1)
    runtime.update_robot_state("table_left", hitter, 1.1)
    outputs = runtime.tick(now=1.1)
    assert outputs["table_right"]["command"]["role"] != "stage"


def test_runtime_motion_config_is_independent_and_repeated_in_commands():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    load_inputs(runtime)
    status = runtime.stage_runtime_motion_config(
        motion_config("table_right", goal_x=5.0, outward_hold_s=7.0)
    )
    assert status["received_command_id"] == "cfg-1"
    assert status["pending"]["goal_x"] == 5.0
    outputs = runtime.tick(now=1.0)
    assert outputs["table_right"]["runtime_motion_config"]["goal_x"] == 5.0
    assert outputs["table_right"]["runtime_motion_config"]["outward_hold_s"] == 7.0
    assert outputs["table_left"]["runtime_motion_config"]["goal_x"] == -0.03
    assert outputs["table_left"]["runtime_motion_config"]["outward_hold_s"] == 3.0


def test_default_goal_x_is_canonical_015_relative_to_each_first_frame():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    runtime.update_robot_state(
        "table_right", robot_state("table_right", -0.20, x=-0.161), 1.0
    )
    runtime.update_robot_state(
        "table_left", robot_state("table_left", 0.20, x=0.042), 1.0
    )

    status = runtime.runtime_motion_config_status()
    assert np.isclose(status["table_right"]["startup_x"], -0.161)
    assert np.isclose(status["table_right"]["active"]["goal_x"], -0.191)
    assert np.isclose(status["table_right"]["active"]["goal_x_canonical"], 0.15)
    assert np.isclose(status["table_left"]["startup_x"], 0.042)
    assert np.isclose(status["table_left"]["active"]["goal_x"], 0.012)
    assert np.isclose(status["table_left"]["active"]["goal_x_canonical"], 0.15)


def test_startup_x_is_latched_and_web_physical_override_reports_canonical_x():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    runtime.update_robot_state(
        "table_right", robot_state("table_right", -0.20, x=-0.161), 1.0
    )
    runtime.update_robot_state(
        "table_right", robot_state("table_right", -0.20, sequence=2, x=0.25), 1.1
    )
    runtime.stage_runtime_motion_config(
        motion_config("table_right", goal_x=-0.151)
    )

    status = runtime.runtime_motion_config_status()["table_right"]
    assert np.isclose(status["startup_x"], -0.161)
    assert np.isclose(status["pending"]["goal_x"], -0.151)
    assert np.isclose(status["pending"]["goal_x_canonical"], 0.19)


def test_runtime_motion_config_duplicate_command_is_idempotent():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    payload = motion_config("table_left", command_id="same")
    first = runtime.stage_runtime_motion_config(payload)
    second = runtime.stage_runtime_motion_config(payload)
    assert first == second
    assert second["pending"]["command_id"] == "same"
    with np.testing.assert_raises_regex(ValueError, "reused"):
        runtime.stage_runtime_motion_config(
            motion_config("table_left", command_id="same", goal_x=0.51)
        )


def test_runtime_motion_config_rejects_cross_robot_lane_conflict():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    with np.testing.assert_raises(ValueError):
        runtime.stage_runtime_motion_config(
            motion_config(
                "table_right",
                outward_y=0.70,
                home_y=0.20,
            )
        )
    status = runtime.runtime_motion_config_status()["table_right"]
    assert status["pending"] is None
    assert status["error"]


def test_runtime_motion_config_tracks_outward_and_home_ack_separately():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    runtime.stage_runtime_motion_config(
        motion_config(
            "table_right",
            command_id="right-config",
            goal_x=0.42,
            outward_y=-1.0,
            home_y=-0.25,
            outward_hold_s=6.0,
        )
    )
    state = robot_state("table_right", -0.20)
    state["runtime_motion_config"] = {
        "received_command_id": "right-config",
        "outward_applied_command_id": "right-config",
        "home_applied_command_id": None,
        "error": "",
    }
    runtime.update_robot_state("table_right", state, 1.0)
    status = runtime.runtime_motion_config_status()["table_right"]
    assert status["active"]["goal_x"] == 0.42
    assert status["active"]["outward_y"] == -1.0
    assert status["active"]["home_y"] == -0.20
    assert status["pending"]["awaiting_outward"] is False
    assert status["pending"]["awaiting_home"] is True

    state = robot_state("table_right", -0.25, sequence=2)
    state["runtime_motion_config"] = {
        "received_command_id": "right-config",
        "outward_applied_command_id": "right-config",
        "home_applied_command_id": "right-config",
        "error": "",
    }
    runtime.update_robot_state("table_right", state, 1.1)
    status = runtime.runtime_motion_config_status()["table_right"]
    assert status["pending"] is None
    assert status["active"]["home_y"] == -0.25
    assert runtime.config.home_y == (-0.25, 0.20)
