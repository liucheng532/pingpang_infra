import copy

import numpy as np
import pytest

from yichao_v7_v11 import HOME_PAIR
from yichao_v7_v11.features import FeatureTracker
from yichao_v7_v11.v7_runtime import V7Runtime

ROBOTS = ("table_right", "table_left")


class FakeNormalizer:
    def normalize(self, q):
        return np.asarray(q, dtype=np.float32) / 3.0


class Pipeline:
    def __init__(self, pair=(0.4, -0.3), valid=True):
        self.pair = list(pair)
        self.valid = valid
        self.calls = []

    def decide(self, features, hitter):
        self.calls.append((features, hitter))
        return {
            "valid": self.valid,
            "reason": None if self.valid else "guard_projection_unavailable",
            "actor_raw": [0.2, -0.2],
            "actor_physical_m": [0.18, -0.18],
            "guard_projected_m": self.pair if self.valid else None,
            "applied_pair_m": self.pair if self.valid else None,
            "guard_intervened": True,
            "filter_mode": "frozen_v7_cbf",
            "cbf_nominal_risk": 0.2,
            "cbf_selected_risk": 0.1,
            "latency_ms": 0.1,
        }


def state(
    robot,
    sequence,
    *,
    phase=None,
    target=None,
    applied=-1,
    session=None,
    error="",
    error_sequence=None,
    error_session=None,
    token=None,
):
    if phase is None:
        phase = "HOME_HOLD" if robot == "table_right" else "OUTWARD_HOLD"
    y = -0.35 if robot == "table_right" else 0.7
    return {
        "schema_version": "v9-robot-state-v1", "robot": robot, "sequence": sequence,
        "source_monotonic_ns": sequence * 20_000_000, "q": [0.0] * 29, "dq": [0.0] * 29,
        "imu_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0], "gyro_xyz": [0.0] * 3,
        "base_position_xyz": [0.0, y, 0.75], "base_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "base_linear_velocity_xyz": [0.0] * 3, "base_angular_velocity_xyz": [0.0] * 3,
        "phase": phase, "ready": True, "state_elapsed_s": 3.5, "stable_elapsed_s": 1.0,
        "target_base_y": y if target is None else target, "home_y": -0.2 if robot == "table_right" else 0.2,
        "time_to_strike_s": -0.5, "valid": True, "emergency_stop": False,
        "last_applied_sequence": applied, "last_planner_session_id": session,
        "last_commit_token": token, "transport_error": error,
        "transport_error_sequence": error_sequence,
        "transport_error_session_id": error_session,
    }


def ball(sequence=1, shot="shot-1", valid=True):
    return {
        "schema_version": "v9-ball-prediction-v1", "sequence": sequence,
        "position": [1.2, 0.1, 1.1], "velocity": [-3.0, 0.0, -0.4],
        "predicted_strike_position": [0.45, 0.1, 1.0],
        "predicted_strike_velocity": [-3.0, 0.0, -0.4], "time_to_strike_s": 0.5,
        "source_timestamp": 10.0, "racket_normal": [1.0, 0.0, 0.0],
        "racket_velocity": [2.0, 0.0, 0.7], "shot_id": shot, "valid": valid,
    }


def make_runtime(mode="active", pipeline=None):
    pipeline = pipeline or Pipeline()
    tracker = FeatureTracker(FakeNormalizer())
    runtime = V7Runtime(
        mode=mode, session_id="session", pipeline=pipeline, feature_tracker=tracker,
        attestation_verified=mode != "shadow",
    )
    for sequence in range(1, 8):
        received = 0.98 + sequence * 0.02
        for robot in ROBOTS:
            runtime.update_robot_state(robot, state(robot, sequence), received)
    runtime.update_ball(ball(), 1.12)
    return runtime, pipeline


def test_physical_relay_uses_fixed_v3_return_and_outward_lanes():
    runtime, _ = make_runtime()
    assert runtime.config.home_y == pytest.approx((-0.20, 0.20))
    assert runtime.config.ready_y == pytest.approx((-0.20, 0.20))
    assert runtime.config.outward_y == pytest.approx((-0.70, 0.70))
    assert runtime.config.teammate_avoidance_y == pytest.approx((-0.65, 0.65))
    assert runtime._motion_config_active["table_right"].home_y == pytest.approx(-0.20)
    assert runtime._motion_config_active["table_right"].outward_y == pytest.approx(-0.70)
    assert runtime._motion_config_active["table_left"].home_y == pytest.approx(0.20)
    assert runtime._motion_config_active["table_left"].outward_y == pytest.approx(0.70)

    # These are V7 model-domain values, not physical relay lanes.
    assert HOME_PAIR == pytest.approx((0.35, -0.35))
    assert runtime.config.workspace_y == pytest.approx((-0.90, 0.90))


def stage(runtime):
    first = runtime.tick(1.14)
    assert len(runtime.pipeline.calls) == 1
    assert not any(payload["command"]["role"] == "hit" for payload in first.values())
    second = runtime.tick(1.16)
    assert all(payload["command"]["role"] == "stage" for payload in second.values())
    return second


def test_stage_preserves_fixed_rule_commands_before_actor_override():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    assert staged["table_left"]["command"]["desired_base_position"][1] == pytest.approx(0.4)
    assert staged["table_right"]["command"]["desired_base_position"][1] == pytest.approx(-0.3)
    assert staged["table_left"]["fixed_command"]["desired_base_position"][1] == pytest.approx(0.7)
    assert staged["table_right"]["fixed_command"]["desired_base_position"][1] == pytest.approx(-0.35)
    assert staged["table_left"]["fixed_command"]["role"] != "stage"
    assert staged["table_right"]["fixed_command"]["role"] != "stage"


def test_stage_and_ack_use_first_valid_shot_receive_time():
    runtime, _ = make_runtime()
    assert runtime._ball["_shot_receive_monotonic"] == pytest.approx(1.12)
    runtime.update_ball(ball(sequence=2, shot="shot-1"), 1.13)
    assert runtime._ball["_receive_monotonic"] == pytest.approx(1.13)
    assert runtime._ball["_shot_receive_monotonic"] == pytest.approx(1.12)
    staged = stage(runtime)
    assert runtime._staging["ball_received"] == pytest.approx(1.12)
    acknowledge(runtime, staged)
    runtime.tick(1.18)
    assert runtime.features.last_fully_acked_shot_id == "shot-1"


def acknowledge(runtime, staged, *, left=True, right=True, position_at_target=False, error=""):
    for robot, enabled in (("table_left", left), ("table_right", right)):
        target = 0.4 if robot == "table_left" else -0.3
        applied = staged[robot]["sequence"] if enabled else -1
        payload = state(
            robot, 8, phase="OUTWARD_HOLD", target=target if enabled else 0.0,
            applied=applied,
            session="session" if enabled else None,
            error=error if enabled else "",
            error_sequence=applied if enabled and error else None,
            error_session="session" if enabled and error else None,
        )
        if position_at_target:
            payload["base_position_xyz"][1] = target
        runtime.update_robot_state(robot, payload, 1.18)


def test_dual_ack_releases_fixed_hit_without_physical_arrival_gate():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    acknowledge(runtime, staged, position_at_target=False)
    outputs = runtime.tick(1.18)
    hits = [robot for robot, payload in outputs.items() if payload["command"]["role"] == "hit"]
    assert hits == ["table_right"]
    assert outputs["table_right"]["planned_active"] is True
    assert runtime.features.previous_targets.tolist() == pytest.approx([0.4, -0.3])
    assert outputs["table_right"]["return_target_y"] == pytest.approx(-0.20)
    assert outputs["table_right"]["post_hit_outward_y"] == pytest.approx(-0.70)
    assert outputs["table_left"]["return_target_y"] == pytest.approx(0.20)
    assert outputs["table_left"]["post_hit_outward_y"] == pytest.approx(0.70)
    # Pelvis is still at old Y, proving ACK is transport acceptance, not 6 cm arrival.
    assert runtime._states["table_left"]["base_position_xyz"][1] == 0.7


def test_single_side_ack_waits_and_does_not_update_previous_targets():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    acknowledge(runtime, staged, left=True, right=False)
    outputs = runtime.tick(1.18)
    assert not any(payload["command"]["role"] == "hit" for payload in outputs.values())
    assert runtime._staging["acknowledged"] == ["table_left"]
    assert runtime.features.previous_targets.tolist() == pytest.approx(HOME_PAIR)


def test_ack_timeout_and_transport_rejection_restore_relay_pair_and_blacklist_shot():
    runtime, _ = make_runtime()
    stage(runtime)
    for robot in ROBOTS:
        runtime.update_robot_state(robot, state(robot, 8), 1.40)
    outputs = runtime.tick(1.42)
    assert runtime.failed_shots["shot-1"] == "pair_stage_ack_timeout"
    assert outputs["table_right"]["command"]["role"] == "return"
    assert outputs["table_right"]["command"]["desired_base_position"][1] == pytest.approx(-0.20)
    assert outputs["table_left"]["command"]["role"] == "outward"
    assert outputs["table_left"]["command"]["desired_base_position"][1] == pytest.approx(0.70)
    assert {payload["fault_recovery_mode"] for payload in outputs.values()} == {"relay_pair"}
    assert all(not payload["planned_active"] for payload in outputs.values())
    again = runtime.tick(1.43)
    assert not any(payload["command"]["role"] == "hit" for payload in again.values())

    rejected, _ = make_runtime()
    staged = stage(rejected)
    acknowledge(rejected, staged, error="command_rejected:test")
    rejected.tick(1.18)
    assert rejected.failed_shots["shot-1"].startswith("stage_rejected")
    assert rejected._staging["terminal"] is True
    reason = rejected.reason
    rejected.tick(1.50)
    assert rejected.reason == reason


def test_stale_transport_error_cannot_reject_a_newer_accepted_stage():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    for robot in ROBOTS:
        target = 0.4 if robot == "table_left" else -0.3
        expected = staged[robot]["sequence"]
        runtime.update_robot_state(
            robot,
            state(
                robot,
                8,
                phase="OUTWARD_HOLD",
                target=target,
                applied=expected,
                session="session",
                error="planner_command_invalid",
                error_sequence=expected - 1,
                error_session="session",
            ),
            1.18,
        )
    outputs = runtime.tick(1.18)
    assert "shot-1" not in runtime.failed_shots
    assert [
        robot for robot, payload in outputs.items()
        if payload["command"]["role"] == "hit"
    ] == ["table_right"]


def test_transport_shadow_publishes_stage_ack_but_never_hit():
    runtime, _ = make_runtime("transport-shadow")
    staged = stage(runtime)
    assert all(payload["valid"] for payload in staged.values())
    acknowledge(runtime, staged)
    outputs = runtime.tick(1.18)
    assert "shot-1" in runtime.completed_transport_shots
    assert runtime.features.previous_targets.tolist() == pytest.approx([0.4, -0.3])
    assert not any(payload["command"]["role"] == "hit" for payload in outputs.values())
    assert not any(payload["planned_active"] for payload in outputs.values())


def test_shadow_has_no_valid_command_even_with_actor_decision():
    runtime, _ = make_runtime("shadow")
    outputs = stage(runtime)
    assert not runtime.command_publish_enabled
    assert all(not payload["valid"] for payload in outputs.values())


def test_invalid_actor_pair_is_permanent_fixed_relay_fallback():
    invalid = Pipeline(valid=False)
    invalid.pair = None
    runtime, pipeline = make_runtime(pipeline=invalid)
    outputs = runtime.tick(1.14)
    assert runtime.failed_shots == {"shot-1": "guard_projection_unavailable"}
    assert outputs["table_right"]["command"]["role"] == "return"
    assert outputs["table_right"]["return_target_y"] == pytest.approx(-0.20)
    assert outputs["table_left"]["command"]["role"] == "outward"
    assert outputs["table_left"]["post_hit_outward_y"] == pytest.approx(0.70)
    runtime.tick(1.16)
    assert len(pipeline.calls) == 1
    assert runtime.features.previous_targets.tolist() == pytest.approx(HOME_PAIR)


def test_v7_changes_only_y_and_latches_one_pair_per_shot():
    runtime, pipeline = make_runtime()
    staged = stage(runtime)
    assert all(payload["command"]["desired_base_position"][0] == 0.0 for payload in staged.values())
    runtime.update_ball(ball(sequence=2, shot="shot-1"), 1.17)
    runtime.tick(1.17)
    assert len(pipeline.calls) == 1


def test_controller_restart_stale_state_and_wrong_session_fail_closed():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    reset = state("table_left", 1)
    reset["source_monotonic_ns"] = 999_000_000
    runtime.update_robot_state("table_left", reset, 1.18)
    outputs = runtime.tick(1.18)
    assert runtime._restart_fault.startswith("controller_restart")
    assert not any(payload["planned_active"] for payload in outputs.values())
    assert all(payload["command"]["role"] == "hold" for payload in outputs.values())
    assert {payload["fault_recovery_mode"] for payload in outputs.values()} == {
        "hold_last_target"
    }


def test_applied_sequence_or_planner_session_regression_fails_immediately():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    acknowledge(runtime, staged)
    runtime.tick(1.18)

    rollback = state(
        "table_left", 9, applied=staged["table_left"]["sequence"] - 1,
        session="session",
    )
    runtime.update_robot_state("table_left", rollback, 1.20)
    outputs = runtime.tick(1.20)
    assert runtime._restart_fault == "controller_applied_sequence_regression:table_left"
    assert not any(payload["planned_active"] for payload in outputs.values())

    changed, _ = make_runtime()
    staged = stage(changed)
    acknowledge(changed, staged)
    changed.tick(1.18)
    wrong_session = state(
        "table_right", 9, applied=staged["table_right"]["sequence"],
        session="another-session",
    )
    changed.update_robot_state("table_right", wrong_session, 1.20)
    outputs = changed.tick(1.20)
    assert changed._restart_fault == "controller_session_regression:table_right"
    assert not any(payload["planned_active"] for payload in outputs.values())


def test_failed_shot_blacklist_is_not_evicted_within_session():
    runtime, pipeline = make_runtime()
    runtime.failed_shots = {f"failed-{index}": "test" for index in range(80)}
    runtime._make_decision("new-shot", "table_left")
    assert "failed-0" in runtime.failed_shots
    assert len(runtime.failed_shots) >= 80
    calls = len(pipeline.calls)
    runtime._make_decision("failed-0", "table_left")
    assert len(pipeline.calls) == calls

    wrong, _ = make_runtime()
    staged = stage(wrong)
    acknowledge(wrong, staged)
    wrong._states["table_left"]["last_planner_session_id"] = "another-session"
    outputs = wrong.tick(1.18)
    assert not any(payload["command"]["role"] == "hit" for payload in outputs.values())

    stale, _ = make_runtime()
    stage(stale)
    outputs = stale.tick(1.40)
    assert stale.failed_shots["shot-1"] in {
        "pair_stage_ack_timeout", "stage_state_invalid_or_stale:table_right"
    }
    assert not any(payload["planned_active"] for payload in outputs.values())


def test_invalid_or_replaced_ball_cancels_staging():
    runtime, _ = make_runtime()
    stage(runtime)
    runtime.update_ball(ball(sequence=2, shot="shot-1", valid=False), 1.18)
    outputs = runtime.tick(1.18)
    assert runtime.failed_shots["shot-1"] == "stage_ball_invalid_stale_or_replaced"
    assert not any(payload["planned_active"] for payload in outputs.values())


def test_hit_post_lock_and_peer_return_only_after_hitter_outward():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    acknowledge(runtime, staged)
    hit = runtime.tick(1.18)
    token = hit["table_right"]["commit_token"]
    for robot in ROBOTS:
        phase = "HIT" if robot == "table_right" else "OUTWARD_HOLD"
        payload = state(robot, 9, phase=phase, target=-0.3 if robot == "table_right" else 0.4,
                        applied=hit[robot]["sequence"], session="session", token=token if robot == "table_right" else None)
        runtime.update_robot_state(robot, payload, 1.20)
    locked = runtime.tick(1.20)
    assert locked["table_left"]["command"]["role"] != "return"
    for robot in ROBOTS:
        phase = "OUTWARD" if robot == "table_right" else "OUTWARD_HOLD"
        payload = state(robot, 10, phase=phase, target=-0.3 if robot == "table_right" else 0.4,
                        applied=locked[robot]["sequence"], session="session", token=token if robot == "table_right" else None)
        runtime.update_robot_state(robot, payload, 1.22)
    returned = runtime.tick(1.22)
    assert returned["table_left"]["command"]["role"] == "return"
    assert returned["table_left"]["command"]["desired_base_position"][1] == pytest.approx(0.20)
    assert returned["table_left"]["return_trigger_reason"] == "peer_outward"


def test_fault_during_hit_does_not_replace_fixed_owned_hit_with_return():
    runtime, _ = make_runtime()
    staged = stage(runtime)
    acknowledge(runtime, staged)
    hit = runtime.tick(1.18)
    token = hit["table_right"]["commit_token"]
    for robot in ROBOTS:
        phase = "HIT" if robot == "table_right" else "OUTWARD_HOLD"
        payload = state(
            robot, 9, phase=phase,
            target=-0.3 if robot == "table_right" else 0.4,
            applied=hit[robot]["sequence"], session="session",
            token=token if robot == "table_right" else None,
        )
        runtime.update_robot_state(robot, payload, 1.20)
    runtime._restart_fault = "controller_restart_or_clock_regression:table_left"
    outputs = runtime.tick(1.20)
    assert outputs["table_right"]["command"]["role"] != "return"
    assert outputs["table_left"]["command"]["role"] not in {"return", "outward"}
    assert {payload["fault_recovery_mode"] for payload in outputs.values()} == {
        "locked_phase_preserved"
    }


def test_non_shadow_requires_attestation():
    with pytest.raises(ValueError, match="attestation"):
        V7Runtime(mode="active", session_id="s", pipeline=Pipeline())


def test_fixed_reservation_hitter_and_commit_token_are_unchanged():
    from doubles_planner.real_ros_runtime import V9RealFixedRelayRuntime

    adapted, _ = make_runtime()
    fixed = V9RealFixedRelayRuntime(shadow=False, session_id="fixed")
    for sequence in range(1, 8):
        received = 0.98 + sequence * 0.02
        for robot in ROBOTS:
            fixed.update_robot_state(robot, state(robot, sequence), received)
    fixed.update_ball(ball(), 1.12)
    fixed_output = fixed.tick(1.14)
    adapted_output = adapted.tick(1.14)
    for robot in ROBOTS:
        assert adapted_output[robot]["hitter"] == fixed_output[robot]["hitter"]
        assert adapted_output[robot]["commit_token"] == fixed_output[robot]["commit_token"]
        assert adapted_output[robot]["pending_shot_id"] == fixed_output[robot]["shot_id"]


def test_simultaneous_hit_output_is_suppressed_and_blacklisted():
    runtime, _ = make_runtime()
    baseline = runtime.tick(1.14)
    forged = copy.deepcopy(baseline)
    for payload in forged.values():
        payload["planned_active"] = True
        payload["valid"] = True
        payload["command"].update(role="hit", active=True)
    runtime._call_fixed = lambda: copy.deepcopy(forged)
    outputs = runtime.tick(1.15)
    assert runtime.failed_shots["shot-1"] == "simultaneous_hit_guard"
    assert not any(payload["planned_active"] for payload in outputs.values())
    assert outputs["table_right"]["command"]["role"] == "return"
    assert outputs["table_left"]["command"]["role"] == "outward"
