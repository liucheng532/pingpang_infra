from __future__ import annotations

import numpy as np

from doubles_planner.fixed_relay import FixedRelayPlanner
from doubles_planner.models import BallPrediction, RobotFeedback
from doubles_planner.real_ros_runtime import (
    BALL_SCHEMA,
    ROBOT_STATE_SCHEMA,
    V9RealFixedRelayRuntime,
)


def _feedback(name: str, y: float, phase: str) -> RobotFeedback:
    return RobotFeedback(
        name=name,
        base_xy=np.asarray([0.0, y]),
        velocity_xy=np.zeros(2),
        timestamp=1.0,
        controller_phase=phase,
        ready=True,
    )


def _prediction() -> BallPrediction:
    return BallPrediction(
        position=np.asarray([0.45, 0.75, 1.0]),
        velocity=np.asarray([-3.0, 0.0, -0.5]),
        time_to_strike=0.5,
        timestamp=1.0,
        racket_normal=np.asarray([1.0, 0.0, 0.0]),
        racket_velocity=np.asarray([3.0, 0.0, 0.5]),
        shot_id="projected-shot",
    )


def _robot_state(robot: str, y: float, phase: str) -> dict:
    return {
        "schema_version": ROBOT_STATE_SCHEMA,
        "robot": robot,
        "sequence": 1,
        "source_monotonic_ns": 1_000_000_000,
        "q": np.zeros(29).tolist(),
        "dq": np.zeros(29).tolist(),
        "imu_quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        "gyro_xyz": [0.0, 0.0, 0.0],
        "base_position_xyz": [0.0, y, 0.75],
        "base_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "base_linear_velocity_xyz": [0.0, 0.0, 0.0],
        "base_angular_velocity_xyz": [0.0, 0.0, 0.0],
        "phase": phase,
        "ready": True,
        "state_elapsed_s": 3.0,
        "stable_elapsed_s": 1.0,
        "target_base_y": y,
        "home_y": y,
        "time_to_strike_s": -0.5,
        "valid": True,
        "emergency_stop": False,
    }


def _ball_payload() -> dict:
    prediction = _prediction()
    return {
        "schema_version": BALL_SCHEMA,
        "sequence": 1,
        "source_timestamp": 1.0,
        "position": [0.0, 0.75, 1.1],
        "velocity": prediction.velocity.tolist(),
        "predicted_strike_position": prediction.position.tolist(),
        "predicted_strike_velocity": prediction.velocity.tolist(),
        "time_to_strike_s": prediction.time_to_strike,
        "racket_normal": prediction.racket_normal.tolist(),
        "racket_velocity": prediction.racket_velocity.tolist(),
        "shot_id": prediction.shot_id,
        "valid": True,
    }


def test_fixed_relay_projects_against_real_outward_lane_without_fallback():
    planner = FixedRelayPlanner()
    right, left = planner.config.robots
    feedback = {
        right: _feedback(right, planner.config.home_y[0], "HOME_HOLD"),
        left: _feedback(left, planner.config.outward_y[1], "OUTWARD_HOLD"),
    }

    result = planner.plan(_prediction(), feedback, now=1.0)

    assert result.commit_requested
    assert "ordered_goal_projection" not in result.fallbacks
    assert result.diagnostics["ordered_goal_projected"] is True
    assert result.diagnostics["fixed_relay_lane_source"] == "outward_y"
    assert np.isclose(
        result.diagnostics["teammate_stage_y"],
        planner.config.outward_y[1],
    )
    gap = (
        result.commands[left].trajectory_base_position[1]
        - result.commands[right].trajectory_base_position[1]
    )
    assert gap >= planner.config.min_separation + planner.config.separation_margin - 1e-12


def test_fixed_relay_runtime_keeps_safe_projection_planned_valid():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    right, left = runtime.config.robots
    runtime.update_robot_state(
        right,
        _robot_state(right, runtime.config.home_y[0], "HOME_HOLD"),
        receive_monotonic=1.0,
    )
    runtime.update_robot_state(
        left,
        _robot_state(left, runtime.config.outward_y[1], "OUTWARD_HOLD"),
        receive_monotonic=1.0,
    )
    runtime.update_ball(_ball_payload(), receive_monotonic=1.0)

    outputs = runtime.tick(now=1.0)

    assert outputs[right]["planned_active"] is True
    assert outputs[right]["planned_valid"] is True
    assert outputs[right]["fallbacks"] == []
    assert outputs[right]["valid"] is False  # Shadow mode still suppresses output.
