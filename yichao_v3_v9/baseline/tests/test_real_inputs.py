from __future__ import annotations

import numpy as np
import pytest

from doubles_planner import DoublesObservationConfig, PlannerConfig, RelayPlanner
from doubles_planner.real_inputs import (
    G1BaseEstimate,
    G1ControllerTelemetry,
    G1RobotInput,
    RealBallPrediction,
    RealInputAdapter,
    UnitreeG1LowState,
)


def _robot(
    name: str,
    y: float,
    *,
    timestamp: float = 1.0,
    geometry: bool = False,
    contact_size: int = 4,
) -> G1RobotInput:
    low = UnitreeG1LowState(
        q=np.zeros(29),
        dq=np.zeros(29),
        imu_quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
        gyro_xyz=np.zeros(3),
        timestamp=timestamp,
    )
    base = G1BaseEstimate(
        base_position_xyz=[0.0, y, 0.8],
        base_orientation_wxyz=[1.0, 0.0, 0.0, 0.0],
        base_linear_velocity_xyz=np.zeros(3),
        base_angular_velocity_xyz=np.zeros(3),
        timestamp=timestamp,
        source="stance_odometry",
    )
    controller = G1ControllerTelemetry(
        phase="HOME_HOLD",
        ready=True,
        timestamp=timestamp,
    )
    values = {
        "name": name,
        "low_state": low,
        "base": base,
        "controller": controller,
    }
    if geometry:
        values.update(
            {
                "hand_position_xyz": [0.0, y, 1.0],
                "hand_velocity_xyz": np.zeros(3),
                "racket_position_xyz": [0.0, y, 1.0],
                "racket_velocity_xyz": np.zeros(3),
                "contact": np.ones(contact_size),
                "geometry_timestamp": timestamp,
            }
        )
    return G1RobotInput(**values)


def _ball(
    *, timestamp: float = 1.0, valid: bool = True, **overrides: object
) -> RealBallPrediction:
    values: dict[str, object] = {
        "position": [0.4, 0.0, 1.0],
        "velocity": [-2.0, 0.2, 0.0],
        "acceleration": [0.0, 0.0, 0.0],
        "predicted_strike_position": [0.4, 0.0, 1.0],
        "predicted_strike_velocity": [-2.0, 0.2, 0.0],
        "time_to_strike_s": 0.4,
        "racket_normal": [1.0, 0.0, 0.0],
        "racket_velocity": [-1.0, 0.0, 0.0],
        "timestamp": timestamp,
        "confidence": 0.9,
        "prediction_age_s": 0.02,
        "valid": valid,
    }
    values.update(overrides)
    return RealBallPrediction(**values)


def test_traditional_adapter_provides_real_inputs_to_all_strategies() -> None:
    adapter = RealInputAdapter((_robot("left", -0.35), _robot("right", 0.35)), _ball())
    prediction, feedback, reasons = adapter.traditional_inputs(1.0)

    assert prediction is not None
    assert not reasons
    assert all(state.valid for state in feedback.values())
    assert all(state.controller_phase == "HOME_HOLD" for state in feedback.values())

    strike_position = np.asarray([0.4, -0.2, 1.0])
    strike_velocity = np.asarray([-2.0, -0.1, 0.0])
    adapter_ball = RealBallPrediction(
        position=[0.2, 0.1, 1.1],
        velocity=[-1.5, 0.3, 0.0],
        acceleration=[0.0, 0.0, 0.0],
        predicted_strike_position=strike_position,
        predicted_strike_velocity=strike_velocity,
        time_to_strike_s=0.4,
        racket_normal=[1.0, 0.0, 0.0],
        racket_velocity=[-1.0, 0.0, 0.0],
        timestamp=1.0,
        confidence=0.9,
        prediction_age_s=0.02,
    )
    strike_prediction, _, _ = RealInputAdapter(adapter.robots, adapter_ball).traditional_inputs(1.0)
    assert strike_prediction is not None
    np.testing.assert_allclose(strike_prediction.position, strike_position)
    np.testing.assert_allclose(strike_prediction.velocity, strike_velocity)

    for strategy in ("heuristic", "cbf", "reachability", "rl"):
        planner = RelayPlanner(PlannerConfig(initial_hitter="left"), strategy=strategy)
        result = adapter.plan(planner, 1.0)
        assert result.strategy == planner.strategy.name
        assert not any(reason.startswith("real_input_invalid:") for reason in result.fallbacks)
        assert set(result.commands) == {"left", "right"}


def test_missing_real_sample_forces_safe_hold() -> None:
    adapter = RealInputAdapter(
        (_robot("left", -0.35, timestamp=0.0), _robot("right", 0.35, timestamp=1.0)),
        _ball(),
    )
    result = adapter.plan(RelayPlanner(PlannerConfig()), 1.0)

    assert result.phase == "safe_hold"
    assert any("real_input_invalid" in reason for reason in result.fallbacks)
    assert all(not command.active for command in result.commands.values())


def test_invalid_ball_prediction_forces_safe_hold() -> None:
    adapter = RealInputAdapter(
        (_robot("left", -0.35), _robot("right", 0.35)),
        _ball(valid=False),
    )
    result = adapter.plan(RelayPlanner(PlannerConfig()), 1.0)

    assert result.phase == "safe_hold"
    assert any("ball_prediction_invalid" in reason for reason in result.fallbacks)
    assert all(not command.active for command in result.commands.values())


def test_traditional_planner_rejects_explicitly_stale_prediction_age() -> None:
    adapter = RealInputAdapter(
        (_robot("left", -0.35), _robot("right", 0.35)),
        _ball(prediction_age_s=0.31),
    )

    prediction, _, reasons = adapter.traditional_inputs(1.0)

    assert prediction is None
    assert "ball_prediction_invalid" in reasons


def test_centralized_adapter_rejects_unavailable_geometry() -> None:
    adapter = RealInputAdapter((_robot("left", -0.35), _robot("right", 0.35)), _ball())
    observation, reasons = adapter.centralized_observation(1.0)

    assert reasons
    assert not observation.feedback_valid()
    assert not observation.robots[0].command_valid
    assert not observation.robots[1].command_valid


def test_centralized_adapter_accepts_explicit_real_geometry() -> None:
    adapter = RealInputAdapter(
        (_robot("left", -0.35, geometry=True), _robot("right", 0.35, geometry=True)),
        _ball(),
    )
    observation, reasons = adapter.centralized_observation(1.0)

    assert not reasons
    assert observation.feedback_valid()
    assert observation.prediction_fresh()
    assert observation.vector_v2().shape == (290,)


@pytest.mark.parametrize("missing", ("predicted_strike_position", "predicted_strike_velocity"))
def test_traditional_planner_requires_explicit_strike_forecast(missing: str) -> None:
    ball = _ball(**{missing: None})
    adapter = RealInputAdapter((_robot("left", -0.35), _robot("right", 0.35)), ball)

    prediction, _, reasons = adapter.traditional_inputs(1.0)

    assert prediction is None
    assert "ball_prediction_invalid" in reasons


@pytest.mark.parametrize("missing", ("acceleration", "confidence", "prediction_age_s"))
def test_centralized_observation_requires_explicit_ball_metadata(missing: str) -> None:
    ball = _ball(**{missing: None})
    adapter = RealInputAdapter(
        (_robot("left", -0.35, geometry=True), _robot("right", 0.35, geometry=True)),
        ball,
    )

    observation, reasons = adapter.centralized_observation(1.0)

    assert "ball_prediction_invalid" in reasons
    assert not observation.ball.valid
    assert not observation.prediction_fresh()


@pytest.mark.parametrize("contact_size", (3, 5))
def test_centralized_observation_rejects_contact_width_mismatch(contact_size: int) -> None:
    config = DoublesObservationConfig(contact_size=4)
    adapter = RealInputAdapter(
        (
            _robot("left", -0.35, geometry=True, contact_size=contact_size),
            _robot("right", 0.35, geometry=True, contact_size=contact_size),
        ),
        _ball(),
    )

    observation, reasons = adapter.centralized_observation(1.0, config=config)

    assert len(reasons) == 2
    assert all(not robot.command_valid for robot in observation.robots)
    assert all(robot.contact.shape == (config.contact_size,) for robot in observation.robots)
    assert all(not np.any(robot.contact) for robot in observation.robots)


def test_centralized_observation_uses_exact_configured_contact_width() -> None:
    config = DoublesObservationConfig(contact_size=2)
    adapter = RealInputAdapter(
        (_robot("left", -0.35, geometry=True, contact_size=2), _robot("right", 0.35, geometry=True, contact_size=2)),
        _ball(),
    )

    observation, reasons = adapter.centralized_observation(1.0, config=config)

    assert not reasons
    assert all(robot.command_valid for robot in observation.robots)
    assert all(robot.contact.shape == (config.contact_size,) for robot in observation.robots)


@pytest.mark.parametrize("phase", (-1, True, False, 1.0, 1.5, np.float32(2.0)))
def test_phase_rejects_noncanonical_numeric_values(phase: object) -> None:
    with pytest.raises(ValueError, match="unknown controller phase"):
        G1ControllerTelemetry(phase=phase, ready=True, timestamp=1.0)


def test_phase_accepts_integer_index_without_coercion() -> None:
    telemetry = G1ControllerTelemetry(phase=np.int64(1), ready=True, timestamp=1.0)

    assert telemetry.phase == "POST_DELAY"


@pytest.mark.parametrize("field", ("ready", "valid"))
def test_controller_boolean_fields_reject_truthy_non_booleans(field: str) -> None:
    values: dict[str, object] = {
        "phase": "HOME_HOLD",
        "ready": True,
        "timestamp": 1.0,
        "valid": True,
    }
    values[field] = "false"

    with pytest.raises(ValueError, match=f"{field} must be boolean"):
        G1ControllerTelemetry(**values)
