"""Strict Unitree G1 input contract for real-robot planner execution.

The planner never reads an Isaac object directly.  A transport adapter (DDS,
ROS 2, or a recorder replay) converts its messages into the records below and
keeps the source timestamps intact.  Missing required records are invalid and
must result in a hold command.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np

from .centralized import (
    BallPolicyState,
    DoublesObservationConfig,
    DoublesPolicyObservation,
    RobotPolicyState,
)
from .models import BallPrediction, RobotFeedback

if TYPE_CHECKING:
    from .core import RelayPlanner
    from .models import PlanResult


G1_DOF = 29
G1_IMU_SIZE = 6
CONTROLLER_PHASE_NAMES = (
    "HIT",
    "POST_DELAY",
    "OUTWARD",
    "OUTWARD_HOLD",
    "RETURN",
    "HOME_HOLD",
)
REAL_INPUT_CONTRACT_VERSION = "unitree-g1-real-v1"


def _array(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32).reshape(-1)
    if result.size != size or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return result.copy()


def _optional_array(value: Any, size: int, name: str) -> np.ndarray | None:
    return None if value is None else _array(value, size, name)


def _timestamp(value: float, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be boolean")
    return bool(value)


def _phase(value: str | int) -> str:
    if isinstance(value, str):
        normalized = value.strip().upper().replace("-", "_")
        if normalized in CONTROLLER_PHASE_NAMES:
            return normalized
        raise ValueError(f"unknown controller phase: {value!r}")
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(f"unknown controller phase: {value!r}")
    index = int(value)
    if not 0 <= index < len(CONTROLLER_PHASE_NAMES):
        raise ValueError(f"unknown controller phase: {value!r}")
    return CONTROLLER_PHASE_NAMES[index]


def _fresh(timestamp: float, now: float, timeout: float) -> bool:
    age = float(now) - float(timestamp)
    return bool(-0.05 <= age <= timeout)


def _gravity_from_quaternion(quaternion_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion_wxyz
    norm = float(np.linalg.norm(quaternion_wxyz))
    if norm <= 1.0e-6:
        raise ValueError("IMU quaternion norm must be positive")
    w, x, y, z = quaternion_wxyz / norm
    return np.asarray(
        [
            2.0 * (x * z - w * y),
            2.0 * (w * x + y * z),
            2.0 * (w * w + z * z) - 1.0,
        ],
        dtype=np.float32,
    )


@dataclass(frozen=True)
class UnitreeG1LowState:
    """Fields available from one Unitree G1 low-level state sample."""

    q: np.ndarray
    dq: np.ndarray
    imu_quaternion_wxyz: np.ndarray
    gyro_xyz: np.ndarray
    timestamp: float
    valid: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "q", _array(self.q, G1_DOF, "q"))
        object.__setattr__(self, "dq", _array(self.dq, G1_DOF, "dq"))
        object.__setattr__(
            self,
            "imu_quaternion_wxyz",
            _array(self.imu_quaternion_wxyz, 4, "imu_quaternion_wxyz"),
        )
        object.__setattr__(self, "gyro_xyz", _array(self.gyro_xyz, 3, "gyro_xyz"))
        object.__setattr__(self, "timestamp", _timestamp(self.timestamp, "timestamp"))
        object.__setattr__(self, "valid", _boolean(self.valid, "valid"))

    @property
    def imu_vector(self) -> np.ndarray:
        return np.concatenate(
            (self.gyro_xyz, _gravity_from_quaternion(self.imu_quaternion_wxyz))
        ).astype(np.float32)


@dataclass(frozen=True)
class G1BaseEstimate:
    """World-frame pelvis pose/velocity from mocap, VIO, or stance odometry."""

    base_position_xyz: np.ndarray
    base_orientation_wxyz: np.ndarray
    base_linear_velocity_xyz: np.ndarray
    base_angular_velocity_xyz: np.ndarray
    timestamp: float
    source: str
    valid: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "base_position_xyz", _array(self.base_position_xyz, 3, "base_position_xyz"))
        object.__setattr__(self, "base_orientation_wxyz", _array(self.base_orientation_wxyz, 4, "base_orientation_wxyz"))
        object.__setattr__(self, "base_linear_velocity_xyz", _array(self.base_linear_velocity_xyz, 3, "base_linear_velocity_xyz"))
        object.__setattr__(self, "base_angular_velocity_xyz", _array(self.base_angular_velocity_xyz, 3, "base_angular_velocity_xyz"))
        object.__setattr__(self, "timestamp", _timestamp(self.timestamp, "timestamp"))
        if not str(self.source).strip():
            raise ValueError("base estimate source must not be empty")
        object.__setattr__(self, "source", str(self.source))
        object.__setattr__(self, "valid", _boolean(self.valid, "valid"))

    @property
    def base_xy(self) -> np.ndarray:
        return self.base_position_xyz[:2].copy()

    @property
    def velocity_xy(self) -> np.ndarray:
        return self.base_linear_velocity_xyz[:2].copy()


@dataclass(frozen=True)
class G1ControllerTelemetry:
    """Phase/readiness state exported by the active G1 controller."""

    phase: str | int
    ready: bool
    timestamp: float
    state_elapsed_s: float = 0.0
    stable_elapsed_s: float = 0.0
    target_base_y: float | None = None
    time_to_strike_s: float | None = None
    valid: bool = True

    def __post_init__(self) -> None:
        object.__setattr__(self, "phase", _phase(self.phase))
        object.__setattr__(self, "ready", _boolean(self.ready, "ready"))
        object.__setattr__(self, "timestamp", _timestamp(self.timestamp, "timestamp"))
        for name in ("state_elapsed_s", "stable_elapsed_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
            object.__setattr__(self, name, value)
        for name in ("target_base_y", "time_to_strike_s"):
            value = getattr(self, name)
            if value is not None:
                object.__setattr__(self, name, _timestamp(value, name))
        object.__setattr__(self, "valid", _boolean(self.valid, "valid"))


@dataclass(frozen=True)
class G1RobotInput:
    """One synchronized G1 sample assembled by the real transport adapter."""

    name: str
    low_state: UnitreeG1LowState
    base: G1BaseEstimate
    controller: G1ControllerTelemetry
    hand_position_xyz: np.ndarray | None = None
    hand_velocity_xyz: np.ndarray | None = None
    racket_position_xyz: np.ndarray | None = None
    racket_velocity_xyz: np.ndarray | None = None
    contact: np.ndarray | None = None
    geometry_timestamp: float | None = None

    def __post_init__(self) -> None:
        if not str(self.name).strip():
            raise ValueError("robot name must not be empty")
        object.__setattr__(self, "name", str(self.name))
        for name in (
            "hand_position_xyz",
            "hand_velocity_xyz",
            "racket_position_xyz",
            "racket_velocity_xyz",
        ):
            object.__setattr__(self, name, _optional_array(getattr(self, name), 3, name))
        if self.contact is not None:
            contact = np.asarray(self.contact, dtype=np.float32).reshape(-1)
            if contact.size == 0 or not np.isfinite(contact).all():
                raise ValueError("contact must contain finite values")
            object.__setattr__(self, "contact", contact.copy())
        if self.geometry_timestamp is not None:
            object.__setattr__(self, "geometry_timestamp", _timestamp(self.geometry_timestamp, "geometry_timestamp"))

    @property
    def sample_timestamp(self) -> float:
        timestamps = [self.low_state.timestamp, self.base.timestamp, self.controller.timestamp]
        if self.geometry_timestamp is not None:
            timestamps.append(self.geometry_timestamp)
        return float(min(timestamps))

    @property
    def geometry_complete(self) -> bool:
        return all(
            value is not None
            for value in (
                self.hand_position_xyz,
                self.hand_velocity_xyz,
                self.racket_position_xyz,
                self.racket_velocity_xyz,
                self.geometry_timestamp,
            )
        )

    def fresh(
        self,
        now: float,
        timeout_s: float,
        *,
        require_geometry: bool = False,
        contact_size: int | None = None,
    ) -> bool:
        required = (
            self.low_state.valid
            and self.base.valid
            and self.controller.valid
            and _fresh(self.low_state.timestamp, now, timeout_s)
            and _fresh(self.base.timestamp, now, timeout_s)
            and _fresh(self.controller.timestamp, now, timeout_s)
        )
        if not required:
            return False
        if require_geometry:
            return bool(
                self.geometry_complete
                and self.contact is not None
                and (contact_size is None or self.contact.size == contact_size)
                and _fresh(float(self.geometry_timestamp), now, timeout_s)
            )
        return True

    def to_feedback(self, now: float, timeout_s: float) -> RobotFeedback:
        valid = self.fresh(now, timeout_s)
        return RobotFeedback(
            name=self.name,
            base_xy=self.base.base_xy,
            velocity_xy=self.base.velocity_xy,
            timestamp=self.sample_timestamp,
            valid=valid,
            controller_phase=self.controller.phase,
            ready=self.controller.ready,
        )

    def to_policy_state(
        self,
        now: float,
        timeout_s: float,
        config: DoublesObservationConfig | None = None,
        *,
        require_geometry: bool = True,
    ) -> RobotPolicyState:
        config = config or DoublesObservationConfig()
        valid = self.fresh(
            now,
            timeout_s,
            require_geometry=require_geometry,
            contact_size=config.contact_size if require_geometry else None,
        )
        contact = np.zeros(config.contact_size, dtype=np.float32)
        if self.contact is not None and self.contact.size == config.contact_size:
            contact = self.contact.copy()
        hand_position = self.hand_position_xyz if self.geometry_complete else None
        hand_velocity = self.hand_velocity_xyz if self.geometry_complete else None
        racket_position = self.racket_position_xyz if self.geometry_complete else None
        racket_velocity = self.racket_velocity_xyz if self.geometry_complete else None
        return RobotPolicyState(
            name=self.name,
            base_position=self.base.base_position_xyz,
            base_orientation_wxyz=self.base.base_orientation_wxyz,
            base_linear_velocity=self.base.base_linear_velocity_xyz,
            base_angular_velocity=self.base.base_angular_velocity_xyz,
            joint_position=self.low_state.q,
            joint_velocity=self.low_state.dq,
            imu=self.low_state.imu_vector,
            contact=contact,
            controller_phase=self.controller.phase,
            state_elapsed_s=self.controller.state_elapsed_s,
            stable_elapsed_s=self.controller.stable_elapsed_s,
            target_base_y=self.controller.target_base_y,
            time_to_strike_s=self.controller.time_to_strike_s,
            ready=self.controller.ready,
            command_valid=valid,
            timestamp=self.sample_timestamp,
            hand_position=hand_position,
            hand_velocity=hand_velocity,
            racket_position=racket_position,
            racket_velocity=racket_velocity,
        )


@dataclass(frozen=True)
class RealBallPrediction:
    """Timestamped ball/paddle prediction aggregate available to the planner."""

    position: np.ndarray
    velocity: np.ndarray
    time_to_strike_s: float
    racket_normal: np.ndarray
    racket_velocity: np.ndarray
    timestamp: float
    acceleration: np.ndarray | None = None
    predicted_strike_position: np.ndarray | None = None
    predicted_strike_velocity: np.ndarray | None = None
    confidence: float | None = None
    prediction_age_s: float | None = None
    shot_id: str | int | None = None
    valid: bool = True

    def __post_init__(self) -> None:
        for name in ("position", "velocity", "racket_normal", "racket_velocity"):
            object.__setattr__(self, name, _array(getattr(self, name), 3, name))
        for name in (
            "acceleration",
            "predicted_strike_position",
            "predicted_strike_velocity",
        ):
            object.__setattr__(self, name, _optional_array(getattr(self, name), 3, name))
        object.__setattr__(self, "time_to_strike_s", _timestamp(self.time_to_strike_s, "time_to_strike_s"))
        object.__setattr__(self, "timestamp", _timestamp(self.timestamp, "timestamp"))
        if self.confidence is not None:
            confidence = float(self.confidence)
            if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                raise ValueError("confidence must be finite and in [0, 1]")
            object.__setattr__(self, "confidence", confidence)
        if self.prediction_age_s is not None:
            age = float(self.prediction_age_s)
            if not math.isfinite(age) or age < 0.0:
                raise ValueError("prediction_age_s must be finite and non-negative")
            object.__setattr__(self, "prediction_age_s", age)
        object.__setattr__(self, "valid", _boolean(self.valid, "valid"))

    @property
    def traditional_fields_complete(self) -> bool:
        return bool(
            self.predicted_strike_position is not None
            and self.predicted_strike_velocity is not None
        )

    @property
    def centralized_fields_complete(self) -> bool:
        return bool(
            self.traditional_fields_complete
            and self.acceleration is not None
            and self.confidence is not None
            and self.prediction_age_s is not None
        )

    def traditional_fresh(self, now: float, timeout_s: float) -> bool:
        prediction_age_fresh = (
            self.prediction_age_s is None
            or float(self.prediction_age_s) <= timeout_s
        )
        return bool(
            self.valid
            and self.traditional_fields_complete
            and prediction_age_fresh
            and _fresh(self.timestamp, now, timeout_s)
        )

    def fresh(self, now: float, timeout_s: float) -> bool:
        return bool(
            self.valid
            and self.centralized_fields_complete
            and float(self.prediction_age_s) <= timeout_s
            and _fresh(self.timestamp, now, timeout_s)
        )

    def to_prediction(self, now: float, timeout_s: float) -> BallPrediction | None:
        if not self.traditional_fresh(now, timeout_s):
            return None
        return BallPrediction(
            position=self.predicted_strike_position,
            velocity=self.predicted_strike_velocity,
            time_to_strike=self.time_to_strike_s,
            timestamp=self.timestamp,
            racket_normal=self.racket_normal,
            racket_velocity=self.racket_velocity,
            shot_id=self.shot_id,
        )

    def to_policy_state(self, *, valid: bool | None = None) -> BallPolicyState:
        state_valid = self.valid if valid is None else self.valid and bool(valid)
        state_valid = bool(state_valid and self.centralized_fields_complete)
        return BallPolicyState(
            position=self.position,
            velocity=self.velocity,
            acceleration=np.zeros(3, dtype=np.float32) if self.acceleration is None else self.acceleration,
            predicted_strike_position=(
                np.zeros(3, dtype=np.float32)
                if self.predicted_strike_position is None
                else self.predicted_strike_position
            ),
            predicted_strike_velocity=(
                np.zeros(3, dtype=np.float32)
                if self.predicted_strike_velocity is None
                else self.predicted_strike_velocity
            ),
            time_to_strike_s=self.time_to_strike_s,
            confidence=0.0 if self.confidence is None else self.confidence,
            prediction_age_s=0.0 if self.prediction_age_s is None else self.prediction_age_s,
            valid=state_valid,
            timestamp=self.timestamp,
            shot_id=self.shot_id,
        )


@dataclass(frozen=True)
class RealInputAdapter:
    """Build traditional or centralized planner inputs from two real G1s."""

    robots: tuple[G1RobotInput, G1RobotInput]
    ball: RealBallPrediction | None
    feedback_timeout_s: float = 0.25
    prediction_timeout_s: float = 0.30

    def __post_init__(self) -> None:
        if len(self.robots) != 2 or self.robots[0].name == self.robots[1].name:
            raise ValueError("robots must contain two uniquely named G1 inputs")
        for name in ("feedback_timeout_s", "prediction_timeout_s"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
            object.__setattr__(self, name, value)

    def traditional_inputs(
        self, now: float
    ) -> tuple[BallPrediction | None, Mapping[str, RobotFeedback], tuple[str, ...]]:
        feedback = {
            robot.name: robot.to_feedback(now, self.feedback_timeout_s)
            for robot in self.robots
        }
        reasons = [
            f"{robot.name}_feedback_invalid"
            for robot in self.robots
            if not robot.fresh(now, self.feedback_timeout_s)
        ]
        prediction = None if self.ball is None else self.ball.to_prediction(now, self.prediction_timeout_s)
        if self.ball is None or prediction is None:
            reasons.append("ball_prediction_invalid")
        return prediction, feedback, tuple(reasons)

    def plan(self, planner: "RelayPlanner", now: float) -> "PlanResult":
        prediction, feedback, reasons = self.traditional_inputs(now)
        if reasons:
            return planner.safe_hold(
                feedback,
                now,
                reason="real_input_invalid:" + ",".join(reasons),
            )
        return planner.plan(prediction, feedback, now)

    def centralized_observation(
        self,
        now: float,
        *,
        relay_stage: str | int = "RESERVED",
        next_hitter_index: int | None = 0,
        active_hitter_index: int | None = None,
        last_hitter_index: int | None = None,
        config: DoublesObservationConfig | None = None,
    ) -> tuple[DoublesPolicyObservation, tuple[str, ...]]:
        config = config or DoublesObservationConfig()
        reasons: list[str] = []
        states = []
        for robot in self.robots:
            if not robot.fresh(
                now,
                self.feedback_timeout_s,
                require_geometry=True,
                contact_size=config.contact_size,
            ):
                reasons.append(f"{robot.name}_centralized_geometry_or_feedback_invalid")
            states.append(robot.to_policy_state(now, self.feedback_timeout_s, config, require_geometry=True))
        if self.ball is None or not self.ball.fresh(now, self.prediction_timeout_s):
            reasons.append("ball_prediction_invalid")
            ball = BallPolicyState(
                position=np.zeros(3, dtype=np.float32),
                velocity=np.zeros(3, dtype=np.float32),
                acceleration=np.zeros(3, dtype=np.float32),
                predicted_strike_position=np.zeros(3, dtype=np.float32),
                predicted_strike_velocity=np.zeros(3, dtype=np.float32),
                time_to_strike_s=0.0,
                confidence=0.0,
                prediction_age_s=0.0,
                timestamp=now,
                valid=False,
            )
        else:
            ball = self.ball.to_policy_state(valid=True)
        observation = DoublesPolicyObservation(
            robots=(states[0], states[1]),
            ball=ball,
            relay_stage=relay_stage,
            timestamp=now,
            next_hitter_index=next_hitter_index,
            active_hitter_index=active_hitter_index,
            last_hitter_index=last_hitter_index,
        )
        return observation, tuple(reasons)


__all__ = [
    "CONTROLLER_PHASE_NAMES",
    "G1BaseEstimate",
    "G1ControllerTelemetry",
    "G1_DOF",
    "G1_IMU_SIZE",
    "G1RobotInput",
    "REAL_INPUT_CONTRACT_VERSION",
    "RealBallPrediction",
    "RealInputAdapter",
    "UnitreeG1LowState",
]
