"""Centralized high-level ABI for IsaacSim doubles relay training.

The module is deliberately independent of Isaac imports so it can be used by
the Isaac task adapter without importing the simulator during package import.
It only defines the structured joint observation, two-head action contract,
and the deterministic commit gate.  The actual rollout and reward execution
live in :mod:`doubles_planner.isaac_rl`.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field, replace
from typing import Any, Sequence

import numpy as np

from .config import PlannerConfig
from .metrics import MetricThresholds, pair_kinematics


RELAY_STAGE_NAMES = ("RESERVED", "PREPARED", "COMMITTED", "STRIKE", "FOLLOW_THROUGH", "HANDOFF")
CONTROLLER_PHASE_NAMES_V2 = ("HIT", "POST_DELAY", "OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD")
SKILL_NAMES = ("HOLD", "PREPARE", "CLEAR", "HIT", "RETURN")
ACTION_VERSION = "doubles-centralized-v2"


def _finite_array(value: Any, width: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32).reshape(-1)
    if result.size != width or not np.isfinite(result).all():
        raise ValueError(f"{name} must contain {width} finite values")
    return result.copy()


def _optional_array(value: Any, width: int, name: str) -> np.ndarray | None:
    return None if value is None else _finite_array(value, width, name)


def _variable_array(value: Any, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32).reshape(-1)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must contain finite values")
    return result.copy()


def _scalar(value: float, name: str, *, nonnegative: bool = False) -> float:
    result = float(value)
    if not math.isfinite(result) or (nonnegative and result < 0.0):
        raise ValueError(f"{name} must be finite" + (" and non-negative" if nonnegative else ""))
    return result


def _phase(value: str | int) -> str:
    if isinstance(value, str):
        value = value.strip().upper().replace("-", "_")
        if value in CONTROLLER_PHASE_NAMES_V2:
            return value
        raise ValueError(f"unknown controller phase: {value!r}")
    try:
        return CONTROLLER_PHASE_NAMES_V2[int(value)]
    except (IndexError, TypeError, ValueError) as error:
        raise ValueError(f"unknown controller phase: {value!r}") from error


def _stage(value: str | int) -> str:
    if isinstance(value, str):
        value = value.strip().upper().replace("-", "_")
        aliases = {"FOLLOWTHROUGH": "FOLLOW_THROUGH", "STRIKE_FOLLOW_THROUGH": "STRIKE"}
        value = aliases.get(value, value)
        if value in RELAY_STAGE_NAMES:
            return value
        raise ValueError(f"unknown relay stage: {value!r}")
    try:
        return RELAY_STAGE_NAMES[int(value)]
    except (IndexError, TypeError, ValueError) as error:
        raise ValueError(f"unknown relay stage: {value!r}") from error


def _skill(value: str | int) -> str:
    if isinstance(value, str):
        value = value.strip().upper().replace("-", "_")
        if value in SKILL_NAMES:
            return value
        raise ValueError(f"unknown skill: {value!r}")
    try:
        return SKILL_NAMES[int(value)]
    except (IndexError, TypeError, ValueError) as error:
        raise ValueError(f"unknown skill: {value!r}") from error


def _skill_index(value: str | int) -> int:
    return SKILL_NAMES.index(_skill(value))


def _clip(value: float, scale: float, limit: float = 1.0) -> float:
    return float(np.clip(float(value) / max(float(scale), 1.0e-6), -limit, limit))


@dataclass(frozen=True)
class DoublesObservationConfig:
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    joint_position_size: int = 29
    joint_velocity_size: int = 29
    imu_size: int = 6
    contact_size: int = 4
    maximum_phase_s: float = 2.0
    maximum_ttc_s: float = 2.0
    maximum_prediction_age_s: float = 0.30
    maximum_feedback_age_s: float = 0.25
    minimum_commit_tts_s: float = 0.12
    maximum_commit_tts_s: float = 0.55
    maximum_velocity_mps: float = 3.0

    def __post_init__(self) -> None:
        for name in ("joint_position_size", "joint_velocity_size", "imu_size", "contact_size"):
            value = getattr(self, name)
            if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or value < 0:
                raise ValueError(f"{name} must be a non-negative integer")
        for name in (
            "maximum_phase_s", "maximum_ttc_s", "maximum_prediction_age_s",
            "maximum_feedback_age_s", "minimum_commit_tts_s", "maximum_commit_tts_s",
            "maximum_velocity_mps",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.maximum_commit_tts_s <= self.minimum_commit_tts_s:
            raise ValueError("commit timing window must be increasing")


@dataclass(frozen=True)
class RobotPolicyState:
    name: str
    base_position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    base_orientation_wxyz: np.ndarray = field(default_factory=lambda: np.asarray([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
    base_linear_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    base_angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    joint_position: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    joint_velocity: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    imu: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    contact: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    controller_phase: str | int = "HOME_HOLD"
    state_elapsed_s: float = 0.0
    stable_elapsed_s: float = 0.0
    target_base_y: float | None = None
    time_to_strike_s: float | None = None
    ready: bool = True
    command_valid: bool = True
    timestamp: float = 0.0
    hand_position: np.ndarray | None = None
    hand_velocity: np.ndarray | None = None
    racket_position: np.ndarray | None = None
    racket_velocity: np.ndarray | None = None

    def __post_init__(self) -> None:
        if not str(self.name).strip():
            raise ValueError("robot name must not be empty")
        object.__setattr__(self, "name", str(self.name))
        for name in ("base_position", "base_linear_velocity", "base_angular_velocity"):
            object.__setattr__(self, name, _finite_array(getattr(self, name), 3, name))
        object.__setattr__(self, "base_orientation_wxyz", _finite_array(self.base_orientation_wxyz, 4, "base_orientation_wxyz"))
        for name in ("joint_position", "joint_velocity", "imu", "contact"):
            object.__setattr__(self, name, _variable_array(getattr(self, name), name))
        for name in ("hand_position", "hand_velocity", "racket_position", "racket_velocity"):
            object.__setattr__(self, name, _optional_array(getattr(self, name), 3, name))
        object.__setattr__(self, "controller_phase", _phase(self.controller_phase))
        object.__setattr__(self, "state_elapsed_s", _scalar(self.state_elapsed_s, "state_elapsed_s", nonnegative=True))
        object.__setattr__(self, "stable_elapsed_s", _scalar(self.stable_elapsed_s, "stable_elapsed_s", nonnegative=True))
        if self.target_base_y is not None:
            object.__setattr__(self, "target_base_y", _scalar(self.target_base_y, "target_base_y"))
        if self.time_to_strike_s is not None:
            object.__setattr__(self, "time_to_strike_s", _scalar(self.time_to_strike_s, "time_to_strike_s"))
        object.__setattr__(self, "timestamp", _scalar(self.timestamp, "timestamp"))
        object.__setattr__(self, "ready", bool(self.ready))
        object.__setattr__(self, "command_valid", bool(self.command_valid))

    @property
    def base_y(self) -> float:
        return float(self.base_position[1])

    def fresh(self, now: float, timeout_s: float) -> bool:
        age = float(now) - self.timestamp
        return bool(self.command_valid and -0.05 <= age <= timeout_s)


@dataclass(frozen=True)
class BallPolicyState:
    position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    predicted_strike_position: np.ndarray | None = None
    predicted_strike_velocity: np.ndarray | None = None
    time_to_strike_s: float = 0.0
    confidence: float = 1.0
    prediction_age_s: float = 0.0
    valid: bool = True
    timestamp: float = 0.0
    shot_id: str | int | None = None

    def __post_init__(self) -> None:
        for name in ("position", "velocity", "acceleration"):
            object.__setattr__(self, name, _finite_array(getattr(self, name), 3, name))
        object.__setattr__(self, "predicted_strike_position", _finite_array(self.position if self.predicted_strike_position is None else self.predicted_strike_position, 3, "predicted_strike_position"))
        object.__setattr__(self, "predicted_strike_velocity", _finite_array(self.velocity if self.predicted_strike_velocity is None else self.predicted_strike_velocity, 3, "predicted_strike_velocity"))
        object.__setattr__(self, "time_to_strike_s", _scalar(self.time_to_strike_s, "time_to_strike_s"))
        confidence = _scalar(self.confidence, "confidence")
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be in [0, 1]")
        object.__setattr__(self, "confidence", confidence)
        object.__setattr__(self, "prediction_age_s", _scalar(self.prediction_age_s, "prediction_age_s", nonnegative=True))
        object.__setattr__(self, "timestamp", _scalar(self.timestamp, "timestamp"))
        object.__setattr__(self, "valid", bool(self.valid))


@dataclass(frozen=True)
class DoublesPolicyObservation:
    robots: tuple[RobotPolicyState, RobotPolicyState]
    ball: BallPolicyState
    relay_stage: str | int = "RESERVED"
    timestamp: float = 0.0
    next_hitter_index: int | None = 0
    active_hitter_index: int | None = None
    last_hitter_index: int | None = None
    reservation_age_s: float = 0.0
    commit_elapsed_s: float = 0.0
    physical_side: int = 0
    previous_base_target_y: tuple[float, float] | None = None
    previous_skill: tuple[str | int, str | int] = ("HOLD", "HOLD")
    emergency_active: bool = False

    def __post_init__(self) -> None:
        if len(self.robots) != 2 or self.robots[0].name == self.robots[1].name:
            raise ValueError("robots must contain two uniquely named states")
        object.__setattr__(self, "relay_stage", _stage(self.relay_stage))
        object.__setattr__(self, "timestamp", _scalar(self.timestamp, "timestamp"))
        for name in ("reservation_age_s", "commit_elapsed_s"):
            object.__setattr__(self, name, _scalar(getattr(self, name), name, nonnegative=True))
        for name in ("next_hitter_index", "active_hitter_index", "last_hitter_index"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)) or int(value) not in (0, 1)):
                raise ValueError(f"{name} must be None, 0, or 1")
            if value is not None:
                object.__setattr__(self, name, int(value))
        if self.physical_side not in (-1, 0, 1):
            raise ValueError("physical_side must be -1, 0, or 1")
        previous = self.previous_base_target_y
        if previous is None:
            previous = (self.robots[0].base_y, self.robots[1].base_y)
        if len(previous) != 2:
            raise ValueError("previous_base_target_y must contain two values")
        object.__setattr__(self, "previous_base_target_y", tuple(_scalar(v, "previous_base_target_y") for v in previous))
        if len(self.previous_skill) != 2:
            raise ValueError("previous_skill must contain two values")
        object.__setattr__(self, "previous_skill", tuple(_skill(v) for v in self.previous_skill))
        object.__setattr__(self, "emergency_active", bool(self.emergency_active))

    @property
    def expected_hitter_index(self) -> int | None:
        if self.next_hitter_index is not None:
            return self.next_hitter_index
        if self.active_hitter_index is not None:
            return self.active_hitter_index
        return None if self.last_hitter_index is None else 1 - self.last_hitter_index

    def feedback_valid(self, config: DoublesObservationConfig | None = None) -> bool:
        config = config or DoublesObservationConfig()
        return all(robot.fresh(self.timestamp, config.maximum_feedback_age_s) for robot in self.robots)

    def prediction_fresh(self, config: DoublesObservationConfig | None = None) -> bool:
        config = config or DoublesObservationConfig()
        age = self.timestamp - self.ball.timestamp
        return bool(self.ball.valid and 0.0 <= self.ball.prediction_age_s <= config.maximum_prediction_age_s and -0.05 <= age <= config.maximum_prediction_age_s)

    def safety_snapshot(self, config: DoublesObservationConfig | None = None) -> dict[str, float | bool | None]:
        config = config or DoublesObservationConfig()
        left, right = self.robots
        base = pair_kinematics(left.base_position, left.base_linear_velocity, right.base_position, right.base_linear_velocity, config.planner.min_separation)

        def pair_optional(position_name: str, velocity_name: str, threshold: float) -> tuple[float, float, float | None, bool]:
            first, second = getattr(left, position_name), getattr(right, position_name)
            if first is None or second is None:
                return config.planner.workspace_y[1] - config.planner.workspace_y[0], 0.0, None, False
            first_v = getattr(left, velocity_name)
            second_v = getattr(right, velocity_name)
            if first_v is None:
                first_v = np.zeros(3, dtype=np.float32)
            if second_v is None:
                second_v = np.zeros(3, dtype=np.float32)
            result = pair_kinematics(first, first_v, second, second_v, threshold)
            return result.distance_m, result.closing_speed_mps, result.time_to_threshold_s, True

        hand = pair_optional("hand_position", "hand_velocity", config.planner.collision_distance)
        racket = pair_optional("racket_position", "racket_velocity", config.planner.collision_distance)
        return {
            "base_distance_m": base.distance_m, "base_clearance_m": base.clearance_m,
            "base_closing_speed_mps": base.closing_speed_mps, "base_ttc_s": base.time_to_threshold_s,
            "hand_distance_m": hand[0], "hand_closing_speed_mps": hand[1], "hand_ttc_s": hand[2], "hand_valid": hand[3],
            "racket_distance_m": racket[0], "racket_closing_speed_mps": racket[1], "racket_ttc_s": racket[2], "racket_valid": racket[3],
            "simultaneous_hit_risk": bool(left.controller_phase in {"HIT", "POST_DELAY"} and right.controller_phase in {"HIT", "POST_DELAY"}),
        }

    def legal_skill_mask(self, config: DoublesObservationConfig | None = None) -> np.ndarray:
        config = config or DoublesObservationConfig()
        mask = np.zeros((2, len(SKILL_NAMES)), dtype=np.bool_)
        mask[:, _skill_index("HOLD")] = True
        fresh = self.feedback_valid(config) and self.prediction_fresh(config)
        safety = self.safety_snapshot(config)
        for index, robot in enumerate(self.robots):
            if robot.controller_phase not in {"HIT", "POST_DELAY"}:
                mask[index, [_skill_index(v) for v in ("PREPARE", "CLEAR", "RETURN")]] = fresh
            if (
                self.relay_stage == "PREPARED" and index == self.expected_hitter_index and fresh
                and config.minimum_commit_tts_s <= self.ball.time_to_strike_s <= config.maximum_commit_tts_s
                and all(state.ready and state.controller_phase == "HOME_HOLD" for state in self.robots)
                and float(safety["base_distance_m"]) >= config.planner.min_separation
                and (safety["base_ttc_s"] is None or float(safety["base_ttc_s"]) >= config.planner.commit_minimum_base_ttc)
                and not self.emergency_active
            ):
                mask[index, _skill_index("HIT")] = True
        return mask

    def vector_v2(self, config: DoublesObservationConfig | None = None) -> np.ndarray:
        config = config or DoublesObservationConfig()
        workspace = max(abs(config.planner.workspace_y[0]), abs(config.planner.workspace_y[1]), 1.0e-6)
        values: list[float] = []
        add = lambda value: values.append(float(value))
        values.extend(float(self.relay_stage == name) for name in RELAY_STAGE_NAMES)
        add(_clip(self.reservation_age_s, config.maximum_phase_s)); add(_clip(self.commit_elapsed_s, config.maximum_phase_s))
        values.extend(float(self.next_hitter_index == i) for i in range(2)); values.extend(float(self.active_hitter_index == i) for i in range(2)); add(float(self.physical_side))
        values.extend(_clip(v, workspace) for v in self.ball.position)
        values.extend(_clip(v, config.maximum_velocity_mps) for v in self.ball.velocity)
        values.extend(_clip(v, config.maximum_velocity_mps) for v in self.ball.acceleration)
        values.extend(_clip(v, workspace) for v in self.ball.predicted_strike_position)
        values.extend(_clip(v, config.maximum_velocity_mps) for v in self.ball.predicted_strike_velocity)
        add(_clip(self.ball.time_to_strike_s, config.maximum_commit_tts_s)); add(self.ball.confidence); add(_clip(self.ball.prediction_age_s, config.maximum_prediction_age_s)); add(float(self.prediction_fresh(config)))

        def padded(array: np.ndarray, width: int) -> list[float]:
            if array.size > width:
                raise ValueError(f"proprioception width {array.size} exceeds schema width {width}")
            result = np.zeros(width, dtype=np.float32); result[:array.size] = array
            return result.tolist()

        for index, robot in enumerate(self.robots):
            values.extend(_clip(v, workspace) for v in robot.base_position)
            values.extend(_clip(v, config.maximum_velocity_mps) for v in robot.base_linear_velocity)
            values.extend(_clip(v, config.maximum_velocity_mps) for v in robot.base_angular_velocity)
            values.extend(float(np.clip(v, -1.0, 1.0)) for v in robot.base_orientation_wxyz)
            values.extend(padded(robot.joint_position, config.joint_position_size)); values.extend(padded(robot.joint_velocity, config.joint_velocity_size)); values.extend(padded(robot.imu, config.imu_size)); values.extend(padded(robot.contact, config.contact_size))
            values.extend(float(robot.controller_phase == phase) for phase in CONTROLLER_PHASE_NAMES_V2)
            add(_clip(robot.state_elapsed_s, config.maximum_phase_s)); add(_clip(robot.stable_elapsed_s, config.maximum_phase_s)); add(_clip(0.0 if robot.target_base_y is None else robot.target_base_y - robot.base_y, workspace)); add(float(robot.target_base_y is not None)); add(_clip(0.0 if robot.time_to_strike_s is None else robot.time_to_strike_s, config.maximum_commit_tts_s)); add(float(robot.time_to_strike_s is not None)); add(float(robot.ready)); add(float(robot.command_valid)); add(float(robot.fresh(self.timestamp, config.maximum_feedback_age_s)))
            for point in (robot.hand_position, robot.racket_position):
                values.extend((0.0, 0.0, 0.0) if point is None else (_clip(v, workspace) for v in point)); add(float(point is not None))
            values.extend((float(self.next_hitter_index == index), float(self.active_hitter_index == index)))

        left, right = self.robots
        values.extend(_clip(v, workspace) for v in right.base_position - left.base_position); values.extend(_clip(v, config.maximum_velocity_mps) for v in right.base_linear_velocity - left.base_linear_velocity)
        safety = self.safety_snapshot(config)
        for name, scale in (("base_distance_m", workspace), ("base_closing_speed_mps", config.maximum_velocity_mps), ("hand_distance_m", workspace), ("hand_closing_speed_mps", config.maximum_velocity_mps), ("racket_distance_m", workspace), ("racket_closing_speed_mps", config.maximum_velocity_mps)):
            add(_clip(float(safety[name]), scale, 1.5))
        for name in ("base_ttc_s", "hand_ttc_s", "racket_ttc_s"):
            value = safety[name]; add(_clip(config.maximum_ttc_s if value is None else float(value), config.maximum_ttc_s)); add(float(value is not None))
        values.extend((_clip(float(safety["base_clearance_m"]), workspace, 1.5), float(safety["hand_valid"]), float(safety["racket_valid"]), float(safety["simultaneous_hit_risk"]), float(self.emergency_active)))
        values.extend(_clip(v, workspace) for v in self.previous_base_target_y)
        for skill in self.previous_skill:
            values.extend(float(_skill_index(skill) == i) for i in range(len(SKILL_NAMES)))
        add(_clip(self.reservation_age_s + self.commit_elapsed_s, config.maximum_phase_s))
        values.extend(float(v) for v in self.legal_skill_mask(config).reshape(-1))
        result = np.asarray(values, dtype=np.float32)
        expected = len(observation_names_v2(config))
        if result.size != expected or not np.isfinite(result).all():
            raise RuntimeError(f"centralized observation schema mismatch/non-finite: {result.size} != {expected}")
        return result


def observation_names_v2(config: DoublesObservationConfig | None = None) -> tuple[str, ...]:
    config = config or DoublesObservationConfig()
    names: list[str] = []
    names.extend(f"stage_{v.lower()}" for v in RELAY_STAGE_NAMES); names.extend(("reservation_age", "commit_elapsed", "next_hitter_left", "next_hitter_right", "active_hitter_left", "active_hitter_right", "physical_side"))
    names.extend(f"ball_{v}" for v in ("position_x", "position_y", "position_z", "velocity_x", "velocity_y", "velocity_z", "acceleration_x", "acceleration_y", "acceleration_z", "predicted_strike_position_x", "predicted_strike_position_y", "predicted_strike_position_z", "predicted_strike_velocity_x", "predicted_strike_velocity_y", "predicted_strike_velocity_z")); names.extend(("ball_time_to_strike", "ball_confidence", "ball_prediction_age", "ball_valid"))
    for robot in ("left", "right"):
        prefix = f"{robot}_"
        names.extend(prefix + v for v in ("base_position_x", "base_position_y", "base_position_z", "base_linear_velocity_x", "base_linear_velocity_y", "base_linear_velocity_z", "base_angular_velocity_x", "base_angular_velocity_y", "base_angular_velocity_z")); names.extend(prefix + f"base_orientation_{v}" for v in ("w", "x", "y", "z")); names.extend(prefix + f"joint_position_{i}" for i in range(config.joint_position_size)); names.extend(prefix + f"joint_velocity_{i}" for i in range(config.joint_velocity_size)); names.extend(prefix + f"imu_{i}" for i in range(config.imu_size)); names.extend(prefix + f"contact_{i}" for i in range(config.contact_size)); names.extend(prefix + f"phase_{v.lower()}" for v in CONTROLLER_PHASE_NAMES_V2)
        names.extend(prefix + v for v in ("state_elapsed", "stable_elapsed", "target_base_error_y", "target_base_valid", "time_to_strike", "time_to_strike_valid", "ready", "command_valid", "feedback_fresh")); names.extend(prefix + f"{point}_{axis}" for point in ("hand_position", "racket_position") for axis in ("x", "y", "z")); names.extend(prefix + v for v in ("hand_valid", "racket_valid", "role_next_hitter", "role_active_hitter"))
    names.extend(("relative_base_x", "relative_base_y", "relative_base_z", "relative_velocity_x", "relative_velocity_y", "relative_velocity_z")); names.extend(("base_distance", "base_closing_speed", "hand_distance", "hand_closing_speed", "racket_distance", "racket_closing_speed")); names.extend(("base_ttc", "base_ttc_valid", "hand_ttc", "hand_ttc_valid", "racket_ttc", "racket_ttc_valid", "base_clearance", "hand_valid", "racket_valid", "simultaneous_hit_risk", "emergency_active")); names.extend(("previous_base_target_left", "previous_base_target_right")); names.extend(f"previous_{skill.lower()}_{robot}" for robot in ("left", "right") for skill in SKILL_NAMES); names.append("decision_age"); names.extend(f"{robot}_skill_{skill.lower()}_legal" for robot in ("left", "right") for skill in SKILL_NAMES)
    return tuple(names)


OBSERVATION_NAMES_V2 = observation_names_v2()
CENTRALIZED_OBSERVATION_SCHEMA_HASH = hashlib.sha256(json.dumps(OBSERVATION_NAMES_V2, separators=(",", ":")).encode()).hexdigest()


def centralized_observation_schema(config: DoublesObservationConfig | None = None) -> dict[str, Any]:
    names = observation_names_v2(config); digest = hashlib.sha256(json.dumps(names, separators=(",", ":")).encode()).hexdigest()
    return {"version": ACTION_VERSION, "observation_size": len(names), "observation_names": list(names), "schema_hash": digest}


@dataclass(frozen=True)
class HitRequest:
    target_position: np.ndarray
    target_velocity: np.ndarray
    time_to_strike_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_position", _finite_array(self.target_position, 3, "target_position")); object.__setattr__(self, "target_velocity", _finite_array(self.target_velocity, 3, "target_velocity")); object.__setattr__(self, "time_to_strike_s", _scalar(self.time_to_strike_s, "time_to_strike_s"))


@dataclass(frozen=True)
class DoublesRobotCommand:
    robot: str
    skill: str
    base_target_y: float
    command_valid: bool = True
    hit_request: HitRequest | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "skill", _skill(self.skill)); object.__setattr__(self, "base_target_y", _scalar(self.base_target_y, "base_target_y")); object.__setattr__(self, "command_valid", bool(self.command_valid))
        if self.skill == "HIT" and self.hit_request is None:
            raise ValueError("HIT command requires hit_request")
        if self.skill != "HIT" and self.hit_request is not None:
            raise ValueError("only HIT may carry hit_request")


@dataclass(frozen=True)
class DoublesCommandBatch:
    commands: tuple[DoublesRobotCommand, DoublesRobotCommand]
    accepted_hitter_index: int | None
    commit_requested: bool
    simultaneous_hit: bool = False
    stale_input: bool = False
    safety_projected: bool = False
    reasons: tuple[str, ...] = ()
    commit_token: str | None = None

    def __post_init__(self) -> None:
        if len(self.commands) != 2 or self.commit_requested != (self.accepted_hitter_index is not None):
            raise ValueError("command batch must contain two commands and a consistent commit flag")

    @property
    def by_robot(self) -> dict[str, DoublesRobotCommand]:
        return {command.robot: command for command in self.commands}


@dataclass(frozen=True)
class DoublesCommandConfig:
    observation: DoublesObservationConfig = field(default_factory=DoublesObservationConfig)
    thresholds: MetricThresholds = field(default_factory=MetricThresholds)
    minimum_commit_tts_s: float = 0.12
    maximum_commit_tts_s: float = 0.55

    def __post_init__(self) -> None:
        if not all(math.isfinite(v) for v in (self.minimum_commit_tts_s, self.maximum_commit_tts_s)) or self.minimum_commit_tts_s < 0.0 or self.maximum_commit_tts_s <= self.minimum_commit_tts_s:
            raise ValueError("commit timing window must be finite, increasing, and non-negative")


class DoublesCommandAction:
    values_per_robot = 1 + len(SKILL_NAMES)
    num_actions = 2 * values_per_robot

    def __init__(self, base_target_y: Sequence[float], skill_logits: np.ndarray) -> None:
        if len(base_target_y) != 2 or not np.isfinite(np.asarray(base_target_y, dtype=float)).all():
            raise ValueError("base_target_y must contain two finite values")
        logits = np.asarray(skill_logits, dtype=np.float32)
        if logits.shape != (2, len(SKILL_NAMES)) or not np.isfinite(logits).all():
            raise ValueError("skill_logits must have shape (2, 5) and be finite")
        self.base_target_y = (float(base_target_y[0]), float(base_target_y[1])); self.skill_logits = logits.copy()

    @classmethod
    def from_array(cls, raw_action: Sequence[float] | np.ndarray, config: DoublesCommandConfig | None = None) -> "DoublesCommandAction":
        config = config or DoublesCommandConfig(); values = np.asarray(raw_action, dtype=np.float32).reshape(-1)
        if values.size != cls.num_actions or not np.isfinite(values).all():
            raise ValueError(f"raw centralized action must contain {cls.num_actions} finite values")
        low, high = config.observation.planner.workspace_y; midpoint = 0.5 * (low + high); half = 0.5 * (high - low); targets = []; logits = np.zeros((2, len(SKILL_NAMES)), dtype=np.float32)
        for index in range(2):
            offset = index * cls.values_per_robot; targets.append(midpoint + half * float(np.clip(values[offset], -1.0, 1.0))); logits[index] = values[offset + 1:offset + cls.values_per_robot]
        return cls(targets, logits)

    def preferred_skills(self) -> tuple[str, str]:
        return tuple(SKILL_NAMES[int(np.argmax(row))] for row in self.skill_logits)  # type: ignore[return-value]


def _hold_batch(observation: DoublesPolicyObservation, *, valid: bool, reasons: Sequence[str], stale: bool = False) -> DoublesCommandBatch:
    commands = tuple(DoublesRobotCommand(robot=robot.name, skill="HOLD", base_target_y=robot.base_y, command_valid=valid) for robot in observation.robots)
    return DoublesCommandBatch(commands=(commands[0], commands[1]), accepted_hitter_index=None, commit_requested=False, stale_input=stale, reasons=tuple(reasons))


def _project_targets(observation: DoublesPolicyObservation, targets: list[float], protected: tuple[bool, bool], config: DoublesCommandConfig) -> tuple[list[float], bool]:
    low, high = config.observation.planner.workspace_y; targets = [float(np.clip(v, low, high)) for v in targets]; minimum = max(config.observation.planner.min_separation, config.thresholds.base_distance_m)
    if targets[1] - targets[0] >= minimum:
        return targets, False
    result = list(targets)
    if protected[0] and not protected[1]: result[1] = min(high, max(result[1], result[0] + minimum))
    elif protected[1] and not protected[0]: result[0] = max(low, min(result[0], result[1] - minimum))
    else:
        center = 0.5 * (result[0] + result[1]); result = [center - 0.5 * minimum, center + 0.5 * minimum]
        if result[0] < low: result = [low, low + minimum]
        if result[1] > high: result = [high - minimum, high]
    return [float(np.clip(v, low, high)) for v in result], True


def guard_doubles_action(raw_action: Sequence[float] | np.ndarray, observation: DoublesPolicyObservation, config: DoublesCommandConfig | None = None, *, strict: bool = False) -> DoublesCommandBatch:
    config = config or DoublesCommandConfig()
    try:
        action = DoublesCommandAction.from_array(raw_action, config)
    except (TypeError, ValueError):
        if strict: raise
        return _hold_batch(observation, valid=False, reasons=("invalid_action",))
    if observation.emergency_active:
        return _hold_batch(observation, valid=True, reasons=("emergency_active",))
    obs_config = replace(config.observation, minimum_commit_tts_s=config.minimum_commit_tts_s, maximum_commit_tts_s=config.maximum_commit_tts_s)
    if not observation.feedback_valid(obs_config) or not observation.prediction_fresh(obs_config):
        return _hold_batch(observation, valid=False, reasons=("stale_input",), stale=True)
    mask = observation.legal_skill_mask(obs_config); preferred = action.preferred_skills(); candidates = [i for i, skill in enumerate(preferred) if skill == "HIT"]; reasons: list[str] = []; accepted: int | None = None
    if len(candidates) > 1: reasons.append("simultaneous_hit_rejected")
    elif candidates:
        candidate = candidates[0]
        if candidate == observation.expected_hitter_index and mask[candidate, _skill_index("HIT")]: accepted = candidate
        else: reasons.append("hit_not_legal")
    targets = [action.base_target_y[0], action.base_target_y[1]]; protected = tuple(robot.controller_phase in {"HIT", "POST_DELAY"} for robot in observation.robots)
    for i, robot in enumerate(observation.robots):
        if protected[i]: targets[i] = robot.target_base_y if robot.target_base_y is not None else robot.base_y
    targets, projected = _project_targets(observation, targets, protected, config)
    if projected: reasons.append("base_separation_projected")
    if accepted is not None:
        peer = 1 - accepted; targets[peer] = config.observation.planner.outward_y[peer]; targets, projected_again = _project_targets(observation, targets, (True, False), config); projected = projected or projected_again
        commands = tuple(DoublesRobotCommand(robot=observation.robots[i].name, skill=("HIT" if i == accepted else "CLEAR"), base_target_y=targets[i], hit_request=(HitRequest(observation.ball.predicted_strike_position, observation.ball.predicted_strike_velocity, observation.ball.time_to_strike_s) if i == accepted else None)) for i in range(2))
        token = None if observation.ball.shot_id is None else str(observation.ball.shot_id); return DoublesCommandBatch(commands=commands, accepted_hitter_index=accepted, commit_requested=True, safety_projected=projected, reasons=tuple(reasons + ["commit_accepted"]), commit_token=token)
    commands = []
    for i, robot in enumerate(observation.robots):
        skill = preferred[i]
        if skill == "HIT" or not mask[i, _skill_index(skill)]:
            if skill == "HIT": reasons.append(f"robot_{i}_hit_masked")
            skill = "HOLD"
        commands.append(DoublesRobotCommand(robot=robot.name, skill=skill, base_target_y=targets[i]))
    return DoublesCommandBatch(commands=(commands[0], commands[1]), accepted_hitter_index=None, commit_requested=False, simultaneous_hit=len(candidates) > 1, safety_projected=projected, reasons=tuple(dict.fromkeys(reasons)))


class CentralizedRelaySupervisor:
    """Edge-triggered latch that keeps role ownership outside PPO."""

    def __init__(self, config: DoublesCommandConfig | None = None) -> None:
        self.config = config or DoublesCommandConfig(); self._latched_hitter: int | None = None; self._latched_shot_id: str | int | None = None; self._token_counter = 0; self._last_stage: str | None = None

    @property
    def latched_hitter(self) -> int | None:
        return self._latched_hitter

    def reset(self) -> None:
        self._latched_hitter = None; self._latched_shot_id = None; self._token_counter = 0; self._last_stage = None

    def step(self, observation: DoublesPolicyObservation, raw_action: Sequence[float] | np.ndarray) -> DoublesCommandBatch:
        shot_id = observation.ball.shot_id; previous_stage = self._last_stage; self._last_stage = observation.relay_stage
        if self._latched_hitter is not None:
            new_cycle = observation.relay_stage in {"RESERVED", "PREPARED"} and previous_stage not in {None, "RESERVED", "PREPARED"}
            if observation.relay_stage in {"RESERVED", "PREPARED"} and (shot_id != self._latched_shot_id or new_cycle):
                self._latched_hitter = None; self._latched_shot_id = None
            elif observation.relay_stage in {"COMMITTED", "STRIKE", "FOLLOW_THROUGH"}:
                batch = guard_doubles_action(raw_action, observation, self.config); commands = list(batch.commands)
                for i, command in enumerate(commands):
                    if i == self._latched_hitter: commands[i] = DoublesRobotCommand(command.robot, "HOLD", command.base_target_y, command.command_valid)
                    elif command.skill == "HIT": commands[i] = DoublesRobotCommand(command.robot, "CLEAR", self.config.observation.planner.outward_y[i], command.command_valid)
                return DoublesCommandBatch(commands=(commands[0], commands[1]), accepted_hitter_index=None, commit_requested=False, simultaneous_hit=batch.simultaneous_hit, stale_input=batch.stale_input, safety_projected=batch.safety_projected, reasons=tuple(dict.fromkeys(batch.reasons + ("commit_already_latched",))), commit_token=str(self._latched_shot_id) if self._latched_shot_id is not None else None)
        batch = guard_doubles_action(raw_action, observation, self.config)
        if batch.accepted_hitter_index is None: return batch
        self._latched_hitter = batch.accepted_hitter_index; self._latched_shot_id = shot_id; self._token_counter += 1; token = batch.commit_token or f"centralized-{self._token_counter}"
        return DoublesCommandBatch(commands=batch.commands, accepted_hitter_index=batch.accepted_hitter_index, commit_requested=True, simultaneous_hit=batch.simultaneous_hit, stale_input=batch.stale_input, safety_projected=batch.safety_projected, reasons=batch.reasons, commit_token=token)


__all__ = ["ACTION_VERSION", "CENTRALIZED_OBSERVATION_SCHEMA_HASH", "CONTROLLER_PHASE_NAMES_V2", "DoublesCommandAction", "DoublesCommandBatch", "DoublesCommandConfig", "DoublesObservationConfig", "DoublesPolicyObservation", "DoublesRobotCommand", "CentralizedRelaySupervisor", "HitRequest", "OBSERVATION_NAMES_V2", "RELAY_STAGE_NAMES", "SKILL_NAMES", "RobotPolicyState", "BallPolicyState", "centralized_observation_schema", "guard_doubles_action", "observation_names_v2"]
