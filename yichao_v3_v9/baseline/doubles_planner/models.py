from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping

import numpy as np


class RelayStage(str, Enum):
    IDLE = "idle"
    RESERVED = "reserved"
    PREPARED = "prepared"
    COMMITTED = "committed"
    STRIKE = "strike"
    FOLLOW_THROUGH = "follow_through"
    HANDOFF = "handoff"
    ABORTED = "aborted"


def _vector(value: Any, size: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size != size:
        raise ValueError(f"{name} must contain {size} values")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain finite values")
    return array.copy()


@dataclass(frozen=True)
class RobotFeedback:
    name: str
    base_xy: np.ndarray
    velocity_xy: np.ndarray = field(default_factory=lambda: np.zeros(2, dtype=float))
    timestamp: float = 0.0
    valid: bool = True
    controller_phase: str | int | None = None
    ready: bool | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "base_xy", _vector(self.base_xy, 2, "base_xy"))
        object.__setattr__(self, "velocity_xy", _vector(self.velocity_xy, 2, "velocity_xy"))
        if not np.isfinite(self.timestamp):
            raise ValueError("feedback timestamp must be finite")
        if self.controller_phase is not None:
            if isinstance(self.controller_phase, (int, np.integer)) and not isinstance(
                self.controller_phase, (bool, np.bool_)
            ):
                phase_names = (
                    "HIT",
                    "POST_DELAY",
                    "OUTWARD",
                    "OUTWARD_HOLD",
                    "RETURN",
                    "HOME_HOLD",
                )
                phase_index = int(self.controller_phase)
                if not 0 <= phase_index < len(phase_names):
                    raise ValueError(f"unknown controller phase: {self.controller_phase!r}")
                phase = phase_names[phase_index]
            else:
                phase = str(self.controller_phase).upper()
            if phase not in {
                "HIT",
                "POST_DELAY",
                "OUTWARD",
                "OUTWARD_HOLD",
                "RETURN",
                "HOME_HOLD",
            }:
                raise ValueError(f"unknown controller phase: {self.controller_phase!r}")
            object.__setattr__(self, "controller_phase", phase)
        if self.ready is not None and not isinstance(self.ready, (bool, np.bool_)):
            raise TypeError("ready must be a bool or None")
        if self.ready is not None:
            object.__setattr__(self, "ready", bool(self.ready))


@dataclass(frozen=True)
class BallPrediction:
    position: np.ndarray
    velocity: np.ndarray
    time_to_strike: float
    timestamp: float
    racket_normal: np.ndarray
    racket_velocity: np.ndarray
    shot_id: str | int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "position", _vector(self.position, 3, "position"))
        object.__setattr__(self, "velocity", _vector(self.velocity, 3, "velocity"))
        object.__setattr__(self, "racket_normal", _vector(self.racket_normal, 3, "racket_normal"))
        object.__setattr__(self, "racket_velocity", _vector(self.racket_velocity, 3, "racket_velocity"))
        if not np.isfinite(self.time_to_strike):
            raise ValueError("time_to_strike must be finite")
        if not np.isfinite(self.timestamp):
            raise ValueError("prediction timestamp must be finite")


@dataclass(frozen=True)
class RobotCommand:
    robot: str
    active: bool
    predicted_ball_position: np.ndarray
    predicted_ball_velocity: np.ndarray
    predicted_ball_predict_time: float
    predicted_racket_normal: np.ndarray
    predicted_racket_velocity: np.ndarray
    desired_base_position: np.ndarray
    trajectory_base_position: np.ndarray | None = None
    role: str = "hold"

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "predicted_ball_position",
            _vector(self.predicted_ball_position, 3, "predicted_ball_position"),
        )
        object.__setattr__(
            self,
            "predicted_ball_velocity",
            _vector(self.predicted_ball_velocity, 3, "predicted_ball_velocity"),
        )
        object.__setattr__(
            self,
            "predicted_racket_normal",
            _vector(self.predicted_racket_normal, 3, "predicted_racket_normal"),
        )
        object.__setattr__(
            self,
            "predicted_racket_velocity",
            _vector(self.predicted_racket_velocity, 3, "predicted_racket_velocity"),
        )
        object.__setattr__(
            self,
            "desired_base_position",
            _vector(self.desired_base_position, 2, "desired_base_position"),
        )
        trajectory_base_position = self.trajectory_base_position
        if trajectory_base_position is None:
            trajectory_base_position = self.desired_base_position
        object.__setattr__(
            self,
            "trajectory_base_position",
            _vector(trajectory_base_position, 2, "trajectory_base_position"),
        )
        if not np.isfinite(self.predicted_ball_predict_time):
            raise ValueError("predicted_ball_predict_time must be finite")
        if self.role not in {
            "hold",
            "reserved_hitter",
            "hit",
            "clear",
            "exit",
            "stage",
            "return",
        }:
            raise ValueError(f"unknown robot command role: {self.role!r}")

    def to_dict(self) -> dict[str, Any]:
        return {
            "robot": self.robot,
            "active": self.active,
            "predicted_ball_position": self.predicted_ball_position.tolist(),
            "predicted_ball_velocity": self.predicted_ball_velocity.tolist(),
            "predicted_ball_predict_time": self.predicted_ball_predict_time,
            "predicted_racket_normal": self.predicted_racket_normal.tolist(),
            "predicted_racket_velocity": self.predicted_racket_velocity.tolist(),
            "desired_base_position": self.desired_base_position.tolist(),
            "trajectory_base_position": self.trajectory_base_position.tolist(),
            "role": self.role,
        }


@dataclass(frozen=True)
class PlanResult:
    sequence: int
    shot_id: str | int | None
    hitter: str | None
    next_hitter: str | None
    phase: str
    strategy: str
    commands: Mapping[str, RobotCommand]
    fallbacks: tuple[str, ...] = ()
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
    relay_stage: str = RelayStage.IDLE.value
    commit_token: str | None = None
    commit_requested: bool = False
    pending_shot_id: str | int | None = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "shot_id": self.shot_id,
            "hitter": self.hitter,
            "next_hitter": self.next_hitter,
            "phase": self.phase,
            "strategy": self.strategy,
            "fallbacks": list(self.fallbacks),
            "diagnostics": dict(self.diagnostics),
            "relay_stage": self.relay_stage,
            "commit_token": self.commit_token,
            "commit_requested": self.commit_requested,
            "pending_shot_id": self.pending_shot_id,
            "commands": {name: command.to_dict() for name, command in self.commands.items()},
        }
