from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math
from typing import Any, Mapping

from .config import PlannerConfig


RUNTIME_MOTION_CONFIG_SCHEMA = "v10-runtime-motion-config-v1"
RUNTIME_MOTION_STATUS_SCHEMA = "v10-doubles-runtime-motion-config-status-v1"
Y_ABS_LIMIT_M = 1.30
Y_DISPLACEMENT_MIN_M = 0.35
Y_DISPLACEMENT_MAX_M = 1.35
OUTWARD_HOLD_MIN_S = 0.0
OUTWARD_HOLD_MAX_S = 30.0


def _finite(payload: Mapping[str, Any], name: str) -> float:
    value = payload.get(name)
    if isinstance(value, bool):
        raise ValueError(f"{name} must be finite")
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be finite") from error
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


@dataclass(frozen=True)
class RuntimeMotionConfig:
    robot_id: str
    command_id: str | None
    published_at: float | None
    goal_x: float | None
    outward_y: float
    home_y: float
    outward_hold_s: float

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    def command_dict(self) -> dict[str, Any] | None:
        if self.command_id is None or self.goal_x is None or self.published_at is None:
            return None
        return {
            "schema": RUNTIME_MOTION_CONFIG_SCHEMA,
            **self.to_dict(),
        }


def default_runtime_motion_configs(config: PlannerConfig) -> dict[str, RuntimeMotionConfig]:
    return {
        robot: RuntimeMotionConfig(
            robot_id=robot,
            command_id=None,
            published_at=None,
            goal_x=None,
            outward_y=float(config.outward_y[index]),
            home_y=float(config.home_y[index]),
            outward_hold_s=3.0,
        )
        for index, robot in enumerate(config.robots)
    }


def parse_runtime_motion_config(
    payload: Mapping[str, Any],
    *,
    expected_robots: tuple[str, ...],
) -> RuntimeMotionConfig:
    if not isinstance(payload, Mapping):
        raise ValueError("runtime motion config must be a JSON object")
    if payload.get("schema") != RUNTIME_MOTION_CONFIG_SCHEMA:
        raise ValueError("runtime motion config has the wrong schema")
    robot_id = payload.get("robot_id")
    if robot_id not in expected_robots:
        raise ValueError("runtime motion config has an unknown robot_id")
    command_id = payload.get("command_id")
    if not isinstance(command_id, str) or not command_id.strip():
        raise ValueError("runtime motion config command_id must be non-empty")
    values = {
        name: _finite(payload, name)
        for name in ("published_at", "goal_x", "outward_y", "home_y", "outward_hold_s")
    }
    for name in ("outward_y", "home_y"):
        if abs(values[name]) > Y_ABS_LIMIT_M:
            raise ValueError(f"{name} must be within +/- {Y_ABS_LIMIT_M:.2f} m")
    displacement = abs(values["outward_y"] - values["home_y"])
    if not Y_DISPLACEMENT_MIN_M <= displacement <= Y_DISPLACEMENT_MAX_M:
        raise ValueError(
            "abs(outward_y-home_y) must be within "
            f"[{Y_DISPLACEMENT_MIN_M:.2f}, {Y_DISPLACEMENT_MAX_M:.2f}] m"
        )
    if not OUTWARD_HOLD_MIN_S <= values["outward_hold_s"] <= OUTWARD_HOLD_MAX_S:
        raise ValueError(
            "outward_hold_s must be within "
            f"[{OUTWARD_HOLD_MIN_S:.1f}, {OUTWARD_HOLD_MAX_S:.1f}] s"
        )
    return RuntimeMotionConfig(
        robot_id=str(robot_id),
        command_id=command_id.strip(),
        **values,
    )


def validate_combined_motion_config(
    planner_config: PlannerConfig,
    configs: Mapping[str, RuntimeMotionConfig],
) -> PlannerConfig:
    home_y = tuple(float(configs[name].home_y) for name in planner_config.robots)
    outward_y = tuple(float(configs[name].outward_y) for name in planner_config.robots)
    return replace(
        planner_config,
        home_y=home_y,
        ready_y=home_y,
        outward_y=outward_y,
    )


__all__ = [
    "RUNTIME_MOTION_CONFIG_SCHEMA",
    "RUNTIME_MOTION_STATUS_SCHEMA",
    "RuntimeMotionConfig",
    "default_runtime_motion_configs",
    "parse_runtime_motion_config",
    "validate_combined_motion_config",
]
