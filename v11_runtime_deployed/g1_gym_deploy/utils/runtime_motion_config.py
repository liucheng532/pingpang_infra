"""Validated ROS payload for deferred V10 movement configuration."""

from dataclasses import asdict, dataclass
import json
import math


SCHEMA = "v10-runtime-motion-config-v1"
Y_ABS_LIMIT_M = 1.3
Y_DISPLACEMENT_MIN_M = 0.35
Y_DISPLACEMENT_MAX_M = 1.35
OUTWARD_HOLD_MIN_S = 0.0
OUTWARD_HOLD_MAX_S = 30.0


@dataclass(frozen=True)
class RuntimeMotionConfig:
    command_id: str
    published_at: float
    robot_id: str
    goal_x: float
    outward_y: float
    home_y: float
    outward_hold_s: float

    def to_dict(self):
        return asdict(self)


def parse_runtime_motion_config(raw, expected_robot_id="table_right"):
    if isinstance(raw, str):
        try:
            payload = json.loads(raw)
        except ValueError as exc:
            raise ValueError("runtime motion config is not valid JSON") from exc
    else:
        payload = raw
    if not isinstance(payload, dict):
        raise ValueError("runtime motion config must be a JSON object")
    if payload.get("schema") != SCHEMA:
        raise ValueError("runtime motion config has the wrong schema")
    command_id = payload.get("command_id")
    if not isinstance(command_id, str) or not command_id.strip():
        raise ValueError("runtime motion config command_id must be non-empty")
    robot_id = payload.get("robot_id")
    if robot_id != expected_robot_id:
        raise ValueError(
            "runtime motion config robot_id must be %s" % expected_robot_id
        )
    fields = {}
    for name in (
        "published_at",
        "goal_x",
        "outward_y",
        "home_y",
        "outward_hold_s",
    ):
        value = payload.get(name)
        if isinstance(value, bool):
            raise ValueError("%s must be finite" % name)
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError("%s must be finite" % name) from exc
        if not math.isfinite(value):
            raise ValueError("%s must be finite" % name)
        fields[name] = value
    for name in ("outward_y", "home_y"):
        if abs(fields[name]) > Y_ABS_LIMIT_M:
            raise ValueError("%s must be within +/- %.2f m" % (name, Y_ABS_LIMIT_M))
    displacement = abs(fields["outward_y"] - fields["home_y"])
    if not Y_DISPLACEMENT_MIN_M <= displacement <= Y_DISPLACEMENT_MAX_M:
        raise ValueError(
            "abs(outward_y-home_y) must be within [%.2f, %.2f] m"
            % (Y_DISPLACEMENT_MIN_M, Y_DISPLACEMENT_MAX_M)
        )
    if not OUTWARD_HOLD_MIN_S <= fields["outward_hold_s"] <= OUTWARD_HOLD_MAX_S:
        raise ValueError(
            "outward_hold_s must be within [%.1f, %.1f] s"
            % (OUTWARD_HOLD_MIN_S, OUTWARD_HOLD_MAX_S)
        )
    return RuntimeMotionConfig(
        command_id=command_id.strip(),
        robot_id=robot_id,
        **fields,
    )
