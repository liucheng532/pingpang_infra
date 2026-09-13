"""Exact V6R10 36D actor and 87D audit feature construction."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import copy
import math
from typing import Any, Mapping

import numpy as np

from . import HOME_PAIR, MODEL_ORDER, MODEL_TO_ROS, ROS_TO_MODEL, WORKSPACE_Y

HISTORY_LENGTH = 5
HISTORY_DT_S = 0.02
FIRST_INTERVAL_S = 1.65
INTERVAL_RANGE_S = (1.5, 1.8)

ACTOR_NAMES = tuple(
    f"ball_history_{history}_relative_{robot}_{axis}"
    for history in range(HISTORY_LENGTH)
    for robot in MODEL_ORDER
    for axis in "xyz"
) + (
    "left_base_x",
    "left_base_y",
    "right_base_x",
    "right_base_y",
    "previous_target_left_y",
    "previous_target_right_y",
)

AUDIT_NAMES = tuple(
    f"{robot}_base_history_{history}_{axis}"
    for robot in MODEL_ORDER
    for history in range(HISTORY_LENGTH)
    for axis in "xy"
) + tuple(
    f"{robot}_joint_position_{joint}" for robot in MODEL_ORDER for joint in range(29)
) + (
    "strike_target_position_x",
    "strike_target_position_y",
    "strike_target_position_z",
    "strike_target_racket_velocity_x",
    "strike_target_racket_velocity_y",
    "strike_target_racket_velocity_z",
    "strike_time_to_strike",
    "scheduled_hitter_sign",
    "relay_interval_normalized",
)

if len(ACTOR_NAMES) != 36 or len(AUDIT_NAMES) != 87:  # pragma: no cover
    raise RuntimeError("frozen feature schema dimensions changed")


def finite_array(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{label}: expected finite {shape}")
    return result


class JointNormalizer:
    """Isaac Lab soft-limit normalization frozen from the designated G1 URDF."""

    JOINT_NAMES = (
        "left_hip_pitch_joint", "right_hip_pitch_joint", "waist_yaw_joint",
        "left_hip_roll_joint", "right_hip_roll_joint", "waist_roll_joint",
        "left_hip_yaw_joint", "right_hip_yaw_joint", "waist_pitch_joint",
        "left_knee_joint", "right_knee_joint", "left_shoulder_pitch_joint",
        "right_shoulder_pitch_joint", "left_ankle_pitch_joint",
        "right_ankle_pitch_joint", "left_shoulder_roll_joint",
        "right_shoulder_roll_joint", "left_ankle_roll_joint",
        "right_ankle_roll_joint", "left_shoulder_yaw_joint",
        "right_shoulder_yaw_joint", "left_elbow_joint", "right_elbow_joint",
        "left_wrist_roll_joint", "right_wrist_roll_joint",
        "left_wrist_pitch_joint", "right_wrist_pitch_joint",
        "left_wrist_yaw_joint", "right_wrist_yaw_joint",
    )
    HARD_LIMITS = np.asarray(
        [
            [-2.5307, 2.8798], [-2.5307, 2.8798], [-2.618, 2.618],
            [-0.5236, 2.9671], [-2.9671, 0.5236], [-0.52, 0.52],
            [-2.7576, 2.7576], [-2.7576, 2.7576], [-0.52, 0.52],
            [-0.087267, 2.8798], [-0.087267, 2.8798], [-3.0892, 2.6704],
            [-3.0892, 2.6704], [-0.87267, 0.5236], [-0.87267, 0.5236],
            [-1.5882, 2.2515], [-2.2515, 1.5882], [-0.2618, 0.2618],
            [-0.2618, 0.2618], [-2.618, 2.618], [-2.618, 2.618],
            [-1.0472, 2.0944], [-1.0472, 2.0944],
            [-1.972222054, 1.972222054], [-1.972222054, 1.972222054],
            [-1.614429558, 1.614429558], [-1.614429558, 1.614429558],
            [-1.614429558, 1.614429558], [-1.614429558, 1.614429558],
        ],
        dtype=np.float32,
    )

    def __init__(self) -> None:
        midpoint = self.HARD_LIMITS.mean(axis=1)
        half = (self.HARD_LIMITS[:, 1] - self.HARD_LIMITS[:, 0]) * np.float32(0.45)
        self.midpoint = midpoint
        self.half = half

    def normalize(self, q: Any) -> np.ndarray:
        values = finite_array(q, (29,), "q_lab_rad")
        return np.clip((values - self.midpoint) / self.half, -1.0, 1.0).astype(
            np.float32
        )


@dataclass(frozen=True)
class FeatureBatch:
    actor: np.ndarray
    audit: np.ndarray
    actor_names: tuple[str, ...]
    audit_names: tuple[str, ...]
    record: dict[str, Any]


def interval_metadata(interval_s: float, *, first: bool) -> dict[str, Any]:
    if not math.isfinite(interval_s) or interval_s <= 0.0:
        raise ValueError("relay interval must be finite and positive")
    clipped = float(np.clip(interval_s, *INTERVAL_RANGE_S))
    return {
        "source": "first_shot_default" if first else "adjacent_fully_acked_shots",
        "raw_s": float(interval_s),
        "clipped_s": clipped,
        "normalized": (clipped - 0.8) / 0.8,
    }


def build_vectors(
    histories: Any,
    normalized_joints: Any,
    ball: Mapping[str, Any],
    previous_targets: Any,
    hitter: str,
    interval: Mapping[str, Any],
    *,
    ball_age_s: float = 0.0,
    history_lag_s: float = 0.0,
    acceleration: Any = (0.0, 0.0, -9.81),
    current_bases: Any | None = None,
) -> FeatureBatch:
    bases_history = finite_array(histories, (2, 5, 3), "base_history")
    joints = finite_array(normalized_joints, (2, 29), "normalized_joints")
    previous = finite_array(previous_targets, (2,), "previous_targets")
    acceleration_vector = finite_array(acceleration, (3,), "ball_acceleration")
    if hitter not in MODEL_ORDER:
        raise ValueError("unknown scheduled hitter")
    if not math.isfinite(ball_age_s) or ball_age_s < 0.0:
        raise ValueError("invalid_ball_age")
    if not math.isfinite(history_lag_s) or history_lag_s < 0.0:
        raise ValueError("invalid_history_lag")

    position = finite_array(ball["position"], (3,), "ball_position")
    velocity = finite_array(ball["velocity"], (3,), "ball_velocity")
    position = (
        position
        + velocity * np.float32(ball_age_s)
        + 0.5 * acceleration_vector * np.float32(ball_age_s**2)
    )
    velocity = velocity + acceleration_vector * np.float32(ball_age_s)
    ages = np.arange(4, -1, -1, dtype=np.float32) * HISTORY_DT_S + history_lag_s
    ball_history = (
        position[None, :]
        - velocity[None, :] * ages[:, None]
        + 0.5 * acceleration_vector[None, :] * np.square(ages[:, None])
    )
    relative = np.stack(
        (ball_history - bases_history[0], ball_history - bases_history[1]), axis=1
    )
    relative = np.clip(
        relative / np.asarray((3.0, 1.5, 1.5), dtype=np.float32), -3.0, 3.0
    ).reshape(-1)
    current = (
        bases_history[:, -1, :]
        if current_bases is None
        else finite_array(current_bases, (2, 3), "current_bases")
    )
    # Training uses max(abs(-.9), abs(.9), 1.0), hence base Y divides by 1.0.
    current_xy = current[:, :2].copy()
    current_xy[:, 0] /= 3.0
    current_xy[:, 1] /= 1.0
    current_xy = np.clip(current_xy, -3.0, 3.0).reshape(-1)
    previous_normalized = np.clip(previous / np.float32(0.9), -1.0, 1.0)
    actor = np.concatenate((relative, current_xy, previous_normalized)).astype(
        np.float32
    )

    audit_bases = bases_history[:, :, :2].copy()
    audit_bases[:, :, 0] /= 3.0
    audit_bases[:, :, 1] /= 1.0
    audit_bases = np.clip(audit_bases, -3.0, 3.0).reshape(-1)
    strike = np.concatenate(
        (
            finite_array(
                ball["predicted_strike_position"], (3,), "strike_position"
            )
            / np.asarray((3.0, 1.5, 1.5), dtype=np.float32),
            finite_array(ball["racket_velocity"], (3,), "racket_velocity") / 4.0,
            np.asarray([float(ball["time_to_strike_s"]) - ball_age_s], dtype=np.float32),
        )
    )
    strike = np.clip(strike, -3.0, 3.0)
    hitter_sign = -1.0 if hitter == "left" else 1.0
    normalized_interval = float(interval["normalized"])
    audit = np.concatenate(
        (
            audit_bases,
            joints.reshape(-1),
            strike,
            np.asarray((hitter_sign, normalized_interval), dtype=np.float32),
        )
    ).astype(np.float32)
    return FeatureBatch(
        finite_array(actor, (36,), "actor_observation"),
        finite_array(audit, (87,), "audit_observation"),
        ACTOR_NAMES,
        AUDIT_NAMES,
        {
            "model_order": list(MODEL_ORDER),
            "ros_mapping": dict(MODEL_TO_ROS),
            "base_history": bases_history.tolist(),
            "normalized_joints": joints.tolist(),
            "previous_targets_m": previous.tolist(),
            "ball": copy.deepcopy(dict(ball)),
            "ball_age_s": float(ball_age_s),
            "history_lag_s": float(history_lag_s),
            "history_dt_s": HISTORY_DT_S,
            "acceleration_assumption": acceleration_vector.tolist(),
            "scheduled_hitter": hitter,
            "interval": dict(interval),
        },
    )


class FeatureTracker:
    def __init__(self, normalizer: JointNormalizer | None = None) -> None:
        self.normalizer = normalizer or JointNormalizer()
        self.samples = {name: deque(maxlen=96) for name in MODEL_ORDER}
        self.previous_targets = np.asarray(HOME_PAIR, dtype=np.float32)
        self.last_fully_acked_shot_id: str | int | None = None
        self.last_fully_acked_receive_monotonic: float | None = None
        self.generation = 0

    def reset_session(self) -> None:
        for samples in self.samples.values():
            samples.clear()
        self.previous_targets = np.asarray(HOME_PAIR, dtype=np.float32)
        self.last_fully_acked_shot_id = None
        self.last_fully_acked_receive_monotonic = None
        self.generation += 1

    def ingest(self, ros_robot: str, state: Mapping[str, Any], received: float) -> None:
        model_robot = ROS_TO_MODEL.get(ros_robot)
        if model_robot is None:
            raise ValueError("unknown robot")
        samples = self.samples[model_robot]
        source_ns = int(state["source_monotonic_ns"])
        if samples and (
            received <= samples[-1][0] or source_ns <= samples[-1][1]
        ):
            raise ValueError(f"controller_clock_regression:{ros_robot}")
        samples.append((float(received), source_ns, copy.deepcopy(dict(state))))

    def interval_for(self, shot_id: str | int, received: float) -> dict[str, Any]:
        if (
            self.last_fully_acked_shot_id is None
            or self.last_fully_acked_receive_monotonic is None
        ):
            return interval_metadata(FIRST_INTERVAL_S, first=True)
        if shot_id == self.last_fully_acked_shot_id:
            raise ValueError("duplicate_fully_acked_shot")
        return interval_metadata(
            received - self.last_fully_acked_receive_monotonic, first=False
        )

    def commit_fully_acked(
        self, shot_id: str | int, received: float, targets: Any
    ) -> None:
        pair = finite_array(targets, (2,), "fully_acked_targets")
        if np.any(pair < WORKSPACE_Y[0]) or np.any(pair > WORKSPACE_Y[1]):
            raise ValueError("fully_acked_targets_outside_workspace")
        if pair[0] - pair[1] < 0.50 - 1.0e-6:
            raise ValueError("fully_acked_targets_gap")
        if shot_id == self.last_fully_acked_shot_id:
            return
        self.previous_targets = pair.copy()
        self.last_fully_acked_shot_id = shot_id
        self.last_fully_acked_receive_monotonic = float(received)

    def build(
        self,
        states: Mapping[str, Mapping[str, Any]],
        ball: Mapping[str, Any],
        now: float,
        hitter: str,
    ) -> FeatureBatch:
        if not ball or not ball.get("valid") or ball.get("shot_id") is None:
            raise ValueError("no_valid_ball")
        ball_age = now - float(ball["_receive_monotonic"])
        if not 0.0 <= ball_age <= 0.30:
            raise ValueError("stale_ball")
        histories = []
        joints = []
        end = min(float(states[name]["_receive_monotonic"]) for name in MODEL_TO_ROS.values())
        grid = end - np.arange(4, -1, -1, dtype=np.float64) * HISTORY_DT_S
        for model_robot in MODEL_ORDER:
            ros_robot = MODEL_TO_ROS[model_robot]
            state = states.get(ros_robot)
            if state is None or not state.get("valid") or state.get("emergency_stop"):
                raise ValueError(f"invalid_state:{ros_robot}")
            if not 0.0 <= now - float(state["_receive_monotonic"]) <= 0.25:
                raise ValueError(f"stale_state:{ros_robot}")
            samples = self.samples[model_robot]
            times = np.asarray([sample[0] for sample in samples], dtype=np.float64)
            if len(times) < 2 or times[0] > grid[0] + 1.0e-9 or times[-1] < grid[-1] - 1.0e-9:
                raise ValueError(f"warming_history:{ros_robot}")
            nearby = times[(times >= grid[0] - 0.1) & (times <= end + 0.1)]
            if len(nearby) > 1 and float(np.max(np.diff(nearby))) > 0.1:
                raise ValueError(f"history_gap:{ros_robot}")
            positions = np.asarray(
                [sample[2]["base_position_xyz"] for sample in samples], dtype=np.float32
            )
            histories.append(
                np.stack(
                    [np.interp(grid, times, positions[:, axis]) for axis in range(3)],
                    axis=1,
                )
            )
            joints.append(self.normalizer.normalize(state["q"]))
        interval = self.interval_for(
            ball["shot_id"],
            float(ball.get("_shot_receive_monotonic", ball["_receive_monotonic"])),
        )
        result = build_vectors(
            histories,
            joints,
            ball,
            self.previous_targets,
            hitter,
            interval,
            ball_age_s=ball_age,
            history_lag_s=now - end,
            current_bases=[states[MODEL_TO_ROS[name]]["base_position_xyz"] for name in MODEL_ORDER],
        )
        result.record["history_end_receive_monotonic"] = end
        result.record["decision_monotonic"] = float(now)
        result.record["tracker_generation"] = self.generation
        return result
