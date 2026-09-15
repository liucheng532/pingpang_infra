"""Exact V7 29D actor and 87D frozen-CBF feature construction."""

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
MAXIMUM_TTS_S = 0.54

ACTOR_NAMES = (
    "hitter_left_onehot",
    "hitter_right_onehot",
    "time_to_strike_s",
    "predicted_hitting_velocity_x",
    "predicted_hitting_velocity_y",
    "predicted_hitting_velocity_z",
    "striking_target_position_x",
    "striking_target_position_y",
    "striking_target_position_z",
) + tuple(
    f"{robot}_base_history_{history}_{axis}"
    for robot in MODEL_ORDER
    for history in range(HISTORY_LENGTH)
    for axis in "xy"
)

SAFE_NAMES = tuple(
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
    "hitter_left_onehot",
    "hitter_right_onehot",
)

if len(ACTOR_NAMES) != 29 or len(SAFE_NAMES) != 87:  # pragma: no cover
    raise RuntimeError("frozen V7 feature schema dimensions changed")


def finite_array(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float32)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{label}: expected finite {shape}")
    return result


class JointNormalizer:
    """Isaac Lab 0.9 soft-limit normalization in LAB joint order."""

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
        self.midpoint = self.HARD_LIMITS.mean(axis=1)
        self.half = (
            (self.HARD_LIMITS[:, 1] - self.HARD_LIMITS[:, 0]) * np.float32(0.45)
        )

    def normalize(self, q: Any) -> np.ndarray:
        values = finite_array(q, (29,), "q_lab_rad")
        return np.clip((values - self.midpoint) / self.half, -1.0, 1.0).astype(
            np.float32
        )


@dataclass(frozen=True)
class FeatureBatch:
    actor: np.ndarray
    safe: np.ndarray
    actor_names: tuple[str, ...]
    safe_names: tuple[str, ...]
    record: dict[str, Any]

def build_vectors(
    histories: Any,
    normalized_joints: Any,
    ball: Mapping[str, Any],
    hitter: str,
    *,
    ball_age_s: float = 0.0,
) -> FeatureBatch:
    """Build fields in the handoff schema order without Isaac imports."""

    bases_history = finite_array(histories, (2, 5, 3), "base_history")
    joints = finite_array(normalized_joints, (2, 29), "normalized_joints")
    if hitter not in MODEL_ORDER:
        raise ValueError("unknown scheduled hitter")
    if not math.isfinite(ball_age_s) or ball_age_s < 0.0:
        raise ValueError("invalid_ball_age")

    tts = float(ball["time_to_strike_s"]) - ball_age_s
    if not math.isfinite(tts):
        raise ValueError("invalid_time_to_strike")
    tts = float(np.clip(tts, 0.0, MAXIMUM_TTS_S))
    hitter_onehot = np.asarray(
        (1.0, 0.0) if hitter == "left" else (0.0, 1.0), dtype=np.float32
    )
    velocity = np.clip(
        finite_array(ball["racket_velocity"], (3,), "predicted_hitting_velocity")
        / np.asarray((4.0, 4.0, 4.0), dtype=np.float32),
        -3.0,
        3.0,
    )
    target = np.clip(
        finite_array(ball["predicted_strike_position"], (3,), "strike_target_position")
        / np.asarray((3.0, 1.5, 1.5), dtype=np.float32),
        -3.0,
        3.0,
    )
    normalized_bases = bases_history[:, :, :2].copy()
    normalized_bases[:, :, 0] /= np.float32(3.0)
    # Training uses max(abs(-.9), abs(.9), 1.0), so Y divides by 1.0.
    normalized_bases[:, :, 1] /= np.float32(1.0)
    normalized_bases = np.clip(normalized_bases, -3.0, 3.0).reshape(-1)

    actor = np.concatenate(
        (hitter_onehot, np.asarray((tts,), dtype=np.float32), velocity, target, normalized_bases)
    ).astype(np.float32)
    safe = np.concatenate(
        (
            normalized_bases,
            joints.reshape(-1),
            target,
            velocity,
            np.asarray((tts,), dtype=np.float32),
            hitter_onehot,
        )
    ).astype(np.float32)
    return FeatureBatch(
        finite_array(actor, (29,), "actor_observation"),
        finite_array(safe, (87,), "safe_observation"),
        ACTOR_NAMES,
        SAFE_NAMES,
        {
            "model_order": list(MODEL_ORDER),
            "ros_mapping": dict(MODEL_TO_ROS),
            "base_history": bases_history.tolist(),
            "normalized_joints": joints.tolist(),
            "ball": copy.deepcopy(dict(ball)),
            "ball_age_s": float(ball_age_s),
            "corrected_time_to_strike_s": tts,
            "history_dt_s": HISTORY_DT_S,
            "scheduled_hitter": hitter,
            "actor_schema": "doubles-shot-observation-v7-absolute-ball-torso-v3",
            "safe_schema": "doubles-safe-observation-v7-hitter-onehot-v1",
        },
    )


class FeatureTracker:
    """Maintain five 20 ms torso samples with strict freshness and clock checks."""

    def __init__(self, normalizer: JointNormalizer | None = None) -> None:
        self.normalizer = normalizer or JointNormalizer()
        self.samples = {name: deque(maxlen=96) for name in MODEL_ORDER}
        self.previous_targets = np.asarray(HOME_PAIR, dtype=np.float32)
        self.last_fully_acked_shot_id: str | int | None = None
        self.generation = 0

    def reset_session(self) -> None:
        for samples in self.samples.values():
            samples.clear()
        self.previous_targets = np.asarray(HOME_PAIR, dtype=np.float32)
        self.last_fully_acked_shot_id = None
        self.generation += 1

    def ingest(self, ros_robot: str, state: Mapping[str, Any], received: float) -> None:
        model_robot = ROS_TO_MODEL.get(ros_robot)
        if model_robot is None:
            raise ValueError("unknown robot")
        samples = self.samples[model_robot]
        source_ns = int(state["source_monotonic_ns"])
        if samples and (received <= samples[-1][0] or source_ns <= samples[-1][1]):
            raise ValueError(f"controller_clock_regression:{ros_robot}")
        samples.append((float(received), source_ns, copy.deepcopy(dict(state))))

    def commit_fully_acked(self, shot_id: str | int, received: float, targets: Any) -> None:
        del received
        pair = finite_array(targets, (2,), "fully_acked_targets")
        if np.any(pair < WORKSPACE_Y[0]) or np.any(pair > WORKSPACE_Y[1]):
            raise ValueError("fully_acked_targets_outside_workspace")
        if pair[0] - pair[1] < 0.50 - 1.0e-6:
            raise ValueError("fully_acked_targets_gap")
        if shot_id == self.last_fully_acked_shot_id:
            return
        self.previous_targets = pair.copy()
        self.last_fully_acked_shot_id = shot_id

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
        result = build_vectors(histories, joints, ball, hitter, ball_age_s=ball_age)
        result.record["history_end_receive_monotonic"] = end
        result.record["decision_monotonic"] = float(now)
        result.record["tracker_generation"] = self.generation
        return result
