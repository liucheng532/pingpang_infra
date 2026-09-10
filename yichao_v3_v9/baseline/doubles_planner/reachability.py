from __future__ import annotations

from typing import Mapping

import numpy as np

from .config import PlannerConfig
from .models import RobotFeedback


def viability_value(
    gap_m: float,
    relative_velocity_mps: float,
    safety_distance_m: float,
    maximum_evasive_relative_acceleration_mps2: float,
    latency_s: float,
) -> float:
    if maximum_evasive_relative_acceleration_mps2 <= 0.0:
        raise ValueError("maximum evasive relative acceleration must be positive")
    if latency_s < 0.0:
        raise ValueError("latency_s must not be negative")
    closing_speed = max(0.0, -float(relative_velocity_mps))
    stopping_distance = (
        closing_speed * latency_s
        + closing_speed * closing_speed
        / (2.0 * maximum_evasive_relative_acceleration_mps2)
    )
    return float(gap_m - safety_distance_m - stopping_distance)


def reachability_waypoints(
    feedback: Mapping[str, RobotFeedback],
    final_goals: Mapping[str, np.ndarray],
    config: PlannerConfig,
    hitter: str,
    controlled_robot: str | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, float | bool]]:
    left, right = config.robots
    if controlled_robot is not None and controlled_robot not in config.robots:
        raise ValueError("controlled_robot must name one configured robot")
    positions = np.asarray(
        [feedback[left].base_xy[1], feedback[right].base_xy[1]],
        dtype=float,
    )
    velocities = np.asarray(
        [feedback[left].velocity_xy[1], feedback[right].velocity_xy[1]],
        dtype=float,
    )
    goals = np.asarray(
        [final_goals[left][1], final_goals[right][1]],
        dtype=float,
    )
    desired_velocity = np.clip(
        config.base_position_gain * (goals - positions),
        -config.max_base_speed,
        config.max_base_speed,
    )
    nominal_acceleration = np.clip(
        (desired_velocity - velocities) / config.cbf_control_period,
        -config.cbf_acceleration_limit,
        config.cbf_acceleration_limit,
    )

    acceleration_values = np.linspace(
        -config.cbf_acceleration_limit,
        config.cbf_acceleration_limit,
        config.reachability_grid_points,
    )
    if controlled_robot is None:
        candidate_left, candidate_right = np.meshgrid(
            acceleration_values,
            acceleration_values,
            indexing="ij",
        )
        candidate_acceleration = np.stack(
            (candidate_left.reshape(-1), candidate_right.reshape(-1)),
            axis=-1,
        )
        available_relative_acceleration = (
            2.0 * config.cbf_acceleration_limit
            - 2.0 * config.reachability_disturbance_acceleration
        )
    elif controlled_robot == left:
        candidate_acceleration = np.column_stack(
            (acceleration_values, np.zeros_like(acceleration_values))
        )
        available_relative_acceleration = (
            config.cbf_acceleration_limit
            - 2.0 * config.reachability_disturbance_acceleration
        )
    else:
        candidate_acceleration = np.column_stack(
            (np.zeros_like(acceleration_values), acceleration_values)
        )
        available_relative_acceleration = (
            config.cbf_acceleration_limit
            - 2.0 * config.reachability_disturbance_acceleration
        )
    available_relative_acceleration = max(1.0e-6, available_relative_acceleration)

    control_dt = config.cbf_control_period
    disturbance = np.asarray(
        [
            config.reachability_disturbance_acceleration,
            -config.reachability_disturbance_acceleration,
        ],
        dtype=float,
    )
    worst_acceleration = candidate_acceleration + disturbance[None, :]
    next_velocity = np.clip(
        velocities[None, :] + worst_acceleration * control_dt,
        -config.max_base_speed,
        config.max_base_speed,
    )
    next_position = positions[None, :] + 0.5 * (
        velocities[None, :] + next_velocity
    ) * control_dt
    next_gap = next_position[:, 1] - next_position[:, 0]
    next_relative_velocity = next_velocity[:, 1] - next_velocity[:, 0]
    safety_distance = config.min_separation + config.separation_margin
    closing_speed = np.maximum(0.0, -next_relative_velocity)
    stopping_distance = (
        closing_speed * config.reachability_latency
        + np.square(closing_speed) / (2.0 * available_relative_acceleration)
    )
    candidate_values = next_gap - safety_distance - stopping_distance
    viable = candidate_values >= 0.0

    weights = np.asarray(
        [
            config.cbf_hitter_weight if hitter == left else 1.0,
            config.cbf_hitter_weight if hitter == right else 1.0,
        ],
        dtype=float,
    )
    costs = np.sum(
        weights[None, :] * np.square(candidate_acceleration - nominal_acceleration[None, :]),
        axis=1,
    )
    if np.any(viable):
        viable_indices = np.flatnonzero(viable)
        selected_index = int(viable_indices[int(np.argmin(costs[viable]))])
        safe_acceleration = candidate_acceleration[selected_index]
        selected_value = float(candidate_values[selected_index])
        feasible = True
    else:
        safe_acceleration = np.asarray(
            [-config.cbf_acceleration_limit, config.cbf_acceleration_limit],
            dtype=float,
        )
        if controlled_robot == left:
            safe_acceleration[1] = 0.0
        elif controlled_robot == right:
            safe_acceleration[0] = 0.0
        selected_value = float(np.max(candidate_values))
        feasible = False

    current_gap = float(positions[1] - positions[0])
    current_relative_velocity = float(velocities[1] - velocities[0])
    current_value = viability_value(
        current_gap,
        current_relative_velocity,
        safety_distance,
        available_relative_acceleration,
        config.reachability_latency,
    )
    emergency = (
        current_value < config.reachability_emergency_buffer
        or not feasible
    )
    active = bool(
        np.max(np.abs(safe_acceleration - nominal_acceleration)) > 1.0e-6
    )
    waypoints: dict[str, np.ndarray] = {}
    if emergency:
        for index, name in enumerate(config.robots):
            state = feedback[name]
            target_y = (
                config.workspace_y[index]
                if controlled_robot is None or name == controlled_robot
                else float(state.base_xy[1])
            )
            waypoints[name] = np.asarray([state.base_xy[0], target_y], dtype=float)
    else:
        safe_velocity = np.clip(
            velocities + safe_acceleration * control_dt,
            -config.max_base_speed,
            config.max_base_speed,
        )
        for index, name in enumerate(config.robots):
            state = feedback[name]
            target_y = float(
                np.clip(
                    state.base_xy[1] + config.cbf_horizon * safe_velocity[index],
                    config.workspace_y[0],
                    config.workspace_y[1],
                )
            )
            waypoints[name] = np.asarray([state.base_xy[0], target_y], dtype=float)
    return waypoints, {
        "reachability_active": active,
        "reachability_emergency": emergency,
        "reachability_feasible": feasible,
        "reachability_value": current_value,
        "reachability_successor_value": selected_value,
        "reachability_relative_velocity": current_relative_velocity,
        "reachability_control_distance": safety_distance,
        "reachability_safe_left_acceleration": float(safe_acceleration[0]),
        "reachability_safe_right_acceleration": float(safe_acceleration[1]),
        "reachability_viable_action_fraction": float(np.mean(viable)),
        "reachability_uncontrolled_nominal_suppressed": controlled_robot is not None,
    }


__all__ = ["reachability_waypoints", "viability_value"]
