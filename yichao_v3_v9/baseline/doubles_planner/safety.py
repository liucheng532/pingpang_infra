from __future__ import annotations

from typing import Mapping

import numpy as np

from .config import PlannerConfig
from .models import RobotFeedback


def _bounded_halfspace_projection(
    nominal_left: float,
    nominal_right: float,
    lower_bound: float,
    bound: float,
    left_weight: float,
    right_weight: float,
) -> tuple[float, float, bool]:
    left = float(np.clip(nominal_left, -bound, bound))
    right = float(np.clip(nominal_right, -bound, bound))
    if right - left >= lower_bound:
        return left, right, False

    feasible_bound = min(float(lower_bound), 2.0 * bound)
    minimum_left = max(-bound, -bound - feasible_bound)
    maximum_left = min(bound, bound - feasible_bound)
    unconstrained_left = (
        left_weight * nominal_left
        + right_weight * (nominal_right - feasible_bound)
    ) / (left_weight + right_weight)
    left = float(np.clip(unconstrained_left, minimum_left, maximum_left))
    right = left + feasible_bound
    return left, right, True


def cbf_waypoints(
    feedback: Mapping[str, RobotFeedback],
    final_goals: Mapping[str, np.ndarray],
    config: PlannerConfig,
    hitter: str,
    controlled_robot: str | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, float | bool]]:
    left, right = config.robots
    if controlled_robot is not None and controlled_robot not in config.robots:
        raise ValueError("controlled_robot must name one configured robot")
    left_y = float(feedback[left].base_xy[1])
    right_y = float(feedback[right].base_xy[1])
    desired_left_velocity = float(
        np.clip(
            config.base_position_gain * (float(final_goals[left][1]) - left_y),
            -config.max_base_speed,
            config.max_base_speed,
        )
    )
    desired_right_velocity = float(
        np.clip(
            config.base_position_gain * (float(final_goals[right][1]) - right_y),
            -config.max_base_speed,
            config.max_base_speed,
        )
    )
    left_velocity = float(feedback[left].velocity_xy[1])
    right_velocity = float(feedback[right].velocity_xy[1])
    nominal_left_acceleration = float(
        np.clip(
            (desired_left_velocity - left_velocity) / config.cbf_control_period,
            -config.cbf_acceleration_limit,
            config.cbf_acceleration_limit,
        )
    )
    nominal_right_acceleration = float(
        np.clip(
            (desired_right_velocity - right_velocity) / config.cbf_control_period,
            -config.cbf_acceleration_limit,
            config.cbf_acceleration_limit,
        )
    )
    gap = right_y - left_y
    relative_velocity = right_velocity - left_velocity
    barrier_distance = config.min_separation + config.separation_margin
    barrier = gap - barrier_distance
    required_relative_acceleration = (
        -2.0 * config.cbf_alpha * relative_velocity
        - config.cbf_alpha * config.cbf_alpha * barrier
    )
    closing_speed = max(0.0, -relative_velocity)
    maximum_relative_acceleration = 2.0 * config.cbf_acceleration_limit
    braking_distance = (
        closing_speed * config.cbf_reaction_time
        + closing_speed * closing_speed / (2.0 * maximum_relative_acceleration)
    )
    braking_barrier = gap - barrier_distance - braking_distance
    if closing_speed > 1.0e-6:
        braking_denominator = config.cbf_reaction_time + closing_speed / maximum_relative_acceleration
        braking_required_acceleration = (
            -relative_velocity - config.cbf_alpha * braking_barrier
        ) / braking_denominator
        required_relative_acceleration = max(
            required_relative_acceleration,
            braking_required_acceleration,
        )
    else:
        braking_required_acceleration = float("-inf")
    weights = {
        left: config.cbf_hitter_weight if hitter == left else 1.0,
        right: config.cbf_hitter_weight if hitter == right else 1.0,
    }
    if controlled_robot is None:
        safe_left_acceleration, safe_right_acceleration, active = _bounded_halfspace_projection(
            nominal_left_acceleration,
            nominal_right_acceleration,
            required_relative_acceleration,
            config.cbf_acceleration_limit,
            weights[left],
            weights[right],
        )
        feasible = required_relative_acceleration <= 2.0 * config.cbf_acceleration_limit
    elif controlled_robot == left:
        maximum_left_acceleration = min(
            config.cbf_acceleration_limit,
            -required_relative_acceleration,
        )
        feasible = maximum_left_acceleration >= -config.cbf_acceleration_limit
        safe_left_acceleration = float(
            np.clip(
                nominal_left_acceleration,
                -config.cbf_acceleration_limit,
                max(-config.cbf_acceleration_limit, maximum_left_acceleration),
            )
        )
        safe_right_acceleration = 0.0
        active = abs(safe_left_acceleration - nominal_left_acceleration) > 1.0e-9
    else:
        minimum_right_acceleration = max(
            -config.cbf_acceleration_limit,
            required_relative_acceleration,
        )
        feasible = minimum_right_acceleration <= config.cbf_acceleration_limit
        safe_left_acceleration = 0.0
        safe_right_acceleration = float(
            np.clip(
                nominal_right_acceleration,
                min(config.cbf_acceleration_limit, minimum_right_acceleration),
                config.cbf_acceleration_limit,
            )
        )
        active = abs(safe_right_acceleration - nominal_right_acceleration) > 1.0e-9
    safe_velocities = {
        left: float(
            np.clip(
                left_velocity + safe_left_acceleration * config.cbf_control_period,
                -config.max_base_speed,
                config.max_base_speed,
            )
        ),
        right: float(
            np.clip(
                right_velocity + safe_right_acceleration * config.cbf_control_period,
                -config.max_base_speed,
                config.max_base_speed,
            )
        ),
    }
    waypoints: dict[str, np.ndarray] = {}
    emergency = braking_barrier < config.cbf_emergency_buffer or not feasible
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
        for name in config.robots:
            state = feedback[name]
            waypoint_y = float(
                np.clip(
                    state.base_xy[1] + config.cbf_horizon * safe_velocities[name],
                    config.workspace_y[0],
                    config.workspace_y[1],
                )
            )
            waypoints[name] = np.asarray([state.base_xy[0], waypoint_y], dtype=float)
    return waypoints, {
        "cbf_active": active,
        "cbf_emergency": emergency,
        "cbf_feasible": feasible,
        "cbf_uncontrolled_nominal_suppressed": controlled_robot is not None,
        "barrier": barrier,
        "relative_velocity": relative_velocity,
        "closing_speed": closing_speed,
        "braking_distance": braking_distance,
        "braking_barrier": braking_barrier,
        "braking_required_acceleration": braking_required_acceleration,
        "control_barrier_distance": barrier_distance,
        "required_relative_acceleration": required_relative_acceleration,
        "safe_left_acceleration": safe_left_acceleration,
        "safe_right_acceleration": safe_right_acceleration,
        "safe_left_velocity": safe_velocities[left],
        "safe_right_velocity": safe_velocities[right],
    }
