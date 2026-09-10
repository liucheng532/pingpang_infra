from __future__ import annotations

import math
from dataclasses import asdict, dataclass, fields
from typing import Any, Mapping


@dataclass(frozen=True)
class PlannerConfig:
    robots: tuple[str, str] = ("left", "right")
    initial_hitter: str | None = None
    base_x: float = 0.0
    home_y: tuple[float, float] = (-0.35, 0.35)
    outward_y: tuple[float, float] = (-0.78, 0.78)
    teammate_avoidance_y: tuple[float, float] = (-0.70, 0.70)
    ready_y: tuple[float, float] = (-0.18, 0.18)
    racket_reach_y: tuple[float, float] = (0.31, -0.31)
    workspace_y: tuple[float, float] = (-0.95, 0.95)
    min_separation: float = 0.55
    collision_distance: float = 0.35
    hard_safety_margin: float = 0.12
    separation_margin: float = 0.12
    max_base_speed: float = 1.50
    max_base_acceleration: float = 4.0
    base_position_gain: float = 12.5
    feedback_timeout: float = 0.25
    prediction_timeout: float = 0.30
    first_hit_reset_seconds: float = 10.0
    shot_arm_time: float = 0.08
    shot_release_time: float = -0.55
    commit_time_to_strike: float = 0.55
    commit_abort_time_to_strike: float = 0.08
    commit_minimum_base_ttc: float = 0.30
    commit_handoff_timeout: float = 2.0
    inactive_time_to_strike: float = -0.5
    strike_reach_tolerance: float = 0.18
    safe_ball_x: float = 0.45
    safe_ball_z: float = 1.0
    preview_gain: float = -0.45
    preview_sigma: float = 0.32
    mpc_grid_points: int = 81
    mpc_coverage_weight: float = 1.0
    mpc_travel_weight: float = 0.18
    mpc_exit_weight: float = 0.35
    cbf_alpha: float = 5.0
    cbf_horizon: float = 0.08
    cbf_control_period: float = 0.02
    cbf_acceleration_limit: float = 3.2
    cbf_reaction_time: float = 0.14
    cbf_emergency_buffer: float = 0.04
    cbf_hitter_weight: float = 7.0
    reachability_grid_points: int = 17
    reachability_disturbance_acceleration: float = 0.35
    reachability_latency: float = 0.14
    reachability_emergency_buffer: float = 0.02
    hold_when_feedback_missing: bool = True
    allow_peer_outward_hold_ready: bool = False
    peer_clear_max_speed: float = 0.15

    def __post_init__(self) -> None:
        if len(self.robots) != 2 or self.robots[0] == self.robots[1]:
            raise ValueError("PlannerConfig.robots must contain two unique names")
        if self.initial_hitter is not None and self.initial_hitter not in self.robots:
            raise ValueError("initial_hitter must be one of PlannerConfig.robots")
        for name in (
            "home_y",
            "outward_y",
            "teammate_avoidance_y",
            "ready_y",
            "racket_reach_y",
            "workspace_y",
        ):
            value = getattr(self, name)
            if len(value) != 2:
                raise ValueError(f"{name} must have two values")
        if self.workspace_y[0] >= self.workspace_y[1]:
            raise ValueError("workspace_y must be increasing")
        if not 0.0 < self.collision_distance < self.min_separation:
            raise ValueError("collision_distance must be below min_separation")
        if self.hard_safety_margin < 0.0:
            raise ValueError("hard_safety_margin must not be negative")
        if self.collision_distance + self.hard_safety_margin > self.min_separation:
            raise ValueError("hard safety distance must not exceed min_separation")
        if self.min_separation >= self.workspace_y[1] - self.workspace_y[0]:
            raise ValueError("workspace is too narrow for min_separation")
        if not (
            self.outward_y[0]
            <= self.teammate_avoidance_y[0]
            < self.home_y[0]
            < self.home_y[1]
            < self.teammate_avoidance_y[1]
            <= self.outward_y[1]
        ):
            raise ValueError(
                "teammate avoidance lanes must lie outward of home and no farther "
                "than outward_y"
            )
        if (
            self.teammate_avoidance_y[1] - self.teammate_avoidance_y[0]
            < self.min_separation + self.separation_margin
        ):
            raise ValueError("teammate avoidance lanes do not provide the planned goal gap")
        if (
            self.max_base_speed <= 0.0
            or self.max_base_acceleration <= 0.0
            or self.base_position_gain <= 0.0
        ):
            raise ValueError("base motion limits and position gain must be positive")
        if self.feedback_timeout <= 0.0 or self.prediction_timeout <= 0.0:
            raise ValueError("timeouts must be positive")
        if self.peer_clear_max_speed <= 0.0:
            raise ValueError("peer_clear_max_speed must be positive")
        if not (
            self.commit_time_to_strike
            > self.commit_abort_time_to_strike
            > self.shot_release_time
        ):
            raise ValueError(
                "commit timing must satisfy commit_time_to_strike > "
                "commit_abort_time_to_strike > shot_release_time"
            )
        if self.commit_minimum_base_ttc < 0.0 or self.commit_handoff_timeout <= 0.0:
            raise ValueError("commit TTC and handoff timeout must be non-negative/positive")
        if self.mpc_grid_points < 3:
            raise ValueError("mpc_grid_points must be at least 3")
        if (
            self.cbf_alpha <= 0.0
            or self.cbf_horizon <= 0.0
            or self.cbf_control_period <= 0.0
            or self.cbf_acceleration_limit <= 0.0
            or self.cbf_reaction_time < 0.0
            or self.cbf_emergency_buffer < 0.0
        ):
            raise ValueError("CBF parameters must be positive")
        if self.cbf_acceleration_limit > self.max_base_acceleration:
            raise ValueError("cbf_acceleration_limit must not exceed max_base_acceleration")
        if self.cbf_alpha * self.cbf_horizon > 1.0:
            raise ValueError("cbf_alpha * cbf_horizon must not exceed 1")
        if not math.isclose(
            self.cbf_horizon * self.base_position_gain,
            1.0,
            rel_tol=0.0,
            abs_tol=1.0e-9,
        ):
            raise ValueError("cbf_horizon * base_position_gain must equal 1")
        if self.reachability_grid_points < 3 or self.reachability_grid_points % 2 == 0:
            raise ValueError("reachability_grid_points must be an odd integer at least 3")
        if (
            self.reachability_disturbance_acceleration < 0.0
            or self.reachability_disturbance_acceleration
            >= 0.5 * self.cbf_acceleration_limit
        ):
            raise ValueError(
                "reachability disturbance must be below half the acceleration limit"
            )
        if self.reachability_latency < 0.0 or self.reachability_emergency_buffer < 0.0:
            raise ValueError("reachability latency and emergency buffer must not be negative")

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "PlannerConfig":
        known = {field.name for field in fields(cls)}
        unknown = sorted(set(values) - known)
        if unknown:
            raise ValueError(f"unknown planner config keys: {unknown}")
        converted = dict(values)
        for name in (
            "robots",
            "home_y",
            "outward_y",
            "teammate_avoidance_y",
            "ready_y",
            "racket_reach_y",
            "workspace_y",
        ):
            if name in converted:
                converted[name] = tuple(converted[name])
        return cls(**converted)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
