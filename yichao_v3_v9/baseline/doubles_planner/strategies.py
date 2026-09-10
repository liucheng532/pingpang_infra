from __future__ import annotations

from dataclasses import dataclass, field
from typing import Mapping, Protocol

import numpy as np

from .config import PlannerConfig
from .geometry import desired_base_y
from .models import BallPrediction, RobotFeedback
from .rl import FrozenQPolicy, load_q_policy


@dataclass(frozen=True)
class StrategyContext:
    config: PlannerConfig
    phase: str
    hitter: str
    next_hitter: str
    prediction: BallPrediction
    feedback: Mapping[str, RobotFeedback]
    previous_goals: Mapping[str, np.ndarray]


@dataclass(frozen=True)
class StrategyDecision:
    goals: Mapping[str, np.ndarray]
    fallbacks: tuple[str, ...] = ()
    diagnostics: Mapping[str, float | str | bool] = field(default_factory=dict)
    use_cbf: bool = False
    use_reachability: bool = False


class PositionStrategy(Protocol):
    name: str

    def decide(self, context: StrategyContext) -> StrategyDecision:
        ...


def _clip_y(value: float, config: PlannerConfig) -> float:
    return float(np.clip(value, config.workspace_y[0], config.workspace_y[1]))


def _goal(feedback: RobotFeedback, y: float) -> np.ndarray:
    return np.asarray([float(feedback.base_xy[0]), float(y)], dtype=float)


def _strike_base_y(context: StrategyContext) -> float:
    return desired_base_y(
        context.config,
        context.hitter,
        float(context.prediction.position[1]),
    )


def project_ordered_pair(
    left_y: float,
    right_y: float,
    minimum_gap: float,
    bounds: tuple[float, float],
    left_weight: float = 1.0,
    right_weight: float = 1.0,
) -> tuple[float, float, bool]:
    left_y = float(np.clip(left_y, bounds[0], bounds[1]))
    right_y = float(np.clip(right_y, bounds[0], bounds[1]))
    if right_y - left_y >= minimum_gap:
        return left_y, right_y, False

    denominator = left_weight + right_weight
    projected_left = (
        left_weight * left_y + right_weight * (right_y - minimum_gap)
    ) / denominator
    projected_right = projected_left + minimum_gap
    if projected_left < bounds[0]:
        projected_left = bounds[0]
        projected_right = bounds[0] + minimum_gap
    if projected_right > bounds[1]:
        projected_right = bounds[1]
        projected_left = bounds[1] - minimum_gap
    return float(projected_left), float(projected_right), True


class V0Strategy:
    """Behavior-compatible baseline from planner commit 398f193.

    Only the hitter moves.  If its final target is too close to the teammate's
    current base, the hitter is frozen for that command.
    """

    name = "v0"

    def decide(self, context: StrategyContext) -> StrategyDecision:
        config = context.config
        goals = {
            name: feedback.base_xy.copy()
            for name, feedback in context.feedback.items()
        }
        hitter = context.hitter
        teammate = context.next_hitter
        target_y = _clip_y(float(context.prediction.position[1]), config)
        fallbacks: list[str] = []
        if abs(target_y - float(context.feedback[teammate].base_xy[1])) < config.min_separation:
            target_y = float(context.feedback[hitter].base_xy[1])
            fallbacks.append("v0_collision_freeze")
        goals[hitter] = _goal(context.feedback[hitter], target_y)
        return StrategyDecision(
            goals=goals,
            fallbacks=tuple(fallbacks),
            diagnostics={"nominal_strike_y": float(context.prediction.position[1])},
        )


class RelayHeuristicStrategy:
    name = "relay_heuristic"

    def _preview_y(self, context: StrategyContext) -> float:
        config = context.config
        next_index = config.robots.index(context.next_hitter)
        return _clip_y(
            config.ready_y[next_index]
            + config.preview_gain * float(context.prediction.position[1]),
            config,
        )

    def decide(self, context: StrategyContext) -> StrategyDecision:
        config = context.config
        left, right = config.robots
        hitter_index = config.robots.index(context.hitter)
        teammate_index = config.robots.index(context.next_hitter)
        strike_y = _strike_base_y(context)
        preview_y = self._preview_y(context)

        if context.phase == "post_hit":
            teammate_stage_y = preview_y
            teammate_stage_mode = "preview"
            nominal = {
                context.hitter: config.outward_y[hitter_index],
                context.next_hitter: teammate_stage_y,
            }
            hitter_weight = 2.0
        elif context.phase == "follow_through":
            teammate_stage_y = config.teammate_avoidance_y[teammate_index]
            teammate_stage_mode = "clear_hold"
            nominal = {
                context.hitter: config.outward_y[hitter_index],
                context.next_hitter: teammate_stage_y,
            }
            hitter_weight = 2.0
        else:
            teammate_stage_y = config.teammate_avoidance_y[teammate_index]
            teammate_stage_mode = "outward"
            nominal = {
                context.hitter: strike_y,
                context.next_hitter: teammate_stage_y,
            }
            hitter_weight = 20.0

        weights = {
            context.hitter: hitter_weight,
            context.next_hitter: 1.0,
        }
        left_y, right_y, projected = project_ordered_pair(
            nominal[left],
            nominal[right],
            config.min_separation + config.separation_margin,
            config.workspace_y,
            left_weight=weights[left],
            right_weight=weights[right],
        )
        goals = {
            left: _goal(context.feedback[left], left_y),
            right: _goal(context.feedback[right], right_y),
        }
        fallbacks = ("ordered_goal_projection",) if projected else ()
        return StrategyDecision(
            goals=goals,
            fallbacks=fallbacks,
            diagnostics={
                "nominal_strike_y": float(context.prediction.position[1]),
                "nominal_base_y": strike_y,
                "preview_y": preview_y,
                "teammate_stage_y": teammate_stage_y,
                "teammate_stage_mode": teammate_stage_mode,
                "goal_gap": right_y - left_y,
            },
        )


class CBFMPCStrategy(RelayHeuristicStrategy):
    name = "cbf_mpc"

    def _select_teammate_goal(self, context: StrategyContext, hitter_goal: float) -> float:
        config = context.config
        teammate = context.next_hitter
        teammate_index = config.robots.index(teammate)
        current_y = float(context.feedback[teammate].base_xy[1])
        expected_next_y = _clip_y(
            config.ready_y[teammate_index]
            + config.preview_gain * float(context.prediction.position[1]),
            config,
        )
        candidates = np.linspace(
            config.workspace_y[0],
            config.workspace_y[1],
            config.mpc_grid_points,
        )
        if teammate == config.robots[0]:
            feasible = candidates <= hitter_goal - config.min_separation - config.separation_margin
        else:
            feasible = candidates >= hitter_goal + config.min_separation + config.separation_margin
        if not np.any(feasible):
            return expected_next_y
        candidates = candidates[feasible]
        travel_time = np.abs(candidates - current_y) / config.max_base_speed
        coverage_cost = np.square(candidates - expected_next_y)
        exit_y = config.outward_y[config.robots.index(context.hitter)]
        exit_conflict = np.maximum(
            0.0,
            config.min_separation + config.separation_margin - np.abs(candidates - exit_y),
        )
        costs = (
            config.mpc_coverage_weight * coverage_cost
            + config.mpc_travel_weight * travel_time
            + config.mpc_exit_weight * np.square(exit_conflict)
        )
        return float(candidates[int(np.argmin(costs))])

    def decide(self, context: StrategyContext) -> StrategyDecision:
        config = context.config
        strike_y = _strike_base_y(context)
        hitter_index = config.robots.index(context.hitter)
        teammate_index = config.robots.index(context.next_hitter)
        hitter_goal = (
            config.outward_y[hitter_index]
            if context.phase in {"post_hit", "follow_through"}
            else strike_y
        )
        if context.phase == "follow_through":
            teammate_goal = _clip_y(
                config.teammate_avoidance_y[teammate_index],
                config,
            )
            teammate_mode = "clear_hold"
        else:
            teammate_goal = self._select_teammate_goal(context, hitter_goal)
            teammate_mode = "preview" if context.phase == "post_hit" else "outward"
        nominal = {
            context.hitter: hitter_goal,
            context.next_hitter: teammate_goal,
        }
        left, right = config.robots
        weights = {context.hitter: 20.0, context.next_hitter: 1.0}
        left_y, right_y, projected = project_ordered_pair(
            nominal[left],
            nominal[right],
            config.min_separation + config.separation_margin,
            config.workspace_y,
            left_weight=weights[left],
            right_weight=weights[right],
        )
        goals = {
            left: _goal(context.feedback[left], left_y),
            right: _goal(context.feedback[right], right_y),
        }
        return StrategyDecision(
            goals=goals,
            fallbacks=("mpc_goal_projection",) if projected else (),
            diagnostics={
                "nominal_strike_y": float(context.prediction.position[1]),
                "nominal_base_y": strike_y,
                "preview_y": teammate_goal,
                "teammate_stage_mode": teammate_mode,
                "goal_gap": right_y - left_y,
            },
            use_cbf=True,
        )


class CBFStrategy(RelayHeuristicStrategy):
    """Relay heuristic nominal with a second-order CBF safety projection."""

    name = "cbf"

    def decide(self, context: StrategyContext) -> StrategyDecision:
        nominal = super().decide(context)
        return StrategyDecision(
            goals=nominal.goals,
            fallbacks=nominal.fallbacks,
            diagnostics={**nominal.diagnostics, "nominal_planner": "relay_heuristic"},
            use_cbf=True,
        )


class ReachabilityStrategy(RelayHeuristicStrategy):
    """Relay heuristic nominal with a robust viability safety projection."""

    name = "reachability"

    def decide(self, context: StrategyContext) -> StrategyDecision:
        nominal = super().decide(context)
        return StrategyDecision(
            goals=nominal.goals,
            fallbacks=nominal.fallbacks,
            diagnostics={**nominal.diagnostics, "nominal_planner": "relay_heuristic"},
            use_reachability=True,
        )


class RLStrategy(RelayHeuristicStrategy):
    """Frozen tabular-Q teammate staging policy with deterministic fallback."""

    name = "rl"

    def __init__(
        self,
        policy: FrozenQPolicy | None = None,
        minimum_visits: int | None = None,
    ) -> None:
        self.policy = policy or load_q_policy()
        if minimum_visits is None:
            minimum_visits = int(self.policy.metadata.get("minimum_deployment_visits", 1))
        if minimum_visits < 1:
            raise ValueError("minimum_visits must be at least 1")
        self.minimum_visits = minimum_visits
        self._cached_hitter: str | None = None
        self._cached_teammate_goal_y: float | None = None
        self._cached_policy_diagnostics: dict[str, float | int | bool] = {}
        self._last_time_to_strike: float | None = None

    def reset(self) -> None:
        self._cached_hitter = None
        self._cached_teammate_goal_y = None
        self._cached_policy_diagnostics = {}
        self._last_time_to_strike = None

    def decide(self, context: StrategyContext) -> StrategyDecision:
        nominal = super().decide(context)
        if context.phase == "follow_through":
            return nominal
        time_to_strike = float(context.prediction.time_to_strike)
        new_shot = (
            self._cached_hitter != context.hitter
            or self._last_time_to_strike is None
            or time_to_strike > self._last_time_to_strike + 0.12
        )
        if new_shot:
            left, right = context.config.robots
            left_state = context.feedback[left]
            right_state = context.feedback[right]
            goal, policy_diagnostics = self.policy.choose_goal(
                context.config.robots.index(context.hitter),
                float(context.prediction.position[1]),
                float(left_state.base_xy[1]),
                float(right_state.base_xy[1]),
                max(0.0, time_to_strike),
                float(right_state.velocity_xy[1] - left_state.velocity_xy[1]),
                minimum_visits=self.minimum_visits,
            )
            self._cached_hitter = context.hitter
            self._cached_teammate_goal_y = goal
            self._cached_policy_diagnostics = policy_diagnostics
        self._last_time_to_strike = time_to_strike

        diagnostics = {
            **nominal.diagnostics,
            **self._cached_policy_diagnostics,
            "rl_minimum_visits": self.minimum_visits,
            "nominal_planner": "tabular_q_learning",
        }
        if self._cached_teammate_goal_y is None:
            return StrategyDecision(
                goals=nominal.goals,
                fallbacks=nominal.fallbacks + ("rl_unseen_state",),
                diagnostics=diagnostics,
            )

        config = context.config
        left, right = config.robots
        candidate = {
            name: np.asarray(nominal.goals[name], dtype=float).copy()
            for name in config.robots
        }
        candidate[context.next_hitter][1] = _clip_y(
            self._cached_teammate_goal_y,
            config,
        )
        weights = {context.hitter: 20.0, context.next_hitter: 1.0}
        left_y, right_y, projected = project_ordered_pair(
            float(candidate[left][1]),
            float(candidate[right][1]),
            config.min_separation + config.separation_margin,
            config.workspace_y,
            left_weight=weights[left],
            right_weight=weights[right],
        )
        goals = {
            left: _goal(context.feedback[left], left_y),
            right: _goal(context.feedback[right], right_y),
        }
        diagnostics.update(
            {
                "rl_teammate_goal_y": self._cached_teammate_goal_y,
                "teammate_stage_mode": "learned",
                "teammate_stage_y": self._cached_teammate_goal_y,
                "goal_gap": right_y - left_y,
            }
        )
        fallbacks = nominal.fallbacks + (("rl_goal_projection",) if projected else ())
        return StrategyDecision(
            goals=goals,
            fallbacks=fallbacks,
            diagnostics=diagnostics,
        )


def make_strategy(name: str) -> PositionStrategy:
    normalized = name.strip().lower().replace("-", "_")
    if normalized in {"v0", "baseline"}:
        return V0Strategy()
    if normalized in {"heuristic", "relay_heuristic"}:
        return RelayHeuristicStrategy()
    if normalized == "cbf":
        return CBFStrategy()
    if normalized in {"mpc", "cbf_mpc"}:
        return CBFMPCStrategy()
    if normalized in {"reach", "viability", "reachability"}:
        return ReachabilityStrategy()
    if normalized in {"q", "q_learning", "rl"}:
        return RLStrategy()
    raise ValueError(f"unknown position strategy: {name}")
