from __future__ import annotations

from dataclasses import replace
from typing import Any, Mapping

import numpy as np

from .config import PlannerConfig
from .core import RelayPlanner
from .models import BallPrediction, RelayStage, RobotCommand, RobotFeedback
from .strategies import RelayHeuristicStrategy, StrategyContext, StrategyDecision


_HIT_LOCKED_PHASES = {"HIT", "POST_DELAY"}
_HIT_PREEMPTIBLE_PHASES = {"OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD"}
_RELEASED_PHASES = {"OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD"}


class _FixedRelayPositionStrategy(RelayHeuristicStrategy):
    """Use the real outward lanes and treat ordered projection as a safe result."""

    name = "fixed_relay"

    def decide(self, context: StrategyContext) -> StrategyDecision:
        fixed_config = replace(
            context.config,
            teammate_avoidance_y=context.config.outward_y,
        )
        decision = super().decide(replace(context, config=fixed_config))
        projected = "ordered_goal_projection" in decision.fallbacks
        return replace(
            decision,
            fallbacks=tuple(
                reason
                for reason in decision.fallbacks
                if reason != "ordered_goal_projection"
            ),
            diagnostics={
                **decision.diagnostics,
                "ordered_goal_projected": projected,
                "fixed_relay_lane_source": "outward_y",
            },
        )


class FixedRelayPlanner(RelayPlanner):
    """Strict relay with fixed deploy-owned home and outward targets."""

    def __init__(self, config: PlannerConfig | None = None) -> None:
        super().__init__(config=config, strategy=_FixedRelayPositionStrategy())

    def _safe_commands(
        self,
        feedback: Mapping[str, RobotFeedback],
        roles: Mapping[str, str] | None = None,
    ) -> dict[str, RobotCommand]:
        return super()._safe_commands(
            feedback,
            roles={name: "hold" for name in self.config.robots},
        )

    def _recover_initial_hitter(
        self,
        prediction: BallPrediction,
        feedback: Mapping[str, RobotFeedback],
    ) -> str:
        candidates = []
        for index, name in enumerate(self.config.robots):
            state = feedback[name]
            if not state.valid or state.controller_phase not in _HIT_PREEMPTIBLE_PHASES:
                continue
            tie_priority = 0 if name == self.config.initial_hitter else 1
            candidates.append(
                (abs(float(state.base_xy[1])), tie_priority, index, name)
            )
        if candidates:
            return min(candidates)[-1]
        return super()._recover_initial_hitter(prediction, feedback)

    def _commit_admission(
        self,
        slot,
        prediction: BallPrediction,
        feedback: Mapping[str, RobotFeedback],
        missing: set[str],
    ) -> dict[str, Any]:
        hitter_feedback = feedback[slot.hitter]
        peer_feedback = feedback[slot.peer]
        hitter_phase = hitter_feedback.controller_phase
        peer_phase = peer_feedback.controller_phase
        hitter_preemptible = (
            hitter_phase in _HIT_PREEMPTIBLE_PHASES
            and slot.hitter not in missing
            and (
                hitter_phase != "OUTWARD_HOLD"
                or hitter_feedback.ready is not False
            )
        )
        peer_available = (
            peer_phase not in _HIT_LOCKED_PHASES
            and slot.peer not in missing
        )
        reasons = []
        if not hitter_preemptible:
            reasons.append(
                "hitter_hit_locked"
                if hitter_phase in _HIT_LOCKED_PHASES
                else "hitter_not_preemptible"
            )
        if not peer_available:
            reasons.append(
                "peer_hit_locked"
                if peer_phase in _HIT_LOCKED_PHASES
                else "peer_state_missing"
            )
        return {
            "admitted": not reasons,
            "reasons": tuple(reasons),
            "hitter_ready": hitter_preemptible,
            "peer_exclusive": peer_phase not in _HIT_LOCKED_PHASES,
            "peer_ready": peer_available,
            "peer_home_ready": peer_phase == "HOME_HOLD",
            "peer_outward_ready": peer_phase == "OUTWARD_HOLD",
            "hitter_controller_phase": hitter_phase or "unknown",
            "peer_controller_phase": peer_phase or "unknown",
            "fixed_relay": True,
            "cbf_active": False,
            "safety_active": False,
            "strike_reachable": True,
        }

    def safe_hold(
        self,
        feedback: Mapping[str, RobotFeedback],
        now: float,
        reason: str = "safe_hold",
    ):
        lifecycle = []
        if "ball_prediction_invalid" in reason:
            if self._pending_shot is not None:
                self._pending_shot.stage = RelayStage.ABORTED
                self._pending_shot = None
                lifecycle.append("pending_shot_cancelled")

            slot = self._active_shot
            state = None if slot is None else feedback.get(slot.hitter)
            phase = None if state is None or not state.valid else state.controller_phase
            if slot is not None and phase in _HIT_LOCKED_PHASES:
                slot.observed_hit_phase = True
                slot.stage = (
                    RelayStage.STRIKE
                    if phase == "HIT"
                    else RelayStage.FOLLOW_THROUGH
                )
            elif (
                slot is not None
                and slot.observed_hit_phase
                and phase in _RELEASED_PHASES
            ):
                self._complete_active(verified=True)
                lifecycle.append("active_handoff_completed")
            elif (
                slot is not None
                and slot.committed_at is not None
                and now - slot.committed_at > self.config.commit_handoff_timeout
            ):
                self._complete_active(verified=False)
                lifecycle.append("active_handoff_timeout")

        if lifecycle:
            reason = reason + "," + ",".join(lifecycle)
        return super().safe_hold(feedback, now, reason=reason)

    def _make_commands(
        self,
        prediction: BallPrediction,
        hitter: str,
        goals: Mapping[str, np.ndarray],
        trajectory_goals: Mapping[str, np.ndarray],
        stage: RelayStage,
    ) -> dict[str, RobotCommand]:
        commands: dict[str, RobotCommand] = {}
        hit_active = stage != RelayStage.FOLLOW_THROUGH
        for index, name in enumerate(self.config.robots):
            active = bool(name == hitter and hit_active)
            if active:
                ball_position = prediction.position
                ball_velocity = prediction.velocity
                predict_time = prediction.time_to_strike
                racket_normal = prediction.racket_normal
                racket_velocity = prediction.racket_velocity
                role = "hit"
            else:
                ball_position = np.asarray(
                    [
                        self.config.safe_ball_x,
                        self.config.home_y[index],
                        self.config.safe_ball_z,
                    ],
                    dtype=float,
                )
                ball_velocity = np.zeros(3, dtype=float)
                predict_time = self.config.inactive_time_to_strike
                racket_normal = np.asarray([1.0, 0.0, 0.0], dtype=float)
                racket_velocity = np.zeros(3, dtype=float)
                role = "hold"
            commands[name] = RobotCommand(
                robot=name,
                active=active,
                predicted_ball_position=ball_position,
                predicted_ball_velocity=ball_velocity,
                predicted_ball_predict_time=predict_time,
                predicted_racket_normal=racket_normal,
                predicted_racket_velocity=racket_velocity,
                desired_base_position=goals[name],
                trajectory_base_position=trajectory_goals[name],
                role=role,
            )
        return commands


__all__ = ["FixedRelayPlanner"]
