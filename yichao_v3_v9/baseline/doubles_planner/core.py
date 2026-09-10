from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from .config import PlannerConfig
from .geometry import desired_base_y
from .models import (
    BallPrediction,
    PlanResult,
    RelayStage,
    RobotCommand,
    RobotFeedback,
)
from .reachability import reachability_waypoints
from .safety import cbf_waypoints
from .strategies import PositionStrategy, StrategyContext, make_strategy


_HIT_LOCKED_PHASES = {"HIT", "POST_DELAY"}
_RELEASED_PHASES = {"OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD"}


@dataclass
class _ShotSlot:
    shot_id: str | int
    hitter: str
    peer: str
    token: str
    reserved_at: float
    reserved_tts: float
    stage: RelayStage = RelayStage.RESERVED
    committed_at: float | None = None
    commit_tts: float | None = None
    observed_hit_phase: bool = False


class RelayPlanner:
    """Strict-alternation relay supervisor above a positioning strategy.

    A shot is assigned as soon as its prediction is armed, but the frozen hit
    policy is not activated until the supervisor emits a one-shot commit event.
    The peer clear command and hit commit therefore share the same decision
    boundary.  CBF/reachability remain downstream safety filters and never own
    the hitter token.
    """

    def __init__(
        self,
        config: PlannerConfig | None = None,
        strategy: str | PositionStrategy = "cbf_mpc",
    ) -> None:
        self.config = config or PlannerConfig()
        self.strategy = make_strategy(strategy) if isinstance(strategy, str) else strategy
        self._sequence = 0
        self._auto_shot_counter = 0
        self._reservation_counter = 0
        self._pending_shot: _ShotSlot | None = None
        self._active_shot: _ShotSlot | None = None
        self._turn_hitter: str | None = None
        self._last_hitter: str | None = None
        self._last_shot_start: float | None = None
        self._last_prediction_time: float | None = None
        self._last_time_to_strike: float | None = None
        self._last_commit_rejection: str | None = None
        self._last_valid_feedback: dict[str, RobotFeedback] = {}
        self._last_goals = {
            name: np.asarray([self.config.base_x, self.config.home_y[index]], dtype=float)
            for index, name in enumerate(self.config.robots)
        }

    def reset(self) -> None:
        reset_strategy = getattr(self.strategy, "reset", None)
        if callable(reset_strategy):
            reset_strategy()
        self._pending_shot = None
        self._active_shot = None
        self._turn_hitter = None
        self._last_hitter = None
        self._last_shot_start = None
        self._last_prediction_time = None
        self._last_time_to_strike = None
        self._last_commit_rejection = None
        self._last_valid_feedback.clear()
        self._last_goals = {
            name: np.asarray([self.config.base_x, self.config.home_y[index]], dtype=float)
            for index, name in enumerate(self.config.robots)
        }

    def defer_commit(self, token: str, reason: str) -> bool:
        """Return a bridge-rejected commit to the pending slot for retry."""

        slot = self._active_shot
        if slot is None or slot.token != token or slot.observed_hit_phase:
            return False
        slot.stage = RelayStage.PREPARED
        slot.committed_at = None
        slot.commit_tts = None
        self._active_shot = None
        self._pending_shot = slot
        self._last_commit_rejection = str(reason)
        return True

    def _other(self, robot: str) -> str:
        left, right = self.config.robots
        return right if robot == left else left

    def _resolve_feedback(
        self,
        feedback: Mapping[str, RobotFeedback],
        now: float,
    ) -> tuple[dict[str, RobotFeedback], tuple[str, ...], set[str]]:
        resolved: dict[str, RobotFeedback] = {}
        fallbacks: list[str] = []
        missing: set[str] = set()
        for index, name in enumerate(self.config.robots):
            state = feedback.get(name)
            fresh = (
                state is not None
                and state.name == name
                and state.valid
                and -0.05 <= now - state.timestamp <= self.config.feedback_timeout
            )
            if fresh:
                resolved[name] = state
                self._last_valid_feedback[name] = state
                continue

            missing.add(name)
            previous = self._last_valid_feedback.get(name)
            if previous is not None:
                base = previous.base_xy
                source = "last_feedback"
            elif name in self._last_goals:
                base = self._last_goals[name]
                source = "last_command"
            else:
                base = np.asarray([self.config.base_x, self.config.home_y[index]], dtype=float)
                source = "home"
            resolved[name] = RobotFeedback(
                name=name,
                base_xy=base,
                velocity_xy=np.zeros(2, dtype=float),
                timestamp=now,
                valid=False,
            )
            fallbacks.append(f"{name}_feedback_{source}")
        return resolved, tuple(fallbacks), missing

    def _safe_commands(
        self,
        feedback: Mapping[str, RobotFeedback],
        roles: Mapping[str, str] | None = None,
    ) -> dict[str, RobotCommand]:
        commands: dict[str, RobotCommand] = {}
        for index, name in enumerate(self.config.robots):
            base = feedback[name].base_xy.copy()
            safe_position = np.asarray(
                [self.config.safe_ball_x, self.config.home_y[index], self.config.safe_ball_z],
                dtype=float,
            )
            commands[name] = RobotCommand(
                robot=name,
                active=False,
                predicted_ball_position=safe_position,
                predicted_ball_velocity=np.zeros(3, dtype=float),
                predicted_ball_predict_time=self.config.inactive_time_to_strike,
                predicted_racket_normal=np.asarray([1.0, 0.0, 0.0], dtype=float),
                predicted_racket_velocity=np.zeros(3, dtype=float),
                desired_base_position=base,
                role=(roles or {}).get(name, "hold"),
            )
        return commands

    def safe_hold(
        self,
        feedback: Mapping[str, RobotFeedback],
        now: float,
        reason: str = "safe_hold",
    ) -> PlanResult:
        resolved, feedback_fallbacks, _ = self._resolve_feedback(feedback, now)
        self._sequence += 1
        visible = self._active_shot or self._pending_shot
        return PlanResult(
            sequence=self._sequence,
            shot_id=visible.shot_id if visible is not None else None,
            hitter=None,
            next_hitter=visible.peer if visible is not None else self._turn_hitter,
            phase="safe_hold",
            strategy=self.strategy.name,
            commands=self._safe_commands(resolved),
            fallbacks=feedback_fallbacks + (reason,),
            relay_stage=RelayStage.IDLE.value,
            commit_token=visible.token if visible is not None else None,
            pending_shot_id=(
                self._pending_shot.shot_id if self._pending_shot is not None else None
            ),
        )

    def _prediction_is_fresh(self, prediction: BallPrediction, now: float) -> bool:
        age = now - prediction.timestamp
        return -0.05 <= age <= self.config.prediction_timeout

    def _incoming_shot_id(self, prediction: BallPrediction) -> str | int | None:
        if prediction.shot_id is not None:
            if prediction.time_to_strike < self.config.shot_arm_time:
                return None
            return prediction.shot_id
        rising = (
            self._last_time_to_strike is None
            or prediction.time_to_strike > self._last_time_to_strike + 0.12
        )
        if (
            self._pending_shot is None
            and prediction.time_to_strike >= self.config.shot_arm_time
            and rising
        ):
            self._auto_shot_counter += 1
            return f"auto-{self._auto_shot_counter}"
        return None

    def _choose_first_hitter(
        self,
        prediction: BallPrediction,
        feedback: Mapping[str, RobotFeedback],
    ) -> str:
        scores = []
        for index, name in enumerate(self.config.robots):
            target_y = desired_base_y(self.config, name, float(prediction.position[1]))
            distance = abs(float(feedback[name].base_xy[1]) - target_y)
            travel_time = distance / self.config.max_base_speed
            unreachable = travel_time > max(0.0, prediction.time_to_strike) + 0.12
            scores.append((unreachable, travel_time, distance, index, name))
        return min(scores)[-1]

    def _recover_initial_hitter(
        self,
        prediction: BallPrediction,
        feedback: Mapping[str, RobotFeedback],
    ) -> str:
        home_ready = [
            name
            for name in self.config.robots
            if feedback[name].valid
            and feedback[name].controller_phase == "HOME_HOLD"
            and feedback[name].ready is not False
        ]
        if len(home_ready) == 1:
            return home_ready[0]
        preferred = self.config.initial_hitter
        if preferred is not None and (not home_ready or preferred in home_ready):
            return preferred
        return self._choose_first_hitter(prediction, feedback)

    def _reserve_new_shot(
        self,
        shot_id: str | int,
        prediction: BallPrediction,
        feedback: Mapping[str, RobotFeedback],
        now: float,
    ) -> _ShotSlot:
        rally_stale = (
            self._last_shot_start is None
            or now - self._last_shot_start > self.config.first_hit_reset_seconds
        )
        if self._active_shot is not None:
            hitter = self._other(self._active_shot.hitter)
        elif rally_stale or self._turn_hitter is None:
            hitter = self._recover_initial_hitter(prediction, feedback)
            self._turn_hitter = hitter
        else:
            hitter = self._turn_hitter
        self._reservation_counter += 1
        slot = _ShotSlot(
            shot_id=shot_id,
            hitter=hitter,
            peer=self._other(hitter),
            token=f"relay-{self._reservation_counter}:{hitter}:{shot_id}",
            reserved_at=now,
            reserved_tts=float(prediction.time_to_strike),
        )
        self._pending_shot = slot
        self._last_shot_start = now
        return slot

    def _commit_admission(
        self,
        slot: _ShotSlot,
        prediction: BallPrediction,
        feedback: Mapping[str, RobotFeedback],
        missing: set[str],
    ) -> dict[str, Any]:
        hitter_feedback = feedback[slot.hitter]
        peer_feedback = feedback[slot.peer]
        hitter_phase = hitter_feedback.controller_phase
        peer_phase = peer_feedback.controller_phase
        hit_preemptible = hitter_phase is None or hitter_phase in _RELEASED_PHASES
        stationary_ready = (
            hitter_feedback.ready is not False
            if hitter_phase in {None, "HOME_HOLD"}
            else True
        )
        hitter_ready = (
            hit_preemptible and stationary_ready and slot.hitter not in missing
        )
        peer_exclusive = peer_phase not in _HIT_LOCKED_PHASES and slot.peer not in missing
        peer_home_ready = (
            peer_phase in {None, "HOME_HOLD"}
            and peer_feedback.ready is not False
            and slot.peer not in missing
        )
        peer_index = self.config.robots.index(slot.peer)
        clear_y = float(self.config.teammate_avoidance_y[peer_index])
        peer_y = float(peer_feedback.base_xy[1])
        beyond_clear_lane = peer_y <= clear_y if clear_y < 0.0 else peer_y >= clear_y
        peer_outward_ready = bool(
            self.config.allow_peer_outward_hold_ready
            and peer_phase == "OUTWARD_HOLD"
            and peer_feedback.ready is not False
            and slot.peer not in missing
            and beyond_clear_lane
            and abs(float(peer_feedback.velocity_xy[1])) <= self.config.peer_clear_max_speed
        )
        peer_ready = peer_home_ready or peer_outward_ready

        displacement = float(peer_feedback.base_xy[1] - hitter_feedback.base_xy[1])
        relative_velocity = float(
            peer_feedback.velocity_xy[1] - hitter_feedback.velocity_xy[1]
        )
        base_gap = abs(displacement)
        direction = float(np.sign(displacement))
        closing_speed = max(0.0, -direction * relative_velocity)
        base_ttc = float("inf") if closing_speed <= 1.0e-9 else base_gap / closing_speed
        hard_distance = self.config.collision_distance + self.config.hard_safety_margin
        base_safe = base_gap >= hard_distance
        ttc_safe = base_ttc >= self.config.commit_minimum_base_ttc

        target_y = desired_base_y(
            self.config,
            slot.hitter,
            float(prediction.position[1]),
        )
        travel_distance = abs(float(hitter_feedback.base_xy[1]) - target_y)
        travel_time = travel_distance / self.config.max_base_speed
        strike_reachable = travel_time <= max(0.0, prediction.time_to_strike) + 0.12

        reasons: list[str] = []
        if not hitter_ready:
            reasons.append("hitter_not_ready")
        if not peer_exclusive:
            reasons.append("peer_hit_locked")
        elif not peer_ready:
            reasons.append("peer_not_ready")
        if not base_safe:
            reasons.append("base_hard_margin")
        if not ttc_safe:
            reasons.append("base_ttc")
        if not strike_reachable:
            reasons.append("strike_unreachable")
        return {
            "admitted": not reasons,
            "reasons": tuple(reasons),
            "hitter_ready": hitter_ready,
            "peer_exclusive": peer_exclusive,
            "peer_ready": peer_ready,
            "peer_home_ready": peer_home_ready,
            "peer_outward_ready": peer_outward_ready,
            "hitter_controller_phase": hitter_phase or "unknown",
            "peer_controller_phase": peer_phase or "unknown",
            "base_gap": base_gap,
            "base_closing_speed": closing_speed,
            "base_ttc": base_ttc,
            "hard_safety_distance": hard_distance,
            "strike_travel_time": travel_time,
            "strike_reachable": strike_reachable,
        }

    def _complete_active(self, verified: bool) -> _ShotSlot:
        slot = self._active_shot
        if slot is None:
            raise RuntimeError("no active shot to complete")
        slot.stage = RelayStage.HANDOFF if verified else RelayStage.ABORTED
        if verified:
            self._last_hitter = slot.hitter
            self._turn_hitter = slot.peer
        self._active_shot = None
        return slot

    def _advance_active_stage(
        self,
        prediction: BallPrediction,
        feedback: Mapping[str, RobotFeedback],
        now: float,
    ) -> _ShotSlot | None:
        slot = self._active_shot
        if slot is None:
            return None
        phase = feedback[slot.hitter].controller_phase
        same_prediction = prediction.shot_id in {None, slot.shot_id}
        if phase in _HIT_LOCKED_PHASES:
            slot.observed_hit_phase = True
            slot.stage = (
                RelayStage.STRIKE if phase == "HIT" else RelayStage.FOLLOW_THROUGH
            )
        elif slot.observed_hit_phase and phase in _RELEASED_PHASES:
            return self._complete_active(verified=True)
        elif phase is None and same_prediction:
            slot.stage = (
                RelayStage.STRIKE
                if prediction.time_to_strike >= 0.0
                else RelayStage.FOLLOW_THROUGH
            )
            if prediction.time_to_strike <= self.config.shot_release_time:
                return self._complete_active(verified=True)

        if (
            slot.committed_at is not None
            and now - slot.committed_at > self.config.commit_handoff_timeout
        ):
            return self._complete_active(verified=False)
        return None

    def _transition_result(
        self,
        slot: _ShotSlot,
        feedback: Mapping[str, RobotFeedback],
        fallbacks: tuple[str, ...],
    ) -> PlanResult:
        self._sequence += 1
        verified = slot.stage == RelayStage.HANDOFF
        if verified:
            transition_fallbacks: tuple[str, ...] = ()
        elif slot.committed_at is None:
            transition_fallbacks = ("late_commit_aborted",)
        else:
            transition_fallbacks = ("commit_handoff_timeout",)
        return PlanResult(
            sequence=self._sequence,
            shot_id=slot.shot_id,
            hitter=slot.hitter,
            next_hitter=slot.peer,
            phase=slot.stage.value,
            strategy=self.strategy.name,
            commands=self._safe_commands(feedback),
            fallbacks=fallbacks + transition_fallbacks,
            diagnostics={
                "relay_stage": slot.stage.value,
                "commit_token": slot.token,
                "handoff_verified": verified,
                "turn_token": self._turn_hitter,
            },
            relay_stage=slot.stage.value,
            commit_token=slot.token,
            pending_shot_id=(
                self._pending_shot.shot_id if self._pending_shot is not None else None
            ),
        )

    def _pending_result(
        self,
        slot: _ShotSlot,
        feedback: Mapping[str, RobotFeedback],
        feedback_fallbacks: tuple[str, ...],
        admission: Mapping[str, Any],
    ) -> PlanResult:
        roles = {slot.hitter: "reserved_hitter", slot.peer: "hold"}
        diagnostics = {
            **admission,
            "relay_stage": slot.stage.value,
            "commit_token": slot.token,
            "turn_token": slot.hitter,
            "hitter_locked": True,
            "reservation_lead_s": slot.reserved_tts,
            "commit_threshold_s": self.config.commit_time_to_strike,
            "peer_clear_trigger": "commit",
        }
        if self._last_commit_rejection is not None:
            diagnostics["last_commit_rejection"] = self._last_commit_rejection
        self._sequence += 1
        return PlanResult(
            sequence=self._sequence,
            shot_id=slot.shot_id,
            hitter=slot.hitter,
            next_hitter=slot.peer,
            phase="reservation",
            strategy=self.strategy.name,
            commands=self._safe_commands(feedback, roles),
            fallbacks=feedback_fallbacks,
            diagnostics=diagnostics,
            relay_stage=slot.stage.value,
            commit_token=slot.token,
            pending_shot_id=slot.shot_id,
        )

    def _make_commands(
        self,
        prediction: BallPrediction,
        hitter: str,
        goals: Mapping[str, np.ndarray],
        trajectory_goals: Mapping[str, np.ndarray],
        stage: RelayStage,
    ) -> dict[str, RobotCommand]:
        commands: dict[str, RobotCommand] = {}
        for index, name in enumerate(self.config.robots):
            active = name == hitter
            if active:
                ball_position = prediction.position
                ball_velocity = prediction.velocity
                predict_time = prediction.time_to_strike
                racket_normal = prediction.racket_normal
                racket_velocity = prediction.racket_velocity
                role = "exit" if stage == RelayStage.FOLLOW_THROUGH else "hit"
            else:
                ball_position = np.asarray(
                    [self.config.safe_ball_x, self.config.home_y[index], self.config.safe_ball_z],
                    dtype=float,
                )
                ball_velocity = np.zeros(3, dtype=float)
                predict_time = self.config.inactive_time_to_strike
                racket_normal = np.asarray([1.0, 0.0, 0.0], dtype=float)
                racket_velocity = np.zeros(3, dtype=float)
                role = "clear"
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

    def _active_result(
        self,
        prediction: BallPrediction,
        resolved: Mapping[str, RobotFeedback],
        feedback_fallbacks: tuple[str, ...],
        missing: set[str],
        commit_requested: bool,
        admission: Mapping[str, Any],
    ) -> PlanResult:
        slot = self._active_shot
        if slot is None:
            raise RuntimeError("active result requested without an active shot")
        follow_through = slot.stage == RelayStage.FOLLOW_THROUGH
        strategy_phase = "follow_through" if follow_through else "pre_hit"
        context = StrategyContext(
            config=self.config,
            phase=strategy_phase,
            hitter=slot.hitter,
            next_hitter=slot.peer,
            prediction=prediction,
            feedback=resolved,
            previous_goals=self._last_goals,
        )
        decision = self.strategy.decide(context)
        trajectory_goals = {
            name: np.asarray(decision.goals[name], dtype=float).copy()
            for name in self.config.robots
        }
        goals = {name: goal.copy() for name, goal in trajectory_goals.items()}
        diagnostics = {
            **decision.diagnostics,
            **admission,
            "relay_stage": slot.stage.value,
            "commit_token": slot.token,
            "commit_requested": commit_requested,
            "turn_token": slot.hitter,
            "hitter_locked": True,
            "reservation_lead_s": slot.reserved_tts,
            "commit_tts_s": slot.commit_tts,
            "peer_clear_trigger": "commit",
        }
        if decision.use_cbf and decision.use_reachability:
            raise ValueError("a strategy cannot request CBF and reachability simultaneously")

        controlled_robot = slot.peer
        if decision.use_cbf:
            goals, safety_diagnostics = cbf_waypoints(
                resolved,
                goals,
                self.config,
                slot.hitter,
                controlled_robot=controlled_robot,
            )
            diagnostics.update(safety_diagnostics)
            diagnostics["cbf_controlled_robot"] = controlled_robot
            diagnostics.update(
                {
                    "safety_filter": "cbf",
                    "safety_active": bool(safety_diagnostics["cbf_active"]),
                    "safety_emergency": bool(safety_diagnostics["cbf_emergency"]),
                    "safety_controlled_robot": controlled_robot,
                    "emergency_abort_authorized": False,
                }
            )
        elif decision.use_reachability:
            goals, safety_diagnostics = reachability_waypoints(
                resolved,
                goals,
                self.config,
                slot.hitter,
                controlled_robot=controlled_robot,
            )
            diagnostics.update(safety_diagnostics)
            diagnostics.update(
                {
                    "safety_filter": "reachability",
                    "safety_active": bool(safety_diagnostics["reachability_active"]),
                    "safety_emergency": bool(
                        safety_diagnostics["reachability_emergency"]
                    ),
                    "safety_controlled_robot": controlled_robot,
                    "emergency_abort_authorized": False,
                }
            )

        if self.config.hold_when_feedback_missing and bool(missing):
            goals = {name: resolved[name].base_xy.copy() for name in self.config.robots}
            trajectory_goals = {
                name: resolved[name].base_xy.copy() for name in self.config.robots
            }
            diagnostics["feedback_safety_hold"] = True

        self._last_goals = {name: goal.copy() for name, goal in goals.items()}
        commands = self._make_commands(
            prediction,
            slot.hitter,
            goals,
            trajectory_goals,
            slot.stage,
        )
        self._sequence += 1
        return PlanResult(
            sequence=self._sequence,
            shot_id=slot.shot_id,
            hitter=slot.hitter,
            next_hitter=slot.peer,
            phase="post_hit" if follow_through else "pre_hit",
            strategy=self.strategy.name,
            commands=commands,
            fallbacks=feedback_fallbacks + decision.fallbacks,
            diagnostics=diagnostics,
            relay_stage=slot.stage.value,
            commit_token=slot.token,
            commit_requested=commit_requested,
            pending_shot_id=(
                self._pending_shot.shot_id if self._pending_shot is not None else None
            ),
        )

    def plan(
        self,
        prediction: BallPrediction | None,
        feedback: Mapping[str, RobotFeedback],
        now: float,
    ) -> PlanResult:
        if not np.isfinite(now):
            raise ValueError("now must be finite")
        resolved, feedback_fallbacks, missing = self._resolve_feedback(feedback, now)
        if prediction is None:
            return self.safe_hold(resolved, now, reason="missing_prediction")
        if not self._prediction_is_fresh(prediction, now):
            return self.safe_hold(resolved, now, reason="stale_prediction")

        transition = self._advance_active_stage(prediction, resolved, now)
        incoming_id = self._incoming_shot_id(prediction)
        known_ids = {
            slot.shot_id
            for slot in (self._active_shot, self._pending_shot)
            if slot is not None
        }
        if incoming_id is not None and incoming_id not in known_ids:
            if self._pending_shot is None:
                self._reserve_new_shot(incoming_id, prediction, resolved, now)

        if (
            self._active_shot is not None
            and prediction.shot_id not in {None, self._active_shot.shot_id}
        ):
            if self._pending_shot is not None:
                admission = self._commit_admission(
                    self._pending_shot, prediction, resolved, missing
                )
                admission = {
                    **admission,
                    "active_commit_token": self._active_shot.token,
                    "commit_blocked_by_active_shot": True,
                }
                result = self._pending_result(
                    self._pending_shot,
                    resolved,
                    feedback_fallbacks,
                    admission,
                )
                self._last_prediction_time = now
                self._last_time_to_strike = prediction.time_to_strike
                return result

        if self._active_shot is not None:
            admission = self._commit_admission(
                self._active_shot, prediction, resolved, missing
            )
            result = self._active_result(
                prediction,
                resolved,
                feedback_fallbacks,
                missing,
                commit_requested=False,
                admission=admission,
            )
        elif self._pending_shot is not None:
            slot = self._pending_shot
            admission = self._commit_admission(slot, prediction, resolved, missing)
            if prediction.time_to_strike < self.config.commit_abort_time_to_strike:
                slot.stage = RelayStage.ABORTED
                self._pending_shot = None
                result = self._transition_result(
                    slot,
                    resolved,
                    feedback_fallbacks + tuple(admission["reasons"]),
                )
            elif (
                prediction.time_to_strike <= self.config.commit_time_to_strike
                and bool(admission["admitted"])
            ):
                slot.stage = RelayStage.COMMITTED
                slot.committed_at = now
                slot.commit_tts = float(prediction.time_to_strike)
                self._active_shot = slot
                self._pending_shot = None
                self._last_commit_rejection = None
                result = self._active_result(
                    prediction,
                    resolved,
                    feedback_fallbacks,
                    missing,
                    commit_requested=True,
                    admission=admission,
                )
            else:
                slot.stage = (
                    RelayStage.PREPARED
                    if bool(admission["admitted"])
                    else RelayStage.RESERVED
                )
                result = self._pending_result(
                    slot,
                    resolved,
                    feedback_fallbacks,
                    admission,
                )
        elif transition is not None:
            result = self._transition_result(transition, resolved, feedback_fallbacks)
        else:
            result = self.safe_hold(resolved, now, reason="shot_not_armed")

        self._last_prediction_time = now
        self._last_time_to_strike = prediction.time_to_strike
        return result
