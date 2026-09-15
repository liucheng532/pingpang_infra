"""V7 actor + frozen CBF adapter around the unchanged Fixed relay state machine."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import replace
import math
import time
from typing import Any, Mapping

import numpy as np

from . import (
    ACTION_INDEX,
    HOME_PAIR,
    MINIMUM_GAP_M,
    MODEL_ORDER,
    MODEL_TO_ROS,
    PLANNER_IDENTITY,
    ROS_TO_MODEL,
    WORKSPACE_Y,
)
from .actor import OnnxActor, OnnxBarrier
from .bootstrap import install_fixed_runtime
from .features import FeatureBatch, FeatureTracker
from .guard import project_v7_pair

install_fixed_runtime()

from doubles_planner.fixed_relay import FixedRelayPlanner  # noqa: E402
from doubles_planner.real_ros_runtime import (  # noqa: E402
    ROBOT_ORDER,
    V9RealFixedRelayRuntime,
)
from doubles_planner.runtime_motion_config import (  # noqa: E402
    default_runtime_motion_configs,
)
from doubles_planner.strategies import StrategyDecision  # noqa: E402


LOCKED_PHASES = {"HIT", "POST_DELAY"}
HOLD_PHASES = {"HOME_HOLD", "OUTWARD_HOLD"}
MODES = {"shadow", "transport-shadow", "active"}
ACK_TIMEOUT_S = 0.25
ACK_TARGET_TOLERANCE_M = 1.0e-4
UNTRUSTED_RECOVERY_PREFIXES = (
    "controller_restart_or_clock_regression:",
    "controller_applied_sequence_regression:",
    "controller_session_regression:",
    "stage_state_missing:",
    "stage_state_invalid_or_stale:",
)


class V7SafetyPipeline:
    def __init__(self, actor: OnnxActor, barrier: OnnxBarrier) -> None:
        self.actor = actor
        self.barrier = barrier

    def decide(self, features: FeatureBatch, hitter: str) -> dict[str, Any]:
        started = time.perf_counter()
        raw = self.actor(features.actor)[0]
        result = project_v7_pair(raw, features.safe, self.barrier, hitter)
        result.update(
            {
                "planner_identity": PLANNER_IDENTITY,
                "model_order": list(MODEL_ORDER),
                "ros_mapping": dict(MODEL_TO_ROS),
                "action_index": dict(ACTION_INDEX),
                "latency_ms": (time.perf_counter() - started) * 1000.0,
            }
        )
        return result


class _ActorPositionStrategy:
    name = "v7_model190_frozen_cbf"

    def __init__(self) -> None:
        self.decision: dict[str, Any] | None = None

    def decide(self, context) -> StrategyDecision:
        decision = self.decision
        if not decision or not decision.get("valid"):
            return StrategyDecision(
                {name: state.base_xy.copy() for name, state in context.feedback.items()},
                fallbacks=("actor_pair_unavailable",),
            )
        pair = np.asarray(decision["applied_pair_m"], dtype=np.float64)
        goals = {}
        for name, state in context.feedback.items():
            index = ACTION_INDEX[ROS_TO_MODEL[name]]
            goals[name] = np.asarray([state.base_xy[0], pair[index]], dtype=np.float64)
        return StrategyDecision(goals, diagnostics={"v7": decision})


class _StagedFixedRelayPlanner(FixedRelayPlanner):
    """Delay only Fixed's commit edge until both stage commands are accepted."""

    def __init__(self, config, strategy: _ActorPositionStrategy) -> None:
        super().__init__(config)
        self.strategy = strategy
        self.prepared_tokens: set[str] = set()
        self.prepare_targets: dict[str, dict[str, float]] = {}
        self.last_rule_commands: dict[str, Any] = {}

    def reset(self) -> None:
        super().reset()
        self.prepared_tokens.clear()
        self.prepare_targets.clear()
        self.last_rule_commands.clear()

    def abort_pending(self, token: str | None) -> None:
        slot = self._pending_shot
        if slot is not None and (token is None or slot.token == token):
            self._pending_shot = None
        if token is not None:
            self.prepared_tokens.discard(token)
            self.prepare_targets.pop(token, None)

    def _complete_active(self, verified):
        slot = super()._complete_active(verified)
        self.prepared_tokens.discard(slot.token)
        self.prepare_targets.pop(slot.token, None)
        return slot

    def _commit_admission(self, slot, prediction, feedback, missing):
        admission = super()._commit_admission(slot, prediction, feedback, missing)
        if slot.token not in self.prepared_tokens:
            admission["admitted"] = False
            admission["reasons"] = (*admission["reasons"], "pair_stage_ack_pending")
        return admission

    def _pending_result(self, slot, feedback, feedback_fallbacks, admission):
        result = super()._pending_result(slot, feedback, feedback_fallbacks, admission)
        # Preserve the unmodified Fixed rule-based pair before replacing it
        # with the jointly staged Actor targets.  This is observation only.
        self.last_rule_commands = {
            name: deepcopy(command) for name, command in result.commands.items()
        }
        decision = self.strategy.decision
        if (
            self._active_shot is not None
            or feedback_fallbacks
            or not decision
            or not decision.get("valid")
            or any(
                not state.valid or state.controller_phase in LOCKED_PHASES
                for state in feedback.values()
            )
        ):
            return result
        pair = np.asarray(decision["applied_pair_m"], dtype=np.float64)
        commands = {}
        targets = {}
        for name in ROBOT_ORDER:
            index = ACTION_INDEX[ROS_TO_MODEL[name]]
            target = float(pair[index])
            goal = np.asarray([feedback[name].base_xy[0], target], dtype=np.float64)
            commands[name] = replace(
                result.commands[name],
                role="stage",
                active=False,
                desired_base_position=goal,
                trajectory_base_position=goal,
            )
            targets[name] = target
        self.prepare_targets[slot.token] = targets
        return replace(result, commands=commands)


class V7Runtime(V9RealFixedRelayRuntime):
    """Three-mode runtime with one-shot V7 decisions and atomic pair ACK."""

    def __init__(
        self,
        *,
        mode: str = "shadow",
        session_id: str | None = None,
        pipeline: V7SafetyPipeline,
        feature_tracker: FeatureTracker | None = None,
        attestation_verified: bool = False,
    ) -> None:
        if mode not in MODES:
            raise ValueError(f"mode must be one of {sorted(MODES)}")
        if mode != "shadow" and not attestation_verified:
            raise ValueError(f"{mode} requires a verified V11 process attestation")
        super().__init__(shadow=mode == "shadow", session_id=session_id)
        self.mode = mode
        # Keep the physical Fixed/V3 relay lanes separate from the V7 model
        # contract.  HOME_PAIR (+/-0.35) seeds model history / the final CBF
        # candidate, while WORKSPACE_Y (+/-0.90) remains the Actor boundary.
        # Physical fault recovery uses the role-aware Fixed/V3 lanes below.
        self.config = replace(
            self.config,
            home_y=(-0.20, 0.20),
            ready_y=(-0.20, 0.20),
            outward_y=(-0.70, 0.70),
            teammate_avoidance_y=(-0.65, 0.65),
            workspace_y=WORKSPACE_Y,
            min_separation=MINIMUM_GAP_M,
            separation_margin=0.0,
        )
        self.position_strategy = _ActorPositionStrategy()
        self.planner = _StagedFixedRelayPlanner(self.config, self.position_strategy)
        self.planner_mode = "fixed_relay"
        self._motion_config_active = default_runtime_motion_configs(self.config)
        self.pipeline = pipeline
        self.features = feature_tracker or FeatureTracker()
        self.last_features: FeatureBatch | None = None
        self.last_inference: dict[str, Any] | None = None
        self.reason = "waiting_inputs"
        self.failed_shots: dict[str | int, str] = {}
        self.completed_transport_shots: set[str | int] = set()
        self._shot_first_received: dict[str | int, float] = {}
        self._decisions: dict[str | int, dict[str, Any]] = {}
        self._decision_features: dict[str | int, FeatureBatch] = {}
        self._staging: dict[str, Any] | None = None
        self._controller_markers: dict[str, tuple[int, int]] = {}
        self._applied_markers: dict[str, int] = {}
        self._controller_session_seen: set[str] = set()
        self._restart_fault: str | None = None
        self._stage_failed_this_tick = False
        self._now = 0.0

    @property
    def command_publish_enabled(self) -> bool:
        return self.mode in {"transport-shadow", "active"}

    def stage_return_timing_config(self, payload):
        raise ValueError("v7 return timing is fixed to peer_outward")

    def return_timing_status(self):
        return {
            robot: {
                "available": False,
                "mode": "peer_outward",
                "blocked_reason": "v7_disables_early_return",
            }
            for robot in ROBOT_ORDER
        }

    def update_robot_state(
        self,
        robot: str,
        payload: Mapping[str, Any],
        receive_monotonic: float | None = None,
    ) -> bool:
        received = time.monotonic() if receive_monotonic is None else float(receive_monotonic)
        sequence = int(payload.get("sequence", -1))
        source_ns = int(payload.get("source_monotonic_ns", -1))
        old = self._controller_markers.get(robot)
        if old is not None and (sequence < old[0] or source_ns <= old[1]):
            self._restart_fault = f"controller_restart_or_clock_regression:{robot}"
        accepted = super().update_robot_state(robot, payload, received)
        if accepted:
            self._controller_markers[robot] = (sequence, source_ns)
            planner_session = payload.get("last_planner_session_id")
            applied_sequence = payload.get("last_applied_sequence")
            if planner_session == self.session_id and type(applied_sequence) is int:
                previous_applied = self._applied_markers.get(robot)
                if previous_applied is not None and applied_sequence < previous_applied:
                    self._restart_fault = f"controller_applied_sequence_regression:{robot}"
                self._applied_markers[robot] = applied_sequence
                self._controller_session_seen.add(robot)
            elif robot in self._controller_session_seen:
                self._restart_fault = f"controller_session_regression:{robot}"
            try:
                self.features.ingest(robot, self._states[robot], received)
            except ValueError as error:
                self._restart_fault = str(error)
        return accepted

    def update_ball(
        self,
        payload: Mapping[str, Any],
        receive_monotonic: float | None = None,
    ) -> bool:
        accepted = super().update_ball(payload, receive_monotonic)
        if not accepted:
            return False
        if self._ball is not None and self._ball.get("valid"):
            shot_id = self._ball.get("shot_id")
            if shot_id is not None:
                first_received = self._shot_first_received.setdefault(
                    shot_id, float(self._ball["_receive_monotonic"])
                )
                self._ball["_shot_receive_monotonic"] = first_received
        return True

    def _state_fresh(self, robot: str) -> bool:
        state = self._states.get(robot)
        return bool(
            state
            and state.get("valid")
            and not state.get("emergency_stop")
            and 0.0 <= self._now - float(state["_receive_monotonic"]) <= 0.25
        )

    def _current_shot(self):
        if not self._ball or not self._ball.get("valid"):
            return None
        return self._ball.get("shot_id")

    def _make_decision(self, shot_id, hitter_ros: str) -> None:
        if shot_id in self._decisions or shot_id in self.failed_shots:
            return
        hitter = ROS_TO_MODEL[hitter_ros]
        try:
            features = self.features.build(self._states, self._ball, self._now, hitter)
            decision = self.pipeline.decide(features, hitter)
            if decision.get("valid"):
                pair = np.asarray(decision.get("applied_pair_m"), dtype=np.float64)
                if (
                    pair.shape != (2,)
                    or not np.isfinite(pair).all()
                    or np.any(pair < WORKSPACE_Y[0] - 1.0e-7)
                    or np.any(pair > WORKSPACE_Y[1] + 1.0e-7)
                    or pair[0] - pair[1] < MINIMUM_GAP_M - 1.0e-6
                ):
                    decision = {**decision, "valid": False, "reason": "invalid_guard_postcondition"}
        except Exception as error:
            features = None
            decision = {
                "valid": False,
                "reason": "inference_failed:" + type(error).__name__,
                "filter_mode": "frozen_v7_cbf",
                "cbf_nominal_risk": None,
                "cbf_selected_risk": None,
                "planner_identity": PLANNER_IDENTITY,
            }
        self.last_features = features
        self.last_inference = decision
        self.reason = decision.get("reason") or "ready"
        if decision.get("valid"):
            self._decisions[shot_id] = decision
            if features is not None:
                self._decision_features[shot_id] = features
        else:
            self.failed_shots[shot_id] = str(decision.get("reason") or "invalid_actor_decision")
        while len(self._decisions) > 64:
            oldest = next(iter(self._decisions))
            self._decisions.pop(oldest, None)
            self._decision_features.pop(oldest, None)

    def _sync_pair_ack(self) -> None:
        stage = self._staging
        if (
            stage is None
            or stage.get("complete")
            or stage.get("terminal")
            or not stage.get("sequence")
        ):
            return
        if self.mode == "shadow":
            return
        ball = self._ball
        if (
            not ball
            or not ball.get("valid")
            or ball.get("shot_id") != stage["shot_id"]
            or not 0.0 <= self._now - float(ball.get("_receive_monotonic", -math.inf)) <= 0.30
        ):
            self._fail_stage("stage_ball_invalid_stale_or_replaced")
            return
        if self._now - stage["started"] > ACK_TIMEOUT_S:
            self._fail_stage("pair_stage_ack_timeout")
            return
        acknowledged = set()
        for robot in ROBOT_ORDER:
            state = self._states.get(robot)
            expected_sequence = stage["sequence"].get(robot)
            if state is None or expected_sequence is None:
                self._fail_stage(f"stage_state_missing:{robot}")
                return
            if not self._state_fresh(robot):
                self._fail_stage(f"stage_state_invalid_or_stale:{robot}")
                return
            applied = state.get("last_applied_sequence")
            session = state.get("last_planner_session_id")
            error = state.get("transport_error")
            error_sequence = state.get("transport_error_sequence")
            error_session = state.get("transport_error_session_id")
            if (
                error
                and error_session == self.session_id
                and type(error_sequence) is int
                and error_sequence == expected_sequence
            ):
                self._fail_stage(f"stage_rejected:{robot}:{error}")
                return
            if session == self.session_id and isinstance(applied, int) and applied >= expected_sequence:
                try:
                    target_matches = abs(
                        float(state.get("target_base_y")) - stage["targets"][robot]
                    ) <= ACK_TARGET_TOLERANCE_M
                except (TypeError, ValueError):
                    target_matches = False
                if target_matches:
                    acknowledged.add(robot)
        stage["acknowledged"] = sorted(acknowledged)
        if len(acknowledged) != len(ROBOT_ORDER):
            return
        stage["complete"] = True
        stage["completed_at"] = self._now
        pair = self._decisions[stage["shot_id"]]["applied_pair_m"]
        self.features.commit_fully_acked(
            stage["shot_id"], stage["ball_received"], pair
        )
        if self.mode == "active":
            self.planner.prepared_tokens.add(stage["token"])
        else:
            self.completed_transport_shots.add(stage["shot_id"])
            self.planner.abort_pending(stage["token"])

    def _fail_stage(self, reason: str) -> None:
        self._stage_failed_this_tick = True
        if self._staging is not None:
            shot_id = self._staging["shot_id"]
            token = self._staging["token"]
            self.failed_shots[shot_id] = reason
            self._staging["failure"] = reason
            self._staging["complete"] = False
            self._staging["terminal"] = True
            self.planner.abort_pending(token)
        else:
            shot_id = self._current_shot()
            if shot_id is not None:
                self.failed_shots[shot_id] = reason
        self.reason = reason

    def _recovery_hitter(self, outputs) -> str | None:
        if self._staging is not None and self._staging.get("hitter") in ROBOT_ORDER:
            return self._staging["hitter"]
        for payload in outputs.values():
            if payload.get("hitter") in ROBOT_ORDER:
                return payload["hitter"]
        return None

    def _hold_outputs(self, outputs, reason: str):
        """Preserve each V11 scheduler's last target when state cannot be trusted."""

        for payload in outputs.values():
            payload["command"].update(active=False, role="hold")
            payload["planned_active"] = False
            payload["planned_valid"] = True
            payload["valid"] = self.command_publish_enabled
            payload["planner_mode"] = "fixed_relay"
            payload["fallbacks"] = [reason]
            payload["fault_recovery_mode"] = "hold_last_target"
        return outputs

    def _relay_recovery_outputs(self, outputs, reason: str, hitter: str):
        """Restore one hitter HOME and one peer OUTWARD using Fixed/V3 lanes."""

        peer = ROBOT_ORDER[1 - ROBOT_ORDER.index(hitter)]
        for robot, payload in outputs.items():
            state = self._states[robot]
            motion = self._desired_runtime_motion_config(robot)
            is_hitter = robot == hitter
            target = float(motion.home_y if is_hitter else motion.outward_y)
            x = float(state["base_position_xyz"][0])
            payload["command"].update(
                active=False,
                role="return" if is_hitter else "outward",
                desired_base_position=[x, target],
                trajectory_base_position=[x, target],
            )
            payload["return_target_y"] = float(motion.home_y)
            payload["post_hit_outward_y"] = float(motion.outward_y)
            payload["planned_active"] = False
            payload["planned_valid"] = True
            payload["valid"] = self.command_publish_enabled
            payload["planner_mode"] = "fixed_relay"
            payload["fallbacks"] = [reason]
            payload["fault_recovery_mode"] = "relay_pair"
            payload["fault_recovery_hitter"] = hitter
            payload["fault_recovery_peer"] = peer
        return outputs

    def _fault_outputs(self, outputs, reason: str):
        # HIT/POST_DELAY are owned by the existing V11 sequence.  Do not move
        # the peer early while the hitter is still executing that sequence.
        if any(
            self._states.get(robot, {}).get("phase") in LOCKED_PHASES
            for robot in ROBOT_ORDER
        ):
            for payload in outputs.values():
                payload["fallbacks"] = list(payload.get("fallbacks", ())) + [reason]
                payload["fault_recovery_mode"] = "locked_phase_preserved"
            return outputs

        hitter = self._recovery_hitter(outputs)
        states_trusted = all(self._state_fresh(robot) for robot in ROBOT_ORDER)
        untrusted_fault = any(
            reason.startswith(prefix) for prefix in UNTRUSTED_RECOVERY_PREFIXES
        )
        if hitter is None or not states_trusted or untrusted_fault:
            return self._hold_outputs(outputs, reason)
        return self._relay_recovery_outputs(outputs, reason, hitter)

    def _tag_outputs(self, outputs):
        shot_id = self._current_shot()
        decision = self._decisions.get(shot_id) if shot_id is not None else None
        features = self._decision_features.get(shot_id) if shot_id is not None else None
        for robot, payload in outputs.items():
            role = payload["command"]["role"]
            if role == "stage":
                payload["planner_mode"] = "v7_model190_frozen_cbf"
            elif role in {"hold", "return", "outward"}:
                payload["planner_mode"] = "fixed_relay"
            payload["planner_identity"] = PLANNER_IDENTITY
            payload["filter_mode"] = "frozen_v7_cbf"
            payload["barrier_risk"] = None if decision is None else decision.get("cbf_selected_risk")
            payload["ensemble_risk"] = None if decision is None else decision.get("cbf_member_risk")
            payload["v7"] = {
                "reason": self.reason,
                "mode": self.mode,
                "identity": PLANNER_IDENTITY,
                "model_order": list(MODEL_ORDER),
                "ros_mapping": dict(MODEL_TO_ROS),
                "host_mapping": {"left": "198", "right": "66"},
                "action_index": dict(ACTION_INDEX),
                "decision": decision,
                "previous_fully_acked_targets_m": self.features.previous_targets.tolist(),
                "staging": deepcopy(self._staging),
                "corrected_time_to_strike_s": (
                    None if features is None else features.record["corrected_time_to_strike_s"]
                ),
            }
        return outputs

    def _capture_stage(self, outputs) -> None:
        sample = outputs[ROBOT_ORDER[0]]
        shot_id = sample.get("shot_id")
        token = sample.get("commit_token")
        if shot_id not in self._decisions or not token:
            return
        if not all(payload["command"]["role"] == "stage" for payload in outputs.values()):
            return
        if self._staging is None or self._staging.get("token") != token:
            targets = self.planner.prepare_targets[token]
            self._staging = {
                "shot_id": shot_id,
                "token": token,
                "hitter": sample.get("hitter"),
                "targets": dict(targets),
                "pair_model_order": list(self._decisions[shot_id]["applied_pair_m"]),
                "started": self._now,
                "ball_received": float(
                    self._ball.get(
                        "_shot_receive_monotonic", self._ball["_receive_monotonic"]
                    )
                ),
                "sequence": {},
                "acknowledged": [],
                "complete": False,
                "failure": None,
                "terminal": False,
            }
        for robot, payload in outputs.items():
            self._staging["sequence"].setdefault(robot, int(payload["sequence"]))

    def _call_fixed(self):
        shot_id = self._current_shot()
        force_invalid = (
            shot_id in self.failed_shots or shot_id in self.completed_transport_shots
        ) if shot_id is not None else False
        if not force_invalid:
            return super().tick(self._now)
        original = self._ball
        self._ball = {**original, "valid": False}
        try:
            return super().tick(self._now)
        finally:
            self._ball = original

    def tick(self, now: float | None = None):
        self._now = time.monotonic() if now is None else float(now)
        if not math.isfinite(self._now):
            raise ValueError("now must be finite")
        self.last_features = None
        self.last_inference = None
        self._stage_failed_this_tick = False
        self._sync_pair_ack()
        shot_id = self._current_shot()
        decision = self._decisions.get(shot_id) if shot_id is not None else None
        self.position_strategy.decision = decision
        outputs = self._call_fixed()
        fixed_commands = {}
        for robot, payload in outputs.items():
            command = payload["command"]
            rule_command = (
                self.planner.last_rule_commands.get(robot)
                if command.get("role") == "stage"
                else None
            )
            fixed_commands[robot] = (
                rule_command.to_dict()
                if rule_command is not None
                else deepcopy(command)
            )

        # The first Fixed pass owns hitter/reservation/token.  Inference then
        # latches one pair for that exact shot; the next 50 Hz pass stages it.
        sample = outputs[ROBOT_ORDER[0]]
        if (
            shot_id is not None
            and shot_id not in self.completed_transport_shots
            and sample.get("hitter") in ROBOT_ORDER
            and not any(
                self._states.get(robot, {}).get("phase") in LOCKED_PHASES
                for robot in ROBOT_ORDER
            )
        ):
            self._make_decision(shot_id, sample["hitter"])
            self.position_strategy.decision = self._decisions.get(shot_id)

        if self._restart_fault:
            self._fail_stage(self._restart_fault)
            outputs = self._fault_outputs(outputs, self._restart_fault)
        elif self._stage_failed_this_tick:
            outputs = self._fault_outputs(outputs, self.reason)
        elif shot_id is not None and shot_id in self.failed_shots:
            self.planner.abort_pending(sample.get("commit_token"))
            outputs = self._fault_outputs(outputs, self.failed_shots[shot_id])
        else:
            self._capture_stage(outputs)

        hit_robots = [
            robot
            for robot, payload in outputs.items()
            if payload.get("planned_active") and payload["command"].get("role") == "hit"
        ]
        if len(hit_robots) > 1:
            self._fail_stage("simultaneous_hit_guard")
            outputs = self._fault_outputs(outputs, "simultaneous_hit_guard")
        if self.mode != "active":
            for payload in outputs.values():
                if payload["command"].get("role") == "hit":
                    payload["command"].update(role="hold", active=False)
                    payload["planned_active"] = False
                    payload["valid"] = False
                    payload["fallbacks"] = list(payload.get("fallbacks", ())) + [
                        "hit_disabled_in_" + self.mode
                    ]
                elif self.mode == "shadow":
                    payload["valid"] = False
                elif payload["command"].get("role") != "stage":
                    payload["valid"] = False
        for robot, payload in outputs.items():
            payload["fixed_command"] = fixed_commands[robot]
        return self._tag_outputs(outputs)

    def _apply_cycle_commands(self, result, outputs) -> None:
        """V3 return: wait for hitter OUTWARD; never use V11 early preview."""

        if self._swap_hitter not in ROBOT_ORDER:
            return
        hitter = self._swap_hitter
        peer = ROBOT_ORDER[1 - ROBOT_ORDER.index(hitter)]
        hitter_state = self._states.get(hitter)
        peer_state = self._states.get(peer)
        if not hitter_state or not peer_state:
            return
        if not self._state_fresh(hitter) or not self._state_fresh(peer):
            self.reason = "stale_or_invalid_feedback"
            return
        if hitter_state.get("last_commit_token") == self._swap_commit_token:
            self._swap_observed_hit_phase = True
        if hitter_state.get("phase") in LOCKED_PHASES:
            self._swap_observed_hit_phase = True
        hitter_outward = self._swap_observed_hit_phase and hitter_state.get("phase") in {
            "OUTWARD",
            "OUTWARD_HOLD",
        }
        if hitter_outward and peer_state.get("phase") != "HOME_HOLD":
            target = float(self._desired_runtime_motion_config(peer).home_y)
            x = float(peer_state["base_position_xyz"][0])
            command = outputs[peer]["command"]
            command.update(
                active=False,
                role="return",
                desired_base_position=[x, target],
                trajectory_base_position=[x, target],
            )
            outputs[peer].update(
                valid=self.mode == "active",
                planned_valid=True,
                planned_active=False,
                return_target_y=target,
                fallbacks=[],
                commit_token=self._swap_commit_token,
                return_trigger_reason="peer_outward",
            )
            self._return_dispatched = True
        if self._return_dispatched and peer_state.get("phase") == "HOME_HOLD":
            self._swap_hitter = None
            self._swap_commit_token = None
            self._swap_committed_at = None
            self._swap_observed_hit_phase = False
            self._return_dispatched = False
            self._staging = None


__all__ = ["ACK_TIMEOUT_S", "V7SafetyPipeline", "V7Runtime"]
