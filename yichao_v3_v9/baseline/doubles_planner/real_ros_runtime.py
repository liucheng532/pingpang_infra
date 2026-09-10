from __future__ import annotations

import copy
import math
import time
from typing import Any, Mapping
import uuid

import numpy as np

from .config import PlannerConfig
from .deployment import make_real_robot_planner
from .fixed_relay import FixedRelayPlanner
from .return_timing import ReturnTiming, TURNAROUND_HOLD_S
from .real_inputs import (
    G1BaseEstimate,
    G1ControllerTelemetry,
    G1RobotInput,
    RealBallPrediction,
    RealInputAdapter,
    UnitreeG1LowState,
)
from .runtime_motion_config import (
    RUNTIME_MOTION_STATUS_SCHEMA,
    RuntimeMotionConfig,
    default_runtime_motion_configs,
    parse_runtime_motion_config,
    validate_combined_motion_config,
)


ROBOT_STATE_SCHEMA = "v9-robot-state-v1"
PLANNER_COMMAND_SCHEMA = "v9-planner-command-v1"
BALL_SCHEMA = "v9-ball-prediction-v1"
ROBOT_ORDER = ("table_right", "table_left")
FEEDBACK_WORKSPACE_Y = (-1.10, 1.10)
STARTUP_CANONICAL_X_M = 0.18
DEFAULT_CANONICAL_GOAL_X_M = 0.15


def v9_real_planner_config() -> PlannerConfig:
    """Map physical robots onto the checkpoint's negative/positive Y slots."""
    return PlannerConfig(
        robots=ROBOT_ORDER,
        initial_hitter="table_right",
        home_y=(-0.20, 0.20),
        outward_y=(-0.70, 0.70),
        teammate_avoidance_y=(-0.65, 0.65),
        ready_y=(-0.20, 0.20),
        racket_reach_y=(0.31, -0.31),
        workspace_y=(-0.95, 0.95),
        allow_peer_outward_hold_ready=True,
    )


def _array(payload: Mapping[str, Any], name: str, size: int) -> np.ndarray:
    value = np.asarray(payload[name], dtype=np.float32).reshape(-1)
    if value.size != size or not np.isfinite(value).all():
        raise ValueError(f"{name} must contain {size} finite values")
    return value


def _boolean(payload: Mapping[str, Any], name: str) -> bool:
    value = payload[name]
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be a bool")
    return value


class V9RealCbfRuntime:
    def __init__(self, *, shadow: bool = True, session_id: str | None = None) -> None:
        self.config = v9_real_planner_config()
        self.planner = make_real_robot_planner("cbf", self.config)
        self.planner_mode = "cbf"
        self.shadow = bool(shadow)
        self.session_id = uuid.uuid4().hex if session_id is None else str(session_id)
        if not self.session_id:
            raise ValueError("session_id must not be empty")
        self._states: dict[str, dict[str, Any]] = {}
        self._state_sequences = {name: -1 for name in ROBOT_ORDER}
        self._ball = None
        self._ball_sequence = -1
        self._swap_hitter = None
        self._swap_commit_token = None
        self._swap_committed_at = None
        self._return_dispatched = False
        self._swap_observed_hit_phase = False
        self._motion_config_active = default_runtime_motion_configs(self.config)
        self._motion_config_pending: dict[str, RuntimeMotionConfig | None] = {
            name: None for name in ROBOT_ORDER
        }
        self._motion_config_received_id = {name: None for name in ROBOT_ORDER}
        self._motion_config_robot_received_id = {name: None for name in ROBOT_ORDER}
        self._motion_config_outward_applied_id = {name: None for name in ROBOT_ORDER}
        self._motion_config_home_applied_id = {name: None for name in ROBOT_ORDER}
        self._motion_config_error = {name: "" for name in ROBOT_ORDER}
        self._startup_x = {name: None for name in ROBOT_ORDER}

    def update_robot_state(
        self,
        robot: str,
        payload: Mapping[str, Any],
        receive_monotonic: float | None = None,
    ) -> bool:
        if robot not in ROBOT_ORDER:
            raise ValueError(f"unknown robot: {robot!r}")
        if payload.get("schema_version") != ROBOT_STATE_SCHEMA:
            raise ValueError("wrong robot state schema_version")
        if payload.get("robot") != robot:
            raise ValueError("state robot does not match topic")
        sequence = int(payload["sequence"])
        if sequence <= self._state_sequences[robot]:
            previous = self._states.get(robot)
            source_ns = int(payload.get("source_monotonic_ns", 0))
            previous_source_ns = 0 if previous is None else int(previous.get("source_monotonic_ns", 0))
            if source_ns <= previous_source_ns:
                return False
        _array(payload, "q", 29)
        _array(payload, "dq", 29)
        _array(payload, "imu_quaternion_wxyz", 4)
        _array(payload, "gyro_xyz", 3)
        _array(payload, "base_position_xyz", 3)
        _array(payload, "base_orientation_wxyz", 4)
        _array(payload, "base_linear_velocity_xyz", 3)
        _array(payload, "base_angular_velocity_xyz", 3)
        _boolean(payload, "ready")
        _boolean(payload, "valid")
        _boolean(payload, "emergency_stop")
        if "time_to_strike_s" in payload:
            tts = payload["time_to_strike_s"]
            if isinstance(tts, bool) or not isinstance(tts, (int, float)) or not math.isfinite(tts):
                raise ValueError("time_to_strike_s must be finite")
        received = time.monotonic() if receive_monotonic is None else float(receive_monotonic)
        if not math.isfinite(received):
            raise ValueError("receive_monotonic must be finite")
        record = copy.deepcopy(dict(payload))
        record["_receive_monotonic"] = received
        self._states[robot] = record
        self._state_sequences[robot] = sequence
        if self._startup_x[robot] is None:
            self._startup_x[robot] = float(record["base_position_xyz"][0])
        active = self._motion_config_active[robot]
        if active.goal_x is None:
            self._motion_config_active[robot] = RuntimeMotionConfig(
                **{
                    **active.to_dict(),
                    "command_id": f"{self.session_id}:default:{robot}",
                    "published_at": time.time(),
                    "goal_x": self._startup_x[robot]
                    + DEFAULT_CANONICAL_GOAL_X_M
                    - STARTUP_CANONICAL_X_M,
                }
            )
        self._sync_runtime_motion_config_status(robot, record)
        return True

    def stage_runtime_motion_config(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        config = parse_runtime_motion_config(payload, expected_robots=ROBOT_ORDER)
        robot = config.robot_id
        if self._motion_config_received_id[robot] == config.command_id:
            existing = self._motion_config_pending[robot] or self._motion_config_active[robot]
            if existing != config:
                self._motion_config_error[robot] = "command_id reused with different values"
                raise ValueError(self._motion_config_error[robot])
            return self.runtime_motion_config_status()[robot]
        active_candidates = dict(self._motion_config_active)
        active_candidates[robot] = config
        desired_candidates = {
            name: self._motion_config_pending[name] or self._motion_config_active[name]
            for name in ROBOT_ORDER
        }
        desired_candidates[robot] = config
        try:
            validate_combined_motion_config(self.config, active_candidates)
            validate_combined_motion_config(self.config, desired_candidates)
        except ValueError as error:
            self._motion_config_error[robot] = str(error)
            raise
        self._motion_config_pending[robot] = config
        self._motion_config_received_id[robot] = config.command_id
        self._motion_config_error[robot] = ""
        return self.runtime_motion_config_status()[robot]

    def _desired_runtime_motion_config(self, robot: str) -> RuntimeMotionConfig:
        return self._motion_config_pending[robot] or self._motion_config_active[robot]

    def _set_active_motion_fields(
        self,
        robot: str,
        pending: RuntimeMotionConfig,
        *,
        outward: bool = False,
        home: bool = False,
    ) -> None:
        current = self._motion_config_active[robot]
        values = current.to_dict()
        values["command_id"] = pending.command_id
        values["published_at"] = pending.published_at
        if outward:
            values.update(
                goal_x=pending.goal_x,
                outward_y=pending.outward_y,
                outward_hold_s=pending.outward_hold_s,
            )
        if home:
            values["home_y"] = pending.home_y
        self._motion_config_active[robot] = RuntimeMotionConfig(**values)
        configs = {
            name: self._motion_config_active[name]
            for name in ROBOT_ORDER
        }
        updated = validate_combined_motion_config(self.config, configs)
        self.config = updated
        self.planner.config = updated
        for name in ROBOT_ORDER:
            if name in self.planner._last_goals:
                self.planner._last_goals[name][1] = float(configs[name].home_y)

    def _sync_runtime_motion_config_status(
        self, robot: str, state: Mapping[str, Any]
    ) -> None:
        status = state.get("runtime_motion_config")
        if not isinstance(status, Mapping):
            return
        self._motion_config_robot_received_id[robot] = status.get("received_command_id")
        self._motion_config_outward_applied_id[robot] = status.get(
            "outward_applied_command_id"
        )
        self._motion_config_home_applied_id[robot] = status.get(
            "home_applied_command_id"
        )
        if status.get("error"):
            self._motion_config_error[robot] = str(status["error"])
        pending = self._motion_config_pending[robot]
        if pending is None:
            return
        outward_done = self._motion_config_outward_applied_id[robot] == pending.command_id
        home_done = self._motion_config_home_applied_id[robot] == pending.command_id
        active = self._motion_config_active[robot]
        if outward_done and (
            active.command_id != pending.command_id
            or active.goal_x != pending.goal_x
            or active.outward_y != pending.outward_y
            or active.outward_hold_s != pending.outward_hold_s
        ):
            self._set_active_motion_fields(robot, pending, outward=True)
        if home_done and self._motion_config_active[robot].home_y != pending.home_y:
            self._set_active_motion_fields(robot, pending, home=True)
        if outward_done and home_done:
            self._motion_config_pending[robot] = None

    def runtime_motion_config_status(self) -> dict[str, Any]:
        result = {}
        for robot in ROBOT_ORDER:
            pending = self._motion_config_pending[robot]
            startup_x = self._startup_x[robot]

            def with_canonical_x(config: RuntimeMotionConfig) -> dict[str, Any]:
                values = config.to_dict()
                values["goal_x_canonical"] = (
                    None
                    if startup_x is None or config.goal_x is None
                    else STARTUP_CANONICAL_X_M + config.goal_x - startup_x
                )
                return values

            result[robot] = {
                "schema": RUNTIME_MOTION_STATUS_SCHEMA,
                "robot_id": robot,
                "startup_x": startup_x,
                "startup_canonical_x": STARTUP_CANONICAL_X_M,
                "received_command_id": self._motion_config_received_id[robot],
                "robot_received_command_id": self._motion_config_robot_received_id[robot],
                "error": self._motion_config_error[robot],
                "active": with_canonical_x(self._motion_config_active[robot]),
                "pending": None if pending is None else {
                    **with_canonical_x(pending),
                    "awaiting_outward": self._motion_config_outward_applied_id[robot]
                    != pending.command_id,
                    "awaiting_home": self._motion_config_home_applied_id[robot]
                    != pending.command_id,
                },
                "outward_applied_command_id": self._motion_config_outward_applied_id[robot],
                "home_applied_command_id": self._motion_config_home_applied_id[robot],
            }
        return result

    def update_ball(
        self,
        payload: Mapping[str, Any],
        receive_monotonic: float | None = None,
    ) -> bool:
        if payload.get("schema_version") != BALL_SCHEMA:
            raise ValueError("wrong ball schema_version")
        sequence = int(payload["sequence"])
        if sequence <= self._ball_sequence:
            return False
        for name in (
            "position",
            "velocity",
            "predicted_strike_position",
            "predicted_strike_velocity",
            "racket_normal",
            "racket_velocity",
        ):
            _array(payload, name, 3)
        _boolean(payload, "valid")
        if payload["valid"] and payload.get("shot_id") is None:
            raise ValueError("valid ball prediction requires shot_id")
        source_timestamp = payload.get("source_timestamp")
        if payload["valid"] and (
            source_timestamp is None or not math.isfinite(float(source_timestamp))
        ):
            raise ValueError("valid ball prediction requires finite source_timestamp")
        received = time.monotonic() if receive_monotonic is None else float(receive_monotonic)
        if not math.isfinite(received):
            raise ValueError("receive_monotonic must be finite")
        record = copy.deepcopy(dict(payload))
        record["_receive_monotonic"] = received
        self._ball = record
        self._ball_sequence = sequence
        return True

    @staticmethod
    def _invalid_robot(robot: str, now: float) -> G1RobotInput:
        return G1RobotInput(
            name=robot,
            low_state=UnitreeG1LowState(
                q=np.zeros(29),
                dq=np.zeros(29),
                imu_quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
                gyro_xyz=np.zeros(3),
                timestamp=now,
                valid=False,
            ),
            base=G1BaseEstimate(
                base_position_xyz=np.array([0.0, -0.35 if robot == "table_right" else 0.35, 0.75]),
                base_orientation_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
                base_linear_velocity_xyz=np.zeros(3),
                base_angular_velocity_xyz=np.zeros(3),
                timestamp=now,
                source="missing",
                valid=False,
            ),
            controller=G1ControllerTelemetry(
                phase="HOME_HOLD",
                ready=False,
                timestamp=now,
                valid=False,
            ),
        )

    def _robot_input(self, robot: str, now: float) -> G1RobotInput:
        payload = self._states.get(robot)
        if payload is None:
            return self._invalid_robot(robot, now)
        timestamp = float(payload["_receive_monotonic"])
        base_position = _array(payload, "base_position_xyz", 3)
        within_workspace = bool(
            FEEDBACK_WORKSPACE_Y[0]
            <= float(base_position[1])
            <= FEEDBACK_WORKSPACE_Y[1]
        )
        valid = bool(
            payload["valid"]
            and not payload["emergency_stop"]
            and within_workspace
        )
        ready = bool(payload["ready"])
        if self.planner_mode == "fixed_relay" and payload["phase"] == "OUTWARD_HOLD":
            ready = ready and float(payload.get("state_elapsed_s", 0.0)) >= float(
                self._desired_runtime_motion_config(robot).outward_hold_s
            )
        return G1RobotInput(
            name=robot,
            low_state=UnitreeG1LowState(
                q=_array(payload, "q", 29),
                dq=_array(payload, "dq", 29),
                imu_quaternion_wxyz=_array(payload, "imu_quaternion_wxyz", 4),
                gyro_xyz=_array(payload, "gyro_xyz", 3),
                timestamp=timestamp,
                valid=valid,
            ),
            base=G1BaseEstimate(
                base_position_xyz=base_position,
                base_orientation_wxyz=_array(payload, "base_orientation_wxyz", 4),
                base_linear_velocity_xyz=_array(payload, "base_linear_velocity_xyz", 3),
                base_angular_velocity_xyz=_array(payload, "base_angular_velocity_xyz", 3),
                timestamp=timestamp,
                source=f"mocap_fk:{robot}",
                valid=valid,
            ),
            controller=G1ControllerTelemetry(
                phase=payload["phase"],
                ready=ready,
                timestamp=timestamp,
                state_elapsed_s=float(payload.get("state_elapsed_s", 0.0)),
                stable_elapsed_s=float(payload.get("stable_elapsed_s", 0.0)),
                target_base_y=payload.get("target_base_y"),
                time_to_strike_s=payload.get("time_to_strike_s"),
                valid=valid,
            ),
        )

    def _ball_input(self) -> RealBallPrediction | None:
        payload = self._ball
        if payload is None:
            return None
        return RealBallPrediction(
            position=_array(payload, "position", 3),
            velocity=_array(payload, "velocity", 3),
            time_to_strike_s=float(payload["time_to_strike_s"]),
            timestamp=float(payload["_receive_monotonic"]),
            racket_normal=_array(payload, "racket_normal", 3),
            racket_velocity=_array(payload, "racket_velocity", 3),
            predicted_strike_position=_array(payload, "predicted_strike_position", 3),
            predicted_strike_velocity=_array(payload, "predicted_strike_velocity", 3),
            shot_id=payload.get("shot_id"),
            valid=bool(payload["valid"]),
        )

    def tick(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        current = time.monotonic() if now is None else float(now)
        robots = tuple(self._robot_input(name, current) for name in ROBOT_ORDER)
        result = RealInputAdapter(robots, self._ball_input()).plan(self.planner, current)
        if (result.commit_requested and result.hitter in ROBOT_ORDER
                and result.commit_token != self._swap_commit_token):
            self._swap_hitter = result.hitter
            self._swap_commit_token = result.commit_token
            self._swap_committed_at = current
            self._return_dispatched = False
            self._swap_observed_hit_phase = False
            self._on_cycle_commit(result.hitter, result.commit_token, current)
        safety_active = bool(
            result.diagnostics.get("safety_active", False)
            or result.diagnostics.get("cbf_active", False)
        )
        safety_override = bool(result.diagnostics.get("safety_emergency", False))
        outputs = {}
        prediction_source_timestamp_s = (
            None
            if self._ball is None or self._ball.get("source_timestamp") is None
            else float(self._ball["source_timestamp"])
        )
        for name in ROBOT_ORDER:
            command = result.commands[name].to_dict()
            planned_active = bool(command["active"])
            command["active"] = planned_active and not self.shadow
            planned_valid = not bool(result.fallbacks) and result.phase != "safe_hold"
            motion_config = self._desired_runtime_motion_config(name)
            outputs[name] = {
                "schema_version": PLANNER_COMMAND_SCHEMA,
                "planner_mode": self.planner_mode,
                "robot": name,
                "session_id": self.session_id,
                "sequence": int(result.sequence),
                "valid": planned_valid and not self.shadow,
                "planned_valid": planned_valid,
                "shadow": self.shadow,
                "planned_active": planned_active,
                "shot_id": result.shot_id,
                "hitter": result.hitter,
                "next_hitter": result.next_hitter,
                "commit_token": result.commit_token,
                "pending_shot_id": result.pending_shot_id,
                "admission_reasons": list(result.diagnostics.get("reasons", ())),
                "relay_stage": result.relay_stage,
                "safety_active": safety_active,
                "safety_override": safety_override,
                "return_target_y": float(motion_config.home_y),
                "post_hit_outward_y": float(motion_config.outward_y),
                "runtime_motion_config": motion_config.command_dict(),
                "prediction_source_timestamp_s": prediction_source_timestamp_s,
                "fallbacks": list(result.fallbacks),
                "command": command,
            }

        self._apply_cycle_commands(result, outputs)
        return outputs

    def _on_cycle_commit(self, hitter: str, token: str, now: float) -> None:
        pass

    def _apply_cycle_commands(self, result, outputs) -> None:
        if self._swap_hitter in ROBOT_ORDER:
            hitter = self._swap_hitter
            peer = ROBOT_ORDER[1 - ROBOT_ORDER.index(hitter)]
            hitter_state = self._states.get(hitter)
            peer_state = self._states.get(peer)
            hitter_clearing = hitter_state is not None and hitter_state.get("phase") in {
                "OUTWARD",
                "OUTWARD_HOLD",
            }
            peer_home = peer_state is not None and peer_state.get("phase") == "HOME_HOLD"
            home_y = float(self._desired_runtime_motion_config(peer).home_y)
            clearance = self.config.min_separation + self.config.separation_margin
            hitter_y = (
                None
                if hitter_state is None
                else float(hitter_state["base_position_xyz"][1])
            )
            hitter_outward_y = float(
                self._desired_runtime_motion_config(hitter).outward_y
            )
            clearance_ready = bool(
                hitter_y is not None
                and (
                    hitter_y - home_y >= clearance
                    if hitter_outward_y > 0.0
                    else home_y - hitter_y >= clearance
                )
            )
            if (
                hitter_clearing
                and clearance_ready
                and peer_state is not None
                and not peer_home
            ):
                current_x = float(peer_state["base_position_xyz"][0])
                command = outputs[peer]["command"]
                command["active"] = False
                command["role"] = "stage"
                command["desired_base_position"] = [current_x, home_y]
                command["trajectory_base_position"] = [current_x, home_y]
                outputs[peer]["valid"] = not self.shadow
                outputs[peer]["planned_valid"] = True
                outputs[peer]["planned_active"] = False
                outputs[peer]["return_target_y"] = home_y
                outputs[peer]["fallbacks"] = []
            elif peer_home:
                self._swap_hitter = None
                self._swap_commit_token = None
                self._swap_committed_at = None
                self._return_dispatched = False
                self._swap_observed_hit_phase = False


class V9RealFixedRelayRuntime(V9RealCbfRuntime):
    def __init__(self, *, shadow: bool = True, session_id: str | None = None) -> None:
        super().__init__(shadow=shadow, session_id=session_id)
        self.planner = FixedRelayPlanner(self.config)
        self.planner_mode = "fixed_relay"
        self._return_timing = ReturnTiming(ROBOT_ORDER)

    def stage_return_timing_config(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        self._return_timing.stage(payload)
        return self.return_timing_status()[payload["robot_id"]]

    def return_timing_status(self) -> dict[str, Any]:
        return copy.deepcopy(self._return_timing.status())

    def _on_cycle_commit(self, hitter: str, token: str, now: float) -> None:
        self._return_timing.begin(hitter, token, now)

    def _return_feedback_fresh(self, state) -> bool:
        return bool(
            state is not None and state.get("valid") and not state.get("emergency_stop")
            and 0 <= self._return_now - state["_receive_monotonic"] <= self.config.feedback_timeout
            and FEEDBACK_WORKSPACE_Y[0] <= state["base_position_xyz"][1] <= FEEDBACK_WORKSPACE_Y[1]
        )

    def _early_return_ready(self, hitter, peer, cycle) -> bool:
        hs, ps = self._states[hitter], self._states[peer]
        if hs.get("last_commit_token") != self._swap_commit_token:
            cycle["blocked_reason"] = "waiting_hit_ack"
            return False
        tts = hs.get("time_to_strike_s")
        if (isinstance(tts, bool) or not isinstance(tts, (float, int))
                or not math.isfinite(tts) or not -.25 <= tts <= .54):
            cycle["blocked_reason"] = "invalid_hitter_tts"
            return False
        cycle["observed_tts_s"] = tts
        if tts > cycle["config"]["lead_ms"] / 1000.0 + 1e-9:
            cycle["blocked_reason"] = "waiting_for_tts"
            return False
        outward_y = self._desired_runtime_motion_config(peer).outward_y
        sign = 1.0 if outward_y > self._desired_runtime_motion_config(peer).home_y else -1.0
        elapsed = ps.get("state_elapsed_s", 0.0)
        if (not isinstance(elapsed, (int, float)) or not math.isfinite(elapsed)
                or elapsed < TURNAROUND_HOLD_S
                or sign * ps["base_linear_velocity_xyz"][1] > .15):
            cycle["blocked_reason"] = "waiting_turnaround"
            return False
        hitter_goal = self._desired_runtime_motion_config(hitter).outward_y
        peer_goal = self._desired_runtime_motion_config(peer).home_y
        # Quantile profiles are only applicable to the canonical ordered lanes.
        hitter_sign = 1.0 if hitter == "table_left" else -1.0
        if (hitter_sign * (hitter_goal - hs["base_position_xyz"][1]) < 0
                or hitter_sign * (peer_goal - ps["base_position_xyz"][1]) <= 0):
            cycle["blocked_reason"] = "unsupported_return_direction"
            return False
        gap = self._return_timing.profile.minimum_gap(
            hitter_state=hs, peer_state=ps, hitter_outward_y=hitter_goal,
            peer_home_y=peer_goal, tts=tts, hitter_is_left=hitter == "table_left",
        )
        cycle["predicted_min_gap_m"] = gap
        if gap < self.config.min_separation + self.config.separation_margin:
            cycle["blocked_reason"] = "path_clearance"
            return False
        cycle["blocked_reason"] = None
        return True

    def _expire_unacknowledged_cycle(self, now: float) -> None:
        if (
            self._swap_commit_token is not None
            and not self._swap_observed_hit_phase
            and self._swap_committed_at is not None
            and now - self._swap_committed_at >= self.config.commit_handoff_timeout
        ):
            for cycle in self._return_timing.cycles.values():
                if cycle is not None and cycle["commit_token"] == self._swap_commit_token:
                    cycle["status"] = "expired"
                    cycle["blocked_reason"] = "waiting_hit_ack"
            self._swap_hitter = None
            self._swap_commit_token = None
            self._swap_committed_at = None
            self._return_dispatched = False
            self._swap_observed_hit_phase = False

    def tick(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        current = time.monotonic() if now is None else float(now)
        self._return_now = current
        self._expire_unacknowledged_cycle(current)
        return super().tick(now=current)

    def _apply_cycle_commands(self, result, outputs) -> None:
        if self._swap_hitter not in ROBOT_ORDER:
            return
        hitter = self._swap_hitter
        peer = ROBOT_ORDER[1 - ROBOT_ORDER.index(hitter)]
        hitter_state = self._states.get(hitter)
        peer_state = self._states.get(peer)
        cycle = self._return_timing.cycles[peer]
        if cycle is None or cycle["commit_token"] != self._swap_commit_token:
            return
        if not all(self._return_feedback_fresh(s) for s in (hitter_state, peer_state)):
            cycle["blocked_reason"] = "stale_or_invalid_feedback"
            return

        hitter_phase = hitter_state.get("phase")
        if (
            self._swap_commit_token is not None
            and hitter_state.get("last_commit_token") == self._swap_commit_token
        ):
            self._swap_observed_hit_phase = True
        if (hitter_phase in {"HIT", "POST_DELAY"}
                and hitter_state.get("last_commit_token") in {None, self._swap_commit_token}):
            self._swap_observed_hit_phase = True
        hitter_started_outward = self._swap_observed_hit_phase and hitter_phase in {
            "OUTWARD",
            "OUTWARD_HOLD",
        }
        if cycle["requested_at"] is None and self._swap_observed_hit_phase:
            cycle["status"] = "waiting_peer_outward" if cycle["config"]["mode"] == "peer_outward" else "waiting"
        peer_phase = peer_state.get("phase")
        if (
            self._swap_observed_hit_phase
            and peer_phase in {"RETURN", "HOME_HOLD"}
        ):
            self._return_dispatched = True
            if cycle["requested_at"] is not None and cycle["acknowledged_at"] is None:
                acknowledged = (
                    peer_phase == "RETURN"
                    and peer_state["_receive_monotonic"] >= cycle["requested_at"]
                    and peer_state.get("last_applied_sequence", cycle["requested_sequence"])
                    >= cycle["requested_sequence"]
                    and peer_state.get("last_planner_session_id", self.session_id) == self.session_id
                )
                if acknowledged:
                    cycle["acknowledged_at"] = peer_state["_receive_monotonic"]
                    cycle["request_to_return_ms"] = 1000.0 * (
                        cycle["acknowledged_at"] - cycle["requested_at"]
                    )
            cycle["status"] = "returning" if peer_phase == "RETURN" else "complete"
            cycle["blocked_reason"] = None
        early = False
        if (not self._return_dispatched and peer_phase == "OUTWARD_HOLD"
                and hitter_phase in {"HIT", "POST_DELAY"}
                and cycle["config"]["mode"] == "strike_time"):
            early = self._early_return_ready(hitter, peer, cycle)
        if (
            not self._return_dispatched
            and (hitter_started_outward or early)
            and peer_phase == "OUTWARD_HOLD"
        ):
            home_y = float(self._desired_runtime_motion_config(peer).home_y)
            current_x = float(peer_state["base_position_xyz"][0])
            command = outputs[peer]["command"]
            command["active"] = False
            command["role"] = "return"
            command["desired_base_position"] = [current_x, home_y]
            command["trajectory_base_position"] = [current_x, home_y]
            outputs[peer]["valid"] = not self.shadow
            outputs[peer]["planned_valid"] = True
            outputs[peer]["planned_active"] = False
            outputs[peer]["return_target_y"] = home_y
            outputs[peer]["fallbacks"] = []
            outputs[peer]["commit_token"] = self._swap_commit_token
            if cycle["requested_at"] is None:
                cycle["requested_at"] = self._return_now
                cycle["requested_sequence"] = outputs[peer]["sequence"]
                cycle["trigger_reason"] = "strike_time" if early else "peer_outward"
            cycle["status"] = "return_requested"
            cycle["blocked_reason"] = None
            outputs[peer]["return_trigger_reason"] = cycle["trigger_reason"]
        if self._return_dispatched and peer_phase == "HOME_HOLD":
            self._swap_hitter = None
            self._swap_commit_token = None
            self._swap_committed_at = None
            self._return_dispatched = False
            self._swap_observed_hit_phase = False
