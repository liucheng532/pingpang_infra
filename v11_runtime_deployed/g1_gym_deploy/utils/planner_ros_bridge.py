from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import threading
import time

import numpy as np

from utils.runtime_motion_config import parse_runtime_motion_config

try:
    import rospy
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - ROS is unavailable in unit tests
    rospy = None
    String = None


COMMAND_SCHEMA = "v9-planner-command-v1"
STATE_SCHEMA = "v9-robot-state-v1"


@dataclass(frozen=True)
class PlannerHitUpdate:
    sequence: int
    commit_token: str
    command_age_s: float
    raw_tts: float
    streamed_tts: float
    racket_target: np.ndarray
    target_velocity: np.ndarray
    prediction_source_timestamp_s: float
    prediction_age_s: float
    starts_hit: bool = False
    post_hit_outward_y: float | None = None
    return_target_y: float | None = None


class PlannerRosBridge:
    """Thread-safe ROS transport around the NumPy deployment scheduler."""

    def __init__(
        self,
        robot_id: str,
        command_timeout_s: float = 0.25,
        external_hit_command_mode: str = "frozen",
    ) -> None:
        if robot_id not in {"table_left", "table_right"}:
            raise ValueError(f"Unknown doubles robot_id: {robot_id!r}")
        if command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be positive")
        if external_hit_command_mode not in {"frozen", "stream"}:
            raise ValueError("external_hit_command_mode must be 'frozen' or 'stream'")
        if rospy is None or String is None or not rospy.core.is_initialized():
            raise RuntimeError("ROS must be initialized before PlannerRosBridge")
        self.robot_id = robot_id
        self.command_timeout_s = float(command_timeout_s)
        self.external_hit_command_mode = external_hit_command_mode
        self._lock = threading.Lock()
        self._latest_command = None
        self._latest_receive_monotonic = 0.0
        self._last_applied_sequence = -1
        self._last_commit_token = None
        self._last_session_id = None
        self._last_error = ""
        self._last_error_sequence = None
        self._last_error_session_id = None
        self._state_sequence = 0
        self._last_command_sequence = -1
        self._last_command_age_s = np.nan
        self._last_raw_tts = np.nan
        self._last_streamed_tts = np.nan
        self._last_stream_update = False
        self._last_prediction_source_timestamp_s = np.nan
        self._last_prediction_age_s = np.nan
        self._last_motion_config_command_id = None
        self._last_motion_config_payload = None
        self._motion_config_received_id = None
        self._motion_config_error = ""
        self._pelvis_samples = deque(maxlen=5)
        self._publisher = rospy.Publisher(
            f"/doubles/{robot_id}/state", String, queue_size=1
        )
        self._subscriber = rospy.Subscriber(
            f"/doubles/{robot_id}/command",
            String,
            self._command_callback,
            queue_size=1,
        )

    @property
    def last_applied_sequence(self) -> int:
        return int(self._last_applied_sequence)

    @property
    def last_commit_token(self):
        return self._last_commit_token

    def _set_transport_error(self, error, *, sequence=None, session_id=None) -> None:
        self._last_error = str(error)
        self._last_error_sequence = sequence if type(sequence) is int else None
        self._last_error_session_id = (
            session_id if isinstance(session_id, str) and session_id else None
        )

    def _clear_transport_error(self) -> None:
        self._last_error = ""
        self._last_error_sequence = None
        self._last_error_session_id = None

    def _command_callback(self, message) -> None:
        sequence = None
        session_id = None
        try:
            payload = json.loads(message.data)
            if payload.get("schema_version") != COMMAND_SCHEMA:
                raise ValueError("wrong command schema_version")
            if payload.get("robot") != self.robot_id:
                raise ValueError("command robot does not match topic")
            sequence = int(payload["sequence"])
            if sequence < 0:
                raise ValueError("command sequence must be non-negative")
            if not isinstance(payload.get("valid"), bool):
                raise ValueError("command valid must be a bool")
            session_id = payload.get("session_id")
            if not isinstance(session_id, str) or not session_id:
                raise ValueError("command session_id must be a non-empty string")
            with self._lock:
                self._latest_command = payload
                self._latest_receive_monotonic = time.monotonic()
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._set_transport_error(
                f"invalid_command:{error}", sequence=sequence, session_id=session_id
            )
            rospy.logwarn_throttle(1.0, f"[{self.robot_id}] {self._last_error}")

    def apply_pending(self, scheduler, pelvis_position: np.ndarray) -> PlannerHitUpdate | None:
        self._last_stream_update = False
        with self._lock:
            payload = None if self._latest_command is None else dict(self._latest_command)
            received = self._latest_receive_monotonic
        if payload is None:
            return
        command_age = time.monotonic() - received
        self._last_command_age_s = float(command_age)
        if command_age > self.command_timeout_s:
            self._set_transport_error(
                "stale_command",
                sequence=int(payload["sequence"]),
                session_id=payload.get("session_id"),
            )
            return
        sequence = int(payload["sequence"])
        self._last_command_sequence = sequence
        session_id = payload["session_id"]
        if session_id != self._last_session_id:
            self._last_session_id = session_id
            self._last_applied_sequence = -1
            self._last_commit_token = None
            self._clear_transport_error()
        if sequence <= self._last_applied_sequence:
            return
        self._last_applied_sequence = sequence
        motion_config_payload = payload.get("runtime_motion_config")
        if motion_config_payload is not None:
            try:
                config = parse_runtime_motion_config(
                    motion_config_payload,
                    expected_robot_id=self.robot_id,
                )
                config_values = config.to_dict()
                if config.command_id == self._last_motion_config_command_id:
                    if config_values != self._last_motion_config_payload:
                        raise ValueError("runtime motion config command_id was reused")
                else:
                    scheduler.stage_runtime_motion_config(
                        command_id=config.command_id,
                        goal_x=config.goal_x,
                        outward_y=config.outward_y,
                        home_y=config.home_y,
                        outward_hold_s=config.outward_hold_s,
                    )
                    self._last_motion_config_command_id = config.command_id
                    self._last_motion_config_payload = config_values
                self._motion_config_received_id = config.command_id
                self._motion_config_error = ""
            except (KeyError, TypeError, ValueError, RuntimeError) as error:
                self._motion_config_error = f"config_rejected:{error}"
        if not payload["valid"]:
            self._set_transport_error(
                "planner_command_invalid", sequence=sequence, session_id=session_id
            )
            return

        command = payload.get("command")
        if not isinstance(command, dict):
            self._set_transport_error(
                "missing_command_payload", sequence=sequence, session_id=session_id
            )
            return
        role = str(command.get("role", "hold"))
        active = bool(command.get("active", False))
        planner_mode = str(payload.get("planner_mode", "cbf"))
        token = payload.get("commit_token")
        try:
            if role == "hit" and active:
                if token is None:
                    raise ValueError("HIT command requires commit_token")
                target = np.asarray(
                    command["predicted_ball_position"], dtype=np.float32
                ).reshape(3)
                velocity = np.asarray(
                    command["predicted_racket_velocity"], dtype=np.float32
                ).reshape(3)
                ball_velocity = np.asarray(
                    command["predicted_ball_velocity"], dtype=np.float32
                ).reshape(3)
                raw_tts = float(command["predicted_ball_predict_time"])
                if not all(
                    np.isfinite(value).all()
                    for value in (target, velocity, ball_velocity)
                ) or not np.isfinite(raw_tts):
                    raise ValueError("HIT command values must be finite")
                source_timestamp = payload.get("prediction_source_timestamp_s")
                if source_timestamp is None or not np.isfinite(float(source_timestamp)):
                    raise ValueError(
                        "HIT command requires finite prediction_source_timestamp_s"
                    )
                source_timestamp = float(source_timestamp)
                now_ros_s = (
                    float(rospy.Time.now().to_sec())
                    if rospy is not None and rospy.core.is_initialized()
                    else time.time()
                )
                prediction_age = now_ros_s - source_timestamp
                if not np.isfinite(prediction_age):
                    raise ValueError("Prediction age must be finite")
                corrected_tts = float(
                    np.clip(raw_tts - prediction_age, -0.5, 0.54)
                )
                self._last_raw_tts = raw_tts
                self._last_streamed_tts = corrected_tts
                self._last_prediction_source_timestamp_s = source_timestamp
                self._last_prediction_age_s = prediction_age
                if token != self._last_commit_token:
                    outward_target_y = float(payload["post_hit_outward_y"])
                    return_target_y = payload.get("return_target_y")
                    return_target_y = (
                        None if return_target_y is None else float(return_target_y)
                    )
                    if not np.isfinite(outward_target_y) or (
                        return_target_y is not None
                        and not np.isfinite(return_target_y)
                    ):
                        raise ValueError("External base targets must be finite")
                    self._last_commit_token = token
                    self._clear_transport_error()
                    return PlannerHitUpdate(
                        sequence=sequence,
                        commit_token=str(token),
                        command_age_s=float(command_age),
                        raw_tts=raw_tts,
                        streamed_tts=corrected_tts,
                        racket_target=target.copy(),
                        target_velocity=velocity.copy(),
                        prediction_source_timestamp_s=source_timestamp,
                        prediction_age_s=prediction_age,
                        starts_hit=True,
                        post_hit_outward_y=outward_target_y,
                        return_target_y=return_target_y,
                    )
                elif (
                    self.external_hit_command_mode == "stream"
                    and getattr(getattr(scheduler, "state", None), "name", None)
                    == "HIT"
                ):
                    self._last_streamed_tts = corrected_tts
                    self._last_stream_update = True
                    self._clear_transport_error()
                    return PlannerHitUpdate(
                        sequence=sequence,
                        commit_token=str(token),
                        command_age_s=float(command_age),
                        raw_tts=raw_tts,
                        streamed_tts=corrected_tts,
                        racket_target=target.copy(),
                        target_velocity=velocity.copy(),
                        prediction_source_timestamp_s=source_timestamp,
                        prediction_age_s=prediction_age,
                        starts_hit=False,
                    )
            elif planner_mode == "fixed_relay":
                if role == "hold" and not active:
                    pass
                elif role == "return" and not active:
                    target_y = float(payload["return_target_y"])
                    if not np.isfinite(target_y):
                        raise ValueError("fixed return target must be finite")
                    goal = np.asarray(
                        [float(np.asarray(pelvis_position).reshape(3)[0]), target_y],
                        dtype=np.float32,
                    )
                    scheduler.set_external_base_target(
                        goal,
                        return_target_y=target_y,
                        safety_override=False,
                        **(
                            {"semantic_phase": "RETURN"}
                            if bool(getattr(scheduler, "native_no_mirror", False))
                            else {}
                        ),
                    )
                elif role == "outward" and not active:
                    target_y = float(payload["post_hit_outward_y"])
                    home_y = float(payload["return_target_y"])
                    if not np.isfinite(target_y) or not np.isfinite(home_y):
                        raise ValueError("fixed outward targets must be finite")
                    goal = np.asarray(
                        [float(np.asarray(pelvis_position).reshape(3)[0]), target_y],
                        dtype=np.float32,
                    )
                    scheduler.set_external_base_target(
                        goal,
                        return_target_y=home_y,
                        safety_override=False,
                        **(
                            {"semantic_phase": "OUTWARD"}
                            if bool(getattr(scheduler, "native_no_mirror", False))
                            else {}
                        ),
                    )
                else:
                    raise ValueError(
                        f"fixed relay rejects role={role!r} active={active}"
                    )
            elif role in {"clear", "exit", "stage"}:
                goal_key = (
                    "desired_base_position"
                    if bool(payload.get("safety_active", False))
                    else "trajectory_base_position"
                )
                goal = np.asarray(command[goal_key], dtype=np.float32).reshape(2).copy()
                goal[0] = float(np.asarray(pelvis_position).reshape(3)[0])
                scheduler.set_external_base_target(
                    goal,
                    return_target_y=payload.get("return_target_y"),
                    safety_override=bool(payload.get("safety_override", False)),
                )
            self._clear_transport_error()
        except (KeyError, TypeError, ValueError, RuntimeError) as error:
            self._set_transport_error(
                f"command_rejected:{error}", sequence=sequence, session_id=session_id
            )
            if rospy is not None and rospy.core.is_initialized():
                rospy.logwarn_throttle(1.0, f"[{self.robot_id}] {self._last_error}")
        return None

    def record_snapshot_fields(self) -> dict:
        return {
            "external_hit_command_mode": 1
            if self.external_hit_command_mode == "stream"
            else 0,
            "planner_command_sequence": max(0, int(self._last_command_sequence)),
            "planner_commit_token": str(self._last_commit_token or "")[:64].encode(),
            "planner_command_age_ms": float(self._last_command_age_s) * 1000.0,
            "planner_raw_tts": float(self._last_raw_tts),
            "planner_streamed_tts": float(self._last_streamed_tts),
            "planner_stream_update": int(self._last_stream_update),
            "planner_prediction_source_timestamp_s": float(
                self._last_prediction_source_timestamp_s
            ),
            "planner_prediction_age_ms": float(self._last_prediction_age_s) * 1000.0,
        }

    def publish_state(self, agent) -> None:
        now_monotonic = time.monotonic()
        pelvis = np.asarray(agent.pelvis_pos, dtype=np.float64).reshape(3)
        self._pelvis_samples.append((now_monotonic, pelvis.copy()))
        linear_velocity = np.zeros(3, dtype=np.float64)
        if len(self._pelvis_samples) >= 2:
            first_t, first_position = self._pelvis_samples[0]
            last_t, last_position = self._pelvis_samples[-1]
            elapsed = last_t - first_t
            if elapsed > 1.0e-6:
                linear_velocity = (last_position - first_position) / elapsed

        q_gym = np.asarray(agent.dof_pos, dtype=np.float32)
        dq_gym = np.asarray(agent.dof_vel, dtype=np.float32)
        q_lab = q_gym[agent.from_gym_to_lab]
        dq_lab = dq_gym[agent.from_gym_to_lab]
        phase = agent.reference.state.name
        state_sources_valid = bool(
            agent._torso_receive_monotonic_ns > 0
            and agent.se.last_body_receive_monotonic_ns > 0
            and agent.se.last_imu_receive_monotonic_ns > 0
        )
        arrays = (
            q_lab,
            dq_lab,
            agent.se.body_quat,
            agent.se.imu_ang_vel,
            pelvis,
            agent.pelvis_quat_wxyz,
            linear_velocity,
            agent.pelvis_angular_velocity_w,
        )
        finite = all(np.isfinite(np.asarray(value)).all() for value in arrays)
        self._state_sequence += 1
        motion_status = agent.reference_scheduler.runtime_motion_config_status()
        payload = {
            "schema_version": STATE_SCHEMA,
            "robot": self.robot_id,
            "sequence": self._state_sequence,
            "source_monotonic_ns": time.monotonic_ns(),
            "q": q_lab.tolist(),
            "dq": dq_lab.tolist(),
            "imu_quaternion_wxyz": np.asarray(agent.se.body_quat, dtype=float).tolist(),
            "gyro_xyz": np.asarray(agent.se.get_body_angular_vel(), dtype=float).tolist(),
            "base_position_xyz": pelvis.tolist(),
            "base_orientation_wxyz": np.asarray(
                agent.pelvis_quat_wxyz, dtype=float
            ).tolist(),
            "base_linear_velocity_xyz": linear_velocity.tolist(),
            "base_angular_velocity_xyz": np.asarray(
                agent.pelvis_angular_velocity_w, dtype=float
            ).tolist(),
            "phase": phase,
            "ready": phase in {"HOME_HOLD", "OUTWARD_HOLD"},
            "state_elapsed_s": float(agent.reference_scheduler.state_elapsed_s),
            "stable_elapsed_s": float(agent.reference_scheduler.stable_elapsed_s),
            "target_base_y": float(pelvis[1] + agent.reference.target_base[1]),
            "home_y": float(agent.reference_scheduler.home_y),
            "time_to_strike_s": float(agent.reference.strike_time),
            "last_applied_sequence": self.last_applied_sequence,
            "last_planner_session_id": self._last_session_id,
            "last_commit_token": self.last_commit_token,
            "runtime_motion_config": {
                "received_command_id": self._motion_config_received_id,
                "error": self._motion_config_error,
                **motion_status,
            },
            "valid": bool(state_sources_valid and finite),
            "emergency_stop": False,
            "transport_error": self._last_error,
            "transport_error_sequence": self._last_error_sequence,
            "transport_error_session_id": self._last_error_session_id,
        }
        self._publisher.publish(
            String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True))
        )
