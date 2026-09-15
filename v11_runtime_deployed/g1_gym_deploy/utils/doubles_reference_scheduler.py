from __future__ import annotations

import hashlib
from collections import deque
from dataclasses import dataclass
from enum import IntEnum
from pathlib import Path

import numpy as np

from utils.data_utils import MoveMotionBank, MotionCommand
from utils.joint_mapping import LAB_JOINT_NAMES
from utils.right_side_right_hand import reflect_right_side_body


class DoublesPhase(IntEnum):
    HIT = 0
    POST_DELAY = 1
    OUTWARD = 2
    OUTWARD_HOLD = 3
    RETURN = 4
    HOME_HOLD = 5


PHASE_ORDER = ("HIT", "POST_DELAY", "OUTWARD", "OUTWARD_HOLD", "RETURN", "HOME_HOLD")
TIME_EPSILON = 1.0e-9
V9_HOLD_READY_SUFFIX = "0302_combined-0368:v0/motion.npz"
V10_CANONICAL_X_CENTER = 0.18
V10_TARGET_BASE_X_CLIP = 0.04
V11_COMMON_HOLD_SHA256 = "9565a8ed1ba22cfc0d759d0a8327dc7dd37989c242d4f4337dcc79c796ea3a8f"


@dataclass(frozen=True)
class DoublesReference:
    strike_time: float
    target_velocity: np.ndarray
    racket_target: np.ndarray
    target_base: np.ndarray
    joint_pos: np.ndarray
    joint_vel: np.ndarray
    hit_joint_pos: np.ndarray
    hit_joint_vel: np.ndarray
    hit_anchor_pos_w: np.ndarray
    phase: np.ndarray
    state: DoublesPhase
    hit_motion_index: int
    move_motion_index: int
    reference_step: int
    outward_hold_elapsed_s: float
    outward_hold_duration_s: float
    outward_hold_entry_reason: str

    @property
    def command(self) -> np.ndarray:
        return np.concatenate((self.joint_pos, self.joint_vel)).astype(np.float32, copy=False)


class DoublesReferenceScheduler:
    """Deployment-time six-state reference scheduler for the 1666-D student."""

    STRIKE_FRAME = 27
    HIT_FPS = 50.0

    def __init__(
        self,
        hit_motions: MotionCommand,
        move_motions: MoveMotionBank,
        control_dt: float = 0.02,
        post_delay_s: float = 0.20,
        readiness_duration_s: float = 0.10,
        readiness_roll_rad: float = 0.15,
        readiness_pitch_rad: float = 0.20,
        readiness_linear_speed_mps: float = 0.25,
        readiness_angular_speed_rps: float = 0.50,
        transition_s: float = 0.10,
        target_hold_reference_transition_s: float = 0.30,
        outward_target_y: float = 0.9125,
        outward_hold_s: float = 3.0,
        stable_position_error_m: float = 0.06,
        stable_lateral_speed_mps: float = 0.15,
        stable_duration_s: float = 0.10,
        stationary_hit_test: str = "none",
        external_control: bool = False,
        hit_reference_lead_steps: int = 0,
        episode_start_x: float | None = None,
        move_side_sign: int = 1,
        native_no_mirror: bool = False,
        v10_relative_x: bool = False,
        target_base_x_clip: float | None = V10_TARGET_BASE_X_CLIP,
        reference_end_forces_hold: bool = False,
        latch_hold_x_on_entry: bool = False,
        common_hold_pose_path: str | None = None,
        common_hold_pose_sha256: str | None = None,
    ):
        self.hit_motions = hit_motions
        self.move_motions = move_motions
        self.control_dt = float(control_dt)
        self.post_delay_s = float(post_delay_s)
        self.readiness_duration_s = float(readiness_duration_s)
        self.readiness_roll_rad = float(readiness_roll_rad)
        self.readiness_pitch_rad = float(readiness_pitch_rad)
        self.readiness_linear_speed_mps = float(readiness_linear_speed_mps)
        self.readiness_angular_speed_rps = float(readiness_angular_speed_rps)
        self.transition_s = float(transition_s)
        self.target_hold_reference_transition_s = float(target_hold_reference_transition_s)
        if isinstance(move_side_sign, bool) or not isinstance(
            move_side_sign, (int, np.integer)
        ) or int(move_side_sign) not in (-1, 1):
            raise ValueError("move_side_sign must be +1 or -1.")
        self.move_side_sign = int(move_side_sign)
        self.native_no_mirror = bool(native_no_mirror)
        self.outward_target_y = self.move_side_sign * abs(float(outward_target_y))
        self.outward_hold_s = float(outward_hold_s)
        self._configured_goal_x = None
        self._pending_motion_config = None
        self._outward_config_command_id = None
        self._home_config_command_id = None
        self.stable_position_error_m = float(stable_position_error_m)
        self.stable_lateral_speed_mps = float(stable_lateral_speed_mps)
        self.stable_duration_s = float(stable_duration_s)
        if stationary_hit_test not in ("none", "hit_home"):
            raise ValueError(
                f"stationary_hit_test must be 'none' or 'hit_home', got {stationary_hit_test!r}."
            )
        self.stationary_hit_test = stationary_hit_test
        self.external_control = bool(external_control)
        if isinstance(hit_reference_lead_steps, bool) or not isinstance(
            hit_reference_lead_steps, (int, np.integer)
        ):
            raise ValueError("hit_reference_lead_steps must be an integer in {0, 1, 2}.")
        if int(hit_reference_lead_steps) not in (0, 1, 2):
            raise ValueError("hit_reference_lead_steps must be one of 0, 1, or 2.")
        self.hit_reference_lead_steps = int(hit_reference_lead_steps)
        if episode_start_x is not None and not np.isfinite(float(episode_start_x)):
            raise ValueError("episode_start_x must be finite when provided.")
        self.episode_start_x_override = (
            None if episode_start_x is None else float(episode_start_x)
        )
        self.v10_relative_x = bool(v10_relative_x)
        if target_base_x_clip is not None and (
            not np.isfinite(float(target_base_x_clip)) or float(target_base_x_clip) <= 0.0
        ):
            raise ValueError("target_base_x_clip must be finite and positive when provided.")
        self.target_base_x_clip = (
            None if target_base_x_clip is None else float(target_base_x_clip)
        )
        self.reference_end_forces_hold = bool(reference_end_forces_hold)
        self.latch_hold_x_on_entry = bool(latch_hold_x_on_entry)
        self._common_hold_joint_pos = self._load_common_hold_joint_pos(
            common_hold_pose_path, common_hold_pose_sha256
        )

        self._hit_q = hit_motions.joint_pos_all.detach().cpu().numpy().astype(np.float32, copy=False)
        self._hit_qd = hit_motions.joint_vel_all.detach().cpu().numpy().astype(np.float32, copy=False)
        self._hit_torso_pos = (
            hit_motions.torso_pos_all.detach().cpu().numpy().astype(np.float32, copy=False)
        )
        self._hit_targets = hit_motions.motion_targets.detach().cpu().numpy().astype(np.float32, copy=False)
        self._hit_lengths = np.asarray(
            [motion.time_step_total for motion in hit_motions.motions], dtype=np.int64
        )
        self._validate_hit_bank()
        self._hold_ready_joint_pos = self._load_v9_hold_ready_joint_pos()
        self._hold_nominal_joint_pos = self._build_nominal_joint_pos()

        self.state = DoublesPhase.HOME_HOLD
        self._pelvis_position = np.zeros(3, dtype=np.float32)
        self._torso_position = np.zeros(3, dtype=np.float32)
        self._base_samples: deque[tuple[float, float, float]] = deque(maxlen=5)
        self._clock_s = 0.0
        self._initialized = False

    def _canonical_move_displacement(self, displacement: float) -> float:
        if self.native_no_mirror:
            return float(displacement)
        return self.move_side_sign * float(displacement)

    def _physicalize_move_joint_values(self, values: np.ndarray) -> np.ndarray:
        result = np.asarray(values, dtype=np.float32).copy()
        if self.move_side_sign < 0 and not self.native_no_mirror:
            result = reflect_right_side_body(result, LAB_JOINT_NAMES)
        return result

    def _move_label_for_state(self, state: DoublesPhase) -> int:
        if state not in (DoublesPhase.OUTWARD, DoublesPhase.RETURN):
            raise ValueError("move label requires OUTWARD or RETURN state")
        if self.native_no_mirror and self.move_side_sign < 0:
            return 0 if state == DoublesPhase.OUTWARD else 1
        return 1 if state == DoublesPhase.OUTWARD else 0

    def _validate_hit_bank(self) -> None:
        if self._hit_q.ndim != 3 or self._hit_q.shape[-1] != 29:
            raise ValueError(f"Hit joint_pos must have shape [M, T, 29], got {self._hit_q.shape}.")
        if self._hit_qd.shape != self._hit_q.shape:
            raise ValueError(f"Hit joint_vel shape {self._hit_qd.shape} does not match {self._hit_q.shape}.")
        if self._hit_targets.shape != (self._hit_q.shape[0], 3):
            raise ValueError(f"Hit targets must have shape [M, 3], got {self._hit_targets.shape}.")
        if self._hit_torso_pos.shape != (*self._hit_q.shape[:2], 3):
            raise ValueError(
                f"Hit torso positions must have shape [M, T, 3], got {self._hit_torso_pos.shape}."
            )
        for motion in self.hit_motions.motions:
            if not np.isclose(float(motion.fps), self.HIT_FPS):
                raise ValueError(f"Hit motion fps must be {self.HIT_FPS}, got {motion.fps}.")

    def _load_v9_hold_ready_joint_pos(self) -> np.ndarray:
        normalized = [path.replace("\\", "/") for path in self.hit_motions.motion_files]
        matches = [index for index, path in enumerate(normalized) if path.endswith(V9_HOLD_READY_SUFFIX)]
        if len(matches) != 1:
            raise RuntimeError(
                f"V9 HOLD ready pose requires exactly one {V9_HOLD_READY_SUFFIX}, found {len(matches)}."
            )
        self._hold_ready_motion_index = matches[0]
        return self._hit_q[self._hold_ready_motion_index, 0].copy()

    @staticmethod
    def _load_common_hold_joint_pos(
        pose_path: str | None, expected_sha256: str | None
    ) -> np.ndarray | None:
        if pose_path is None:
            if expected_sha256 is not None:
                raise ValueError("common_hold_pose_sha256 requires common_hold_pose_path.")
            return None
        path = Path(pose_path).expanduser().resolve()
        if expected_sha256 != V11_COMMON_HOLD_SHA256:
            raise ValueError("V11 common-HOLD SHA256 contract is missing or incorrect.")
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != expected_sha256:
            raise ValueError(
                f"V11 common-HOLD asset SHA256 is {digest}, expected {expected_sha256}."
            )
        with np.load(path) as data:
            joint_pos = np.asarray(data["joint_pos"], dtype=np.float32)
            joint_vel = np.asarray(data["joint_vel"], dtype=np.float32)
        if joint_pos.shape != (1, 29) or joint_vel.shape != (1, 29):
            raise ValueError(
                f"V11 common-HOLD asset must contain [1,29] q/qd, got {joint_pos.shape}/{joint_vel.shape}."
            )
        if not np.isfinite(joint_pos).all() or not np.array_equal(joint_vel, np.zeros_like(joint_vel)):
            raise ValueError("V11 common-HOLD q must be finite and qd must be zero.")
        waist_ids = [LAB_JOINT_NAMES.index(name) for name in (
            "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"
        )]
        if not np.array_equal(joint_pos[0, waist_ids], np.zeros(3, dtype=np.float32)):
            raise ValueError("V11 common-HOLD waist must be neutral.")
        return joint_pos[0].copy()

    @staticmethod
    def _build_nominal_joint_pos() -> np.ndarray:
        values = {
            "left_hip_pitch_joint": -0.312,
            "right_hip_pitch_joint": -0.312,
            "left_knee_joint": 0.669,
            "right_knee_joint": 0.669,
            "left_ankle_pitch_joint": -0.363,
            "right_ankle_pitch_joint": -0.363,
            "left_shoulder_pitch_joint": -0.2,
            "right_shoulder_pitch_joint": -0.2,
            "left_shoulder_roll_joint": 0.2,
            "right_shoulder_roll_joint": -0.2,
            "left_elbow_joint": -0.2,
            "right_elbow_joint": -0.2,
        }
        return np.asarray([values.get(name, 0.0) for name in LAB_JOINT_NAMES], dtype=np.float32)

    @staticmethod
    def _one_hot(state: DoublesPhase) -> np.ndarray:
        value = np.zeros(len(PHASE_ORDER), dtype=np.float32)
        value[int(state)] = 1.0
        return value

    def reset(
        self,
        pelvis_position: np.ndarray,
        torso_position: np.ndarray,
    ) -> DoublesReference:
        pelvis = np.asarray(pelvis_position, dtype=np.float32).reshape(3)
        torso = np.asarray(torso_position, dtype=np.float32).reshape(3)
        if not np.isfinite(pelvis).all() or not np.isfinite(torso).all():
            raise ValueError("Scheduler reset positions must be finite.")
        self._pelvis_position = pelvis.copy()
        self._torso_position = torso.copy()
        self.episode_start_x = (
            float(pelvis[0])
            if self.episode_start_x_override is None
            else self.episode_start_x_override
        )
        self.session_start_x = float(pelvis[0])
        self.goal_x = self.episode_start_x
        self.target_hold_x = self.goal_x
        self.home_hold_x = self.goal_x
        self._outward_x_anchor_valid = False
        self.home_y = float(pelvis[1])
        self._clock_s = 0.0
        self._base_samples.clear()
        self._base_samples.append((self._clock_s, float(pelvis[0]), self.home_y))

        self.state = DoublesPhase.HOME_HOLD
        self.state_elapsed_s = 0.0
        self.segment_elapsed_s = 0.0
        self.stable_elapsed_s = 0.0
        self.outward_hold_elapsed_s = 0.0
        self.outward_hold_duration_s = self.outward_hold_s
        self.outward_hold_entry_reason = "none"
        self.transition_remaining_s = 0.0
        self.reference_transition_remaining_s = 0.0
        self.reference_transition_duration_s = self.transition_s
        self.hit_armed = True
        self.stationary_hit_crossed = False
        self.stationary_post_hit_elapsed_s = 0.0
        self.pending_outward_target_y = self.outward_target_y
        self.pending_outward_target_valid = False
        self.hit_anchor_compensation = np.zeros(3, dtype=np.float32)

        self.hit_motion_index = 0
        self.hit_step = 0
        self.move_motion_index = self.move_motions.nearest_index(
            self._canonical_move_displacement(self.home_y - self.outward_target_y),
            label=self._move_label_for_state(DoublesPhase.RETURN),
        )
        self.move_step = int(self.move_motions.lengths[self.move_motion_index] - 1)
        home_q, _, _ = self.move_motions.frame(self.move_motion_index, self.move_step)
        self._current_q = self._physicalize_move_joint_values(
            self._stationary_hold_q(home_q)
        )
        self._current_qd = np.zeros(29, dtype=np.float32)
        self._previous_q = self._current_q.copy()
        self._previous_qd = self._current_qd.copy()
        self._previous_phase = self._one_hot(self.state)

        self.strike_time = -0.5
        self.racket_target = np.zeros(3, dtype=np.float32)
        self.target_velocity = np.zeros(3, dtype=np.float32)
        self._initialized = True
        return self.output()

    def _blended_reference(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        if self.transition_s <= 0.0:
            old_weight = 0.0
        else:
            old_weight = float(np.clip(self.transition_remaining_s / self.transition_s, 0.0, 1.0))
        phase_new_weight = 1.0 - old_weight
        phase = phase_new_weight * self._one_hot(self.state) + old_weight * self._previous_phase

        reference_old_weight = old_weight
        reference_derivative = 0.0
        if self.state == DoublesPhase.OUTWARD_HOLD:
            duration = max(self.reference_transition_duration_s, 1.0e-6)
            progress = float(
                np.clip(1.0 - self.reference_transition_remaining_s / duration, 0.0, 1.0)
            )
            smooth_new_weight = progress * progress * (3.0 - 2.0 * progress)
            reference_old_weight = 1.0 - smooth_new_weight
            reference_derivative = 6.0 * progress * (1.0 - progress) / duration
        reference_new_weight = 1.0 - reference_old_weight
        q = reference_new_weight * self._current_q + reference_old_weight * self._previous_q
        qd = reference_new_weight * self._current_qd + reference_old_weight * self._previous_qd
        if self.state == DoublesPhase.OUTWARD_HOLD:
            qd = qd + reference_derivative * (self._current_q - self._previous_q)
        return q.astype(np.float32), qd.astype(np.float32), phase.astype(np.float32)

    def _set_state(self, state: DoublesPhase) -> None:
        old_q, old_qd, old_phase = self._blended_reference()
        self._previous_q = old_q
        self._previous_qd = old_qd
        self._previous_phase = old_phase
        self.state = state
        self.state_elapsed_s = 0.0
        self.segment_elapsed_s = 0.0
        self.stable_elapsed_s = 0.0
        self.outward_hold_elapsed_s = 0.0
        self.transition_remaining_s = self.transition_s
        self.reference_transition_duration_s = (
            self.target_hold_reference_transition_s
            if state == DoublesPhase.OUTWARD_HOLD
            else self.transition_s
        )
        self.reference_transition_remaining_s = self.reference_transition_duration_s

    def _select_hit(self, racket_target: np.ndarray, torso_position: np.ndarray) -> int:
        anchor = np.asarray(torso_position, dtype=np.float32).copy()
        anchor[2] = 0.0
        relative_target = np.asarray(racket_target, dtype=np.float32) - anchor
        error = np.linalg.norm(self._hit_targets - relative_target[None, :], axis=1)
        return int(np.argmin(error))

    def _start_hit(
        self,
        time_to_strike: float,
        racket_target: np.ndarray,
        target_velocity: np.ndarray,
    ) -> None:
        self.hit_motion_index = self._select_hit(racket_target, self._torso_position)
        length = int(self._hit_lengths[self.hit_motion_index])
        base_step = int(self.STRIKE_FRAME - time_to_strike * self.HIT_FPS)
        self.hit_step = int(
            np.clip(base_step + self.hit_reference_lead_steps, 0, length - 1)
        )
        self.strike_time = float(np.clip(time_to_strike, -0.5, 0.54))
        self.racket_target = np.asarray(racket_target, dtype=np.float32).reshape(3).copy()
        self.target_velocity = np.asarray(target_velocity, dtype=np.float32).reshape(3).copy()
        self.hit_anchor_compensation = self._torso_position.copy()
        self.hit_anchor_compensation[2] = 0.0
        self.stationary_hit_crossed = False
        self.stationary_post_hit_elapsed_s = 0.0
        self._set_state(DoublesPhase.HIT)
        self._refresh_hit_reference()
        self.hit_armed = False

    def _refresh_hit_reference(self) -> None:
        self._current_q = self._hit_q[self.hit_motion_index, self.hit_step].copy()
        self._current_qd = self._hit_qd[self.hit_motion_index, self.hit_step].copy()

    def _capture_outward_x_anchor(self) -> None:
        self.episode_start_x = (
            float(self._pelvis_position[0])
            if self.episode_start_x_override is None
            else self.episode_start_x_override
        )
        self.goal_x = self.episode_start_x
        if self._configured_goal_x is not None:
            self.goal_x = float(self._configured_goal_x)
        self.target_hold_x = self.goal_x
        self._outward_x_anchor_valid = True

    def stage_runtime_motion_config(
        self,
        command_id: str,
        goal_x: float,
        outward_y: float,
        home_y: float,
        outward_hold_s: float,
    ) -> None:
        values = np.asarray(
            [goal_x, outward_y, home_y, outward_hold_s], dtype=float
        )
        if not np.isfinite(values).all():
            raise ValueError("Runtime motion config values must be finite.")
        command_id = str(command_id)
        if not command_id:
            raise ValueError("Runtime motion config command_id must be non-empty.")
        if self.stationary_hit_test == "hit_home":
            return
        pending = self._pending_motion_config
        if pending is not None and pending["command_id"] == command_id:
            expected = (
                float(pending["goal_x"]),
                float(pending["outward_y"]),
                float(pending["home_y"]),
                float(pending["outward_hold_s"]),
            )
            if not np.allclose(values, expected, rtol=0.0, atol=0.0):
                raise ValueError("Runtime motion config command_id was reused.")
            return
        if (
            self._outward_config_command_id == command_id
            and self._home_config_command_id == command_id
        ):
            expected = (
                float(self.goal_x),
                float(self.outward_target_y),
                float(self.home_y),
                float(self.outward_hold_s),
            )
            if not np.allclose(values, expected, rtol=0.0, atol=0.0):
                raise ValueError("Runtime motion config command_id was reused.")
            return
        self._pending_motion_config = {
            "command_id": command_id,
            "goal_x": float(goal_x),
            "outward_y": float(outward_y),
            "home_y": float(home_y),
            "outward_hold_s": float(outward_hold_s),
            "awaiting_outward": True,
            "awaiting_home": True,
        }

    def _clear_applied_motion_config(self) -> None:
        pending = self._pending_motion_config
        if pending is not None and not (
            pending["awaiting_outward"] or pending["awaiting_home"]
        ):
            self._pending_motion_config = None

    def _apply_pending_outward_config(self) -> None:
        pending = self._pending_motion_config
        if pending is None or not pending["awaiting_outward"]:
            return
        self._configured_goal_x = float(pending["goal_x"])
        self.goal_x = self._configured_goal_x
        self.outward_target_y = float(pending["outward_y"])
        self.outward_hold_s = float(pending["outward_hold_s"])
        pending["awaiting_outward"] = False
        self._outward_config_command_id = pending["command_id"]
        self._clear_applied_motion_config()

    def _apply_pending_home_config(self) -> None:
        pending = self._pending_motion_config
        if pending is None or not pending["awaiting_home"]:
            return
        self.home_y = float(pending["home_y"])
        pending["awaiting_home"] = False
        self._home_config_command_id = pending["command_id"]
        self._clear_applied_motion_config()

    def runtime_motion_config_status(self) -> dict:
        pending = None
        if self._pending_motion_config is not None:
            pending = dict(self._pending_motion_config)
        return {
            "active": {
                "goal_x": float(self.goal_x),
                "outward_y": float(self.outward_target_y),
                "home_y": float(self.home_y),
                "outward_hold_s": float(self.outward_hold_s),
                "target_base_x_clip": self.target_base_x_clip,
            },
            "pending": pending,
            "outward_applied_command_id": self._outward_config_command_id,
            "home_applied_command_id": self._home_config_command_id,
            "session_start_x": float(self.session_start_x),
            "x_safe_min": None,
            "x_safe_max": None,
        }

    def _start_outward(self) -> None:
        # RobustTeacher pure-Y semantics: do not recover X drift accumulated
        # during HIT/POST_DELAY as part of the lateral move segment.
        self._capture_outward_x_anchor()
        if self.pending_outward_target_valid:
            self.outward_target_y = float(self.pending_outward_target_y)
            self.pending_outward_target_valid = False
        self._apply_pending_outward_config()
        displacement = self.outward_target_y - float(self._pelvis_position[1])
        self.move_motion_index = self.move_motions.nearest_index(
            self._canonical_move_displacement(displacement),
            label=self._move_label_for_state(DoublesPhase.OUTWARD),
        )
        self.move_step = 0
        self._set_state(DoublesPhase.OUTWARD)
        self._refresh_move_reference()

    def set_external_hit(
        self,
        racket_target: np.ndarray,
        target_velocity: np.ndarray,
        time_to_strike_s: float,
        ball_velocity: np.ndarray | None = None,
    ) -> None:
        target = np.asarray(racket_target, dtype=np.float32).reshape(3)
        velocity = np.asarray(target_velocity, dtype=np.float32).reshape(3)
        strike_time = float(time_to_strike_s)
        if not np.isfinite(target).all() or not np.isfinite(velocity).all():
            raise ValueError("External hit target and velocity must be finite.")
        if not np.isfinite(strike_time):
            raise ValueError("External hit time must be finite.")
        if ball_velocity is not None:
            ball = np.asarray(ball_velocity, dtype=np.float32).reshape(3)
            if not np.isfinite(ball).all():
                raise ValueError("External ball velocity must be finite.")
        if self.state in (DoublesPhase.HIT, DoublesPhase.POST_DELAY):
            raise RuntimeError("External hit cannot interrupt HIT or POST_DELAY.")
        self.external_control = True
        self.pending_outward_target_valid = False
        self.home_y = float(self._pelvis_position[1])
        self._start_hit(float(np.clip(strike_time, -0.5, 0.54)), target, velocity)

    def clear_external_control(self) -> None:
        self.external_control = False
        self.pending_outward_target_valid = False

    def set_external_outward_target(self, target_y: float) -> None:
        value = float(target_y)
        if not np.isfinite(value):
            raise ValueError("External outward target must be finite.")
        if self.stationary_hit_test == "hit_home":
            return
        self.external_control = True
        self.pending_outward_target_y = value
        self.pending_outward_target_valid = True

    def set_external_home_target(self, target_y: float) -> None:
        value = float(target_y)
        if not np.isfinite(value):
            raise ValueError("External home target must be finite.")
        if self.stationary_hit_test == "hit_home":
            return
        self.external_control = True
        self.home_y = value

    def bootstrap_outward_hold(self, home_y: float, outward_y: float | None = None) -> None:
        if not self._initialized:
            raise RuntimeError("Scheduler must be reset before OUTWARD_HOLD bootstrap.")
        home = float(home_y)
        target = float(self._pelvis_position[1]) if outward_y is None else float(outward_y)
        if not np.isfinite(home) or not np.isfinite(target):
            raise ValueError("Bootstrap home/outward targets must be finite.")
        if self.stationary_hit_test == "hit_home":
            raise RuntimeError("OUTWARD_HOLD bootstrap is disabled by stationary hit_home mode.")
        displacement = target - home
        motion_index = self.move_motions.nearest_index(
            self._canonical_move_displacement(displacement),
            label=self._move_label_for_state(DoublesPhase.OUTWARD),
        )
        self.external_control = True
        self.home_y = home
        self.outward_target_y = target
        self.move_motion_index = motion_index
        self.move_step = int(self.move_motions.lengths[motion_index] - 1)
        self._capture_outward_x_anchor()
        self._start_outward_hold("bootstrap")

    def _external_motion_index(self, displacement: float) -> int:
        canonical_displacement = self._canonical_move_displacement(displacement)
        candidates = np.flatnonzero(
            self.move_motions.targets_y * canonical_displacement > 0.0
        )
        disabled = tuple(getattr(self.move_motions, "DISABLED_INDICES", ()))
        candidates = candidates[~np.isin(candidates, disabled)]
        if candidates.size == 0:
            raise ValueError("Move bank has no same-sign reference for an external base target.")
        error = np.abs(
            self.move_motions.targets_y[candidates] - canonical_displacement
        )
        return int(candidates[int(np.argmin(error))])

    def set_external_base_target(
        self,
        target_xy: np.ndarray,
        return_target_y: float | None = None,
        safety_override: bool = False,
        semantic_phase: str | None = None,
    ) -> None:
        target = np.asarray(target_xy, dtype=np.float32).reshape(2)
        if not np.isfinite(target).all():
            raise ValueError("External base target must be finite.")
        if not isinstance(safety_override, (bool, np.bool_)):
            raise TypeError("safety_override must be a bool.")
        if abs(float(target[0]) - float(self._pelvis_position[0])) > 1.0e-4:
            raise ValueError("Deployment scheduler supports lateral-Y base targets only.")
        if self.stationary_hit_test == "hit_home":
            return
        if self.state in (DoublesPhase.HIT, DoublesPhase.POST_DELAY) and not safety_override:
            raise RuntimeError("External base targets require a locomotion phase or HOME_HOLD.")

        goal_y = float(target[1])
        displacement = goal_y - float(self._pelvis_position[1])
        if abs(displacement) <= 1.0e-4:
            return
        motion_index = self._external_motion_index(displacement)
        label = int(self.move_motions.labels[motion_index])
        if semantic_phase is None:
            requested_state = DoublesPhase.OUTWARD if label == 1 else DoublesPhase.RETURN
        else:
            try:
                requested_state = DoublesPhase[str(semantic_phase)]
            except KeyError as error:
                raise ValueError("semantic_phase must be OUTWARD or RETURN") from error
            if requested_state not in (DoublesPhase.OUTWARD, DoublesPhase.RETURN):
                raise ValueError("semantic_phase must be OUTWARD or RETURN")
        self.external_control = True

        if self.state == DoublesPhase.OUTWARD and requested_state == DoublesPhase.OUTWARD:
            self.outward_target_y = goal_y
            if return_target_y is not None:
                self.home_y = float(return_target_y)
            return
        if self.state == DoublesPhase.RETURN and requested_state == DoublesPhase.RETURN:
            self._apply_pending_home_config()
            self.home_y = goal_y
            return

        self.move_motion_index = motion_index
        self.move_step = 0
        self.segment_elapsed_s = 0.0
        if requested_state == DoublesPhase.OUTWARD:
            self._capture_outward_x_anchor()
            self.home_y = (
                float(self._pelvis_position[1])
                if return_target_y is None
                else float(return_target_y)
            )
            self.outward_target_y = goal_y
            self._set_state(DoublesPhase.OUTWARD)
        else:
            if not self._outward_x_anchor_valid:
                self._capture_outward_x_anchor()
            self._apply_pending_home_config()
            self.outward_target_y = float(self._pelvis_position[1])
            self.home_y = goal_y
            self._set_state(DoublesPhase.RETURN)
        self._refresh_move_reference()

    def _start_return(self) -> None:
        if not self._outward_x_anchor_valid:
            self._capture_outward_x_anchor()
        self._apply_pending_home_config()
        displacement = self.home_y - float(self._pelvis_position[1])
        self.move_motion_index = self.move_motions.nearest_index(
            self._canonical_move_displacement(displacement),
            label=self._move_label_for_state(DoublesPhase.RETURN),
        )
        self.move_step = 0
        self._set_state(DoublesPhase.RETURN)
        self._refresh_move_reference()

    def _stationary_hold_q(self, lower_body_q: np.ndarray) -> np.ndarray:
        q = np.asarray(lower_body_q, dtype=np.float32).copy()
        for index, name in enumerate(LAB_JOINT_NAMES):
            if "shoulder" in name or "elbow" in name or "wrist" in name:
                q[index] = self._hold_ready_joint_pos[index]
            elif name in ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"):
                q[index] = 0.0
        return q

    def _v9_outward_hold_q(self) -> np.ndarray:
        q = self._hold_ready_joint_pos.copy()
        for index, name in enumerate(LAB_JOINT_NAMES):
            if "shoulder" in name or "elbow" in name or "wrist" in name:
                q[index] = self._hold_nominal_joint_pos[index]
            elif name in ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"):
                q[index] = 0.0
        return q

    def _start_outward_hold(self, entry_reason: str) -> None:
        if self.latch_hold_x_on_entry:
            self.target_hold_x = float(self._pelvis_position[0])
        self._set_state(DoublesPhase.OUTWARD_HOLD)
        self.outward_hold_duration_s = self.outward_hold_s
        self.outward_hold_entry_reason = str(entry_reason)
        self.move_step = int(self.move_motions.lengths[self.move_motion_index] - 1)
        hold_q = (
            self._v9_outward_hold_q()
            if self._common_hold_joint_pos is None
            else self._common_hold_joint_pos
        )
        self._current_q = self._physicalize_move_joint_values(hold_q)
        self._current_qd = np.zeros(29, dtype=np.float32)

    def _start_home_hold(self) -> None:
        if self.latch_hold_x_on_entry:
            self.home_hold_x = float(self._pelvis_position[0])
        self.move_step = int(self.move_motions.lengths[self.move_motion_index] - 1)
        self._set_state(DoublesPhase.HOME_HOLD)
        if self._common_hold_joint_pos is None:
            q, _, _ = self.move_motions.frame(self.move_motion_index, self.move_step)
            hold_q = self._stationary_hold_q(q)
        else:
            hold_q = self._common_hold_joint_pos
        self._current_q = self._physicalize_move_joint_values(hold_q)
        self._current_qd = np.zeros(29, dtype=np.float32)
        self.strike_time = -0.5
        self.racket_target = np.zeros(3, dtype=np.float32)
        self.target_velocity = np.zeros(3, dtype=np.float32)
        self._outward_x_anchor_valid = False

    def _refresh_move_reference(self) -> bool:
        fps = float(self.move_motions.fps[self.move_motion_index])
        requested_step = int(np.round(self.segment_elapsed_s * fps))
        q, qd, step = self.move_motions.frame(self.move_motion_index, requested_step)
        self.move_step = step
        self._current_q = self._physicalize_move_joint_values(q)
        reference_finished = step >= int(self.move_motions.lengths[self.move_motion_index] - 1)
        self._current_qd = (
            np.zeros(29, dtype=np.float32)
            if reference_finished
            else self._physicalize_move_joint_values(qd)
        )
        return reference_finished

    def _lateral_speed(self) -> float:
        return self._planar_velocity()[1]

    def _planar_velocity(self) -> tuple[float, float]:
        if len(self._base_samples) < 2:
            return 0.0, 0.0
        first_t, first_x, first_y = self._base_samples[0]
        last_t, last_x, last_y = self._base_samples[-1]
        elapsed = last_t - first_t
        if elapsed <= 1.0e-9:
            return 0.0, 0.0
        return float((last_x - first_x) / elapsed), float((last_y - first_y) / elapsed)

    def _handoff_ready(
        self,
        pelvis_rpy: np.ndarray,
        pelvis_angular_velocity_w: np.ndarray,
    ) -> bool:
        planar_velocity = self._planar_velocity()
        return bool(
            abs(float(pelvis_rpy[0])) <= self.readiness_roll_rad
            and abs(float(pelvis_rpy[1])) <= self.readiness_pitch_rad
            and abs(planar_velocity[0]) <= self.readiness_linear_speed_mps
            and abs(planar_velocity[1]) <= self.readiness_linear_speed_mps
            and abs(float(pelvis_angular_velocity_w[0])) <= self.readiness_angular_speed_rps
            and abs(float(pelvis_angular_velocity_w[1])) <= self.readiness_angular_speed_rps
        )

    def _stable_at(self, target_y: float) -> bool:
        position_ok = abs(float(target_y) - float(self._pelvis_position[1])) <= self.stable_position_error_m
        velocity_ok = abs(self._lateral_speed()) <= self.stable_lateral_speed_mps
        if not self.v10_relative_x:
            return position_ok and velocity_ok
        x_velocity = self._planar_velocity()[0]
        x_position_ok = abs(self.goal_x - float(self._pelvis_position[0])) <= 0.04
        x_velocity_ok = abs(x_velocity) <= 0.12
        return position_ok and velocity_ok and x_position_ok and x_velocity_ok

    def sync_robot_pose(self, pelvis_position: np.ndarray, torso_position: np.ndarray) -> None:
        """Refresh the pose used by external commands without advancing the scheduler."""
        pelvis = np.asarray(pelvis_position, dtype=np.float32).reshape(3).copy()
        torso = np.asarray(torso_position, dtype=np.float32).reshape(3).copy()
        if not np.isfinite(pelvis).all() or not np.isfinite(torso).all():
            raise ValueError("Scheduler robot positions must be finite.")
        self._pelvis_position = pelvis
        self._torso_position = torso

    def update(
        self,
        time_to_strike: float,
        racket_target: np.ndarray,
        target_velocity: np.ndarray,
        pelvis_position: np.ndarray,
        torso_position: np.ndarray,
        pelvis_rpy: np.ndarray,
        pelvis_angular_velocity_w: np.ndarray,
        dt: float | None = None,
        start_external_hit: bool = False,
    ) -> DoublesReference:
        if not self._initialized:
            self.reset(pelvis_position, torso_position)
        dt = self.control_dt if dt is None else float(dt)
        self._clock_s += dt
        self.sync_robot_pose(pelvis_position, torso_position)
        pelvis_rpy = np.asarray(pelvis_rpy, dtype=np.float32).reshape(3)
        pelvis_angular_velocity_w = np.asarray(
            pelvis_angular_velocity_w, dtype=np.float32
        ).reshape(3)
        if not all(
            np.isfinite(value).all()
            for value in (
                self._pelvis_position,
                self._torso_position,
                pelvis_rpy,
                pelvis_angular_velocity_w,
            )
        ):
            raise ValueError("Scheduler update state must be finite.")
        self._base_samples.append(
            (self._clock_s, float(self._pelvis_position[0]), float(self._pelvis_position[1]))
        )
        self.state_elapsed_s += dt
        self.transition_remaining_s = max(0.0, self.transition_remaining_s - dt)
        self.reference_transition_remaining_s = max(
            0.0, self.reference_transition_remaining_s - dt
        )
        tts = float(np.clip(time_to_strike, -0.5, 0.54))

        if start_external_hit:
            if self.state in (DoublesPhase.HIT, DoublesPhase.POST_DELAY):
                raise RuntimeError("External hit cannot interrupt HIT or POST_DELAY.")
            target = np.asarray(racket_target, dtype=np.float32).reshape(3)
            velocity = np.asarray(target_velocity, dtype=np.float32).reshape(3)
            if not np.isfinite(target).all() or not np.isfinite(velocity).all():
                raise ValueError("External hit target and velocity must be finite.")
            self.external_control = True
            self.pending_outward_target_valid = False
            self.home_y = float(self._pelvis_position[1])
            self._start_hit(tts, target, velocity)
            return self.output()

        if not self.external_control:
            if tts <= -0.5 + 1.0e-6:
                self.hit_armed = True
            elif (
                self.state not in (DoublesPhase.HIT, DoublesPhase.POST_DELAY)
                and self.hit_armed
                and tts < 0.54
            ):
                self._start_hit(tts, racket_target, target_velocity)
                return self.output()

        if self.state == DoublesPhase.HOME_HOLD:
            return self.output()

        if self.state == DoublesPhase.HIT:
            if not self.stationary_hit_crossed and tts > -0.5 + 1.0e-6:
                self.strike_time = tts
                self.racket_target = np.asarray(racket_target, dtype=np.float32).reshape(3).copy()
                self.target_velocity = np.asarray(target_velocity, dtype=np.float32).reshape(3).copy()
            else:
                self.strike_time = max(-0.5, self.strike_time - dt)
            self.hit_step = min(self.hit_step + 1, int(self._hit_lengths[self.hit_motion_index] - 1))
            self._refresh_hit_reference()
            if self.strike_time <= 0.0:
                if self.stationary_hit_test == "hit_home":
                    self.stationary_hit_crossed = True
                    self.stationary_post_hit_elapsed_s += dt
                    if self.stationary_post_hit_elapsed_s + TIME_EPSILON >= self.post_delay_s:
                        self._start_home_hold()
                else:
                    self._set_state(DoublesPhase.POST_DELAY)
            return self.output()

        if self.state == DoublesPhase.POST_DELAY:
            self.strike_time = max(-0.5, self.strike_time - dt)
            self.hit_step = min(self.hit_step + 1, int(self._hit_lengths[self.hit_motion_index] - 1))
            self._refresh_hit_reference()
            if self.state_elapsed_s + TIME_EPSILON >= self.post_delay_s:
                self._start_outward()
            return self.output()

        if self.state == DoublesPhase.OUTWARD_HOLD:
            self.outward_hold_elapsed_s += dt
            if (
                not self.external_control
                and self.outward_hold_elapsed_s + TIME_EPSILON >= self.outward_hold_duration_s
            ):
                self._start_return()
            return self.output()

        self.segment_elapsed_s += dt
        reference_finished = self._refresh_move_reference()
        target_y = self.outward_target_y if self.state == DoublesPhase.OUTWARD else self.home_y
        if self._stable_at(target_y):
            self.stable_elapsed_s += dt
        else:
            self.stable_elapsed_s = 0.0

        if self.state == DoublesPhase.OUTWARD:
            stable_done = self.stable_elapsed_s + TIME_EPSILON >= self.stable_duration_s
            if stable_done or reference_finished:
                self._start_outward_hold("stable" if stable_done else "reference_end")
        elif self.state == DoublesPhase.RETURN:
            stable_done = self.stable_elapsed_s + TIME_EPSILON >= self.stable_duration_s
            if stable_done or (self.reference_end_forces_hold and reference_finished):
                self._start_home_hold()
        return self.output()

    def canonical_pelvis_x(self) -> float:
        if not self.v10_relative_x:
            return float(self._pelvis_position[0])
        return V10_CANONICAL_X_CENTER + float(self._pelvis_position[0]) - self.session_start_x

    def _desired_x(self) -> float:
        if not self.v10_relative_x:
            return self.episode_start_x
        if self.state == DoublesPhase.OUTWARD:
            fps = float(self.move_motions.fps[self.move_motion_index])
            duration_s = max(
                (int(self.move_motions.lengths[self.move_motion_index]) - 1) / fps,
                1.0e-6,
            )
            progress = float(np.clip(self.segment_elapsed_s / duration_s, 0.0, 1.0))
            weight = progress * progress * (3.0 - 2.0 * progress)
            return self.episode_start_x + weight * (self.goal_x - self.episode_start_x)
        if self.state == DoublesPhase.OUTWARD_HOLD:
            return self.target_hold_x
        if self.state == DoublesPhase.HOME_HOLD:
            return self.home_hold_x
        return self.goal_x

    def output(self) -> DoublesReference:
        if not self._initialized:
            raise RuntimeError("DoublesReferenceScheduler.reset() must be called before output().")
        q, qd, phase = self._blended_reference()
        hit_active = self.state in (DoublesPhase.HIT, DoublesPhase.POST_DELAY)
        if self.state in (DoublesPhase.OUTWARD, DoublesPhase.OUTWARD_HOLD):
            target_y = self.outward_target_y
        else:
            target_y = self.home_y
        target_base_x = self._desired_x() - float(self._pelvis_position[0])
        if self.v10_relative_x and self.target_base_x_clip is not None:
            target_base_x = float(
                np.clip(
                    target_base_x,
                    -self.target_base_x_clip,
                    self.target_base_x_clip,
                )
            )
        target_base = np.array(
            [target_base_x, target_y - self._pelvis_position[1]],
            dtype=np.float32,
        )
        if hit_active:
            target_base.fill(0.0)
            strike_time = float(np.clip(self.strike_time, -0.5, 0.54))
            racket_target = self.racket_target.copy()
            target_velocity = self.target_velocity.copy()
        else:
            strike_time = -0.5
            racket_target = np.zeros(3, dtype=np.float32)
            target_velocity = np.zeros(3, dtype=np.float32)
        reference_step = self.hit_step if hit_active else self.move_step
        if hit_active:
            hit_joint_pos = self._hit_q[self.hit_motion_index, self.hit_step].copy()
            hit_joint_vel = self._hit_qd[self.hit_motion_index, self.hit_step].copy()
            hit_anchor_pos_w = (
                self._hit_torso_pos[self.hit_motion_index, self.hit_step]
                + self.hit_anchor_compensation
            ).astype(np.float32, copy=False)
        else:
            hit_joint_pos = q.copy()
            hit_joint_vel = qd.copy()
            hit_anchor_pos_w = self._torso_position.copy()
        return DoublesReference(
            strike_time=strike_time,
            target_velocity=target_velocity,
            racket_target=racket_target,
            target_base=target_base,
            joint_pos=q,
            joint_vel=qd,
            hit_joint_pos=hit_joint_pos,
            hit_joint_vel=hit_joint_vel,
            hit_anchor_pos_w=hit_anchor_pos_w,
            phase=phase,
            state=self.state,
            hit_motion_index=int(self.hit_motion_index),
            move_motion_index=int(self.move_motion_index),
            reference_step=int(reference_step),
            outward_hold_elapsed_s=float(self.outward_hold_elapsed_s),
            outward_hold_duration_s=float(self.outward_hold_duration_s),
            outward_hold_entry_reason=self.outward_hold_entry_reason,
        )
