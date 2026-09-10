"""IsaacSim-only centralized PPO environment for doubles relay.

The trainable policy is deliberately high level.  Every action is decoded into
two lateral base targets and two skill nominations, while the frozen student
selected by ``IsaacDoublesPPOConfig.checkpoint`` is stepped in Isaac for every
low-level frame.  The
relay gate owns shot alternation and the commit edge; a safety projection can
reject or move a target, but it never cancels an already-started HIT phase.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from .centralized import (
    ACTION_VERSION,
    OBSERVATION_NAMES_V2,
    SKILL_NAMES,
    DoublesCommandAction,
    DoublesObservationConfig,
    observation_names_v2,
)


@dataclass(frozen=True)
class IsaacRelayRewardConfig:
    """Reward weights for one high-level Isaac transition."""

    strike_position_scale_m: float = 0.04
    strike_velocity_scale_mps: float = 0.50
    strike_orientation_scale_rad: float = 0.05
    strike_timing_scale_s: float = 0.02
    strike_position_weight: float = 2.0
    strike_velocity_weight: float = 1.0
    strike_orientation_weight: float = 0.6
    strike_timing_weight: float = 1.0
    base_distance_threshold_m: float = 0.47
    hand_distance_threshold_m: float = 0.20
    racket_distance_threshold_m: float = 0.25
    base_ttc_threshold_s: float = 0.35
    hand_ttc_threshold_s: float = 0.15
    racket_ttc_threshold_s: float = 0.15
    base_safety_weight: float = 5.0
    hand_safety_weight: float = 8.0
    racket_safety_weight: float = 8.0
    ttc_safety_weight: float = 3.0
    simultaneous_hit_penalty: float = 25.0
    simultaneous_hit_request_penalty: float = 2.0
    phase_interruption_penalty: float = 12.0
    safety_projection_penalty: float = 0.20
    masked_commit_penalty: float = 0.50
    stale_input_penalty: float = 1.0
    commit_reward: float = 0.20
    peer_clear_reward: float = 0.20
    ready_at_commit_reward: float = 0.20
    handoff_reward: float = 1.0
    timeout_penalty: float = 8.0
    termination_penalty: float = 12.0

    def __post_init__(self) -> None:
        for name, value in vars(self).items():
            if not math.isfinite(float(value)) or float(value) < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        for name in (
            "strike_position_scale_m",
            "strike_velocity_scale_mps",
            "strike_orientation_scale_rad",
            "strike_timing_scale_s",
            "base_distance_threshold_m",
            "hand_distance_threshold_m",
            "racket_distance_threshold_m",
            "base_ttc_threshold_s",
            "hand_ttc_threshold_s",
            "racket_ttc_threshold_s",
        ):
            if float(getattr(self, name)) <= 0.0:
                raise ValueError(f"{name} must be positive")


def compute_isaac_relay_reward(
    *,
    strike_mask: Any,
    position_error_m: Any,
    velocity_error_mps: Any,
    orientation_error_rad: Any,
    timing_error_s: Any,
    base_distance_m: Any,
    hand_distance_m: Any,
    racket_distance_m: Any,
    base_ttc_s: Any,
    hand_ttc_s: Any,
    racket_ttc_s: Any,
    simultaneous_hit: Any,
    phase_interruption: Any,
    safety_projected: Any,
    masked_commit: Any,
    stale_input: Any,
    commit_started: Any,
    peer_clear_at_commit: Any,
    ready_at_commit: Any,
    handoff_completed: Any,
    timeout: Any,
    simultaneous_hit_request: Any = None,
    terminated: Any = None,
    config: IsaacRelayRewardConfig | None = None,
) -> tuple[Any, dict[str, Any]]:
    """Compute finite per-environment rewards on the input tensor device."""

    import torch

    config = config or IsaacRelayRewardConfig()
    reference = torch.as_tensor(position_error_m)
    if reference.numel() == 0:
        raise ValueError("reward inputs must contain at least one environment")
    batch_size = reference.numel()

    def tensor(value: Any) -> torch.Tensor:
        result = torch.as_tensor(value, device=reference.device, dtype=torch.float32).reshape(-1)
        if result.numel() == 1 and batch_size != 1:
            result = result.expand(batch_size)
        if result.numel() != batch_size:
            raise ValueError(f"reward input has {result.numel()} values; expected {batch_size}")
        return torch.nan_to_num(result, nan=0.0, posinf=10.0, neginf=-10.0)

    def flag(value: Any) -> torch.Tensor:
        return tensor(value).to(dtype=torch.float32)

    strike = flag(strike_mask)
    position = torch.exp(-torch.square(tensor(position_error_m) / config.strike_position_scale_m)) * strike * config.strike_position_weight
    velocity = torch.exp(-torch.square(tensor(velocity_error_mps) / config.strike_velocity_scale_mps)) * strike * config.strike_velocity_weight
    orientation = torch.exp(-torch.square(tensor(orientation_error_rad) / config.strike_orientation_scale_rad)) * strike * config.strike_orientation_weight
    timing = torch.exp(-torch.square(tensor(timing_error_s) / config.strike_timing_scale_s)) * strike * config.strike_timing_weight

    def clearance_penalty(distance: Any, threshold: float, weight: float) -> torch.Tensor:
        return -weight * torch.square(torch.relu(threshold - tensor(distance)) / threshold)

    def ttc_penalty(value: Any, threshold: float) -> torch.Tensor:
        ttc = tensor(value)
        return -config.ttc_safety_weight * torch.square(torch.relu(threshold - ttc) / threshold)

    if terminated is None:
        terminated = torch.zeros(batch_size, device=reference.device)
    if simultaneous_hit_request is None:
        simultaneous_hit_request = torch.zeros(batch_size, device=reference.device)
    terms: dict[str, torch.Tensor] = {
        "strike_position_quality": position,
        "strike_velocity_quality": velocity,
        "strike_orientation_quality": orientation,
        "strike_timing_quality": timing,
        "base_clearance": clearance_penalty(base_distance_m, config.base_distance_threshold_m, config.base_safety_weight),
        "hand_clearance": clearance_penalty(hand_distance_m, config.hand_distance_threshold_m, config.hand_safety_weight),
        "racket_clearance": clearance_penalty(racket_distance_m, config.racket_distance_threshold_m, config.racket_safety_weight),
        "base_ttc": ttc_penalty(base_ttc_s, config.base_ttc_threshold_s),
        "hand_ttc": ttc_penalty(hand_ttc_s, config.hand_ttc_threshold_s),
        "racket_ttc": ttc_penalty(racket_ttc_s, config.racket_ttc_threshold_s),
        "simultaneous_hit": -config.simultaneous_hit_penalty * flag(simultaneous_hit),
        "simultaneous_hit_request": -config.simultaneous_hit_request_penalty * flag(simultaneous_hit_request),
        "phase_interruption": -config.phase_interruption_penalty * flag(phase_interruption),
        "safety_projection": -config.safety_projection_penalty * flag(safety_projected),
        "masked_commit": -config.masked_commit_penalty * flag(masked_commit),
        "stale_input": -config.stale_input_penalty * flag(stale_input),
        "commit_started": config.commit_reward * flag(commit_started),
        "peer_clear_at_commit": config.peer_clear_reward * flag(peer_clear_at_commit),
        "ready_at_commit": config.ready_at_commit_reward * flag(ready_at_commit),
        "handoff_completed": config.handoff_reward * flag(handoff_completed),
        "timeout": -config.timeout_penalty * flag(timeout),
        "termination": -config.termination_penalty * flag(terminated),
    }
    total = torch.stack(tuple(terms.values()), dim=0).sum(dim=0)
    return total, terms


@dataclass(frozen=True)
class IsaacDoublesPPOConfig:
    controller_root: Path = Path("../pingpang_controller")
    checkpoint: Path = Path("model_23000.pt")
    move_motion_data: Path | None = None
    hit_motion_data: Path | None = None
    num_envs: int = 64
    low_level_steps: int = 2
    warmup_steps: int = 150
    max_episode_steps: int = 600
    shots_per_episode: int = 8
    seed: int = 10000
    device: str = "cuda:0"
    task: str = "Tracking-Doubles-Left-RobustTeacher-Hold6-G1-v0"
    render_mode: str | None = None
    locomotion_preemption: bool = False
    lead_time_s: float = 0.85
    reservation_duration_s: float = 0.08
    minimum_commit_tts_s: float = 0.12
    maximum_commit_tts_s: float = 0.55
    minimum_hand_distance_m: float = 0.20
    minimum_racket_distance_m: float = 0.25
    incoming_ball_velocity_mps: tuple[float, float, float] = (-3.2, 0.0, -0.25)
    target_racket_velocity_mps: tuple[float, float, float] = (1.9, 0.0, 0.7)
    ball_target_abs_y_range_m: tuple[float, float] = (0.08, 0.50)
    ball_target_z_range_m: tuple[float, float] = (0.95, 1.15)
    ball_velocity_x_range_mps: tuple[float, float] = (-3.8, -2.6)
    ball_velocity_y_range_mps: tuple[float, float] = (-0.60, 0.60)
    ball_velocity_z_range_mps: tuple[float, float] = (-0.50, 0.10)
    observation: DoublesObservationConfig = field(default_factory=DoublesObservationConfig)
    reward: IsaacRelayRewardConfig = field(default_factory=IsaacRelayRewardConfig)

    def __post_init__(self) -> None:
        if any(
            value <= 0
            for value in (
                self.num_envs,
                self.low_level_steps,
                self.max_episode_steps,
                self.shots_per_episode,
            )
        ) or self.warmup_steps < 0:
            raise ValueError("Isaac PPO sizes must be positive")
        if self.lead_time_s <= 0.0 or self.reservation_duration_s < 0.0:
            raise ValueError("lead time must be positive and reservation duration non-negative")
        if self.minimum_commit_tts_s < 0.0 or self.maximum_commit_tts_s <= self.minimum_commit_tts_s:
            raise ValueError("commit timing window must be increasing")
        if self.maximum_commit_tts_s > self.lead_time_s:
            raise ValueError("maximum commit time cannot exceed reservation lead time")
        for name in ("incoming_ball_velocity_mps", "target_racket_velocity_mps"):
            value = np.asarray(getattr(self, name), dtype=float).reshape(-1)
            if value.size != 3 or not np.isfinite(value).all():
                raise ValueError(f"{name} must contain three finite values")
        for name in (
            "ball_target_abs_y_range_m",
            "ball_target_z_range_m",
            "ball_velocity_x_range_mps",
            "ball_velocity_y_range_mps",
            "ball_velocity_z_range_mps",
        ):
            low, high = getattr(self, name)
            if not math.isfinite(low) or not math.isfinite(high) or high < low:
                raise ValueError(f"{name} must be finite and ordered")
        workspace_low, workspace_high = self.observation.planner.workspace_y
        minimum_gap = max(self.observation.planner.min_separation, self.reward.base_distance_threshold_m)
        if workspace_high - workspace_low < minimum_gap:
            raise ValueError("planner workspace is narrower than the required base separation")


class IsaacDoublesHighLevelEnv:
    """Batched high-level environment around a real Isaac tracking task."""

    num_actions = DoublesCommandAction.num_actions
    _RESERVED, _PREPARED, _COMMITTED, _STRIKE, _FOLLOW_THROUGH, _HANDOFF = range(6)

    def __init__(
        self,
        env: Any,
        policy: Any,
        *,
        config: IsaacDoublesPPOConfig,
        commands: Mapping[str, Any],
        robots: Mapping[str, Any],
        action_terms: Mapping[str, Any],
        joint_names: Sequence[str],
        body_indices: Mapping[str, Mapping[str, Any]],
        mirror_observation: Callable[..., Any],
        mirror_action: Callable[..., Any],
        distill_hit_errors: Callable[..., Any],
        auto_reset: bool = True,
    ) -> None:
        import torch

        self.env = env
        self.unwrapped = env.unwrapped
        self.policy = policy.eval()
        self.policy.requires_grad_(False)
        self.config = config
        self.commands = dict(commands)
        self.robots = dict(robots)
        self.action_terms = dict(action_terms)
        self.joint_names = tuple(joint_names)
        self.body_indices = body_indices
        self.mirror_observation = mirror_observation
        self.mirror_action = mirror_action
        self.distill_hit_errors = distill_hit_errors
        self.device = torch.device(self.unwrapped.device)
        self.num_envs = int(self.unwrapped.num_envs)
        if self.num_envs != config.num_envs:
            raise ValueError(f"Isaac scene has {self.num_envs} envs, config requested {config.num_envs}")
        self.observation_names = observation_names_v2(config.observation)
        self.observation_size = len(self.observation_names)
        self._name_to_index = {name: index for index, name in enumerate(self.observation_names)}
        self.max_episode_length = config.max_episode_steps
        self.episode_length_buf = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.episode_return = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        self.sim_time = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        self.stage = torch.full((self.num_envs,), self._RESERVED, dtype=torch.long, device=self.device)
        self.stage_elapsed = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        self.next_hitter = torch.arange(self.num_envs, device=self.device, dtype=torch.long) % 2
        self.active_hitter = torch.full((self.num_envs,), -1, dtype=torch.long, device=self.device)
        self.shots_completed = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.shot_id = torch.zeros(self.num_envs, dtype=torch.long, device=self.device)
        self.ball_position = torch.zeros(self.num_envs, 3, dtype=torch.float32, device=self.device)
        self.ball_target = torch.zeros_like(self.ball_position)
        self.ball_velocity = torch.zeros_like(self.ball_position)
        self.ball_acceleration = torch.zeros_like(self.ball_position)
        self.ball_prediction_age = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        self.time_to_strike = torch.zeros(self.num_envs, dtype=torch.float32, device=self.device)
        self.previous_phase = torch.full((self.num_envs, 2), 5, dtype=torch.long, device=self.device)
        self.previous_targets = torch.zeros((self.num_envs, 2), dtype=torch.float32, device=self.device)
        self.previous_skills = torch.zeros((self.num_envs, 2), dtype=torch.long, device=self.device)
        self._last_observations: Any | None = None
        self._last_underlying_done = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        self._reset_internal(torch.arange(self.num_envs, device=self.device), new_episode=True)
        if auto_reset:
            self.reset()

    def _all_ids(self) -> Any:
        import torch

        return torch.arange(self.num_envs, device=self.device, dtype=torch.long)

    def _as_ids(self, env_ids: Any) -> Any:
        import torch

        return torch.as_tensor(env_ids, device=self.device, dtype=torch.long).reshape(-1)

    def _set_command_clear(self, ids: Any) -> None:
        for command in self.commands.values():
            clear = getattr(command, "clear_external_control", None)
            if callable(clear):
                try:
                    clear(ids)
                except (RuntimeError, ValueError):
                    pass

    def _reset_commands_to_home(self, ids: Any) -> None:
        """Give the high-level relay exclusive ownership after an Isaac reset.

        ``DoublesDistillCommand`` intentionally starts in ``HIT`` for the
        single-policy teacher task.  That default is unsafe for a centralized
        relay: both commands can enter a spontaneous hit while the wrapper is
        warming the frozen students.  Use the controller's phase transition
        helper when available, then suppress its autonomous next-hit timer so
        only the relay commit gate can start a stroke.
        """

        import torch

        ids = self._as_ids(ids)
        if ids.numel() == 0:
            return
        origins = self._env_origins()
        for name in ("left", "right"):
            command = self.commands[name]
            clear = getattr(command, "clear_external_control", None)
            if callable(clear):
                try:
                    clear(ids)
                except (RuntimeError, ValueError, AttributeError) as error:
                    raise RuntimeError(f"failed to clear {name} external control") from error

            state = getattr(command, "doubles_state", None)
            if state is None:
                continue
            home_state = int(getattr(command, "HOME_HOLD", 5))
            set_state = getattr(command, "_set_state", None)
            if callable(set_state):
                try:
                    set_state(ids, home_state)
                except (RuntimeError, ValueError, AttributeError) as error:
                    raise RuntimeError(f"failed to place {name} command in HOME_HOLD") from error
            else:
                state[ids] = home_state
            state[ids] = home_state

            pelvis = int(self.body_indices[name]["pelvis"])
            current_y = self.robots[name].data.body_pos_w[ids, pelvis, 1] - origins[ids, 1]
            for attribute in ("home_y", "home_target_y", "target_y"):
                value = getattr(command, attribute, None)
                if value is not None:
                    value[ids] = current_y
            for attribute, fill in (
                ("since_strike_s", 0.0),
                ("post_delay_s", 0.0),
                ("return_earliest_s", 0.0),
                ("return_latest_s", 0.0),
                ("state_elapsed_s", 0.0),
                ("segment_elapsed_s", 0.0),
                ("stable_elapsed_s", 0.0),
                ("outward_hold_elapsed_s", 0.0),
                ("outward_hold_duration_s", 0.0),
            ):
                value = getattr(command, attribute, None)
                if value is not None:
                    value[ids] = fill
            next_hit = getattr(command, "next_hit_s", None)
            if next_hit is not None:
                next_hit[ids] = float("inf")
            external = getattr(command, "external_hit_control", None)
            if external is not None:
                external[ids] = False
            pending = getattr(command, "pending_outward_target_valid", None)
            if pending is not None:
                pending[ids] = False
            refresh = getattr(command, "_refresh_external_move", None)
            if callable(refresh):
                try:
                    refresh(ids)
                except (RuntimeError, ValueError, AttributeError) as error:
                    raise RuntimeError(f"failed to refresh {name} HOME_HOLD reference") from error
            previous_state = getattr(command, "previous_state", None)
            if previous_state is not None:
                previous_state[ids] = home_state
            transition = getattr(command, "transition_remaining_s", None)
            if transition is not None:
                transition[ids] = 0.0
            body_valid = getattr(command, "body_transition_valid", None)
            if body_valid is not None:
                body_valid[ids] = False
            for previous_name, current_name in (
                ("previous_joint_pos", "student_joint_pos"),
                ("previous_joint_vel", "student_joint_vel"),
                ("previous_anchor_pos_w", "student_anchor_pos_w"),
                ("previous_body_pos", "bodytarget_step"),
                ("previous_body_quat", "body_quat_step"),
                ("previous_body_lin_vel", "body_lin_vel_step"),
                ("previous_body_ang_vel", "body_ang_vel_step"),
            ):
                previous = getattr(command, previous_name, None)
                current = getattr(command, current_name, None)
                if previous is not None and current is not None:
                    previous[ids] = current[ids]

    def _sample_ball(self, ids: Any) -> None:
        import torch

        if ids.numel() == 0:
            return
        minimum_lead = min(self.config.lead_time_s, self.config.maximum_commit_tts_s + 0.10)
        self.time_to_strike[ids] = minimum_lead + (self.config.lead_time_s - minimum_lead) * torch.rand(ids.numel(), device=self.device)
        self.ball_target[ids] = 0.0
        self.ball_target[ids, 0] = 0.45
        z_low, z_high = self.config.ball_target_z_range_m
        self.ball_target[ids, 2] = z_low + (z_high - z_low) * torch.rand(ids.numel(), device=self.device)
        side = torch.where(
            self.next_hitter[ids] == 0,
            -torch.ones(ids.numel(), device=self.device),
            torch.ones(ids.numel(), device=self.device),
        )
        y_low, y_high = self.config.ball_target_abs_y_range_m
        self.ball_target[ids, 1] = side * (y_low + (y_high - y_low) * torch.rand(ids.numel(), device=self.device))
        x_low, x_high = self.config.ball_velocity_x_range_mps
        lateral_low, lateral_high = self.config.ball_velocity_y_range_mps
        vertical_low, vertical_high = self.config.ball_velocity_z_range_mps
        self.ball_velocity[ids, 0] = x_low + (x_high - x_low) * torch.rand(ids.numel(), device=self.device)
        self.ball_velocity[ids, 1] = lateral_low + (lateral_high - lateral_low) * torch.rand(ids.numel(), device=self.device)
        self.ball_velocity[ids, 2] = vertical_low + (vertical_high - vertical_low) * torch.rand(ids.numel(), device=self.device)
        self.ball_acceleration[ids] = torch.tensor([0.0, 0.0, -9.81], device=self.device)
        self.ball_position[ids] = (
            self.ball_target[ids]
            - self.ball_velocity[ids] * self.time_to_strike[ids, None]
            - 0.5 * self.ball_acceleration[ids] * torch.square(self.time_to_strike[ids, None])
        )
        self.ball_prediction_age[ids] = 0.0

    def _reset_internal(
        self,
        env_ids: Any,
        *,
        new_episode: bool = True,
        commands_already_reset: Any | None = None,
    ) -> None:
        import torch

        ids = self._as_ids(env_ids)
        if ids.numel() == 0:
            return
        if commands_already_reset is None:
            self._reset_commands_to_home(ids)
        else:
            already = self._done_tensor(commands_already_reset, self.device, self.num_envs)
            self._reset_commands_to_home(ids[~already[ids]])
        self.stage[ids] = self._RESERVED
        self.stage_elapsed[ids] = 0.0
        self.sim_time[ids] = 0.0
        self.active_hitter[ids] = -1
        self.previous_phase[ids] = 5
        self.previous_targets[ids] = 0.0
        self.previous_skills[ids] = 0
        self.time_to_strike[ids] = self.config.lead_time_s
        if new_episode:
            self.episode_length_buf[ids] = 0
            self.episode_return[ids] = 0.0
            self.shots_completed[ids] = 0
            self.next_hitter[ids] = ids % 2
            self.shot_id[ids] += 1
        self._sample_ball(ids)

    def _start_next_shot(self, ids: Any) -> None:
        ids = self._as_ids(ids)
        if ids.numel() == 0:
            return
        self.next_hitter[ids] = 1 - self.next_hitter[ids]
        self.active_hitter[ids] = -1
        self.stage[ids] = self._RESERVED
        self.stage_elapsed[ids] = 0.0
        self.shot_id[ids] += 1
        self._sample_ball(ids)

    def reset(self) -> tuple[Any, dict[str, Any]]:
        observations, info = self.env.reset()
        self._reset_internal(self._all_ids(), new_episode=True)
        if self.config.warmup_steps:
            observations = self._run_low_level(observations, self.config.warmup_steps, advance_planner=False)
            self._reset_internal(self._all_ids(), new_episode=True)
        self._last_observations = observations
        return self._build_observation(), dict(info or {})

    @staticmethod
    def _done_tensor(value: Any, device: Any, size: int) -> Any:
        import torch

        result = torch.as_tensor(value, device=device, dtype=torch.bool).reshape(-1)
        if result.numel() == 1 and size != 1:
            result = result.expand(size)
        if result.numel() != size:
            raise ValueError(f"done tensor has {result.numel()} values; expected {size}")
        return result

    def _low_level_step(
        self,
        observations: Mapping[str, Any],
        *,
        inactive: Any | None = None,
    ) -> tuple[Any, Any, Any, Mapping[str, Any]]:
        import torch

        def history_parameter(value: Any) -> Any:
            parameter = torch.as_tensor(value, device=self.device)
            if parameter.ndim == 2 and parameter.shape[0] == self.num_envs:
                return parameter[:, None, :]
            return parameter

        with torch.no_grad():
            right_action = self.policy(observations["policy"])
            left_observation = self.mirror_observation(
                observations["policy_left"],
                self.joint_names,
                history_parameter(self.action_terms["left"]._offset),
                history_parameter(self.action_terms["left"]._scale),
                history_parameter(self.action_terms["right"]._offset),
                history_parameter(self.action_terms["right"]._scale),
            )
            left_right_action = self.policy(left_observation)
            left_action = self.mirror_action(
                left_right_action,
                self.joint_names,
                self.action_terms["right"]._offset,
                self.action_terms["right"]._scale,
                self.action_terms["left"]._offset,
                self.action_terms["left"]._scale,
            )
            combined = torch.cat((right_action, left_action), dim=-1)
        inactive_mask = (
            torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
            if inactive is None
            else self._done_tensor(inactive, self.device, self.num_envs)
        )
        pending_infeasible = self._guard_controller_pending_outward_targets()
        pending_infeasible &= ~inactive_mask
        if pending_infeasible.any():
            ids = torch.nonzero(pending_infeasible, as_tuple=False).reshape(-1)
            try:
                self.unwrapped.reset(env_ids=ids)
                self._reset_commands_to_home(ids)
            except (RuntimeError, TypeError, ValueError, AttributeError) as error:
                raise RuntimeError(
                    f"failed to reset infeasible pending outward environments: env_ids={ids.tolist()}"
                ) from error
        combined[inactive_mask | pending_infeasible] = 0.0
        self._guard_controller_return_targets()
        try:
            result = self.env.step(combined)
        except (RuntimeError, TypeError, ValueError) as error:
            details: list[str] = []
            origins = self._env_origins()
            for name in ("left", "right"):
                command = self.commands[name]
                state = self._phase_tensor(name)
                if not torch.any((state == 2) | (state == 3)):
                    continue
                pelvis = int(self.body_indices[name]["pelvis"])
                current = self.robots[name].data.body_pos_w[:, pelvis, 1] - origins[:, 1]
                target = getattr(command, "home_target_y", None)
                if target is None:
                    continue
                motion_index = getattr(command, "return_motion_index", None)
                reference = getattr(command, "motion_targets_t", None)
                reference_y = "unknown"
                if motion_index is not None and reference is not None:
                    reference_y = str(reference[motion_index, 1].detach().cpu().tolist())
                details.append(
                    f"{name}:phase={state.detach().cpu().tolist()},pelvis_y={current.detach().cpu().tolist()},"
                    f"home_target_y={target.detach().cpu().tolist()},return_reference_y={reference_y}"
                )
            suffix = f"; {' | '.join(details)}" if details else ""
            raise type(error)(f"Isaac doubles controller step failed: {error}{suffix}") from error
        if len(result) != 5:
            raise RuntimeError("Isaac tracking environment must return observations, reward, terminated, truncated, info")
        observations, _reward, terminated, truncated, info = result
        terminated_tensor = self._done_tensor(terminated, self.device, self.num_envs)
        truncated_tensor = self._done_tensor(truncated, self.device, self.num_envs) | pending_infeasible
        step_info = dict(info or {})
        step_info["commands_reset"] = pending_infeasible
        return observations, terminated_tensor, truncated_tensor, step_info

    def _guard_controller_pending_outward_targets(self) -> Any:
        import torch

        infeasible = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        origins = self._env_origins()
        for robot_index, name in enumerate(("left", "right")):
            command = self.commands[name]
            pending = getattr(command, "pending_outward_target_valid", None)
            pending_target = getattr(command, "pending_outward_target_y", None)
            cfg = getattr(command, "cfg", None)
            outward_target = float(getattr(cfg, "outward_target_y", 0.0))
            if pending is None or pending_target is None or abs(outward_target) <= 1.0e-6:
                continue
            try:
                valid = torch.as_tensor(pending, device=self.device, dtype=torch.bool).reshape(-1)
                target = torch.as_tensor(
                    pending_target, device=self.device, dtype=torch.float32
                ).reshape(-1)
            except (RuntimeError, TypeError, ValueError):
                continue
            if valid.numel() != self.num_envs or target.numel() != self.num_envs:
                continue
            pelvis = int(self.body_indices[name]["pelvis"])
            current = self.robots[name].data.body_pos_w[:, pelvis, 1] - origins[:, 1]
            velocity = self.robots[name].data.body_lin_vel_w[:, pelvis, 1]
            sign = 1.0 if outward_target > 0.0 else -1.0
            margin = self._controller_step_target_margin(command)
            predicted = current + velocity * self._step_dt()
            nominal = torch.full_like(target, outward_target)
            preferred = torch.where(
                (target - predicted) * sign >= margin,
                target,
                nominal,
            )
            corrected, feasible = self._project_controller_motion_target(
                robot_index,
                current,
                preferred,
                expected_class=True,
                allow_stationary=False,
            )
            failed = valid & ~feasible
            infeasible |= failed
            corrected = torch.where(valid & feasible, corrected, target)
            if isinstance(pending_target, torch.Tensor):
                pending_target.copy_(corrected.reshape_as(pending_target))
            else:
                try:
                    pending_target[...] = corrected.detach().cpu().numpy()
                except (TypeError, ValueError):
                    continue
        return infeasible

    def _project_physical_outward_target(
        self,
        robot_index: int,
        current: Any,
        *,
        outward_speed: Any | None = None,
        transition_horizon_s: Any | None = None,
    ) -> tuple[Any, Any]:
        import torch

        name = ("left", "right")[robot_index]
        command = self.commands[name]
        cfg = getattr(command, "cfg", None)
        outward = float(getattr(cfg, "outward_target_y", 0.0))
        if abs(outward) <= 1.0e-6:
            raise ValueError(f"{name} command has no physical outward direction")
        value = torch.as_tensor(current, device=self.device, dtype=torch.float32).reshape(-1)
        sign = 1.0 if outward > 0.0 else -1.0
        low, high = self.config.observation.planner.workspace_y
        boundary = float(high if sign > 0.0 else low)
        speed = torch.zeros_like(value) if outward_speed is None else torch.as_tensor(
            outward_speed, device=self.device, dtype=torch.float32
        ).reshape(-1)
        horizon = torch.zeros_like(value) if transition_horizon_s is None else torch.as_tensor(
            transition_horizon_s, device=self.device, dtype=torch.float32
        ).reshape(-1)
        if speed.shape != value.shape or horizon.shape != value.shape:
            raise ValueError("outward speed and transition horizon must match the target batch")
        runway = (boundary - value) * sign
        required_runway = self._controller_step_target_margin(command) + torch.relu(speed) * torch.clamp(horizon, min=0.0)
        finite = torch.isfinite(value) & torch.isfinite(speed) & torch.isfinite(horizon)
        runway_feasible = finite & (horizon >= 0.0) & (runway >= required_runway)
        nominal = torch.full_like(value, outward)
        projected, motion_feasible = self._project_controller_motion_target(
            robot_index,
            value,
            nominal,
            expected_class=True,
            allow_stationary=False,
        )
        return projected, runway_feasible & motion_feasible

    def _physical_outward_target(
        self,
        robot_index: int,
        current: Any,
        *,
        outward_speed: Any | None = None,
        transition_horizon_s: Any | None = None,
    ) -> Any:
        import torch

        name = ("left", "right")[robot_index]
        projected, feasible = self._project_physical_outward_target(
            robot_index,
            current,
            outward_speed=outward_speed,
            transition_horizon_s=transition_horizon_s,
        )
        if not feasible.all():
            ids = torch.nonzero(~feasible, as_tuple=False).reshape(-1).tolist()
            raise ValueError(f"{name} has insufficient outward runway for HIT commit: env_ids={ids}")
        return projected

    @staticmethod
    def _commit_post_delay(command: Any) -> float:
        cfg = command.cfg
        if bool(getattr(cfg, "robust_handoff_readiness", False)):
            return float(cfg.readiness_min_post_delay_s)
        return max(tuple(float(value) for value in cfg.post_delay_range_s))

    def _controller_step_target_margin(self, command: Any) -> float:
        cfg = getattr(command, "cfg", None)
        deadband = max(float(getattr(cfg, "external_target_deadband_m", 0.01)), 0.0)
        speed = max(float(self.config.observation.planner.max_base_speed), 0.0)
        return deadband + speed * self._step_dt() + 1.0e-3

    def _controller_motion_choice(
        self,
        command: Any,
        displacement: Any,
        *,
        expected_class: bool,
        allow_stationary: bool,
    ) -> tuple[Any, Any, Any]:
        import torch

        value = torch.as_tensor(displacement, device=self.device, dtype=torch.float32).reshape(-1)
        targets = torch.as_tensor(command.motion_targets_t, device=self.device, dtype=torch.float32)
        labels = torch.as_tensor(command.motion_labels_t, device=self.device, dtype=torch.bool).reshape(-1)
        actual = torch.as_tensor(
            command._motion_pelvis_displacement_y,
            device=self.device,
            dtype=torch.float32,
        ).reshape(-1)
        outbound = torch.as_tensor(command._outbound_motion_indices, device=self.device, dtype=torch.long).reshape(-1)
        returning = torch.as_tensor(command._return_motion_indices, device=self.device, dtype=torch.long).reshape(-1)
        if targets.ndim != 2 or targets.shape[1] < 2 or targets.shape[0] != labels.numel() or actual.numel() != labels.numel():
            raise ValueError("controller motion reference tensors have inconsistent shapes")
        candidates = torch.cat((outbound, returning))
        if candidates.numel() == 0:
            raise ValueError("controller motion bank has no selectable references")
        csv_y = targets[:, 1]
        deadband = max(float(getattr(command.cfg, "external_target_deadband_m", 0.01)), 0.0)
        moving = torch.abs(value) > deadband
        selected = torch.zeros(value.numel(), device=self.device, dtype=torch.long)
        selectable = torch.zeros(value.numel(), device=self.device, dtype=torch.bool)
        if moving.any():
            candidate_y = csv_y[candidates]
            same_sign = value[moving, None] * candidate_y[None, :] > 0.0
            has_same_sign = same_sign.any(dim=1)
            errors = torch.abs(value[moving, None] - candidate_y[None, :])
            errors = torch.where(same_sign, errors, torch.full_like(errors, torch.inf))
            selected[moving] = candidates[torch.argmin(errors, dim=1)]
            selectable[moving] = has_same_sign
        stationary = ~moving
        if stationary.any() and allow_stationary:
            stationary_candidates = returning if expected_class else outbound
            if stationary_candidates.numel() > 0:
                errors = torch.abs(value[stationary, None] - csv_y[stationary_candidates][None, :])
                selected[stationary] = stationary_candidates[torch.argmin(errors, dim=1)]
                selectable[stationary] = True
        scale = self._controller_motion_scale(command, selected, value)
        feasible = (
            selectable
            & (labels[selected] == bool(expected_class))
            & torch.isfinite(scale)
            & (scale >= -1.0e-6)
        )
        return feasible, selected, scale

    def _controller_motion_scale(self, command: Any, indices: Any, displacement: Any) -> Any:
        import torch

        motion_indices = torch.as_tensor(indices, device=self.device, dtype=torch.long).reshape(-1)
        value = torch.as_tensor(displacement, device=self.device, dtype=torch.float32).reshape(-1)
        actual = torch.as_tensor(
            command._motion_pelvis_displacement_y,
            device=self.device,
            dtype=torch.float32,
        ).reshape(-1)[motion_indices]
        csv_y = torch.as_tensor(command.motion_targets_t, device=self.device, dtype=torch.float32)[
            motion_indices, 1
        ]
        scale_base = torch.where(torch.abs(actual) > 1.0e-4, actual, csv_y)
        scale_sign = torch.sign(scale_base)
        fallback_sign = torch.sign(value)
        scale_sign = torch.where(
            scale_sign == 0.0,
            torch.where(fallback_sign == 0.0, torch.ones_like(fallback_sign), fallback_sign),
            scale_sign,
        )
        scale_base = scale_sign * torch.clamp(torch.abs(scale_base), min=1.0e-4)
        return value / scale_base

    def _project_controller_motion_target(
        self,
        robot_index: int,
        current: Any,
        requested: Any,
        *,
        expected_class: bool,
        allow_stationary: bool,
    ) -> tuple[Any, Any]:
        import torch

        name = ("left", "right")[robot_index]
        command = self.commands[name]
        current_y = torch.as_tensor(current, device=self.device, dtype=torch.float32).reshape(-1)
        requested_y = torch.as_tensor(requested, device=self.device, dtype=torch.float32).reshape(-1)
        if current_y.shape != requested_y.shape:
            raise ValueError("current and requested controller targets must have matching shapes")
        outward = float(getattr(getattr(command, "cfg", None), "outward_target_y", 0.0))
        if abs(outward) <= 1.0e-6:
            raise ValueError(f"{name} command has no physical outward direction")
        direction = (1.0 if outward > 0.0 else -1.0) * (1.0 if expected_class else -1.0)
        margin = self._controller_step_target_margin(command)
        low, high = self.config.observation.planner.workspace_y
        nominal_value = (
            outward
            if expected_class
            else float(self.config.observation.planner.home_y[robot_index])
        )
        nominal = torch.full_like(current_y, nominal_value)
        boundary = torch.full_like(current_y, float(high if direction > 0.0 else low))
        minimum_step = current_y + direction * margin
        target_y = torch.as_tensor(command.motion_targets_t, device=self.device, dtype=torch.float32)[:, 1]
        labels = torch.as_tensor(command.motion_labels_t, device=self.device, dtype=torch.bool).reshape(-1)
        reference_steps = current_y[:, None] + target_y[labels == bool(expected_class)][None, :]
        candidates = torch.cat(
            (
                requested_y[:, None],
                nominal[:, None],
                minimum_step[:, None],
                boundary[:, None],
                reference_steps,
            ),
            dim=1,
        )
        if allow_stationary:
            candidates = torch.cat((candidates, current_y[:, None]), dim=1)
        candidates = torch.clamp(candidates, float(low), float(high))
        count, width = candidates.shape
        displacement = (candidates - current_y[:, None]).reshape(-1)
        feasible, _selected, _scale = self._controller_motion_choice(
            command,
            displacement,
            expected_class=expected_class,
            allow_stationary=allow_stationary,
        )
        feasible = feasible.reshape(count, width)
        finite = torch.isfinite(candidates) & torch.isfinite(current_y[:, None])
        feasible &= finite
        if allow_stationary:
            deadband = max(float(getattr(command.cfg, "external_target_deadband_m", 0.01)), 0.0)
            displacement_matrix = candidates - current_y[:, None]
            feasible &= (torch.abs(displacement_matrix) <= deadband) | (
                displacement_matrix * direction > deadband
            )
        else:
            feasible &= (candidates - current_y[:, None]) * direction >= margin
        costs = torch.abs(candidates - requested_y[:, None])
        if allow_stationary:
            costs[:, -1] += 1.0e3
        costs = torch.where(feasible, costs, torch.full_like(costs, torch.inf))
        choice = torch.argmin(costs, dim=1)
        any_feasible = feasible.any(dim=1)
        projected = candidates.gather(1, choice[:, None]).squeeze(1)
        projected = torch.where(any_feasible, projected, requested_y)
        return projected, any_feasible

    def _guard_controller_return_targets(self) -> None:
        import torch

        origins = self._env_origins()
        for robot_index, name in enumerate(("left", "right")):
            command = self.commands[name]
            target = getattr(command, "home_target_y", None)
            cfg = getattr(command, "cfg", None)
            outward_target = float(getattr(cfg, "outward_target_y", 0.0))
            if target is None or abs(outward_target) <= 1.0e-6:
                continue
            phase = self._phase_tensor(name)
            pelvis = int(self.body_indices[name]["pelvis"])
            current = self.robots[name].data.body_pos_w[:, pelvis, 1] - origins[:, 1]
            try:
                target_value = torch.as_tensor(target, device=self.device, dtype=torch.float32)
            except (RuntimeError, TypeError, ValueError):
                continue
            if target_value.numel() != self.num_envs:
                continue
            target_tensor = target_value.reshape(-1)
            deadband = max(float(getattr(cfg, "external_target_deadband_m", 0.01)), 0.02)
            active = phase == int(getattr(command, "OUTWARD_HOLD", 3))
            outward_goal = getattr(command, "target_y", None)
            if outward_goal is not None:
                try:
                    outward_value = torch.as_tensor(outward_goal, device=self.device, dtype=torch.float32).reshape(-1)
                except (RuntimeError, TypeError, ValueError):
                    outward_value = None
                if outward_value is not None and outward_value.numel() == self.num_envs:
                    outward_reached = (outward_value - current) * (1.0 if outward_target > 0.0 else -1.0) <= deadband
                    active |= (phase == int(getattr(command, "OUTWARD", 2))) & outward_reached
            if not active.any():
                continue
            corrected, feasible = self._project_controller_motion_target(
                robot_index,
                current,
                target_tensor,
                expected_class=False,
                allow_stationary=True,
            )
            failed = active & ~feasible
            if failed.any():
                ids = torch.nonzero(failed, as_tuple=False).reshape(-1).tolist()
                raise RuntimeError(f"{name} return target has no safe controller reference: env_ids={ids}")
            changed = active & (torch.abs(corrected - target_tensor) > 1.0e-6)
            if not changed.any():
                continue
            target_tensor[changed] = corrected[changed]
            if isinstance(target, torch.Tensor):
                target.copy_(target_tensor.reshape_as(target))
            else:
                try:
                    target[...] = target_tensor.detach().cpu().numpy()
                except (TypeError, ValueError):
                    continue

    def _run_low_level(self, observations: Any, steps: int, *, advance_planner: bool) -> Any:
        import torch

        for _ in range(int(steps)):
            observations, terminated, truncated, info = self._low_level_step(observations)
            done = terminated | truncated
            if done.any():
                self._reset_internal(
                    torch.nonzero(done, as_tuple=False).reshape(-1),
                    new_episode=True,
                    commands_already_reset=info.get("commands_reset", False),
                )
            if advance_planner:
                self._advance_planner_time(~done)
        return observations

    def _step_dt(self) -> float:
        value = getattr(self.unwrapped, "step_dt", None)
        if value is None:
            cfg = getattr(self.unwrapped, "cfg", None)
            sim_cfg = getattr(cfg, "sim", None)
            value = getattr(cfg, "decimation", 1) * getattr(sim_cfg, "dt", 0.02)
        try:
            return float(value)
        except (TypeError, ValueError):
            return 0.02

    def _env_origins(self) -> Any:
        import torch

        origins = getattr(getattr(self.unwrapped, "scene", None), "env_origins", None)
        if origins is None:
            return torch.zeros((self.num_envs, 3), device=self.device)
        result = torch.as_tensor(origins, device=self.device, dtype=torch.float32)
        if result.ndim != 2 or result.shape[0] != self.num_envs or result.shape[1] < 3:
            return torch.zeros((self.num_envs, 3), device=self.device)
        return result[:, :3]

    def _phase_tensor(self, name: str) -> Any:
        return self.commands[name].doubles_state.to(device=self.device, dtype=self.previous_phase.dtype)

    def _ready_tensor(self, name: str) -> Any:
        import torch

        command = self.commands[name]
        phase = self._phase_tensor(name)
        if self.config.locomotion_preemption:
            return phase >= int(command.OUTWARD)
        readiness = getattr(command, "_handoff_ready", None)
        if callable(readiness):
            try:
                return readiness().to(device=self.device, dtype=torch.bool)
            except (RuntimeError, ValueError, AttributeError):
                pass
        return phase == int(command.HOME_HOLD)

    def _commit_ready(self, phases: Any) -> Any:
        import torch

        if self.config.locomotion_preemption:
            return phases >= int(self.commands["left"].OUTWARD)
        ready = torch.stack((self._ready_tensor("left"), self._ready_tensor("right")), dim=1)
        return ready & (phases == int(self.commands["left"].HOME_HOLD))

    def _base_positions(self) -> tuple[Any, Any, Any, Any]:
        origins = self._env_origins()
        positions: list[Any] = []
        velocities: list[Any] = []
        for name in ("left", "right"):
            robot = self.robots[name]
            pelvis = int(self.body_indices[name]["pelvis"])
            positions.append(robot.data.body_pos_w[:, pelvis, :3] - origins)
            velocities.append(robot.data.body_lin_vel_w[:, pelvis, :3])
        return positions[0], positions[1], velocities[0], velocities[1]

    def _point_state(self, name: str, body_key: str) -> tuple[Any, Any]:
        origins = self._env_origins()
        positions: list[Any] = []
        velocities: list[Any] = []
        for robot_name in ("left", "right"):
            robot = self.robots[robot_name]
            body_ids = self.body_indices[robot_name][body_key]
            if not isinstance(body_ids, (tuple, list)):
                body_ids = (int(body_ids),)
            ids = list(body_ids)
            positions.append(robot.data.body_pos_w[:, ids, :3] - origins[:, None, :])
            velocities.append(robot.data.body_lin_vel_w[:, ids, :3])
        return (positions[0], positions[1]), (velocities[0], velocities[1])

    @staticmethod
    def _pair_point_metrics(left_pos: Any, right_pos: Any, left_vel: Any, right_vel: Any, threshold: float) -> tuple[Any, Any, Any]:
        import torch

        displacement = right_pos[:, None, :, :] - left_pos[:, :, None, :]
        relative_velocity = right_vel[:, None, :, :] - left_vel[:, :, None, :]
        distance = torch.linalg.vector_norm(displacement, dim=-1).clamp_min(1.0e-6)
        closing = torch.relu(-torch.sum(displacement * relative_velocity, dim=-1) / distance)
        ttc = torch.where(
            closing > 1.0e-5,
            torch.relu(distance - threshold) / closing.clamp_min(1.0e-5),
            torch.full_like(distance, float("inf")),
        )
        flat_distance = distance.flatten(start_dim=1)
        flat_closing = closing.flatten(start_dim=1)
        flat_ttc = ttc.flatten(start_dim=1)
        nearest = torch.argmin(flat_distance, dim=1)
        return (
            flat_distance.gather(1, nearest[:, None]).squeeze(1),
            flat_closing.gather(1, nearest[:, None]).squeeze(1),
            flat_ttc.gather(1, torch.argmin(flat_ttc, dim=1)[:, None]).squeeze(1),
        )

    def _pair_safety(self) -> dict[str, Any]:
        import torch

        left_pos, right_pos, left_vel, right_vel = self._base_positions()
        displacement = right_pos - left_pos
        base_distance = torch.linalg.vector_norm(displacement, dim=-1).clamp_min(1.0e-6)
        relative_velocity = right_vel - left_vel
        base_closing = torch.relu(-torch.sum(displacement * relative_velocity, dim=-1) / base_distance)
        base_ttc = torch.where(
            base_closing > 1.0e-5,
            torch.relu(base_distance - self.config.reward.base_distance_threshold_m) / base_closing.clamp_min(1.0e-5),
            torch.full_like(base_distance, float("inf")),
        )
        (left_hands, right_hands), (left_hand_vel, right_hand_vel) = self._point_state("hands", "hands")
        (left_racket, right_racket), (left_racket_vel, right_racket_vel) = self._point_state("racket", "racket")
        hand_distance, hand_closing, hand_ttc = self._pair_point_metrics(left_hands, right_hands, left_hand_vel, right_hand_vel, self.config.reward.hand_distance_threshold_m)
        racket_distance, racket_closing, racket_ttc = self._pair_point_metrics(left_racket, right_racket, left_racket_vel, right_racket_vel, self.config.reward.racket_distance_threshold_m)
        return {
            "base_distance": base_distance,
            "base_closing": base_closing,
            "base_ttc": base_ttc,
            "hand_distance": hand_distance,
            "hand_closing": hand_closing,
            "hand_ttc": hand_ttc,
            "racket_distance": racket_distance,
            "racket_closing": racket_closing,
            "racket_ttc": racket_ttc,
        }

    def _advance_planner_time(self, active: Any | None = None) -> None:
        import torch

        dt = self._step_dt()
        if active is None:
            active = torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
        else:
            active = torch.as_tensor(active, device=self.device, dtype=torch.bool).reshape(-1)
            if active.numel() != self.num_envs:
                raise ValueError("planner active mask must match the Isaac batch")
        self.sim_time[active] += dt
        self.stage_elapsed[active] += dt
        self.time_to_strike[active] -= dt
        self.ball_position[active] += self.ball_velocity[active] * dt
        self.ball_velocity[active] += self.ball_acceleration[active] * dt
        self.ball_prediction_age[active] += dt

    def _set_stage(self, ids: Any, stage: int) -> None:
        ids = self._as_ids(ids)
        if ids.numel() == 0:
            return
        self.stage[ids] = int(stage)
        self.stage_elapsed[ids] = 0.0

    def _advance_stage(
        self,
        strike_mask: Any | None = None,
        *,
        active: Any | None = None,
    ) -> None:
        import torch

        active_mask = (
            torch.ones(self.num_envs, dtype=torch.bool, device=self.device)
            if active is None
            else self._done_tensor(active, self.device, self.num_envs)
        )
        phases = torch.stack((self._phase_tensor("left"), self._phase_tensor("right")), dim=1)
        stage = self.stage.clone()
        commit_ready = self._commit_ready(phases).all(dim=1)
        prepared = active_mask & stage.eq(self._RESERVED) & commit_ready & (self.stage_elapsed >= self.config.reservation_duration_s) & (self.time_to_strike <= self.config.lead_time_s) & (self.time_to_strike >= self.config.minimum_commit_tts_s)
        self._set_stage(torch.nonzero(prepared, as_tuple=False).reshape(-1), self._PREPARED)

        crossed = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device) if strike_mask is None else torch.as_tensor(strike_mask, device=self.device, dtype=torch.bool)
        row = torch.arange(self.num_envs, device=self.device)
        valid_hitter = (self.active_hitter >= 0) & (self.active_hitter < 2)
        hitter = self.active_hitter.clamp(0, 1)
        started = active_mask & stage.eq(self._COMMITTED) & valid_hitter & (crossed | (phases[row, hitter] >= 1))
        self._set_stage(torch.nonzero(started, as_tuple=False).reshape(-1), self._STRIKE)

        follow = active_mask & stage.eq(self._STRIKE) & valid_hitter & (phases[row, hitter] >= 2)
        self._set_stage(torch.nonzero(follow, as_tuple=False).reshape(-1), self._FOLLOW_THROUGH)

        handoff = active_mask & stage.eq(self._FOLLOW_THROUGH) & commit_ready
        self._set_stage(torch.nonzero(handoff, as_tuple=False).reshape(-1), self._HANDOFF)

    def _put(self, output: Any, name: str, value: Any) -> None:
        import torch

        index = self._name_to_index.get(name)
        if index is None:
            return
        tensor = torch.as_tensor(value, device=self.device, dtype=output.dtype)
        if tensor.ndim == 0:
            tensor = tensor.expand(self.num_envs)
        tensor = tensor.reshape(-1)
        if tensor.numel() == 1 and self.num_envs != 1:
            tensor = tensor.expand(self.num_envs)
        if tensor.numel() != self.num_envs:
            raise ValueError(f"observation feature {name!r} has {tensor.numel()} values")
        output[:, index] = torch.nan_to_num(tensor, nan=0.0, posinf=1.0, neginf=-1.0)

    def _put_vector(self, output: Any, names: Sequence[str], value: Any, scale: float = 1.0) -> None:
        for index, name in enumerate(names):
            self._put(output, name, value[:, index] / max(float(scale), 1.0e-6))

    def _proprio(self, name: str) -> tuple[Any, Any]:
        import torch

        command = self.commands[name]
        robot = self.robots[name]
        pelvis = int(self.body_indices[name]["pelvis"])
        lin = robot.data.body_lin_vel_w[:, pelvis, :3]
        ang = robot.data.body_ang_vel_w[:, pelvis, :3]
        gravity = getattr(robot.data, "projected_gravity_b", None)
        if gravity is None:
            imu = torch.cat((ang, lin), dim=-1)
        else:
            imu = torch.cat((ang, gravity[:, :3]), dim=-1)
        sensor = getattr(command, "_contact_sensor", None)
        if sensor is None:
            contact = torch.zeros((self.num_envs, self.config.observation.contact_size), device=self.device)
        else:
            try:
                forces = sensor.data.net_forces_w
                force_norm = torch.linalg.vector_norm(forces, dim=-1).reshape(self.num_envs, -1)
                contact = torch.zeros((self.num_envs, self.config.observation.contact_size), device=self.device)
                width = min(contact.shape[1], force_norm.shape[1])
                contact[:, :width] = force_norm[:, :width] / 100.0
            except (RuntimeError, ValueError, AttributeError):
                contact = torch.zeros((self.num_envs, self.config.observation.contact_size), device=self.device)
        return imu, contact

    def _scene_ball_state(self) -> None:
        """Use a scene ball if supplied; otherwise retain the Isaac-side predictor."""

        origins = self._env_origins()
        for key in ("ball", "pingpong_ball"):
            try:
                asset = self.unwrapped.scene[key]
                position = getattr(asset.data, "root_pos_w", None)
                velocity = getattr(asset.data, "root_lin_vel_w", None)
                if position is not None:
                    self.ball_position[:] = position[:, :3] - origins
                if velocity is not None:
                    self.ball_velocity[:] = velocity[:, :3]
                return
            except (KeyError, RuntimeError, AttributeError, TypeError):
                continue

    def _build_observation(self) -> Any:
        import torch

        self._scene_ball_state()
        output = torch.zeros((self.num_envs, self.observation_size), dtype=torch.float32, device=self.device)
        for index, stage_name in enumerate(("reserved", "prepared", "committed", "strike", "follow_through", "handoff")):
            self._put(output, f"stage_{stage_name}", self.stage == index)
        self._put(output, "reservation_age", self.stage_elapsed / self.config.observation.maximum_phase_s)
        self._put(output, "commit_elapsed", self.stage_elapsed / self.config.observation.maximum_phase_s)
        for index, name in enumerate(("left", "right")):
            self._put(output, f"next_hitter_{name}", self.next_hitter == index)
            self._put(output, f"active_hitter_{name}", self.active_hitter == index)
        self._put(output, "physical_side", 0.0)
        self._put_vector(output, ("ball_position_x", "ball_position_y", "ball_position_z"), self.ball_position, 1.0)
        self._put_vector(output, ("ball_velocity_x", "ball_velocity_y", "ball_velocity_z"), self.ball_velocity, 3.0)
        self._put_vector(output, ("ball_acceleration_x", "ball_acceleration_y", "ball_acceleration_z"), self.ball_acceleration, 10.0)
        self._put_vector(output, ("ball_predicted_strike_position_x", "ball_predicted_strike_position_y", "ball_predicted_strike_position_z"), self.ball_target, 1.0)
        prediction_horizon = torch.clamp(self.time_to_strike, min=0.0)
        predicted_velocity = self.ball_velocity + self.ball_acceleration * prediction_horizon[:, None]
        self._put_vector(output, ("ball_predicted_strike_velocity_x", "ball_predicted_strike_velocity_y", "ball_predicted_strike_velocity_z"), torch.clamp(predicted_velocity, -3.0, 3.0), 3.0)
        self._put(output, "ball_time_to_strike", torch.clamp(self.time_to_strike, min=-self.config.maximum_commit_tts_s, max=self.config.maximum_commit_tts_s) / self.config.maximum_commit_tts_s)
        self._put(output, "ball_confidence", 1.0)
        self._put(output, "ball_prediction_age", self.ball_prediction_age / self.config.observation.maximum_prediction_age_s)
        self._put(output, "ball_valid", 1.0)

        origins = self._env_origins()
        for robot_index, name in enumerate(("left", "right")):
            robot = self.robots[name]
            command = self.commands[name]
            pelvis = int(self.body_indices[name]["pelvis"])
            pos = robot.data.body_pos_w[:, pelvis, :3] - origins
            lin = robot.data.body_lin_vel_w[:, pelvis, :3]
            ang = robot.data.body_ang_vel_w[:, pelvis, :3]
            quat = robot.data.body_quat_w[:, pelvis, :4]
            self._put_vector(output, tuple(f"{name}_base_position_{axis}" for axis in "xyz"), pos, 1.0)
            self._put_vector(output, tuple(f"{name}_base_linear_velocity_{axis}" for axis in "xyz"), lin, 3.0)
            self._put_vector(output, tuple(f"{name}_base_angular_velocity_{axis}" for axis in "xyz"), ang, 3.0)
            self._put_vector(output, tuple(f"{name}_base_orientation_{axis}" for axis in "wxyz"), quat, 1.0)
            joint_pos = robot.data.joint_pos
            joint_vel = robot.data.joint_vel
            for joint_index in range(self.config.observation.joint_position_size):
                value = joint_pos[:, joint_index] if joint_index < joint_pos.shape[1] else 0.0
                self._put(output, f"{name}_joint_position_{joint_index}", torch.clamp(value, -3.0, 3.0) / 3.0)
            for joint_index in range(self.config.observation.joint_velocity_size):
                value = joint_vel[:, joint_index] if joint_index < joint_vel.shape[1] else 0.0
                self._put(output, f"{name}_joint_velocity_{joint_index}", torch.clamp(value, -10.0, 10.0) / 3.0)
            imu, contact = self._proprio(name)
            for feature_index in range(self.config.observation.imu_size):
                self._put(output, f"{name}_imu_{feature_index}", imu[:, feature_index] if feature_index < imu.shape[1] else 0.0)
            for feature_index in range(self.config.observation.contact_size):
                self._put(output, f"{name}_contact_{feature_index}", contact[:, feature_index] if feature_index < contact.shape[1] else 0.0)
            phase = self._phase_tensor(name)
            for phase_index, phase_name in enumerate(("hit", "post_delay", "outward", "outward_hold", "return", "home_hold")):
                self._put(output, f"{name}_phase_{phase_name}", phase == phase_index)
            target_y = getattr(command, "target_y", None)
            if target_y is None:
                target_error = torch.zeros(self.num_envs, device=self.device)
                target_valid = torch.zeros(self.num_envs, device=self.device)
            else:
                target_error = target_y.to(device=self.device) - pos[:, 1]
                target_valid = torch.ones(self.num_envs, device=self.device)
            self._put(output, f"{name}_state_elapsed", command.state_elapsed_s / self.config.observation.maximum_phase_s)
            self._put(output, f"{name}_stable_elapsed", command.stable_elapsed_s / self.config.observation.maximum_phase_s)
            self._put(output, f"{name}_target_base_error_y", target_error)
            self._put(output, f"{name}_target_base_valid", target_valid)
            self._put(output, f"{name}_time_to_strike", torch.clamp(command.hit.time_to_strike_s, min=-self.config.maximum_commit_tts_s, max=self.config.maximum_commit_tts_s) / self.config.maximum_commit_tts_s)
            self._put(output, f"{name}_time_to_strike_valid", 1.0)
            ready = self._ready_tensor(name)
            self._put(output, f"{name}_ready", ready)
            self._put(output, f"{name}_command_valid", 1.0)
            self._put(output, f"{name}_feedback_fresh", 1.0)
            for point_name, body_key in (("hand_position", "hands"), ("racket_position", "racket")):
                body_ids = self.body_indices[name][body_key]
                if not isinstance(body_ids, (tuple, list)):
                    body_ids = (int(body_ids),)
                point = robot.data.body_pos_w[:, list(body_ids), :3].mean(dim=1) - origins
                self._put_vector(output, tuple(f"{name}_{point_name}_{axis}" for axis in "xyz"), point, 1.0)
                self._put(output, f"{name}_{point_name.split('_')[0]}_valid", 1.0)
            self._put(output, f"{name}_role_next_hitter", self.next_hitter == robot_index)
            self._put(output, f"{name}_role_active_hitter", self.active_hitter == robot_index)

        left_pos, right_pos, left_vel, right_vel = self._base_positions()
        safety = self._pair_safety()
        self._put_vector(output, ("relative_base_x", "relative_base_y", "relative_base_z"), right_pos - left_pos, 1.0)
        self._put_vector(output, ("relative_velocity_x", "relative_velocity_y", "relative_velocity_z"), right_vel - left_vel, 3.0)
        self._put(output, "base_distance", safety["base_distance"])
        self._put(output, "base_closing_speed", safety["base_closing"] / 3.0)
        self._put(output, "hand_distance", safety["hand_distance"])
        self._put(output, "hand_closing_speed", safety["hand_closing"] / 3.0)
        self._put(output, "racket_distance", safety["racket_distance"])
        self._put(output, "racket_closing_speed", safety["racket_closing"] / 3.0)
        for name in ("base_ttc", "hand_ttc", "racket_ttc"):
            self._put(output, name, torch.nan_to_num(safety[name], nan=2.0, posinf=2.0, neginf=0.0) / self.config.observation.maximum_ttc_s)
            self._put(output, f"{name}_valid", 1.0)
        self._put(output, "base_clearance", safety["base_distance"] - self.config.reward.base_distance_threshold_m)
        self._put(output, "hand_valid", 1.0)
        self._put(output, "racket_valid", 1.0)
        phases = torch.stack((self._phase_tensor("left"), self._phase_tensor("right")), dim=1)
        self._put(output, "simultaneous_hit_risk", (phases <= 1).all(dim=1))
        self._put(output, "emergency_active", 0.0)
        self._put(output, "previous_base_target_left", self.previous_targets[:, 0])
        self._put(output, "previous_base_target_right", self.previous_targets[:, 1])
        for robot_index, name in enumerate(("left", "right")):
            for skill_index, skill in enumerate(SKILL_NAMES):
                self._put(output, f"previous_{skill.lower()}_{name}", self.previous_skills[:, robot_index] == skill_index)
        self._put(output, "decision_age", self.stage_elapsed / self.config.observation.maximum_phase_s)
        mask = self._skill_mask()
        for robot_index, name in enumerate(("left", "right")):
            for skill_index, skill in enumerate(SKILL_NAMES):
                self._put(output, f"{name}_skill_{skill.lower()}_legal", mask[:, robot_index, skill_index])
        if output.shape[1] != len(OBSERVATION_NAMES_V2) and self.config.observation == DoublesObservationConfig():
            raise RuntimeError("centralized observation schema changed without updating the version")
        return torch.nan_to_num(output, nan=0.0, posinf=1.0, neginf=-1.0)

    def _skill_mask(self) -> Any:
        import torch

        mask = torch.zeros((self.num_envs, 2, len(SKILL_NAMES)), dtype=torch.bool, device=self.device)
        mask[:, :, 0] = True
        phases = torch.stack((self._phase_tensor("left"), self._phase_tensor("right")), dim=1)
        commit_ready = self._commit_ready(phases)
        ids = self._all_ids()
        left_pos, right_pos, left_vel, right_vel = self._base_positions()
        positions = (left_pos, right_pos)
        velocities = (left_vel, right_vel)
        motion_feasible = torch.zeros((self.num_envs, 2), dtype=torch.bool, device=self.device)
        runway_feasible = torch.zeros_like(motion_feasible)
        for robot_index, name in enumerate(("left", "right")):
            outward_target = torch.full(
                (self.num_envs,),
                float(self.config.observation.planner.outward_y[robot_index]),
                device=self.device,
            )
            home_target = torch.full(
                (self.num_envs,),
                float(self.config.observation.planner.home_y[robot_index]),
                device=self.device,
            )
            try:
                _target, _return, motion_feasible[:, robot_index] = self._project_controller_motion_pair(
                    robot_index,
                    ids,
                    positions[robot_index][:, 1],
                    outward_target,
                    home_target,
                )
            except (RuntimeError, TypeError, ValueError, AttributeError):
                motion_feasible[:, robot_index] = False
            command = self.commands[name]
            try:
                outward_sign = 1.0 if float(command.cfg.outward_target_y) > 0.0 else -1.0
                post_delay = self._commit_post_delay(command)
            except (TypeError, ValueError, AttributeError):
                outward_sign = 1.0
                post_delay = float("nan")
            transition_horizon = torch.clamp(self.time_to_strike, min=0.0) + post_delay
            try:
                _outward, runway_feasible[:, robot_index] = self._project_physical_outward_target(
                    robot_index,
                    positions[robot_index][:, 1],
                    outward_speed=velocities[robot_index][:, 1] * outward_sign,
                    transition_horizon_s=transition_horizon,
                )
            except (RuntimeError, TypeError, ValueError, AttributeError):
                runway_feasible[:, robot_index] = False
        movable = phases >= 2 if self.config.locomotion_preemption else (phases == 2) | (phases == 5)
        can_prepare = (self.stage[:, None] <= self._PREPARED) & commit_ready
        mask[:, :, 1] = can_prepare & motion_feasible
        mask[:, :, 2] = (self.stage[:, None] >= self._COMMITTED) & movable & motion_feasible & (torch.arange(2, device=self.device)[None, :] != self.active_hitter[:, None])
        mask[:, :, 4] = False
        safety = self._pair_safety()
        minimum_hand = max(self.config.minimum_hand_distance_m, self.config.reward.hand_distance_threshold_m)
        minimum_racket = max(self.config.minimum_racket_distance_m, self.config.reward.racket_distance_threshold_m)
        base_threshold = max(self.config.reward.base_distance_threshold_m, self.config.observation.planner.min_separation)
        safe = (
            (safety["base_distance"] >= base_threshold)
            & (safety["hand_distance"] >= minimum_hand)
            & (safety["racket_distance"] >= minimum_racket)
            & (torch.nan_to_num(safety["base_ttc"], nan=0.0, posinf=10.0) >= self.config.reward.base_ttc_threshold_s)
            & (torch.nan_to_num(safety["hand_ttc"], nan=0.0, posinf=10.0) >= self.config.reward.hand_ttc_threshold_s)
            & (torch.nan_to_num(safety["racket_ttc"], nan=0.0, posinf=10.0) >= self.config.reward.racket_ttc_threshold_s)
        )
        hit_legal = (
            self.stage == self._PREPARED
            ) & (self.time_to_strike >= self.config.minimum_commit_tts_s) & (self.time_to_strike <= self.config.maximum_commit_tts_s) & commit_ready.all(dim=1) & safe
        rows = torch.arange(self.num_envs, device=self.device)
        hitter_runway = runway_feasible[rows, self.next_hitter]
        peer_motion = motion_feasible[rows, 1 - self.next_hitter]
        mask[rows, self.next_hitter, 3] = hit_legal & hitter_runway & peer_motion
        return mask

    def _project_targets(self, targets: Any, protected: Any) -> tuple[Any, Any]:
        import torch

        low, high = self.config.observation.planner.workspace_y
        minimum = max(self.config.observation.planner.min_separation, self.config.reward.base_distance_threshold_m)
        result = torch.clamp(targets, float(low), float(high))
        projected = (result[:, 1] - result[:, 0]) < minimum
        left = result[:, 0]
        right = result[:, 1]
        left_protected, right_protected = protected[:, 0], protected[:, 1]
        only_left = projected & left_protected & ~right_protected
        only_right = projected & right_protected & ~left_protected
        right = torch.where(only_left, torch.clamp(left + minimum, max=float(high)), right)
        left = torch.where(only_right, torch.clamp(right - minimum, min=float(low)), left)
        movable_pair = projected & ~(left_protected | right_protected)
        centered_left = torch.clamp(0.5 * (left + right - minimum), min=float(low), max=float(high - minimum))
        left = torch.where(movable_pair, centered_left, left)
        right = torch.where(movable_pair, centered_left + minimum, right)
        return torch.stack((left, right), dim=1), projected

    def _snapshot_command_state(self, command: Any, ids: Any) -> dict[str, dict[str, Any]]:
        import torch

        snapshots: dict[str, dict[str, Any]] = {}
        if ids.numel() == 0:
            return snapshots
        for scope, owner in (("command", command), ("hit", getattr(command, "hit", None))):
            if owner is None:
                continue
            fields: dict[str, Any] = {}
            for name, value in vars(owner).items():
                if (
                    isinstance(value, torch.Tensor)
                    and value.ndim > 0
                    and value.shape[0] == self.num_envs
                ):
                    owner_ids = ids.to(device=value.device)
                    fields[name] = value[owner_ids].clone()
            if fields:
                snapshots[scope] = fields
        return snapshots

    @staticmethod
    def _restore_command_state(
        command: Any,
        ids: Any,
        snapshots: Mapping[str, Mapping[str, Any]],
    ) -> None:
        for scope, fields in snapshots.items():
            owner = command if scope == "command" else getattr(command, scope)
            for name, value in fields.items():
                target = getattr(owner, name)
                owner_ids = ids.to(device=target.device)
                target[owner_ids] = value

    def _set_controller_base_target(
        self,
        command: Any,
        ids: Any,
        target_xy: Any,
        return_target_y: Any,
    ) -> None:
        states = command.doubles_state[ids]
        same_phase = (states == int(command.OUTWARD)) | (states == int(command.RETURN))
        protected_ids = ids[same_phase]
        snapshots = self._snapshot_command_state(command, protected_ids)
        try:
            command.set_external_base_target(
                env_ids=ids,
                target_xy=target_xy,
                return_target_y=return_target_y,
                safety_override=False,
            )
        except (RuntimeError, TypeError, ValueError, AttributeError):
            self._restore_command_state(command, protected_ids, snapshots)
            raise

    def _project_controller_motion_pair(
        self,
        robot_index: int,
        ids: Any,
        current_y: Any,
        target: Any,
        return_target: Any,
    ) -> tuple[Any, Any, Any]:
        import torch

        name = ("left", "right")[robot_index]
        command = self.commands[name]
        phase = self._phase_tensor(name)[ids]
        scale_mask = phase == int(getattr(command, "OUTWARD", 2))
        home_y = torch.as_tensor(command.home_y, device=self.device, dtype=torch.float32)[ids]
        outbound_index = torch.as_tensor(
            command.outbound_motion_index,
            device=self.device,
            dtype=torch.long,
        )[ids]
        projected_target, target_feasible = self._project_controller_motion_target(
            robot_index,
            current_y,
            target,
            expected_class=True,
            allow_stationary=False,
        )
        existing_scale = self._controller_motion_scale(
            command,
            outbound_index,
            projected_target - home_y,
        )
        labels = torch.as_tensor(command.motion_labels_t, device=self.device, dtype=torch.bool)
        target_feasible &= ~scale_mask | (
            labels[outbound_index]
            & torch.isfinite(existing_scale)
            & (existing_scale >= -1.0e-6)
        )
        projected_return, return_feasible = self._project_controller_motion_target(
            robot_index,
            projected_target,
            return_target,
            expected_class=False,
            allow_stationary=True,
        )
        return projected_target, projected_return, target_feasible & return_feasible

    def _command_base_target(
        self,
        robot_index: int,
        ids: Any,
        target: Any,
        return_target: Any | None = None,
    ) -> tuple[Any, Any]:
        import torch

        name = ("left", "right")[robot_index]
        command = self.commands[name]
        robot = self.robots[name]
        origins = self._env_origins()
        pelvis = int(self.body_indices[name]["pelvis"])
        current = robot.data.body_pos_w[ids, pelvis, :3] - origins[ids]
        return_y = (
            torch.full_like(target, float(self.config.observation.planner.home_y[robot_index]))
            if return_target is None
            else return_target
        )
        projected_target, projected_return, feasible = self._project_controller_motion_pair(
            robot_index,
            ids,
            current[:, 1],
            target,
            return_y,
        )
        if not feasible.all():
            failed = ids[~feasible].detach().cpu().tolist()
            raise ValueError(f"{name} base target has no controller-feasible motion pair: env_ids={failed}")
        self._set_controller_base_target(
            command,
            ids,
            torch.stack((current[:, 0], projected_target), dim=1),
            projected_return,
        )
        return projected_target, projected_return

    def _commit_one(self, env_id: int, hitter_index: int) -> bool:
        import torch

        ids = torch.tensor([env_id], dtype=torch.long, device=self.device)
        hitter_name = ("left", "right")[hitter_index]
        peer_index = 1 - hitter_index
        peer_name = ("left", "right")[peer_index]
        hitter_command = self.commands[hitter_name]
        peer_command = self.commands[peer_name]
        origins = self._env_origins()
        hitter_robot = self.robots[hitter_name]
        hitter_pelvis = int(self.body_indices[hitter_name]["pelvis"])
        peer_target = torch.full((1,), float(self.config.observation.planner.outward_y[peer_index]), device=self.device)
        home_target = torch.full((1,), float(self.config.observation.planner.home_y[peer_index]), device=self.device)
        command_snapshots: list[tuple[Any, Mapping[str, Mapping[str, Any]]]] = []
        try:
            hitter_pos = hitter_robot.data.body_pos_w[ids, hitter_pelvis, :3] - origins[ids]
            hitter_velocity_y = hitter_robot.data.body_lin_vel_w[ids, hitter_pelvis, 1]
            hitter_sign = 1.0 if float(hitter_command.cfg.outward_target_y) > 0.0 else -1.0
            post_delay = self._commit_post_delay(hitter_command)
            transition_horizon = torch.clamp(self.time_to_strike[ids], min=0.0) + post_delay
            hitter_outward_target = self._physical_outward_target(
                hitter_index,
                hitter_pos[:, 1],
                outward_speed=hitter_velocity_y * hitter_sign,
                transition_horizon_s=transition_horizon,
            )
            command_snapshots = [
                (hitter_command, self._snapshot_command_state(hitter_command, ids)),
                (peer_command, self._snapshot_command_state(peer_command, ids)),
            ]
            self._command_base_target(peer_index, ids, peer_target, home_target)
            hitter_command.set_external_hit(
                env_ids=ids,
                racket_target=self.ball_target[ids],
                target_velocity=torch.tensor(self.config.target_racket_velocity_mps, device=self.device).reshape(1, 3),
                time_to_strike_s=self.time_to_strike[ids],
                ball_velocity=self.ball_velocity[ids],
            )
            hitter_command.set_external_outward_target(
                env_ids=ids,
                target_y=hitter_outward_target,
            )
            peer_phase = int(self._phase_tensor(peer_name)[env_id].item())
            if peer_phase not in (2, 3, 4):
                raise RuntimeError("peer clear hook did not enter a move phase")
            hitter_phase = int(self._phase_tensor(hitter_name)[env_id].item())
            if hitter_phase != 0:
                raise RuntimeError("hitter hook did not enter HIT")
        except (RuntimeError, TypeError, ValueError, AttributeError):
            try:
                for command, snapshots in reversed(command_snapshots):
                    self._restore_command_state(command, ids, snapshots)
            except (RuntimeError, TypeError, ValueError, AttributeError) as restore_error:
                raise RuntimeError(
                    f"failed to roll back rejected HIT commit for env_id={env_id}"
                ) from restore_error
            return False
        self.active_hitter[env_id] = hitter_index
        self._set_stage(ids, self._COMMITTED)
        return True

    def _apply_high_level_action(self, actions: Any) -> dict[str, Any]:
        import torch

        raw = torch.as_tensor(actions, device=self.device, dtype=torch.float32)
        if raw.shape != (self.num_envs, self.num_actions):
            raise ValueError(f"high-level action shape must be {(self.num_envs, self.num_actions)}")
        finite = torch.isfinite(raw).all(dim=1)
        raw = torch.nan_to_num(raw, nan=0.0, posinf=0.0, neginf=0.0)
        low, high = self.config.observation.planner.workspace_y
        midpoint = 0.5 * (low + high)
        half = 0.5 * (high - low)
        targets = torch.stack((midpoint + half * raw[:, 0].clamp(-1.0, 1.0), midpoint + half * raw[:, 6].clamp(-1.0, 1.0)), dim=1)
        skills = torch.stack((torch.argmax(raw[:, 1:6], dim=1), torch.argmax(raw[:, 7:12], dim=1)), dim=1)
        home = torch.tensor(self.config.observation.planner.home_y, device=self.device, dtype=targets.dtype)
        outward = torch.tensor(self.config.observation.planner.outward_y, device=self.device, dtype=targets.dtype)
        prepare_direction = torch.sign(outward - home)
        prepare_distance = torch.abs(outward - home)
        prepare_progress = torch.clamp((targets - home) * prepare_direction, min=0.0)
        prepare_targets = home + torch.minimum(prepare_progress, prepare_distance) * prepare_direction
        targets = torch.where(skills == 1, prepare_targets, targets)
        phases = torch.stack((self._phase_tensor("left"), self._phase_tensor("right")), dim=1)
        protected = phases <= 1
        targets, projected = self._project_targets(targets, protected | (skills == 1))
        mask = self._skill_mask()
        candidates = skills == 3
        simultaneous = candidates.all(dim=1)
        candidate_ids = torch.arange(self.num_envs, device=self.device)
        candidate_index = torch.where(candidates[:, 0], torch.zeros_like(self.next_hitter), torch.ones_like(self.next_hitter))
        accepted = candidates.any(dim=1) & ~simultaneous & (candidate_index == self.next_hitter)
        accepted &= mask[candidate_ids, self.next_hitter, 3]
        accepted &= finite
        commit_started = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        peer_clear = torch.zeros_like(commit_started)
        ready_at_commit = torch.zeros_like(commit_started)
        left_pos, right_pos, _left_vel, _right_vel = self._base_positions()
        applied_targets = torch.stack((left_pos[:, 1], right_pos[:, 1]), dim=1).detach().clone()
        applied_skills = torch.zeros_like(skills)
        for env_id in torch.nonzero(accepted, as_tuple=False).reshape(-1).tolist():
            hitter = int(self.next_hitter[env_id].item())
            ready_at_commit[env_id] = True
            if self._commit_one(env_id, hitter):
                commit_started[env_id] = True
                peer_clear[env_id] = True
                peer = 1 - hitter
                applied_skills[env_id, hitter] = 3
                applied_skills[env_id, peer] = 2
                applied_targets[env_id, peer] = self.commands[("left", "right")[peer]].target_y[env_id]
            else:
                accepted[env_id] = False
        masked_commit = candidates.any(dim=1) & ~commit_started

        for robot_index, name in enumerate(("left", "right")):
            command = self.commands[name]
            ids_mask = (skills[:, robot_index] != 0) & ~commit_started
            row_ids = torch.arange(self.num_envs, device=self.device)
            ids_mask &= mask[row_ids, robot_index, skills[:, robot_index].clamp(0, len(SKILL_NAMES) - 1)]
            ids_mask &= self.active_hitter != robot_index
            ids = torch.nonzero(ids_mask, as_tuple=False).reshape(-1)
            if ids.numel() == 0:
                continue
            skill = skills[ids, robot_index]
            target = targets[ids, robot_index]
            target = torch.where(skill == 2, torch.full_like(target, float(self.config.observation.planner.outward_y[robot_index])), target)
            target = torch.where(skill == 4, torch.full_like(target, float(self.config.observation.planner.home_y[robot_index])), target)
            for env_id in ids.tolist():
                row = torch.tensor([env_id], dtype=torch.long, device=self.device)
                try:
                    row_mask = ids == env_id
                    row_target = target[row_mask]
                    row_return = torch.full_like(
                        row_target,
                        float(self.config.observation.planner.home_y[robot_index]),
                    )
                    applied_target, _applied_return = self._command_base_target(
                        robot_index,
                        row,
                        row_target,
                        row_return,
                    )
                    applied_targets[env_id, robot_index] = applied_target[0]
                    applied_skills[env_id, robot_index] = skill[row_mask][0]
                except (RuntimeError, ValueError, AttributeError):
                    pass
        self.previous_targets = applied_targets
        self.previous_skills = applied_skills
        return {
            "commit_started": commit_started,
            "peer_clear_at_commit": peer_clear,
            "ready_at_commit": ready_at_commit & commit_started,
            "simultaneous_hit": torch.zeros_like(simultaneous),
            "simultaneous_hit_request": simultaneous,
            "safety_projected": projected,
            "masked_commit": masked_commit,
            "stale_input": ~finite,
        }

    def _low_level_rollout(self, observations: Any, events: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
        import torch

        position_error = torch.zeros(self.num_envs, device=self.device)
        velocity_error = torch.zeros_like(position_error)
        orientation_error = torch.zeros_like(position_error)
        timing_error = torch.zeros_like(position_error)
        strike_mask = torch.zeros(self.num_envs, dtype=torch.bool, device=self.device)
        phase_interruption = torch.zeros_like(strike_mask)
        terminated = torch.zeros_like(strike_mask)
        truncated = torch.zeros_like(strike_mask)
        inactive = torch.zeros_like(strike_mask)
        commands_reset = torch.zeros_like(strike_mask)
        safety = self._pair_safety()
        minimum_safety = {name: value.clone() for name, value in safety.items()}
        simultaneous_phase = torch.zeros_like(strike_mask)
        for _ in range(self.config.low_level_steps):
            active_before = ~inactive
            before_tts = torch.stack((self.commands["left"].hit.time_to_strike_s, self.commands["right"].hit.time_to_strike_s), dim=1).clone()
            before_phase = torch.stack((self._phase_tensor("left"), self._phase_tensor("right")), dim=1).clone()
            observations, terminated_now, truncated_now, low_level_info = self._low_level_step(
                observations,
                inactive=inactive,
            )
            commands_reset_now = self._done_tensor(
                low_level_info.get("commands_reset", False),
                self.device,
                self.num_envs,
            ) & active_before
            commands_reset |= commands_reset_now
            done_now = terminated_now | truncated_now
            newly_done = active_before & done_now
            terminated |= active_before & terminated_now
            truncated |= active_before & truncated_now
            after_tts = torch.stack((self.commands["left"].hit.time_to_strike_s, self.commands["right"].hit.time_to_strike_s), dim=1)
            after_phase = torch.stack((self._phase_tensor("left"), self._phase_tensor("right")), dim=1)
            protected_before = before_phase <= 1
            protected_after = after_phase <= 1
            simultaneous_phase |= active_before & (protected_before | protected_after).all(dim=1)
            active = self.active_hitter.clamp_min(0)
            crossed = ((before_tts > 0.0) & (after_tts <= 0.0)) | ((before_phase == 0) & (after_phase == 1))
            strike_now = (
                active_before
                & crossed.gather(1, active[:, None]).squeeze(1)
                & (self.active_hitter >= 0)
            )
            strike_mask |= strike_now
            phase_interruption |= (
                active_before
                & ((after_phase < before_phase) & (before_phase > 1)).any(dim=1)
                & ~terminated_now
            )
            if strike_now.any():
                for robot_index, name in enumerate(("left", "right")):
                    ids = torch.nonzero(strike_now & (self.active_hitter == robot_index), as_tuple=False).reshape(-1)
                    if ids.numel() == 0:
                        continue
                    try:
                        pos, ori, vel = self.distill_hit_errors(self.commands[name])
                        position_error[ids] = pos[ids]
                        orientation_error[ids] = ori[ids]
                        velocity_error[ids] = vel[ids]
                    except (RuntimeError, ValueError, AttributeError):
                        position_error[ids] = 1.0
                        orientation_error[ids] = 1.0
                        velocity_error[ids] = 1.0
                    timing_error[ids] = torch.abs(after_tts[ids, robot_index])
            continuing = active_before & ~done_now
            self._advance_planner_time(continuing)
            self._advance_stage(strike_now, active=continuing)
            safety = self._pair_safety()
            for name in ("base_distance", "base_ttc", "hand_distance", "hand_ttc", "racket_distance", "racket_ttc"):
                updated = torch.minimum(minimum_safety[name], safety[name])
                minimum_safety[name] = torch.where(continuing, updated, minimum_safety[name])
            for name in ("base_closing", "hand_closing", "racket_closing"):
                updated = torch.maximum(minimum_safety[name], safety[name])
                minimum_safety[name] = torch.where(continuing, updated, minimum_safety[name])
            reset_here = newly_done & ~commands_reset_now
            if reset_here.any():
                self._reset_commands_to_home(
                    torch.nonzero(reset_here, as_tuple=False).reshape(-1)
                )
            commands_reset |= reset_here
            inactive |= done_now
        events.update(
            {
                "strike_mask": strike_mask,
                "position_error_m": position_error,
                "velocity_error_mps": velocity_error,
                "orientation_error_rad": orientation_error,
                "timing_error_s": timing_error,
                "phase_interruption": phase_interruption,
                "base_distance_m": minimum_safety["base_distance"],
                "hand_distance_m": minimum_safety["hand_distance"],
                "racket_distance_m": minimum_safety["racket_distance"],
                "base_ttc_s": minimum_safety["base_ttc"],
                "hand_ttc_s": minimum_safety["hand_ttc"],
                "racket_ttc_s": minimum_safety["racket_ttc"],
                "terminated": terminated,
                "truncated": truncated,
                "commands_reset": commands_reset,
            }
        )
        events["simultaneous_hit"] |= simultaneous_phase
        return observations, events

    def _reward_kwargs(self, events: Mapping[str, Any]) -> dict[str, Any]:
        names = (
            "strike_mask",
            "position_error_m",
            "velocity_error_mps",
            "orientation_error_rad",
            "timing_error_s",
            "base_distance_m",
            "hand_distance_m",
            "racket_distance_m",
            "base_ttc_s",
            "hand_ttc_s",
            "racket_ttc_s",
            "simultaneous_hit",
            "simultaneous_hit_request",
            "phase_interruption",
            "safety_projected",
            "masked_commit",
            "stale_input",
            "commit_started",
            "peer_clear_at_commit",
            "ready_at_commit",
            "handoff_completed",
            "timeout",
            "terminated",
        )
        return {name: events[name] for name in names}

    def step(self, actions: Any) -> tuple[Any, Any, Any, dict[str, Any]]:
        import torch

        if self._last_observations is None:
            self.reset()
        self._advance_stage()
        events = self._apply_high_level_action(actions)
        observations, events = self._low_level_rollout(self._last_observations, events)
        self._last_observations = observations
        rollout_done = events["terminated"] | events["truncated"]
        self._advance_stage(events["strike_mask"], active=~rollout_done)
        handoff_completed = self.stage == self._HANDOFF
        waiting_for_commit = (self.stage == self._RESERVED) | (self.stage == self._PREPARED)
        timeout = (waiting_for_commit & (self.time_to_strike < self.config.minimum_commit_tts_s)) | events["truncated"]
        events["handoff_completed"] = handoff_completed
        events["timeout"] = timeout
        reward, terms = compute_isaac_relay_reward(config=self.config.reward, **self._reward_kwargs(events))
        self.episode_length_buf += 1
        self.shots_completed += handoff_completed.to(dtype=torch.long)
        self.episode_return += reward.detach()
        done = events["terminated"] | timeout | (self.episode_length_buf >= self.config.max_episode_steps) | (self.shots_completed >= self.config.shots_per_episode)
        timeouts = timeout | (self.episode_length_buf >= self.config.max_episode_steps)
        completed_length = self.episode_length_buf.detach().clone()
        completed_return = self.episode_return.detach().clone()
        completed_stage = self.stage.detach().clone()
        done_ids = torch.nonzero(done, as_tuple=False).reshape(-1)
        handoff_ids = torch.nonzero(handoff_completed & ~done, as_tuple=False).reshape(-1)
        physics_reset_ids = torch.nonzero(done & ~events["terminated"] & ~events["truncated"], as_tuple=False).reshape(-1)
        if physics_reset_ids.numel():
            try:
                observations, _ = self.unwrapped.reset(env_ids=physics_reset_ids)
            except (TypeError, RuntimeError, AttributeError):
                if physics_reset_ids.numel() == self.num_envs:
                    observations, _ = self.env.reset()
        if done_ids.numel():
            self._reset_internal(
                done_ids,
                new_episode=True,
                commands_already_reset=events["commands_reset"],
            )
        if handoff_ids.numel():
            self._start_next_shot(handoff_ids)
        self._last_observations = observations
        info = {
            "time_outs": timeouts,
            "reward_terms": terms,
            "episode_reward": torch.where(done, completed_return, torch.full_like(reward, float("nan"))),
            "episode_length": torch.where(done, completed_length, torch.zeros_like(completed_length)),
            "commit_started": events["commit_started"],
            "peer_clear_at_commit": events["peer_clear_at_commit"],
            "handoff_completed": events["handoff_completed"],
            "simultaneous_hit": events["simultaneous_hit"],
            "simultaneous_hit_request": events["simultaneous_hit_request"],
            "safety_projected": events["safety_projected"],
            "masked_commit": events["masked_commit"],
            "timeout": events["timeout"],
            "terminated": events["terminated"],
            "base_distance_m": events["base_distance_m"],
            "hand_distance_m": events["hand_distance_m"],
            "racket_distance_m": events["racket_distance_m"],
            "stage": completed_stage,
        }
        return self._build_observation(), reward, done, info

    def close(self) -> None:
        self.env.close()


@dataclass(frozen=True)
class IsaacPPORunnerConfig:
    seed: int = 10000
    num_steps_per_env: int = 32
    max_iterations: int = 5000
    save_interval: int = 100
    actor_hidden_dims: tuple[int, ...] = (256, 256)
    critic_hidden_dims: tuple[int, ...] = (256, 256)
    learning_rate: float = 3.0e-4
    entropy_coef: float = 0.005

    def to_dict(self) -> dict[str, Any]:
        return {
            "seed": self.seed,
            "num_steps_per_env": self.num_steps_per_env,
            "max_iterations": self.max_iterations,
            "save_interval": self.save_interval,
            "experiment_name": "pingpang_doubles_high_level",
            "run_name": "centralized_dual_command",
            "logger": "tensorboard",
            "obs_groups": {"actor": ["policy"], "critic": ["policy"]},
            "actor": {
                "class_name": "MLPModel",
                "hidden_dims": list(self.actor_hidden_dims),
                "activation": "elu",
                "obs_normalization": True,
                "distribution_cfg": {"class_name": "GaussianDistribution", "init_std": 0.5, "std_type": "log"},
            },
            "critic": {
                "class_name": "MLPModel",
                "hidden_dims": list(self.critic_hidden_dims),
                "activation": "elu",
                "obs_normalization": True,
                "distribution_cfg": None,
            },
            "algorithm": {
                "class_name": "PPO",
                "value_loss_coef": 1.0,
                "use_clipped_value_loss": True,
                "clip_param": 0.2,
                "entropy_coef": self.entropy_coef,
                "num_learning_epochs": 5,
                "num_mini_batches": 4,
                "learning_rate": self.learning_rate,
                "schedule": "adaptive",
                "gamma": 0.99,
                "lam": 0.95,
                "desired_kl": 0.01,
                "max_grad_norm": 1.0,
                "normalize_advantage_per_mini_batch": False,
                "share_cnn_encoders": False,
                "rnd_cfg": None,
                "symmetry_cfg": None,
            },
            "check_for_nan": True,
            "clip_actions": 10.0,
        }


def make_rsl_rl_vec_env(environment: IsaacDoublesHighLevelEnv, device: str = "cuda:0") -> Any:
    """Adapt the 12-D outer loop to rsl-rl's VecEnv protocol."""

    import torch
    from rsl_rl.env import VecEnv
    from tensordict import TensorDict

    class IsaacRslRlVecEnv(VecEnv):
        def __init__(self) -> None:
            self.env = environment
            self.num_envs = environment.num_envs
            self.num_actions = environment.num_actions
            self.max_episode_length = environment.max_episode_length
            self.device = torch.device(device)
            self.episode_length_buf = environment.episode_length_buf
            self.cfg = {"name": "isaac_doubles_high_level", "action_version": ACTION_VERSION}
            if getattr(environment, "_last_observations", None) is None and callable(getattr(environment, "reset", None)):
                environment.reset()

        def get_observations(self) -> Any:
            observation = self.env._build_observation().to(self.device)
            return TensorDict({"policy": observation}, batch_size=[self.num_envs], device=self.device)

        def step(self, actions: Any) -> tuple[Any, Any, Any, dict[str, Any]]:
            actions = torch.as_tensor(actions, device=self.device, dtype=torch.float32)
            observations, rewards, dones, info = self.env.step(actions)
            self.episode_length_buf = self.env.episode_length_buf
            logs = {
                f"/reward/{name}": value.mean()
                for name, value in info.get("reward_terms", {}).items()
                if isinstance(value, torch.Tensor)
            }
            logs["/relay/commit_rate"] = info["commit_started"].float().mean()
            logs["/relay/simultaneous_hit_rate"] = info["simultaneous_hit"].float().mean()
            request = info.get("simultaneous_hit_request", info["simultaneous_hit"])
            logs["/relay/simultaneous_hit_request_rate"] = request.float().mean()
            logs["/relay/masked_commit_rate"] = info["masked_commit"].float().mean()
            logs["/relay/safety_projection_rate"] = info["safety_projected"].float().mean()
            return (
                TensorDict({"policy": observations.to(self.device)}, batch_size=[self.num_envs], device=self.device),
                rewards.to(self.device),
                dones.to(self.device, dtype=torch.bool),
                {"time_outs": info["time_outs"].to(self.device, dtype=torch.bool), "log": logs},
            )

        def close(self) -> None:
            self.env.close()

    return IsaacRslRlVecEnv()


def _first_existing(paths: Sequence[Path], label: str) -> Path:
    for path in paths:
        candidate = path.expanduser().resolve()
        if candidate.is_dir():
            return candidate
    raise FileNotFoundError(f"No {label} directory found; checked: {', '.join(str(path) for path in paths)}")


def _require_requested_controller_root(requested: Path, selected: Path) -> Path:
    requested_root = requested.expanduser().resolve()
    selected_root = selected.expanduser().resolve()
    if selected_root != requested_root:
        raise RuntimeError(
            f"requested controller root {requested_root} failed ABI selection; "
            f"refusing fallback controller {selected_root}"
        )
    return selected_root


def _load_student_policy(
    path: Path,
    device: Any,
    StudentPolicy: Any,
    validate_student_checkpoint_metadata: Callable[..., Any],
) -> tuple[Any, int]:
    import torch

    checkpoint_path = path.expanduser().resolve()
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"frozen student checkpoint not found: {checkpoint_path}")
    try:
        checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=True)
    except Exception as error:
        raise RuntimeError(
            f"failed to safely load frozen student checkpoint: {checkpoint_path}"
        ) from error
    if not isinstance(checkpoint, dict):
        raise RuntimeError(
            f"frozen student checkpoint payload must be a dict, got {type(checkpoint).__name__}"
        )
    validate_student_checkpoint_metadata(checkpoint, expected_obs_dim=1666)
    if "student_state_dict" not in checkpoint:
        raise RuntimeError("frozen student checkpoint is missing student_state_dict")
    policy = StudentPolicy(1666).to(device).eval()
    policy.load_state_dict(checkpoint["student_state_dict"], strict=True)
    policy.requires_grad_(False)
    return policy, int(checkpoint.get("iteration", 0))


def create_isaac_doubles_high_level_env(config: IsaacDoublesPPOConfig) -> IsaacDoublesHighLevelEnv:
    """Create the two-robot Isaac scene and centralized PPO wrapper.

    Isaac AppLauncher must already be active.  The controller source path is
    inserted before importing ``whole_body_tracking`` so the selected ABI is
    deterministic in long-running training processes.
    """

    import gymnasium as gym
    from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
    from doubles_planner.isaac_bridge import mirror_command_sources
    from doubles_planner.left_asset import build_left_handed_urdf, build_right_handed_urdf
    from doubles_planner.mirror import mirror_policy_action, mirror_student_observation
    from run_isaac_doubles_relay import (
        _configure_scene,
        _controller_root,
        _ensure_unitree_asset,
    )

    root = _require_requested_controller_root(
        config.controller_root,
        _controller_root(config.controller_root),
    )
    _ensure_unitree_asset(root)
    source_root = root / "source" / "whole_body_tracking"
    if str(source_root) not in sys.path:
        sys.path.insert(0, str(source_root))
    import whole_body_tracking
    import whole_body_tracking.tasks  # noqa: F401

    package_file = getattr(whole_body_tracking, "__file__", None)
    if package_file is None:
        raise RuntimeError("whole_body_tracking package has no source path")
    try:
        Path(package_file).resolve().relative_to(source_root.resolve())
    except ValueError as error:
        raise RuntimeError(
            f"whole_body_tracking was imported from {package_file}, not {source_root}"
        ) from error
    from whole_body_tracking.tasks.tracking.mdp.rewards import _distill_hit_errors
    from whole_body_tracking.utils.multi_expert_distillation import StudentPolicy, validate_student_checkpoint_metadata

    move_root = _first_existing(
        tuple(path for path in (config.move_motion_data, root / "data" / "0718-move-160-80hz", root / "0718-move-160", root.parent / "pingpang_controller_v6_i23000" / "data" / "0718-move-160-80hz") if path is not None),
        "move motion data",
    )
    hit_root = _first_existing(
        tuple(path for path in (config.hit_motion_data, root / "0302_combined", root.parent / "pingpang_controller" / "0302_combined") if path is not None),
        "hit motion data",
    )
    env_cfg = parse_env_cfg(config.task, device=config.device, num_envs=config.num_envs)
    env_cfg.seed = config.seed
    env_cfg._planner_move_root = str(move_root)
    env_cfg._planner_hit_root = str(hit_root)
    canonical = Path(env_cfg.scene.robot.spawn.asset_path).resolve()
    right_urdf = build_right_handed_urdf(canonical, canonical.with_name("high_level_right.generated.urdf"))
    left_urdf = build_left_handed_urdf(canonical, canonical.with_name("high_level_left.generated.urdf"))
    env_cfg.scene.robot.spawn.asset_path = str(right_urdf)
    env_cfg = _configure_scene(
        env_cfg,
        root,
        left_urdf,
        2.0,
        "rear",
        num_envs=config.num_envs,
        task=config.task,
    )
    env_cfg._planner_move_root = None
    env_cfg._planner_hit_root = None
    env_kwargs: dict[str, Any] = {"cfg": env_cfg}
    if config.render_mode is not None:
        env_kwargs["render_mode"] = config.render_mode
    env = gym.make(config.task, **env_kwargs)
    env.reset()
    unwrapped = env.unwrapped
    commands = {"right": unwrapped.command_manager.get_term("motion"), "left": unwrapped.command_manager.get_term("motion_left")}
    robots = {"right": unwrapped.scene["robot"], "left": unwrapped.scene["robot_left"]}
    joint_names = tuple(robots["left"].joint_names)
    body_names = tuple(commands["left"].cfg.body_names)
    mirror_command_sources(commands["left"], joint_names, body_names)
    action_terms = {"right": unwrapped.action_manager.get_term("joint_pos"), "left": unwrapped.action_manager.get_term("joint_pos_left")}
    body_indices: dict[str, dict[str, Any]] = {}
    for name, robot in robots.items():
        hands, hand_names = robot.find_bodies(["left_wrist_yaw_link", "right_wrist_yaw_link"], preserve_order=True)
        racket, racket_names = robot.find_bodies("racket", preserve_order=True)
        pelvis, pelvis_names = robot.find_bodies("pelvis", preserve_order=True)
        if hand_names != ["left_wrist_yaw_link", "right_wrist_yaw_link"] or racket_names != ["racket"] or pelvis_names != ["pelvis"]:
            raise RuntimeError(f"Unexpected body names for {name}: hands={hand_names}, racket={racket_names}, pelvis={pelvis_names}")
        body_indices[name] = {"hands": tuple(hands), "racket": int(racket[0]), "pelvis": int(pelvis[0])}
    policy, _ = _load_student_policy(
        config.checkpoint,
        unwrapped.device,
        StudentPolicy,
        validate_student_checkpoint_metadata,
    )
    return IsaacDoublesHighLevelEnv(
        env,
        policy,
        config=config,
        commands=commands,
        robots=robots,
        action_terms=action_terms,
        joint_names=joint_names,
        body_indices=body_indices,
        mirror_observation=mirror_student_observation,
        mirror_action=mirror_policy_action,
        distill_hit_errors=_distill_hit_errors,
        auto_reset=True,
    )


__all__ = [
    "IsaacDoublesHighLevelEnv",
    "IsaacDoublesPPOConfig",
    "IsaacPPORunnerConfig",
    "IsaacRelayRewardConfig",
    "compute_isaac_relay_reward",
    "create_isaac_doubles_high_level_env",
    "make_rsl_rl_vec_env",
]
