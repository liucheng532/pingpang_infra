from __future__ import annotations

from collections import OrderedDict

import torch


FRAME_TERMS = OrderedDict(
    (
        ("strike", slice(0, 1)),
        ("target_vel", slice(1, 4)),
        ("racket", slice(4, 7)),
        ("target_base", slice(7, 9)),
        ("task_anchor", slice(9, 12)),
        ("orientation", slice(12, 18)),
        ("base_ang_vel", slice(18, 21)),
        ("joint_pos", slice(21, 50)),
        ("joint_vel", slice(50, 79)),
        ("actions", slice(79, 108)),
        ("command", slice(108, 166)),
    )
)
FRAME_DIM = 166
PHASE_DIM = 6
CURRENT_OBS_DIM = FRAME_DIM + PHASE_DIM
TEACHER_TERM_DIMS = OrderedDict(
    (
        ("strike", 1),
        ("target_vel", 3),
        ("racket", 3),
        ("motion_anchor", 3),
        ("task_anchor", 3),
        ("orientation", 6),
        ("command", 58),
        ("base_ang_vel", 3),
        ("joint_pos", 29),
        ("joint_vel", 29),
        ("actions", 29),
    )
)
TEACHER_FRAME_DIM = sum(TEACHER_TERM_DIMS.values())
TEACHER_HISTORY_DIM = TEACHER_FRAME_DIM * 10
TEACHER_TASK_TERMS = frozenset({"strike", "target_vel", "racket", "motion_anchor", "command"})


class HistoryWrapper:
    """Build the term-major 1660-frame-history + six-state current-phase ABI."""

    def __init__(self, env, enable_teacher_history=False, teacher_history_mode="distill-reset"):
        self.env = env
        self.obs_history_length = int(self.env.num_history_length)
        self.num_obs = int(self.env.num_obs)
        if self.num_obs != CURRENT_OBS_DIM:
            raise ValueError(f"LCMAgent current observation must be {CURRENT_OBS_DIM}, got {self.num_obs}.")
        self.num_obs_history = FRAME_DIM * self.obs_history_length + PHASE_DIM
        self._buffers: dict[str, torch.Tensor] = {}
        self._teacher_buffers: dict[str, torch.Tensor] = {}
        self._teacher_hit_active = False
        self.enable_teacher_history = bool(enable_teacher_history)
        if teacher_history_mode not in ("distill-reset", "legacy-continuous"):
            raise ValueError("Unknown HIT Teacher history mode.")
        self.teacher_history_mode = teacher_history_mode
        self.obs_history = torch.zeros(
            self.env.num_envs,
            self.num_obs_history,
            dtype=torch.float32,
            device=self.env.device,
            requires_grad=False,
        )

    def _validate_obs(self, obs: torch.Tensor) -> torch.Tensor:
        if not torch.is_tensor(obs):
            obs = torch.as_tensor(obs, dtype=torch.float32, device=self.env.device)
        if obs.ndim != 2 or obs.shape != (self.env.num_envs, CURRENT_OBS_DIM):
            raise ValueError(
                f"Current observation must have shape ({self.env.num_envs}, {CURRENT_OBS_DIM}), got {tuple(obs.shape)}."
            )
        return obs

    def _flatten(self, obs: torch.Tensor) -> torch.Tensor:
        histories = [self._buffers[name].reshape(self.env.num_envs, -1) for name in FRAME_TERMS]
        self.obs_history = torch.cat((*histories, obs[:, FRAME_DIM:CURRENT_OBS_DIM]), dim=-1)
        if self.obs_history.shape[-1] != self.num_obs_history:
            raise RuntimeError(f"Student history ABI is {self.obs_history.shape[-1]}, expected {self.num_obs_history}.")
        return self.obs_history

    def _teacher_terms(self, obs: torch.Tensor) -> dict[str, torch.Tensor]:
        frame = self.env.get_teacher_frame(obs)
        if not torch.is_tensor(frame):
            frame = torch.as_tensor(frame, dtype=torch.float32, device=self.env.device)
        if frame.shape != (self.env.num_envs, TEACHER_FRAME_DIM):
            raise RuntimeError(
                f"Teacher frame must have shape ({self.env.num_envs}, {TEACHER_FRAME_DIM}), "
                f"got {tuple(frame.shape)}."
            )
        values = torch.split(frame, tuple(TEACHER_TERM_DIMS.values()), dim=-1)
        return dict(zip(TEACHER_TERM_DIMS, values))

    def _flatten_teacher(self) -> torch.Tensor:
        history = torch.cat(
            [self._teacher_buffers[name].reshape(self.env.num_envs, -1) for name in TEACHER_TERM_DIMS],
            dim=-1,
        )
        if history.shape != (self.env.num_envs, TEACHER_HISTORY_DIM):
            raise RuntimeError(f"Teacher history has invalid shape {tuple(history.shape)}.")
        return history

    def _append_teacher(self, obs: torch.Tensor) -> torch.Tensor:
        terms = self._teacher_terms(obs)
        hit_active = int(self.env.reference.state) <= 1
        task_reset = (
            self.teacher_history_mode == "distill-reset"
            and hit_active
            and not self._teacher_hit_active
        )
        for name, value in terms.items():
            if self.teacher_history_mode == "legacy-continuous":
                self._teacher_buffers[name] = torch.cat(
                    (self._teacher_buffers[name][:, 1:], value.unsqueeze(1)), dim=1
                )
            elif task_reset and name in TEACHER_TASK_TERMS:
                self._teacher_buffers[name] = value.unsqueeze(1).repeat(1, self.obs_history_length, 1)
            elif name not in TEACHER_TASK_TERMS or hit_active:
                self._teacher_buffers[name] = torch.cat(
                    (self._teacher_buffers[name][:, 1:], value.unsqueeze(1)), dim=1
                )
        self._teacher_hit_active = hit_active
        return self._flatten_teacher()

    def _result(self, obs: torch.Tensor, append: bool) -> dict:
        reference = getattr(self.env, "reference", None)
        phase_id = (
            int(reference.state)
            if reference is not None
            else int(torch.argmax(obs[0, FRAME_DIM:CURRENT_OBS_DIM]).item())
        )
        result = {
            "obs": obs,
            "obs_history": self._append(obs) if append else self._flatten(obs),
            "phase_id": phase_id,
        }
        if self.enable_teacher_history:
            result["teacher_obs_history"] = (
                self._append_teacher(obs) if append else self._flatten_teacher()
            )
        return result

    def _append(self, obs: torch.Tensor) -> torch.Tensor:
        obs = self._validate_obs(obs)
        if not self._buffers:
            raise RuntimeError("HistoryWrapper.reset() must be called before step()/get_obs().")
        for name, term_slice in FRAME_TERMS.items():
            value = obs[:, term_slice]
            self._buffers[name] = torch.cat((self._buffers[name][:, 1:], value.unsqueeze(1)), dim=1)
        return self._flatten(obs)

    def apply_action(self, action, hard_reset=False):
        return self.env.apply_action(action, hard_reset=hard_reset)

    def observe(self):
        obs = self._validate_obs(self.env.observe())
        return self._result(obs, append=True)

    def step(self, action):
        obs = self._validate_obs(self.env.step(action))
        return self._result(obs, append=True)

    def get_obs(self):
        obs = self._validate_obs(self.env.get_obs())
        return self._result(obs, append=True)

    def reset(self):
        obs = self._validate_obs(self.env.reset())
        self._buffers = {}
        for name, term_slice in FRAME_TERMS.items():
            value = obs[:, term_slice]
            if name == "actions":
                value = torch.zeros_like(value)
            self._buffers[name] = value.unsqueeze(1).repeat(1, self.obs_history_length, 1)
        if self.enable_teacher_history:
            teacher_terms = self._teacher_terms(obs)
            self._teacher_buffers = {
                name: value.unsqueeze(1).repeat(1, self.obs_history_length, 1)
                for name, value in teacher_terms.items()
            }
            self._teacher_hit_active = int(self.env.reference.state) <= 1
        return self._result(obs, append=False)

    def __getattr__(self, name):
        return getattr(self.env, name)
