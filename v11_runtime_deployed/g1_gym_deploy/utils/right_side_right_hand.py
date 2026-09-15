"""Canonical table-side adapter for a physical right-hand robot on table_right."""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from utils.left_right_mirror import (
    mirror_joint_values,
    mirror_policy_action,
    mirror_student_observation,
)


STUDENT_OBSERVATION_DIM = 1666
STUDENT_HISTORY_LENGTH = 10
G1_ACTION_DIM = 29
ARM_TOKENS = ("shoulder", "elbow", "wrist")

_JOINT_HISTORY_SLICES = (
    slice(210, 500),
    slice(500, 790),
    slice(790, 1080),
)
_COMMAND_SLICE = slice(1080, 1660)


def _copy(values: Any) -> Any:
    clone = getattr(values, "clone", None)
    if callable(clone):
        return clone()
    copy = getattr(values, "copy", None)
    if callable(copy):
        return copy()
    return np.array(values, copy=True)


def _to_cpu_numpy(values: Any) -> np.ndarray:
    detach = getattr(values, "detach", None)
    if callable(detach):
        return detach().cpu().numpy()
    return np.asarray(values)


def arm_joint_indices(joint_names: Sequence[str]) -> tuple[int, ...]:
    return tuple(
        index
        for index, name in enumerate(joint_names)
        if any(token in name for token in ARM_TOKENS)
    )


def reflect_right_side_body(values: Any, joint_names: Sequence[str]) -> Any:
    """Reflect legs/waist while preserving both physical arms exactly."""
    source = values
    reflected = mirror_joint_values(source, joint_names)
    arms = arm_joint_indices(joint_names)
    reflected[..., list(arms)] = source[..., list(arms)]
    return reflected


def reflect_right_side_action(
    action: Any,
    joint_names: Sequence[str],
    action_offset: Any,
    action_scale: Any,
) -> Any:
    """Reflect a normalized locomotion action without changing either arm."""
    reflected = mirror_policy_action(
        action,
        joint_names,
        action_offset,
        action_scale,
        action_offset,
        action_scale,
    )
    arms = arm_joint_indices(joint_names)
    reflected[..., list(arms)] = action[..., list(arms)]
    return reflected


def canonicalize_right_side_student_observation(
    observation: Any,
    joint_names: Sequence[str],
    action_offset: Any,
    action_scale: Any,
) -> Any:
    """Canonicalize table-right geometry and lower body, preserving arm histories."""
    source = _to_cpu_numpy(observation)
    if source.shape[-1] != STUDENT_OBSERVATION_DIM:
        raise ValueError(
            f"student observation must end in {STUDENT_OBSERVATION_DIM}, got {source.shape}"
        )
    canonical = mirror_student_observation(
        source,
        joint_names,
        action_offset,
        action_scale,
        action_offset,
        action_scale,
    )
    arms = list(arm_joint_indices(joint_names))
    for term_slice in _JOINT_HISTORY_SLICES:
        source_term = source[..., term_slice].reshape(
            (*source.shape[:-1], STUDENT_HISTORY_LENGTH, G1_ACTION_DIM)
        )
        canonical_term = canonical[..., term_slice].reshape(source_term.shape)
        canonical_term[..., arms] = source_term[..., arms]

    source_command = source[..., _COMMAND_SLICE].reshape(
        (*source.shape[:-1], STUDENT_HISTORY_LENGTH, 2, G1_ACTION_DIM)
    )
    canonical_command = canonical[..., _COMMAND_SLICE].reshape(source_command.shape)
    canonical_command[..., arms] = source_command[..., arms]
    return canonical


class RightSideRightHandStudentPolicy:
    """Run the canonical V10 MOVE policy on table_right without mirroring arms."""

    def __init__(self, policy: Any, joint_names: Sequence[str], action_offset: Any, action_scale: Any):
        self.policy = policy
        self.joint_names = tuple(joint_names)
        self.action_offset = np.asarray(action_offset, dtype=np.float32)
        self.action_scale = np.asarray(action_scale, dtype=np.float32)
        if self.action_offset.shape != (G1_ACTION_DIM,) or self.action_scale.shape != (
            G1_ACTION_DIM,
        ):
            raise ValueError("right-side action offset/scale must have shape (29,)")
        if np.any(self.action_scale == 0.0):
            raise ValueError("right-side action scale must not contain zero")
        self.last_policy_observation = None
        self.last_canonical_action = None
        self.last_physical_action = None

    def canonical_observation(self, physical_observation: Any) -> Any:
        return canonicalize_right_side_student_observation(
            physical_observation,
            self.joint_names,
            self.action_offset,
            self.action_scale,
        )

    def __call__(self, physical_observation: Any) -> Any:
        canonical_observation = self.canonical_observation(physical_observation)
        canonical_action = self.policy(canonical_observation)
        physical_action = reflect_right_side_action(
            canonical_action,
            self.joint_names,
            self.action_offset,
            self.action_scale,
        )
        self.last_policy_observation = _copy(canonical_observation)
        self.last_canonical_action = _copy(canonical_action)
        self.last_physical_action = _copy(physical_action)
        return physical_action

    def __getattr__(self, name: str) -> Any:
        return getattr(self.policy, name)
