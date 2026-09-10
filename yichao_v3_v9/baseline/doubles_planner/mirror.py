"""Left/right reflection adapter for the frozen 1666 -> 29 G1 policy.

The policy was trained for a right-handed player.  A physical left-handed
player is adapted by reflecting its complete observation into the policy's
right-handed frame and reflecting the policy output back through absolute
joint-position targets.

All public transforms operate on the final axis and preserve arbitrary leading
dimensions.  They intentionally use only array/tensor duck-typed operations,
so NumPy arrays and torch tensors are both supported without importing torch.
"""

from __future__ import annotations

from functools import lru_cache
from typing import Any, NamedTuple, Sequence

import numpy as np


G1_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

G1_ACTION_DIM = 29
STUDENT_OBSERVATION_DIM = 1666
STUDENT_HISTORY_LENGTH = 10


class JointMirrorSpec(NamedTuple):
    """Permutation and signs for an output vector in ``joint_names`` order."""

    permutation: tuple[int, ...]
    signs: tuple[int, ...]


def _reflected_joint_name(name: str) -> str:
    if name.startswith("left_"):
        return "right_" + name[len("left_") :]
    if name.startswith("right_"):
        return "left_" + name[len("right_") :]
    return name


def _joint_axis_sign(name: str) -> int:
    return -1 if name.endswith(("_roll_joint", "_yaw_joint")) else 1


@lru_cache(maxsize=None)
def _cached_joint_mirror_spec(joint_names: tuple[str, ...]) -> JointMirrorSpec:
    if len(joint_names) != G1_ACTION_DIM or set(joint_names) != set(G1_JOINT_NAMES):
        expected = set(G1_JOINT_NAMES)
        actual = set(joint_names)
        missing = sorted(expected - actual)
        unexpected = sorted(actual - expected)
        duplicates = sorted({name for name in joint_names if joint_names.count(name) > 1})
        details = []
        if missing:
            details.append(f"missing={missing}")
        if unexpected:
            details.append(f"unexpected={unexpected}")
        if duplicates:
            details.append(f"duplicates={duplicates}")
        if len(joint_names) != G1_ACTION_DIM:
            details.append(f"count={len(joint_names)}")
        suffix = "; ".join(details) or "invalid joint-name set"
        raise ValueError(f"joint_names must contain each of the exact 29 G1 joints once; {suffix}")

    indices = {name: index for index, name in enumerate(joint_names)}
    permutation = tuple(indices[_reflected_joint_name(name)] for name in joint_names)
    signs = tuple(_joint_axis_sign(name) for name in joint_names)
    return JointMirrorSpec(permutation, signs)


def joint_mirror_spec(joint_names: Sequence[str]) -> JointMirrorSpec:
    """Validate the exact G1 joint-name set and build its mirror mapping.

    The caller's order is retained.  For output joint ``i``, ``permutation[i]``
    identifies the reflected source joint and ``signs[i]`` is its axis sign.
    """

    try:
        names = tuple(joint_names)
    except TypeError as error:
        raise ValueError("joint_names must be an iterable of 29 strings") from error
    if any(not isinstance(name, str) for name in names):
        raise ValueError("joint_names must contain only strings")
    return _cached_joint_mirror_spec(names)


def _as_array_or_tensor(values: Any) -> Any:
    if hasattr(values, "shape") and hasattr(values, "__getitem__"):
        return values
    return np.asarray(values)


def _copy_array_or_tensor(values: Any) -> Any:
    clone = getattr(values, "clone", None)
    if callable(clone):
        return clone()
    copy = getattr(values, "copy", None)
    if callable(copy):
        return copy()
    return np.array(values, copy=True)


def _require_last_dimension(values: Any, size: int, name: str) -> None:
    shape = tuple(values.shape)
    if not shape or shape[-1] != size:
        actual = shape[-1] if shape else None
        raise ValueError(f"{name} must have final dimension {size}, got {actual}")


def _mirror_components(values: Any, size: int, negative: tuple[int, ...], name: str) -> Any:
    array = _as_array_or_tensor(values)
    _require_last_dimension(array, size, name)
    mirrored = _copy_array_or_tensor(array)
    mirrored[..., list(negative)] = -mirrored[..., list(negative)]
    return mirrored


def mirror_polar_vectors(values: Any) -> Any:
    """Reflect positions and linear velocities across XZ: ``[x, -y, z]``."""

    return _mirror_components(values, 3, (1,), "polar vectors")


def mirror_axial_vectors(values: Any) -> Any:
    """Reflect angular velocities across XZ: ``[-x, y, -z]``."""

    return _mirror_components(values, 3, (0, 2), "axial vectors")


def mirror_quaternions_wxyz(values: Any) -> Any:
    """Apply ``S R S`` to WXYZ quaternions: ``[w, -x, y, -z]``."""

    return _mirror_components(values, 4, (1, 3), "WXYZ quaternions")


def mirror_orientation_6d(values: Any) -> Any:
    """Reflect Isaac Lab's flattened first-two-rotation-column encoding.

    ``matrix_from_quat(q)[..., :2].reshape(..., 6)`` has reflection signs
    ``[+1, -1, -1, +1, +1, -1]``.
    """

    return _mirror_components(values, 6, (1, 2, 5), "6D orientations")


def mirror_joint_values(values: Any, joint_names: Sequence[str]) -> Any:
    """Swap left/right G1 joints and apply roll/yaw axis signs."""

    spec = joint_mirror_spec(joint_names)
    array = _as_array_or_tensor(values)
    _require_last_dimension(array, G1_ACTION_DIM, "joint values")
    mirrored = _copy_array_or_tensor(array[..., list(spec.permutation)])
    negative = [index for index, sign in enumerate(spec.signs) if sign < 0]
    mirrored[..., negative] = -mirrored[..., negative]
    return mirrored


_OBSERVATION_TERM_SLICES = {
    "strike_time": slice(0, 10),
    "target_velocity": slice(10, 40),
    "racket_target": slice(40, 70),
    "target_base_xy": slice(70, 90),
    "task_anchor": slice(90, 120),
    "robot_orientation": slice(120, 180),
    "base_angular_velocity": slice(180, 210),
    "joint_position": slice(210, 500),
    "joint_velocity": slice(500, 790),
    "previous_action": slice(790, 1080),
    "generated_command": slice(1080, 1660),
    "current_phase": slice(1660, 1666),
}


def _reshape_history(term: Any, width: int) -> Any:
    return term.reshape((*term.shape[:-1], STUDENT_HISTORY_LENGTH, width))


def _assign_history_term(
    observation: Any,
    term_name: str,
    width: int,
    transform: Any,
) -> None:
    term_slice = _OBSERVATION_TERM_SLICES[term_name]
    flattened = observation[..., term_slice]
    history = _reshape_history(flattened, width)
    observation[..., term_slice] = transform(history).reshape(flattened.shape)


def mirror_student_observation(
    observation: Any,
    joint_names: Sequence[str],
    source_action_offset: Any | None = None,
    source_action_scale: Any | None = None,
    target_action_offset: Any | None = None,
    target_action_scale: Any | None = None,
) -> Any:
    """Mirror the exact term-major 1666-D student observation ABI.

    Each historical term contains ten frames ordered oldest-to-newest.  Strike
    time and current phase are invariant; every other term follows the geometry
    and joint conventions documented in ``TEAMMATE_HANDOFF_20260812.md``.
    """

    joint_mirror_spec(joint_names)
    source = _as_array_or_tensor(observation)
    _require_last_dimension(source, STUDENT_OBSERVATION_DIM, "student observation")
    mirrored = _copy_array_or_tensor(source)

    _assign_history_term(mirrored, "target_velocity", 3, mirror_polar_vectors)
    _assign_history_term(mirrored, "racket_target", 3, mirror_polar_vectors)
    _assign_history_term(
        mirrored,
        "target_base_xy",
        2,
        lambda values: _mirror_components(values, 2, (1,), "target base XY"),
    )
    _assign_history_term(mirrored, "task_anchor", 3, mirror_polar_vectors)
    _assign_history_term(mirrored, "robot_orientation", 6, mirror_orientation_6d)
    _assign_history_term(mirrored, "base_angular_velocity", 3, mirror_axial_vectors)
    _assign_history_term(
        mirrored,
        "joint_position",
        G1_ACTION_DIM,
        lambda values: mirror_joint_values(values, joint_names),
    )
    _assign_history_term(
        mirrored,
        "joint_velocity",
        G1_ACTION_DIM,
        lambda values: mirror_joint_values(values, joint_names),
    )
    action_parameters = (
        source_action_offset,
        source_action_scale,
        target_action_offset,
        target_action_scale,
    )
    if all(parameter is None for parameter in action_parameters):
        action_transform = lambda values: mirror_joint_values(values, joint_names)
    elif source_action_offset is None or source_action_scale is None:
        raise ValueError("source action offset and scale must be provided together")
    else:
        action_transform = lambda values: mirror_policy_action(
            values,
            joint_names,
            source_action_offset,
            source_action_scale,
            target_action_offset,
            target_action_scale,
        )
    _assign_history_term(
        mirrored,
        "previous_action",
        G1_ACTION_DIM,
        action_transform,
    )

    command_slice = _OBSERVATION_TERM_SLICES["generated_command"]
    flattened_command = mirrored[..., command_slice]
    command = _reshape_history(flattened_command, 2 * G1_ACTION_DIM)
    mirrored_command = _copy_array_or_tensor(command)
    mirrored_command[..., :G1_ACTION_DIM] = mirror_joint_values(
        command[..., :G1_ACTION_DIM], joint_names
    )
    mirrored_command[..., G1_ACTION_DIM:] = mirror_joint_values(
        command[..., G1_ACTION_DIM:], joint_names
    )
    mirrored[..., command_slice] = mirrored_command.reshape(flattened_command.shape)
    return mirrored


def _coerce_parameter_like(values: Any, reference: Any, name: str) -> Any:
    parameter = _as_array_or_tensor(values)
    _require_last_dimension(parameter, G1_ACTION_DIM, name)
    if hasattr(reference, "new_tensor") and not hasattr(parameter, "device"):
        parameter = reference.new_tensor(parameter)
    elif isinstance(reference, np.ndarray) and not isinstance(parameter, np.ndarray):
        parameter = np.asarray(parameter, dtype=reference.dtype)
    return parameter


def _contains_zero(values: Any) -> bool:
    result = (values == 0).any()
    item = getattr(result, "item", None)
    return bool(item() if callable(item) else result)


def mirror_policy_action(
    action: Any,
    joint_names: Sequence[str],
    source_action_offset: Any,
    source_action_scale: Any,
    target_action_offset: Any | None = None,
    target_action_scale: Any | None = None,
) -> Any:
    """Reflect a normalized action between distinct action parameterizations.

    Mirroring normalized coordinates directly is incorrect when joint defaults
    are asymmetric.  This function denormalizes with the source offsets/scales,
    mirrors the absolute position targets, then normalizes with the target
    offsets/scales.  Omitting the target parameters preserves the legacy case
    where both action terms use the same parameterization.
    """

    normalized = _as_array_or_tensor(action)
    _require_last_dimension(normalized, G1_ACTION_DIM, "policy action")
    source_offset = _coerce_parameter_like(source_action_offset, normalized, "source action offset")
    source_scale = _coerce_parameter_like(source_action_scale, normalized, "source action scale")
    if target_action_offset is None:
        target_action_offset = source_action_offset
    if target_action_scale is None:
        target_action_scale = source_action_scale
    target_offset = _coerce_parameter_like(target_action_offset, normalized, "target action offset")
    target_scale = _coerce_parameter_like(target_action_scale, normalized, "target action scale")
    if _contains_zero(source_scale) or _contains_zero(target_scale):
        raise ValueError("action scales must not contain zero")
    absolute_target = normalized * source_scale + source_offset
    mirrored_target = mirror_joint_values(absolute_target, joint_names)
    return (mirrored_target - target_offset) / target_scale


class LeftHandStudentWrapper:
    """Callable adapter that makes a right-handed policy drive a left player."""

    def __init__(
        self,
        policy: Any,
        joint_names: Sequence[str],
        right_action_offset: Any,
        right_action_scale: Any,
        left_action_offset: Any | None = None,
        left_action_scale: Any | None = None,
    ) -> None:
        joint_mirror_spec(joint_names)
        self.policy = policy
        self.joint_names = tuple(joint_names)
        self.right_action_offset = right_action_offset
        self.right_action_scale = right_action_scale
        self.left_action_offset = right_action_offset if left_action_offset is None else left_action_offset
        self.left_action_scale = right_action_scale if left_action_scale is None else left_action_scale

    def __call__(self, observation: Any, *args: Any, **kwargs: Any) -> Any:
        right_observation = mirror_student_observation(
            observation,
            self.joint_names,
            self.left_action_offset,
            self.left_action_scale,
            self.right_action_offset,
            self.right_action_scale,
        )
        right_action = self.policy(right_observation, *args, **kwargs)
        return mirror_policy_action(
            right_action,
            self.joint_names,
            self.right_action_offset,
            self.right_action_scale,
            self.left_action_offset,
            self.left_action_scale,
        )


__all__ = [
    "G1_ACTION_DIM",
    "G1_JOINT_NAMES",
    "JointMirrorSpec",
    "LeftHandStudentWrapper",
    "STUDENT_HISTORY_LENGTH",
    "STUDENT_OBSERVATION_DIM",
    "joint_mirror_spec",
    "mirror_axial_vectors",
    "mirror_joint_values",
    "mirror_orientation_6d",
    "mirror_polar_vectors",
    "mirror_policy_action",
    "mirror_quaternions_wxyz",
    "mirror_student_observation",
]
