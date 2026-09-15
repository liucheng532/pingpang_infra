"""Reflection adapters for running the canonical V9 policy on a left-hand G1."""

from __future__ import annotations

from functools import lru_cache
from typing import Any, NamedTuple, Sequence

import numpy as np


G1_ACTION_DIM = 29
STUDENT_OBSERVATION_DIM = 1666
TEACHER_OBSERVATION_DIM = 1670
STUDENT_HISTORY_LENGTH = 10
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


class JointMirrorSpec(NamedTuple):
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
        raise ValueError("joint_names must contain each of the exact 29 G1 joints once")
    indices = {name: index for index, name in enumerate(joint_names)}
    reflected = tuple(_reflected_joint_name(name) for name in joint_names)
    missing = sorted(set(reflected) - set(indices))
    if missing:
        raise ValueError(f"joint_names is not closed under left/right reflection: {missing}")
    return JointMirrorSpec(
        tuple(indices[name] for name in reflected),
        tuple(_joint_axis_sign(name) for name in joint_names),
    )


def joint_mirror_spec(joint_names: Sequence[str]) -> JointMirrorSpec:
    names = tuple(joint_names)
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


def _to_cpu_numpy(values: Any) -> np.ndarray:
    detach = getattr(values, "detach", None)
    if callable(detach):
        return detach().cpu().numpy()
    return np.asarray(values)


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
    return _mirror_components(values, 3, (1,), "polar vectors")


def mirror_planar_vectors(values: Any) -> Any:
    return _mirror_components(values, 2, (1,), "planar vectors")


def mirror_axial_vectors(values: Any) -> Any:
    return _mirror_components(values, 3, (0, 2), "axial vectors")


def mirror_rpy(values: Any) -> Any:
    return _mirror_components(values, 3, (0, 2), "roll-pitch-yaw")


def mirror_orientation_6d(values: Any) -> Any:
    return _mirror_components(values, 6, (1, 2, 5), "6D orientations")


def mirror_joint_values(values: Any, joint_names: Sequence[str]) -> Any:
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

_TEACHER_TERM_SLICES = {
    "strike_time": slice(0, 10),
    "target_velocity": slice(10, 40),
    "racket_target": slice(40, 70),
    "motion_anchor": slice(70, 100),
    "task_anchor": slice(100, 130),
    "robot_orientation": slice(130, 190),
    "generated_command": slice(190, 770),
    "base_angular_velocity": slice(770, 800),
    "joint_position": slice(800, 1090),
    "joint_velocity": slice(1090, 1380),
    "previous_action": slice(1380, 1670),
}


def _reshape_history(term: Any, width: int) -> Any:
    return term.reshape((*term.shape[:-1], STUDENT_HISTORY_LENGTH, width))


def _assign_history_term(observation: Any, term_name: str, width: int, transform: Any) -> None:
    term_slice = _OBSERVATION_TERM_SLICES[term_name]
    flattened = observation[..., term_slice]
    history = _reshape_history(flattened, width)
    observation[..., term_slice] = transform(history).reshape(flattened.shape)


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


def mirror_student_observation(
    observation: Any,
    joint_names: Sequence[str],
    source_action_offset: Any,
    source_action_scale: Any,
    target_action_offset: Any | None = None,
    target_action_scale: Any | None = None,
) -> Any:
    joint_mirror_spec(joint_names)
    source = _as_array_or_tensor(observation)
    _require_last_dimension(source, STUDENT_OBSERVATION_DIM, "student observation")
    mirrored = _copy_array_or_tensor(source)

    _assign_history_term(mirrored, "target_velocity", 3, mirror_polar_vectors)
    _assign_history_term(mirrored, "racket_target", 3, mirror_polar_vectors)
    _assign_history_term(mirrored, "target_base_xy", 2, mirror_planar_vectors)
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
    _assign_history_term(
        mirrored,
        "previous_action",
        G1_ACTION_DIM,
        lambda values: mirror_policy_action(
            values,
            joint_names,
            source_action_offset,
            source_action_scale,
            target_action_offset,
            target_action_scale,
        ),
    )

    command_slice = _OBSERVATION_TERM_SLICES["generated_command"]
    flattened = mirrored[..., command_slice]
    command = _reshape_history(flattened, 2 * G1_ACTION_DIM)
    mirrored_command = _copy_array_or_tensor(command)
    mirrored_command[..., :G1_ACTION_DIM] = mirror_joint_values(
        command[..., :G1_ACTION_DIM], joint_names
    )
    mirrored_command[..., G1_ACTION_DIM:] = mirror_joint_values(
        command[..., G1_ACTION_DIM:], joint_names
    )
    mirrored[..., command_slice] = mirrored_command.reshape(flattened.shape)
    return mirrored


def mirror_teacher_observation(
    observation: Any,
    joint_names: Sequence[str],
    source_action_offset: Any,
    source_action_scale: Any,
    target_action_offset: Any | None = None,
    target_action_scale: Any | None = None,
) -> Any:
    """Reflect the frozen HIT Teacher's term-major 1670-D history."""
    joint_mirror_spec(joint_names)
    source = _as_array_or_tensor(observation)
    _require_last_dimension(source, TEACHER_OBSERVATION_DIM, "teacher observation")
    mirrored = _copy_array_or_tensor(source)

    def assign(name: str, width: int, transform: Any) -> None:
        term_slice = _TEACHER_TERM_SLICES[name]
        flattened = mirrored[..., term_slice]
        history = _reshape_history(flattened, width)
        mirrored[..., term_slice] = transform(history).reshape(flattened.shape)

    assign("target_velocity", 3, mirror_polar_vectors)
    assign("racket_target", 3, mirror_polar_vectors)
    assign("motion_anchor", 3, mirror_polar_vectors)
    assign("task_anchor", 3, mirror_polar_vectors)
    assign("robot_orientation", 6, mirror_orientation_6d)
    assign("base_angular_velocity", 3, mirror_axial_vectors)
    assign("joint_position", G1_ACTION_DIM, lambda values: mirror_joint_values(values, joint_names))
    assign("joint_velocity", G1_ACTION_DIM, lambda values: mirror_joint_values(values, joint_names))
    assign(
        "previous_action",
        G1_ACTION_DIM,
        lambda values: mirror_policy_action(
            values,
            joint_names,
            source_action_offset,
            source_action_scale,
            target_action_offset,
            target_action_scale,
        ),
    )

    command_slice = _TEACHER_TERM_SLICES["generated_command"]
    flattened = mirrored[..., command_slice]
    command = _reshape_history(flattened, 2 * G1_ACTION_DIM)
    mirrored_command = _copy_array_or_tensor(command)
    mirrored_command[..., :G1_ACTION_DIM] = mirror_joint_values(
        command[..., :G1_ACTION_DIM], joint_names
    )
    mirrored_command[..., G1_ACTION_DIM:] = mirror_joint_values(
        command[..., G1_ACTION_DIM:], joint_names
    )
    mirrored[..., command_slice] = mirrored_command.reshape(flattened.shape)
    return mirrored


class MirroredStudentPolicy:
    """Make a canonical right-hand policy act on a physical left-hand robot."""

    def __init__(
        self,
        policy: Any,
        joint_names: Sequence[str],
        canonical_action_offset: Any,
        canonical_action_scale: Any,
        physical_action_offset: Any,
        physical_action_scale: Any,
    ) -> None:
        joint_mirror_spec(joint_names)
        if _contains_zero(np.asarray(canonical_action_scale)) or _contains_zero(
            np.asarray(physical_action_scale)
        ):
            raise ValueError("action scales must not contain zero")
        self.policy = policy
        self.joint_names = tuple(joint_names)
        self.canonical_action_offset = canonical_action_offset
        self.canonical_action_scale = canonical_action_scale
        self.physical_action_offset = physical_action_offset
        self.physical_action_scale = physical_action_scale
        self.last_policy_observation = None
        self.last_canonical_action = None
        self.last_physical_action = None

    def __call__(self, physical_observation: Any) -> Any:
        # The deployed ONNX session is CPU-only.  Mirroring term-by-term on a
        # CUDA history tensor launches many tiny kernels and then synchronizes
        # for ONNX, which reduced the real control loop from 50 Hz to ~35 Hz.
        # Copy once and keep the entire mirror/ONNX/action path on CPU.
        physical_observation_cpu = _to_cpu_numpy(physical_observation)
        canonical_observation = mirror_student_observation(
            physical_observation_cpu,
            self.joint_names,
            self.physical_action_offset,
            self.physical_action_scale,
            self.canonical_action_offset,
            self.canonical_action_scale,
        )
        canonical_action = self.policy(canonical_observation)
        physical_action = mirror_policy_action(
            canonical_action,
            self.joint_names,
            self.canonical_action_offset,
            self.canonical_action_scale,
            self.physical_action_offset,
            self.physical_action_scale,
        )
        self.last_policy_observation = _copy_array_or_tensor(canonical_observation)
        self.last_canonical_action = _copy_array_or_tensor(canonical_action)
        self.last_physical_action = _copy_array_or_tensor(physical_action)
        diagnostics = getattr(self.policy, "last_diagnostics", {})
        if diagnostics and diagnostics.get("mode") == "active":
            canonical_teacher = np.asarray(
                diagnostics.get("teacher_action"), dtype=np.float32
            ).reshape(1, G1_ACTION_DIM)
            physical_teacher = _to_cpu_numpy(self.mirror_action(canonical_teacher))
            physical_output = _to_cpu_numpy(physical_action)
            delta = np.asarray(physical_output - physical_teacher, dtype=np.float32)
            left_arm_names = (
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint",
                "left_shoulder_yaw_joint",
                "left_elbow_joint",
                "left_wrist_roll_joint",
                "left_wrist_pitch_joint",
                "left_wrist_yaw_joint",
            )
            left_arm = {self.joint_names.index(name) for name in left_arm_names}
            inactive = [index for index in range(G1_ACTION_DIM) if index not in left_arm]
            if float(np.max(np.abs(delta[..., inactive]))) > 1.0e-5:
                raise RuntimeError("Mirrored V01 residual changed a non-left-Arm7 joint")
            if float(np.max(np.abs(delta))) > 0.1501:
                raise RuntimeError("Mirrored V01 residual exceeds the trained 0.15 bound")
        return physical_action

    def canonical_observation(self, physical_observation: Any) -> Any:
        return mirror_student_observation(
            _to_cpu_numpy(physical_observation),
            self.joint_names,
            self.physical_action_offset,
            self.physical_action_scale,
            self.canonical_action_offset,
            self.canonical_action_scale,
        )

    def __getattr__(self, name: str) -> Any:
        return getattr(self.policy, name)


class MirroredTeacherPolicy:
    """Run the canonical HIT Teacher/V01 runtime on a physical left-hand G1."""

    def __init__(
        self,
        policy: Any,
        joint_names: Sequence[str],
        canonical_action_offset: Any,
        canonical_action_scale: Any,
        physical_action_offset: Any,
        physical_action_scale: Any,
    ) -> None:
        joint_mirror_spec(joint_names)
        self.policy = policy
        self.joint_names = tuple(joint_names)
        self.canonical_action_offset = canonical_action_offset
        self.canonical_action_scale = canonical_action_scale
        self.physical_action_offset = physical_action_offset
        self.physical_action_scale = physical_action_scale
        self.last_policy_observation = None
        self.last_canonical_action = None
        self.last_physical_action = None

    def __call__(self, physical_observation: Any) -> Any:
        canonical_observation = mirror_teacher_observation(
            _to_cpu_numpy(physical_observation),
            self.joint_names,
            self.physical_action_offset,
            self.physical_action_scale,
            self.canonical_action_offset,
            self.canonical_action_scale,
        )
        policy_observation = (
            physical_observation.new_tensor(canonical_observation)
            if hasattr(physical_observation, "new_tensor")
            else canonical_observation
        )
        canonical_action = self.policy(policy_observation)
        physical_action = mirror_policy_action(
            canonical_action,
            self.joint_names,
            self.canonical_action_offset,
            self.canonical_action_scale,
            self.physical_action_offset,
            self.physical_action_scale,
        )
        self.last_policy_observation = _copy_array_or_tensor(canonical_observation)
        self.last_canonical_action = _copy_array_or_tensor(canonical_action)
        self.last_physical_action = _copy_array_or_tensor(physical_action)
        return physical_action

    def mirror_action(self, canonical_action: Any) -> Any:
        return mirror_policy_action(
            canonical_action,
            self.joint_names,
            self.canonical_action_offset,
            self.canonical_action_scale,
            self.physical_action_offset,
            self.physical_action_scale,
        )

    def __getattr__(self, name: str) -> Any:
        return getattr(self.policy, name)
