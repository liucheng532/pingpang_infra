"""Isaac-facing adapter for the transport-neutral doubles planner.

The adapter is intentionally thin: the frozen student remains responsible for
whole-body tracking, while this module owns manifest loading, command-source
reflection, and the two atomic hooks exposed by the compatible controller.
"""

from __future__ import annotations

import os
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from .mirror import (
    mirror_axial_vectors,
    mirror_joint_values,
    mirror_polar_vectors,
    mirror_quaternions_wxyz,
)
from .models import BallPrediction, PlanResult, RobotFeedback


_BASE_TARGET_UPDATE_DEADBAND_M = 0.08
_HOME_HOLD_REPLAN_DELAY_S = 0.25
_HIT_LOCKED_PHASES = frozenset({"HIT", "POST_DELAY"})
_LOCOMOTION_PHASES = frozenset({"OUTWARD", "OUTWARD_HOLD", "RETURN"})


@dataclass(frozen=True)
class CommitResult:
    accepted: bool
    duplicate: bool
    reason: str
    token: str | None


@dataclass(frozen=True)
class BaseTargetResult:
    accepted: bool
    reason: str
    robot: str
    phase: str
    safety_override: bool


def load_move_manifest(root: str | os.PathLike[str]) -> tuple[list[str], list[list[float]], list[bool]]:
    """Load the 0718 move bank using the controller's manifest convention."""

    root_path = Path(root).expanduser().resolve()
    rows = _manifest_rows(root_path / "dataindex.csv")
    files: list[str] = []
    targets: list[list[float]] = []
    labels: list[bool] = []
    for line_number, parts in rows:
        if len(parts) < 3 or parts[1] not in {"0", "1"}:
            raise ValueError(f"Invalid move manifest row {line_number}: {' '.join(parts)!r}")
        index, label, target_y = parts[:3]
        motion = _motion_path(root_path, index)
        files.append(str(motion))
        targets.append([0.0, float(target_y), 0.0])
        labels.append(bool(int(label)))
    if len(files) != 155:
        raise ValueError(f"Expected 155 move references, found {len(files)}")
    return files, targets, labels


def load_hit_manifest(root: str | os.PathLike[str]) -> tuple[list[str], list[list[float]], list[bool]]:
    """Load the 0302 hit bank using the controller's manifest convention."""

    root_path = Path(root).expanduser().resolve()
    rows = _manifest_rows(root_path / "dataindex.csv")
    files: list[str] = []
    targets: list[list[float]] = []
    labels: list[bool] = []
    for line_number, parts in rows:
        if len(parts) < 3 or parts[1] not in {"0", "1"}:
            raise ValueError(f"Invalid hit manifest row {line_number}: {' '.join(parts)!r}")
        index, label, target_text = parts[:3]
        values = [float(value) for value in target_text.replace("[", "").replace("]", "").split(",")]
        if len(values) != 3:
            raise ValueError(f"Hit target on row {line_number} must contain xyz values")
        files.append(str(_motion_path(root_path, index)))
        targets.append(values)
        labels.append(bool(int(label)))
    if len(files) != 441:
        raise ValueError(f"Expected 441 hit references, found {len(files)}")
    return files, targets, labels


def configure_command_cfg(command_cfg: Any, move_root: str | os.PathLike[str], hit_root: str | os.PathLike[str]) -> None:
    """Populate a ``DoublesDistillCommandCfg`` from two asset roots."""

    move_files, move_targets, move_labels = load_move_manifest(move_root)
    hit_files, hit_targets, hit_labels = load_hit_manifest(hit_root)
    command_cfg.motion_files = move_files
    command_cfg.motion_target = move_targets
    command_cfg.backhand = move_labels
    command_cfg.hit_motion_files = hit_files
    command_cfg.hit_motion_targets = hit_targets
    command_cfg.hit_backhand = hit_labels
    command_cfg.hold_pose_files = hit_files
    command_cfg.debug_vis = False


def mirror_command_sources(command: Any, joint_names: tuple[str, ...], body_names: tuple[str, ...]) -> None:
    """Reflect all frozen reference banks owned by one left-handed command.

    This must run after command construction and before the first observation;
    the environment is reset once more by the runner after this transformation.
    """

    if getattr(command, "_planner_sources_mirrored", False):
        return
    body_permutation = _body_permutation(body_names)

    command.joint_pos_all = mirror_joint_values(command.joint_pos_all, joint_names)
    command.joint_vel_all = mirror_joint_values(command.joint_vel_all, joint_names)
    command.body_pos_all = _mirror_body(command.body_pos_all, body_permutation, mirror_polar_vectors)
    command.body_quat_all = _mirror_body(command.body_quat_all, body_permutation, mirror_quaternions_wxyz)
    command.body_lin_vel_all = _mirror_body(command.body_lin_vel_all, body_permutation, mirror_polar_vectors)
    command.body_ang_vel_all = _mirror_body(command.body_ang_vel_all, body_permutation, mirror_axial_vectors)
    command.motion_targets_t = mirror_polar_vectors(command.motion_targets_t)
    command._outbound_motion_indices = _where(~command.motion_labels_t, command.device)
    command._return_motion_indices = _where(command.motion_labels_t, command.device)
    _refresh_motion_displacements(command)

    if hasattr(command, "_hold_ready_joint_pos"):
        command._hold_ready_joint_pos = mirror_joint_values(command._hold_ready_joint_pos, joint_names)

    hit = command.hit
    hit.joint_pos_all = mirror_joint_values(hit.joint_pos_all, joint_names)
    hit.joint_vel_all = mirror_joint_values(hit.joint_vel_all, joint_names)
    hit.body_pos_all = _mirror_body(hit.body_pos_all, body_permutation, mirror_polar_vectors)
    hit.body_quat_all = _mirror_body(hit.body_quat_all, body_permutation, mirror_quaternions_wxyz)
    racket_index = body_names.index("racket")
    hit.body_quat_all[..., racket_index, :] = _append_local_z_pi(
        hit.body_quat_all[..., racket_index, :]
    )
    command.body_quat_all[..., racket_index, :] = _append_local_z_pi(
        command.body_quat_all[..., racket_index, :]
    )
    hit.motion_targets = mirror_polar_vectors(hit.motion_targets)
    hit.motion_labels = ~hit.motion_labels
    command._planner_sources_mirrored = True


class IsaacControllerBridge:
    """Translate a :class:`PlanResult` into controller command hooks."""

    def __init__(
        self,
        commands: Mapping[str, Any],
        *,
        allow_locomotion_preemption: bool = False,
    ) -> None:
        self.commands = dict(commands)
        self.allow_locomotion_preemption = bool(allow_locomotion_preemption)
        self._committed_tokens: set[str] = set()
        self._base_target_rejections: list[BaseTargetResult] = []
        if len(self.commands) != 2:
            raise ValueError("IsaacControllerBridge requires exactly two commands")
        for name, command in self.commands.items():
            missing = [
                method
                for method in ("set_external_hit", "set_external_base_target", "set_external_outward_target")
                if not callable(getattr(command, method, None))
            ]
            if missing:
                raise TypeError(f"controller command {name!r} is missing hooks: {', '.join(missing)}")

    def feedback(self, now: float) -> dict[str, RobotFeedback]:
        result: dict[str, RobotFeedback] = {}
        for name, command in self.commands.items():
            pelvis = command.robot_pelvis_pos_origin
            velocity = command.robot.data.body_lin_vel_w[:, command.robot_pelvis_body_index]
            phase = self._phase_name(command)
            ready = self._accepts_hit(phase)
            readiness = getattr(command, "_handoff_ready", None)
            if phase == "HOME_HOLD" and ready and callable(readiness):
                ready = bool(readiness()[0].item())
            result[name] = RobotFeedback(
                name=name,
                base_xy=pelvis[0, :2].detach().cpu().numpy(),
                velocity_xy=velocity[0, :2].detach().cpu().numpy(),
                timestamp=now,
                controller_phase=phase,
                ready=ready,
            )
        return result

    @staticmethod
    def _phase_name(command: Any) -> str:
        phase = int(command.doubles_state[0].item())
        names = {
            command.HIT: "HIT",
            command.POST_DELAY: "POST_DELAY",
            command.OUTWARD: "OUTWARD",
            command.OUTWARD_HOLD: "OUTWARD_HOLD",
            command.RETURN: "RETURN",
            command.HOME_HOLD: "HOME_HOLD",
        }
        try:
            return names[phase]
        except KeyError as error:
            raise RuntimeError(f"unknown controller phase id: {phase}") from error

    def _accepts_hit(self, phase: str) -> bool:
        return phase == "HOME_HOLD" or (
            self.allow_locomotion_preemption and phase in _LOCOMOTION_PHASES
        )

    def inject_hit(self, robot: str, prediction: BallPrediction) -> None:
        command = self.commands[robot]
        command.set_external_hit(
            env_ids=_tensor([0], command.device, dtype="long"),
            racket_target=_tensor(prediction.position[None, :], command.device),
            target_velocity=_tensor(prediction.racket_velocity[None, :], command.device),
            time_to_strike_s=_tensor([prediction.time_to_strike], command.device),
            ball_velocity=_tensor(prediction.velocity[None, :], command.device),
        )

    def commit(self, plan: PlanResult, prediction: BallPrediction) -> CommitResult:
        """Atomically admit peer CLEAR and exactly one frozen-policy HIT."""

        token = plan.commit_token
        if token is None:
            return CommitResult(False, False, "missing_commit_token", None)
        if token in self._committed_tokens:
            return CommitResult(True, True, "duplicate_commit_ignored", token)
        if not plan.commit_requested or plan.relay_stage != "committed":
            return CommitResult(False, False, "plan_not_committed", token)
        if plan.hitter not in self.commands or plan.next_hitter not in self.commands:
            return CommitResult(False, False, "unknown_relay_robot", token)
        active = [name for name, command in plan.commands.items() if command.active]
        if active != [plan.hitter]:
            return CommitResult(False, False, "invalid_active_policy_set", token)
        if plan.commands[plan.next_hitter].role != "clear":
            return CommitResult(False, False, "missing_peer_clear_role", token)

        hitter_command = self.commands[plan.hitter]
        peer_command = self.commands[plan.next_hitter]
        if not self._accepts_hit(self._phase_name(hitter_command)):
            return CommitResult(False, False, "hitter_not_home_hold", token)
        peer_phase = self._phase_name(peer_command)
        if peer_phase in _HIT_LOCKED_PHASES:
            return CommitResult(False, False, "peer_hit_locked", token)
        if not self._accepts_hit(peer_phase):
            return CommitResult(False, False, "peer_not_home_hold", token)

        self.apply_positioning(plan)
        self.inject_hit(plan.hitter, prediction)
        self._committed_tokens.add(token)
        return CommitResult(True, False, "accepted", token)

    def inject_base_target(
        self,
        robot: str,
        target_xy: np.ndarray,
        return_target_y: float | None = None,
        safety_override: bool = False,
    ) -> BaseTargetResult:
        command = self.commands[robot]
        if abs(float(target_xy[0]) - float(command.robot_pelvis_pos_origin[0, 0].item())) > 1.0e-4:
            raise ValueError("Isaac doubles bridge only supports the trained lateral-Y base interface")
        return_arg = None if return_target_y is None else _tensor([return_target_y], command.device)
        phase_name = self._phase_name(command)
        rollback_fields = (
            "home_y",
            "target_y",
            "home_target_y",
            "move_distance",
            "outbound_y_scale",
            "return_y_scale",
            "stable_elapsed_s",
        )
        rollback_state = {
            field: getattr(command, field).clone()
            for field in rollback_fields
            if hasattr(command, field)
        }
        try:
            command.set_external_base_target(
                env_ids=_tensor([0], command.device, dtype="long"),
                target_xy=_tensor(np.asarray(target_xy)[None, :], command.device),
                return_target_y=return_arg,
                safety_override=safety_override,
            )
        except (RuntimeError, ValueError) as error:
            if (
                safety_override
                and "would reverse the selected motion reference" in str(error)
            ):
                for field, value in rollback_state.items():
                    getattr(command, field).copy_(value)
                result = BaseTargetResult(
                    accepted=False,
                    reason="selected_motion_reference_reversal",
                    robot=robot,
                    phase=phase_name,
                    safety_override=True,
                )
                self._base_target_rejections.append(result)
                return result
            phase = int(command.doubles_state[0].item())
            controller_state = {
                "pelvis_y": float(command.robot_pelvis_pos_origin[0, 1].item()),
                "target_y": float(command.target_y[0].item()),
                "home_target_y": float(command.home_target_y[0].item()),
            }
            if hasattr(command, "return_motion_index") and hasattr(
                command,
                "motion_targets_t",
            ):
                return_index = int(command.return_motion_index[0].item())
                controller_state["return_motion_index"] = return_index
                controller_state["return_reference_y"] = float(
                    command.motion_targets_t[return_index, 1].item()
                )
            raise type(error)(
                f"{robot} base target failed in controller phase {phase}: "
                f"target_y={float(target_xy[1]):+.6f}, "
                f"return_target_y={return_target_y}, safety_override={safety_override}; "
                f"controller_state={controller_state}; {error}"
            ) from error
        return BaseTargetResult(
            accepted=True,
            reason="accepted",
            robot=robot,
            phase=phase_name,
            safety_override=safety_override,
        )

    def base_target_diagnostics(self) -> dict[str, object]:
        reasons = Counter(result.reason for result in self._base_target_rejections)
        robots = Counter(result.robot for result in self._base_target_rejections)
        phases = Counter(
            f"{result.robot}:{result.phase}" for result in self._base_target_rejections
        )
        return {
            "rejection_count": len(self._base_target_rejections),
            "rejection_reasons": dict(sorted(reasons.items())),
            "rejection_by_robot": dict(sorted(robots.items())),
            "rejection_by_phase": dict(sorted(phases.items())),
        }

    def apply_positioning(self, plan: PlanResult) -> None:
        safety_active = bool(
            plan.diagnostics.get(
                "safety_active",
                plan.diagnostics.get("cbf_active", False)
                or plan.diagnostics.get("reachability_active", False),
            )
        )
        safety_emergency = bool(
            plan.diagnostics.get(
                "safety_emergency",
                plan.diagnostics.get("cbf_emergency", False),
            )
        )
        safety_controlled_robot = plan.diagnostics.get(
            "safety_controlled_robot",
            plan.diagnostics.get("cbf_controlled_robot"),
        )
        emergency_abort_authorized = bool(
            plan.diagnostics.get(
                "emergency_abort_authorized",
                plan.relay_stage == "idle",
            )
        )
        for name, command in self.commands.items():
            planned_command = plan.commands[name]
            trajectory_goal = planned_command.trajectory_base_position
            goal = trajectory_goal
            if safety_emergency:
                if safety_controlled_robot in self.commands and name != safety_controlled_robot:
                    continue
                controller_phase = int(command.doubles_state[0].item())
                if controller_phase <= command.POST_DELAY and not emergency_abort_authorized:
                    continue
                if controller_phase == command.OUTWARD_HOLD:
                    continue
                goal = self._emergency_phase_feasible_goal(
                    command,
                    planned_command.desired_base_position,
                )
                self.inject_base_target(
                    name,
                    goal,
                    return_target_y=float(command.home_target_y[0].item()),
                    safety_override=True,
                )
                continue
            controller_phase = int(command.doubles_state[0].item())
            motion_streamable = controller_phase in {
                command.OUTWARD,
                command.OUTWARD_HOLD,
                command.RETURN,
                command.HOME_HOLD,
            }
            safety_streamable = motion_streamable
            safety_controls_name = safety_controlled_robot in {name, "both"}
            outward_stage = (
                plan.next_hitter == name
                and (
                    (
                        plan.relay_stage
                        in {"committed", "strike", "follow_through"}
                        and planned_command.role == "clear"
                    )
                    or (
                        plan.phase == "pre_hit"
                        and plan.diagnostics.get("teammate_stage_mode") == "outward"
                    )
                )
            )
            if outward_stage:
                if controller_phase == command.RETURN and not safety_active:
                    continue
                stage_goal = self._policy_feasible_next_hitter_goal(
                    command,
                    trajectory_goal,
                )
                preview_goal = stage_goal.copy()
                preview_goal[1] = float(plan.diagnostics["preview_y"])
                preview_goal = self._policy_feasible_next_hitter_goal(
                    command,
                    preview_goal,
                )
                if controller_phase <= command.POST_DELAY:
                    command.set_external_outward_target(
                        env_ids=_tensor([0], command.device, dtype="long"),
                        target_y=_tensor([float(stage_goal[1])], command.device),
                    )
                else:
                    stream_safety = (
                        safety_active
                        and safety_controls_name
                        and safety_streamable
                    )
                    goal = (
                        planned_command.desired_base_position
                        if stream_safety
                        else stage_goal
                    )
                    if stream_safety:
                        goal = self._policy_feasible_next_hitter_goal(command, goal)
                        goal = self._return_reference_feasible_goal(command, goal)
                    return_goal = preview_goal
                    return_goal = self._outward_return_feasible_goal(
                        command,
                        goal,
                        return_goal,
                    )
                    target_error = self._base_target_error(
                        command,
                        controller_phase,
                        float(goal[1]),
                    )
                    return_error = abs(
                        float(command.home_target_y[0].item()) - float(return_goal[1])
                    )
                    home_hold_ready = (
                        stream_safety
                        or plan.commit_requested
                        or controller_phase != command.HOME_HOLD
                        or float(command.state_elapsed_s[0].item())
                        >= _HOME_HOLD_REPLAN_DELAY_S
                    )
                    if (
                        motion_streamable
                        and home_hold_ready
                        and max(target_error, return_error)
                        > _BASE_TARGET_UPDATE_DEADBAND_M
                    ):
                        self.inject_base_target(
                            name,
                            goal,
                            return_target_y=float(return_goal[1]),
                            safety_override=stream_safety,
                        )
                continue
            if safety_active and safety_controls_name and safety_streamable:
                goal = planned_command.desired_base_position
                if plan.next_hitter == name:
                    goal = self._policy_feasible_next_hitter_goal(command, goal)
                    return_goal = self._policy_feasible_next_hitter_goal(
                        command,
                        trajectory_goal,
                    )
                else:
                    return_goal = np.asarray(
                        [
                            float(command.robot_pelvis_pos_origin[0, 0].item()),
                            float(command.home_target_y[0].item()),
                        ],
                        dtype=float,
                    )
                goal = self._return_reference_feasible_goal(command, goal)
                return_goal = self._outward_return_feasible_goal(
                    command,
                    goal,
                    return_goal,
                )
                self.inject_base_target(
                    name,
                    goal,
                    return_target_y=float(return_goal[1]),
                    safety_override=True,
                )
                continue
            if (
                (plan.phase == "post_hit" or plan.relay_stage == "follow_through")
                and plan.hitter == name
            ):
                if controller_phase <= command.POST_DELAY:
                    command.set_external_outward_target(
                        env_ids=_tensor([0], command.device, dtype="long"),
                        target_y=_tensor([float(goal[1])], command.device),
                    )
                elif controller_phase == command.OUTWARD:
                    target_error = self._base_target_error(command, controller_phase, float(goal[1]))
                    if target_error > _BASE_TARGET_UPDATE_DEADBAND_M:
                        self.inject_base_target(
                            name,
                            goal,
                            return_target_y=float(command.home_target_y[0].item()),
                        )
            if plan.phase in {"pre_hit", "post_hit"} and plan.next_hitter == name:
                goal = self._policy_feasible_next_hitter_goal(command, goal)
                phase = int(command.doubles_state[0].item())
                movable_phases = {command.OUTWARD, command.HOME_HOLD}
                if plan.phase == "post_hit":
                    movable_phases.update({command.OUTWARD_HOLD, command.RETURN})
                target_error = self._base_target_error(command, phase, float(goal[1]))
                home_hold_ready = (
                    phase != command.HOME_HOLD
                    or float(command.state_elapsed_s[0].item()) >= _HOME_HOLD_REPLAN_DELAY_S
                )
                if (
                    phase in movable_phases
                    and home_hold_ready
                    and target_error > _BASE_TARGET_UPDATE_DEADBAND_M
                ):
                    self.inject_base_target(name, goal, return_target_y=float(goal[1]))

    @staticmethod
    def _policy_feasible_next_hitter_goal(command: Any, goal: np.ndarray) -> np.ndarray:
        result = np.asarray(goal, dtype=float).copy()
        cfg = getattr(command, "cfg", None)
        home_range = getattr(cfg, "home_target_y_range", None)
        outward_target_y = float(getattr(cfg, "outward_target_y", 0.0))
        if home_range is None or len(home_range) != 2 or outward_target_y == 0.0:
            return result
        home_magnitude = max(abs(float(home_range[0])), abs(float(home_range[1])))
        if outward_target_y < 0.0:
            result[1] = min(float(result[1]), -home_magnitude)
        else:
            result[1] = max(float(result[1]), home_magnitude)
        return result

    @staticmethod
    def _emergency_phase_feasible_goal(command: Any, goal: np.ndarray) -> np.ndarray:
        result = np.asarray(goal, dtype=float).copy()
        phase = int(command.doubles_state[0].item())
        cfg = getattr(command, "cfg", None)
        if phase == command.RETURN:
            target_y = float(command.target_y[0].item())
            return_y = float(command.home_target_y[0].item())
            return_direction = 0.0
            if hasattr(command, "return_motion_index") and hasattr(
                command,
                "motion_targets_t",
            ):
                return_index = int(command.return_motion_index[0].item())
                return_direction = float(command.motion_targets_t[return_index, 1].item())
            if abs(return_direction) <= 1.0e-9:
                return_direction = return_y - target_y
            if abs(return_direction) <= 1.0e-9:
                return_direction = -float(getattr(cfg, "outward_target_y", 0.0))
            if abs(return_direction) <= 1.0e-9 or (return_y - target_y) * return_direction >= 0.0:
                result[1] = return_y
            else:
                result[1] = target_y
            return result
        if phase == command.OUTWARD:
            outward_target_y = float(getattr(cfg, "outward_target_y", 0.0))
            if outward_target_y < 0.0 and result[1] < 0.0:
                result[1] = max(float(result[1]), outward_target_y)
            elif outward_target_y > 0.0 and result[1] > 0.0:
                result[1] = min(float(result[1]), outward_target_y)
        return result

    @staticmethod
    def _return_reference_feasible_goal(command: Any, goal: np.ndarray) -> np.ndarray:
        result = np.asarray(goal, dtype=float).copy()
        if int(command.doubles_state[0].item()) != command.RETURN:
            return result
        target_y = float(command.target_y[0].item())
        return_direction = 0.0
        if hasattr(command, "return_motion_index") and hasattr(
            command,
            "motion_targets_t",
        ):
            return_index = int(command.return_motion_index[0].item())
            return_direction = float(command.motion_targets_t[return_index, 1].item())
        if abs(return_direction) <= 1.0e-9:
            return_y = float(command.home_target_y[0].item())
            return_direction = return_y - target_y
        if abs(return_direction) <= 1.0e-9:
            outward_target_y = float(
                getattr(getattr(command, "cfg", None), "outward_target_y", 0.0)
            )
            return_direction = -outward_target_y
        if return_direction > 0.0:
            result[1] = max(float(result[1]), target_y)
        elif return_direction < 0.0:
            result[1] = min(float(result[1]), target_y)
        return result

    @staticmethod
    def _outward_return_feasible_goal(
        command: Any,
        outward_goal: np.ndarray,
        return_goal: np.ndarray,
    ) -> np.ndarray:
        result = np.asarray(return_goal, dtype=float).copy()
        outward_target_y = float(
            getattr(getattr(command, "cfg", None), "outward_target_y", 0.0)
        )
        if outward_target_y < 0.0:
            result[1] = max(float(result[1]), float(outward_goal[1]))
        elif outward_target_y > 0.0:
            result[1] = min(float(result[1]), float(outward_goal[1]))
        return result

    @staticmethod
    def _base_target_error(command: Any, phase: int, goal_y: float) -> float:
        if phase in {command.OUTWARD, command.OUTWARD_HOLD}:
            terminal_y = float(command.target_y[0].item())
        elif phase == command.RETURN:
            terminal_y = float(command.home_target_y[0].item())
        else:
            terminal_y = float(command.robot_pelvis_pos_origin[0, 1].item())
        return abs(terminal_y - goal_y)


def _manifest_rows(path: Path) -> list[tuple[int, list[str]]]:
    if not path.is_file():
        raise FileNotFoundError(path)
    rows: list[tuple[int, list[str]]] = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines()[1:], start=2):
        if line.strip():
            rows.append((line_number, line.split(maxsplit=2)))
    return rows


def _motion_path(root: Path, index: str) -> Path:
    folder = index if index.endswith(":v0") else f"{root.name}-{index}:v0"
    motion = root / folder / "motion.npz"
    if not motion.is_file():
        raise FileNotFoundError(motion)
    return motion


def _body_permutation(names: tuple[str, ...]) -> tuple[int, ...]:
    indices = {name: index for index, name in enumerate(names)}
    permutation: list[int] = []
    for name in names:
        if name.startswith("left_"):
            reflected = "right_" + name[5:]
        elif name.startswith("right_"):
            reflected = "left_" + name[6:]
        else:
            reflected = name
        if reflected not in indices:
            raise ValueError(f"body_names has no reflected counterpart for {name!r}")
        permutation.append(indices[reflected])
    return tuple(permutation)


def _mirror_body(values: Any, permutation: tuple[int, ...], transform: Any) -> Any:
    reordered = values[..., list(permutation), :]
    return transform(reordered)


def _append_local_z_pi(quaternion: Any) -> Any:
    return quaternion[..., [3, 2, 1, 0]] * _sign_tensor(quaternion, [-1.0, 1.0, -1.0, 1.0])


def _sign_tensor(reference: Any, values: list[float]) -> Any:
    if hasattr(reference, "new_tensor"):
        return reference.new_tensor(values)
    return np.asarray(values, dtype=np.asarray(reference).dtype)


def _where(mask: Any, device: Any) -> Any:
    import torch

    return torch.where(mask)[0].to(device=device)


def _tensor(value: Any, device: Any, dtype: str | None = None) -> Any:
    import torch

    torch_dtype = torch.long if dtype == "long" else torch.float32
    return torch.as_tensor(value, device=device, dtype=torch_dtype)


def _refresh_motion_displacements(command: Any) -> None:
    motion_ids = _arange(command._motion_time_totals_all.shape[0], command.device)
    last_steps = command._motion_time_totals_all - 1
    command._motion_pelvis_start_y = command.body_pos_all[:, 0, command.motion_pelvis_body_index, 1]
    command._motion_pelvis_end_y = command.body_pos_all[motion_ids, last_steps, command.motion_pelvis_body_index, 1]
    command._motion_pelvis_displacement_y = command._motion_pelvis_end_y - command._motion_pelvis_start_y


def _arange(count: int, device: Any) -> Any:
    import torch

    return torch.arange(count, device=device)


__all__ = [
    "BaseTargetResult",
    "CommitResult",
    "IsaacControllerBridge",
    "configure_command_cfg",
    "load_hit_manifest",
    "load_move_manifest",
    "mirror_command_sources",
]
