"""Physical-left wrapper around the unchanged canonical V9 reference scheduler."""

from __future__ import annotations

from typing import Any

import numpy as np

from utils.doubles_reference_scheduler import DoublesReference, DoublesReferenceScheduler
from utils.joint_mapping import LAB_JOINT_NAMES
from utils.left_right_mirror import (
    mirror_axial_vectors,
    mirror_joint_values,
    mirror_planar_vectors,
    mirror_polar_vectors,
    mirror_rpy,
)


class MirroredReferenceScheduler:
    def __init__(self, scheduler: DoublesReferenceScheduler) -> None:
        self.scheduler = scheduler

    @property
    def state(self):
        return self.scheduler.state

    @property
    def home_y(self) -> float:
        return -float(self.scheduler.home_y)

    @property
    def outward_target_y(self) -> float:
        return -float(self.scheduler.outward_target_y)

    @property
    def episode_start_x(self) -> float:
        return float(self.scheduler.episode_start_x)

    def _physical_reference(self, reference: DoublesReference) -> DoublesReference:
        return DoublesReference(
            strike_time=reference.strike_time,
            target_velocity=mirror_polar_vectors(reference.target_velocity),
            racket_target=mirror_polar_vectors(reference.racket_target),
            target_base=mirror_planar_vectors(reference.target_base),
            joint_pos=mirror_joint_values(reference.joint_pos, LAB_JOINT_NAMES),
            joint_vel=mirror_joint_values(reference.joint_vel, LAB_JOINT_NAMES),
            hit_joint_pos=mirror_joint_values(reference.hit_joint_pos, LAB_JOINT_NAMES),
            hit_joint_vel=mirror_joint_values(reference.hit_joint_vel, LAB_JOINT_NAMES),
            hit_anchor_pos_w=mirror_polar_vectors(reference.hit_anchor_pos_w),
            phase=reference.phase.copy(),
            state=reference.state,
            hit_motion_index=reference.hit_motion_index,
            move_motion_index=reference.move_motion_index,
            reference_step=reference.reference_step,
            outward_hold_elapsed_s=reference.outward_hold_elapsed_s,
            outward_hold_duration_s=reference.outward_hold_duration_s,
            outward_hold_entry_reason=reference.outward_hold_entry_reason,
        )

    def reset(self, pelvis_position: np.ndarray, torso_position: np.ndarray) -> DoublesReference:
        reference = self.scheduler.reset(
            mirror_polar_vectors(np.asarray(pelvis_position, dtype=np.float32)),
            mirror_polar_vectors(np.asarray(torso_position, dtype=np.float32)),
        )
        return self._physical_reference(reference)

    def sync_robot_pose(self, pelvis_position: np.ndarray, torso_position: np.ndarray) -> None:
        self.scheduler.sync_robot_pose(
            mirror_polar_vectors(np.asarray(pelvis_position, dtype=np.float32)),
            mirror_polar_vectors(np.asarray(torso_position, dtype=np.float32)),
        )

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
        reference = self.scheduler.update(
            time_to_strike,
            mirror_polar_vectors(np.asarray(racket_target, dtype=np.float32)),
            mirror_polar_vectors(np.asarray(target_velocity, dtype=np.float32)),
            mirror_polar_vectors(np.asarray(pelvis_position, dtype=np.float32)),
            mirror_polar_vectors(np.asarray(torso_position, dtype=np.float32)),
            mirror_rpy(np.asarray(pelvis_rpy, dtype=np.float32)),
            mirror_axial_vectors(np.asarray(pelvis_angular_velocity_w, dtype=np.float32)),
            dt=dt,
            start_external_hit=start_external_hit,
        )
        return self._physical_reference(reference)

    def output(self) -> DoublesReference:
        return self._physical_reference(self.scheduler.output())

    def set_external_hit(
        self,
        racket_target: np.ndarray,
        target_velocity: np.ndarray,
        time_to_strike_s: float,
        ball_velocity: np.ndarray | None = None,
    ) -> None:
        self.scheduler.set_external_hit(
            mirror_polar_vectors(np.asarray(racket_target, dtype=np.float32)),
            mirror_polar_vectors(np.asarray(target_velocity, dtype=np.float32)),
            time_to_strike_s,
            None
            if ball_velocity is None
            else mirror_polar_vectors(np.asarray(ball_velocity, dtype=np.float32)),
        )

    def set_external_outward_target(self, target_y: float) -> None:
        self.scheduler.set_external_outward_target(-float(target_y))

    def set_external_home_target(self, target_y: float) -> None:
        self.scheduler.set_external_home_target(-float(target_y))

    def stage_runtime_motion_config(
        self,
        command_id: str,
        goal_x: float,
        outward_y: float,
        home_y: float,
        outward_hold_s: float,
    ) -> None:
        self.scheduler.stage_runtime_motion_config(
            command_id=command_id,
            goal_x=goal_x,
            outward_y=-float(outward_y),
            home_y=-float(home_y),
            outward_hold_s=outward_hold_s,
        )

    def runtime_motion_config_status(self) -> dict:
        status = self.scheduler.runtime_motion_config_status()
        active = dict(status["active"])
        active["outward_y"] = -float(active["outward_y"])
        active["home_y"] = -float(active["home_y"])
        pending = status["pending"]
        if pending is not None:
            pending = dict(pending)
            pending["outward_y"] = -float(pending["outward_y"])
            pending["home_y"] = -float(pending["home_y"])
        return {**status, "active": active, "pending": pending}

    def bootstrap_outward_hold(self, home_y: float, outward_y: float | None = None) -> None:
        self.scheduler.bootstrap_outward_hold(
            -float(home_y),
            None if outward_y is None else -float(outward_y),
        )

    def set_external_base_target(
        self,
        target_xy: np.ndarray,
        return_target_y: float | None = None,
        safety_override: bool = False,
    ) -> None:
        self.scheduler.set_external_base_target(
            mirror_planar_vectors(np.asarray(target_xy, dtype=np.float32)),
            None if return_target_y is None else -float(return_target_y),
            safety_override=safety_override,
        )

    def clear_external_control(self) -> None:
        self.scheduler.clear_external_control()

    def __getattr__(self, name: str) -> Any:
        return getattr(self.scheduler, name)
