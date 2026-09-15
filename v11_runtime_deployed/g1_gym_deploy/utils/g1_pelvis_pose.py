from __future__ import annotations

import numpy as np

from utils.math import matrix_from_quat_wxyz, rotmat_to_quat_xyzw


class G1PelvisPoseEstimator:
    """Reconstruct the G1 pelvis world pose from torso mocap and waist joints."""

    WAIST_YAW_INDEX = 12
    WAIST_ROLL_INDEX = 13
    WAIST_PITCH_INDEX = 14
    WAIST_LIMITS_RAD = np.array([2.618, 0.52, 0.52], dtype=np.float64)
    # Hardware state can transiently exceed the nominal URDF soft limit by a
    # few hundredths of a radian.  This check is meant to catch a broken joint
    # mapping or corrupt state, not to crash the policy during a recoverable
    # physical excursion.  FK continues to use the measured angle unchanged.
    WAIST_HARD_LIMIT_MARGIN_RAD = 0.10
    WAIST_ROLL_OFFSET_M = np.array([-0.0039635, 0.0, 0.044], dtype=np.float64)

    @staticmethod
    def _rot_x(angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)

    @staticmethod
    def _rot_y(angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)

    @staticmethod
    def _rot_z(angle: float) -> np.ndarray:
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)

    def estimate(
        self,
        torso_pos_origin: np.ndarray,
        torso_quat_origin_wxyz: np.ndarray,
        joint_pos_gym: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        torso_pos = np.asarray(torso_pos_origin, dtype=np.float64).reshape(-1)
        torso_quat = np.asarray(torso_quat_origin_wxyz, dtype=np.float64).reshape(-1)
        joint_pos = np.asarray(joint_pos_gym, dtype=np.float64).reshape(-1)
        if torso_pos.shape != (3,):
            raise ValueError(f"torso_pos_origin must have shape (3,), got {torso_pos.shape}.")
        if torso_quat.shape != (4,):
            raise ValueError(f"torso_quat_origin_wxyz must have shape (4,), got {torso_quat.shape}.")
        if joint_pos.shape != (29,):
            raise ValueError(f"joint_pos_gym must have shape (29,), got {joint_pos.shape}.")
        if not np.isfinite(torso_pos).all() or not np.isfinite(torso_quat).all():
            raise ValueError("Torso pose contains non-finite values.")
        if not np.isfinite(joint_pos).all():
            raise ValueError("Joint position contains non-finite values.")

        quat_norm = float(np.linalg.norm(torso_quat))
        if quat_norm < 1.0e-8:
            raise ValueError("Torso quaternion has near-zero norm.")
        torso_quat = torso_quat / quat_norm

        waist = joint_pos[
            [self.WAIST_YAW_INDEX, self.WAIST_ROLL_INDEX, self.WAIST_PITCH_INDEX]
        ]
        hard_limits = self.WAIST_LIMITS_RAD + self.WAIST_HARD_LIMIT_MARGIN_RAD
        if np.any(np.abs(waist) > hard_limits):
            raise ValueError(
                f"Waist joint position {waist.tolist()} exceeds URDF limits "
                f"{self.WAIST_LIMITS_RAD.tolist()} plus hard margin "
                f"{self.WAIST_HARD_LIMIT_MARGIN_RAD} rad."
            )
        yaw, roll, pitch = waist

        rotation_origin_torso = matrix_from_quat_wxyz(torso_quat)
        rotation_pelvis_torso = self._rot_z(yaw) @ self._rot_x(roll) @ self._rot_y(pitch)
        position_pelvis_torso = self._rot_z(yaw) @ self.WAIST_ROLL_OFFSET_M

        rotation_origin_pelvis = rotation_origin_torso @ rotation_pelvis_torso.T
        position_origin_pelvis = torso_pos - rotation_origin_pelvis @ position_pelvis_torso
        pelvis_quat_xyzw = rotmat_to_quat_xyzw(rotation_origin_pelvis)
        pelvis_quat_wxyz = pelvis_quat_xyzw[[3, 0, 1, 2]]
        if pelvis_quat_wxyz[0] < 0.0:
            pelvis_quat_wxyz *= -1.0

        return position_origin_pelvis.astype(np.float32), pelvis_quat_wxyz.astype(np.float32)
