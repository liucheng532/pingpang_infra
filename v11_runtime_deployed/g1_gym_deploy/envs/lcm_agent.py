import time
import os
import sys
import csv
import json
import threading
from datetime import datetime

from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
from std_msgs.msg import Float32
try:
    from std_msgs.msg import String
except ImportError:  # pragma: no cover - lightweight test stubs omit String
    String = None

try:
    import rospy
except ImportError:  # ROS may be unavailable in some deployments
    rospy = None

import copy
import lcm
import numpy as np
import torch
import random
from utils.cheetah_state_estimator import StateEstimator, monotonic_raw_ns
from utils.math import matrix_from_quat_wxyz, quat_from_rpy_wxyz, quat_to_rotmat_xyzw, quaternion_to_rpy
from lcm_types.pd_tau_targets_lcmt import pd_tau_targets_lcmt
from utils.command_profile import RCControllerProfile
from utils.data_utils import MotionCommand, MoveMotionBank
from utils.doubles_reference_scheduler import DoublesPhase, DoublesReferenceScheduler
from utils.g1_pelvis_pose import G1PelvisPoseEstimator
from utils.joint_mapping import FROM_GYM_TO_LAB, FROM_LAB_TO_GYM
from utils.mirrored_reference_scheduler import MirroredReferenceScheduler
from utils.planner_ros_bridge import PlannerRosBridge
from utils.runtime_motion_config import parse_runtime_motion_config


def _to_torch(x, device=None):
    if torch.is_tensor(x):
        return x
    return torch.as_tensor(x, dtype=torch.float32, device=device)


def _quat_conjugate(q):
    w, x, y, z = torch.unbind(q, dim=-1)
    return torch.stack((w, -x, -y, -z), dim=-1)


def _quat_mul(q1, q2):
    w1, x1, y1, z1 = torch.unbind(q1, dim=-1)
    w2, x2, y2, z2 = torch.unbind(q2, dim=-1)
    return torch.stack(
        (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ),
        dim=-1,
    )


def _quat_rotate(q, v):
    zeros = torch.zeros_like(v[..., :1])
    v_as_quat = torch.cat((zeros, v), dim=-1)
    return _quat_mul(_quat_mul(q, v_as_quat), _quat_conjugate(q))[..., 1:]


def quat_inv(q):
    """Quaternion inverse for normalized WXYZ quaternions."""
    q = np.asarray(q, dtype=float)
    q_norm = np.linalg.norm(q, axis=-1, keepdims=True)
    q = q / np.clip(q_norm, 1e-9, None)
    q_inv = q.copy()
    q_inv[..., 1:] *= -1.0
    return q_inv


def yaw_quat(q):
    """Extract yaw-only quaternion (WXYZ) from full WXYZ quaternion."""
    q = np.asarray(q, dtype=float)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    half_yaw = 0.5 * yaw
    cy = np.cos(half_yaw)
    sy = np.sin(half_yaw)
    zeros = np.zeros_like(cy)
    return np.stack((cy, zeros, zeros, sy), axis=-1)


def quat_apply(q, v):
    """Rotate vectors by WXYZ quaternion(s)."""
    q = np.asarray(q, dtype=float)
    v = np.asarray(v, dtype=float)
    q_vec = q[..., 1:]
    q_w = q[..., :1]
    t = 2.0 * np.cross(q_vec, v)
    return v + q_w * t + np.cross(q_vec, t)


def subtract_frame_transforms(quat_a, quat_b, pos_a, pos_b):
    device = quat_a.device if torch.is_tensor(quat_a) else None
    qa = _to_torch(quat_a, device=device)
    qb = _to_torch(quat_b, device=qa.device)
    pa = _to_torch(pos_a, device=qa.device)
    pb = _to_torch(pos_b, device=qa.device)

    if qa.ndim == 1:
        qa = qa.unsqueeze(0)
    if qb.ndim == 1:
        qb = qb.unsqueeze(0)
    if pa.ndim == 1:
        pa = pa.unsqueeze(0)
    if pb.ndim == 1:
        pb = pb.unsqueeze(0)

    qb_conj = _quat_conjugate(qb)
    rel_pos = _quat_rotate(qb_conj, pa - pb)
    rel_quat = _quat_mul(qb_conj, qa)
    return rel_pos, rel_quat


ARMATURE_5020 = 0.003609725
ARMATURE_7520_14 = 0.010177520
ARMATURE_7520_22 = 0.025101925
ARMATURE_4010 = 0.00425

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_5020 = ARMATURE_5020 * NATURAL_FREQ**2
STIFFNESS_7520_14 = ARMATURE_7520_14 * NATURAL_FREQ**2
STIFFNESS_7520_22 = ARMATURE_7520_22 * NATURAL_FREQ**2
STIFFNESS_4010 = ARMATURE_4010 * NATURAL_FREQ**2

DAMPING_5020 = 2.0 * DAMPING_RATIO * ARMATURE_5020 * NATURAL_FREQ
DAMPING_7520_14 = 2.0 * DAMPING_RATIO * ARMATURE_7520_14 * NATURAL_FREQ
DAMPING_7520_22 = 2.0 * DAMPING_RATIO * ARMATURE_7520_22 * NATURAL_FREQ
DAMPING_4010 = 2.0 * DAMPING_RATIO * ARMATURE_4010 * NATURAL_FREQ

class beyond_mimic_config:

    dof_names = ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 
                'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 

                'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 
                'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint', 

                'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint', 
                
                'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 
                'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
                
                'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 
                'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint']

    stiffness = {'hip_yaw': STIFFNESS_7520_14,
                'hip_roll': STIFFNESS_7520_22,
                'hip_pitch': STIFFNESS_7520_14,
                'knee': STIFFNESS_7520_22,
                'ankle': 2.0 * STIFFNESS_5020,

                'waist_yaw': 300.0,
                'waist_roll': 300.0,
                'waist_pitch': 300.0,

                'shoulder_pitch': STIFFNESS_5020,
                'shoulder_roll': STIFFNESS_5020,
                'shoulder_yaw': STIFFNESS_5020,
                
                'elbow': STIFFNESS_5020,
                'wrist_roll': STIFFNESS_5020,
                'wrist_pitch': STIFFNESS_4010,
                'wrist_yaw': STIFFNESS_4010,
                    }  # [N*m/rad]
    damping = { 'hip_yaw': DAMPING_7520_14,
                'hip_roll': DAMPING_7520_22,
                'hip_pitch': DAMPING_7520_14,
                'knee': DAMPING_7520_22,
                'ankle': 2.0 * DAMPING_5020,
                
                'waist_yaw': 5.0,
                'waist_roll': 5.0,
                'waist_pitch': 5.0,
                
                'shoulder_pitch': DAMPING_5020,
                'shoulder_roll': DAMPING_5020,
                'shoulder_yaw': DAMPING_5020,

                'elbow': DAMPING_5020,
                'wrist_roll': DAMPING_5020,
                'wrist_pitch': DAMPING_4010,
                'wrist_yaw': DAMPING_4010,
                    }  # [N*m/rad]  # [N*m*s/rad]
    
    armature = { 'hip_yaw': ARMATURE_7520_14,
                'hip_roll': ARMATURE_7520_22,
                'hip_pitch': ARMATURE_7520_14,
                'knee': ARMATURE_7520_22,
                'ankle': 2.0 * ARMATURE_5020,
                
                'waist_yaw': ARMATURE_7520_14,
                'waist_roll': 2.0 * ARMATURE_5020,
                'waist_pitch': 2.0 * ARMATURE_5020,
                
                'shoulder_pitch': ARMATURE_5020,
                'shoulder_roll': ARMATURE_5020,
                'shoulder_yaw': ARMATURE_5020,

                'elbow': ARMATURE_5020,
                'wrist_roll': ARMATURE_5020,
                'wrist_pitch': ARMATURE_4010,
                'wrist_yaw': ARMATURE_4010,
                    }  # [N*m/rad]  # [N*m*s/rad]
    torque_limit = { 'hip_yaw': 88.0,
                'hip_roll': 139.0,
                'hip_pitch': 88.0,
                'knee': 139.0,
                'ankle': 50.0,
                
                'waist_yaw': 88.0,
                'waist_roll': 50.0,
                'waist_pitch': 50.0,
                
                'shoulder_pitch':  25.0,
                'shoulder_roll':  25.0,
                'shoulder_yaw':  25.0,

                'elbow':  25.0,
                'wrist_roll':  25.0,
                'wrist_pitch': 5.0,
                'wrist_yaw': 5.0,
                    }  # [N*m/rad]  # [N*m*s/rad]
    action_scale = 0.25



TORSO_POSE_ORIGIN_TOPIC = "/torso_pose_origin"
PREDICTED_BALL_POSITION_TOPIC = "/predicted_ball_position"
PREDICTED_RACKET_VELOCITY_TOPIC = "/predicted_racket_velocity"
PREDICTED_BALL_PREDICT_TIME_TOPIC = "/predicted_ball_predict_time"



class LCMAgent():
    def __init__(
        self,
        se: StateEstimator,
        command_profile: RCControllerProfile,
        hit_motion_data="/home/unitree/haoran/doubles-student-1666-deploy/data/0302_combined",
        move_motion_data="/home/unitree/haoran/doubles-student-1666-deploy/data/0718-move-160-80hz",
        shadow=False,
        mirror_left_hand=False,
        external_planner=False,
        robot_id=None,
        torso_topic=None,
        command_timeout_s=0.25,
        planner_home_y=None,
        bootstrap_outward_hold=False,
        startup_home_current=False,
        stationary_hit_test="none",
        hit_reference_lead_steps=0,
        external_hit_command_mode="frozen",
        episode_start_x=None,
        raw_ball_source=None,
        racket_hand="right",
        right_side_canonicalization=False,
        native_no_mirror=False,
        action_clip=100.0,
        v10_relative_x=False,
        target_base_x_clip=0.04,
        common_hold_pose_path=None,
        common_hold_pose_sha256=None,
        move_excluded_source_ids=None,
        expected_active_move_count=None,
    ):
        self.se = se
        self.command_profile = command_profile

        self.dt = 0.02
        self.timestep = 0
        self.num_envs = 1
        self.num_actions = 29
        self.num_dofs = 29
        self.num_history_length = 10
        self.shadow = bool(shadow)
        self.mirror_left_hand = bool(mirror_left_hand)
        if racket_hand not in ("right", "left"):
            raise ValueError("racket_hand must be 'right' or 'left'.")
        self.racket_hand = racket_hand
        self.right_side_canonicalization = bool(right_side_canonicalization)
        self.native_no_mirror = bool(native_no_mirror)
        if sum(
            int(value)
            for value in (
                self.mirror_left_hand,
                self.right_side_canonicalization,
                self.native_no_mirror,
            )
        ) > 1:
            raise ValueError("left-hand mirror, reflected side, and native mode are exclusive.")
        self.external_planner = bool(external_planner)
        self.robot_id = robot_id
        self.torso_topic = torso_topic or TORSO_POSE_ORIGIN_TOPIC
        self.planner_home_y = None if planner_home_y is None else float(planner_home_y)
        self.bootstrap_outward_hold = bool(bootstrap_outward_hold)
        self.startup_home_current = bool(startup_home_current)
        self.stationary_hit_test = stationary_hit_test
        self.external_hit_command_mode = external_hit_command_mode
        self.episode_start_x_override = (
            None if episode_start_x is None else float(episode_start_x)
        )
        self.action_clip = float(action_clip)
        if not np.isfinite(self.action_clip) or self.action_clip <= 0.0:
            raise ValueError("action_clip must be finite and positive.")
        self.v10_relative_x = bool(v10_relative_x)
        self.raw_ball_source = raw_ball_source
        self._command_seq = 0
        self.last_command_seq = 0
        self.last_command_publish_monotonic_raw_ns = 0

        self.motions = MotionCommand(hit_motion_data, device="cpu")
        self.move_motions = MoveMotionBank(
            move_motion_data,
            expected_count=155,
            expected_fps=80.0,
            excluded_source_ids=move_excluded_source_ids,
            expected_active_count=expected_active_move_count,
        )
        canonical_reference_scheduler = DoublesReferenceScheduler(
            self.motions,
            self.move_motions,
            control_dt=self.dt,
            stationary_hit_test=self.stationary_hit_test,
            external_control=self.external_planner,
            hit_reference_lead_steps=hit_reference_lead_steps,
            episode_start_x=self.episode_start_x_override,
            move_side_sign=(
                -1
                if self.right_side_canonicalization or self.native_no_mirror
                else 1
            ),
            native_no_mirror=self.native_no_mirror,
            v10_relative_x=self.v10_relative_x,
            target_base_x_clip=target_base_x_clip,
            reference_end_forces_hold=self.v10_relative_x,
            latch_hold_x_on_entry=self.v10_relative_x,
            common_hold_pose_path=common_hold_pose_path,
            common_hold_pose_sha256=common_hold_pose_sha256,
        )
        self.reference_scheduler = (
            MirroredReferenceScheduler(canonical_reference_scheduler)
            if self.mirror_left_hand
            else canonical_reference_scheduler
        )
        if common_hold_pose_path is not None:
            print(
                "[V11_CONTRACT] "
                f"common_hold_sha256={common_hold_pose_sha256} "
                f"move_pool_active={self.move_motions.active_count} "
                f"excluded_source_ids={sorted(self.move_motions.excluded_source_ids or ())} "
                f"mirror_left_hand={self.mirror_left_hand} action_clip={self.action_clip} "
                f"target_base_x_clip={target_base_x_clip}"
            )
        self.target_range = np.array([[0.4, 0.5], [-0.75, 0.75], [0.85, 1.4]])
        self.time_range = [-0.5, 0.6]
        self.max_infer_time = 0.54
        self.handle_time = self.time_range[0]
        self.handle_timestep = None
        self.handle_receive_walltime = None
        self._tts_receive_monotonic_ns = 0
        self._tts_source_stamp_s = 0.0
        self.strike_time = self.time_range[0]

        self.ractarget = np.array( [0.5 * (self.target_range[0, 0] + self.target_range[0, 1]), 
                                        0.5 * (self.target_range[1, 0] + self.target_range[1, 1]),
                                        0.5 * (self.target_range[2, 0] + self.target_range[2, 1])])

        self.ballvel = np.array([2.0, 0.0, 0.5])
        self.recvvel = np.array([2.0, 0.0, 0.5])
        self.recvtarget = np.array( [0.5 * (self.target_range[0, 0] + self.target_range[0, 1]), 
                                        0.5 * (self.target_range[1, 0] + self.target_range[1, 1]),
                                        0.5 * (self.target_range[2, 0] + self.target_range[2, 1])])

        
        self.torso_pos = np.zeros(3, dtype=float)
        self.quat_mocap = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.pelvis_pose_estimator = G1PelvisPoseEstimator()
        self.pelvis_pos = np.zeros(3, dtype=np.float32)
        self.pelvis_quat_wxyz = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.delay_time = 0.0
        self._last_summary_time = 0.0

        # Mirrored deployment uses a CPU-only ONNX session.  Keeping its small
        # observation/history tensors on CUDA forces a synchronization and copy
        # every control step, so mirror mode stays entirely on CPU.
        self.device = (
            "cpu"
            if self.mirror_left_hand or self.right_side_canonicalization
            else ("cuda:0" if torch.cuda.is_available() else "cpu")
        )
        self.num_obs = 172
        # A zero-valued reference is only a construction placeholder. reset() refuses
        # to arm until fresh torso and joint streams can produce a valid pelvis pose.
        self.reference = self.reference_scheduler.reset(self.pelvis_pos, self.torso_pos)
        self.motion_index = self.reference.move_motion_index
        self.motion_step = self.reference.reference_step
        self.motion_commands = self.reference.command
        self.action_scale = np.zeros(self.num_actions)
        for i in range(self.num_dofs):
            name = beyond_mimic_config.dof_names[i]
            for dof_name in beyond_mimic_config.armature.keys():
                if dof_name in name:
                    self.action_scale[i] = beyond_mimic_config.action_scale * \
                        beyond_mimic_config.torque_limit[dof_name] / beyond_mimic_config.stiffness[dof_name]

        self.default_dof_pos = np.array([-0.312,  0.000,  0.000,  0.669, -0.363,  0.000, 
                                         -0.312,  0.000,  0.000,  0.669, -0.363,  0.000,
                                         0.0, 0.0, 0.0,
                                         -0.200,  0.200, 0.000, -0.200, 0.00, 0.0, 0.0, 
                                         -0.200, -0.200, 0.000, -0.200, 0.00, 0.0, 0.0], dtype=float) 
        self.init_yaw = self.se.euler[2]

        self.from_gym_to_lab = list(FROM_GYM_TO_LAB)
        self.from_lab_to_gym = list(FROM_LAB_TO_GYM)

        self.actions = torch.zeros(1, self.num_actions)

        self.dof_pos = np.zeros(self.num_dofs)
        self.dof_vel = np.zeros(self.num_dofs)
        self.body_angular_vel = np.zeros(3)
        self.joint_pos_target = np.zeros(29) # hardware read 29 dofs

        self.torques = np.zeros(self.num_dofs)


        self.torso_orientation = np.array([1.0, 0.0, 0.0, 1.0, 0.0 ,0.0], dtype=float)
        self._torso_pose_stamp = None
        self._torso_source_stamp_s = 0.0
        self._torso_receive_monotonic_ns = 0
        self._target_source_stamp_s = 0.0
        self._target_receive_monotonic_ns = 0
        self._velocity_source_stamp_s = 0.0
        self._velocity_receive_monotonic_ns = 0
        self._torso_sub = None
        self._runtime_motion_config_lock = threading.Lock()
        self._runtime_motion_config_inbox = None
        self._runtime_motion_config_received_id = None
        self._runtime_motion_config_error = ""
        self._runtime_motion_config_sub = None
        self._runtime_motion_config_status_pub = None
        self._last_runtime_motion_status_publish = 0.0
        self.planner_bridge = None
        self._init_ros_subscription()
        self._init_runtime_motion_config_transport()
        if self.external_planner:
            if self.robot_id not in {"table_left", "table_right"}:
                raise ValueError("external planner mode requires table_left/table_right robot_id")
            self.planner_bridge = PlannerRosBridge(
                self.robot_id,
                command_timeout_s=command_timeout_s,
                external_hit_command_mode=external_hit_command_mode,
            )

    def close_csv_logger(self):
        pass

    def _validate_pose_sources(self):
        failures = []
        if int(self._torso_receive_monotonic_ns) <= 0:
            failures.append("torso pose has not been received")
        if int(self.se.last_body_receive_monotonic_ns) <= 0:
            failures.append("joint state has not been received")
        if failures:
            raise RuntimeError("Cannot build pelvis-aligned observation: " + "; ".join(failures))

    def _update_robot_state(self, require_fresh=True):
        if require_fresh:
            self._validate_pose_sources()
        self.dof_pos = self.se.get_dof_pos()
        self.dof_vel = self.se.get_dof_vel()
        self.body_angular_vel = self.se.get_body_angular_vel()
        self.pelvis_pos, self.pelvis_quat_wxyz = self.pelvis_pose_estimator.estimate(
            self.torso_pos,
            self.quat_mocap,
            self.dof_pos,
        )
        pelvis_quat_xyzw = self.pelvis_quat_wxyz[[1, 2, 3, 0]]
        self.pelvis_rpy = quaternion_to_rpy(pelvis_quat_xyzw)
        self.pelvis_angular_velocity_w = (
            matrix_from_quat_wxyz(self.pelvis_quat_wxyz) @ self.body_angular_vel
        )

    def get_obs(self, refresh_state=True):
        if refresh_state:
            self._update_robot_state()

        roll, pitch, yaw = self.se.euler
        quat_mocap_xyzw = np.array(
            [self.quat_mocap[1], self.quat_mocap[2], self.quat_mocap[3], self.quat_mocap[0]]
        )
        rpy_mocap = quaternion_to_rpy(quat_mocap_xyzw)
        yaw_mocap = rpy_mocap[2]
        yaw -= self.init_yaw
        quat_norm = quat_from_rpy_wxyz(roll, pitch, yaw_mocap)
        orientation_matrix = matrix_from_quat_wxyz(quat_norm)
        self.torso_rotation = orientation_matrix
        self.torso_orientation = orientation_matrix[:, :2].reshape(-1)
        action_lab = self.actions[:, self.from_gym_to_lab].detach().cpu().numpy()
        joint_pos_lab = self.dof_pos[self.from_gym_to_lab] - self.default_dof_pos[self.from_gym_to_lab]
        joint_vel_lab = self.dof_vel[self.from_gym_to_lab]
        task_anchor = np.array(
            [
                self.reference_scheduler.canonical_pelvis_x(),
                self.pelvis_pos[1],
                self.torso_pos[2],
            ],
            dtype=np.float32,
        )
        ob = np.concatenate(
            (
                np.array([[self.reference.strike_time]], dtype=np.float32),
                self.reference.target_velocity.reshape(1, 3),
                self.reference.racket_target.reshape(1, 3),
                self.reference.target_base.reshape(1, 2),
                task_anchor.reshape(1, 3),
                self.torso_orientation.reshape(1, 6),
                self.body_angular_vel.reshape(1, 3),
                joint_pos_lab.reshape(1, 29),
                joint_vel_lab.reshape(1, 29),
                action_lab.reshape(1, 29),
                self.reference.command.reshape(1, 58),
                self.reference.phase.reshape(1, 6),
            ),
            axis=1,
        )
        if ob.shape != (1, self.num_obs):
            raise RuntimeError(f"Current student observation has shape {ob.shape}, expected (1, {self.num_obs}).")
        if not np.isfinite(ob).all():
            raise RuntimeError("Current student observation contains non-finite values.")

        now = time.time()
        if now - self._last_summary_time >= 1.0:
            print(
                f"[DEPLOY] phase={self.reference.state.name} tts={self.reference.strike_time:+.3f} "
                f"pelvis_y={self.pelvis_pos[1]:+.3f} torso_y={self.torso_pos[1]:+.3f} "
                f"target_base_y={self.reference.target_base[1]:+.3f} "
                f"hit_motion={self.reference.hit_motion_index} move_motion={self.reference.move_motion_index} "
                f"ref_step={self.reference.reference_step} shadow={self.shadow} "
                f"mirror_left_hand={self.mirror_left_hand}"
            )
            self._last_summary_time = now
        return torch.tensor(ob, device=self.device, dtype=torch.float32)

    def get_teacher_frame(self, _student_obs=None):
        """Build the frozen HIT Teacher's exact 167-D current-frame contract."""
        reference = self.reference
        motion_anchor_pos_b = self.torso_rotation.T @ (
            np.asarray(reference.hit_anchor_pos_w, dtype=np.float32) - self.torso_pos
        )
        action_lab = self.actions[:, self.from_gym_to_lab].detach().cpu().numpy()
        joint_pos_lab = self.dof_pos[self.from_gym_to_lab] - self.default_dof_pos[self.from_gym_to_lab]
        joint_vel_lab = self.dof_vel[self.from_gym_to_lab]
        hit_command = np.concatenate((reference.hit_joint_pos, reference.hit_joint_vel))
        frame = np.concatenate(
            (
                np.array([[reference.strike_time]], dtype=np.float32),
                reference.target_velocity.reshape(1, 3),
                reference.racket_target.reshape(1, 3),
                motion_anchor_pos_b.reshape(1, 3),
                self.torso_pos.reshape(1, 3),
                self.torso_orientation.reshape(1, 6),
                hit_command.reshape(1, 58),
                self.body_angular_vel.reshape(1, 3),
                joint_pos_lab.reshape(1, 29),
                joint_vel_lab.reshape(1, 29),
                action_lab.reshape(1, 29),
            ),
            axis=1,
        ).astype(np.float32, copy=False)
        if frame.shape != (1, 167) or not np.isfinite(frame).all():
            raise RuntimeError(f"Invalid HIT Teacher frame: shape={frame.shape}.")
        return torch.tensor(frame, device=self.device, dtype=torch.float32)

    def _init_ros_subscription(self):
        """Subscribe to the active single-robot ChingMu planner topics."""
        if rospy is None:
            return
        if self._torso_sub is not None:
            return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node('lcm_agent', anonymous=True)
        except Exception:
            # If initialization fails (e.g., multiple nodes in same process), skip subscription.
            return
        self._torso_sub = rospy.Subscriber(
            self.torso_topic,
            PoseStamped,
            self._torso_pose_callback,
            queue_size=1,
        )
        if self.external_planner:
            return
        # predicted position is published as PointStamped (point), not PoseStamped
        self._ractarget_sub = rospy.Subscriber(
            PREDICTED_BALL_POSITION_TOPIC,
            PointStamped,
            self._ractarget_callback,
            queue_size=1,
        )


        self._ballvel_sub = rospy.Subscriber(
            PREDICTED_RACKET_VELOCITY_TOPIC,
            TwistStamped,
            self._ballvel_callback,
            queue_size=1,
        )


        # predicted time is published as PointStamped (point.x stores predict_time)
        self._time_to_strike_sub = rospy.Subscriber(
            PREDICTED_BALL_PREDICT_TIME_TOPIC,
            PointStamped,
            self._striketime_callback,
            queue_size=1,
        )

    def _init_runtime_motion_config_transport(self):
        if (
            rospy is None
            or String is None
            or self.external_planner
            or self.robot_id != "table_right"
        ):
            return
        self._runtime_motion_config_status_pub = rospy.Publisher(
            "/doubles/table_right/runtime_motion_config_status",
            String,
            queue_size=1,
            latch=True,
        )
        self._runtime_motion_config_sub = rospy.Subscriber(
            "/doubles/table_right/runtime_motion_config",
            String,
            self._runtime_motion_config_callback,
            queue_size=1,
        )

    def _runtime_motion_config_callback(self, message):
        try:
            config = parse_runtime_motion_config(
                message.data,
                expected_robot_id="table_right",
            )
        except Exception as exc:
            with self._runtime_motion_config_lock:
                self._runtime_motion_config_error = str(exc)
            return
        with self._runtime_motion_config_lock:
            self._runtime_motion_config_inbox = config
            self._runtime_motion_config_received_id = config.command_id
            self._runtime_motion_config_error = ""

    def _apply_pending_runtime_motion_config(self):
        lock = getattr(self, "_runtime_motion_config_lock", None)
        if lock is None:
            return
        with lock:
            config = self._runtime_motion_config_inbox
            self._runtime_motion_config_inbox = None
        if config is None:
            return
        try:
            self.reference_scheduler.stage_runtime_motion_config(
                command_id=config.command_id,
                goal_x=config.goal_x,
                outward_y=config.outward_y,
                home_y=config.home_y,
                outward_hold_s=config.outward_hold_s,
            )
        except Exception as exc:
            with self._runtime_motion_config_lock:
                self._runtime_motion_config_error = str(exc)

    def _publish_runtime_motion_config_status(self, force=False):
        publisher = getattr(self, "_runtime_motion_config_status_pub", None)
        if publisher is None:
            return
        now = time.monotonic()
        if not force and now - self._last_runtime_motion_status_publish < 0.2:
            return
        self._last_runtime_motion_status_publish = now
        with self._runtime_motion_config_lock:
            received_id = self._runtime_motion_config_received_id
            error = self._runtime_motion_config_error
        status = self.reference_scheduler.runtime_motion_config_status()
        payload = {
            "schema": "v10-runtime-motion-config-status-v1",
            "robot_id": "table_right",
            "published_at": time.time(),
            "phase": self.reference.state.name,
            "received_command_id": received_id,
            "error": error,
            **status,
        }
        publisher.publish(
            String(data=json.dumps(payload, allow_nan=False, separators=(",", ":")))
        )

    def _torso_pose_callback(self, msg: PoseStamped):
        """ROS callback updating torso pose from mocap stream."""
        self._torso_receive_monotonic_ns = time.monotonic_ns()
        self.torso_pos[:] = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            float(msg.pose.position.z),
        )
        self.quat_mocap = np.array(
            [
                msg.pose.orientation.w,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
            ],
            dtype=float,
        )
    
        if rospy is not None:
            self._torso_pose_stamp = msg.header.stamp
            self._torso_source_stamp_s = float(msg.header.stamp.to_sec())

    def _ractarget_callback(self, msg: PointStamped):
        """ROS callback: update `ractarget` from `PointStamped` publisher."""
        self._target_receive_monotonic_ns = time.monotonic_ns()
        self._target_source_stamp_s = float(msg.header.stamp.to_sec())
        self.recvtarget[:] = (
            np.clip(float(msg.point.x), self.target_range[0, 0],self.target_range[0, 1]),
            np.clip(float(msg.point.y), self.target_range[1, 0],self.target_range[1, 1]),
            np.clip(float(msg.point.z), self.target_range[2, 0],self.target_range[2, 1]),
        )

    def _ballvel_callback(self, msg: TwistStamped):
        """ROS callback: update `ballvel` from `TwistStamped` publisher."""
        self._velocity_receive_monotonic_ns = time.monotonic_ns()
        self._velocity_source_stamp_s = float(msg.header.stamp.to_sec())
        self.recvvel[:] = (
            np.clip(float(msg.twist.linear.x),  0.0, 6.0),
            np.clip(float(msg.twist.linear.y), -2.0, 2.0),
            np.clip(float(msg.twist.linear.z), -2.0, 2.0),
        )

    def _striketime_callback(self, msg: PointStamped):
        """ROS callback: update `strike_time` from `PointStamped` publisher (point.x stores predict_time)."""
        self.handle_time = float(msg.point.x)
        self.handle_timestep = msg.header.stamp
        self.handle_receive_walltime = time.time()
        self._tts_receive_monotonic_ns = time.monotonic_ns()
        self._tts_source_stamp_s = float(msg.header.stamp.to_sec())

    def _corrected_time_to_strike(self):
        if self.handle_timestep is None:
            return self.time_range[0]
        delay = None
        if rospy is not None:
            try:
                delay = max(0.0, (rospy.Time.now() - self.handle_timestep).to_sec())
            except Exception:
                delay = None
        if delay is None and self.handle_receive_walltime is not None:
            delay = max(0.0, time.time() - self.handle_receive_walltime)
        self.delay_time = 0.0 if delay is None else delay
        return float(np.clip(self.handle_time - self.delay_time, self.time_range[0], self.time_range[1]))

    def publish_action(self, hard_reset=False):
        action_use = self.actions.reshape(-1).cpu().numpy()
        scaled_pos_target_action = action_use * self.action_scale + self.default_dof_pos[:]  
        self.joint_pos_target = scaled_pos_target_action
        if self.shadow:
            return
        command_for_robot = pd_tau_targets_lcmt()
        command_for_robot.q_des = self.joint_pos_target
        self._command_seq += 1
        self.last_command_seq = self._command_seq
        self.last_command_publish_monotonic_raw_ns = monotonic_raw_ns()
        # This field was previously unused by g1_control.  It now carries the
        # command sequence without changing the production LCM schema.
        command_for_robot.timestamp_us = self.last_command_seq
        self.se.lc.publish("pd_plustau_targets", command_for_robot.encode())

    def append_command_timing(self, snapshot):
        """Attach timestamps captured after the matching q_des was published."""
        snapshot.update(
            {
                "command_seq": int(self.last_command_seq),
                "command_publish_monotonic_raw_ns": int(
                    self.last_command_publish_monotonic_raw_ns
                ),
                "trace_command_seq": int(self.se.trace_command_seq),
                "command_lcm_receive_monotonic_raw_ns": int(
                    self.se.command_lcm_receive_monotonic_raw_ns
                ),
                "command_control_monotonic_raw_ns": int(
                    self.se.command_control_monotonic_raw_ns
                ),
                "command_dds_write_monotonic_raw_ns": int(
                    self.se.command_dds_write_monotonic_raw_ns
                ),
                "body_source_monotonic_raw_ns": int(
                    self.se.body_source_monotonic_raw_ns
                ),
                "body_receive_monotonic_raw_ns": int(
                    self.se.last_body_receive_monotonic_raw_ns
                ),
                "imu_source_monotonic_raw_ns": int(
                    self.se.imu_source_monotonic_raw_ns
                ),
                "imu_receive_monotonic_raw_ns": int(
                    self.se.last_imu_receive_monotonic_raw_ns
                ),
            }
        )

    @staticmethod
    def _record_array(value, width):
        if torch.is_tensor(value):
            value = value.detach().reshape(-1).cpu().numpy()
        return np.asarray(value, dtype=np.float32).reshape(width).copy()

    def make_record_snapshot(
        self,
        obs,
        obs_history,
        action_lab,
        action_gym,
        loop_period_s,
        inference_s,
        policy_obs_history=None,
        canonical_action_lab=None,
    ):
        """Capture the exact policy input and matching command without performing I/O."""
        action_lab_np = self._record_array(action_lab, 29)
        action_gym_np = self._record_array(action_gym, 29)
        policy_obs_history_np = self._record_array(
            obs_history if policy_obs_history is None else policy_obs_history,
            1666,
        )
        canonical_action_lab_np = self._record_array(
            action_lab if canonical_action_lab is None else canonical_action_lab,
            29,
        )
        q_des = action_gym_np * self.action_scale.astype(np.float32) + self.default_dof_pos.astype(np.float32)
        reference = self.reference
        torso_stamp = float(self._torso_source_stamp_s)
        target_stamp = float(self._target_source_stamp_s)
        velocity_stamp = float(self._velocity_source_stamp_s)
        tts_stamp = float(self._tts_source_stamp_s)
        monotonic_time_ns = time.monotonic_ns()
        snapshot = {
            "wall_time_ns": time.time_ns(),
            "monotonic_time_ns": monotonic_time_ns,
            "loop_period_s": float(loop_period_s),
            "inference_s": float(inference_s),
            "shadow": int(self.shadow),
            "mirror_left_hand": int(self.mirror_left_hand),
            "racket_hand": 0 if self.racket_hand == "right" else 1,
            "right_side_canonicalization": int(self.right_side_canonicalization),
            "arm_transform_applied": int(self.mirror_left_hand),
            "table_right_move_mode": (
                2
                if self.native_no_mirror
                else (1 if self.right_side_canonicalization else 0)
            ),
            "observation_transform_applied": int(
                self.mirror_left_hand or self.right_side_canonicalization
            ),
            "action_transform_applied": int(
                self.mirror_left_hand or self.right_side_canonicalization
            ),
            "phase_id": int(reference.state),
            "phase": np.asarray(reference.phase, dtype=np.float32).copy(),
            "outward_hold_elapsed_s": float(reference.outward_hold_elapsed_s),
            "outward_hold_duration_s": float(reference.outward_hold_duration_s),
            "outward_hold_entry_reason": reference.outward_hold_entry_reason.encode("utf-8")[:16],
            "raw_tts": float(self.handle_time),
            "corrected_tts": float(reference.strike_time),
            "torso_pos": np.asarray(self.torso_pos, dtype=np.float32).copy(),
            "torso_quat_wxyz": np.asarray(self.quat_mocap, dtype=np.float32).copy(),
            "pelvis_pos": np.asarray(self.pelvis_pos, dtype=np.float32).copy(),
            "pelvis_quat_wxyz": np.asarray(self.pelvis_quat_wxyz, dtype=np.float32).copy(),
            "torso_minus_pelvis": np.asarray(
                self.torso_pos - self.pelvis_pos, dtype=np.float32
            ).copy(),
            "waist_joint_pos": np.asarray(self.dof_pos[12:15], dtype=np.float32).copy(),
            "torso_source_stamp_s": torso_stamp,
            "torso_receive_monotonic_ns": int(self._torso_receive_monotonic_ns),
            "target_source_stamp_s": target_stamp,
            "target_receive_monotonic_ns": int(self._target_receive_monotonic_ns),
            "velocity_source_stamp_s": velocity_stamp,
            "velocity_receive_monotonic_ns": int(self._velocity_receive_monotonic_ns),
            "tts_source_stamp_s": tts_stamp,
            "tts_receive_monotonic_ns": int(self._tts_receive_monotonic_ns),
            "joint_pos": np.asarray(self.dof_pos, dtype=np.float32).copy(),
            "joint_vel": np.asarray(self.dof_vel, dtype=np.float32).copy(),
            "body_rpy": np.asarray(self.se.euler, dtype=np.float32).copy(),
            "body_quat": np.asarray(self.se.body_quat, dtype=np.float32).copy(),
            "body_angular_vel": np.asarray(self.body_angular_vel, dtype=np.float32).copy(),
            "body_receive_monotonic_ns": int(self.se.last_body_receive_monotonic_ns),
            "imu_receive_monotonic_ns": int(self.se.last_imu_receive_monotonic_ns),
            "obs": self._record_array(obs, 172),
            "obs_history": self._record_array(obs_history, 1666),
            "policy_obs_history": policy_obs_history_np,
            "reference_joint_pos": np.asarray(reference.joint_pos, dtype=np.float32).copy(),
            "reference_joint_vel": np.asarray(reference.joint_vel, dtype=np.float32).copy(),
            "canonical_action_lab": canonical_action_lab_np,
            "action_lab": action_lab_np,
            "action_gym": action_gym_np,
            "q_des": q_des,
            "target_base": np.asarray(reference.target_base, dtype=np.float32).copy(),
            "physical_target_base": np.asarray(
                reference.target_base, dtype=np.float32
            ).copy(),
            "policy_target_base": np.asarray(
                [policy_obs_history_np[88], policy_obs_history_np[89]],
                dtype=np.float32,
            ),
            "episode_start_x_override": (
                np.nan
                if self.episode_start_x_override is None
                else self.episode_start_x_override
            ),
            "episode_start_x": float(self.reference_scheduler.episode_start_x),
            "target_base_x": float(reference.target_base[0]),
            "hit_motion_index": int(reference.hit_motion_index),
            "move_motion_index": int(reference.move_motion_index),
            "move_motion_source_label": int(
                self.move_motions.labels[reference.move_motion_index]
            ),
            "move_semantic_phase": int(reference.state),
            "reference_step": int(reference.reference_step),
            "home_y": float(self.reference_scheduler.home_y),
            "outward_target_y": float(self.reference_scheduler.outward_target_y),
        }
        if self.planner_bridge is not None:
            snapshot.update(self.planner_bridge.record_snapshot_fields())
        raw_ball_source = getattr(self, "raw_ball_source", None)
        if raw_ball_source is not None:
            ball = raw_ball_source.sample(now_monotonic_ns=monotonic_time_ns)
            snapshot.update(
                {
                    "raw_ball_valid": int(ball["valid"]),
                    "raw_ball_reason": str(ball["reason"])[:24].encode(),
                    "raw_ball_source_time_s": float(ball["source_time_s"]),
                    "raw_ball_frame": int(ball["frame"]),
                    "raw_ball_rigid_body_id": int(ball["rigid_body_id"]),
                    "raw_ball_position": np.asarray(ball["position"], dtype=np.float32),
                    "raw_ball_velocity": np.asarray(ball["velocity"], dtype=np.float32),
                    "raw_ball_age_ms": float(ball["age_ms"]),
                    "raw_ball_receive_monotonic_ns": int(ball["receive_monotonic_ns"]),
                    "raw_ball_sample_count": int(ball["sample_count"]),
                }
            )
        return snapshot

    def reset(self):
        self.actions = torch.zeros(1, 29)
        self.timestep = 0
        self._update_robot_state()
        self.init_yaw = self.se.euler[2]
        self.reference = self.reference_scheduler.reset(self.pelvis_pos, self.torso_pos)
        self._apply_pending_runtime_motion_config()
        self._apply_startup_scheduler_state()
        self.reference = self.reference_scheduler.output()
        self.strike_time = self.reference.strike_time
        self.motion_index = self.reference.move_motion_index
        self.motion_step = self.reference.reference_step
        self.motion_commands = self.reference.command
        return self.get_obs(refresh_state=False)

    def _apply_startup_scheduler_state(self):
        if getattr(self, "stationary_hit_test", "none") == "hit_home":
            return
        if self.planner_home_y is not None and not self.startup_home_current:
            self.reference_scheduler.set_external_home_target(self.planner_home_y)
        if self.bootstrap_outward_hold:
            if self.planner_home_y is None:
                raise RuntimeError("OUTWARD_HOLD bootstrap requires planner_home_y")
            self.reference_scheduler.bootstrap_outward_hold(
                self.planner_home_y,
                outward_y=float(self.pelvis_pos[1]),
            )

    def apply_action(self, actions, hard_reset=False):
        self.actions = torch.clip(actions, -self.action_clip, self.action_clip)
        if not torch.isfinite(self.actions).all():
            raise RuntimeError("Policy action contains non-finite values.")
        self.publish_action(hard_reset=hard_reset)

    def observe(self):
        self._update_robot_state(require_fresh=False)
        self._apply_pending_runtime_motion_config()
        # Command validation/selection and this tick's reference use one pose snapshot.
        pelvis_position = self.pelvis_pos.copy()
        torso_position = self.torso_pos.copy()
        self.reference_scheduler.sync_robot_pose(pelvis_position, torso_position)
        planner_hit_update = None
        if self.planner_bridge is not None:
            planner_hit_update = self.planner_bridge.apply_pending(
                self.reference_scheduler, pelvis_position
            )
        if (
            planner_hit_update is not None
            and (
                planner_hit_update.starts_hit
                or self.reference.state == DoublesPhase.HIT
            )
        ):
            corrected_tts = float(planner_hit_update.streamed_tts)
            self.recvtarget[:] = planner_hit_update.racket_target
            self.recvvel[:] = planner_hit_update.target_velocity
            self.handle_time = float(planner_hit_update.raw_tts)
            self.delay_time = float(planner_hit_update.prediction_age_s)
        else:
            corrected_tts = self._corrected_time_to_strike()
        previous_state = self.reference.state
        self.reference = self.reference_scheduler.update(
            corrected_tts,
            self.recvtarget,
            self.recvvel,
            pelvis_position,
            torso_position,
            self.pelvis_rpy,
            self.pelvis_angular_velocity_w,
            dt=self.dt,
            start_external_hit=bool(
                planner_hit_update is not None and planner_hit_update.starts_hit
            ),
        )
        if planner_hit_update is not None and planner_hit_update.starts_hit:
            self.reference_scheduler.set_external_outward_target(
                planner_hit_update.post_hit_outward_y
            )
            if planner_hit_update.return_target_y is not None:
                self.reference_scheduler.set_external_home_target(
                    planner_hit_update.return_target_y
                )
            self.reference = self.reference_scheduler.output()
        if self.reference.state != previous_state:
            print(
                f"[STATE] {previous_state.name}->{self.reference.state.name} "
                f"tts={self.reference.strike_time:+.3f} pelvis_y={self.pelvis_pos[1]:+.3f} "
                f"torso_y={self.torso_pos[1]:+.3f} "
                f"target_base_y={self.reference.target_base[1]:+.3f}"
            )
        self._publish_runtime_motion_config_status(
            force=self.reference.state != previous_state
        )
        self.strike_time = self.reference.strike_time
        self.ractarget = self.reference.racket_target
        self.ballvel = self.reference.target_velocity
        self.motion_index = (
            self.reference.hit_motion_index
            if self.reference.state.value <= 1
            else self.reference.move_motion_index
        )
        self.motion_step = self.reference.reference_step
        self.motion_commands = self.reference.command
        # Build target_base and task_anchor from the same 50 Hz pelvis snapshot.
        obs = self.get_obs(refresh_state=False)
        if self.planner_bridge is not None:
            self.planner_bridge.publish_state(self)
        self.timestep += 1
        return obs

    def step(self, actions, hard_reset=False):
        self.apply_action(actions, hard_reset=hard_reset)
        return self.observe()
