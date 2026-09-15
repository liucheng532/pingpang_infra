import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path

DEPLOY_PYTHON_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_PYTHON_ROOT))

import lcm
import numpy as np
import onnx
import onnxruntime as ort
import torch
import rospy

from envs.history_wrapper import HistoryWrapper
from envs.lcm_agent import LCMAgent
from utils.cheetah_state_estimator import StateEstimator
from utils.command_profile import RCControllerProfile
from utils.async_deploy_recorder import AsyncDeployRecorder
from utils.deployment_runner import DeploymentRunner
from utils.doubles_reference_scheduler import PHASE_ORDER, V10_TARGET_BASE_X_CLIP
from utils.hit_teacher_override_policy import (
    TeacherHitOverridePolicy,
    TEACHER_OBS_DIM,
)
from utils.joint_mapping import LAB_JOINT_NAMES
from utils.left_right_mirror import (
    MirroredStudentPolicy,
    MirroredTeacherPolicy,
    joint_mirror_spec,
)
from utils.v01_hit_residual_adapter import V01HitResidualAdapter
from utils.v11_teacher_residual_adapter import V11TeacherResidualAdapter
from utils.raw_ball_diagnostics import BALL_RIGID_BODY_ID, RAW_BALL_TOPIC, RawBallStateSource
from utils.right_side_right_hand import RightSideRightHandStudentPolicy


DEFAULT_DEPLOY_ROOT = str(Path(__file__).resolve().parents[2])
DEFAULT_POLICY = (
    f"{DEFAULT_DEPLOY_ROOT}/policy/v9_model19000/"
    "student_v9_m14500_timedhandoff_1666_model19000.onnx"
)
DEFAULT_HIT_MOTIONS = f"{DEFAULT_DEPLOY_ROOT}/data/0302_combined"
DEFAULT_MOVE_MOTIONS = f"{DEFAULT_DEPLOY_ROOT}/data/0718-move-160-80hz"
DEFAULT_HIT_TEACHER = "/home/unitree/haoran/sports-mimic-dev-camera/policy/stable/0303-best.onnx"
DEFAULT_V01_RUNTIME_MODULE = (
    "/home/unitree/haoran/v01-arm7-residual-deploy-safe-20260810-50hz/"
    "g1_gym_deploy/utils/residual_policy.py"
)
DEFAULT_V01_RESIDUAL = (
    "/home/unitree/haoran/v01-arm7-residual-deploy-safe-20260810-50hz/"
    "policy/v01_arm7_i9500.onnx"
)
DEFAULT_V01_CHECKPOINT = (
    "/home/unitree/haoran/v01-arm7-residual-deploy-safe-20260810-50hz/"
    "policy/provenance/v01_arm7_i9500.pt"
)
DEFAULT_V11_RESIDUAL = f"{DEFAULT_DEPLOY_ROOT}/policy/v11_teacher_arm7_i15000/v11_teacher_arm7_i15000_hard1.onnx"
DEFAULT_V11_RESIDUAL_CHECKPOINT = f"{DEFAULT_DEPLOY_ROOT}/policy/v11_teacher_arm7_i15000/model_15000.pt"
EXPECTED_CHECKPOINT_SHA256 = "f2d31b802a37c014971bf83a0b9d1a61e2b8727b42ffbed3dd153b7e4d67127d"
EXPECTED_HIT_TEACHER_SHA256 = "5a4e650813f227f704ae2040df73ece8da11dae1746bfc8b6e982c68e9efc94c"
EXPECTED_MOVE_TEACHER_SHA256 = "9255177c1ae6eced0a88547ea0d362cced22b3fbda2bd044221f43846345f260"
V9_CHECKPOINT_SHA256 = "2c70e278b3d5d4c96e26c20b3d7a6c4943c486c5c30c599b6a25b04ba358eb8a"
V9_MOVE_TEACHER_SHA256 = "8716d2e5865674f210e1261a18c5ed2b370b7de2ce641e6f10387f416ae11296"
V10_CHECKPOINT_SHA256 = "8e186759e1151027f071636a623c7336bbede5b8c59ac8cbc257af12ebc6b97f"
V10_MOVE_TEACHER_SHA256 = "3e7966dbea8f5c95418f6434bedaf7aaaa46a31112f8b1d03cc8a98266e12774"
V10_REFEND_CHECKPOINT_SHA256 = "46ba6e220b9f8d9c61eba951e1fcf02c6766988ccfbbac2c4f937d502efbd79f"
V10_REFEND_MOVE_TEACHER_SHA256 = "e6d63869d97bd7ad4656a26f5a97f4c0e545b8ae9a52f5b1a25ec6f538d45bcb"
V10_REFEND_MOVE_TEACHER_CONTRACT = "v10_xrecovery_refend_latchedhold_cleanpool8_model30000"
V10_REFEND_MOVE_TEACHER_SOURCE_COMMIT = "2120851064ba25f57803d4d7c159f75a1b2fd218"
V10_REFEND_ACTION_CONTRACT = {
    "clip": 10.0,
    "teacher_target": "clipped_before_behavior_cloning",
    "student_prediction": "raw_with_loss_gradient",
    "rollout": "clipped_after_teacher_student_blend",
}
V11_CHECKPOINT_SHA256 = "04461633b8ad75a2b173ceacf34257fae437cbf935f6effe8b279464586bea12"
V11_MOVE_TEACHER_SHA256 = "5bcf39a8a27b4dd831068711b5a7b823867494087dedb6f1312d37da8e7ae2af"
V11_MOVE_TEACHER_CONTRACT = "v11_commonhold_fkupright_rawwrist_r2_model299"
V11_STUDENT_SOURCE_COMMIT = "2efd2c28ee1a021b574bc604aa20d5d76bba24f7"
V11_MOVE_TEACHER_SOURCE_COMMIT = "190ec6bd1d56de123924a1ffcdab1e6a8a10c79a"
V11_RESUME_SOURCE_SHA256 = "10f5ba7fb24d354256d2952b283180eed89c7baf42f31bc96f1359098f1cd76d"
V11_COMMON_HOLD_SHA256 = "9565a8ed1ba22cfc0d759d0a8327dc7dd37989c242d4f4337dcc79c796ea3a8f"
V11_EXCLUDED_MOVE_SOURCE_IDS = [41, 47, 69, 78, 118, 119, 138, 139]
EXPECTED_MOVE_TEACHER_BY_CHECKPOINT = {
    EXPECTED_CHECKPOINT_SHA256: EXPECTED_MOVE_TEACHER_SHA256,
    V9_CHECKPOINT_SHA256: V9_MOVE_TEACHER_SHA256,
    V10_CHECKPOINT_SHA256: V10_MOVE_TEACHER_SHA256,
    V10_REFEND_CHECKPOINT_SHA256: V10_REFEND_MOVE_TEACHER_SHA256,
    V11_CHECKPOINT_SHA256: V11_MOVE_TEACHER_SHA256,
}
EXPECTED_HIT_MANIFEST_SHA256 = "157bfe81e828f42661e3414a720791ef1968b8488841d5d354dcdc40d7726f03"
EXPECTED_MOVE_MANIFEST_SHA256 = "2684d5cfb1a4b9d10a4eb8df3b4a945b20662fcc06bdde599bd32da393a4e90d"

def _last_static_dimension(shape, name):
    if not shape:
        raise ValueError(f"ONNX {name} has no shape information.")
    dimension = shape[-1]
    if isinstance(dimension, (int, np.integer)):
        return int(dimension)
    raise ValueError(f"ONNX {name} last dimension must be static, got {shape}.")


def validate_student_io(input_shape, output_shape):
    input_dim = _last_static_dimension(input_shape, "input")
    output_dim = _last_static_dimension(output_shape, "output")
    if input_dim != 1666:
        raise ValueError(f"Student ONNX input must be 1666, got {input_dim}.")
    if output_dim != 29:
        raise ValueError(f"Student ONNX output must be 29, got {output_dim}.")


def _load_sidecar(policy_path: Path):
    sidecar_path = Path(f"{policy_path}.json")
    if not sidecar_path.is_file():
        raise FileNotFoundError(f"Required student metadata sidecar not found: {sidecar_path}")
    with sidecar_path.open("r", encoding="utf-8") as stream:
        metadata = json.load(stream)
    history_length = metadata.get("history_length", metadata.get("obs_history_length"))
    if history_length != 10:
        raise ValueError(f"{sidecar_path} history_length must be 10, got {history_length!r}.")
    phase_order = tuple(metadata.get("phase_order", ()))
    if phase_order != PHASE_ORDER:
        raise ValueError(f"{sidecar_path} phase_order must be {PHASE_ORDER}, got {phase_order}.")
    if "input_dim" in metadata and int(metadata["input_dim"]) != 1666:
        raise ValueError(f"{sidecar_path} input_dim must be 1666.")
    if "output_dim" in metadata and int(metadata["output_dim"]) != 29:
        raise ValueError(f"{sidecar_path} output_dim must be 29.")
    checkpoint_sha256 = metadata.get("checkpoint_sha256")
    if checkpoint_sha256 not in EXPECTED_MOVE_TEACHER_BY_CHECKPOINT:
        raise ValueError(
            f"{sidecar_path} checkpoint_sha256 is not an approved deployment checkpoint: "
            f"{checkpoint_sha256!r}."
        )
    expected_hashes = {
        "hit_teacher_sha256": EXPECTED_HIT_TEACHER_SHA256,
        "move_teacher_sha256": EXPECTED_MOVE_TEACHER_BY_CHECKPOINT[checkpoint_sha256],
        "hit_motion_manifest_sha256": EXPECTED_HIT_MANIFEST_SHA256,
        "move_motion_manifest_sha256": EXPECTED_MOVE_MANIFEST_SHA256,
    }
    for name, expected in expected_hashes.items():
        if metadata.get(name) != expected:
            raise ValueError(f"{sidecar_path} {name} must be {expected}, got {metadata.get(name)!r}.")
    if checkpoint_sha256 == V10_REFEND_CHECKPOINT_SHA256:
        expected_contracts = {
            "move_teacher_contract": V10_REFEND_MOVE_TEACHER_CONTRACT,
            "move_teacher_source_commit": V10_REFEND_MOVE_TEACHER_SOURCE_COMMIT,
            "teacher_transition_contract": "reference_end_outward_return_v1",
            "distill_transition_contract": "v9_stable_or_reference_end_v1",
            "teacher_hold_x_contract": "latch_phase_entry_then_feedback",
            "distill_hold_x_contract": "latch_phase_entry_then_feedback",
            "action_contract": V10_REFEND_ACTION_CONTRACT,
            "deployment_x_contract": {
                "kind": "startup_relative_safe_band_v1",
                "canonical_center_m": 0.18,
                "relative_half_width_m": 0.04,
                "target_base_x_clip_m": 0.07,
            },
        }
        for name, expected in expected_contracts.items():
            if metadata.get(name) != expected:
                raise ValueError(f"{sidecar_path} has the wrong {name} contract.")
        x_recovery = metadata.get("x_recovery_contract") or {}
        if x_recovery.get("enabled") is not True or x_recovery.get("safe_x_range") != [0.14, 0.22]:
            raise ValueError(f"{sidecar_path} has the wrong X-recovery contract.")
        move_pool = metadata.get("move_pool_contract") or {}
        if (
            move_pool.get("manifest_sha256") != EXPECTED_MOVE_MANIFEST_SHA256
            or move_pool.get("excluded_source_ids") != [41, 47, 69, 78, 118, 119, 138, 139]
            or move_pool.get("exclusion_sha256")
            != "82045bd01817022444bb2cde0f391123c840d197f8d15ae7b6e35793dbb17005"
        ):
            raise ValueError(f"{sidecar_path} has the wrong cleanpool147 contract.")
        if metadata.get("numerical_isolation_contract") != {
            "kind": "per_environment_reset_v1",
            "joint_position_margin_rad": 1.0,
            "joint_velocity_abs_limit_rad_s": 100.0,
            "root_linear_speed_limit_m_s": 20.0,
            "root_angular_speed_limit_rad_s": 50.0,
        }:
            raise ValueError(f"{sidecar_path} has the wrong training numerical-isolation contract.")
        checkpoint_path = policy_path.parent / str(metadata.get("checkpoint_file"))
        if not checkpoint_path.is_file():
            raise FileNotFoundError(f"Frozen student checkpoint not found: {checkpoint_path}")
        if hashlib.sha256(checkpoint_path.read_bytes()).hexdigest() != checkpoint_sha256:
            raise ValueError(f"{checkpoint_path} SHA256 does not match the sidecar.")
    elif checkpoint_sha256 == V11_CHECKPOINT_SHA256:
        expected_contracts = {
            "student_source_commit": V11_STUDENT_SOURCE_COMMIT,
            "move_teacher_contract": V11_MOVE_TEACHER_CONTRACT,
            "move_teacher_source_commit": V11_MOVE_TEACHER_SOURCE_COMMIT,
            "teacher_transition_contract": "reference_end_outward_return_v1",
            "distill_transition_contract": "v9_stable_or_reference_end_v1",
            "teacher_hold_x_contract": "latch_phase_entry_then_feedback",
            "distill_hold_x_contract": "latch_phase_entry_then_feedback",
            "action_contract": V10_REFEND_ACTION_CONTRACT,
            "deployment_x_contract": {
                "kind": "startup_relative_safe_band_v1",
                "canonical_center_m": 0.18,
                "relative_half_width_m": 0.04,
                "target_base_x_clip_m": 0.04,
            },
        }
        for name, expected in expected_contracts.items():
            if metadata.get(name) != expected:
                raise ValueError(f"{sidecar_path} has the wrong V11 {name} contract.")
        common_hold = metadata.get("distill_common_hold_contract") or {}
        if common_hold.get("asset_sha256") != V11_COMMON_HOLD_SHA256:
            raise ValueError(f"{sidecar_path} has the wrong V11 common-HOLD contract.")
        common_hold_path = policy_path.parent / str(metadata.get("common_hold_pose_file"))
        if not common_hold_path.is_file():
            raise FileNotFoundError(f"V11 common-HOLD asset not found: {common_hold_path}")
        if hashlib.sha256(common_hold_path.read_bytes()).hexdigest() != V11_COMMON_HOLD_SHA256:
            raise ValueError(f"{common_hold_path} SHA256 does not match the V11 contract.")
        move_pool = metadata.get("move_pool_contract") or {}
        if (
            move_pool.get("manifest_sha256") != EXPECTED_MOVE_MANIFEST_SHA256
            or move_pool.get("excluded_source_ids") != V11_EXCLUDED_MOVE_SOURCE_IDS
            or move_pool.get("active_count") != 147
        ):
            raise ValueError(f"{sidecar_path} has the wrong V11 cleanpool147 contract.")
        resume_source = metadata.get("resume_source") or {}
        if (
            resume_source.get("iteration") != 8000
            or resume_source.get("sha256") != V11_RESUME_SOURCE_SHA256
            or resume_source.get("mode") != "full_student_optimizer_absolute_iteration"
        ):
            raise ValueError(f"{sidecar_path} has the wrong V11 resume contract.")
        if metadata.get("numerical_isolation_contract") != {
            "kind": "per_environment_reset_v1",
            "joint_position_margin_rad": 1.0,
            "joint_velocity_abs_limit_rad_s": 100.0,
            "root_linear_speed_limit_m_s": 20.0,
            "root_angular_speed_limit_rad_s": 50.0,
        }:
            raise ValueError(f"{sidecar_path} has the wrong V11 training numerical-isolation contract.")
        checkpoint_path = policy_path.parent / str(metadata.get("checkpoint_file"))
        if not checkpoint_path.is_file():
            raise FileNotFoundError(f"Frozen V11 checkpoint not found: {checkpoint_path}")
        if hashlib.sha256(checkpoint_path.read_bytes()).hexdigest() != checkpoint_sha256:
            raise ValueError(f"{checkpoint_path} SHA256 does not match the V11 sidecar.")
    digest = hashlib.sha256(policy_path.read_bytes()).hexdigest()
    if metadata.get("onnx_sha256") != digest:
        raise ValueError(f"{sidecar_path} ONNX SHA256 does not match {policy_path}.")
    return metadata


def _validate_motion_manifest(root, expected_sha256, name):
    manifest = Path(root).expanduser().resolve() / "dataindex.csv"
    if not manifest.is_file():
        raise FileNotFoundError(f"{name} motion manifest not found: {manifest}")
    digest = hashlib.sha256(manifest.read_bytes()).hexdigest()
    if digest != expected_sha256:
        raise ValueError(f"{name} motion manifest SHA256 is {digest}, expected {expected_sha256}.")


def load_onnx_policy(path, device=None):
    policy_path = Path(path).expanduser().resolve()
    if not policy_path.is_file():
        raise FileNotFoundError(f"Student ONNX not found: {policy_path}")

    available = ort.get_available_providers()
    use_cuda = (device is None or str(device).startswith("cuda")) and "CUDAExecutionProvider" in available
    providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if use_cuda else ["CPUExecutionProvider"]
    output_device = "cuda:0" if use_cuda and torch.cuda.is_available() else "cpu"
    print(f"ONNX providers={providers}; policy output device={output_device}")

    session = ort.InferenceSession(str(policy_path), providers=providers)
    if len(session.get_inputs()) != 1 or len(session.get_outputs()) != 1:
        raise ValueError("Student ONNX must expose exactly one input and one output.")
    input_spec = session.get_inputs()[0]
    output_spec = session.get_outputs()[0]
    validate_student_io(input_spec.shape, output_spec.shape)
    sidecar = _load_sidecar(policy_path)

    metadata = {}
    try:
        model_onnx = onnx.load(str(policy_path))
        metadata = {entry.key: entry.value for entry in model_onnx.metadata_props}
    except Exception:
        pass

    def run_inference(observation):
        if torch.is_tensor(observation):
            obs_in = observation.unsqueeze(0) if observation.dim() == 1 else observation
            obs_np = obs_in.detach().cpu().numpy()
        else:
            obs_np = np.asarray(observation)
            if obs_np.ndim == 1:
                obs_np = obs_np[None, :]
        if obs_np.shape[-1] != 1666:
            raise ValueError(f"Policy observation must end in 1666, got {tuple(obs_np.shape)}.")
        ort_inputs = {input_spec.name: obs_np.astype(np.float32, copy=False)}
        actions_np = session.run([output_spec.name], ort_inputs)[0]
        if actions_np.shape[-1] != 29 or not np.isfinite(actions_np).all():
            raise RuntimeError(f"Invalid student action output: shape={actions_np.shape}.")
        return torch.from_numpy(actions_np).to(output_device)

    run_inference.metadata = metadata
    run_inference.sidecar = sidecar
    run_inference.providers = tuple(providers)
    return run_inference


def load_hit_teacher_policy(path):
    teacher_path = Path(path).expanduser().resolve()
    if not teacher_path.is_file():
        raise FileNotFoundError(f"HIT Teacher ONNX not found: {teacher_path}")
    digest = hashlib.sha256(teacher_path.read_bytes()).hexdigest()
    if digest != EXPECTED_HIT_TEACHER_SHA256:
        raise ValueError(
            f"HIT Teacher SHA256 is {digest}, expected {EXPECTED_HIT_TEACHER_SHA256}."
        )
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(
        str(teacher_path), sess_options=options, providers=["CPUExecutionProvider"]
    )
    input_spec = session.get_inputs()[0]
    output_spec = session.get_outputs()[0]
    if _last_static_dimension(input_spec.shape, "HIT Teacher input") != TEACHER_OBS_DIM:
        raise ValueError(f"HIT Teacher input must be {TEACHER_OBS_DIM}.")
    if _last_static_dimension(output_spec.shape, "HIT Teacher output") != 29:
        raise ValueError("HIT Teacher output must be 29.")

    def run_inference(observation):
        if torch.is_tensor(observation):
            observation = observation.detach().cpu().numpy()
        observation = np.asarray(observation, dtype=np.float32).reshape(1, TEACHER_OBS_DIM)
        action = session.run([output_spec.name], {input_spec.name: observation})[0]
        if action.shape != (1, 29) or not np.isfinite(action).all():
            raise RuntimeError(f"Invalid HIT Teacher action: shape={action.shape}.")
        return torch.from_numpy(action)

    run_inference.providers = tuple(session.get_providers())
    run_inference.sha256 = digest
    return run_inference


def load_and_run_policy(args, se=None):
    if args.hit_arm7_residual_mode != "off" and args.hit_policy_source != "teacher":
        raise ValueError("HIT residual requires --hit-policy-source teacher.")
    if not 0.0 <= args.hit_arm7_residual_scale <= 1.0:
        raise ValueError("--hit-arm7-residual-scale must be within [0, 1].")
    _validate_motion_manifest(args.hit_motion_data, EXPECTED_HIT_MANIFEST_SHA256, "hit")
    _validate_motion_manifest(args.move_motion_data, EXPECTED_MOVE_MANIFEST_SHA256, "move")
    policy_sidecar = _load_sidecar(Path(args.policy).expanduser().resolve())
    checkpoint_sha256 = policy_sidecar.get("checkpoint_sha256")
    relative_x_contract = checkpoint_sha256 in (
        V10_REFEND_CHECKPOINT_SHA256,
        V11_CHECKPOINT_SHA256,
    )
    v11_common_hold = checkpoint_sha256 == V11_CHECKPOINT_SHA256
    if args.hit_arm7_residual_family == "v11-teacher" and args.hit_arm7_residual_mode != "off":
        if not v11_common_hold or args.hit_teacher_history != "distill-reset":
            raise ValueError("V11 Teacher residual requires V11 i42500 and distill-reset history.")
    if bool(args.v10_relative_x) != relative_x_contract:
        raise ValueError(
            "--startup-relative-x/--v10-relative-x must match the frozen Student contract."
        )
    if args.no_target_base_x_clip and not v11_common_hold:
        raise ValueError("--no-target-base-x-clip is only supported by the V11 runtime.")
    common_hold_path = (
        Path(args.policy).expanduser().resolve().parent
        / str(policy_sidecar.get("common_hold_pose_file"))
        if v11_common_hold
        else None
    )
    right_side_canonicalization = (
        args.external_planner
        and args.robot_id == "table_right"
        and args.racket_hand == "right"
        and args.table_right_move_mode == "reflected"
    )
    native_no_mirror = (
        args.external_planner
        and args.robot_id == "table_right"
        and args.racket_hand == "right"
        and args.table_right_move_mode == "native-no-mirror"
    )
    command_profile = RCControllerProfile(dt=0.02, state_estimator=se)
    hardware_agent = LCMAgent(
        se,
        command_profile,
        hit_motion_data=args.hit_motion_data,
        move_motion_data=args.move_motion_data,
        shadow=args.shadow,
        mirror_left_hand=args.mirror_left_hand,
        external_planner=args.external_planner,
        robot_id=args.robot_id,
        torso_topic=args.torso_topic,
        command_timeout_s=args.command_timeout,
        planner_home_y=args.planner_home_y,
        bootstrap_outward_hold=args.bootstrap_outward_hold,
        startup_home_current=args.startup_home_current,
        stationary_hit_test=args.stationary_hit_test,
        hit_reference_lead_steps=args.hit_reference_lead_steps,
        external_hit_command_mode=args.external_hit_command_mode,
        episode_start_x=args.episode_start_x,
        racket_hand=args.racket_hand,
        right_side_canonicalization=right_side_canonicalization,
        native_no_mirror=native_no_mirror,
        action_clip=(10.0 if relative_x_contract else 100.0),
        v10_relative_x=relative_x_contract,
        target_base_x_clip=(
            None if args.no_target_base_x_clip else V10_TARGET_BASE_X_CLIP
        ),
        common_hold_pose_path=(None if common_hold_path is None else str(common_hold_path)),
        common_hold_pose_sha256=(V11_COMMON_HOLD_SHA256 if v11_common_hold else None),
        move_excluded_source_ids=(V11_EXCLUDED_MOVE_SOURCE_IDS if v11_common_hold else None),
        expected_active_move_count=(147 if v11_common_hold else None),
    )
    if args.record_dir is not None:
        hardware_agent.raw_ball_source = RawBallStateSource(
            topic=args.raw_ball_topic,
            max_age_s=args.raw_ball_max_age_ms / 1000.0,
            expected_rigid_body_id=args.raw_ball_rigid_body_id,
        )
    se.spin()
    startup_deadline = time.monotonic() + 10.0
    while (
        hardware_agent._torso_receive_monotonic_ns <= 0
        or se.last_body_receive_monotonic_ns <= 0
    ):
        if rospy.is_shutdown() or time.monotonic() >= startup_deadline:
            raise RuntimeError("Timed out waiting for initial torso and joint state")
        time.sleep(0.05)
    lab_indices = np.asarray(hardware_agent.from_gym_to_lab, dtype=np.int64)
    action_offset_lab = hardware_agent.default_dof_pos[lab_indices].astype(np.float32)
    action_scale_lab = hardware_agent.action_scale[lab_indices].astype(np.float32)
    joint_mirror_spec(LAB_JOINT_NAMES)
    if not np.isfinite(action_offset_lab).all() or not np.isfinite(action_scale_lab).all():
        raise ValueError("Action offset/scale must be finite before policy construction.")
    if np.any(action_scale_lab == 0.0):
        raise ValueError("Action scale must not contain zero before policy construction.")
    teacher_enabled = args.hit_policy_source == "teacher"
    hardware_agent = HistoryWrapper(
        hardware_agent,
        enable_teacher_history=teacher_enabled,
        teacher_history_mode=args.hit_teacher_history,
    )

    student_policy = load_onnx_policy(args.policy, device=hardware_agent.device)
    if args.mirror_left_hand:
        student_policy = MirroredStudentPolicy(
            student_policy,
            LAB_JOINT_NAMES,
            canonical_action_offset=action_offset_lab,
            canonical_action_scale=action_scale_lab,
            physical_action_offset=action_offset_lab,
            physical_action_scale=action_scale_lab,
        )
        print("Left-hand mirror enabled: physical reference -> canonical obs -> physical action")
    elif right_side_canonicalization:
        student_policy = RightSideRightHandStudentPolicy(
            student_policy,
            LAB_JOINT_NAMES,
            action_offset=action_offset_lab,
            action_scale=action_scale_lab,
        )
        print("table_right canonicalization enabled: legs/waist reflected; arms unchanged")
    elif native_no_mirror:
        print("table_right native-no-mirror enabled: raw observation/reference/action")
    policy = student_policy
    if teacher_enabled:
        if args.hit_arm7_residual_mode == "off":
            teacher_policy = load_hit_teacher_policy(args.hit_teacher)
        else:
            residual_adapter = (
                V11TeacherResidualAdapter
                if args.hit_arm7_residual_family == "v11-teacher"
                else V01HitResidualAdapter
            )
            teacher_policy = residual_adapter(
                args.hit_arm7_runtime_module,
                args.hit_teacher,
                args.hit_arm7_residual_onnx,
                args.hit_arm7_residual_checkpoint,
                mode=args.hit_arm7_residual_mode,
                residual_scale=args.hit_arm7_residual_scale,
                ball_topic=args.hit_arm7_ball_topic,
                ball_max_age_s=args.hit_arm7_ball_max_age_ms / 1000.0,
                diagnostics_path=args.hit_arm7_diagnostics_jsonl,
                mirror_ball_y=args.mirror_left_hand,
            )
        if args.mirror_left_hand:
            teacher_policy = MirroredTeacherPolicy(
                teacher_policy,
                LAB_JOINT_NAMES,
                canonical_action_offset=action_offset_lab,
                canonical_action_scale=action_scale_lab,
                physical_action_offset=action_offset_lab,
                physical_action_scale=action_scale_lab,
            )
        policy = TeacherHitOverridePolicy(
            student_policy,
            teacher_policy,
            hit_policy_source=args.hit_policy_source,
        )
        print(
            f"HIT policy source={args.hit_policy_source}; teacher_history={args.hit_teacher_history}; "
            f"lead={args.hit_reference_lead_steps}; residual={args.hit_arm7_residual_mode}; "
            f"residual_family={args.hit_arm7_residual_family}; "
            f"scale={args.hit_arm7_residual_scale:.3f}; mirror={args.mirror_left_hand}; "
            f"racket_hand={args.racket_hand}; move_mode={args.table_right_move_mode}; "
            f"right_side={right_side_canonicalization}; native={native_no_mirror}"
        )
    recorder = None
    if args.record_dir:
        recorder = AsyncDeployRecorder(
            args.record_dir,
            duration_s=args.record_duration,
            mode=args.record_mode,
            pre_trigger_s=args.record_pre_trigger,
            post_trigger_s=args.record_post_trigger,
            metadata={
                "policy_path": str(Path(args.policy).expanduser().resolve()),
                "policy_sidecar": policy.sidecar,
                "hit_motion_data": str(Path(args.hit_motion_data).expanduser().resolve()),
                "move_motion_data": str(Path(args.move_motion_data).expanduser().resolve()),
                "shadow": bool(args.shadow),
                "mirror_left_hand": bool(args.mirror_left_hand),
                "racket_hand": args.racket_hand,
                "right_side_canonicalization": bool(right_side_canonicalization),
                "arm_transform_applied": bool(args.mirror_left_hand),
                "table_right_move_mode": args.table_right_move_mode,
                "observation_transform_applied": bool(
                    args.mirror_left_hand or right_side_canonicalization
                ),
                "action_transform_applied": bool(
                    args.mirror_left_hand or right_side_canonicalization
                ),
                "external_planner": bool(args.external_planner),
                "external_hit_command_mode": args.external_hit_command_mode,
                "robot_id": args.robot_id,
                "torso_topic": args.torso_topic,
                "lcm_url": args.lcm_url,
                "planner_home_y": args.planner_home_y,
                "bootstrap_outward_hold": bool(args.bootstrap_outward_hold),
                "startup_home_current": bool(args.startup_home_current),
                "stationary_hit_test": args.stationary_hit_test,
                "episode_start_x_override": args.episode_start_x,
                "v10_relative_x": relative_x_contract,
                "v11_common_hold": v11_common_hold,
                "action_clip": 10.0 if relative_x_contract else 100.0,
                "target_base_x_clip": (
                    None if args.no_target_base_x_clip else V10_TARGET_BASE_X_CLIP
                ),
                "hit_policy_source": args.hit_policy_source,
                "hit_teacher_path": str(Path(args.hit_teacher).expanduser().resolve()),
                "hit_teacher_sha256": EXPECTED_HIT_TEACHER_SHA256,
                "hit_teacher_history": args.hit_teacher_history,
                "hit_reference_lead_steps": args.hit_reference_lead_steps,
                "hit_arm7_residual_mode": args.hit_arm7_residual_mode,
                "hit_arm7_residual_family": args.hit_arm7_residual_family,
                "hit_arm7_residual_gate": (
                    "hit_post_one_valid" if args.hit_arm7_residual_family == "v11-teacher"
                    else "v01_tts"
                ),
                "hit_arm7_residual_scale": args.hit_arm7_residual_scale,
                "hit_arm7_residual_onnx": args.hit_arm7_residual_onnx,
                "hit_arm7_residual_checkpoint": args.hit_arm7_residual_checkpoint,
                "hit_arm7_ball_topic": args.hit_arm7_ball_topic,
                "hit_arm7_ball_max_age_ms": args.hit_arm7_ball_max_age_ms,
                "hit_arm7_ball_mirror_y": bool(args.mirror_left_hand),
                "raw_ball_topic": args.raw_ball_topic,
                "raw_ball_max_age_ms": args.raw_ball_max_age_ms,
                "raw_ball_rigid_body_id": args.raw_ball_rigid_body_id,
                "record_mode": args.record_mode,
                "record_pre_trigger_s": args.record_pre_trigger,
                "record_post_trigger_s": args.record_post_trigger,
            },
        )
        print(f"Recorder enabled: {recorder.session_dir}")
    deployment_runner = DeploymentRunner(se=None, shadow=args.shadow, recorder=recorder)
    deployment_runner.add_control_agent(hardware_agent, "hardware_closed_loop")
    deployment_runner.add_policy(policy)
    deployment_runner.add_command_profile(command_profile)
    deployment_runner.run(max_steps=10_000_000)


def parse_args():
    parser = argparse.ArgumentParser(description="Deploy the 1666-D six-state doubles student policy.")
    parser.add_argument("--policy", default=DEFAULT_POLICY)
    parser.add_argument(
        "--hit-policy-source", choices=("student", "teacher"), default="student"
    )
    parser.add_argument("--hit-teacher", default=DEFAULT_HIT_TEACHER)
    parser.add_argument(
        "--hit-teacher-history",
        choices=("distill-reset", "legacy-continuous"),
        default="distill-reset",
    )
    parser.add_argument(
        "--hit-reference-lead-steps", choices=(0, 1, 2), type=int, default=0
    )
    parser.add_argument(
        "--hit-arm7-residual-mode", choices=("off", "shadow", "active"), default="off"
    )
    parser.add_argument("--hit-arm7-residual-scale", type=float, default=1.0)
    parser.add_argument("--hit-arm7-residual-family", choices=("v01", "v11-teacher"), default="v01")
    parser.add_argument("--hit-arm7-runtime-module", default=DEFAULT_V01_RUNTIME_MODULE)
    parser.add_argument("--hit-arm7-residual-onnx", default=None)
    parser.add_argument("--hit-arm7-residual-checkpoint", default=None)
    parser.add_argument("--hit-arm7-ball-topic", default="/residual/mocap_ball_state")
    parser.add_argument("--hit-arm7-ball-max-age-ms", type=float, default=60.0)
    parser.add_argument("--hit-arm7-diagnostics-jsonl", default=None)
    parser.add_argument("--raw-ball-topic", default=RAW_BALL_TOPIC)
    parser.add_argument("--raw-ball-max-age-ms", type=float, default=60.0)
    parser.add_argument("--raw-ball-rigid-body-id", type=int, default=BALL_RIGID_BODY_ID,
                        help="Expected raw mocap ball ID for diagnostics (default: 30300)")
    parser.add_argument("--hit-motion-data", default=DEFAULT_HIT_MOTIONS)
    parser.add_argument("--move-motion-data", default=DEFAULT_MOVE_MOTIONS)
    parser.add_argument(
        "--record-dir",
        default=None,
        help="Enable the non-blocking recorder and create a timestamped session below this directory.",
    )
    parser.add_argument(
        "--record-duration",
        default=15.0,
        type=float,
        help="Recorder capacity in seconds at 50 Hz (default: 15).",
    )
    parser.add_argument(
        "--record-mode",
        choices=("bounded", "circular", "triggered"),
        default="circular",
    )
    parser.add_argument("--record-pre-trigger", type=float, default=2.0)
    parser.add_argument("--record-post-trigger", type=float, default=3.0)
    parser.add_argument(
        "--shadow",
        action="store_true",
        help="Read sensors and run the scheduler/policy without calibration or q_des publication.",
    )
    parser.add_argument(
        "--mirror-left-hand",
        action="store_true",
        help="Mirror physical left-hand reference/observation/action around the canonical V9 policy.",
    )
    parser.add_argument("--racket-hand", choices=("right", "left"), default="right")
    parser.add_argument(
        "--table-right-move-mode",
        choices=("reflected", "native-no-mirror"),
        default="reflected",
    )
    parser.add_argument(
        "--external-planner",
        action="store_true",
        help="Disable autonomous ball-topic HIT triggers and consume namespaced planner commands.",
    )
    parser.add_argument("--robot-id", choices=("table_left", "table_right"), default=None)
    parser.add_argument("--torso-topic", default=None)
    parser.add_argument("--command-timeout", type=float, default=0.25)
    parser.add_argument(
        "--external-hit-command-mode",
        choices=("frozen", "stream"),
        default="frozen",
    )
    parser.add_argument("--planner-home-y", type=float, default=None)
    parser.add_argument(
        "--episode-start-x",
        type=float,
        default=None,
        help="Fixed episode X anchor; unset recaptures pelvis X at each OUTWARD start.",
    )
    parser.add_argument("--bootstrap-outward-hold", action="store_true")
    parser.add_argument("--startup-home-current", action="store_true")
    parser.add_argument(
        "--stationary-hit-test",
        choices=("none", "hit_home"),
        default="none",
        help="Use HOME_HOLD <-> HIT only and ignore planner locomotion commands.",
    )
    parser.add_argument(
        "--v10-relative-x",
        "--startup-relative-x",
        dest="v10_relative_x",
        action="store_true",
        help="Enable the frozen startup-relative X/RefEnd/HOLD contract.",
    )
    parser.add_argument(
        "--no-target-base-x-clip",
        action="store_true",
        help="V11 experiment: pass the full target-base X error to the Student.",
    )
    parser.add_argument(
        "--lcm-url",
        default=None,
        help="Per-robot LCM URL; must match LCM_DEFAULT_URL used by g1_control.",
    )
    args = parser.parse_args()
    is_v11_residual = args.hit_arm7_residual_family == "v11-teacher"
    if args.hit_arm7_residual_onnx is None:
        args.hit_arm7_residual_onnx = DEFAULT_V11_RESIDUAL if is_v11_residual else DEFAULT_V01_RESIDUAL
    if args.hit_arm7_residual_checkpoint is None:
        args.hit_arm7_residual_checkpoint = DEFAULT_V11_RESIDUAL_CHECKPOINT if is_v11_residual else DEFAULT_V01_CHECKPOINT
    return args


if __name__ == "__main__":
    args = parse_args()
    if args.external_planner and args.robot_id is None:
        raise SystemExit("--external-planner requires --robot-id")
    if args.external_planner and args.planner_home_y is None:
        raise SystemExit("--external-planner requires --planner-home-y")
    if args.stationary_hit_test == "hit_home" and args.bootstrap_outward_hold:
        raise SystemExit("--stationary-hit-test hit_home conflicts with --bootstrap-outward-hold")
    if args.racket_hand == "right" and args.mirror_left_hand:
        raise SystemExit("right-hand deployment must not use --mirror-left-hand")
    if args.racket_hand == "left" and not args.mirror_left_hand:
        raise SystemExit("left-hand deployment requires --mirror-left-hand")
    if args.robot_id == "table_left" and args.mirror_left_hand:
        raise SystemExit("table_left must use the canonical non-mirrored policy")
    if args.table_right_move_mode == "native-no-mirror" and not (
        args.external_planner
        and args.robot_id == "table_right"
        and args.racket_hand == "right"
        and not args.mirror_left_hand
    ):
        raise SystemExit(
            "native-no-mirror requires external table_right with racket-hand right"
        )
    if args.torso_topic is None:
        args.torso_topic = (
            f"/doubles/{args.robot_id}/torso_pose_origin"
            if args.external_planner
            else "/torso_pose_origin"
        )
    if args.lcm_url is None:
        suffix = "66" if args.robot_id == "table_right" else "198"
        args.lcm_url = f"udpm://239.255.76.{suffix}:7667?ttl=0"
    if not rospy.core.is_initialized():
        node_suffix = args.robot_id or "single"
        rospy.init_node(f"doubles_v9_policy_{node_suffix}", anonymous=False)
    policy_lcm = lcm.LCM(args.lcm_url)
    state_estimator = StateEstimator(policy_lcm)
    load_and_run_policy(args, se=state_estimator)
