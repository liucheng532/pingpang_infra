"""Diagnostic V9 pelvis reconstruction from co-recorded torso and joint streams.

Workstation receipt alignment prevents accidental cross-run/future joins; it does
not certify simultaneous sensor sampling or cross-machine clock alignment.
"""
from utils.g1_pelvis_pose import G1PelvisPoseEstimator
from utils.joint_mapping import FROM_LAB_TO_GYM, LAB_JOINT_NAMES
from .telemetry import number, vector


def reconstruct_base(torso, joints, maximum_receipt_skew_s=.05):
    slot = torso['robot_id']
    roles = {'66': 'table_right', '198': 'table_left'}
    if slot not in roles or joints['robot_id'] != slot:
        raise ValueError('torso_joint_robot_mismatch')
    if torso['role'] != roles[slot] or joints['role'] != roles[slot]:
        raise ValueError('robot_role_mismatch')
    if torso['frame'] != 'mocap_origin':
        raise ValueError('unverified_pose_frame')
    if joints['joint_names'] != list(LAB_JOINT_NAMES):
        raise ValueError('unexpected_joint_name_order')
    if joints['normalization_applied'] is not False or joints['reflection_applied'] is not False:
        raise ValueError('physical_unscaled_unmirrored_joints_required')
    receipt = number(torso['received_workstation_monotonic_s'])
    joint_receipt = number(joints['received_workstation_monotonic_s'])
    timeout = number(maximum_receipt_skew_s)
    if not 0 < timeout <= .05:
        raise ValueError('receipt_skew_limit_must_not_exceed_50ms')
    skew = receipt - joint_receipt
    if not 0 <= skew <= timeout:
        raise ValueError('future_or_old_joint_receipt')
    q_lab = vector(joints['q_lab_rad'], 29)
    q_gym = [q_lab[i] for i in FROM_LAB_TO_GYM]
    pos = vector(torso['torso_position_xyz'], 3)
    quat = vector(torso['torso_orientation_xyzw'], 4, quaternion=True)
    base, orientation = G1PelvisPoseEstimator().estimate(pos, [quat[3], *quat[:3]], q_gym)
    return {'kind': 'base_feedback_diagnostic', 'robot_id': slot, 'role': roles[slot],
            'frame': 'mocap_origin', 'base_position_xyz': base.tolist(),
            'base_orientation_wxyz': orientation.tolist(),
            'torso_position_xyz': pos, 'waist_q_sdk_rad': q_gym[12:15],
            'received_workstation_monotonic_s': receipt,
            'joint_received_workstation_monotonic_s': joint_receipt,
            'joint_receipt_age_s': skew,
            'torso_source_clock': torso['source_clock'], 'joint_source_clock': joints['source_clock'],
            'torso_source_counter': torso['source_counter'],
            'joint_source_counter': joints['source_counter'], 'joint_session_id': joints['session_id'],
            'calibration_file_sha256': torso['calibration_file_sha256'],
            'estimator_basis': 'frozen V9 G1PelvisPoseEstimator',
            'sensor_sample_alignment_verified': False, 'sensor_age_verified': False,
            'training_coordinates_accepted': False, 'real_input_accepted': False, 'command_ack': False}
