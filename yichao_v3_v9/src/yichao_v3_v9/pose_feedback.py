"""Convert existing ROS torso/rigid streams to diagnostic torso poses in origin.

The rigid-1 debug stream is a raw tracker pose, despite its 'racket' name.
It needs the tracker-in-torso inverse. The primary torso stream is already
transformed. Neither path reapplies the table calibration or estimates pelvis.
"""
import copy
import numpy as np

from utils.math import matrix_from_quat_xyzw, rotmat_to_quat_xyzw
from .telemetry import integer, number, vector


PRIMARY = '/torso_pose_origin'
SECONDARY = '/debug/racket_rigid1_pose_origin'
DOUBLES_TORSO = {'/doubles/table_left/torso_pose_origin': ('198', 'table_left'),
                 '/doubles/table_right/torso_pose_origin': ('66', 'table_right')}


class DoublesTorsoFeedback:
    """Already transformed dual Predictor poses; never reapply calibration.

    Topic roles are the deployment mapping, not evidence of physical rigid IDs
    or alignment of the producer's clock to hardware capture time.
    """
    def __init__(self, publisher):
        if not isinstance(publisher, str) or not publisher.startswith('/'):
            raise ValueError('publisher_identity_required')
        self.publisher, self.previous = publisher, {}

    def parse(self, record):
        if record.get('kind') != 'raw_input' or record.get('topic') not in DOUBLES_TORSO:
            raise ValueError('dual_torso_input_required')
        if record.get('publisher') != self.publisher:
            raise ValueError('publisher_changed')
        topic = record['topic']; payload = record['payload']; h = payload['header']
        if h['frame_id'] != 'mocap_origin':
            raise ValueError('unexpected_pose_frame')
        counter = integer(h['seq'])
        sec, ns = integer(h['stamp']['secs']), integer(h['stamp']['nsecs'])
        if ns >= 1000000000:
            raise ValueError('invalid_ros_nanoseconds')
        source, received = sec + ns/1e9, number(record['receive_monotonic_s'])
        if source <= 0 or received < 0:
            raise ValueError('invalid_pose_clock')
        pos, quat = payload['pose']['position'], payload['pose']['orientation']
        pos = vector([pos[k] for k in ('x','y','z')], 3)
        quat = vector([quat[k] for k in ('x','y','z','w')], 4, quaternion=True)
        old = self.previous.get(topic)
        if old and (counter <= old[0] or source <= old[1] or received < old[2]):
            raise ValueError('duplicate_or_out_of_order_pose')
        self.previous[topic] = counter, source, received
        slot, role = DOUBLES_TORSO[topic]
        return {'kind':'torso_feedback','robot_id':slot,'role':role,'source_topic':topic,
            'source_counter':counter,'source_clock':{'predictor_published_ros_stamp_s':source},
            'received_workstation_monotonic_s':received,'frame':'mocap_origin',
            'torso_position_xyz':pos,'torso_orientation_xyzw':quat,
            'tracker_extrinsic_applied':False,'table_transform_applied':False,
            'position_kind':'producer torso, not pelvis/base','command_ack':False,
            'physical_robot_mapping_verified':False,'source_clock_alignment_verified':False,
            'sensor_age_verified':False,'training_coordinates_accepted':False,
            'real_input_accepted':False}


class PoseFeedback:
    def __init__(self, calibration, calibration_sha256, publisher):
        if not isinstance(calibration_sha256, str) or len(calibration_sha256) != 64:
            raise ValueError('calibration_identity_required')
        if not isinstance(publisher, str) or not publisher.startswith('/'):
            raise ValueError('publisher_identity_required')
        self.publisher = publisher
        self.calibration_sha256 = calibration_sha256
        self.config = copy.deepcopy(calibration)
        if self.config['schema'] != 'pingpang.runtime_calibration' or self.config['schema_version'] != 1:
            raise ValueError('unknown_calibration_schema')
        left = self.config['calibrations']['double_left_robot_tracker']
        right = self.config['calibrations']['double_right_robot_tracker']
        if left['rigid_id'] != 1 or right['rigid_id'] != 0 or left['side'] != 'left' or right['side'] != 'right':
            raise ValueError('unexpected_0_right_1_left_mapping')
        ext = left['tracker_in_torso']
        if ext['transform'] != 'tracker_to_torso':
            raise ValueError('unknown_tracker_transform_direction')
        self.ext_t = np.asarray(vector(ext['translation_m'], 3))
        self.ext_r = matrix_from_quat_xyzw(vector(ext['quaternion_xyzw'], 4, quaternion=True))
        self.previous = {}

    def parse(self, record):
        if record.get('kind') != 'raw_input':
            raise ValueError('raw_input_required')
        topic = record['topic']
        if topic not in (PRIMARY, SECONDARY):
            raise ValueError('unsupported_pose_topic')
        if record['publisher'] != self.publisher:
            raise ValueError('publisher_changed')
        received = number(record['receive_monotonic_s'])
        if received < 0:
            raise ValueError('invalid_receipt_clock')
        payload = record['payload']
        if topic == PRIMARY:
            h = payload['header']
            if h['frame_id'] != 'mocap_origin':
                raise ValueError('unexpected_pose_frame')
            counter = integer(h['seq'])
            sec, ns = integer(h['stamp']['secs']), integer(h['stamp']['nsecs'])
            if ns >= 1000000000:
                raise ValueError('invalid_ros_nanoseconds')
            source = sec + ns / 1e9
            p, q = payload['pose']['position'], payload['pose']['orientation']
            pos = vector([p[k] for k in ('x', 'y', 'z')], 3)
            quat = vector([q[k] for k in ('x', 'y', 'z', 'w')], 4, quaternion=True)
            clock = 'predictor_processing_ros_stamp_s'
            slot, role, rigid, transformed = '66', 'table_right', 0, False
        else:
            values = vector(payload['data'], 10)
            source, frame, sensor = values[:3]
            if frame < 0 or not frame.is_integer() or sensor != 1.:
                raise ValueError('unexpected_rigid_id_or_frame')
            counter = int(frame)
            raw_r = matrix_from_quat_xyzw(vector(values[6:10], 4, quaternion=True))
            # Match Mocap._apply_torso_branch: T_origin_tracker @ inv(T_torso_tracker).
            rotation = raw_r @ self.ext_r.T
            pos = (np.asarray(values[3:6]) - rotation @ self.ext_t).tolist()
            quat = rotmat_to_quat_xyzw(rotation).tolist()
            clock = 'chingmu_source_timestamp_s'
            slot, role, rigid, transformed = '198', 'table_left', 1, True
        if source <= 0:
            raise ValueError('invalid_source_timestamp')
        old = self.previous.get(topic)
        if old and (counter <= old[0] or source <= old[1] or received < old[2]):
            raise ValueError('duplicate_or_out_of_order_pose')
        self.previous[topic] = (counter, source, received)
        return {'kind': 'torso_feedback', 'robot_id': slot, 'role': role,
                'configured_rigid_id': rigid, 'source_topic': topic,
                'source_counter': counter, 'source_clock': {clock: source},
                'received_workstation_monotonic_s': received,
                'frame': 'mocap_origin', 'torso_position_xyz': pos,
                'torso_orientation_xyzw': quat,
                'tracker_extrinsic_applied': transformed, 'table_transform_applied': False,
                'calibration_file_sha256': self.calibration_sha256,
                'physical_robot_mapping_verified': False,
                'source_clock_alignment_verified': False, 'sensor_age_verified': False,
                'training_coordinates_accepted': False, 'real_input_accepted': False,
                'position_kind': 'torso, not pelvis/base', 'command_ack': False}
