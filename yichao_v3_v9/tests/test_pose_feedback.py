import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from yichao_v3_v9.pose_feedback import PoseFeedback, DoublesTorsoFeedback, DOUBLES_TORSO, PRIMARY, SECONDARY


def config():
    return {'schema': 'pingpang.runtime_calibration', 'schema_version': 1,
            'calibrations': {'double_left_robot_tracker': {'rigid_id': 1, 'side': 'left',
                'tracker_in_torso': {'transform': 'tracker_to_torso',
                                     'translation_m': [1., 0., 0.],
                                     'quaternion_xyzw': [0., 0., 0., 1.]}},
                'double_right_robot_tracker': {'rigid_id': 0, 'side': 'right'}}}


def raw(topic=SECONDARY):
    p = {'data': [1000., 1., 1., 3., 4., 5., 0., 0., 0., 1.]}
    if topic == PRIMARY:
        p = {'header': {'frame_id': 'mocap_origin', 'seq': 1,
                        'stamp': {'secs': 1000, 'nsecs': 123}},
             'pose': {'position': {'x': 3., 'y': 4., 'z': 5.},
                      'orientation': {'x': 0., 'y': 0., 'z': 0., 'w': 1.}}}
    return {'kind': 'raw_input', 'topic': topic, 'publisher': '/existing_predictor',
            'receive_monotonic_s': 50., 'payload': p}


def adapter(c=None):
    return PoseFeedback(c or config(), 'a' * 64, '/existing_predictor')


class PoseTests(unittest.TestCase):
    def test_dual_torso_roles_preserve_pose_and_source_clock(self):
        converter=DoublesTorsoFeedback('/existing_predictor')
        for topic,(slot,role) in DOUBLES_TORSO.items():
            record=raw(PRIMARY);record['topic']=topic;original=copy.deepcopy(record)
            out=converter.parse(record)
            self.assertEqual((out['robot_id'],out['role']),(slot,role))
            self.assertEqual(out['torso_position_xyz'],[3.,4.,5.])
            self.assertEqual(out['torso_orientation_xyzw'],[0.,0.,0.,1.])
            self.assertAlmostEqual(out['source_clock']['predictor_published_ros_stamp_s'],1000.000000123)
            self.assertFalse(out['real_input_accepted'])
            self.assertFalse(out['tracker_extrinsic_applied'])
            self.assertEqual(record,original)
            with self.assertRaisesRegex(ValueError,'duplicate_or_out_of_order_pose'):converter.parse(record)

    def test_dual_torso_rejects_changed_publisher_frame_and_quaternion(self):
        for kind in ('publisher','frame','quaternion'):
            r=raw(PRIMARY);r['topic']='/doubles/table_left/torso_pose_origin'
            if kind=='publisher':r['publisher']='/replacement'
            if kind=='frame':r['payload']['header']['frame_id']='table'
            if kind=='quaternion':r['payload']['pose']['orientation']['w']=0.
            with self.assertRaises(ValueError):DoublesTorsoFeedback('/existing_predictor').parse(r)

    def test_inverse_tracker_translation(self):
        out = adapter().parse(raw())
        self.assertEqual(out['torso_position_xyz'], [2., 4., 5.])
        self.assertEqual(out['robot_id'], '198')
        self.assertFalse(out['table_transform_applied'])
        self.assertFalse(out['real_input_accepted'])
        self.assertFalse(out['command_ack'])

    def test_rotated_tracker_offset(self):
        record = raw(); record['payload']['data'][6:] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
        out = adapter().parse(record)
        for a, b in zip(out['torso_position_xyz'], [3., 3., 5.]):
            self.assertAlmostEqual(a, b)

    def test_nonidentity_extrinsic_inverse(self):
        c = config(); c['calibrations']['double_left_robot_tracker']['tracker_in_torso']['quaternion_xyzw'] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
        out = adapter(c).parse(raw())
        for a, b in zip(out['torso_position_xyz'], [3., 5., 5.]):
            self.assertAlmostEqual(a, b)

    def test_primary_torso_not_transformed_twice_or_retimed(self):
        record = raw(PRIMARY); original = copy.deepcopy(record)
        out = adapter().parse(record)
        self.assertEqual(out['torso_position_xyz'], [3., 4., 5.])
        self.assertEqual(out['robot_id'], '66')
        self.assertFalse(out['tracker_extrinsic_applied'])
        self.assertAlmostEqual(out['source_clock']['predictor_processing_ros_stamp_s'], 1000.000000123)
        self.assertFalse(out['sensor_age_verified'])
        self.assertEqual(record, original)

    def test_wrong_sensor_quaternion_publisher_and_frame_rejected(self):
        for i, value in ((2, 10.), (1, 1.5), (0, -1.)):
            record = raw(); record['payload']['data'][i] = value
            with self.assertRaises(ValueError): adapter().parse(record)
        record = raw(); record['payload']['data'][6:] = [0., 0., 0., 0.]
        with self.assertRaises(ValueError): adapter().parse(record)
        record = raw(); record['publisher'] = '/replacement'
        with self.assertRaises(ValueError): adapter().parse(record)
        record = raw(PRIMARY); record['payload']['header']['frame_id'] = 'table'
        with self.assertRaises(ValueError): adapter().parse(record)

    def test_old_mapping_and_duplicate_rejected(self):
        c = config(); c['calibrations']['double_left_robot_tracker']['rigid_id'] = 10
        with self.assertRaises(ValueError): adapter(c)
        a = adapter(); a.parse(raw())
        with self.assertRaises(ValueError): a.parse(raw())


if __name__ == '__main__':
    unittest.main()
