import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from yichao_v3_v9.base_feedback import reconstruct_base
from utils.joint_mapping import LAB_JOINT_NAMES


def inputs(slot='66'):
    role = 'table_right' if slot == '66' else 'table_left'
    torso = {'robot_id': slot, 'role': role, 'frame': 'mocap_origin',
             'received_workstation_monotonic_s': 100., 'torso_position_xyz': [2., 3., 1.],
             'torso_orientation_xyzw': [0., 0., 0., 1.], 'source_clock': {'ros_processing_s': 1e6},
             'source_counter': 1, 'calibration_file_sha256': 'a'*64}
    joints = {'robot_id': slot, 'role': role, 'joint_names': list(LAB_JOINT_NAMES),
              'normalization_applied': False, 'reflection_applied': False,
              'received_workstation_monotonic_s': 99.99, 'q_lab_rad': [0.]*29,
              'source_clock': {'robot_receive_s': 200.}, 'source_counter': 2, 'session_id': 'test'}
    return torso, joints


class BaseTests(unittest.TestCase):
    def test_neutral_waist_offset_and_robot_symmetry(self):
        for slot in ('66', '198'):
            out = reconstruct_base(*inputs(slot))
            for actual, expected in zip(out['base_position_xyz'], (2.0039635, 3., .956)):
                self.assertAlmostEqual(actual, expected, places=6)
            self.assertFalse(out['real_input_accepted'])
            self.assertFalse(out['sensor_sample_alignment_verified'])

    def test_waist_yaw_uses_named_lab_index(self):
        torso, joints = inputs()
        joints['q_lab_rad'][LAB_JOINT_NAMES.index('waist_yaw_joint')] = math.pi/2
        torso['torso_orientation_xyzw'] = [0., 0., math.sqrt(.5), math.sqrt(.5)]
        out = reconstruct_base(torso, joints)
        for actual, expected in zip(out['base_position_xyz'], (2., 3.0039635, .956)):
            self.assertAlmostEqual(actual, expected, places=6)
        for actual, expected in zip(out['base_orientation_wxyz'], (1., 0., 0., 0.)):
            self.assertAlmostEqual(actual, expected, places=6)

    def test_future_and_old_samples_rejected(self):
        for time in (100.001, 99.94):
            torso, joints = inputs(); joints['received_workstation_monotonic_s'] = time
            with self.assertRaisesRegex(ValueError, 'future_or_old_joint_receipt'):
                reconstruct_base(torso, joints)
        with self.assertRaises(ValueError): reconstruct_base(*inputs(), maximum_receipt_skew_s=.3)

    def test_mixed_robot_order_and_scaled_q_rejected(self):
        torso, joints = inputs(); joints['robot_id'] = '198'
        with self.assertRaises(ValueError): reconstruct_base(torso, joints)
        torso, joints = inputs(); joints['joint_names'].reverse()
        with self.assertRaises(ValueError): reconstruct_base(torso, joints)
        torso, joints = inputs(); joints['normalization_applied'] = True
        with self.assertRaises(ValueError): reconstruct_base(torso, joints)

    def test_physical_limits_and_quaternion_validation(self):
        torso, joints = inputs(); joints['q_lab_rad'][LAB_JOINT_NAMES.index('waist_roll_joint')] = 1.
        with self.assertRaises(ValueError): reconstruct_base(torso, joints)
        torso, joints = inputs(); torso['torso_orientation_xyzw'] = [0.,0.,0.,0.]
        with self.assertRaises(ValueError): reconstruct_base(torso, joints)

    def test_preserves_both_source_clocks_and_inputs(self):
        torso, joints = inputs(); before = copy.deepcopy((torso, joints))
        out = reconstruct_base(torso, joints)
        self.assertEqual(out['torso_source_clock'], {'ros_processing_s': 1e6})
        self.assertEqual(out['joint_source_clock'], {'robot_receive_s': 200.})
        self.assertFalse(out['sensor_age_verified'])
        self.assertFalse(out['command_ack'])
        self.assertEqual((torso, joints), before)


if __name__ == '__main__':
    unittest.main()
