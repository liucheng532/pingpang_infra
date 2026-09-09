import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from yichao_v3_v9.joint_feedback import JointFeedback


def sample(slot='66', tick=1, stamp=100., received=500.):
    return {'robot_id': slot, 'session_id': 'read-only-test',
            'received_workstation_monotonic_s': received,
            'payload': {'kind': 'lowstate', 'tick': tick, 'receive_monotonic_s': stamp,
                        'q_sdk_order': [float(i) for i in range(29)],
                        'dq_sdk_order': [-float(i) for i in range(29)],
                        'imu_quaternion_raw': [1., 0., 0., 0.]}}


class JointFeedbackTests(unittest.TestCase):
    def test_sdk_to_lab_anatomical_indices_no_mirror(self):
        raw = sample(); original = copy.deepcopy(raw)
        out = JointFeedback().ingest(raw)
        q = dict(zip(out['joint_names'], out['q_lab_rad']))
        self.assertEqual(q['left_hip_pitch_joint'], 0.)
        self.assertEqual(q['right_hip_pitch_joint'], 6.)
        self.assertEqual(q['waist_yaw_joint'], 12.)
        self.assertEqual(q['left_elbow_joint'], 18.)
        self.assertEqual(q['right_elbow_joint'], 25.)
        self.assertEqual(q['right_wrist_yaw_joint'], 28.)
        self.assertEqual(out['q_lab_rad'][:6], [0., 6., 12., 1., 7., 13.])
        self.assertFalse(out['reflection_applied'])
        self.assertEqual(raw, original)

    def test_stock_state_is_not_reordered_twice(self):
        expected = JointFeedback().ingest(sample('198'))
        envelope = sample('198')
        envelope['payload'] = {'schema_version': 'v9-robot-state-v1',
            'robot': 'table_left', 'sequence': 1, 'source_monotonic_ns': 100000000001,
            'valid': True, 'q': expected['q_lab_rad'], 'dq': expected['dq_lab_rad_s'],
            'imu_quaternion_wxyz': [1., 0., 0., 0.]}
        out = JointFeedback().ingest(envelope)
        self.assertEqual(out['q_lab_rad'], expected['q_lab_rad'])
        self.assertEqual(out['source_clock']['robot_state_publication_monotonic_ns'], 100000000001)
        envelope['payload']['robot'] = 'table_right'
        with self.assertRaisesRegex(ValueError, 'robot_role_mismatch'):
            JointFeedback().ingest(envelope)

    def test_source_clock_never_subtracted_from_workstation(self):
        out = JointFeedback().ingest(sample(stamp=1e6, received=5.))
        self.assertEqual(out['source_clock']['robot_dds_receive_monotonic_s'], 1e6)
        self.assertFalse(out['sensor_age_verified'])
        self.assertFalse(out['real_input_accepted'])
        self.assertFalse(out['command_ack'])

    def test_duplicate_restart_and_backward_clocks_latch(self):
        for key, value in [('tick', 1), ('tick', 0), ('receive_monotonic_s', 99.)]:
            stream = JointFeedback(); stream.ingest(sample())
            raw = sample(tick=2, stamp=101., received=501.)
            raw['payload'][key] = value
            with self.assertRaises(ValueError): stream.ingest(raw)
            with self.assertRaisesRegex(ValueError, 'stream_fault_latched'):
                stream.ingest(sample(tick=3, stamp=102., received=502.))
        for key, value in [('session_id', 'restarted'), ('received_workstation_monotonic_s', 499.)]:
            stream = JointFeedback(); stream.ingest(sample())
            raw = sample(tick=2, stamp=101., received=501.); raw[key] = value
            with self.assertRaises(ValueError): stream.ingest(raw)

    def test_missing_robot_stale_and_fault_are_not_ready(self):
        stream = JointFeedback(); stream.ingest(sample())
        self.assertEqual(stream.snapshot(500.)['missing_robots'], ['198'])
        stream.ingest(sample('198', received=500.1))
        out = stream.snapshot(500.2)
        self.assertTrue(out['both_streams_recently_received'])
        self.assertFalse(out['synchronized_sample'])
        self.assertFalse(out['real_input_accepted'])
        self.assertEqual(stream.snapshot(501.)['stale_receive_robots'], ['66', '198'])
        with self.assertRaises(ValueError): stream.ingest(sample())
        self.assertFalse(stream.snapshot(500.2)['both_streams_recently_received'])

    def test_malformed_vectors_flags_and_unknown_identity(self):
        for key, value in [('q_sdk_order', [0.] * 28), ('dq_sdk_order', [float('nan')] * 29),
                           ('imu_quaternion_raw', [0., 0., 0., 0.]), ('tick', True)]:
            raw = sample(); raw['payload'][key] = value
            with self.assertRaises(ValueError): JointFeedback().ingest(raw)
        with self.assertRaises(ValueError): JointFeedback().ingest(sample('left'))
        for invalid in ([], None, 1):
            raw = sample(); raw['payload'] = invalid
            with self.assertRaisesRegex(ValueError, 'joint_payload_must_be_object'):
                JointFeedback().ingest(raw)
        with self.assertRaisesRegex(ValueError, 'joint_envelope_must_be_object'):
            JointFeedback().ingest([])
        raw = sample(); raw['payload'].update(real_input_accepted=True, command_ack=True)
        out = JointFeedback().ingest(raw)
        self.assertFalse(out['real_input_accepted']); self.assertFalse(out['command_ack'])


if __name__ == '__main__':
    unittest.main()
