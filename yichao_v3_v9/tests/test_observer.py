import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from yichao_v3_v9.telemetry import TOPICS, TelemetryParser, strict_json
from yichao_v3_v9.observer import Writer, SegmentedWriter, capture, preflight, pose_payload, replay

BALL = '/doubles/ball_prediction'
STATE = '/doubles/table_left/state'
TORSO = '/doubles/table_right/torso_pose_origin'


def fixture(topic=BALL):
    if topic == BALL:
        return dict(schema_version='v9-ball-prediction-v1', sequence=1, source_timestamp=100.,
                    shot_id='mocap-1', valid=True, position=[0, 0, 1], velocity=[1, 0, 0],
                    predicted_strike_position=[0, 0, 1], predicted_strike_velocity=[1, 0, 0],
                    time_to_strike_s=.5, racket_normal=[1, 0, 0], racket_velocity=[4, 0, 0])
    if topic == STATE:
        return dict(schema_version='v9-robot-state-v1', robot='table_left', sequence=1,
                    source_monotonic_ns=777000000000, valid=True, q=[0]*29, dq=[0]*29,
                    imu_quaternion_wxyz=[1, 0, 0, 0], gyro_xyz=[0]*3,
                    base_position_xyz=[0, .2, .8], base_orientation_wxyz=[1, 0, 0, 0],
                    base_linear_velocity_xyz=[0]*3, base_angular_velocity_xyz=[0]*3,
                    phase='HOME_HOLD', ready=True, state_elapsed_s=1., stable_elapsed_s=.5,
                    target_base_y=.2, home_y=.2, time_to_strike_s=.5,
                    last_applied_sequence=100, last_planner_session_id='someone-elses-controller',
                    last_commit_token='someone-elses-token', emergency_stop=False, transport_error=None)
    return dict(header=dict(seq=1, stamp=dict(secs=100, nsecs=0), frame_id='origin_frame_w'),
                position_xyz=[0, -.2, .8], orientation_xyzw=[0, 0, 0, 1])


def receive(dt=0):
    return dict(monotonic_s=10.+dt, wall_s=100.+dt, ros_s=100.+dt)


def good_graph():
    return {t: ['/' + TOPICS[t][0] + '_synthetic_source'] for t in TOPICS}, {t: v[1] for t, v in TOPICS.items()}


class ObserverTest(unittest.TestCase):
    def test_known_wire_schemas_still_do_not_certify_real_inputs(self):
        for topic in (BALL, STATE, TORSO):
            result = TelemetryParser().parse(topic, fixture(topic), receive(), 'synthetic_wire_fixture')
            self.assertTrue(result['wire_valid'], result)
            self.assertFalse(result['real_input_accepted'])
            self.assertFalse(result['sensor_age_verified'])
            self.assertFalse(result['command_ack'])
        result = TelemetryParser().parse(STATE, fixture(STATE), receive())
        self.assertEqual(result['source_time'], 777000000000)
        self.assertNotIn('candidate_ros_age_s', result)
        self.assertEqual(result['slot_candidate']['robot'], 198)
        self.assertFalse(result['slot_candidate']['mapping_verified'])

    def test_invalid_field_and_role_cases(self):
        cases = [(BALL, 'shot_id', 1), (BALL, 'valid', 1), (BALL, 'position', [0, 1]),
                 (BALL, 'racket_velocity', [float('nan'), 0, 0]),
                 (BALL, 'source_timestamp', None), (BALL, 'time_to_strike_s', -.1),
                 (STATE, 'q', [0]*28), (STATE, 'dq', [float('inf')]*29),
                 (STATE, 'robot', 'table_right'), (STATE, 'phase', 'READY'),
                 (STATE, 'imu_quaternion_wxyz', [0]*4), (STATE, 'valid', False),
                 (STATE, 'source_monotonic_ns', 1.2), (TORSO, 'orientation_xyzw', [1]*4)]
        for topic, key, value in cases:
            with self.subTest(topic=topic, key=key, value=value):
                payload = fixture(topic)
                payload[key] = value
                self.assertFalse(TelemetryParser().parse(topic, payload, receive())['wire_valid'])
        payload = fixture()
        del payload['racket_velocity']
        self.assertFalse(TelemetryParser().parse(BALL, payload, receive())['wire_valid'])
        self.assertFalse(TelemetryParser().parse('/doubles/command', fixture(), receive())['wire_valid'])

    def test_duplicate_restart_and_out_of_order_latch(self):
        for seq, source in ((1, 100.02), (0, 101.), (2, 99.)):
            parser = TelemetryParser()
            self.assertTrue(parser.parse(BALL, fixture(), receive())['wire_valid'])
            d = fixture()
            d.update(sequence=seq, source_timestamp=source)
            self.assertFalse(parser.parse(BALL, d, receive(.02))['wire_valid'])
            d.update(sequence=3, source_timestamp=100.04)
            self.assertFalse(parser.parse(BALL, d, receive(.04))['wire_valid'])

    def test_clock_jump_stale_future_and_receive_gap(self):
        for field in ('wall_s', 'ros_s', 'monotonic_s'):
            parser = TelemetryParser()
            parser.parse(BALL, fixture(), receive())
            r = receive(.02)
            r[field] += 1
            d = fixture(); d.update(sequence=2, source_timestamp=100.02)
            self.assertIn('clock jump', parser.parse(BALL, d, r)['reason'])
        for timestamp in (0, 99., 101.):
            d = fixture(); d['source_timestamp'] = timestamp
            self.assertFalse(TelemetryParser().parse(BALL, d, receive())['wire_valid'])
        parser = TelemetryParser()
        parser.parse(STATE, fixture(STATE), receive())
        d = fixture(STATE); d.update(sequence=2, source_monotonic_ns=778000000000)
        self.assertTrue(parser.parse(STATE, d, receive(1.))['receive_gap_exceeded'])

    def test_json_rejects_duplicates_nonfinite_and_oversize(self):
        for text in ('{"x":1,"x":2}', '{"q":NaN}', '[]', ' '*65537):
            with self.assertRaises(ValueError):
                strict_json(text)
        self.assertEqual(strict_json('{"shot_id":"mocap-1"}')['shot_id'], 'mocap-1')

    def test_pose_header_and_quaternion_are_preserved(self):
        ns = types.SimpleNamespace
        msg = ns(header=ns(seq=1, stamp=ns(secs=100, nsecs=0), frame_id='origin_frame_w'),
                 pose=ns(position=ns(x=0, y=-.2, z=.8), orientation=ns(x=0, y=0, z=0, w=1)))
        self.assertEqual(pose_payload(msg), fixture(TORSO))
        for secs, nsecs in ((0, 0), (100, 1000000000), (-1, 0)):
            d = fixture(TORSO); d['header']['stamp'] = dict(secs=secs, nsecs=nsecs)
            self.assertFalse(TelemetryParser().parse(TORSO, d, receive())['wire_valid'])

    def test_topic_type_listing_is_not_a_publisher(self):
        _, types_ = good_graph()
        check = preflight(lambda: ({}, types_))
        self.assertFalse(check['ready_to_subscribe'])
        self.assertEqual(len(check['missing_publishers']), 5)
        loader = Mock(side_effect=AssertionError('must not import ROS'))
        with self.assertRaises(ValueError):
            capture(.1, Mock(), check, ros_loader=loader)
        loader.assert_not_called()

    def test_graph_failure_type_mismatch_and_source_change(self):
        def unavailable():
            raise OSError('offline')
        self.assertFalse(preflight(unavailable)['ready_to_subscribe'])
        pubs, types_ = good_graph(); types_[BALL] = 'std_msgs/Int32'
        self.assertEqual(preflight(lambda: (pubs, types_))['wrong_types'], [BALL])
        check = preflight(good_graph)
        pubs, types_ = good_graph(); pubs[BALL] = ['/different_source']
        loader = Mock()
        with self.assertRaises(ValueError):
            capture(.1, Mock(), check, lambda: (pubs, types_), loader)
        loader.assert_not_called()

    def test_mock_capture_creates_only_allowlisted_subscribers_and_cleans_up(self):
        ros = Mock()
        ros.get_param.return_value = False
        ros.is_shutdown.return_value = False
        ros.Time.now.return_value.to_sec.return_value = 100.
        String, Pose = type('String', (), {}), type('PoseStamped', (), {})
        records = []
        capture(.03, records.append, preflight(good_graph), good_graph, lambda: (ros, String, Pose))
        ros.Publisher.assert_not_called()
        self.assertEqual([c.args[0] for c in ros.Subscriber.call_args_list], list(TOPICS))
        self.assertEqual(ros.Subscriber.return_value.unregister.call_count, 5)
        self.assertTrue(ros.init_node.call_args.kwargs['disable_rosout'])
        self.assertTrue(ros.init_node.call_args.kwargs['disable_rostime'])
        self.assertEqual(records[-1]['reason'], 'duration_complete')
        self.assertFalse(records[-1]['real_input_accepted'])

    def test_capture_failure_unregisters_and_sim_time_does_not_start(self):
        ros = Mock(); ros.get_param.return_value = True
        with self.assertRaises(ValueError):
            capture(.03, Mock(), preflight(good_graph), good_graph, lambda: (ros, object, object))
        ros.init_node.assert_not_called()
        ros.get_param.return_value = False
        def broken_write(record):
            raise OSError('output disk error')
        with self.assertRaises(OSError):
            capture(.03, broken_write, preflight(good_graph), good_graph, lambda: (ros, object, object))
        self.assertEqual(ros.Subscriber.return_value.unregister.call_count, 5)
        ros.signal_shutdown.assert_called_once()

    def test_callbacks_record_publisher_and_reject_unknown_source(self):
        ros = Mock(); ros.get_param.return_value = False; ros.is_shutdown.return_value = False
        ros.Time.now.return_value.to_sec.return_value = 100.
        def subscribe(topic, typ, callback, **kwargs):
            if topic == BALL:
                for caller in ('/ball_synthetic_source', '/unexpected_source'):
                    msg = types.SimpleNamespace(data=json.dumps(fixture()), _connection_header={'callerid': caller})
                    callback(msg, topic)
            return Mock()
        ros.Subscriber.side_effect = subscribe
        records = []
        capture(.04, records.append, preflight(good_graph), good_graph, lambda: (ros, object, object))
        self.assertEqual(records[1]['publisher'], '/ball_synthetic_source')
        self.assertTrue(records[1]['diagnostic']['wire_valid'])
        self.assertFalse(records[1]['diagnostic']['command_ack'])
        self.assertEqual(records[2]['kind'], 'parse_error')
        self.assertIn('publisher', records[2]['reason'])
        self.assertEqual(records[-1]['wire_valid'], 1)
        self.assertEqual(records[-1]['invalid'], 1)

    def test_stream_gap_and_publisher_loss_stop_capture(self):
        ros = Mock(); ros.get_param.return_value = False; ros.is_shutdown.return_value = False
        records = []
        capture(.5, records.append, preflight(good_graph), good_graph, lambda: (ros, object, object))
        self.assertEqual(records[-1]['reason'], 'input_stream_gap')
        records = []
        reader = Mock(side_effect=[good_graph(), ({}, {})])
        capture(.1, records.append, preflight(good_graph), reader, lambda: (ros, object, object))
        self.assertEqual(records[-1]['reason'], 'publisher_graph_changed')

    def test_output_budget_and_json_finiteness(self):
        stream = io.StringIO(); writer = Writer(stream)
        writer({'command_ack': False})
        self.assertFalse(json.loads(stream.getvalue())['command_ack'])
        writer.total = 16*1024*1024
        with self.assertRaises(ValueError):
            writer({})

    def test_persistent_logs_preserve_every_record_across_segments(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'poses.jsonl'
            records = [{'sequence': i, 'text': '姿态'} for i in range(20)]
            with path.open('x') as stream:
                writer = SegmentedWriter(stream, segment_bytes=100)
                try:
                    for record in records:
                        writer(record)
                    self.assertGreater(writer.segment, 1)
                    self.assertGreater(writer.total, 100)
                finally:
                    writer.close()
                self.assertFalse(stream.closed)
            actual = [json.loads(line) for p in sorted(Path(tmp).glob('poses.jsonl*'))
                      for line in p.read_text().splitlines()]
            self.assertEqual(actual, records)
            self.assertTrue(all(p.stat().st_size <= 100 for p in Path(tmp).iterdir()))

    def test_persistent_logs_do_not_overwrite_existing_segments_or_accept_nan(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'poses.jsonl'
            previous = Path(str(path)+'.0001')
            previous.write_text('previous evidence\n')
            with path.open('x') as stream:
                writer = SegmentedWriter(stream, segment_bytes=10)
                with self.assertRaises(ValueError):
                    writer({'value': float('nan')})
                writer({})
                with self.assertRaises(FileExistsError):
                    writer({'next': 1})
                writer.close()
            self.assertEqual(previous.read_text(), 'previous evidence\n')

    def test_replay_consumes_capture_metadata_and_preserves_provenance(self):
        events = [preflight(good_graph), {'kind': 'capture_start'},
                  {'topic': BALL, 'payload': fixture(), 'receive': receive(), 'provenance': 'live_ros'},
                  {'kind': 'parse_error', 'reason': 'invalid JSON'}, {'kind': 'capture_end'}]
        records = []
        replay(io.StringIO(''.join(json.dumps(x)+'\n' for x in events)), records.append)
        self.assertEqual(len(records), 5)
        self.assertEqual(records[2]['diagnostic']['provenance'], 'live_ros')
        self.assertTrue(records[2]['diagnostic']['wire_valid'])
        self.assertFalse(records[2]['diagnostic']['real_input_accepted'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
