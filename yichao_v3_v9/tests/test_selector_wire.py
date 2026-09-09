import copy
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from yichao_v3_v9.launch import build_launch, control_conflicts, environment
from yichao_v3_v9.v9_wire import encode, inspect_stock_feedback
from utils.planner_ros_bridge import PlannerRosBridge


def command(kind='move', robot='198'):
    c = dict(schema='yichao-v3-v9-command-v1', session='synthetic-wire-test',
             sequence=1, shot='shot-1', token='token-1', robot=robot, kind=kind,
             target_y=.3 if robot=='198' else -.3, issued_at=10., expires_at=10.25,
             hit=None, return_y=.2 if robot=='198' else -.2)
    if kind == 'hit':
        c['hit'] = dict(position=[0, .2, 1], racket_velocity=[4, 0, 0], ball_velocity=[-4, 0, 0],
                        tts=.4, outward_y=.9, return_y=.2)
    return c


def stock_bridge(payload):
    # Exercise the frozen bridge's pure apply method without constructing ROS.
    bridge = object.__new__(PlannerRosBridge)
    bridge._lock = threading.Lock()
    bridge._latest_command = payload
    bridge._latest_receive_monotonic = time.monotonic()
    bridge.command_timeout_s = .25
    bridge._last_session_id = None
    bridge._last_applied_sequence = -1
    bridge._last_commit_token = None
    bridge._last_error = ''
    return bridge


class WireTests(unittest.TestCase):
    def test_move_and_clear_are_consumed_as_base_requests(self):
        for kind in ('move', 'clear'):
            for robot in ('66', '198'):
                c = command(kind, robot)
                wire = encode(c, 10.01, .1)
                self.assertEqual(wire['robot'], 'table_left' if robot=='198' else 'table_right')
                scheduler = Mock()
                bridge = stock_bridge(wire)
                bridge.apply_pending(scheduler, [0., 0., .75])
                scheduler.set_external_base_target.assert_called_once()
                scheduler.set_external_hit.assert_not_called()
                target = scheduler.set_external_base_target.call_args.args[0]
                self.assertAlmostEqual(target[0], 0.)  # stock V9 uses current pelvis X
                self.assertAlmostEqual(target[1], c['target_y'])
                self.assertEqual(bridge._last_error, '')
                self.assertNotEqual(wire['command']['role'], 'hold')

    def test_return_is_explicit_and_hit_requires_enable(self):
        wire = encode(command('return'), 10., 0.)
        scheduler = Mock(); stock_bridge(wire).apply_pending(scheduler, [0, .3, .75])
        scheduler.set_external_base_target.assert_called_once()
        self.assertAlmostEqual(scheduler.set_external_base_target.call_args.kwargs['return_target_y'], .3)
        with self.assertRaises(ValueError): encode(command('hit'), 10., 0.)
        with self.assertRaises(ValueError): encode(command('hit'), 10., 0., allow_hit='false')
        c = command('hit'); wire = encode(c, 10.05, 0., allow_hit=True)
        self.assertAlmostEqual(wire['command']['predicted_ball_predict_time'], .35)
        scheduler = Mock(); bridge = stock_bridge(wire)
        bridge.apply_pending(scheduler, [0, .3, .75])
        scheduler.set_external_hit.assert_called_once()
        scheduler.set_external_outward_target.assert_called_once_with(.9)
        self.assertEqual(bridge.last_commit_token, c['token'])

    def test_entire_hit_is_checked_before_emission(self):
        for field, value in [('position',[0, 1]), ('racket_velocity',[float('nan'), 0, 0]),
                             ('outward_y',float('inf')), ('return_y',2), ('tts',-.1)]:
            c = command('hit'); c['hit'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                encode(c, 10., 0., allow_hit=True)
        for now in (9.9, 10.26):
            with self.assertRaises(ValueError): encode(command(), now, 0.)
        with self.assertRaises(ValueError): encode(command(), 10., float('nan'))

    def test_source_is_immutable_and_telemetry_never_becomes_ack(self):
        c = command('hit'); original = copy.deepcopy(c)
        wire = encode(c, 10.1, 0., allow_hit=True)
        self.assertEqual(c, original)
        json.dumps(wire, allow_nan=False)
        state = dict(robot='table_left',last_planner_session_id=c['session'],
                     last_applied_sequence=c['sequence'],last_commit_token=c['token'],transport_error='')
        status = inspect_stock_feedback(c, state)
        self.assertTrue(status['identity_matches'] and status['sequence_matches'] and status['token_matches'])
        self.assertFalse(status['command_ack'])
        self.assertFalse(status['real_execution_confirmed'])


class SelectorTests(unittest.TestCase):
    def test_fixed_selection_preserves_entry_and_marks_shadow_control_ownership(self):
        for mode in ('shadow','active'):
            plan = build_launch('fixed',mode,'/tmp/example')
            self.assertTrue(plan.may_publish_control)
            self.assertTrue(plan.command[3].endswith('/scripts/run_v9_real_fixed_relay.py'))
            self.assertEqual('--active' in plan.command, mode=='active')
            self.assertNotIn('TableTennis.py', ' '.join(plan.command))
            self.assertNotIn('ros_web_monitor', ' '.join(plan.command))
            env = environment(plan, '192.168.123.165')
            self.assertEqual(env['PYTHONPATH'],'/opt/ros/noetic/lib/python3/dist-packages')

    def test_yichao_replay_is_selected_without_control_transport(self):
        plan = build_launch('yichao','replay','/tmp/example',input_file='fixtures/two_shots.jsonl')
        self.assertFalse(plan.may_publish_control)
        self.assertFalse(plan.blockers)
        self.assertIn('199',plan.command)
        self.assertIn('replay.jsonl',plan.command[-1])

    def test_unimplemented_live_yichao_cannot_fall_back_to_fixed(self):
        plan = build_launch('yichao','active','/tmp/example')
        self.assertTrue(plan.blockers)
        self.assertEqual(plan.command,[])

    def test_yichao_shadow_selects_input_only_diagnostic(self):
        plan = build_launch('yichao','shadow','/tmp/example',subscriber_ip='192.168.123.165')
        self.assertFalse(plan.blockers)
        self.assertFalse(plan.may_publish_control)
        self.assertIn('--learned-shadow',plan.command)
        self.assertNotIn('--active',plan.command)
        with self.assertRaises(ValueError): build_launch('yichao','shadow','/tmp/example')
        private=build_launch('yichao','shadow','/tmp/example',subscriber_ip='172.16.3.126',relay_session='test')
        self.assertTrue(private.may_publish_control)
        self.assertTrue(any(x.endswith('run_runtime_relay.py') for x in private.command))
        with self.assertRaises(ValueError):
            build_launch('yichao','shadow','/tmp/example',subscriber_ip='172.16.3.126',
                         relay_session='test',input_file='fixtures/two_shots.jsonl')

    def test_observe_is_explicit_and_not_mislabelled_inference(self):
        plan = build_launch('yichao','observe','/tmp/example',subscriber_ip='192.168.123.165')
        self.assertFalse(plan.may_publish_control)
        self.assertTrue(any(x.endswith('capture_existing_inputs.py') for x in plan.command))
        with self.assertRaises(ValueError): build_launch('yichao','observe','/tmp/example')

    def test_command_publisher_conflict_includes_fixed_shadow(self):
        self.assertEqual(control_conflicts({'/torso_pose_origin':['/predictor']}),{})
        pubs={'/doubles/table_left/command':['/old_fixed_shadow']}
        self.assertEqual(control_conflicts(pubs),pubs)


if __name__ == '__main__':
    unittest.main(verbosity=2)
