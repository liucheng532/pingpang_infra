"""Bounded launcher checks: no SSH, tmux mutation, ROS node, or motor access."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
from subprocess import CompletedProcess
import sys
import tempfile
import unittest
from unittest.mock import patch

from yichao_v3_v9 import ROOT
from yichao_v3_v9 import active_two_terminal as active
from yichao_v3_v9 import two_terminal


def tool_module(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'tools' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


control = tool_module('run_active_control')
readiness = tool_module('wait_active_ready')


def completed(argv=None, code=0, stdout='', stderr=''):
    return CompletedProcess(argv or [], code, stdout, stderr)


class ActiveLauncherTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.dest = self.root / 'output' / 'sessions' / 'active_test'
        self.dest.mkdir(parents=True)
        self.session = {
            'session': self.dest.name, 'directory': str(self.dest),
            'root': str(self.root), 'mode': 'active', 'tmux': active.TMUX,
            'pid': 1234, 'process_start': '9000', 'created_monotonic': 10.,
        }

    def write_session(self):
        (self.root / 'output' / 'current_active.json').write_text(json.dumps(self.session))

    def test_both_asset_checks_finish_before_first_motor_window(self):
        events = []

        def check_remote(slot, argv, timeout=30):
            self.assertEqual(argv[-1], '--check')
            self.assertNotIn('--start', argv)
            events.append(('asset_check', slot, Path(argv[2]).name))
            return '{}'

        def first_window(*args, **kwargs):
            events.append(('motor_window', args[0]))
            raise RuntimeError('stop at first mocked motor window')

        with patch.object(active, 'config', return_value={}), \
                patch.object(active, 'session', return_value=self.session), \
                patch.object(active, 'graph', return_value=({}, {})), \
                patch.object(active, 'preflight_robot'), \
                patch.object(active, 'remote', side_effect=check_remote), \
                patch.object(active.subprocess, 'run', return_value=completed(code=1)), \
                patch.object(active, 'window', side_effect=first_window):
            with self.assertRaisesRegex(RuntimeError, 'mocked motor window'):
                active.active_robots()
        self.assertEqual(events[-1][0], 'motor_window')
        self.assertEqual(set(events[:-1]), {
            ('asset_check', slot, tool)
            for slot in ('66', '198')
            for tool in ('run_active_control.py', 'run_onboard_active.py')
        })

    def test_one_occupied_robot_prevents_all_motor_window_creation(self):
        def occupied(slot):
            if slot == '66':
                raise RuntimeError('existing control process')

        with patch.object(active, 'config', return_value={}), \
                patch.object(active, 'session', return_value=self.session), \
                patch.object(active, 'graph', return_value=({}, {})), \
                patch.object(active, 'preflight_robot', side_effect=occupied), \
                patch.object(active, 'remote', return_value='{}'), \
                patch.object(active.subprocess, 'run', return_value=completed(code=1)), \
                patch.object(active, 'window') as window:
            with self.assertRaisesRegex(RuntimeError, 'existing control'):
                active.active_robots()
            window.assert_not_called()
        self.assertFalse((self.dest / 'robots_requested.json').exists())

    def test_existing_tmux_session_is_preserved_before_robot_checks(self):
        with patch.object(active.subprocess, 'run', return_value=completed()), \
                patch.object(active, 'preflight_robot') as robot, \
                patch.object(active, 'remote') as remote:
            with self.assertRaisesRegex(RuntimeError, 'already exists'):
                active.preflight_all({})
            robot.assert_not_called()
            remote.assert_not_called()

    def test_partial_startup_error_records_owned_windows_without_stopping_them(self):
        with patch.object(active, 'config', return_value={}), \
                patch.object(active, 'session', return_value=self.session), \
                patch.object(active, 'preflight_all'), \
                patch.object(active, 'window'), \
                patch.object(active, 'wait_window', side_effect=RuntimeError('readiness failed')), \
                patch.object(active, 'capture', return_value='owned startup output'), \
                patch.object(active.subprocess, 'run', return_value=completed()) as run:
            with self.assertRaisesRegex(RuntimeError, 'readiness failed'):
                active.active_robots()
        calls = [call.args[0] for call in run.call_args_list]
        self.assertFalse(any(command[1] in ('send-keys', 'kill-session') for command in calls))
        failure = json.loads((self.dest / 'startup_failure.json').read_text())
        self.assertEqual(set(failure['started_windows']), {'g1-198', 'g1-66'})
        self.assertIs(failure['motor_programs_retained'], True)
        self.assertIs(failure['physical_stop_on_exit'], False)

    def test_stale_pid_and_expired_startup_session_are_rejected(self):
        self.write_session()
        with patch.object(active, 'ROOT', self.root), \
                patch.object(active, 'process_start', return_value='different'), \
                patch.object(active.time, 'monotonic', return_value=11.):
            with self.assertRaisesRegex(ValueError, 'PID changed'):
                active.session()
        with patch.object(active, 'ROOT', self.root), \
                patch.object(active, 'process_start', return_value='9000'), \
                patch.object(active.time, 'monotonic', return_value=611.):
            with self.assertRaisesRegex(ValueError, 'expired'):
                active.session()

    def test_dead_pane_cannot_be_ready_from_previous_marker(self):
        with patch.object(active.subprocess, 'run', return_value=completed(stdout='1\n')), \
                patch.object(active, 'capture', return_value='Press R2 to calibrate'):
            with self.assertRaisesRegex(RuntimeError, 'exited'):
                active.wait_window('policy-66', 'Press R2 to calibrate', timeout=.1)

    def test_stop_without_physical_disarm_makes_no_tmux_calls(self):
        with patch.object(active.subprocess, 'run') as run:
            with self.assertRaisesRegex(ValueError, 'physically disarm'):
                active.stop(False)
            run.assert_not_called()

    def test_stop_refuses_different_tmux_owner(self):
        with patch.object(active, 'config'), \
                patch.object(active, 'session', return_value=self.session), \
                patch.object(active.subprocess, 'run', return_value=completed(stdout='another_session\n')) as run:
            with self.assertRaisesRegex(RuntimeError, 'ownership mismatch'):
                active.stop(True)
        self.assertTrue(all(call.args[0][1] not in ('send-keys', 'kill-session')
                            for call in run.call_args_list))

    def test_disarmed_partial_startup_stops_only_present_owned_windows(self):
        def tmux(argv, **kwargs):
            if argv[1] == 'show-options':
                return completed(argv, stdout='active_test\n')
            if argv[1] == 'list-windows':
                return completed(argv, stdout='g1-198\ng1-66\n')
            return completed(argv)

        with patch.object(active, 'config'), \
                patch.object(active, 'session', return_value=self.session), \
                patch.object(active, 'capture', return_value='owned driver output'), \
                patch.object(active.time, 'sleep'), \
                patch.object(active.subprocess, 'run', side_effect=tmux) as run, \
                contextlib.redirect_stdout(io.StringIO()):
            active.stop(True)
        commands = [call.args[0] for call in run.call_args_list]
        targets = {command[command.index('-t') + 1]
                   for command in commands if command[1] == 'send-keys'}
        self.assertEqual(targets, {active.TMUX + ':g1-198', active.TMUX + ':g1-66'})
        self.assertEqual(sum(command[1] == 'kill-session' for command in commands), 1)
        self.assertTrue((self.dest / 'g1-198_stop.log').exists())
        self.assertTrue((self.dest / 'g1-66_stop.log').exists())

    def test_unexpected_window_prevents_cleanup_even_with_matching_session(self):
        def tmux(argv, **kwargs):
            return completed(argv, stdout=('active_test\n' if argv[1] == 'show-options'
                                           else 'g1-198\nother_operator\n'))

        with patch.object(active, 'config'), \
                patch.object(active, 'session', return_value=self.session), \
                patch.object(active.subprocess, 'run', side_effect=tmux) as run:
            with self.assertRaisesRegex(RuntimeError, 'unexpected windows'):
                active.stop(True)
        self.assertTrue(all(call.args[0][1] not in ('send-keys', 'kill-session')
                            for call in run.call_args_list))

    def test_check_dispatch_never_calls_active_start_functions(self):
        with patch.object(active, 'config', return_value={}), \
                patch.object(active, 'preflight_all') as check, \
                patch.object(active, 'active_stack') as stack, \
                patch.object(active, 'active_robots') as robots, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(active.run('robots', 'check'), 0)
            check.assert_called_once_with({})
            stack.assert_not_called()
            robots.assert_not_called()

    def test_two_terminal_routes_active_check_without_shadow_start(self):
        argv = ['two_terminal', 'robots', 'check', 'active']
        with patch.object(two_terminal, 'ROOT', self.root), \
                patch.object(sys, 'argv', argv), \
                patch.object(two_terminal.signal, 'signal'), \
                patch.object(two_terminal, 'robots') as shadow_robots, \
                patch.object(active, 'run', return_value=0) as dispatch:
            self.assertEqual(two_terminal.main(), 0)
            dispatch.assert_called_once_with('robots', 'check', motors_disabled=False)
            shadow_robots.assert_not_called()

    def test_optional_home_reference_is_passed_only_to_right_policy(self):
        for mode in ('outward-hold','home-hold'):
            with self.subTest(mode=mode), patch.object(active,'config',return_value={}), \
                    patch.object(active,'session',return_value=self.session), \
                    patch.object(active,'preflight_all'), patch.object(active,'window') as window, \
                    patch.object(active,'wait_window'), \
                    patch.object(active,'remote',return_value=json.dumps({'pid':4321})), \
                    patch.object(active.subprocess,'run',return_value=completed()), \
                    contextlib.redirect_stdout(io.StringIO()):
                active.active_robots(right_startup=mode)
            jobs={c.args[0]:c.args[2] for c in window.call_args_list}
            self.assertNotIn('--right-startup',jobs['policy-198'])
            self.assertEqual('--right-startup' in jobs['policy-66'],mode=='home-hold')
            if mode=='home-hold':self.assertEqual(jobs['policy-66'][-2:],['--right-startup','home-hold'])
            result=json.loads((self.dest/'robots_result.json').read_text())
            self.assertFalse(result['policy_inference_waits_for_ball'])
            self.assertEqual(result['right_startup'],mode)

    def test_optional_home_selection_routes_to_active_robot_start(self):
        argv=['entry','robots','start','active','--right-startup','home-hold']
        with patch.object(two_terminal,'ROOT',self.root), patch.object(sys,'argv',argv), \
                patch.object(two_terminal.signal,'signal'), \
                patch.object(active,'run',return_value=0) as dispatch:
            self.assertEqual(two_terminal.main(),0)
        dispatch.assert_called_once_with('robots','start',motors_disabled=False,right_startup='home-hold')

    def test_home_reference_cannot_be_selected_for_shadow(self):
        with patch.object(sys,'argv',['entry','robots','start','shadow','--right-startup','home-hold']), \
                contextlib.redirect_stderr(io.StringIO()), patch.object(active,'run') as dispatch:
            with self.assertRaises(SystemExit):two_terminal.main()
        dispatch.assert_not_called()

    def test_driver_check_never_executes_binary_or_creates_control_record(self):
        with patch.object(control, 'ROOT', self.root), \
                patch.object(sys, 'argv', ['run_active_control', '--robot', '66',
                                          '--session', 'active_test', '--check']), \
                patch.object(control, 'validate', return_value={}) as validate, \
                patch('yichao_v3_v9.onboard_isolation.control_processes', return_value=[]), \
                patch.object(control.os, 'execve') as execute, \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(control.main(), 0)
            validate.assert_called_once_with(self.root, '66')
            execute.assert_not_called()
        self.assertFalse((self.root / 'output' / 'active_test').exists())


class ActiveReadinessTests(unittest.TestCase):
    @staticmethod
    def samples():
        return [(9.98, {'valid': True, 'sequence': 1, 'yichao_relay': {
                    'session': 'active_test', 'robot': '66', 'execution_mode': 'active',
                    'experimental_active': True, 'control_stage': 'policy',
                    'control_session_invalidated': False, 'sensors_recent': True}}),
                (10., {'valid': True, 'sequence': 2, 'yichao_relay': {
                    'session': 'active_test', 'robot': '66', 'execution_mode': 'active',
                    'experimental_active': True, 'control_stage': 'policy',
                    'control_session_invalidated': False, 'sensors_recent': True}})]

    def test_two_fresh_policy_samples_are_required(self):
        rows = self.samples()
        self.assertTrue(readiness.ready(rows, 'active_test', '66', 10.01))
        self.assertFalse(readiness.ready(rows[-1:], 'active_test', '66', 10.01))
        self.assertFalse(readiness.ready(rows, 'active_test', '66', 10.26))
        self.assertFalse(readiness.ready(rows, 'old_session', '66', 10.01))
        self.assertFalse(readiness.ready(rows, 'active_test', '198', 10.01))

    def test_duplicate_sequence_and_recalibration_never_establish_readiness(self):
        rows = self.samples()
        rows[1][1]['sequence'] = 1
        self.assertFalse(readiness.ready(rows, 'active_test', '66', 10.01))
        rows = self.samples()
        rows[0][1]['yichao_relay']['control_stage'] = 'calibrating'
        self.assertFalse(readiness.ready(rows, 'active_test', '66', 10.01))
        rows = self.samples()
        rows[1][1]['yichao_relay']['control_session_invalidated'] = True
        self.assertFalse(readiness.ready(rows, 'active_test', '66', 10.01))


if __name__ == '__main__':
    unittest.main()
