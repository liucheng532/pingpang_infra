"""Recovery refuses takeover and sessions that have already started Planner."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from subprocess import CompletedProcess
from yichao_v3_v9 import ROOT

spec = importlib.util.spec_from_file_location('recover_trial', ROOT/'tools/recover_active_trial.py')
recovery = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recovery)


class RecoveryTests(unittest.TestCase):
    def test_running_input_session_cannot_be_recovered(self):
        with patch.object(recovery.active, 'config'), \
                patch.object(recovery.active, 'session', return_value={'pid': 123, 'process_start': '45'}), \
                patch.object(recovery, 'process_start', return_value='45'), \
                patch.object(recovery.subprocess, 'run') as run:
            with self.assertRaisesRegex(RuntimeError, 'still running'):
                recovery.check_recovery()
            run.assert_not_called()

    def test_previous_planner_activity_prevents_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            Path(tmp, 'relay.jsonl').write_text('previous trial\n')
            data = {'pid': 123, 'process_start': '45', 'directory': tmp}
            with patch.object(recovery.active, 'config'), \
                    patch.object(recovery.active, 'session', return_value=data), \
                    patch.object(recovery, 'process_start', side_effect=FileNotFoundError), \
                    patch.object(recovery.subprocess, 'run') as run:
                with self.assertRaisesRegex(RuntimeError, 'before Planner'):
                    recovery.check_recovery()
                run.assert_not_called()

    def test_live_policy_window_is_never_replaced(self):
        with tempfile.TemporaryDirectory() as tmp:
            data = {'pid': 123, 'process_start': '45', 'directory': tmp, 'session': 'owned'}
            results = [CompletedProcess([], 0, 'owned\n', ''),
                       CompletedProcess([], 0, 'g1-198 0\ng1-66 0\npolicy-198 0\npolicy-66 1\n', '')]
            with patch.object(recovery.active, 'config'), \
                    patch.object(recovery.active, 'session', return_value=data), \
                    patch.object(recovery, 'process_start', side_effect=FileNotFoundError), \
                    patch.object(recovery.subprocess, 'run', side_effect=results) as run, \
                    patch.object(recovery.active, 'remote') as remote:
                with self.assertRaisesRegex(RuntimeError, 'two exited policies'):
                    recovery.check_recovery()
                remote.assert_not_called()
                self.assertFalse(any('respawn-pane' in c.args[0] for c in run.call_args_list))
