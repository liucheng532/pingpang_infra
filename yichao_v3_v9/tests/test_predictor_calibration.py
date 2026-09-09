"""Regression for daily calibration changes without opening ROS or the SDK."""
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import Mock, patch

from yichao_v3_v9 import ROOT
from yichao_v3_v9.predictor_calibration import capture_calibration


def payload(left=0, right=1, version='2026-09-09T22:52:07+08:00'):
    calibrations = {'table': {
        'type': 'table', 'parameter_name': 'TABLE_IN_WORLD',
        'transform': 'table_to_world',
        'matrix_4x4': [[1, 0, 0, 2], [0, 1, 0, 3], [0, 0, 1, 4], [0, 0, 0, 1]],
    }}
    for side, rigid_id in [('left', left), ('right', right)]:
        calibrations['double_'+side+'_robot_tracker'] = {
            'type': 'robot_tracker', 'side': side, 'rigid_id': rigid_id,
            'tracker_in_torso': {'transform': 'tracker_to_torso',
                                'translation_m': [0.1, 0.2, 0.3],
                                'quaternion_xyzw': [0, 0, 0, 1]},
        }
    return {'schema': 'pingpang.runtime_calibration', 'schema_version': 1,
            'updated_at': version, 'calibrations': calibrations}


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.snapshot = self.root/'predictor_snapshot'
        (self.snapshot/'calib').mkdir(parents=True)
        # Use the actual frozen Predictor validator on either development or
        # workstation checkout; do not replace its schema checks with a mock.
        deployed = ROOT/'predictor_snapshot_20260908'
        if not deployed.exists():
            deployed = (ROOT.parent/'odl@172.16.3.126/home/odl/codebase/'
                        'yichao_v3_v9_e14fd5b/predictor_snapshot_20260908')
        shutil.copyfile(deployed/'calib/runtime_calibration.py',
                        self.snapshot/'calib/runtime_calibration.py')
        self.old = self.snapshot/'calibration_config.json'
        self.old.write_text(json.dumps(payload(1, 0, '2026-09-08T16:45:26+08:00')))
        self.source = self.root/'shared_calibration.json'
        self.source.write_text(json.dumps(payload()))

    def capture(self, session):
        dest = self.root/session
        dest.mkdir()
        return capture_calibration(self.snapshot, self.source, dest)

    def test_next_launch_picks_new_calibration_existing_session_keeps_exact_copy(self):
        first_bytes = self.source.read_bytes()
        old_bytes = self.old.read_bytes()
        first = self.capture('first')
        self.assertEqual(first['trackers']['198']['rigid_id'], 0)
        self.assertEqual(first['trackers']['66']['rigid_id'], 1)
        self.assertEqual(first['sha256'], hashlib.sha256(first_bytes).hexdigest())
        self.assertEqual(first['table']['matrix_4x4'], payload()['calibrations']['table']['matrix_4x4'])
        changed = payload(5, 7, '2026-09-10T10:00:00+08:00')
        changed['calibrations']['table']['matrix_4x4'][0][3] = 9
        changed['calibrations']['double_right_robot_tracker']['tracker_in_torso']['translation_m'][0] = .6
        self.source.write_text(json.dumps(changed))
        second = self.capture('second')
        self.assertEqual(second['trackers']['66']['rigid_id'], 7)
        self.assertEqual(second['trackers']['66']['translation_m'][0], .6)
        self.assertEqual(second['table']['matrix_4x4'][0][3], 9)
        self.assertEqual(second['updated_at'], changed['updated_at'])
        self.assertNotEqual(first['sha256'], second['sha256'])
        self.assertEqual(Path(first['snapshot_path']).read_bytes(), first_bytes)
        self.assertEqual(Path(second['snapshot_path']).read_bytes(), self.source.read_bytes())
        self.assertEqual(self.old.read_bytes(), old_bytes)
        self.assertEqual(json.loads((self.root/'first/calibration_provenance.json').read_text()), first)

    def test_missing_shared_config_does_not_fall_back_to_old_snapshot(self):
        self.source.unlink()
        with self.assertRaises(FileNotFoundError):
            self.capture('missing')
        self.assertFalse((self.root/'missing/calibration_config.json').exists())

    def test_invalid_configs_are_rejected_before_writing_run_copy(self):
        cases = [('invalid_json', '{'), ('duplicate_ids', json.dumps(payload(0, 0)))]
        missing = payload()
        del missing['calibrations']['table']
        cases.append(('missing_table', json.dumps(missing)))
        missing = payload()
        del missing['calibrations']['double_right_robot_tracker']
        cases.append(('missing_tracker', json.dumps(missing)))
        for name, blob in cases:
            with self.subTest(name=name):
                self.source.write_text(blob)
                with self.assertRaises((ValueError, RuntimeError)):
                    self.capture(name)
                self.assertFalse((self.root/name/'calibration_config.json').exists())

    def test_snapshot_path_cannot_be_used_as_shared_source(self):
        self.source = self.old
        with self.assertRaisesRegex(ValueError, 'outside the code snapshot'):
            self.capture('old')

    def test_launcher_passes_current_copy_and_records_provenance_before_spawn(self):
        spec = importlib.util.spec_from_file_location('predictor_launcher', ROOT/'tools/run_predictor_snapshot.py')
        launcher = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(launcher)
        (self.root/'config').mkdir()
        files = [self.old, self.snapshot/'calib/runtime_calibration.py']
        manifest = {'snapshot_root': str(self.snapshot), 'calibration_source': str(self.source),
                    'sha256': {str(p.relative_to(self.root)): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in files}}
        (self.root/'config/predictor_snapshot.json').write_text(json.dumps(manifest))
        observed = []

        def spawn(_command, **kwargs):
            config_path = Path(kwargs['env']['PINGPANG_CALIBRATION_CONFIG'])
            run = json.loads((kwargs['cwd']/'run.json').read_text())
            self.assertEqual(config_path, kwargs['cwd']/'calibration_config.json')
            self.assertEqual(config_path.read_bytes(), self.source.read_bytes())
            self.assertEqual(run['calibration']['snapshot_path'], str(config_path))
            self.assertEqual(run['calibration']['trackers']['66']['rigid_id'], 1)
            observed.append(run)
            return Mock(pid=123, returncode=0, poll=Mock(return_value=0))

        with patch.object(launcher, 'ROOT', self.root), \
                patch.object(launcher, 'graph', return_value=({}, {})), \
                patch.object(Path, 'glob', return_value=[]), \
                patch.object(launcher.signal, 'signal'), \
                patch.object(launcher.subprocess, 'Popen', side_effect=spawn) as popen, \
                patch.object(launcher.sys, 'argv', ['launcher', '--session', 'calibration_test']), \
                patch.dict('os.environ', {'PINGPANG_CALIBRATION_CONFIG': str(self.old)}):
            self.assertEqual(launcher.main(), 0)
            self.assertEqual(popen.call_count, 1)
            self.assertEqual(len(observed), 1)
            self.source.unlink()
            with patch.object(launcher.sys, 'argv', ['launcher', '--session', 'missing_test']):
                with self.assertRaises(FileNotFoundError):
                    launcher.main()
            self.assertEqual(popen.call_count, 1)
