import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from yichao_v3_v9.run_log import RunLog


class RunLogTests(unittest.TestCase):
    def test_rotates_and_keeps_detached_records_and_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'planner.jsonl'
            with RunLog(path,{'session':'test'},segment_bytes=500) as log:
                for i in range(30):
                    record={'kind':'sample','values':[i]}
                    log(record)
                    record['values'][0]=-1
            rows=[json.loads(line) for p in sorted(path.parent.glob('planner.jsonl*'))
                  for line in p.read_text().splitlines()]
            self.assertEqual([r['values'][0] for r in rows if r['kind']=='sample'],list(range(30)))
            self.assertEqual(rows[0]['kind'],'run_start')
            self.assertEqual(rows[-1]['log_write_errors'],0)
            self.assertEqual(rows[-1]['log_dropped_records'],0)

    def test_exception_still_records_exit_and_trace(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'planner.jsonl'
            with self.assertRaisesRegex(ValueError,'test failure'):
                with RunLog(path,{}) as log:raise ValueError('test failure')
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(rows[-1]['exit_reason'],'error')
            self.assertIn('test failure',rows[-2]['traceback'])

    def test_disk_write_error_does_not_escape_into_control(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'planner.jsonl'
            with patch('yichao_v3_v9.run_log.SegmentedWriter.__call__',side_effect=OSError('disk full')):
                with RunLog(path,{}) as log:
                    log({'kind':'planner_decision'})
                    log.pending.join()
                    self.assertGreater(log.write_errors,0)
