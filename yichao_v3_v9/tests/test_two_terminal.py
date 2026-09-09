import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import patch
from yichao_v3_v9 import two_terminal as m


class TwoTerminalTests(TestCase):
    def test_stale_pid_and_expired_session_cannot_start_robots(self):
        with TemporaryDirectory() as d:
            root=Path(d);dest=root/'output/sessions/test';dest.mkdir(parents=True)
            p=root/'session.json'
            data={'session':'test','directory':str(dest),'root':str(root),'mode':'shadow',
                  'pid':os.getpid(),'process_start':m.process_start(os.getpid()),'created_monotonic':10.}
            p.write_text(json.dumps(data))
            with patch.object(m,'ROOT',root),patch.object(m.time,'monotonic',return_value=11.):
                self.assertEqual(m.load_live_session(p)['session'],'test')
                data['process_start']='old';p.write_text(json.dumps(data))
                with self.assertRaisesRegex(ValueError,'reused'):m.load_live_session(p)
            data['process_start']=m.process_start(os.getpid());p.write_text(json.dumps(data))
            with patch.object(m,'ROOT',root),patch.object(m.time,'monotonic',return_value=131.):
                with self.assertRaisesRegex(ValueError,'expired'):m.load_live_session(p)

    def test_active_dispatches_separate_entry(self):
        for kind in ['stack','robots']:
            with TemporaryDirectory() as temporary, patch.object(m,'ROOT',Path(temporary)), \
                    patch.object(sys,'argv',['entry',kind,'start','active']),patch.object(m,'stack') as s,patch.object(m,'robots') as r,patch('yichao_v3_v9.active_two_terminal.run',return_value=0) as active:
                self.assertEqual(m.main(),0)
                active.assert_called_once_with(kind,'start',motors_disabled=False)
                s.assert_not_called();r.assert_not_called()

    def test_ssh_existing_controller_fails_before_launch(self):
        with patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],0,'[1234]','')):
            with self.assertRaisesRegex(RuntimeError,'existing controller'):m.preflight_robot('66')
        with patch.object(m.subprocess,'run',return_value=subprocess.CompletedProcess([],1,'','denied')):
            with self.assertRaisesRegex(RuntimeError,'SSH preflight failed'):m.preflight_robot('198')

    def test_child_cleanup_only_owns_its_children(self):
        with TemporaryDirectory() as d:
            c=m.Children(Path(d))
            p=c.start('owned',[sys.executable,'-c','import time;time.sleep(30)'])
            result=c.close()
            self.assertIsNotNone(p.poll())
            self.assertEqual(result['owned']['owned_pid'],p.pid)
