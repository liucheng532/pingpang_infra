import copy
import unittest
import tempfile
from pathlib import Path
from unittest.mock import Mock
import numpy as np

from yichao_v3_v9.clock_bounds import ClockBounds
from yichao_v3_v9.executor import OfflineExecutor
from yichao_v3_v9.movement_receiver import MovementReceiver, SCHEMA
from yichao_v3_v9.movement_client import MovementClient
from yichao_v3_v9.onboard_isolation import ReadOnlyLCM, control_processes


def command(slot='198', target=.23):
    return dict(schema=SCHEMA, session='movement_test', robot=slot, sequence=1,
                shot='one', token='one_move', issued_at=10., expires_at=10.25,
                target_y=target, kind='move')


class MovementTransactionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline = OfflineExecutor('movement_test')

    def setUp(self):
        from yichao_v3_v9.executor import clone_scheduler
        self.schedulers = {s: clone_scheduler(v) for s,v in self.baseline.schedulers.items()}

    def apply(self, receiver, c, **kwargs):
        return receiver.apply(c, workstation_time_interval=kwargs.get('interval', [10.01,10.02]),
                              position=self.baseline.positions[c.get('robot', '198')],
                              sensors_recent=kwargs.get('recent', True))

    def test_frozen_scheduler_accepts_both_physical_directions_once(self):
        for slot,target in [('198', .23), ('66', -.9425)]:
            scheduler = self.schedulers[slot]
            receiver = MovementReceiver('movement_test', slot, scheduler)
            c = command(slot, target)
            ack = self.apply(receiver, c)
            self.assertEqual(ack['status'], 'accepted', ack)
            self.assertAlmostEqual(ack['applied_target_y'], target, places=4)
            self.assertEqual(self.apply(receiver,c), ack)
            altered = {**c, 'target_y': target+.001}
            self.assertEqual(self.apply(receiver,altered)['reason'], 'sequence_payload_conflict')

    def test_invalid_fields_and_hit_never_mutate_scheduler(self):
        for patch in [dict(kind='hit'), dict(target_y=float('nan')), dict(target_y=1.3),
                      dict(target_y=.4), dict(session='previous_session'), dict(sequence=True),
                      dict(expires_at=11.), dict(extra=1)]:
            scheduler = self.schedulers['198']
            receiver = MovementReceiver('movement_test','198',scheduler)
            before = copy.deepcopy(scheduler.output())
            ack = self.apply(receiver,{**command(), **patch})
            self.assertEqual(ack['status'],'rejected',ack)
            np.testing.assert_array_equal(before.target_base,scheduler.output().target_base)
            self.assertFalse(receiver.history)
        for kwargs in [dict(interval=[9.9,9.91]),dict(interval=[10.24,10.26]),
                       dict(interval=[10.01,10.1]),dict(recent=False)]:
            r=MovementReceiver('movement_test','198',self.schedulers['198'])
            self.assertEqual(self.apply(r,command(),**kwargs)['status'],'rejected')

    def test_completion_requires_stability_and_stays_terminal_on_retransmission(self):
        # A zero-displacement request is enough to exercise the actual scheduler
        # HOLD path; it must still wait for measured stability, not just an ACK.
        r=MovementReceiver('movement_test','198',self.schedulers['198'])
        c=command(target=.2)
        self.assertEqual(self.apply(r,c)['status'],'accepted')
        for t in [10.02,10.04,10.06,10.08,10.10]:
            ack=r.observe(now=t,position=[0,.2,.75],velocity_y=0,sensors_recent=True)
            self.assertFalse(ack['completed'])
        ack=r.observe(now=10.12,position=[0,.2,.75],velocity_y=0,sensors_recent=True)
        self.assertTrue(ack['completed'])
        self.assertEqual(self.apply(r,c,interval=[11.,11.01])['status'],'completed')
        self.assertTrue(r.observe(now=11.,position=[0,.3,.75],velocity_y=1,sensors_recent=False)['completed'])

    def test_pending_timeout_does_not_depend_on_live_ball_features(self):
        client=MovementClient('movement_test',Mock())
        client.pending=command()
        self.assertEqual(client.poll(10.1)['command'],command())
        self.assertIsNone(client.poll(10.26)['command'])
        self.assertIn('feedback_timeout',client.fault)

    def test_clock_expiry_jump_and_uncertainty(self):
        c=ClockBounds()
        c.update(10.,110.001,110.002,10.004)
        lo,hi=c.workstation_interval(110.05)
        self.assertLess(lo,10.05)
        self.assertGreater(hi,10.05)
        with self.assertRaises(ValueError): c.workstation_interval(112.)
        with self.assertRaises(ValueError): c.update(11.,115.,115.001,11.004)
        with self.assertRaises(ValueError): ClockBounds().update(10.,110.,110.001,10.1)

    def test_shadow_lcm_rejects_every_publication(self):
        raw=Mock(); bus=ReadOnlyLCM(raw)
        bus.subscribe('body_control_data',lambda x: None)
        raw.subscribe.assert_called_once()
        for channel in ['pd_plustau_targets','unexpected']:
            with self.assertRaises(RuntimeError): bus.publish(channel,b'data')
        raw.publish.assert_not_called()

    def test_existing_policy_process_blocks_another_onboard_shadow(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for pid,args in [(900001,b'python3\0/x/deploy_policy.py\0--shadow\0'),
                             (900002,b'/x/g1_control\0eth0\0'),
                             (900003,b'python3\0/x/run_onboard_movement.py\0'),
                             (900004,b'python3\0/x/inspect_deploy_policy.py\0')]:
                p=root/str(pid);p.mkdir();(p/'cmdline').write_bytes(args)
            self.assertEqual(control_processes(root),[900001,900002,900003])


if __name__ == '__main__': unittest.main()
