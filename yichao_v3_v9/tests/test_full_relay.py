import copy
import unittest
import numpy as np
from unittest.mock import Mock
from yichao_v3_v9.command_receiver import CommandReceiver,EVIDENCE
from yichao_v3_v9.executor import OfflineExecutor,clone_scheduler
from yichao_v3_v9.relay import Relay
from common import command,features,record


class FullRelayTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.baseline=OfflineExecutor('test')

    def setUp(self):
        self.schedulers={s:clone_scheduler(v) for s,v in self.baseline.schedulers.items()}
        self.receivers={s:CommandReceiver('test',s,v) for s,v in self.schedulers.items()}

    def apply(self,c,now=None,position=None):
        now=c['issued_at'] if now is None else now
        return self.receivers[c['robot']].apply(c,workstation_time_interval=[now,now+.001],
            position=self.baseline.positions[c['robot']] if position is None else position,sensors_recent=True)

    def test_real_scheduler_prepare_hit_and_return_both_robot_roles(self):
        for slot in ('66','198'):
            sign=-1 if slot=='66' else 1
            initial=self.baseline.positions[slot].copy()
            prepare=command(slot,'move',float(initial[1]),session='test')
            self.assertEqual(self.apply(prepare)['status'],'accepted')
            hit=command(slot,'hit',float(initial[1]),seq=2,session='test',now=10.08)
            hit['hit']['outward_y']=sign*1.0
            ack=self.apply(hit)
            self.assertEqual(ack['status'],'accepted',ack)
            self.assertEqual(self.schedulers[slot].state.name,'HIT')
            self.assertEqual(self.apply(hit,now=11.)['status'],'accepted')
            self.assertEqual(len(self.receivers[slot].hit_shots),1)
            return_cmd=command(slot,'return',sign*.2,seq=3,session='test',now=10.1)
            self.assertEqual(self.apply(return_cmd)['reason'],'hit_locked')
            # Synthetic pose reaches the outward goal; run the real scheduler
            # through HIT, POST_DELAY and the motion-bank reference.
            pos=np.array([0.,sign*1.,.75],np.float32)
            for i in range(250):
                self.schedulers[slot].update(-.5,np.zeros(3),np.zeros(3),pos,pos,np.zeros(3),np.zeros(3),dt=.02)
                self.receivers[slot].observe(now=10.1+i*.02,position=pos,velocity_y=0,sensors_recent=True)
            self.assertTrue(self.receivers[slot].active['completed'])
            return_cmd.update(issued_at=15.2,expires_at=15.45)
            ack=self.apply(return_cmd,position=pos)
            self.assertEqual(ack['status'],'accepted',ack)
            self.assertEqual(ack['phase'],'RETURN')
            self.assertFalse(ack['completed'])

    def test_invalid_hit_is_never_partially_applied_and_requires_matching_prepare(self):
        c=command('198','hit',.2,session='test')
        self.assertEqual(self.apply(c)['reason'],'matching_prepare_required')
        self.apply(command('198','move',.2,session='test'))
        c.update(sequence=2,token='hit2')
        c['hit']['return_y']=float('nan')
        self.assertEqual(self.apply(c)['status'],'rejected')
        self.assertEqual(self.schedulers['198'].state.name,'HOME_HOLD')
        self.assertFalse(self.receivers['198'].hit_shots)

    def test_active_accepts_next_shot_after_missed_prepare_or_settled_return(self):
        receiver=self.receivers['198']
        receiver.allow_prepare_preemption=True
        self.assertEqual(self.apply(command(target=.3))['status'],'accepted')
        next_move=command(target=.4,seq=2)
        next_move['shot']='next'
        self.assertEqual(self.apply(next_move)['status'],'accepted')
        # A return reference has finished, but its measured target completion
        # is still false (the real undershoot case).
        receiver.active['kind']='return'
        receiver.active['completed']=False
        self.schedulers['198'].state=type(self.schedulers['198'].state).HOME_HOLD
        later=command(target=.3,seq=3);later['shot']='later'
        self.assertEqual(self.apply(later)['status'],'accepted')

    def test_active_next_shot_cannot_preempt_unfinished_return_motion(self):
        receiver=self.receivers['198'];receiver.allow_prepare_preemption=True
        self.apply(command(target=.4))
        receiver.active['kind']='return'
        self.schedulers['198'].state=type(self.schedulers['198'].state).RETURN
        later=command(target=.3,seq=2);later['shot']='later'
        self.assertEqual(self.apply(later)['reason'],'previous_relay_not_completed')

    def test_large_monotonic_clock_and_reference_pose_skew(self):
        c=command('198','move',.3,session='test',now=10000000.125)
        # The new measured pelvis can differ from the scheduler's preceding
        # frame. It must still latch absolute Y without accidentally asking X.
        ack=self.apply(c,position=[.003,.205,.75])
        self.assertEqual(ack['status'],'accepted',ack)
        self.assertAlmostEqual(ack['applied_target_y'],.3)

    def test_training_reservation_commits_during_locomotion_after_explicit_ack(self):
        pipeline=Mock()
        pipeline.decide.return_value={'valid':True,'target_y':[-.8,.3]}
        pipeline.risks.return_value=np.array([.1]);pipeline.threshold=.39
        relay=Relay('test',[-.2,.2],feedback_evidence=EVIDENCE,training_timing=True)
        f=features();commands=relay.step(f,pipeline)
        for c in commands:relay.observe([self.apply(c)])
        self.assertEqual(relay.state,'PREPARED')
        self.assertFalse(relay.completed)
        f.record['now']=10.06
        self.assertEqual(relay.step(f,pipeline),[])
        f.record['now']=10.08
        for slot,y in [('66',-.88),('198',.22)]:
            pos=np.array([0.,y,.75],np.float32)
            self.schedulers[slot].update(-.5,np.zeros(3),np.zeros(3),pos,pos,np.zeros(3),np.zeros(3),dt=.02)
            f.record['robots'][slot]['phase']=self.schedulers[slot].state.name
            f.record['robots'][slot]['base_position']=pos.tolist()
        emitted=relay.step(f,pipeline)
        self.assertEqual([c['kind'] for c in emitted],['clear'])
        self.assertEqual(relay.state,'CLEARING')
        for c in emitted:
            ack=self.apply(c)
            self.assertEqual(ack['status'],'accepted',ack)
            relay.observe([ack])
        f.record['now']=10.1
        emitted=relay.step(f,pipeline)
        self.assertEqual([c['kind'] for c in emitted],['hit'])
        self.assertEqual(relay.state,'COMMITTED')
        self.assertEqual(self.apply(emitted[0])['status'],'accepted')
        self.assertEqual(len(self.receivers['198'].hit_shots),1)

    def test_peer_clear_rejection_prevents_hit(self):
        pipeline=Mock()
        pipeline.decide.return_value={'valid':True,'target_y':[-.8,.3]}
        pipeline.risks.return_value=np.array([.1]);pipeline.threshold=.39
        relay=Relay('test',[-.2,.2],feedback_evidence=EVIDENCE,training_timing=True)
        f=features()
        for c in relay.step(f,pipeline):relay.observe([self.apply(c)])
        f.record['now']=10.08
        for slot,y in [('66',-.88),('198',.22)]:
            pos=np.array([0.,y,.75],np.float32)
            self.schedulers[slot].update(-.5,np.zeros(3),np.zeros(3),pos,pos,np.zeros(3),np.zeros(3),dt=.02)
            f.record['robots'][slot]['phase']=self.schedulers[slot].state.name
            f.record['robots'][slot]['base_position']=pos.tolist()
        commands=relay.step(f,pipeline)
        self.assertEqual([c['kind'] for c in commands],['clear'])
        # A robot can reject after preparation, e.g. when its sensor lease
        # expires. The other robot must never receive a HIT in that case.
        c=commands[0]
        ack=self.receivers[c['robot']].apply(c,workstation_time_interval=[10.08,10.081],
            position=self.baseline.positions[c['robot']],sensors_recent=False)
        self.assertEqual(ack['status'],'rejected')
        relay.observe([ack])
        self.assertEqual(relay.state,'FAULT')
        f.record['now']=10.1
        self.assertEqual(relay.step(f,pipeline),[])
        self.assertTrue(all(not receiver.hit_shots for receiver in self.receivers.values()))


if __name__=='__main__':unittest.main()
