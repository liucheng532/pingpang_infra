import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from yichao_v3_v9.target_feedback import TargetFeedback
from test_selector_wire import command
from test_observer import fixture, STATE


def state(c, index=1, dt=.01):
    s=fixture(STATE)
    s.update(robot='table_left' if c['robot']=='198' else 'table_right',
        sequence=index,source_monotonic_ns=1000000000+index*20000000,
        last_planner_session_id=c['session'],last_applied_sequence=c['sequence'],
        transport_error='',target_base_y=c['target_y'],base_position_xyz=[0,c['target_y'],.75])
    return s,{'monotonic_s':10.+dt,'wall_s':1000.+dt,'ros_s':1000.+dt}


class FeedbackTests(unittest.TestCase):
    def test_sequence_alone_never_means_applied(self):
        for field,value in [('transport_error','planner_command_invalid'),('target_base_y',.9),('phase','HIT')]:
            c=command();t=TargetFeedback(c['session']);t.register(c,10.)
            s,r=state(c);s[field]=value;out=t.observe('198',s,r)
            self.assertFalse(out['target_observed']);self.assertFalse(out['command_ack'])

    def test_hold_requires_continuous_position_and_speed_evidence(self):
        c=command();t=TargetFeedback(c['session']);t.register(c,10.)
        for index in range(1,8):
            s,r=state(c,index,index*.02);out=t.observe('198',s,r)
            if index==1:self.assertFalse(out['settled_observed'])
        self.assertTrue(out['settled_observed']);self.assertFalse(out['command_ack'])

    def test_return_phase_is_not_completion_and_stability_resets(self):
        c=command('return');t=TargetFeedback(c['session']);t.register(c,10.)
        s,r=state(c);s['phase']='RETURN'
        self.assertFalse(t.observe('198',s,r)['settled_observed'])
        for index in range(2,8):
            s,r=state(c,index,index*.02);t.observe('198',s,r)
        s,r=state(c,8,.16);s['base_linear_velocity_xyz'][1]=.3
        self.assertFalse(t.observe('198',s,r)['settled_observed'])

    def test_old_session_waits_but_takeover_after_observation_faults(self):
        c=command();t=TargetFeedback(c['session']);t.register(c,10.)
        s,r=state(c);s['last_planner_session_id']='someone_else'
        self.assertEqual(t.observe('198',s,r)['reason'],'waiting_for_matching_session')
        s,r=state(c,2,.04);self.assertTrue(t.observe('198',s,r)['target_observed'])
        s,r=state(c,3,.06);s['last_planner_session_id']='someone_else'
        self.assertFalse(t.observe('198',s,r)['target_observed']);self.assertIsNotNone(t.fault)

    def test_late_duplicate_and_superseded_are_rejected(self):
        for variant in ('late','duplicate','superseded'):
            c=command();t=TargetFeedback(c['session']);t.register(c,10.)
            s,r=state(c,1,.26 if variant=='late' else .01)
            if variant=='superseded':s['last_applied_sequence']+=1
            if variant=='duplicate':t.observe('198',s,r)
            self.assertFalse(t.observe('198',s,r)['target_observed'])

    def test_pair_waits_for_both_and_never_updates_from_nominal(self):
        c=command();t=TargetFeedback(c['session']);original=copy.deepcopy(c)
        t.register(c,10.);s,r=state(c);t.observe('198',s,r)
        self.assertIsNone(t.paired_targets())
        left=command(robot='66');t.register(left,10.);s,r=state(left);t.observe('66',s,r)
        self.assertEqual(t.paired_targets()['targets_y'],[-.3,.3])
        self.assertEqual(c,original)
        self.assertFalse(t.paired_targets()['command_ack'])

    def test_hit_cannot_use_movement_feedback_contract(self):
        c=command('hit');t=TargetFeedback(c['session'])
        with self.assertRaises(ValueError):t.register(c,10.)


if __name__=='__main__':unittest.main(verbosity=2)
