import unittest
from unittest.mock import Mock
from yichao_v3_v9.relay import Relay
from yichao_v3_v9.runtime_relay import RuntimeRelay
from common import features,record
import test_active_runtime as fixtures


class FirstHitterTests(unittest.TestCase):
    def setUp(self):
        self.pipeline=Mock()
        self.pipeline.decide.return_value={'valid':True,'target_y':[-.3,.8]}
        self.relay=Relay('test',[-.35,.35],first_hitter='66',handoff_timeout_s=None)

    def test_runtime_defaults_to_right_for_active_and_shadow(self):
        fixture=fixtures.ActiveRuntimeTests();fixture.setUp()
        for mode in ('active','shadow'):
            client=RuntimeRelay('synthetic',fixture.pipeline,fixture.inputs,mode=mode,
                                experimental_active=mode=='active')
            self.assertEqual(client.relay.hitter,'66')
            self.assertEqual(client.relay.next_hitter,'66')
            self.assertEqual(client.relay.snapshot()['first_hitter'],'66')

    def test_first_shot_is_right_even_when_left_is_closer_to_center(self):
        self.relay.step(features(record(positions={'66':[0,-.9,.75],'198':[0,.2,.75]})),self.pipeline)
        self.assertEqual(self.relay.hitter,'66')

    def test_finished_right_turn_hands_next_turn_to_left(self):
        self.relay.state='RETURNING';self.relay.hitter='66';self.relay.completed={'66','198'}
        self.relay.observe([])
        self.assertEqual(self.relay.state,'COMPLETE')
        self.assertEqual(self.relay.next_hitter,'198')
        self.relay.last_hit_at=10.
        self.relay.step(features(record(now=13.,shot='next')),self.pipeline)
        self.assertEqual(self.relay.hitter,'198')

    def test_idle_restart_is_right_even_when_next_turn_was_left(self):
        self.relay.state='COMPLETE';self.relay.next_hitter='198';self.relay.last_hit_at=10.
        self.relay.step(features(record(now=21.,shot='new_rally',
            positions={'66':[0,-.9,.75],'198':[0,.2,.75]})),self.pipeline)
        self.assertEqual(self.relay.hitter,'66')

    def test_missed_opening_keeps_right_for_next_incoming_ball(self):
        self.relay.step(features(),self.pipeline)
        self.relay.skip('missed_shot:commit_tts_expired')
        self.relay.step(features(record(now=11.,shot='next')),self.pipeline)
        self.assertEqual(self.relay.hitter,'66')

    def test_idle_r2_calibration_waits_then_can_prepare_right_opening(self):
        import copy
        fixture=fixtures.ActiveRuntimeTests();fixture.setUp()
        client=RuntimeRelay('synthetic',fixture.pipeline,fixture.inputs,
                            mode='active',experimental_active=True)
        for rows in fixture.inputs.states.values():
            for row in rows:row['payload']['yichao_relay']['control_stage']='calibrating'
            client.observe(copy.deepcopy(rows[-1]['payload']))
        self.assertEqual(client.tick(10.1)['commands'],[])
        self.assertIsNone(client.fault)
        for rows in fixture.inputs.states.values():
            for row in rows:row['payload']['yichao_relay']['control_stage']='policy'
            client.observe(copy.deepcopy(rows[-1]['payload']))
        result=client.tick(10.1)
        self.assertEqual(len(result['commands']),2)
        self.assertEqual(client.relay.hitter,'66')
        self.assertIsNone(client.fault)
