import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock
import test_active_runtime as fixture
from yichao_v3_v9.runtime_relay import RuntimeRelay
from yichao_v3_v9.runtime_recovery import restore_handoff


class ActiveContinuityTests(unittest.TestCase):
    def setUp(self):
        self.fixture = fixture.ActiveRuntimeTests()
        self.fixture.setUp()
        self.client = self.fixture.client(experimental_active=True, shots=0)

    def advance(self,seconds,new_shot=False):
        for rows in self.fixture.inputs.states.values():
            for row in rows:row['receipt']+=seconds
        if new_shot:
            self.fixture.inputs.ball['received_workstation_monotonic_s']+=seconds
            self.fixture.inputs.ball['diagnostic_shot_id']+='-next'

    def test_missed_prediction_resets_then_accepts_next_ball(self):
        first=self.client.tick(10.1)
        self.assertEqual(len(first['commands']),2)
        self.advance(.4)
        missed=self.client.tick(10.5)
        self.assertEqual(missed['commands'],[])
        self.assertIsNone(self.client.fault)
        self.assertEqual(self.client.relay.skipped_count,1)
        self.advance(.02,new_shot=True)
        self.fixture.inputs.ball['received_workstation_monotonic_s']=10.52
        self.assertEqual(len(self.client.tick(10.52)['commands']),2)

    def test_fresh_history_recovers_after_gap_without_latching_fault(self):
        self.client.tick(10.1)
        self.client.relay.state='PREPARED'
        original=self.fixture.inputs.handoff_features
        self.fixture.inputs.handoff_features=Mock(side_effect=ValueError('base_history_gap'))
        self.advance(.1)
        self.assertEqual(self.client.tick(10.2)['commands'],[])
        self.assertIsNone(self.client.fault)
        self.fixture.inputs.handoff_features=original
        result=self.client.tick(10.2)
        self.assertIsNone(self.client.fault,result)
        self.assertEqual([c['kind'] for c in result['commands']],['clear'])

    def test_slow_decision_is_logged_and_commands_remain_available(self):
        import time
        def slow(features):
            time.sleep(.025)
            return {'valid':True,'target_y':[-.8,.3]}
        self.fixture.pipeline.decide.side_effect=slow
        result=self.client.tick(10.1)
        self.assertEqual(len(result['commands']),2)
        self.assertIsNone(self.client.fault)
        self.assertIn('slow_planner_step',[e['kind'] for e in result['events']])
        event=next(e for e in result['events'] if e['kind']=='planner_input')
        self.assertEqual(len(event['actor_observation']),36)
        self.assertEqual(len(event['safe_observation']),85)

    def test_late_hit_retry_keeps_original_packet_and_does_not_repeat_execution(self):
        from yichao_v3_v9.relay import Relay
        from common import command
        relay=Relay('test',[-.2,.2],handoff_timeout_s=None)
        hit=command('198','hit',.2)
        relay.state='COMMITTED';relay.pending={'198':hit};relay.last_hit_at=10.
        relay.shot=hit['shot']
        self.assertEqual(relay.poll({'now':30.,'shot_id':hit['shot']}),[hit])

    def test_return_expiry_renews_only_return_and_continues(self):
        from common import command
        relay=self.client.relay
        old=command('198','return',.2,session='synthetic')
        old['return_y']=.2
        relay.state='RETURNING';relay.pending={'198':old};relay.shot=old['shot']
        relay.decision={'target_y':[-.8,.2]};relay.last_hit_at=10.
        ack={k:old[k] for k in ('session','robot','sequence','shot','token','kind')}
        ack.update(status='rejected',reason='expired_or_future_command',evidence=relay.feedback_evidence)
        relay.observe([ack])
        result=relay.poll({'now':30.,'shot_id':old['shot']})
        self.assertEqual([c['kind'] for c in result],['return'])
        self.assertGreater(result[0]['expires_at'],30.)
        self.assertNotEqual(result[0]['token'],old['token'])
        self.assertIsNone(relay.fault)

    def test_posthit_input_gap_waits_then_recovers(self):
        self.client.tick(10.1)
        relay=self.client.relay
        relay.state='COMMITTED';relay.accepted={'66','198'};relay.last_hit_at=10.1
        self.assertEqual(self.client.tick(11.)['commands'],[])
        self.assertIsNone(self.client.fault)
        self.advance(.9)
        self.assertIsNone(self.client.tick(11.)['reason'])

    def test_input_parser_resumes_after_sensor_wait_and_expired_command_report(self):
        from test_runtime_shadow import state
        inputs=self.fixture.inputs;inputs.continuous=True
        topic='/doubles/table_right/state'
        for seq,now,recent,error in [(7,10.12,False,''),(8,10.14,True,'movement_rejected:expired_or_future_command')]:
            record=state(topic,seq,now)
            payload=json.loads(record['payload']['data'])
            payload['yichao_relay']=self.fixture.evidence('66')
            payload['yichao_relay'].update(sensors_recent=recent,feedback={'status':'rejected'})
            payload['transport_error']=error
            record['payload']['data']=json.dumps(payload)
            inputs.ingest(record)
            self.assertIsNone(inputs.fault)
        self.assertEqual(inputs.states['66'][-1]['receipt'],10.14)

    def test_return_reference_hold_allows_next_turn_without_fake_arrival(self):
        relay=self.client.relay
        relay.state='RETURNING';relay.accepted={'66','198'};relay.completed={'66'};relay.shot='shot'
        robots={s:{'base_position':[0,y,.8],'base_velocity':[0,0,0],'phase':'HOME_HOLD'}
                for s,y in [('66',-.3),('198',.7)]}
        self.client._outward_release(robots,10.)
        self.client._outward_release(robots,10.11)
        self.assertEqual(relay.state,'COMPLETE')
        self.assertEqual(relay.completed,{'66'})
        self.assertEqual(relay.next_hitter,'66')

    def test_continuous_active_keeps_waiting_after_five_seconds(self):
        self.client.tick(10.1)
        self.client.relay.state = 'COMMITTED'
        self.client.relay.last_hit_at = 10.2
        self.client.relay.accepted = {'66', '198'}
        for rows in self.fixture.inputs.states.values():
            for i, row in enumerate(list(rows)[-2:]):row['receipt'] = 20.+i*.02
        self.fixture.inputs.ball = None
        result = self.client.tick(20.02)
        self.assertIsNone(result['reason'])
        self.assertEqual(result['commands'], [])
        self.assertFalse(self.client.completed)
        self.assertIsNone(self.client.fault)

    def test_reference_end_release_does_not_fabricate_target_completion(self):
        relay = self.client.relay
        relay.state = 'COMMITTED';relay.accepted = {'66', '198'};relay.completed = {'198'}
        robots = {s: {'base_position':[0, y, .8], 'base_velocity':[0, .01, 0],
                      'phase':'OUTWARD_HOLD'} for s,y in [('66', -.85), ('198', .89)]}
        self.client._outward_release(robots, 10.)
        self.assertEqual(relay.state, 'COMMITTED')
        self.client._outward_release(robots, 10.11)
        self.assertEqual(relay.state, 'NEED_RETURN')
        self.assertEqual(relay.completed, {'198'})
        self.assertEqual(self.client.outward_release['outward_target_completed'], ['198'])

    def test_release_requires_stable_separation_and_both_applied_receipts(self):
        relay = self.client.relay;relay.state = 'COMMITTED';relay.accepted = {'198'}
        robots = {s: {'base_position':[0, y, .8], 'base_velocity':[0, 0, 0],
                      'phase':'OUTWARD_HOLD'} for s,y in [('66', -.85), ('198', .89)]}
        self.client._outward_release(robots, 10.)
        self.client._outward_release(robots, 11.)
        self.assertEqual(relay.state, 'COMMITTED')
        relay.accepted.add('66');robots['66']['phase']='HIT'
        self.client._outward_release(robots, 12.)
        self.assertEqual(relay.state, 'COMMITTED')
        robots['66']['phase']='OUTWARD_HOLD';robots['198']['base_position'][1]=-.7
        self.client._outward_release(robots, 13.)
        self.assertEqual(relay.state, 'COMMITTED')

    def test_recovery_cannot_bypass_warmup_or_reissue_old_hit(self):
        self.client.relay.state = 'COMMITTED';self.client.resuming=True
        self.fixture.inputs.states = {'66': [], '198': []}
        result=self.client.tick(100.)
        self.assertEqual(result['commands'], [])
        self.assertIsNone(self.client.fault)
        self.assertTrue(self.client.resuming)

    def test_only_posthit_timeout_logs_can_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'relay.jsonl'
            p.write_text(json.dumps({'kind':'summary','mode':'active','fault':'prepare_timeout'})+'\n')
            with self.assertRaisesRegex(ValueError,'only_logged_handoff_timeout'):
                restore_handoff(self.client,p)

    def test_aged_context_is_only_allowed_for_explicit_handoff(self):
        inputs=self.fixture.inputs
        now=10.1
        old=copy.deepcopy(inputs.ball)
        old['received_workstation_monotonic_s']=0.1
        with self.assertRaisesRegex(ValueError,'ball_receive_age'):
            inputs.handoff_features(now,old)
        features=inputs.handoff_features(now,old,context_max_age_s=None)
        self.assertTrue(features.record['handoff_only'])
        self.assertEqual(features.record['ball_receive_monotonic_s'],0.1)
