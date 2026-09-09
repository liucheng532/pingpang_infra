"""Transaction identity, applied-target memory and fresh-input boundaries."""
import copy
import json
import unittest
from unittest.mock import Mock
import numpy as np

from test_runtime_shadow import ready, state
from yichao_v3_v9.runtime_shadow import STATE_TOPICS
from yichao_v3_v9.runtime_relay import RuntimeRelay
from yichao_v3_v9.command_receiver import EVIDENCE


class RuntimeRelayContractTests(unittest.TestCase):
    def setUp(self):
        self.pipeline=Mock()
        self.pipeline.home=np.array([-.35,.35])/1.2
        self.pipeline.decide.return_value={'valid':True,'target_y':[-.8,.3]}
        self.pipeline.risks.return_value=np.array([.1])
        self.pipeline.threshold=.39
        self.inputs=ready(self.pipeline)
        self.client=RuntimeRelay('synthetic',self.pipeline,self.inputs,first_hitter='198')
        self.seq=6
        for topic,slot in STATE_TOPICS.items():
            for row in self.inputs.states[slot]:
                row['payload']['yichao_relay']=self.evidence(slot)

    def evidence(self,slot,feedback=None):
        return dict(schema='yichao-onboard-relay-state-v1',session='synthetic',robot=slot,
            execution_mode='shadow',hit_enabled=True,sensors_recent=True,
            clock_workstation_interval=[10.,10.001],feedback=feedback,input_contract_accepted=False)

    def advance(self,now):
        self.seq+=1
        for topic,slot in STATE_TOPICS.items():
            raw=state(topic,self.seq,now)
            payload=json.loads(raw['payload']['data'])
            payload['yichao_relay']=self.evidence(slot)
            raw['payload']['data']=json.dumps(payload)
            self.inputs.ingest(raw)

    def ack(self,command):
        slot=command['robot'];payload=copy.deepcopy(self.inputs.states[slot][-1]['payload'])
        ack={k:command[k] for k in ('session','robot','sequence','shot','token','kind')}
        ack.update(status='accepted',completed=False,evidence=EVIDENCE,execution_mode='shadow',
                   goal_y=command['target_y'],applied_target_y=command['target_y'])
        payload['yichao_relay']=self.evidence(slot,ack)
        self.client.observe(payload)

    def start(self):
        result=self.client.tick(10.1)
        self.assertIsNone(result['reason'],result)
        self.assertEqual([c['kind'] for c in result['commands']],['move','move'])
        return result['commands']

    def test_prepare_wait_latches_targets_and_never_rebuilds_model_features(self):
        commands=self.start()
        self.inputs.features=Mock(side_effect=ValueError('base_history_gap'))
        self.inputs.handoff_features=Mock(side_effect=ValueError('base_history_gap'))
        self.pipeline.decide.return_value={'valid':True,'target_y':[-.2,.9]}
        self.inputs.ball=copy.deepcopy(self.inputs.ball)
        self.inputs.ball['diagnostic_shot_id']='synthetic/another-shot'
        self.advance(10.12)
        result=self.client.tick(10.12)
        self.assertIsNone(result['reason'])
        self.assertEqual(result['commands'],commands)
        self.pipeline.decide.assert_called_once()
        self.inputs.features.assert_not_called()
        self.inputs.handoff_features.assert_not_called()

    def test_partial_ack_updates_only_applied_slot_and_retries_original_token(self):
        commands=self.start()
        self.ack(commands[1])
        self.advance(10.12)
        result=self.client.tick(10.12)
        self.assertEqual(result['commands'],[commands[0]])
        np.testing.assert_allclose(self.client.relay.previous_targets,[-.35,.3])
        self.pipeline.decide.assert_called_once()

    def test_ack_wait_still_rejects_expired_robot_state(self):
        self.start()
        result=self.client.tick(10.36)
        self.assertEqual(result['commands'],[])
        self.assertIn('robot_state_expired',self.client.fault)

    def test_new_clear_requires_fresh_safety_features_after_both_acks(self):
        commands=self.start()
        for command in commands:self.ack(command)
        self.assertEqual(self.client.relay.state,'PREPARED')
        self.inputs.handoff_features=Mock(side_effect=ValueError('base_history_gap'))
        self.advance(10.2)
        result=self.client.tick(10.2)
        self.assertEqual(result['commands'],[])
        self.assertIn('base_history_gap',self.client.fault)
        self.inputs.handoff_features.assert_called_once()
        self.pipeline.decide.assert_called_once()

    def test_fresh_other_shot_cannot_extend_pending_hit_prediction(self):
        self.start()
        original=copy.deepcopy(self.client.shot_ball)
        self.inputs.ball=copy.deepcopy(original)
        self.inputs.ball.update(diagnostic_shot_id='synthetic/another-shot',
                                received_workstation_monotonic_s=10.41)
        self.client.relay.state='CLEARING'
        with self.assertRaisesRegex(ValueError,'latched_shot_prediction_expired'):
            self.client._shot_context(10.41,before_hit=True)
        self.assertEqual(self.client.shot_ball,original)

    def test_same_shot_refresh_keeps_actor_target_and_ages_tts(self):
        commands=self.start()
        prediction=copy.deepcopy(self.inputs.ball)
        prediction.update(received_workstation_monotonic_s=10.12,time_to_strike_s=.4)
        self.inputs.ball=prediction
        context=self.client._shot_context(10.14,before_hit=True)
        self.assertAlmostEqual(context['time_to_strike_s'],.38)
        self.assertEqual(self.client.relay.pending['66'],commands[0])
        self.pipeline.decide.assert_called_once()

    def test_poll_refuses_states_that_select_new_targets(self):
        with self.assertRaisesRegex(ValueError,'requires_features'):
            self.client.relay.poll({})

    def test_after_hit_waits_for_physical_feedback_not_old_ball_freshness(self):
        self.start()
        self.client.relay.state='COMMITTED'
        self.client.relay.last_hit_at=10.5
        self.client.relay.accepted={'66','198'}
        self.inputs.ball=None
        self.advance(15.2)
        result=self.client.tick(15.2)
        self.assertIsNone(result['reason'])
        self.assertFalse(self.client.completed)
        self.assertEqual(result['commands'],[])
        self.advance(15.51)
        result=self.client.tick(15.51)
        self.assertEqual(self.client.fault,'handoff_timeout')
        self.assertEqual(result['commands'],[])


if __name__=='__main__':unittest.main()
