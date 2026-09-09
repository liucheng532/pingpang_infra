"""Experimental active admission uses measured policy readiness, never fake acceptance."""
import copy
import unittest
from unittest.mock import Mock

import numpy as np

from test_runtime_shadow import ready
from yichao_v3_v9.runtime_relay import RuntimeRelay


class ActiveRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.pipeline = Mock()
        self.pipeline.home = np.array([-.35, .35]) / 1.2
        self.pipeline.decide.return_value = {'valid': True, 'target_y': [-.8, .3]}
        self.pipeline.risks.return_value = np.array([.1])
        self.pipeline.threshold = .39
        self.inputs = ready(self.pipeline)
        for slot, rows in self.inputs.states.items():
            for row in rows:
                row['payload']['yichao_relay'] = self.evidence(slot)
        # Gate failures must precede feature construction, rather than happen
        # accidentally because a mutated readiness fixture has a history gap.
        self.inputs.features = Mock(wraps=self.inputs.features)

    @staticmethod
    def evidence(slot):
        return {
            'schema': 'yichao-onboard-relay-state-v1',
            'session': 'synthetic', 'robot': slot,
            'execution_mode': 'active', 'hit_enabled': True,
            'sensors_recent': True, 'clock_workstation_interval': [10., 10.001],
            'feedback': None, 'input_contract_accepted': False,
            'experimental_active': True, 'control_stage': 'policy',
            'control_session_invalidated': False,
        }

    def client(self, **kwargs):
        # Existing protocol fixtures exercise a left opening; new defaults are
        # covered separately by the right-first tests and scheduler replay.
        kwargs.setdefault('first_hitter','198')
        return RuntimeRelay('synthetic', self.pipeline, self.inputs,
                            mode='active', **kwargs)

    def assert_admission_rejected(self, client, now=10.1):
        result = client.tick(now)
        self.assertEqual(result['commands'], [])
        self.assertIsNotNone(result['reason'])
        self.pipeline.decide.assert_not_called()
        self.inputs.features.assert_not_called()
        self.assertIs(result['real_input_accepted'], False)
        return result

    def test_default_constructor_does_not_treat_trial_evidence_as_acceptance(self):
        self.assert_admission_rejected(self.client())

    def test_one_waiting_robot_prevents_actor_and_target_commands(self):
        self.inputs.states['66'][-1]['payload']['yichao_relay']['control_stage'] = 'waiting_r2'
        self.assert_admission_rejected(self.client(experimental_active=True))

    def test_both_policy_samples_and_explicit_trial_produce_two_moves(self):
        result = self.client(experimental_active=True).tick(10.1)
        self.assertIsNone(result['reason'], result)
        self.assertEqual([(c['robot'], c['kind']) for c in result['commands']],
                         [('66', 'move'), ('198', 'move')])
        self.pipeline.decide.assert_called_once()
        self.assertIs(result['real_input_accepted'], False)
        for rows in self.inputs.states.values():
            self.assertIs(rows[-1]['payload']['yichao_relay']['input_contract_accepted'], False)

    def test_robot_cannot_join_trial_without_its_explicit_opt_in(self):
        self.inputs.states['198'][-1]['payload']['yichao_relay'].pop('experimental_active')
        self.assert_admission_rejected(self.client(experimental_active=True))

    def test_single_policy_transition_sample_is_insufficient(self):
        self.inputs.states['198'][-2]['payload']['yichao_relay']['control_stage'] = 'calibrating'
        self.assert_admission_rejected(self.client(experimental_active=True))

    def test_stale_penultimate_state_cannot_establish_two_sample_readiness(self):
        self.inputs.states['66'][-2]['receipt'] = 9.8
        self.assert_admission_rejected(self.client(experimental_active=True))

    def test_missing_penultimate_state_cannot_establish_readiness(self):
        rows = self.inputs.states['66']
        self.inputs.states['66'] = type(rows)([rows[-1]], maxlen=rows.maxlen)
        self.assert_admission_rejected(self.client(experimental_active=True))

    def test_invalidated_policy_session_does_not_resume_from_ready_label(self):
        self.inputs.states['66'][-1]['payload']['yichao_relay']['control_session_invalidated'] = True
        self.assert_admission_rejected(self.client(experimental_active=True))

    def test_stage_loss_after_move_faults_without_new_hit_or_replanning(self):
        client = self.client(experimental_active=True)
        first = client.tick(10.1)
        self.assertEqual([c['kind'] for c in first['commands']], ['move', 'move'])
        previous = copy.deepcopy(client.relay.pending)
        self.inputs.states['198'][-1]['payload']['yichao_relay']['control_stage'] = 'calibrating'
        result = client.tick(10.12)
        self.assertEqual(result['commands'], [])
        self.assertIsNotNone(client.fault)
        self.assertEqual(client.relay.state, 'FAULT')
        self.assertEqual(client.relay.pending, previous)
        self.pipeline.decide.assert_called_once()
        self.assertIs(result['real_input_accepted'], False)
        self.assertIs(result['existing_motion_stops_on_exit'], False)


if __name__ == '__main__':
    unittest.main()
