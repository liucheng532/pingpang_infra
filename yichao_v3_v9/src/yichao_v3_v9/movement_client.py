"""Single-robot movement pilot: actor/filter -> target latch -> explicit ACK.

The model retains its full workspace. A pilot step outside five centimetres is
rejected, never silently clipped. The stationary peer is re-scored in the filter.
"""
import copy
import time
import numpy as np

from .inputs import SLOTS, array, finite
from .movement_receiver import SCHEMA, FEEDBACK


class MovementClient:
    def __init__(self, session, pipeline, robot='198', *, mode='shadow'):
        if robot not in SLOTS or mode not in ('shadow', 'active') or not isinstance(session, str) or not session:
            raise ValueError('movement_client_configuration')
        self.session, self.pipeline, self.robot, self.mode = session, pipeline, robot, mode
        self.pending = None
        self.sequence = 0
        self.shots = set()
        self.previous_targets = None
        self.fault = None
        self.decision = None
        self.completed = False
        self.accepted = False
        self.last_feedback_state_sequence = -1

    def bootstrap(self, states):
        values = []
        for slot in SLOTS:
            state = states[slot]
            evidence = state['yichao_movement']
            if (evidence['session'] != self.session or evidence['robot'] != slot
                    or evidence['execution_mode'] != self.mode or evidence['hit_enabled'] is not False
                    or state['phase'] not in ('HOME_HOLD', 'OUTWARD_HOLD')
                    or state['valid'] is not True or evidence['sensors_recent'] is not True
                    or state['stable_elapsed_s'] < .1):
                raise ValueError('receiver_bootstrap_not_ready')
            target = finite(evidence['initial_target_y'], 'initial_scheduler_target')
            if abs(target) > 1.2 or abs(target-state['base_position_xyz'][1]) > .02:
                raise ValueError('initial_scheduler_target_not_reached')
            values.append(target)
        self.previous_targets = values

    def observe(self, state):
        if not self.pending or self.fault:
            return
        try:
            evidence = state['yichao_movement']
            if (evidence['session'] != self.session or evidence['robot'] != self.robot
                    or evidence['execution_mode'] != self.mode):
                raise ValueError('receiver_identity_or_mode_changed')
            seq = state['sequence']
            if type(seq) is not int or seq <= self.last_feedback_state_sequence:
                raise ValueError('feedback_state_out_of_order')
            self.last_feedback_state_sequence = seq
            ack = evidence['feedback']
            if ack is None:
                return
            if ack.get('schema') != FEEDBACK:
                raise ValueError('feedback_schema')
            if any(ack.get(k) != self.pending[k] for k in ('session', 'robot', 'sequence', 'shot', 'token')):
                return
            if ack['status'] == 'rejected':
                raise ValueError('receiver_rejected:'+str(ack.get('reason')))
            applied = finite(ack['applied_target_y'], 'applied_target')
            if abs(applied-self.pending['target_y']) > 1e-4:
                raise ValueError('applied_target_mismatch')
            if ack.get('evidence') != 'dedicated_receiver_after_scheduler_application':
                raise ValueError('unsupported_application_evidence')
            if state['valid'] is not True or evidence['sensors_recent'] is not True:
                raise ValueError('feedback_sensors_invalid')
            if ack['status'] not in ('accepted', 'completed'):
                raise ValueError('feedback_status')
            # Only actual application feedback updates cross-shot memory.
            self.previous_targets[SLOTS.index(self.robot)] = applied
            self.accepted = True
            if (ack['status'] == 'completed' and ack['completed'] is True
                    and ack['phase'] in ('HOME_HOLD', 'OUTWARD_HOLD')
                    and finite(ack['stable_elapsed_s'], 'stable_elapsed') >= .1-1e-9
                    and abs(finite(ack['position_y'], 'position_y')-applied) <= .02):
                self.completed = True
        except (KeyError, TypeError, ValueError) as exc:
            self.fault = str(exc)

    def poll(self, now):
        """Advance an existing transaction even when the incoming ball disappears."""
        now = finite(now, 'decision_now')
        if self.fault:
            return {'reason': self.fault, 'command': None}
        if self.pending:
            if self.completed:
                return {'reason': 'single_movement_completed', 'command': None}
            if self.accepted:
                if now-self.pending['issued_at'] > 5.:
                    self.fault = 'settling_timeout_existing_motion_may_continue'
                return {'reason': self.fault or 'awaiting_settling', 'command': None}
            if now > self.pending['expires_at']:
                # An accepted movement may continue until settling. Do not
                # reinterpret timeout/stop-publishing as a physical stop.
                self.fault = 'movement_feedback_timeout_existing_motion_may_continue'
                return {'reason': self.fault, 'command': None}
            return {'reason': 'awaiting_application_feedback', 'command': copy.deepcopy(self.pending)}
        return None

    def step(self, features, states, now):
        now = finite(now, 'decision_now')
        existing = self.poll(now)
        if existing is not None:
            return existing
        try:
            for slot in SLOTS:
                e = states[slot]['yichao_movement']
                if (e['session'] != self.session or e['robot'] != slot
                        or e['execution_mode'] != self.mode or e['hit_enabled'] is not False
                        or e['sensors_recent'] is not True or e['clock_workstation_interval'] is None):
                    raise ValueError('receiver_or_clock_not_ready')
                if self.mode == 'active' and e.get('input_contract_accepted') is not True:
                    raise ValueError('real_input_contract_not_accepted')
            if self.previous_targets is None:
                self.bootstrap(states)
            if self.mode == 'active' and features.record.get('real_input_accepted') is not True:
                raise ValueError('feature_source_contract_not_accepted')
            shot = features.record['shot_id']
            if shot in self.shots:
                return {'reason': 'shot_already_decided', 'command': None}
            self.shots.add(shot)
            source = copy.deepcopy(features)
            source.actor[-2:] = np.asarray(self.previous_targets, np.float32)/np.float32(1.2)
            start = time.perf_counter()
            decision = self.pipeline.decide(source)
            self.decision = decision
            if not decision['valid']:
                raise ValueError('no_safe_candidate')
            index = SLOTS.index(self.robot)
            actual_pair = np.asarray([states[s]['base_position_xyz'][1] for s in SLOTS], np.float32)
            target = finite(decision['target_y'][index], 'actor_target')
            if abs(target-float(actual_pair[index])) > .05:
                raise ValueError('target_exceeds_single_test_5cm_limit')
            actual_pair[index] = target
            if actual_pair[1]-actual_pair[0] < .45:
                raise ValueError('stationary_peer_clearance')
            risk = float(self.pipeline.risks(source.safe, (actual_pair/1.2)[None, :])[0])
            if not np.isfinite(risk) or risk > self.pipeline.threshold:
                raise ValueError('stationary_peer_filter_rejected')
            if time.perf_counter()-start > .02:
                raise ValueError('decision_deadline_exceeded')
            self.sequence += 1
            self.pending = {'schema': SCHEMA, 'session': self.session, 'robot': self.robot,
                'sequence': self.sequence, 'shot': shot, 'token': self.session+':move:'+str(self.sequence),
                'issued_at': now, 'expires_at': now+.25, 'target_y': target, 'kind': 'move'}
            return {'reason': None, 'command': copy.deepcopy(self.pending),
                    'decision': decision, 'actual_pair_risk': risk, 'actual_pair_y': actual_pair.tolist()}
        except (KeyError, TypeError, ValueError) as exc:
            return {'reason': str(exc), 'command': None, 'decision': self.decision}
