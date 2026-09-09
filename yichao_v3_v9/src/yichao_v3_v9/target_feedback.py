"""Correlate V9 movement target telemetry with a registered outgoing command.

Observed target and settling evidence are distinct from an authoritative bridge
ACK. No motor/ROS transport is created and no simulator ACK is synthesized.
"""
import copy
from types import SimpleNamespace

from .executor import OfflineExecutor
from .telemetry import TelemetryParser, number, integer
from .v9_wire import ROLES


class TargetFeedback:
    def __init__(self, session):
        if not isinstance(session, str) or not session:
            raise ValueError('explicit_session_required')
        self.session = session
        self.parser = TelemetryParser()
        self.pending, self.last_sequence, self.last_source = {}, {}, {}
        self.fault = None

    def register(self, command, now):
        """Register an intended movement; this is not evidence it was sent."""
        if self.fault:
            raise ValueError('feedback_fault_latched:'+self.fault)
        OfflineExecutor._validate(SimpleNamespace(session=self.session), command, number(now))
        if command['kind'] == 'hit':
            raise ValueError('movement_feedback_does_not_admit_hit')
        slot = command['robot']
        if command['sequence'] <= self.last_sequence.get(slot, -1):
            raise ValueError('command_sequence_not_increasing')
        old = self.pending.get(slot)
        if old and not old['settled']:
            raise ValueError('previous_movement_not_settled')
        self.last_sequence[slot] = command['sequence']
        self.pending[slot] = {'command': copy.deepcopy(command), 'observed': False,
                              'settled': False, 'stable_since': None, 'last_receipt': None}

    def observe(self, slot, state, receive):
        result = {'kind': 'v9_movement_target_evidence', 'robot': slot,
                  'command_ack': False, 'real_input_accepted': False,
                  'target_observed': False, 'settled_observed': False}
        entry = self.pending.get(slot)
        if not entry:
            return {**result, 'reason': 'no_registered_movement'}
        c = entry['command']
        result.update(session=c['session'], sequence=c['sequence'], token=c['token'],
                      shot=c['shot'], target_y=c['target_y'])
        if self.fault:
            return {**result, 'reason': 'feedback_fault_latched:'+self.fault}
        try:
            topic = '/doubles/'+ROLES[slot]+'/state'
            parsed = self.parser.parse(topic, state, receive)
            if not parsed['wire_valid']:
                raise ValueError(parsed['reason'])
            now = number(receive['monotonic_s'])
            source = integer(state['source_monotonic_ns'], positive=True)
            if source <= self.last_source.get(slot, -1):
                raise ValueError('state_source_time_not_increasing')
            self.last_source[slot] = source
            if state['emergency_stop'] is not False:
                raise ValueError('emergency_stop')
            if now < c['issued_at']:
                raise ValueError('state_received_before_command')
            if state['last_planner_session_id'] != self.session:
                if entry['observed']:
                    raise ValueError('planner_session_changed_after_target_observation')
                return {**result, 'reason': 'waiting_for_matching_session'}
            sequence = state['last_applied_sequence']
            if type(sequence) is not int:
                raise ValueError('invalid_applied_sequence')
            if sequence < c['sequence']:
                return {**result, 'reason': 'waiting_for_matching_sequence'}
            if sequence > c['sequence']:
                raise ValueError('command_superseded')
            if state['transport_error'] != '':
                raise ValueError('bridge_error:'+str(state['transport_error']))
            if not entry['observed'] and now > c['expires_at']:
                raise ValueError('target_observation_deadline_expired')
            if abs(number(state['target_base_y'])-c['target_y']) > 1e-4:
                raise ValueError('reported_target_differs_from_command')
            allowed = {'RETURN', 'HOME_HOLD'} if c['kind'] == 'return' else {'RETURN', 'HOME_HOLD', 'OUTWARD', 'OUTWARD_HOLD'}
            if state['phase'] not in allowed:
                raise ValueError('phase_incompatible_with_movement')
            entry['observed'] = True
            # Require our own uninterrupted observation window as well as the
            # scheduler's stable timer. One HOLD snapshot is not completion.
            hold = state['phase'] in {'HOME_HOLD', 'OUTWARD_HOLD'}
            near = abs(state['base_position_xyz'][1]-c['target_y']) <= .06
            slow = abs(state['base_linear_velocity_xyz'][1]) <= .15
            continuous = entry['last_receipt'] is None or now-entry['last_receipt'] <= .05+1e-9
            if not (hold and near and slow and continuous):
                entry['stable_since'] = None
                entry['settled'] = False
            elif entry['stable_since'] is None:
                entry['stable_since'] = now
            if entry['stable_since'] is not None:
                entry['settled'] = now-entry['stable_since'] >= .1-1e-9 and state['stable_elapsed_s'] >= .1
            entry['last_receipt'] = now
            return {**result, 'target_observed': True, 'settled_observed': entry['settled'],
                    'reason': 'settled_target_observed' if entry['settled'] else 'matching_target_observed',
                    'state_sequence': state['sequence'], 'source_monotonic_ns': source,
                    'receive_monotonic_s': now, 'phase': state['phase'],
                    'position_y': state['base_position_xyz'][1],
                    'evidence_limit': 'matching stock telemetry; not authoritative ACK or sensor freshness acceptance'}
        except (KeyError, TypeError, ValueError) as exc:
            self.fault = str(exc)
            return {**result, 'reason': self.fault}

    def paired_targets(self):
        """Return a same-shot observed pair for diagnostics, never mix two shots."""
        if self.fault or set(self.pending) != set(ROLES):
            return None
        entries = [self.pending[s] for s in ('66', '198')]
        if not all(e['observed'] for e in entries) or len({e['command']['shot'] for e in entries}) != 1:
            return None
        return {'targets_y': [e['command']['target_y'] for e in entries],
                'shot': entries[0]['command']['shot'], 'session': self.session,
                'command_ack': False, 'real_input_accepted': False}
