"""Existing V9 telemetry -> V3 inference -> non-executable movement previews.

This observer follows the running controller's trajectory. Receipt-time history
and externally observed targets are explicit diagnostic assumptions, not sensor
time alignment, Yichao ACKs, or acceptance of the real control interface.
"""
from collections import deque
import copy
import time
import numpy as np

from .ball_feedback import BallFeedback
from .inputs import Features, SLOTS, array
from .joint_normalization import JointNormalizer
from .telemetry import TelemetryParser, strict_json, number
from .v9_wire import encode
from utils.joint_mapping import LAB_JOINT_NAMES

MONITOR = '/table_tennis_planner_monitor'
DOUBLES_BALL = '/doubles/ball_prediction'
STATE_TOPICS = {'/doubles/table_right/state': '66', '/doubles/table_left/state': '198'}
INPUT_TOPICS = (*STATE_TOPICS, MONITOR, DOUBLES_BALL)


class RuntimeShadow:
    def __init__(self, session, pipeline, *, provenance='recorded_real_ros', ball_topic=MONITOR, continuous=False):
        if not isinstance(session, str) or not 0 < len(session) <= 100:
            raise ValueError('explicit_shadow_session_required')
        if provenance not in ('recorded_real_ros', 'live_ros', 'synthetic_wire_fixture'):
            raise ValueError('unknown_shadow_provenance')
        if ball_topic not in (MONITOR, DOUBLES_BALL):
            raise ValueError('unsupported_ball_topic')
        self.ball_topic = ball_topic
        self.monitor_clock = None
        self.session, self.pipeline, self.provenance = session, pipeline, provenance
        self.parser, self.ball_parser = TelemetryParser(), BallFeedback(session)
        self.normalizer = JointNormalizer()
        self.states = {s: deque(maxlen=32) for s in SLOTS}
        self.ball = None
        self.fault = None
        self.continuous=continuous;self.waiting_reason=None
        self.publishers = {}
        self.seen = set()
        self.sequence = 0
        self.last_now = None
        self.counts = {'decisions': 0, 'rejected': 0, 'latched_shots': 0}

    def _result(self, reason, **extra):
        return {'kind': 'yichao_runtime_shadow', 'reason': reason,
                'provenance': self.provenance, 'real_input_accepted': False,
                'command_ack': False, 'control_commands_sent': 0,
                'movement_preview': [], **extra}

    def ingest(self, record):
        """Use the existing capture tool's raw_input record, preserving all clocks."""
        topic = record.get('topic')
        if record.get('kind') != 'raw_input' or topic not in INPUT_TOPICS:
            return
        if topic in (MONITOR, DOUBLES_BALL) and topic != self.ball_topic:
            return  # one explicit ball source per run, never merge two shot streams
        if self.fault:
            return
        try:
            self.waiting_reason=None
            publisher = record['publisher']
            if not isinstance(publisher, str) or not publisher:
                raise ValueError('publisher_identity_missing')
            if publisher != self.publishers.setdefault(topic, publisher):
                raise ValueError('publisher_changed')
            payload = strict_json(record['payload']['data'])
            receipt, wall = number(record['receive_monotonic_s']), number(record['receive_wall_s'])
            if topic == MONITOR:
                # Predictor processing time is on this workstation. This does
                # not prove how old the underlying mocap sample was.
                age = wall - number(payload['timestamp'])
                if not -.01 <= age <= .30:
                    raise ValueError('monitor_processing_timestamp_future_or_stale')
                if self.monitor_clock and abs((wall-self.monitor_clock[1])-(receipt-self.monitor_clock[0])) > .1:
                    raise ValueError('monitor_wall_clock_jump')
                self.monitor_clock = (receipt, wall)
                self.ball = self.ball_parser.parse(payload, receipt)
                self.ball['processing_age_at_receipt_s'] = age
            elif topic == DOUBLES_BALL:
                # Invalid ball means idle/expired, not permission to reuse an
                # earlier prediction. Valid predictions use the frozen parser.
                if payload.get('valid') is False:
                    self.ball = None
                    return
                parsed = self.parser.parse(topic, payload, {'monotonic_s': receipt,
                    'wall_s': wall, 'ros_s': wall}, self.provenance)
                if not parsed['wire_valid']:
                    raise ValueError(parsed['reason'])
                self.ball = {'prediction_fields_valid': True,
                    'diagnostic_shot_id': self.session+'/'+payload['shot_id'],
                    'received_workstation_monotonic_s': receipt,
                    'source_clock': {'predictor_processing_ros_s': payload['source_timestamp']},
                    **{k: copy.deepcopy(payload[k]) for k in ('position', 'velocity', 'racket_velocity', 'time_to_strike_s')},
                    'strike_position': copy.deepcopy(payload['predicted_strike_position']),
                    'strike_velocity': copy.deepcopy(payload['predicted_strike_velocity'])}
            else:
                if 'yichao_movement' in payload or 'yichao_relay' in payload:
                    key='yichao_relay' if 'yichao_relay' in payload else 'yichao_movement'
                    evidence = payload[key]
                    if (evidence.get('session') != self.session or
                            evidence.get('robot') != STATE_TOPICS[topic] or
                            evidence.get('schema') != 'yichao-onboard-'+key.removeprefix('yichao_')+'-state-v1'):
                        raise ValueError('onboard_state_identity')
                    if evidence.get('sensors_recent') is not True:
                        raise ValueError('onboard_sensor_freshness')
                parsed = self.parser.parse(topic, payload, {'monotonic_s': receipt,
                                           'wall_s': wall, 'ros_s': wall}, self.provenance)
                if not parsed['wire_valid']:
                    raise ValueError(parsed['reason'])
                if payload.get('emergency_stop') is not False:
                    raise ValueError('emergency_stop_or_missing_flag')
                reported_rejection=(self.continuous and payload.get('yichao_relay',{}).get('feedback',{})
                    and payload['yichao_relay']['feedback'].get('status')=='rejected'
                    and str(payload.get('transport_error','')).startswith(
                        ('movement_rejected:','control_stage_not_policy:waiting_inputs')))
                clock_wait=self.continuous and str(payload.get('transport_error','')).startswith('clock_not_ready:')
                if payload.get('transport_error') not in ('', None) and not reported_rejection and not clock_wait:
                    raise ValueError('controller_reports_transport_error')
                samples = self.states[STATE_TOPICS[topic]]
                if samples and payload['source_monotonic_ns'] <= samples[-1]['payload']['source_monotonic_ns']:
                    raise ValueError('robot_source_time_not_increasing')
                samples.append({'receipt': receipt, 'payload': copy.deepcopy(payload)})
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            reason=str(exc)
            if self.continuous and reason in ('onboard_sensor_freshness','producer reports invalid',
                    'candidate ROS source time is future/stale','monitor_processing_timestamp_future_or_stale'):
                self.waiting_reason=reason
                if topic in STATE_TOPICS:self.states[STATE_TOPICS[topic]].clear()
                else:self.ball=None
            else:self.fault=reason

    def features(self, now):
        """Receipt-grid diagnostic input; never routes through the accepted-input gate."""
        return self._features(now, self.ball, handoff=False)

    def handoff_features(self, now, shot_ball, *, context_max_age_s=5.):
        """Refresh robot features for an existing shot's diagnostic handoff.

        The retained strike target/racket velocity describe that shot only.
        Its TTS is aged; no incoming-ball freshness or new-HIT acceptance is
        inferred. The actor vector is diagnostic and must not start a shot.
        """
        return self._features(now, shot_ball, handoff=True,context_max_age_s=context_max_age_s)

    def _features(self, now, ball, *, handoff, context_max_age_s=5.):
        if self.fault:
            raise ValueError('input_fault_latched:' + self.fault)
        if any(not self.states[s] for s in SLOTS):
            raise ValueError('missing_v9_robot_state')
        if ball is None or not ball['prediction_fields_valid']:
            raise ValueError('no_valid_incoming_shot')
        now = number(now)
        receipt = ball['received_workstation_monotonic_s']
        max_age = (float('inf') if context_max_age_s is None else context_max_age_s) if handoff else .30
        if not 0 <= now - receipt <= max_age:
            raise ValueError('ball_receive_age')
        # End at the newest time both state streams can bracket; no future
        # samples relative to now, extrapolation, or repeated startup padding.
        end = min(self.states[s][-1]['receipt'] for s in SLOTS)
        grid = end - np.arange(4, -1, -1, dtype=np.float64) * .02
        histories, joints, previous, selected, source_evidence = [], [], [], {}, {}
        for slot in SLOTS:
            samples = list(self.states[slot])
            ts = np.array([s['receipt'] for s in samples])
            if not 0 <= now - ts[-1] <= .25 or not 0 <= now-end <= .05:
                raise ValueError('robot_receive_age_or_pair_skew')
            if ts[0] > grid[0] + 1e-9:
                raise ValueError('base_history_warming_up')
            first = max(0, int(np.searchsorted(ts, grid[0], side='right')) - 1)
            if np.any(np.diff(ts[first:]) > .05 + 1e-9):
                raise ValueError('base_history_gap')
            xyz = np.asarray([s['payload']['base_position_xyz'] for s in samples])
            history = np.stack([np.interp(grid, ts, xyz[:, i]) for i in range(3)], axis=1).astype(np.float32)
            index = max(0, int(np.searchsorted(ts, end + 1e-9, side='right')) - 1)
            state = samples[index]['payload']
            if not handoff and state['phase'] in ('HIT', 'POST_DELAY'):
                raise ValueError('robot_hit_locked')
            histories.append(history)
            joints.append(self.normalizer.normalize(state['q'], LAB_JOINT_NAMES))
            target = number(state['target_base_y'])
            if abs(target) > 1.2:
                raise ValueError('observed_target_out_of_workspace')
            previous.append(target)
            selected[slot] = state
            source_evidence[slot] = {'source_monotonic_ns': state['source_monotonic_ns'],
                'state_sequence': state['sequence'], 'receive_monotonic_s': samples[index]['receipt'],
                'last_planner_session_id': state.get('last_planner_session_id'),
                'last_commit_token': state.get('last_commit_token')}
        elapsed = end - receipt
        if not handoff and abs(elapsed) > .05:
            raise ValueError('ball_state_receipt_skew')
        tts = ball['time_to_strike_s'] - elapsed
        if not handoff and (not 0 < ball['time_to_strike_s'] - (now-receipt) <= 2 or not 0 < tts <= 2):
            raise ValueError('expired_tts')
        # Frozen training _reset_ball initializes acceleration to zero. This is
        # a training-model reconstruction assumption, not measured acceleration.
        position = array(ball['position'], (3,), 'ball_position')
        velocity = array(ball['velocity'], (3,), 'ball_velocity')
        ball_history = position + (grid-receipt).astype(np.float32)[:, None]*velocity
        scale = np.array([3, 1.5, 1.5], np.float32)
        relative = np.stack((ball_history-histories[0], ball_history-histories[1]), axis=1)
        base_scale = np.array([3, 1.2], np.float32)
        actor = np.concatenate((np.clip(relative/scale, -3, 3).reshape(-1),
            np.clip(np.array([h[-1, :2] for h in histories])/base_scale, -3, 3).reshape(-1),
            np.asarray(previous, np.float32)/np.float32(1.2))).astype(np.float32)
        strike = np.clip(np.concatenate((array(ball['strike_position'], (3,), 'strike')/scale,
            array(ball['racket_velocity'], (3,), 'racket_velocity')/4, [tts])), -3, 3)
        safe = np.concatenate((*[np.clip(h[:, :2]/base_scale, -3, 3).reshape(-1) for h in histories],
                               *joints, strike)).astype(np.float32)
        evidence = {'now': now, 'shot_id': ball['diagnostic_shot_id'], 'sample_time': end,
            'handoff_only': handoff,
            'safe_observation_basis': 'fresh robot histories/joints; retained shot strike target/racket velocity with aged TTS' if handoff else 'current diagnostic input',
            'base_history_times': grid.tolist(), 'base_histories': [h.tolist() for h in histories],
            'previous_target_basis': 'current targets reported by existing controller; NOT previous Yichao shot ACK',
            'observed_targets_y': previous, 'robot_sources': source_evidence,
            'ball_source_clock': ball['source_clock'], 'ball_receive_monotonic_s': receipt,
            'assumptions': ['20ms receipt-time interpolated base grid; sensor alignment unverified',
                'mocap_origin treated as training common frame for diagnostics; calibration/heading acceptance pending',
                'zero-acceleration ball reconstruction from frozen training initialization',
                'external controller target used as counterfactual previous-target input'],
            'joint_normalization_urdf_sha256': self.normalizer.profile['urdf_sha256'],
            'robots': selected}
        return Features(array(actor, (36,), 'actor'), array(safe, (85,), 'safe'), evidence, self.provenance)

    def tick(self, now):
        now = number(now)
        if self.last_now is not None and now < self.last_now:
            self.fault = 'decision_clock_went_backwards'
        self.last_now = now
        try:
            features = self.features(now)
        except (KeyError, TypeError, ValueError) as exc:
            self.counts['rejected'] += 1
            return self._result(str(exc))
        shot = features.record['shot_id']
        if shot in self.seen:
            return self._result('shot_already_decided', shot_id=shot)
        self.seen.add(shot)
        self.counts['latched_shots'] += 1
        start = time.perf_counter()
        try:
            decision = self.pipeline.decide(features)
            elapsed = time.perf_counter() - start
            out = self._result(None, shot_id=shot, actor=features.actor.tolist(),
                safe=features.safe.tolist(), input_evidence=features.record,
                decision=decision, processing_s=elapsed)
            self.counts['decisions'] += 1
            if elapsed > .02:
                out['reason'] = 'decision_deadline_exceeded'
            elif not decision['valid']:
                out['reason'] = 'no_safe_candidate'
            else:
                # Logs only. Even accidental forwarding to the stock bridge
                # leaves these envelopes invalid; there is no active option.
                previews = []
                for i, slot in enumerate(SLOTS):
                    self.sequence += 1
                    command = {'schema': 'yichao-v3-v9-command-v1', 'session': self.session,
                        'sequence': self.sequence, 'shot': shot, 'token': f'{self.session}:{self.sequence}',
                        'robot': slot, 'kind': 'move', 'target_y': float(decision['target_y'][i]),
                        'issued_at': now, 'expires_at': now+.02, 'hit': None}
                    wire = encode(command, now, float(features.record['base_histories'][i][-1][0]))
                    wire.update(valid=False, shadow=True, planned_valid=True, shot_id=shot,
                                preview_only=True)
                    previews.append(wire)
                out['movement_preview'] = previews
            return out
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            self.fault = 'inference_or_wire_failure:' + str(exc)
            return self._result(self.fault, shot_id=shot)

    def summary(self):
        return self._result('summary', counts=dict(self.counts), input_fault=self.fault,
                            ball_topic=self.ball_topic,
                            received_state_counts={s: len(v) for s, v in self.states.items()})
