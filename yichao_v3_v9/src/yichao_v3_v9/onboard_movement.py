"""Dedicated movement bridge installed by the isolated onboard launcher.

All ROS names must be remapped into one Yichao session. Existing V9 source,
scheduler, mirror and policy are reused; command admission and ACK are private.
"""
import json
import threading
import time

from .clock_bounds import ClockBounds
from .movement_receiver import MovementReceiver
from .telemetry import strict_json
from utils.planner_ros_bridge import PlannerRosBridge


def bridge_class(session, *, shadow, protocol='movement', active_stage=None):
    import rospy
    from std_msgs.msg import String
    if not shadow and not callable(active_stage):
        raise ValueError('active_bridge_requires_control_stage')
    if protocol not in ('movement','relay'):raise ValueError('command_protocol')
    if protocol=='relay':
        from .command_receiver import CommandReceiver
        receiver_class=CommandReceiver
    else:receiver_class=MovementReceiver
    state_key='yichao_relay' if protocol=='relay' else 'yichao_movement'
    allowed_kinds=('move','clear','hit','return') if protocol=='relay' else ('move',)

    class Bridge(PlannerRosBridge):
        def __init__(self, robot_id, command_timeout_s=.25):
            self.prefix = '/yichao_v3_v9/'+session+'/'+robot_id
            for suffix in ('command', 'state'):
                if rospy.resolve_name('/doubles/'+robot_id+'/'+suffix) != self.prefix+'/'+suffix:
                    raise RuntimeError('onboard movement requires private ROS remapping')
            self.slot = '66' if robot_id == 'table_right' else '198'
            self.receiver = None
            self.feedback = None
            self.clock = ClockBounds()
            self.clock_lock = threading.Lock()
            self.probes = {}
            self.clock_error = None
            self.sensors_recent = False
            self.extra = {}
            self._latest_control_generation = None
            self._policy_generation = None
            self._control_session_invalidated = False
            self._agent = None
            super().__init__(robot_id, command_timeout_s)
            self.reply = rospy.Publisher(self.prefix+'/clock_reply', String, queue_size=1, tcp_nodelay=True)
            self.clock_sub = rospy.Subscriber(self.prefix+'/clock_request', String, self._clock_message, queue_size=1, tcp_nodelay=True)
            original = self._publisher
            owner = self

            class WithEvidence:
                def publish(self, message):
                    payload = strict_json(message.data)
                    payload[state_key] = {**owner.extra,
                        'session': session, 'robot': owner.slot,
                        'schema': 'yichao-onboard-'+protocol+'-state-v1', 'hit_enabled': protocol=='relay',
                        'execution_mode': 'shadow' if shadow else 'active',
                        'experimental_active': not shadow,
                        'real_input_accepted': False,
                        **owner._control_evidence(),
                        'feedback': owner.feedback,
                        'initial_target_y': payload['target_base_y']}
                    original.publish(String(data=json.dumps(payload, allow_nan=False)))

                def unregister(self):
                    original.unregister()

            self._publisher = WithEvidence()

        def _control_evidence(self):
            if active_stage is None:
                return {}
            return {'control_stage': active_stage(),
                    'control_generation': getattr(active_stage, 'generation', 0),
                    'calibration_generation': getattr(active_stage, 'calibration_generation', 0),
                    'control_session_invalidated': self._control_session_invalidated,
                    'control_reason': getattr(active_stage, 'reason', None)}

        def _reject_control_stage(self, command, reason):
            self.feedback = {'schema': ('yichao-v3-v9-feedback-v1' if protocol == 'relay'
                                      else 'yichao-v9-movement-feedback-v1'),
                'evidence': 'active_stage_admission', 'session': session, 'robot': self.slot,
                **{key: command.get(key) for key in ('sequence', 'shot', 'token', 'kind')},
                'status': 'rejected', 'completed': False, 'reason': reason,
                'execution_mode': 'active'}
            self._last_error = reason

        def _admission_reason(self):
            if active_stage is None:
                return None
            stage = active_stage()
            generation = getattr(active_stage, 'calibration_generation',
                                 getattr(active_stage, 'generation', 0))
            # Recalibration before any command has nothing to invalidate.
            # Once a command is queued/applied, retain the transaction boundary.
            has_transaction=(self._latest_command is not None or
                self.receiver is not None and self.receiver.sequence>=0)
            if self._policy_generation is not None and (
                    stage not in ('policy','waiting_inputs') or generation != self._policy_generation
                    ) and has_transaction:
                self._control_session_invalidated = True
            if self._control_session_invalidated:
                return 'control_recalibrated_restart_session_required'
            if stage != 'policy':
                return 'control_stage_not_policy:'+str(stage)
            self._policy_generation = generation
            return None

        def _current_sensor_ages(self, agent):
            now = time.monotonic()
            raw = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
            return {'body_source_age_s': (raw-agent.se.body_source_monotonic_raw_ns)/1e9,
                    'imu_source_age_s': (raw-agent.se.imu_source_monotonic_raw_ns)/1e9,
                    'torso_receive_age_s': now-agent._torso_receive_monotonic_ns/1e9}

        def _clock_message(self, message):
            r1 = time.monotonic()
            try:
                d = strict_json(message.data)
                if d['session'] != session or d['robot'] != self.slot:
                    raise ValueError('clock_identity')
                nonce = d['nonce']
                if not isinstance(nonce, str) or not 0 < len(nonce) <= 64:
                    raise ValueError('clock_nonce')
                with self.clock_lock:
                    if d['kind'] == 'probe':
                        self.probes = {k: v for k, v in self.probes.items() if r1-v[1] < 1.}
                        if len(self.probes) >= 8 or nonce in self.probes:
                            raise ValueError('clock_probe_limit_or_reuse')
                        r2 = time.monotonic()
                        self.probes[nonce] = (d['workstation_send'], r1, r2)
                        self.reply.publish(String(data=json.dumps({**d, 'kind': 'reply',
                            'robot_receive': r1, 'robot_send': r2})))
                    elif d['kind'] == 'commit':
                        saved = self.probes.pop(nonce)
                        if tuple(d[k] for k in ('workstation_send', 'robot_receive', 'robot_send')) != saved:
                            raise ValueError('clock_exchange_changed')
                        self.clock.update(*saved, d['workstation_receive'])
                        self.clock_error = None
                    else:
                        raise ValueError('clock_message_kind')
            except (KeyError, TypeError, ValueError) as exc:
                self.clock_error = str(exc)

        def _command_callback(self, message):
            try:
                d = strict_json(message.data)
                if d['session'] != session or d['robot'] != self.slot or d['kind'] not in allowed_kinds:
                    raise ValueError('command_protocol_or_private_session')
                with self._lock:
                    reason = self._admission_reason()
                    if reason is not None:
                        self._latest_command = None
                        self._reject_control_stage(d, reason)
                        return
                    self._latest_command = d
                    self._latest_control_generation = getattr(active_stage, 'generation', 0)
                    self._latest_receive_monotonic = time.monotonic()
            except (KeyError, TypeError, ValueError) as exc:
                self._last_error = 'invalid_movement:'+str(exc)

        def apply_pending(self, scheduler, pelvis_position):
            if self.receiver is None:
                kwargs={'allow_prepare_preemption':not shadow} if protocol=='relay' else {}
                self.receiver = receiver_class(session, self.slot, scheduler, **kwargs)
            with self._lock:
                command = self._latest_command
            if command is None:
                return
            reason = self._admission_reason()
            if active_stage is not None and reason is None and (
                    self._latest_control_generation != getattr(active_stage, 'generation', 0)):
                reason = 'control_generation_changed'
            if reason is not None:
                self._reject_control_stage(command, reason)
                with self._lock:
                    if self._latest_command is command:
                        self._latest_command = None
                return
            try:
                with self.clock_lock:
                    interval = self.clock.workstation_interval(time.monotonic())
                recent = self.sensors_recent
                if active_stage is not None:
                    recent = self._agent is not None and all(
                        0 <= age <= .25 for age in self._current_sensor_ages(self._agent).values())
                feedback = self.receiver.apply(command, workstation_time_interval=interval,
                    position=pelvis_position, sensors_recent=recent)
                feedback['execution_mode'] = 'shadow' if shadow else 'active'
                self.feedback = feedback
                if feedback['status'] in ('accepted', 'completed'):
                    self._last_session_id = session
                    self._last_applied_sequence = feedback['sequence']
                    if feedback.get('kind')=='hit':self._last_commit_token=feedback['token']
                    self._last_error = ''
                else:
                    self._last_error = 'movement_rejected:'+str(feedback['reason'])
                # Rejection is terminal for this receipt too. Retrying it every
                # cycle must not later turn rejection into silent application.
                with self._lock:
                    if self._latest_command is command:
                        self._latest_command = None
            except (TypeError, ValueError) as exc:
                self._last_error = 'clock_not_ready:'+str(exc)

        def publish_state(self, agent):
            now = time.monotonic()
            self._agent = agent
            ages = self._current_sensor_ages(agent)
            admission_reason = self._admission_reason()
            if admission_reason is not None:
                with self._lock:
                    if self._latest_command is not None:
                        self._reject_control_stage(self._latest_command, admission_reason)
                    self._latest_command = None
            self.sensors_recent = all(0 <= age <= .25 for age in ages.values())
            with self.clock_lock:
                interval_error = None
                try:
                    interval = self.clock.workstation_interval(now)
                except ValueError as exc:
                    interval = None
                    interval_error = str(exc)
            self.extra = {**ages, 'sensors_recent': self.sensors_recent,
                'clock_workstation_interval': interval,
                'clock_error': self.clock_error,
                'clock_interval_error': interval_error,
                'torso_source_stamp_s': agent._torso_source_stamp_s,
                'torso_sensor_age_verified': False,
                'input_contract_accepted': False}
            if self.receiver is not None and admission_reason is None:
                speed = 0.
                if self._pelvis_samples:
                    old_time, old_position = self._pelvis_samples[-1]
                    dt = now-old_time
                    speed = float((agent.pelvis_pos[1]-old_position[1])/dt) if dt > 0 else float('inf')
                if speed != float('inf'):
                    feedback = self.receiver.observe(now=now, position=agent.pelvis_pos,
                        velocity_y=speed, sensors_recent=self.sensors_recent)
                    if feedback and (self.feedback is None or
                            self.feedback.get('status') != 'rejected'):
                        feedback['execution_mode'] = 'shadow' if shadow else 'active'
                        self.feedback = feedback
            super().publish_state(agent)

    return Bridge
