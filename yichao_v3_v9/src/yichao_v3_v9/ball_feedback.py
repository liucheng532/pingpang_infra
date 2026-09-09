"""Extract coherent ball/strike fields from the existing Predictor monitor JSON.

No independent topic synchronization or fabricated sensor timestamps. A shot
identity is local to this diagnostic session, not a robot command token or ACK.
"""
from .telemetry import boolean, integer, number, vector


class BallFeedback:
    def __init__(self, session_id):
        if not isinstance(session_id, str) or not 0 < len(session_id) <= 128:
            raise ValueError('session_identity_required')
        self.session_id = session_id
        self.previous = None
        self.previous_state = None
        self.waiting_seen = False
        self.shot_id = None
        self.shots = 0
        self.fault = None

    def parse(self, payload, received_workstation_monotonic_s):
        if self.fault:
            raise ValueError('ball_stream_fault_latched:'+self.fault)
        if payload['version'] != 1:
            raise ValueError('unsupported_monitor_version')
        seq = integer(payload['sequence'])
        stamp = number(payload['timestamp'])
        received = number(received_workstation_monotonic_s)
        if stamp <= 0 or received < 0:
            raise ValueError('invalid_monitor_timestamp')
        if self.previous and (seq <= self.previous[0] or stamp <= self.previous[1] or received < self.previous[2]):
            self.fault = 'duplicate_or_out_of_order_monitor'
            raise ValueError(self.fault)
        planner = payload['planner']
        state = planner['state']
        if state not in ('waiting', 'tracking', 'draining', 'cooldown'):
            raise ValueError('unknown_predictor_state')
        armed, held = boolean(planner['planning_armed']), boolean(planner['hold_post_hit_output'])
        self.previous = (seq, stamp, received)
        if state == 'waiting':
            self.waiting_seen, self.shot_id = True, None
        elif state == 'tracking' and self.previous_state != 'tracking':
            if self.waiting_seen:
                self.shots += 1
                self.shot_id = self.session_id+'/shot-'+str(self.shots)
            else:
                self.shot_id = None
            self.waiting_seen = False
        elif state in ('draining', 'cooldown'):
            self.waiting_seen, self.shot_id = False, None
        self.previous_state = state
        out = {'kind': 'ball_feedback', 'diagnostic_shot_id': self.shot_id,
               'predictor_state': state, 'source_sequence': seq,
               'source_clock': {'predictor_processing_ros_s': stamp},
               'received_workstation_monotonic_s': received,
               'prediction_fields_valid': False, 'frame': 'mocap_origin',
               'real_input_accepted': False, 'sensor_age_verified': False, 'command_ack': False}
        if state != 'tracking' or not armed or held or self.shot_id is None:
            out['reason'] = 'inactive_disarmed_held_or_started_midshot'
            return out
        tts = number(planner['time_to_strike'])
        if not 0 < tts <= 2:
            out['reason'] = 'expired_or_invalid_tts'
            return out
        ball, prediction, racket = payload['ball'], payload['prediction'], payload['racket']
        if not 0 <= number(ball['age_s']) <= .30:
            out['reason'] = 'producer_reported_ball_stale'
            return out
        out.update(position=vector(ball['current_origin_filtered'], 3),
                   velocity=vector(ball['current_velocity_origin'], 3),
                   strike_position=vector(prediction['position_origin'], 3),
                   strike_velocity=vector(prediction['velocity_origin'], 3),
                   racket_normal=vector(racket['normal_origin'], 3),
                   racket_velocity=vector(racket['velocity_origin'], 3),
                   time_to_strike_s=tts, prediction_fields_valid=True, reason=None)
        out['physical_acceleration_available'] = False
        out['sensor_source_timestamp_available'] = False
        return out
