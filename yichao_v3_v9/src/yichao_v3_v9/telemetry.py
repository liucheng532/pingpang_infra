"""Parse live wire formats for diagnostics. Never constructs an accepted V3 input."""
import json
import math

TOPICS = {
    '/doubles/ball_prediction': ('ball', 'std_msgs/String'),
    '/doubles/table_left/state': ('state', 'std_msgs/String'),
    '/doubles/table_right/state': ('state', 'std_msgs/String'),
    '/doubles/table_left/torso_pose_origin': ('torso', 'geometry_msgs/PoseStamped'),
    '/doubles/table_right/torso_pose_origin': ('torso', 'geometry_msgs/PoseStamped'),
}
PHASES = {'HIT', 'POST_DELAY', 'OUTWARD', 'OUTWARD_HOLD', 'RETURN', 'HOME_HOLD'}
SLOTS = {'table_right': {'robot': 66, 'training_slot': 0, 'mapping_verified': False},
         'table_left': {'robot': 198, 'training_slot': 1, 'mapping_verified': False}}
UNRESOLVED = ['common_coordinates_and_heading', 'training_joint_order_and_soft_limits',
              'training_history_grid', 'source_clock_and_sensor_age',
              'confirmed_initial_applied_target']


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('finite number required')
    return float(value)


def integer(value, positive=False):
    if type(value) is not int or value < (1 if positive else 0):
        raise ValueError('nonnegative integer required')
    return value


def vector(value, size, quaternion=False):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError('vector dimension must be %s' % size)
    result = [number(x) for x in value]
    if quaternion and abs(sum(x*x for x in result) - 1.0) > .02:
        raise ValueError('quaternion must have unit norm; no silent normalization')
    return result


def boolean(value):
    if type(value) is not bool:
        raise ValueError('boolean required')
    return value


def strict_json(text):
    if not isinstance(text, str) or len(text.encode('utf-8')) > 65536:
        raise ValueError('JSON message exceeds 64 KiB or is not text')
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('duplicate JSON field: ' + key)
            result[key] = value
        return result
    def constant(value):
        raise ValueError('nonfinite JSON constant: ' + value)
    value = json.loads(text, object_pairs_hook=pairs, parse_constant=constant)
    if not isinstance(value, dict):
        raise ValueError('JSON object required')
    return value


class TelemetryParser:
    """Strict diagnostic schema, per-source ordering, and receive-gap checks.

    Recent reception cannot prove sensor freshness. Robot publication monotonic
    clocks are never subtracted from the workstation clock. Ordering failures
    remain latched until a new observer run; a restart is not auto-authorized.
    """
    def __init__(self):
        self.last = {}
        self.faults = {}

    def parse(self, topic, payload, receive, provenance='live_ros'):
        out = {'topic': topic, 'provenance': provenance, 'wire_valid': False,
               'real_input_accepted': False, 'sensor_age_verified': False,
               'command_ack': False, 'unresolved': list(UNRESOLVED), 'receive': receive}
        try:
            if topic not in TOPICS:
                raise ValueError('topic outside input allowlist')
            kind = TOPICS[topic][0]
            mono = number(receive['monotonic_s'])
            wall = number(receive['wall_s'])
            ros = number(receive['ros_s']) if receive.get('ros_s') is not None else None
            if mono < 0 or wall <= 0 or (ros is not None and ros <= 0):
                raise ValueError('invalid receive clock')
            if not isinstance(payload, dict):
                raise ValueError('payload must be an object')
            d = payload
            role = topic.split('/')[2] if kind != 'ball' else None
            if kind == 'ball':
                if d['schema_version'] != 'v9-ball-prediction-v1':
                    raise ValueError('unknown ball schema')
                sequence = integer(d['sequence'])
                source = number(d['source_timestamp'])
                if source <= 0:
                    raise ValueError('zero or negative source timestamp')
                if not boolean(d['valid']):
                    raise ValueError('producer reports invalid')
                if not isinstance(d['shot_id'], str) or not 0 < len(d['shot_id']) <= 128:
                    raise ValueError('nonempty string shot_id required (Predictor uses mocap-N)')
                for name in ('position', 'velocity', 'predicted_strike_position',
                             'predicted_strike_velocity', 'racket_normal', 'racket_velocity'):
                    vector(d[name], 3)
                if not 0 <= number(d['time_to_strike_s']):
                    raise ValueError('negative time to strike')
                domain = 'predictor_ros_candidate_unverified'
            elif kind == 'state':
                if d['schema_version'] != 'v9-robot-state-v1' or d['robot'] != role:
                    raise ValueError('robot role or state schema mismatch')
                sequence = integer(d['sequence'])
                source = integer(d['source_monotonic_ns'], positive=True)
                if not boolean(d['valid']):
                    raise ValueError('producer reports invalid')
                for name, size in (('q', 29), ('dq', 29), ('gyro_xyz', 3),
                                   ('base_position_xyz', 3), ('base_linear_velocity_xyz', 3),
                                   ('base_angular_velocity_xyz', 3)):
                    vector(d[name], size)
                for name in ('imu_quaternion_wxyz', 'base_orientation_wxyz'):
                    vector(d[name], 4, quaternion=True)
                if d['phase'] not in PHASES:
                    raise ValueError('unknown phase')
                boolean(d['ready'])
                for name in ('state_elapsed_s', 'stable_elapsed_s'):
                    if number(d[name]) < 0:
                        raise ValueError('negative elapsed time')
                for name in ('target_base_y', 'home_y', 'time_to_strike_s'):
                    number(d[name])
                domain = 'robot_%s_publication_monotonic_ns' % SLOTS[role]['robot']
                out['slot_candidate'] = SLOTS[role]
                out['telemetry_is_not_execution_ack'] = True
            else:
                header = d['header']
                sequence = integer(header['seq'])
                secs = integer(header['stamp']['secs'])
                nsecs = integer(header['stamp']['nsecs'])
                if nsecs >= 1_000_000_000 or secs + nsecs == 0:
                    raise ValueError('invalid ROS header stamp')
                source = secs * 1_000_000_000 + nsecs
                if not isinstance(header['frame_id'], str) or not header['frame_id']:
                    raise ValueError('missing coordinate frame')
                vector(d['position_xyz'], 3)
                vector(d['orientation_xyzw'], 4, quaternion=True)
                domain = 'torso_ros_header_ns_unverified'
                out['slot_candidate'] = SLOTS[role]
            out.update(source_time=source, source_domain=domain, sequence=sequence)
            prior = self.last.get(topic)
            if topic in self.faults:
                raise ValueError(self.faults[topic])
            if prior:
                old_seq, old_source, old_mono, old_wall, old_ros = prior
                if sequence <= old_seq or source < old_source:
                    self.faults[topic] = 'duplicate/out-of-order/restart: new run required'
                elif mono <= old_mono or abs((wall-old_wall) - (mono-old_mono)) > .1:
                    self.faults[topic] = 'receive clock jump: new run required'
                elif ros is not None and old_ros is not None and abs((ros-old_ros) - (mono-old_mono)) > .1:
                    self.faults[topic] = 'ROS clock jump: new run required'
                if topic in self.faults:
                    raise ValueError(self.faults[topic])
                out['receive_gap_s'] = mono - old_mono
                out['receive_gap_exceeded'] = mono - old_mono > (.30 if kind == 'ball' else .25)
            if kind in ('ball', 'torso') and ros is not None:
                age = ros - (source if kind == 'ball' else source / 1e9)
                out['candidate_ros_age_s'] = age
                # This can reject obvious errors but never certify sensor freshness.
                if age < -.01 or age > (.30 if kind == 'ball' else .25):
                    raise ValueError('candidate ROS source time is future/stale')
            self.last[topic] = (sequence, source, mono, wall, ros)
            out['wire_valid'] = True
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            out['reason'] = str(exc)
        return out
