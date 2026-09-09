"""Strict records plus the 36D/85D V3 ABI, independent of callback frequency."""
from dataclasses import dataclass
import copy
import numpy as np

SLOTS = ('66', '198')  # training left/negative/mirrored, right/positive/canonical
PHASES = ('HIT', 'POST_DELAY', 'OUTWARD', 'OUTWARD_HOLD', 'RETURN', 'HOME_HOLD')


def array(value, shape, label):
    result = np.asarray(value, dtype=np.float32)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{label}: expected finite {shape}')
    return result


def finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
        raise ValueError(f'{label}: expected finite number')
    return float(value)


@dataclass
class Features:
    actor: np.ndarray
    safe: np.ndarray
    record: dict
    provenance: str


class InputAdapter:
    def __init__(self, contract):
        self.contract = contract
        self.last_now = None
        self.last_sources = {}

    def build(self, record, previous_targets):
        # The reference Planner package requires Python >=3.10. The onboard
        # Python 3.8 receiver only needs the validation helpers and scheduler;
        # keep training/reference imports inside the workstation-only builder.
        from doubles_planner.real_inputs import (RealInputAdapter, G1RobotInput, UnitreeG1LowState,
            G1BaseEstimate, G1ControllerTelemetry, RealBallPrediction)
        r = copy.deepcopy(record)
        now = finite(r['now'], 'now')
        if self.last_now is not None and now <= self.last_now:
            raise ValueError('clock_jump_or_out_of_order_record')
        self.last_now = now
        provenance = r['provenance']
        if provenance not in ('synthetic', 'real'):
            raise ValueError('unknown provenance')
        if provenance == 'real':
            # No data record can self-authorize missing training/calibration evidence.
            raise ValueError('real_interface_not_accepted: coordinate, history, joint limits, initial memory and clock evidence pending')
        if r['frame'] != 'training_common_fixture' or r['clock_domain'] != 'replay_monotonic':
            raise ValueError('unverified_frame_or_clock')
        if r['emergency_stop'] is not False:
            raise ValueError('emergency_stop')
        dt = finite(self.contract['fixture_history_dt_s'], 'history_dt')
        timestamps = np.asarray(r['base_history_times'], dtype=np.float64)
        if timestamps.shape != (5,) or not np.isfinite(timestamps).all() or not np.allclose(timestamps, now - np.arange(4,-1,-1)*dt, atol=1e-7, rtol=0):
            raise ValueError('base_history_sampling_grid')
        if set(r['robots']) != set(SLOTS):
            raise ValueError('robot_slot_mapping')
        robots, histories, positions, joints = [], [], [], []
        source_updates = {}
        for slot, name in zip(SLOTS, ('left', 'right')):
            raw = r['robots'][slot]
            if raw['robot_id'] != slot or raw['role'] != {'66':'table_right','198':'table_left'}[slot]:
                raise ValueError('robot_identity')
            expected_times = {}
            for key in ('low', 'base', 'controller'):
                stamp = raw['timestamps'][key]
                expected_times[key] = self._stamp(stamp, now, .25, slot+'/'+key, source_updates)
            history = array(raw['base_history'], (5,3), 'base_history')
            position = array(raw['base_position'], (3,), 'base_position')
            if not np.allclose(history[-1], position, atol=1e-6, rtol=0):
                raise ValueError('base_history_endpoint')
            if abs(expected_times['base']-timestamps[-1]) > 1e-6:
                raise ValueError('base_source_history_timestamp')
            for key in ('imu_quaternion','base_quaternion'):
                quat = array(raw[key], (4,), key)
                if abs(float(np.linalg.norm(quat))-1) > .02:
                    raise ValueError('invalid_quaternion')
            if raw['phase'] not in PHASES or type(raw['ready']) is not bool or type(raw['valid']) is not bool:
                raise ValueError('invalid_phase_or_flags')
            array(raw['dq'], (29,), 'dq')
            low = UnitreeG1LowState(raw['q'], raw['dq'], raw['imu_quaternion'], raw['gyro'], expected_times['low'], valid=raw['valid'])
            base = G1BaseEstimate(position, raw['base_quaternion'], raw['base_velocity'], raw['base_angular_velocity'], expected_times['base'], 'synthetic_reference', valid=raw['valid'], position_history_xyz=history, history_timestamps=timestamps, history_valid=np.ones(5,dtype=bool))
            ctrl = G1ControllerTelemetry(raw['phase'], raw['ready'], expected_times['controller'], raw['phase_elapsed_s'], raw['stable_elapsed_s'], valid=raw['valid'])
            robots.append(G1RobotInput(name, low, base, ctrl))
            if raw['joint_order'] != self.contract['fixture_joint_order']:
                raise ValueError('joint_order')
            limits = array(self.contract['fixture_joint_limits'], (29,2), 'joint_limits')
            half = (limits[:,1]-limits[:,0])/2
            if np.any(half <= 1e-5):
                raise ValueError('invalid_soft_limits')
            q = array(raw['q'], (29,), 'q')
            joints.append(np.clip((q-limits.mean(axis=1))/half,-1,1))
            histories.append(history)
            positions.append(position)
        ball = r['ball']
        bt = self._stamp(ball['timestamp'], now, .30, 'ball', source_updates)
        tts = finite(ball['time_to_strike_s'], 'tts') - (now-bt)
        if not 0 < tts <= 2:
            raise ValueError('invalid_or_expired_tts')
        ball['time_to_strike_s'] = tts
        ball['decision_timestamp'] = now
        bp = RealBallPrediction(ball['position'], ball['velocity'], tts, ball['racket_normal'], ball['racket_velocity'], bt,
            acceleration=ball['acceleration'], predicted_strike_position=ball['strike_position'], predicted_strike_velocity=ball['strike_velocity'], shot_id=r['shot_id'], valid=ball['valid'])
        _, _, reasons = RealInputAdapter(tuple(robots), bp).traditional_inputs(now)
        if reasons:
            raise ValueError(','.join(reasons))
        if not isinstance(r['shot_id'], str) or not r['shot_id']:
            raise ValueError('missing_shot_id')
        acceleration = array(ball['acceleration'], (3,), 'acceleration')
        # Extrapolate state to decision time, then reconstruct five past points.
        elapsed = now-bt
        position = bp.position + bp.velocity*elapsed + .5*acceleration*elapsed**2
        velocity = bp.velocity + acceleration*elapsed
        ages = np.arange(4,-1,-1,dtype=np.float32)*dt
        ball_history = position[None,:]-velocity[None,:]*ages[:,None]+.5*acceleration*ages[:,None]**2
        relative = np.stack((ball_history-histories[0],ball_history-histories[1]), axis=1)
        relative = np.clip(relative/np.array([3,1.5,1.5],np.float32),-3,3).reshape(-1)
        bases = np.clip(np.asarray(positions)[:,:2]/np.array([3,1.2],np.float32),-3,3).reshape(-1)
        previous = array(previous_targets,(2,), 'previous_confirmed_targets')
        if np.any(np.abs(previous)>1.2):
            raise ValueError('previous_target_out_of_workspace')
        actor = np.concatenate((relative,bases,previous/np.float32(1.2))).astype(np.float32)
        base_features = [np.clip(h[:,:2]/np.array([3,1.2],np.float32),-3,3).reshape(-1) for h in histories]
        strike = np.clip(np.concatenate((bp.predicted_strike_position/np.array([3,1.5,1.5],np.float32),bp.racket_velocity/4,[tts])),-3,3)
        safe = np.concatenate((*base_features,*joints,strike)).astype(np.float32)
        self.last_sources.update(source_updates)
        return Features(array(actor,(36,),'actor'),array(safe,(85,),'safe'),r,provenance)

    def _stamp(self, stamp, now, timeout, name, updates):
        if stamp['domain'] != 'replay_monotonic':
            raise ValueError('cross_machine_clock_unverified')
        source = finite(stamp['source'], 'source_timestamp')
        received = finite(stamp['received'], 'receive_timestamp')
        uncertainty = finite(stamp['uncertainty_s'], 'clock_uncertainty')
        if uncertainty < 0 or source-uncertainty > received or received > now or now-source+uncertainty > timeout or source+uncertainty > now:
            raise ValueError('stale_future_or_uncertain_timestamp:'+name)
        if source < self.last_sources.get(name, -float('inf')):
            raise ValueError('out_of_order_source:'+name)
        updates[name] = source
        return source
