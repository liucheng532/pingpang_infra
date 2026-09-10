"""Yichao's 36D/85D ABI built from the existing Fixed state/ball messages.

Robot slots are physical negative/positive Y: table_right (66), table_left (198).
The bridge already exports physical LAB-order joints and pelvis in origin frame;
there is no second FK, table transform or left/right reflection here.
"""
from collections import deque
from dataclasses import dataclass
import copy
import numpy as np

ROBOT_ORDER = ('table_right', 'table_left')
HISTORY_DT = .02


def array(value, shape, label):
    result = np.asarray(value, dtype=np.float32)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f'{label}: expected finite {shape}')
    return result


@dataclass
class Features:
    actor: np.ndarray
    safe: np.ndarray
    record: dict
    provenance: str = 'fixed_ros_receipt_grid'


def build_vectors(histories, joints, ball, previous_targets, *, age=0., acceleration=(0., 0., -9.81), history_lag=0., current_bases=None):
    """Pure ABI conversion; acceleration is an explicit ballistic assumption."""
    h = array(histories, (2, 5, 3), 'base_history')
    q = array(joints, (2, 29), 'normalized_joints')
    previous = array(previous_targets, (2,), 'previous_applied_targets')
    a = array(acceleration, (3,), 'ball_acceleration')
    p = array(ball['position'], (3,), 'ball_position')
    v = array(ball['velocity'], (3,), 'ball_velocity')
    if not np.isfinite(age) or age < 0:
        raise ValueError('invalid_ball_age')
    p = p + v * age + .5 * a * age ** 2
    v = v + a * age
    ages = np.arange(4, -1, -1, dtype=np.float32) * HISTORY_DT + history_lag
    bh = p - v * ages[:, None] + .5 * a * ages[:, None] ** 2
    relative = np.stack((bh-h[0], bh-h[1]), axis=1)
    relative = np.clip(relative / np.array([3., 1.5, 1.5], np.float32), -3, 3).reshape(-1)
    positions = h[:, -1, :] if current_bases is None else array(current_bases, (2, 3), 'current_bases')
    bases = np.clip(positions[:, :2] / np.array([3., 1.2], np.float32), -3, 3).reshape(-1)
    actor = np.concatenate((relative, bases, np.clip(previous/1.2, -1, 1)))
    base_xy = np.clip(h[:, :, :2] / np.array([3., 1.2], np.float32), -3, 3).reshape(-1)
    tts = float(ball['time_to_strike_s']) - age
    strike = np.clip(np.concatenate((array(ball['predicted_strike_position'], (3,), 'strike') / [3., 1.5, 1.5],
                                    array(ball['racket_velocity'], (3,), 'racket_velocity') / 4., [tts])), -3, 3)
    safe = np.concatenate((base_xy, q.reshape(-1), strike))
    return Features(array(actor, (36,), 'actor'), array(safe, (85,), 'safe'),
                    {'base_history': h.tolist(), 'normalized_joints': q.tolist(),
                     'previous_targets': previous.tolist(), 'ball': copy.deepcopy(ball),
                     'ball_age_s': age, 'acceleration_assumption': a.tolist(), 'history_dt_s': HISTORY_DT})


class FixedStateFeatures:
    def __init__(self, normalizer):
        self.normalizer = normalizer
        self.samples = {name: deque(maxlen=64) for name in ROBOT_ORDER}

    def ingest(self, robot, state, received):
        samples = self.samples[robot]
        if samples and (received <= samples[-1][0] or
                        state['source_monotonic_ns'] <= samples[-1][1]['source_monotonic_ns']):
            samples.clear()  # source restart/clock discontinuity: refill, never latch a fault
        samples.append((float(received), copy.deepcopy(state)))

    def build(self, states, ball, now):
        if ball is None or not ball.get('valid'):
            raise ValueError('no_valid_ball')
        if not 0 <= now-ball['_receive_monotonic'] <= .30:
            raise ValueError('stale_ball')
        histories, joints, previous = [], [], []
        for name in ROBOT_ORDER:
            state = states.get(name)
            if state is None or not state['valid'] or state['emergency_stop']:
                raise ValueError('invalid_state:'+name)
            if not 0 <= now-state['_receive_monotonic'] <= .25:
                raise ValueError('stale_state:'+name)
            samples = self.samples[name]
            # One common receipt grid, ending at the older robot's latest sample.
            end = min(s['_receive_monotonic'] for s in states.values())
            grid = end - np.arange(4, -1, -1) * HISTORY_DT
            times = np.array([s[0] for s in samples])
            if len(times) < 2 or times[0] > grid[0]+1e-9 or times[-1] < grid[-1]-1e-9:
                raise ValueError('warming_history:'+name)
            relevant = times[(times >= grid[0]-.1) & (times <= end+.1)]
            if len(relevant) > 1 and np.max(np.diff(relevant)) > .1:
                raise ValueError('history_gap:'+name)
            positions = np.asarray([s[1]['base_position_xyz'] for s in samples])
            histories.append(np.stack([np.interp(grid, times, positions[:, i]) for i in range(3)], axis=1))
            joints.append(self.normalizer.normalize(state['q'], self.normalizer.profile['joint_names']))
            previous.append(float(state['target_base_y']))
        features = build_vectors(histories, joints, ball, previous, age=now-ball['_receive_monotonic'],
                                 history_lag=now-end,
                                 current_bases=[states[n]['base_position_xyz'] for n in ROBOT_ORDER])
        features.record['history_end_receive_monotonic'] = end
        features.record['decision_monotonic'] = now
        return features
