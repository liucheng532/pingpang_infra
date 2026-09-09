"""Movement-only receiver around a frozen V9 scheduler, with explicit feedback.

The caller owns transport, sensor freshness and control admission. This class
never publishes low commands and cannot admit HIT, including on retransmission.
"""
import copy
import hashlib
import json
import math
import numpy as np

from .executor import clone_scheduler
from .inputs import array, finite


SCHEMA = 'yichao-v9-movement-v1'
FEEDBACK = 'yichao-v9-movement-feedback-v1'


class MovementReceiver:
    def __init__(self, session, robot, scheduler, *, maximum_step_m=.05):
        if not isinstance(session, str) or not session or robot not in ('66', '198'):
            raise ValueError('receiver_identity')
        if not 0 < finite(maximum_step_m, 'maximum_step_m') <= .05:
            raise ValueError('first_movement_step_must_not_exceed_5cm')
        self.session, self.robot, self.scheduler = session, robot, scheduler
        self.maximum_step_m = maximum_step_m
        self.sequence = -1
        self.history = {}
        self.active = None
        self.stable_since = None
        self.last_observation = None

    def apply(self, command, *, workstation_time_interval, position, sensors_recent):
        """Time interval is obtained from a bounded clock exchange, not reception.

        The pair [earliest, latest] expresses current robot time in workstation
        monotonic seconds, including uncertainty. Both ends must be in the TTL.
        """
        result = {'schema': FEEDBACK, 'session': self.session, 'robot': self.robot,
                  'status': 'rejected', 'hit_enabled': False, 'completed': False}
        if isinstance(command, dict):
            result.update({k: command.get(k) for k in ('sequence', 'shot', 'token')})
        try:
            required = {'schema', 'session', 'robot', 'sequence', 'shot', 'token',
                        'issued_at', 'expires_at', 'target_y', 'kind'}
            if not isinstance(command, dict) or set(command) != required or command['schema'] != SCHEMA:
                raise ValueError('movement_schema')
            c = command
            if c['session'] != self.session or c['robot'] != self.robot or c['kind'] != 'move':
                raise ValueError('identity_or_kind')
            for k in ('shot', 'token'):
                if not isinstance(c[k], str) or not 0 < len(c[k]) <= 256:
                    raise ValueError('invalid_'+k)
            if type(c['sequence']) is not int or c['sequence'] < 0:
                raise ValueError('sequence')
            target = finite(c['target_y'], 'target_y')
            issued, expires = finite(c['issued_at'], 'issued_at'), finite(c['expires_at'], 'expires_at')
            if abs(target) > 1.2 or not 0 < expires-issued <= .25+1e-9:
                raise ValueError('workspace_or_ttl')
            if not isinstance(workstation_time_interval, (tuple, list)) or len(workstation_time_interval) != 2:
                raise ValueError('clock_interval_required')
            early, late = [finite(x, 'clock_bound') for x in workstation_time_interval]
            if not early <= late or late-early > .02:
                raise ValueError('clock_uncertainty')
            digest = hashlib.sha256(json.dumps(c, sort_keys=True, allow_nan=False).encode()).hexdigest()
            cached = self.history.get(c['sequence'])
            if cached:
                if digest != cached[0]:
                    raise ValueError('sequence_payload_conflict')
                return copy.deepcopy(cached[1])  # report prior result, never apply twice
            if not issued <= early <= late <= expires:
                raise ValueError('expired_or_future_movement')
            if sensors_recent is not True:
                raise ValueError('sensor_freshness_required')
            if c['sequence'] <= self.sequence:
                raise ValueError('old_sequence')
            if any(row[1].get('token') == c['token'] for row in self.history.values()):
                raise ValueError('reused_token')
            if self.active and not self.active['completed']:
                raise ValueError('movement_still_in_progress')
            current = array(position, (3,), 'current_pelvis')
            if abs(target-float(current[1])) > self.maximum_step_m+1e-6:
                raise ValueError('pilot_displacement_limit')
            if self.scheduler.state.name in ('HIT', 'POST_DELAY'):
                raise ValueError('hit_locked')
            preview = clone_scheduler(self.scheduler)
            preview.set_external_base_target(np.array([current[0], target], np.float32),
                                             return_target_y=target, safety_override=False)
            applied = float(current[1]+preview.output().target_base[1])
            if abs(applied-target) > 1e-4:
                raise ValueError('scheduler_did_not_latch_requested_target')
            # Preserve the scheduler object referenced by the hardware agent.
            destination = getattr(self.scheduler, 'scheduler', self.scheduler)
            source = getattr(preview, 'scheduler', preview)
            destination.__dict__.update(source.__dict__)
            self.sequence = c['sequence']
            result.update(status='accepted', applied_target_y=applied,
                          phase=self.scheduler.state.name, reason=None,
                          evidence='dedicated_receiver_after_scheduler_application')
            self.active = copy.deepcopy(result)
            self.stable_since = None
            self.last_observation = None
            self.history[c['sequence']] = (digest, copy.deepcopy(result))
            return result
        except (KeyError, TypeError, ValueError, RuntimeError) as exc:
            return {**result, 'reason': str(exc)}

    def observe(self, *, now, position, velocity_y, sensors_recent):
        if self.active is None:
            return None
        if self.active['completed']:
            return copy.deepcopy(self.active)
        now = finite(now, 'observation_time')
        p = array(position, (3,), 'pelvis')
        velocity = finite(velocity_y, 'velocity_y')
        contiguous = self.last_observation is None or 0 < now-self.last_observation <= .05+1e-9
        self.last_observation = now
        at_goal = abs(float(p[1])-self.active['applied_target_y']) <= .02
        hold = self.scheduler.state.name in ('HOME_HOLD', 'OUTWARD_HOLD')
        if not (sensors_recent is True and contiguous and at_goal and hold and abs(velocity) <= .1):
            self.stable_since = None
        elif self.stable_since is None:
            self.stable_since = now
        complete = self.stable_since is not None and now-self.stable_since >= .1-1e-9
        self.active.update(completed=complete, status='completed' if complete else 'accepted',
                           phase=self.scheduler.state.name, position_y=float(p[1]),
                           stable_elapsed_s=0. if self.stable_since is None else now-self.stable_since)
        digest, _ = self.history[self.sequence]
        self.history[self.sequence] = (digest, copy.deepcopy(self.active))
        return copy.deepcopy(self.active)
