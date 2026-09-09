"""Pure Yichao command -> existing V9 wire translation; no ROS/network imports.

This proves expressibility only. The stock bridge has no authoritative per-command
ACK contract, so its last_applied_sequence must not advance the Yichao relay.
"""
import copy
import math
from types import SimpleNamespace

from .executor import OfflineExecutor

ROLES = {'66': 'table_right', '198': 'table_left'}


def encode(command, now, base_x, *, allow_hit=False):
    """Prevalidate every source field before constructing any downstream request."""
    c = copy.deepcopy(command)
    if type(allow_hit) is not bool:
        raise ValueError('allow_hit must be an explicit boolean')
    if not isinstance(c, dict) or not isinstance(c.get('session'), str) or not c['session']:
        raise ValueError('explicit source session required')
    if isinstance(base_x, bool) or not isinstance(base_x, (int, float)) or not math.isfinite(base_x):
        raise ValueError('finite base X required')
    # Reuse the tested complete source-envelope validator without constructing
    # schedulers or motion banks. _validate only reads the session member.
    OfflineExecutor._validate(SimpleNamespace(session=c['session']), c, now)
    if c['kind'] == 'hit' and not allow_hit:
        raise ValueError('HIT disabled for this adapter')
    goal = [float(base_x), c['target_y']]
    fields = {'role': 'stage', 'active': False,
              'desired_base_position': goal, 'trajectory_base_position': goal}
    wire = {'schema_version': 'v9-planner-command-v1', 'robot': ROLES[c['robot']],
            'session_id': c['session'], 'sequence': c['sequence'], 'valid': True,
            'planner_mode': 'yichao_v3', 'commit_token': c['token'],
            'safety_active': True, 'safety_override': False,
            'return_target_y': c.get('return_y'), 'command': fields}
    if c['kind'] == 'clear':
        fields['role'] = 'clear'
    elif c['kind'] == 'return':
        # The stock V9 fixed-return branch is the explicit base-target request.
        wire['planner_mode'] = 'fixed_relay'
        wire['return_target_y'] = c['target_y']
        fields['role'] = 'return'
    elif c['kind'] == 'hit':
        hit = c['hit']
        fields.update(role='hit', active=True,
                      predicted_ball_position=hit['position'],
                      predicted_racket_velocity=hit['racket_velocity'],
                      predicted_ball_velocity=hit['ball_velocity'],
                      predicted_ball_predict_time=hit['tts']-(now-c['issued_at']))
        wire['post_hit_outward_y'] = hit['outward_y']
        wire['return_target_y'] = hit['return_y']
    return wire


def inspect_stock_feedback(command, state):
    """Correlate telemetry for diagnostics; never synthesize accepted/completed ACK."""
    expected = ROLES.get(command.get('robot'))
    return {'kind': 'stock_v9_feedback_diagnostic', 'command_ack': False,
            'real_execution_confirmed': False,
            'identity_matches': state.get('robot') == expected
                and state.get('last_planner_session_id') == command.get('session'),
            'sequence_matches': state.get('last_applied_sequence') == command.get('sequence'),
            'token_matches': state.get('last_commit_token') == command.get('token'),
            'reported_error': state.get('transport_error'),
            'reason': 'stock_bridge_sequence_is_not_authoritative_ack'}
