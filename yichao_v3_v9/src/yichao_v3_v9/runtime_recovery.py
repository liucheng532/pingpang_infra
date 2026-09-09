"""Restore a logged post-HIT transaction; live receipts must confirm it anew."""
import copy
import json
from pathlib import Path


def restore_handoff(client, source):
    if client.mode != 'active' or not client.experimental_active:
        raise ValueError('active_recovery_required')
    source = Path(source)
    files = [source, *sorted(source.parent.glob(source.name+'.[0-9][0-9][0-9][0-9]'))]
    snapshots = []; commands = {}; balls = {}; finished = set(); summary = None
    for path in files:
        with path.open() as stream:
            for line in stream:
                r = json.loads(line)
                if r.get('kind') == 'summary':
                    summary = r
                elif r.get('kind') == 'relay_step':
                    if r.get('mode') != 'active' or r['relay']['session'] != client.session:
                        raise ValueError('recovery_log_identity')
                    if r['relay']['state'] == 'COMMITTED':
                        snapshots = [r['relay']]
                    finished.update(r.get('completed_shots', []))
                    for c in r.get('commands', []):
                        if c['session'] != client.session:
                            raise ValueError('recovery_command_session')
                        old = commands.setdefault(c['sequence'], c)
                        if old != c:
                            raise ValueError('recovery_sequence_conflict')
                elif r.get('kind') == 'raw_input' and r.get('topic') == '/doubles/ball_prediction':
                    payload = json.loads(r['payload']['data'])
                    if payload.get('valid'):
                        balls[client.session+'/'+payload['shot_id']] = (payload, r['receive_monotonic_s'])
    if not summary or summary.get('mode') != 'active' or summary.get('fault') != 'handoff_timeout':
        raise ValueError('only_logged_handoff_timeout_can_resume')
    if not snapshots:
        raise ValueError('missing_committed_handoff')
    saved = snapshots[-1]; shot = saved['shot']
    current = [c for c in commands.values() if c['shot'] == shot]
    pending = {c['robot']: c for c in current if c['kind'] in ('clear', 'hit')}
    if (len(pending) != 2 or sorted(c['kind'] for c in pending.values()) != ['clear', 'hit']
            or any(c['kind'] == 'return' for c in current)):
        raise ValueError('recovery_requires_unreturned_hit_and_clear')
    hit = next(c for c in pending.values() if c['kind'] == 'hit')
    if hit['robot'] != saved['hitter'] or shot not in balls:
        raise ValueError('recovery_hitter_or_context_missing')
    p, received = balls[shot]
    client.shot_ball = {'prediction_fields_valid': True, 'diagnostic_shot_id': shot,
        'received_workstation_monotonic_s': received,
        'source_clock': {'predictor_processing_ros_s': p['source_timestamp']},
        **{k: copy.deepcopy(p[k]) for k in ('position', 'velocity', 'racket_velocity', 'time_to_strike_s')},
        'strike_position': copy.deepcopy(p['predicted_strike_position']),
        'strike_velocity': copy.deepcopy(p['predicted_strike_velocity'])}
    relay = client.relay
    relay.state = 'COMMITTED'; relay.shot = shot; relay.hitter = saved['hitter']
    relay.sequence = max(commands); relay.seen = {c['shot'] for c in commands.values()}
    relay.previous_targets = copy.deepcopy(saved['previous_applied_targets'])
    relay.decision = copy.deepcopy(saved['decision']); relay.commit_plan = copy.deepcopy(saved['commit_plan'])
    relay.protocol_checks = copy.deepcopy(saved.get('protocol_checks', []))
    relay.pending = copy.deepcopy(pending)
    # Do not treat saved ACKs or positions as current completion evidence.
    relay.accepted = set(); relay.completed = set()
    relay.started = min(c['issued_at'] for c in current); relay.last_hit_at = hit['issued_at']
    relay.fault = None; client.finished = finished; client.resuming = True
    return {'session': client.session, 'shot': shot, 'sequence': relay.sequence,
            'state': relay.state, 'source': str(source), 'live_receipts_required': True,
            'hit_republication_allowed': False}
