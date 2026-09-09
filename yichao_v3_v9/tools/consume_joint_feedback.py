"""Bounded stdin joint stream consumer. Records diagnostics; publishes nothing.

Each line is {robot_id, session_id, payload}. The launcher supplies robot identity
from its SSH connection, not from a user-controlled payload field. This process
timestamps receipt on the workstation and preserves the original robot clock.
An SSH relay is diagnostic transport, not an accepted control-time input link.
"""
import argparse
import json
import os
from pathlib import Path
import selectors
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from yichao_v3_v9.joint_feedback import JointFeedback
from yichao_v3_v9.telemetry import strict_json


def consume(output, seconds=30):
    if not 0 < seconds <= 30:
        raise ValueError('duration must be in (0,30] seconds')
    stream = JointFeedback()
    accepted = {'66': 0, '198': 0}
    rejected, recorded_bytes = 0, 0
    pending = b''
    deadline = time.monotonic() + seconds
    source_summaries = {}
    stop = 'eof'
    with Path(output).open('x') as log, selectors.DefaultSelector() as poll:
        def write(record):
            nonlocal recorded_bytes
            line = json.dumps(record, allow_nan=False) + '\n'
            recorded_bytes += len(line.encode())
            if recorded_bytes > 16 * 1024 * 1024:
                raise ValueError('output_size_limit')
            log.write(line)

        write({'kind': 'capture_start', 'transport': 'stdin SSH diagnostic relay',
               'started_workstation_wall_s': time.time(), 'real_input_accepted': False,
               'control_publishers_created': 0})
        poll.register(sys.stdin.fileno(), selectors.EVENT_READ)
        while time.monotonic() < deadline:
            if not poll.select(timeout=min(.25, max(0., deadline-time.monotonic()))):
                continue
            chunk = os.read(sys.stdin.fileno(), 65536)
            if not chunk:
                if pending:
                    rejected += 1
                    write({'kind': 'input_rejected', 'reason': 'truncated_final_record'})
                break
            pending += chunk
            if len(pending) > 131072:
                stop = 'input_buffer_limit'; break
            while b'\n' in pending:
                line, pending = pending.split(b'\n', 1)
                now = time.monotonic()
                try:
                    envelope = strict_json(line.decode())
                    envelope['received_workstation_monotonic_s'] = now
                    slot = envelope['robot_id']
                    if not isinstance(slot, str) or slot not in accepted:
                        raise ValueError('unknown_robot_id')
                    raw = envelope['payload']
                    if not isinstance(raw, dict):
                        raise ValueError('joint_payload_must_be_object')
                    if raw.get('kind') == 'summary':
                        source_summaries[slot] = raw
                        write({'kind': 'source_summary', 'robot_id': slot, 'payload': raw})
                        continue
                    feedback = stream.ingest(envelope)
                    accepted[slot] += 1
                    write({'kind': 'joint_sample', 'envelope': envelope, 'feedback': feedback})
                except (KeyError, TypeError, ValueError, UnicodeError) as exc:
                    rejected += 1
                    write({'kind': 'input_rejected', 'reason': str(exc),
                           'received_workstation_monotonic_s': now})
            if len(pending) > 65536:
                stop = 'record_size_limit'; break
        else:
            stop = 'duration_limit'
        summary = {'kind': 'capture_summary', 'valid_joint_samples': accepted,
                   'rejected': rejected, 'stop_reason': stop,
                   'source_summaries': source_summaries,
                   'pair_at_exit': stream.snapshot(time.monotonic()),
                   'control_publishers_created': 0, 'real_input_accepted': False}
        write(summary)
    print(json.dumps(summary, allow_nan=False), flush=True)
    return 0 if all(accepted.values()) and not rejected and stop == 'eof' else 2


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=30)
    args = parser.parse_args()
    raise SystemExit(consume(args.output, args.seconds))
