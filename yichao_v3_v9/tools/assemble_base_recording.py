"""Join co-recorded diagnostic poses/joints and reconstruct V9 pelvis positions."""
import argparse
import bisect
import collections
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from yichao_v3_v9.pose_feedback import PoseFeedback, PRIMARY, SECONDARY
from yichao_v3_v9.base_feedback import reconstruct_base


def records(path):
    if path.stat().st_size > 16*1024*1024:
        raise ValueError('recording_size_limit')
    with path.open() as stream:
        for line in stream:
            if len(line) > 65536:
                raise ValueError('record_size_limit')
            yield json.loads(line)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--poses', type=Path, required=True)
    p.add_argument('--joints', type=Path, required=True)
    p.add_argument('--calibration', type=Path, required=True)
    p.add_argument('--publisher', required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    data = args.calibration.read_bytes()
    adapter = PoseFeedback(json.loads(data), hashlib.sha256(data).hexdigest(), args.publisher)
    streams = {'66': [], '198': []}
    for record in records(args.joints):
        if record.get('kind') == 'joint_sample':
            joint = record['feedback']
            streams[joint['robot_id']].append(joint)
    times = {}
    for slot, samples in streams.items():
        times[slot] = [x['received_workstation_monotonic_s'] for x in samples]
        if any(a > b for a,b in zip(times[slot], times[slot][1:])):
            raise ValueError('joint_recording_receipts_not_ordered')
        if len({x['session_id'] for x in samples}) > 1:
            raise ValueError('multiple_joint_sessions_in_one_recording')
    counts, rejected, latest, maximum_skew = {'66':0,'198':0}, collections.Counter(), {}, 0.
    with args.output.open('x') as output:
        for record in records(args.poses):
            if record.get('kind') != 'raw_input' or record.get('topic') not in (PRIMARY, SECONDARY):
                continue
            try:
                torso = adapter.parse(record); slot = torso['robot_id']
                i = bisect.bisect_right(times[slot], torso['received_workstation_monotonic_s']) - 1
                if i < 0:
                    raise ValueError('no_previous_joint_sample')
                base = reconstruct_base(torso, streams[slot][i])
                counts[slot] += 1; latest[slot] = base
                maximum_skew = max(maximum_skew, base['joint_receipt_age_s'])
                output.write(json.dumps(base, allow_nan=False)+'\n')
            except (KeyError, TypeError, ValueError) as exc:
                rejected[str(exc)] += 1
                output.write(json.dumps({'kind':'base_rejected','reason':str(exc),
                    'topic':record['topic'],'received_workstation_monotonic_s':record['receive_monotonic_s']})+'\n')
        summary = {'kind':'base_assembly_summary','assembled':counts,'rejected':dict(rejected),
                   'latest':latest,'maximum_joint_receipt_age_s':maximum_skew,
                   'evidence':'co-recorded real samples joined by workstation receipt, sensor-time alignment unverified',
                   'real_input_accepted':False,'command_ack':False,'control_commands_sent':0}
        output.write(json.dumps(summary, allow_nan=False)+'\n')
    print(json.dumps(summary, allow_nan=False), flush=True)
    return 0 if all(counts.values()) else 2


if __name__ == '__main__':
    raise SystemExit(main())
