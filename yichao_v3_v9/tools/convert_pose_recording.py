"""Convert recorded existing ROS pose streams; no live publishers or control."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from yichao_v3_v9.pose_feedback import PoseFeedback, PRIMARY, SECONDARY


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--calibration', type=Path, required=True)
    p.add_argument('--publisher', required=True)
    args = p.parse_args()
    calibration_bytes = args.calibration.read_bytes()
    calibration_hash = hashlib.sha256(calibration_bytes).hexdigest()
    adapter = PoseFeedback(json.loads(calibration_bytes), calibration_hash, args.publisher)
    counts = {'66': 0, '198': 0}
    failures, latest = [], {}
    with args.input.open() as source, args.output.open('x') as output:
        for line in source:
            if len(line) > 65536:
                raise ValueError('input_record_size_limit')
            record = json.loads(line)
            if record.get('kind') != 'raw_input' or record.get('topic') not in (PRIMARY, SECONDARY):
                continue
            try:
                result = adapter.parse(record)
                counts[result['robot_id']] += 1
                latest[result['robot_id']] = result
                output.write(json.dumps({'kind': 'pose_conversion', 'raw': record, 'torso': result}, allow_nan=False)+'\n')
            except (KeyError, TypeError, ValueError) as exc:
                failures.append(str(exc))
                output.write(json.dumps({'kind': 'pose_rejected', 'reason': str(exc), 'raw': record})+'\n')
        summary = {'kind': 'pose_conversion_summary', 'converted': counts,
                   'rejected': len(failures), 'reasons': sorted(set(failures)), 'latest': latest,
                   'calibration_sha256': calibration_hash,
                   'evidence': 'recorded real poses, configured extrinsics; physical mapping and model frame unaccepted',
                   'real_input_accepted': False, 'command_ack': False, 'control_commands_sent': 0}
        output.write(json.dumps(summary, allow_nan=False)+'\n')
    print(json.dumps(summary, allow_nan=False), flush=True)
    return 0 if all(counts.values()) and not failures else 2


if __name__ == '__main__':
    raise SystemExit(main())
