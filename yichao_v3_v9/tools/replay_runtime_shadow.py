"""File-only replay of existing ROS captures through the diagnostic V3 runtime."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from yichao_v3_v9.runtime_shadow import RuntimeShadow, INPUT_TOPICS, MONITOR, DOUBLES_BALL
from yichao_v3_v9.inputs import Features
from yichao_v3_v9.inference import Pipeline
from yichao_v3_v9.observer import Writer


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--actor', type=int, choices=(190, 199), default=199)
    p.add_argument('--ball-topic', choices=(MONITOR, DOUBLES_BALL), default=MONITOR)
    args = p.parse_args()
    if args.input.stat().st_size > 16*1024*1024:
        p.error('input exceeds 16 MiB')
    pipeline = Pipeline(args.actor)
    pipeline.decide(Features(np.zeros(36, np.float32), np.zeros(85, np.float32), {}, 'synthetic'))
    runtime = RuntimeShadow('recording-replay', pipeline, ball_topic=args.ball_topic)
    with args.input.open() as src, args.output.open('x') as dst:
        write = Writer(dst)
        for line in src:
            if len(line) > 65536:
                raise ValueError('record exceeds 64 KiB')
            record = json.loads(line)
            if record.get('kind') != 'raw_input' or record.get('topic') not in INPUT_TOPICS:
                continue
            runtime.ingest(record)
            write(runtime.tick(record['receive_monotonic_s']))
        result = runtime.summary()
        write(result)
    print(json.dumps(result))
    return 2 if runtime.fault else 0


if __name__ == '__main__':
    raise SystemExit(main())
