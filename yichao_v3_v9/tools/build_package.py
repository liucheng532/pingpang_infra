#!/usr/bin/env python3
"""Build a Planner-only package; controller/predictor references are excluded."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

ROOT=Path(__file__).resolve().parents[1]


def selected_files():
    paths=[]
    for directory in ('src','config','baseline/doubles_planner','vendor/training_assets'):
        paths.extend(p for p in (ROOT/directory).rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc')
    for name in ('README.md','MONITOR.md','scripts/run_yichao_planner.py','scripts/run_yichao_stack.py',
                 'baseline/scripts/run_doubles_stack.py','baseline/SOURCE_MANIFEST.json',
                 'scripts/start_yichao_workstation.sh','scripts/start_yichao_robots.sh'):
        paths.append(ROOT/name)
    model_dir=ROOT/'vendor/handoff/20260906_v3_planner_190_199/onnx'
    paths.extend(model_dir/name for name in ('rl_planner_actor_model_190.onnx','rl_planner_actor_model_199.onnx','safe_filter_v3.onnx'))
    return sorted(set(paths))


def build(destination):
    destination=Path(destination);destination.mkdir(parents=True,exist_ok=True)
    files=selected_files()
    manifest={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (destination/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with tarfile.open(destination/'planner.tar.gz','w:gz') as archive:
        for p in files:archive.add(p,arcname=str(p.relative_to(ROOT)),recursive=False)
        archive.add(destination/'manifest.json',arcname='DEPLOYMENT_MANIFEST.json')
    return {'files':len(files),'bytes':sum(p.stat().st_size for p in files),'archive':str(destination/'planner.tar.gz')}


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('destination',type=Path)
    print(json.dumps(build(parser.parse_args().destination),indent=2))
