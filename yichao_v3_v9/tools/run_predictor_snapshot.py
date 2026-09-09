"""Predictor with frozen code, current shared calibration and private logs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from yichao_v3_v9.observer import graph
from yichao_v3_v9.predictor_calibration import capture_calibration


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True)
    p.add_argument('--seconds',type=float,default=180.)
    p.add_argument('--until-interrupted',action='store_true',help='keep this private input source alive for an operator-controlled session')
    args=p.parse_args()
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',args.session) or not 0<args.seconds<=300:
        p.error('safe session and duration <=300 seconds required')
    manifest=json.loads((ROOT/'config/predictor_snapshot.json').read_text())
    snapshot=Path(manifest['snapshot_root']);snapshot.resolve().relative_to(ROOT.resolve())
    for name,digest in manifest['sha256'].items():
        path=ROOT/name;path.resolve().relative_to(ROOT.resolve())
        if hashlib.sha256(path.read_bytes()).hexdigest()!=digest:
            raise RuntimeError('Predictor snapshot changed: '+name)
    pubs,_=graph()
    protected=('/doubles/ball_prediction','/table_tennis_planner_monitor','/torso_pose_origin',
        '/doubles/table_left/torso_pose_origin','/doubles/table_right/torso_pose_origin',
        '/doubles/table_left/command','/doubles/table_right/command')
    if any(pubs.get(t) for t in protected):
        raise RuntimeError('existing Predictor/control publisher; preserve existing run')
    for entry in Path('/proc').glob('[0-9]*/cmdline'):
        if entry.parent.name==str(os.getpid()):continue
        try:tokens=entry.read_bytes().decode().split(chr(0))
        except (OSError,UnicodeDecodeError):continue
        if any(Path(t).name in ('TableTennis.py','run_predictor_snapshot.py') for t in tokens if t):
            raise RuntimeError('Predictor process already exists')
    dest=ROOT/'output/predictor_runs'/args.session
    dest.mkdir(parents=True,exist_ok=False)
    calibration=capture_calibration(snapshot,manifest['calibration_source'],dest)
    env=os.environ.copy()
    env.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',
        OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:11311',
        ROS_IP='172.16.3.126',PYTHONPATH=str(ROOT/'vendor/ros1'),
        PINGPANG_CALIBRATION_CONFIG=calibration['snapshot_path'],
        CHINGMU_SDK_DIR=str(snapshot/'sdk'),CHINGMU_ENABLE_LOG='0')
    env.pop('ROS_HOSTNAME',None)
    # Redirect the Predictor's existing configurable module value before it
    # is imported by TableTennis; every run gets its own CSV.
    code=('import sys;sys.path[:0]='+repr([str(snapshot),str(ROOT/'src')])+
          ';import params;params.PREDICTION_DATA_PATH='+repr(str(dest/'prediction_data.csv'))+
          ';import Mocap;from yichao_v3_v9.predictor_freshness import install;install(Mocap,'+
          repr(str(dest/'mocap_source.jsonl'))+');import TableTennis;TableTennis.__main__()')
    command=[str(ROOT/'.venv/bin/python'),'-B','-u','-c',code]
    record={'session':args.session,'snapshot':str(snapshot),'duration_s':None if args.until_interrupted else args.seconds,
            'command':command,'log_dir':str(dest),'controller_started':False,
            'source_freshness_extension':True,'camera_capture_age_verified':False,
            'calibration':calibration}
    (dest/'run.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({'calibration':calibration}),flush=True)
    with (dest/'stdout.log').open('x') as out,(dest/'stderr.log').open('x') as err:
        child=subprocess.Popen(command,cwd=dest,env=env,stdout=out,stderr=err,start_new_session=True)
        record['owned_pid']=child.pid
        (dest/'run.json').write_text(json.dumps(record,indent=2)+'\n')
        print(json.dumps({'started':True,'owned_pid':child.pid,'log_dir':str(dest)}),flush=True)
        def stop(_sig,_frame):
            record['stop_reason'] = 'launcher_signal'
            # Interrupt wait immediately so finally owns the complete bounded
            # shutdown. Merely signalling a stuck SDK child left the launcher
            # waiting for the original duration and could orphan that child.
            raise KeyboardInterrupt
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
        try:
            try:child.wait(timeout=None if args.until_interrupted else args.seconds)
            except subprocess.TimeoutExpired:record.setdefault('stop_reason', 'duration_limit')
            except KeyboardInterrupt:pass
        finally:
            record['shutdown_signals'] = []
            if child.poll() is None:
                record['shutdown_signals'].append('SIGINT')
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    record['shutdown_signals'].append('SIGTERM')
                    child.terminate()
                    try:child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        record['shutdown_signals'].append('SIGKILL')
                        child.kill();child.wait()
            record.setdefault('stop_reason', 'child_exit')
            record['exit_code']=child.returncode
            (dest/'run.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({'finished':True,'exit':child.returncode,'log_dir':str(dest)}),flush=True)
    return child.returncode


if __name__=='__main__':raise SystemExit(main())
