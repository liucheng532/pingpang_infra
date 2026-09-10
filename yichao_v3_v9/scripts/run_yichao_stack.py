#!/usr/bin/env python3
"""Start the Yichao planner, optional monitor, and explicitly requested original Predictor.

Predictor and both robot controllers keep their existing launchers and files.
By default Predictor is not started; --with-predictor opts into supervising it.
Without --start this prints a launch plan and performs no ROS or device I/O.
"""
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import uuid
import argparse

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import yichao_v3_v9
spec = importlib.util.spec_from_file_location('fixed_stack_supervisor', ROOT/'baseline/scripts/run_doubles_stack.py')
fixed = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = fixed
spec.loader.exec_module(fixed)


def components(session, active=False, monitor=True, predictor=False, monitor_port=8089):
    python = sys.executable
    items = [fixed.Component('planner', ROOT,
        (python, '-B', '-u', str(ROOT/'scripts/run_yichao_planner.py'),
         '--session-dir', str(session/'planner'), *(('--active',) if active else ())))]
    if predictor:
        items.insert(0, fixed.Component('predictor', fixed.PREDICTOR_ROOT,
            (str(fixed.PREDICTOR_PYTHON), '-B', '-u', str(fixed.PREDICTOR_ROOT/'scripts/run_predictor.py'))))
    if monitor:
        items.append(fixed.Component('monitor', ROOT,
            (python, '-B', '-u', '-m', 'yichao_v3_v9.monitor.server', '--port', str(monitor_port),
             '--log-dir', str(session/'monitor'), *(('--enable-config',) if active else ()))))
    return items


def environment():
    env = os.environ.copy()
    env.setdefault('ROS_MASTER_URI', 'http://192.168.123.165:11311')
    env.setdefault('ROS_IP', '192.168.123.165')
    env.pop('ROS_HOSTNAME', None)
    env['PINGPANG_CALIBRATION_CONFIG'] = str(fixed.CALIBRATION_CONFIG)
    env['PYTHONPATH'] = str(ROOT/'src')+':/opt/ros/noetic/lib/python3/dist-packages:'+env.get('PYTHONPATH','')
    return env


def existing_planners(process_table=None, predictor=False):
    table = process_table if process_table is not None else subprocess.check_output(['ps','-eo','pid=,args='], text=True)
    markers = ('scripts/run_v9_real_fixed_relay.py', 'scripts/run_yichao_planner.py', 'tools/run_active_control.py')
    if predictor:
        markers += ('TableTennis.py', 'scripts/run_predictor.py')
    return [line.strip() for line in table.splitlines()
            if len(line.strip().split(maxsplit=1)) == 2 and
            int(line.strip().split(maxsplit=1)[0]) != os.getpid() and any(m in line for m in markers)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', action='store_true', help='Start workstation processes; otherwise print the plan')
    parser.add_argument('--active', action='store_true', help='Publish robot commands; Fixed planner must be stopped first')
    parser.add_argument('--without-monitor', action='store_true')
    parser.add_argument('--with-predictor', action='store_true', help='Also supervise the original Predictor, without changing its files')
    parser.add_argument('--monitor-port', type=int, default=8089)
    args = parser.parse_args()
    session = ROOT/'output/sessions'/('fixed_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8])
    items = components(session, args.active, not args.without_monitor, args.with_predictor, args.monitor_port)
    if not args.start:
        print(json.dumps({'start': False, 'active': args.active, 'session': str(session),
                          'components': [{'name': c.name, 'cwd': str(c.cwd), 'argv': c.command} for c in items]}, indent=2))
        return 0
    blockers = existing_planners(predictor=args.with_predictor)
    if blockers:
        raise SystemExit('Existing planner processes (not stopped automatically):\n'+'\n'.join(blockers))
    if not args.without_monitor:
        with socket.socket() as probe:
            try:
                probe.bind(('0.0.0.0', args.monitor_port))
            except OSError as error:
                raise SystemExit(f'Monitor port {args.monitor_port} unavailable: {error}')
    if args.with_predictor:
        for path in (fixed.PREDICTOR_PYTHON, fixed.PREDICTOR_ROOT/'scripts/run_predictor.py', fixed.CALIBRATION_CONFIG):
            if not path.is_file():
                raise SystemExit(f'Missing Predictor dependency: {path}')
    session.mkdir(parents=True)
    metadata = {'session': session.name, 'owner_pid': os.getpid(),
                'owner_starttime': Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()[19],
                'active': args.active, 'components': []}
    env = environment()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP): signal.signal(sig, lambda *_: stop.set())
    started, logs = [], []
    failed = None
    try:
        for component in items:
            log = (session/(component.name+'.log')).open('xb');logs.append(log)
            process = subprocess.Popen(fixed.supervised_command(component.command), cwd=component.cwd, env=env,
                                       start_new_session=True, stdout=log, stderr=subprocess.STDOUT)
            started.append((component, process))
            metadata['components'].append({'name':component.name, 'pid':process.pid, 'argv':component.command,
                'starttime':Path(f'/proc/{process.pid}/stat').read_text().rsplit(')',1)[1].split()[19]})
        (session/'run.json').write_text(json.dumps(metadata,indent=2)+'\n')
        print('Yichao session: '+str(session), flush=True)
        while not stop.wait(.1):
            for component,process in started:
                if process.poll() is not None:
                    failed = component.name;stop.set();break
    finally:
        fixed.stop_components(started)
        for log in logs: log.close()
        metadata['exit_codes'] = {c.name:p.returncode for c,p in started}
        metadata['failed_component'] = failed
        (session/'run.json').write_text(json.dumps(metadata,indent=2)+'\n')
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
