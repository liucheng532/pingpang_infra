"""Recover pre-Planner startup while retaining both exact native drivers."""
import fcntl
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from yichao_v3_v9 import active_two_terminal as active
from yichao_v3_v9.two_terminal import Children, process_start, ssh, wait_until
from yichao_v3_v9.observer import graph


def check_recovery():
    config = active.config()
    data = active.session(require_alive=False)
    try:
        current = process_start(data['pid'])
    except FileNotFoundError:
        current = None
    if current == data['process_start']:
        raise RuntimeError('workstation session still running')
    dest = Path(data['directory'])
    if any(dest.rglob('relay.jsonl')) or any(dest.rglob('policy_ready.json')):
        raise RuntimeError('recovery only supports startup before Planner or both-policy readiness')
    owner = subprocess.run(['tmux', 'show-options', '-qv', '-t', active.TMUX,
                            '@yichao_session'], capture_output=True, text=True, check=True)
    if owner.stdout.strip() != data['session']:
        raise RuntimeError('native session ownership mismatch')
    panes = subprocess.run(['tmux', 'list-windows', '-t', active.TMUX,
                            '-F', '#{window_name} #{pane_dead}'],
                           capture_output=True, text=True, check=True)
    actual = dict(line.split() for line in panes.stdout.splitlines())
    expected = {kind+'-'+slot: '0' if kind == 'g1' else '1'
                for kind in ('g1', 'policy') for slot in active.ROLES}
    if actual != expected:
        raise RuntimeError('requires exactly two live drivers and two exited policies')
    pubs, _ = graph()
    if any(pubs.get('/doubles/'+role+'/command') for role in active.ROLES.values()):
        raise RuntimeError('another command publisher exists')
    drivers = {}
    for slot in active.ROLES:
        code = ('import sys,json,pathlib,runpy;root=pathlib.Path('+repr(active.ONBOARD)+');'
                'sys.path.insert(0,str(root/"src"));'
                'm=runpy.run_path(str(root/"tools/run_onboard_active.py"));'
                'd=json.loads((root/"output"/'+repr(data['session'])+'/'
                +repr('control_'+slot+'.json')+').read_text());'
                'assert d["session"]=='+repr(data['session'])+';'
                'assert d["robot"]=='+repr(slot)+';'
                'm["verify_control_manifest"](root,'+repr(slot)+');'
                'assert m["verify_control_process"](root,'+repr(slot)+',d["pid"])==d["process_start"];'
                'print(json.dumps(d))')
        drivers[slot] = json.loads(active.remote(slot, [active.ONBOARD+'/.venv/bin/python', '-B', '-c', code]))
    return config, data, drivers


def main():
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11311', PYTHONNOUSERSITE='1',
                      OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    lock = (ROOT/'output/two_terminal_stack.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    config, data, drivers = check_recovery()
    dest = Path(data['directory'])/('recovery_'+time.strftime('%Y%m%d_%H%M%S'))
    dest.mkdir(exist_ok=False)
    (dest/'previous_workstation_session.json').write_text(json.dumps(data, indent=2)+'\n')
    (dest/'retained_drivers.json').write_text(json.dumps(drivers, indent=2)+'\n')
    for slot in active.ROLES:
        (dest/('previous_policy_'+slot+'.log')).write_text(active.capture('policy-'+slot))
    children = Children(dest)
    result = {'native_drivers_retained': True, 'native_drivers_started': 0,
              'native_drivers_stopped': 0, 'session': data['session'], 'directory': str(dest)}
    try:
        input_run = data['session']+'_r'+str(int(time.time()))
        children.start('predictor', [sys.executable, '-B', str(ROOT/'tools/run_predictor_snapshot.py'),
            '--session', input_run, '--until-interrupted'])
        required = ['/doubles/ball_prediction']+['/doubles/'+r+'/torso_pose_origin' for r in active.ROLES.values()]
        wait_until(lambda: all(graph()[0].get(t) for t in required), 20, 'Predictor inputs', children)
        pubs, _ = graph()
        owners = [pubs[t] for t in required]
        if any(len(o) != 1 for o in owners) or len({o[0] for o in owners}) != 1:
            raise RuntimeError('one common input publisher required')
        children.start('poses', [sys.executable, '-B', str(ROOT/'tools/relay_diagnostic_poses.py'),
            '--run-id', data['session'], '--input-mode', 'doubles-torso', '--publisher', owners[0][0],
            '--subscriber-ip', config['workstation']['ros_ip'], '--until-interrupted',
            '--output', str(dest/'poses.jsonl')])
        prefix = '/yichao_v3_v9/'+data['session']
        wait_until(lambda: all(graph()[0].get(prefix+'/'+r+'/torso_pose_origin')
                              for r in active.ROLES.values()), 15, 'private poses', children)
        data.update(pid=os.getpid(), process_start=process_start(os.getpid()),
                    created_monotonic=time.monotonic(), log_directory=str(dest))
        (dest/'session.json').write_text(json.dumps(data, indent=2)+'\n')
        (ROOT/'output/current_active.json').write_text(json.dumps(data, indent=2)+'\n')
        for slot, role in active.ROLES.items():
            argv = [active.ONBOARD+'/.venv/bin/python', '-B', '-u',
                active.ONBOARD+'/tools/run_onboard_active.py', '--robot', slot,
                '--session', data['session'], '--g1-pid', str(drivers[slot]['pid']),
                '--torso-topic', prefix+'/'+role+'/torso_pose_origin', '--record-dir',
                active.ONBOARD+'/output/'+data['session']+'/'+dest.name+'/policy_'+slot]
            login = ssh(slot, 'exec '+shlex.join(argv))
            command = shlex.join([login[0], '-tt', *login[1:]])
            # No -k: tmux refuses to replace a live process.
            subprocess.run(['tmux', 'respawn-pane', '-t', active.TMUX+':policy-'+slot, command], check=True)
        for slot in active.ROLES:
            active.wait_window('policy-'+slot, 'Press R2 to move to the V9 default pose')
        print('Inputs and both Python policies restored. Native drivers unchanged. Each robot is waiting for first R2.', flush=True)
        waiter = children.start('readiness', [sys.executable, '-B', str(ROOT/'tools/wait_active_ready.py'),
            '--session', data['session'], '--output', str(dest/'policy_ready.json')])
        while waiter.poll() is None:
            if any(p.poll() is not None for label, p, _ in children.items if label != 'readiness'):
                raise RuntimeError('input component failed during recovered startup')
            time.sleep(.2)
        if waiter.returncode:
            raise RuntimeError('recovered policy readiness failed')
        client = children.start('relay', [sys.executable, '-B', str(ROOT/'tools/run_runtime_relay.py'),
            '--session', data['session'], '--mode', 'active', '--experimental-active', '--actor', '199',
            '--seconds', '0', '--shots', '0',
            '--subscriber-ip', config['workstation']['ros_ip'], '--output', str(dest/'relay.jsonl')])
        print('Both robots POLICY; Yichao Planner started. You may feed balls.', flush=True)
        while client.poll() is None:
            if any(p.poll() is not None for label, p, _ in children.items if label in ('predictor', 'poses')):
                raise RuntimeError('input component failed after recovered startup')
            time.sleep(.2)
        result['relay_exit'] = client.returncode
        print('Planner finished. Inputs and native motor programs remain running.', flush=True)
        while True:
            time.sleep(.5)
    finally:
        result['workstation_processes'] = children.close()
        (dest/'recovery_result.json').write_text(json.dumps(result, indent=2)+'\n')


if __name__ == '__main__':
    main()
