"""Select one Planner process while preserving existing Predictor/Monitor processes."""
import argparse
from dataclasses import asdict, dataclass
import datetime
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import uuid

from . import ROOT

BASELINE = Path('/home/odl/codebase/pingpang_doubles_v9_runtime')
BASELINE_PYTHON = Path('/home/odl/miniconda3/envs/tabletennis-active-v1/bin/python')
COMMAND_TOPICS = ('/doubles/table_left/command', '/doubles/table_right/command')
LIVE_BLOCKERS = (
    'real_36D_85D_input_contract_not_accepted',
    'onboard_shadow_and_movement_feedback_not_accepted',
    'full_HIT_RETURN_relay_transport_not_validated_on_robots',
)


@dataclass
class Launch:
    planner: str
    mode: str
    command: list
    cwd: str
    may_publish_control: bool
    blockers: list


def build_launch(planner, mode, run_dir, *, actor=199, input_file=None, subscriber_ip=None,relay_session=None):
    run_dir = Path(run_dir)
    private_python = str(ROOT/'.venv/bin/python')
    if planner not in ('fixed', 'yichao') or mode not in ('replay', 'observe', 'shadow', 'active'):
        raise ValueError('unknown planner or mode')
    if actor not in (190, 199):
        raise ValueError('actor must be 190 or 199')
    if input_file is not None and mode != 'replay':
        raise ValueError('--input is for replay only')
    if relay_session is not None:
        import re
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',relay_session):raise ValueError('safe relay session required')
        if planner!='yichao' or mode!='shadow':raise ValueError('private relay session currently requires Yichao shadow')
        if not subscriber_ip:raise ValueError('relay requires workstation subscriber IP')
        cmd=[private_python,'-B',str(ROOT/'tools/run_runtime_relay.py'),'--session',relay_session,
             '--mode','shadow','--actor',str(actor),'--subscriber-ip',subscriber_ip,
             '--seconds','30','--shots','2','--output',str(run_dir/'relay.jsonl')]
        return Launch(planner,mode,cmd,str(ROOT),True,[])
    if mode == 'replay':
        if planner != 'yichao' or input_file is None:
            raise ValueError('replay requires Yichao and an input file')
        cmd = [str(ROOT/'run_workstation_offline.sh'), '--actor', str(actor),
               '--input', str(Path(input_file).resolve()), '--output', str(run_dir/'replay.jsonl')]
        return Launch(planner, mode, cmd, str(ROOT), False, [])
    if mode == 'observe':
        if not subscriber_ip:
            raise ValueError('observe requires the assigned workstation subscriber IP')
        cmd = [private_python, '-B', str(ROOT/'tools/capture_existing_inputs.py'), '--seconds', '20',
               '--subscriber-ip', subscriber_ip, '--output', str(run_dir/'inputs.jsonl')]
        return Launch(planner, mode, cmd, str(ROOT), False, [])
    if planner == 'yichao' and mode == 'shadow':
        if not subscriber_ip:
            raise ValueError('shadow requires the assigned workstation subscriber IP')
        cmd = [private_python, '-B', str(ROOT/'tools/capture_existing_inputs.py'),
               '--seconds', '20', '--subscriber-ip', subscriber_ip, '--learned-shadow',
               '--actor', str(actor), '--output', str(run_dir/'shadow.jsonl')]
        return Launch(planner, mode, cmd, str(ROOT), False, [])
    if planner == 'yichao':
        # Diagnostic shadow never certifies sensor timing or command feedback.
        # Active stays closed until the real control interface is accepted.
        return Launch(planner, mode, [], str(ROOT), mode == 'active', list(LIVE_BLOCKERS))
    command = [str(BASELINE_PYTHON), '-B', '-u', str(BASELINE/'scripts/run_v9_real_fixed_relay.py')]
    if mode == 'active':
        command.append('--active')
    # The original fixed SHADOW entry also constructs command publishers and
    # sends invalid command envelopes. Treat either mode as a control-topic owner.
    return Launch(planner, mode, command, str(BASELINE), True, [])


def control_conflicts(publishers):
    return {t: sorted(publishers[t]) for t in COMMAND_TOPICS if publishers.get(t)}


def environment(launch, subscriber_ip):
    env = os.environ.copy()
    env.update(PYTHONNOUSERSITE='1', PYTHONDONTWRITEBYTECODE='1', OMP_NUM_THREADS='1',
               OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
    if launch.planner == 'fixed' and launch.mode in ('active', 'shadow'):
        if not subscriber_ip:
            raise ValueError('fixed ROS entry requires --subscriber-ip')
        env.update(ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP=subscriber_ip,
                   PYTHONPATH='/opt/ros/noetic/lib/python3/dist-packages')
        env.pop('ROS_HOSTNAME', None)
    else:
        env['PYTHONPATH'] = str(ROOT/'src')
    return env


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--planner', choices=('fixed', 'yichao'), required=True)
    p.add_argument('--mode', choices=('replay', 'observe', 'shadow', 'active'), required=True)
    p.add_argument('--actor', type=int, choices=(190, 199), default=199)
    p.add_argument('--input', type=Path)
    p.add_argument('--subscriber-ip')
    p.add_argument('--dry-run', action='store_true', help='print selection/capabilities; start nothing')
    p.add_argument('--relay-session',help='use already-running dedicated V9 shadow receivers in this private session')
    args = p.parse_args()
    run_id = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')+'_'+uuid.uuid4().hex[:8]
    run_dir = ROOT/'output/runs'/run_id
    try:
        launch = build_launch(args.planner, args.mode, run_dir, actor=args.actor,
                              input_file=args.input, subscriber_ip=args.subscriber_ip,relay_session=args.relay_session)
        env = environment(launch, args.subscriber_ip)
    except ValueError as exc:
        p.error(str(exc))
    result = {**asdict(launch), 'run_id': run_id, 'run_dir': str(run_dir),
              'ready_to_start': not launch.blockers,
              'predictor_started': False, 'monitor_started': False,
              'robot_controller_started': False,
              'diagnostic_shadow': args.planner == 'yichao' and args.mode == 'shadow' and args.relay_session is None,
              'private_relay_shadow':args.relay_session is not None,
              'real_input_accepted': False}
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    if launch.blockers:
        return 2
    if args.dry_run:
        return 0
    run_dir.mkdir(parents=True, exist_ok=False)
    lock_stream = None
    if launch.may_publish_control:
        from .observer import configure_subscriber_ip, graph
        configure_subscriber_ip(args.subscriber_ip)
        # Serialize this selector's own instances. ROS graph ownership is also
        # checked, but this is not a distributed lease against unrelated programs.
        lock_stream = (ROOT/'output/planner_control.lock').open('a')
        try:
            fcntl.flock(lock_stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('another selector owns the Planner control lock')
        publishers, _ = graph()
        conflicts = control_conflicts(publishers)
        if conflicts:
            result['control_conflicts'] = conflicts
            (run_dir/'run.json').write_text(json.dumps(result, indent=2)+'\n')
            print('Refusing concurrent command publishers: '+json.dumps(conflicts), file=sys.stderr)
            return 2
    entry = next((Path(x) for x in launch.command if x.endswith('.py')), Path(launch.command[0]))
    result['entry_sha256'] = hashlib.sha256(entry.read_bytes()).hexdigest()
    result['release_manifest_sha256'] = hashlib.sha256((ROOT/'config/assets.lock.json').read_bytes()).hexdigest()
    metadata_path = run_dir/'run.json'
    metadata_path.write_text(json.dumps(result, indent=2)+'\n')
    with (run_dir/'stdout.log').open('x') as stdout, (run_dir/'stderr.log').open('x') as stderr:
        child = subprocess.Popen(launch.command, cwd=launch.cwd, env=env, stdout=stdout,
                                 stderr=stderr, start_new_session=True)
        result['owned_pid'] = child.pid
        metadata_path.write_text(json.dumps(result, indent=2)+'\n')
        def relay_signal(signum, _frame):
            if child.poll() is None:
                os.kill(child.pid, signum)  # only the exact process started above
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            signal.signal(sig, relay_signal)
        code = child.wait()
    result['exit_code'] = code
    metadata_path.write_text(json.dumps(result, indent=2)+'\n')
    if lock_stream is not None:
        lock_stream.close()
    print(json.dumps({'run_dir': str(run_dir), 'exit_code': code}), flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
