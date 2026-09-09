"""Coordinate the two workstation terminals for a bounded private V9 session."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import uuid

from . import ROOT
from .observer import graph

SSH_HOSTS = {'66':'192.168.123.164', '198':'192.168.124.164'}
CONTROL_NAMES = ('g1_control','deploy_policy.py','run_onboard_movement.py','run_onboard_relay.py',
                 'run_onboard_active.py','run_active_control.py')


def process_start(pid):
    # comm may contain spaces or parentheses; fields after the final ')' start
    # with state (field 3), and starttime is field 22.
    return Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()[19]


def load_live_session(path):
    data=json.loads(path.read_text())
    if data['mode']!='shadow' or data['root']!=str(ROOT):
        raise ValueError('session mode/root mismatch')
    if process_start(data['pid'])!=data['process_start']:
        raise ValueError('old workstation PID was reused')
    if not 0 <= time.monotonic()-data['created_monotonic'] < 120:
        raise ValueError('session expired; restart terminal 1')
    dest=Path(data['directory']).resolve()
    if dest.parent != (ROOT/'output/sessions').resolve() or dest.name!=data['session']:
        raise ValueError('session path mismatch')
    return data


def ssh(slot, code):
    return ['ssh','-i','/home/odl/.ssh/pingpang','-o','BatchMode=yes',
            '-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=5',
            'unitree@'+SSH_HOSTS[slot],code]


def preflight_robot(slot):
    code=('from pathlib import Path; import json; names='+repr(CONTROL_NAMES)+'; '
          'found=[]\nfor p in Path("/proc").glob("[0-9]*/cmdline"):\n'
          ' try:a=p.read_bytes().decode().split(chr(0))\n'
          ' except (OSError,UnicodeDecodeError):continue\n'
          ' if any(Path(t).name in names for t in a if t):found.append(int(p.parent.name))\n'
          'print(json.dumps(found))\n')
    result=subprocess.run(ssh(slot,shlex.join(['python3','-B','-c',code])),
                          capture_output=True,text=True,timeout=12)
    if result.returncode:raise RuntimeError('robot '+slot+' SSH preflight failed: '+result.stderr.strip())
    pids=json.loads(result.stdout)
    if pids:raise RuntimeError('robot '+slot+' has existing controller/policy PIDs '+str(pids))


class Children:
    def __init__(self,dest):self.dest=dest;self.items=[]
    def start(self,label,argv):
        out=(self.dest/(label+'.log')).open('x')
        try:p=subprocess.Popen(argv,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
        except BaseException:out.close();raise
        self.items.append((label,p,out))
        print(label+' started; log='+str(self.dest/(label+'.log')),flush=True)
        return p
    def close(self):
        for _,p,_ in reversed(self.items):
            if p.poll() is None:p.send_signal(signal.SIGINT)
        deadline=time.monotonic()+12
        for _,p,_ in reversed(self.items):
            try:p.wait(timeout=max(.1,deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                p.terminate()
                try:p.wait(timeout=2)
                except subprocess.TimeoutExpired:p.kill();p.wait()
        report={label:{'owned_pid':p.pid,'exit':p.returncode} for label,p,_ in self.items}
        for _,_,out in self.items:out.close()
        return report


def wait_until(predicate,seconds,label,children=None):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if children and any(p.poll() is not None for _,p,_ in children.items):
            raise RuntimeError('a session component exited while waiting for '+label)
        if predicate():return
        time.sleep(.2)
    raise RuntimeError('timeout waiting for '+label)


def stack():
    from importlib.util import spec_from_file_location,module_from_spec
    cfg=json.loads((ROOT/'config/deployment.json').read_text())
    if str(ROOT)!=cfg['workstation']['root']:raise RuntimeError('run these entries on the Planner workstation')
    dest=ROOT/'output/sessions'/('v9_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    dest.mkdir(parents=True,exist_ok=False)
    children=Children(dest);status={'completed':False,'active_available':False}
    session={'session':dest.name,'root':str(ROOT),'directory':str(dest),'mode':'shadow',
             'pid':os.getpid(),'process_start':process_start(os.getpid()),'created_monotonic':time.monotonic()}
    pointer=ROOT/'output/current_two_terminal.json'
    try:
        pubs,_=graph()
        if any(pubs.get('/doubles/'+r+'/command') for r in ('table_left','table_right')):
            raise RuntimeError('existing baseline command publisher; preserve it and complete handoff first')
        required=['/doubles/ball_prediction']+['/doubles/'+r+'/torso_pose_origin' for r in ('table_left','table_right')]
        if not any(pubs.get(t) for t in required):
            if any(pubs.get('/doubles/'+r+'/state') for r in ('table_left','table_right')):
                raise RuntimeError('existing robot state publishers; do not introduce a new Predictor into their input chain')
            children.start('predictor',[sys.executable,'-B',str(ROOT/'tools/run_predictor_snapshot.py'),
                '--session',dest.name,'--seconds','180'])
            wait_until(lambda:all(graph()[0].get(t) for t in required),15,'Predictor inputs',children)
        pubs,_=graph()
        owners=[pubs.get(t,[]) for t in required]
        if any(len(x)!=1 for x in owners) or len({x[0] for x in owners})!=1:
            raise RuntimeError('one common Predictor publisher required for ball and both torsos')
        spec=spec_from_file_location('prepare_yichao',ROOT/'tools/prepare_shadow_session.py')
        mod=module_from_spec(spec);spec.loader.exec_module(mod)
        cmds=mod.commands(cfg,dest.name,owners[0][0]);session['commands']=cmds
        pointer.write_text(json.dumps(session,indent=2)+'\n')
        (dest/'session.json').write_text(json.dumps(session,indent=2)+'\n')
        print('Terminal 1 ready. In terminal 2 run: bash scripts/start_yichao_dual_robots.sh start shadow',flush=True)
        # Wait before starting the 60s pose relay, so human terminal switching
        # does not consume the onboard test duration.
        wait_until(lambda:(dest/'robots_requested.json').exists(),100,'terminal 2',children)
        requested=json.loads((dest/'robots_requested.json').read_text())
        if process_start(requested['pid'])!=requested['process_start']:raise RuntimeError('terminal 2 exited')
        poses=next(x for x in cmds if x['label']=='poses')
        children.start('poses',poses['argv'])
        prefix='/yichao_v3_v9/'+dest.name
        wait_until(lambda:all(graph()[0].get(prefix+'/'+r+'/state') for r in ('table_left','table_right')),
                   22,'both private V9 states',children)
        relay=next(x for x in cmds if x['label']=='relay_shadow')
        child=children.start('relay',relay['argv']);code=child.wait(timeout=35)
        status.update(relay_exit=code,completed=code==0)
        print('Relay finished: '+('requested shots completed in shadow' if code==0 else 'not completed; inspect relay.log/relay.jsonl'),flush=True)
        # Keep pose input alive until the bounded robot jobs have exited.
        # Cutting it off immediately after the 20s client invalidates the
        # remaining onboard recording period.
        wait_until(lambda:(dest/'robots_result.json').exists(),35,'robot shutdown record')
    finally:
        status['processes']=children.close()
        (dest/'workstation_result.json').write_text(json.dumps(status,indent=2)+'\n')
    return 0 if status['completed'] else 2


def robots():
    session=load_live_session(ROOT/'output/current_two_terminal.json');dest=Path(session['directory'])
    # Both read-only checks complete before starting either robot.
    with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(preflight_robot,('66','198')))
    (dest/'robots_requested.json').write_text(json.dumps({'pid':os.getpid(),'process_start':process_start(os.getpid())})+'\n')
    prefix='/yichao_v3_v9/'+session['session']
    wait_until(lambda:all(graph()[0].get(prefix+'/'+r+'/torso_pose_origin') for r in ('table_left','table_right')),
               12,'private pose relay')
    children=Children(dest);report={}
    try:
        for item in session['commands']:
            label=item['label']
            if not label.startswith(('telemetry_','policy_shadow_')):continue
            slot=label.rsplit('_',1)[1]
            # SSH disconnect cannot leave an unbounded job: both remote
            # programs also enforce their own <=45s lifetime.
            children.start(label,ssh(slot,'exec '+shlex.join(item['argv'])))
        end=time.monotonic()+65
        while any(p.poll() is None for _,p,_ in children.items):
            if any(p.poll() not in (None,0) for _,p,_ in children.items):raise RuntimeError('onboard component failed; inspect session logs')
            if time.monotonic()>end:raise RuntimeError('onboard shutdown deadline')
            time.sleep(.2)
    finally:
        report=children.close()
        (dest/'robots_result.json').write_text(json.dumps(report,indent=2)+'\n')
    return 0 if all(v['exit']==0 for v in report.values()) else 2


def main():
    p=argparse.ArgumentParser(description='Two-terminal Yichao actor199/filter + V9 i19000 launcher')
    p.add_argument('component',choices=('stack','robots'))
    p.add_argument('action',nargs='?',choices=('start','check','status','stop'),default='start')
    p.add_argument('mode',nargs='?',choices=('shadow','active'),default='shadow')
    p.add_argument('--motors-disabled',action='store_true',help='only for active stop after physical disarm')
    p.add_argument('--right-startup',choices=('outward-hold','home-hold'),default=None,
                   help='optional 66 startup reference for robots start active; policy always runs after R2')
    args=p.parse_args()
    if args.right_startup is not None and (args.component,args.action,args.mode)!=('robots','start','active'):
        p.error('--right-startup requires robots start active')
    if args.mode=='shadow' and (args.action!='start' or args.motors_disabled):p.error('shadow entry supports start only')
    (ROOT/'output').mkdir(exist_ok=True)
    lock=(ROOT/'output'/('two_terminal_'+args.component+'.lock')).open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:p.error('this component is already running')
    os.environ.update(PYTHONNOUSERSITE='1',PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',
        OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',ROS_MASTER_URI='http://127.0.0.1:11311')
    def interrupted(sig,frame):raise KeyboardInterrupt
    for sig in (signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    try:
        if args.mode=='active':
            from .active_two_terminal import run
            options={} if args.right_startup is None else {'right_startup':args.right_startup}
            return run(args.component,args.action,motors_disabled=args.motors_disabled,**options)
        return stack() if args.component=='stack' else robots()
    except (OSError,ValueError,RuntimeError,subprocess.SubprocessError) as exc:
        print('Yichao session stopped: '+str(exc),file=sys.stderr);return 2
    except KeyboardInterrupt:return 130

if __name__=='__main__':raise SystemExit(main())
