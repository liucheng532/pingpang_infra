"""Operator-controlled active trial, using persistent private motor sessions.

Workstation failures never automatically kill g1/policy or claim physical stop.
The operator must physically disarm before requesting software cleanup.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import time
import uuid

from . import ROOT
from .observer import graph
from .two_terminal import Children,process_start,preflight_robot,ssh,wait_until

TMUX='yichao_v9_active'
ROLES={'198':'table_left','66':'table_right'}
ONBOARD='/home/unitree/haoran/yichao_v3_v9_onboard_20260907T162746Z'


def config():
    c=json.loads((ROOT/'config/deployment.json').read_text())
    if str(ROOT)!=c['workstation']['root']:raise RuntimeError('active commands must run on the Planner workstation')
    release=json.loads((ROOT/'config/active_release.json').read_text())
    if (release.get('mode')!='experimental_operator_controlled' or release.get('actor')!=199
            or release.get('low_level')!='V9 i19000' or release.get('physical_accepted') is not False):
        raise RuntimeError('active release identity mismatch')
    return c


def session(require_alive=True):
    d=json.loads((ROOT/'output/current_active.json').read_text())
    dest=Path(d['directory']).resolve()
    if (d['mode']!='active' or d['root']!=str(ROOT) or d['tmux']!=TMUX
            or dest.parent!=(ROOT/'output/sessions').resolve() or dest.name!=d['session']):
        raise ValueError('active session identity/path mismatch')
    if require_alive:
        if process_start(d['pid'])!=d['process_start']:raise ValueError('active workstation PID changed')
        if not 0<=time.monotonic()-d['created_monotonic']<600:raise ValueError('active startup session expired')
    return d


def remote(slot,argv,timeout=30):
    r=subprocess.run(ssh(slot,shlex.join(argv)),capture_output=True,text=True,timeout=timeout)
    if r.returncode:raise RuntimeError('robot '+slot+': '+r.stderr.strip()+r.stdout[-1000:])
    return r.stdout


def preflight_all(c):
    if subprocess.run(['tmux','has-session','-t',TMUX],capture_output=True).returncode==0:
        raise RuntimeError('active tmux session already exists; preserve it')
    pubs,_=graph()
    if any(pubs.get('/doubles/'+r+'/command') for r in ROLES.values()):
        raise RuntimeError('baseline command publisher exists; no takeover')
    # Complete both read-only checks before either native driver starts.
    def check(slot):
        preflight_robot(slot)
        remote(slot,[ONBOARD+'/.venv/bin/python','-B',ONBOARD+'/tools/run_active_control.py',
                     '--robot',slot,'--session','active_preflight','--check'])
        remote(slot,[ONBOARD+'/.venv/bin/python','-B',ONBOARD+'/tools/run_onboard_active.py',
                     '--robot',slot,'--session','active_preflight','--check'],60)
    with ThreadPoolExecutor(max_workers=2) as pool:list(pool.map(check,ROLES))


def active_stack():
    c=config()
    # Early read-only check avoids preparing a second input chain for occupied robots.
    preflight_all(c)
    dest=ROOT/'output/sessions'/('active_'+time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
    dest.mkdir(parents=True)
    children=Children(dest);report={'mode':'active','experimental_active':True,'physical_stop_on_exit':False}
    data={'session':dest.name,'directory':str(dest),'root':str(ROOT),'mode':'active','tmux':TMUX,
          'pid':os.getpid(),'process_start':process_start(os.getpid()),'created_monotonic':time.monotonic()}
    try:
        pubs,_=graph()
        required=['/doubles/ball_prediction']+['/doubles/'+r+'/torso_pose_origin' for r in ROLES.values()]
        if not any(pubs.get(t) for t in required):
            if any(pubs.get('/doubles/'+r+'/state') for r in ROLES.values()):
                raise RuntimeError('existing robot state publisher; do not introduce new input')
            children.start('predictor',[sys.executable,'-B',str(ROOT/'tools/run_predictor_snapshot.py'),
                '--session',dest.name,'--until-interrupted'])
            wait_until(lambda:all(graph()[0].get(t) for t in required),20,'Predictor inputs',children)
        pubs,_=graph();owners=[pubs.get(t,[]) for t in required]
        if any(len(v)!=1 for v in owners) or len({v[0] for v in owners})!=1:
            raise RuntimeError('one common ball/torso Predictor publisher required')
        children.start('poses',[sys.executable,'-B',str(ROOT/'tools/relay_diagnostic_poses.py'),
            '--run-id',dest.name,'--input-mode','doubles-torso','--publisher',owners[0][0],
            '--subscriber-ip',c['workstation']['ros_ip'],'--until-interrupted','--output',str(dest/'poses.jsonl')])
        prefix='/yichao_v3_v9/'+dest.name
        wait_until(lambda:all(graph()[0].get(prefix+'/'+r+'/torso_pose_origin') for r in ROLES.values()),
                   15,'private pose publishers',children)
        (dest/'session.json').write_text(json.dumps(data,indent=2)+'\n')
        (ROOT/'output/current_active.json').write_text(json.dumps(data,indent=2)+'\n')
        print('Terminal 1 ready: run bash scripts/start_yichao_dual_robots.sh start active',flush=True)
        waiter=children.start('readiness',[sys.executable,'-B',str(ROOT/'tools/wait_active_ready.py'),
            '--session',dest.name,'--output',str(dest/'policy_ready.json')])
        while waiter.poll() is None:
            if any(p.poll() is not None for label,p,_ in children.items if label!='readiness'):
                raise RuntimeError('input component exited while waiting for R2; motor session is retained')
            time.sleep(.2)
        if waiter.returncode!=0:raise RuntimeError('both-policy readiness failed; inspect readiness.log')
        print('Both robots are in POLICY. Starting Yichao actor 199 + filter. You may now feed balls.',flush=True)
        client=children.start('relay',[sys.executable,'-B',str(ROOT/'tools/run_runtime_relay.py'),
            '--session',dest.name,'--mode','active','--experimental-active','--actor','199','--seconds','0',
            '--shots','0','--subscriber-ip',c['workstation']['ros_ip'],
            '--output',str(dest/'relay.jsonl')])
        while client.poll() is None:
            if any(p.poll() is not None for label,p,_ in children.items if label in ('predictor','poses')):
                raise RuntimeError('active input component exited; motor programs remain running')
            time.sleep(.2)
        report['relay_exit']=client.returncode
        print('Yichao relay ended (exit '+str(client.returncode)+'). Motor programs remain running; '
              'physically disarm before software cleanup. Inputs stay alive until Ctrl-C.',flush=True)
        (dest/'active_result.json').write_text(json.dumps(report,indent=2)+'\n')
        # Do not withdraw pose input or kill motor programs when Planner finishes.
        while True:time.sleep(.5)
    finally:
        report['workstation_processes']=children.close()
        (dest/'active_result.json').write_text(json.dumps(report,indent=2)+'\n')
        print('Workstation components ended. This did not stop/disarm robot motors.',flush=True)


def window(name,slot,argv,*,first=False):
    login=ssh(slot,'exec '+shlex.join(argv))
    command=shlex.join([login[0],'-tt',*login[1:]])
    if first:
        subprocess.run(['tmux','new-session','-d','-s',TMUX,'-n',name,command],check=True)
    else:subprocess.run(['tmux','new-window','-d','-t',TMUX,'-n',name,command],check=True)
    subprocess.run(['tmux','set-window-option','-t',TMUX+':'+name,'remain-on-exit','on'],check=True,capture_output=True)


def capture(name):
    r=subprocess.run(['tmux','capture-pane','-p','-t',TMUX+':'+name,'-S','-200'],capture_output=True,text=True)
    return r.stdout


def wait_window(name,marker,timeout=90):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        r=subprocess.run(['tmux','display-message','-p','-t',TMUX+':'+name,'#{pane_dead}'],capture_output=True,text=True)
        if r.returncode or r.stdout.strip()=='1':raise RuntimeError(name+' exited: '+capture(name)[-2500:])
        if marker in capture(name):return
        time.sleep(.2)
    raise RuntimeError('timeout waiting for '+name+': '+marker)


def active_robots(right_startup='outward-hold'):
    if right_startup not in ('outward-hold','home-hold'):
        raise ValueError('invalid_right_startup')
    c=config();d=session();dest=Path(d['directory']);preflight_all(c)
    (dest/'robots_requested.json').write_text(json.dumps({'pid':os.getpid(),'mode':'active',
                                                       'right_startup':right_startup})+'\n')
    jobs=[]
    try:
        for i,slot in enumerate(ROLES):
            name='g1-'+slot
            window(name,slot,[ONBOARD+'/.venv/bin/python','-B',ONBOARD+'/tools/run_active_control.py',
                '--robot',slot,'--session',d['session'],'--start'],first=i==0)
            jobs.append(name)
            if i==0:
                subprocess.run(['tmux','set-option','-t',TMUX,'@yichao_session',d['session']],check=True,capture_output=True)
        for slot in ROLES:wait_window('g1-'+slot,'G1 type:',30)
        for slot,role in ROLES.items():
            path=ONBOARD+'/output/'+d['session']+'/control_'+slot+'.json'
            code=('import json,pathlib;d=json.loads(pathlib.Path('+repr(path)+').read_text());'
                  'p=pathlib.Path("/proc",str(d["pid"]));'
                  'assert p.joinpath("stat").read_text().rsplit(")",1)[1].split()[19]==d["process_start"];'
                  'assert p.joinpath("cmdline").read_bytes().split(bytes([0]))[0].decode()==d["binary"];'
                  'print(json.dumps(d))')
            info=json.loads(remote(slot,['python3','-B','-c',code]))
            name='policy-'+slot
            startup_args=['--right-startup','home-hold'] if slot=='66' and right_startup=='home-hold' else []
            window(name,slot,[ONBOARD+'/.venv/bin/python','-B','-u',ONBOARD+'/tools/run_onboard_active.py',
                '--robot',slot,'--session',d['session'],'--g1-pid',str(info['pid']),
                '--torso-topic','/yichao_v3_v9/'+d['session']+'/'+role+'/torso_pose_origin',
                '--record-dir',ONBOARD+'/output/'+d['session']+'/policy_'+slot]+startup_args)
            jobs.append(name)
        for slot in ROLES:wait_window('policy-'+slot,'Press R2 to move to the V9 default pose')
        result={'startup_complete':True,'jobs':jobs,'mode':'active','motor_programs_persistent':True,
                'right_startup':right_startup,'policy_inference_waits_for_ball':False}
        (dest/'robots_result.json').write_text(json.dumps(result,indent=2)+'\n')
        print('Both V9 policies are waiting for R2. On each robot: first R2 -> default; '
              'wait for calibration, then second R2 -> policy. Terminal 1 waits for both.',flush=True)
        print('Inspect: tmux attach -t '+TMUX+' (Ctrl-B, D detaches). R2 during policy recalibrates; '
              'it is not a torque-off emergency stop.',flush=True)
    except BaseException as e:
        (dest/'startup_failure.json').write_text(json.dumps({'error':str(e),'started_windows':jobs,
            'motor_programs_retained':True,'physical_stop_on_exit':False},indent=2)+'\n')
        for name in jobs:(dest/(name+'_startup.log')).write_text(capture(name))
        raise


def status():
    config()
    r=subprocess.run(['tmux','list-windows','-t',TMUX,'-F','#{window_name} dead=#{pane_dead} pid=#{pane_pid}'],capture_output=True,text=True)
    print(r.stdout or r.stderr,end='')
    return r.returncode


def stop(motors_disabled):
    if not motors_disabled:raise ValueError('physically disarm first, then use stop active --motors-disabled; software stop is not emergency stop')
    config();d=session(require_alive=False)
    r=subprocess.run(['tmux','show-options','-qv','-t',TMUX,'@yichao_session'],capture_output=True,text=True)
    if r.returncode or r.stdout.strip()!=d['session']:raise RuntimeError('tmux session ownership mismatch; no process stopped')
    r=subprocess.run(['tmux','list-windows','-t',TMUX,'-F','#{window_name}'],capture_output=True,text=True,check=True)
    windows=set(r.stdout.splitlines())
    expected={kind+'-'+slot for kind in ('policy','g1') for slot in ROLES}
    if not windows or not windows<=expected:
        raise RuntimeError('unexpected windows in owned session; no process stopped')
    for kind in ('policy','g1'):
        for slot in ROLES:
            name=kind+'-'+slot
            if name not in windows:continue
            (Path(d['directory'])/(name+'_stop.log')).write_text(capture(name))
            subprocess.run(['tmux','send-keys','-t',TMUX+':'+name,'C-c'],check=True)
        time.sleep(2)
    subprocess.run(['tmux','kill-session','-t',TMUX],check=True)
    print('Owned software session removed after operator-confirmed physical disarm. Check robot PIDs before any restart.')


def run(component,action,*,motors_disabled=False,right_startup='outward-hold'):
    if action=='check':preflight_all(config());print('Both active release checks passed. No motor program started.');return 0
    if action=='status':return status()
    if action=='stop':stop(motors_disabled);return 0
    if component=='stack':active_stack()
    else:active_robots(right_startup=right_startup)
    return 0
