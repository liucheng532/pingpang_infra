"""Prepare exact host commands and a session manifest; start no processes."""
import argparse
import json
from pathlib import Path
import re
import shlex

ROOT=Path(__file__).resolve().parents[1]


def commands(config, session, publisher):
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',session):
        raise ValueError('safe session name required')
    if not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_/]*',publisher):
        raise ValueError('explicit existing Predictor publisher required')
    if config['execution_mode']!='shadow' or config['active_accepted'] is not False:
        raise ValueError('this preparation entry supports unaccepted shadow only')
    roles={s:c['ros_role'] for s,c in config['robots'].items()}
    if roles!={'198':'table_left','66':'table_right'}:
        raise ValueError('198 must be table_left and 66 table_right')
    ws=config['workstation']['root'];robot=config['onboard_root']
    base=ws+'/output/sessions/'+session
    prefix='/yichao_v3_v9/'+session
    result=[]
    def add(host,label,argv):
        result.append({'host':host,'label':label,'argv':argv,'shell':shlex.join(argv)})
    env=['env','PYTHONNOUSERSITE=1','PYTHONDONTWRITEBYTECODE=1','OMP_NUM_THREADS=1',
         'OPENBLAS_NUM_THREADS=1','MKL_NUM_THREADS=1','nice','-n','19','ionice','-c','3']
    add(config['workstation']['ssh'],'poses',env+[ws+'/.venv/bin/python','-B',ws+'/tools/relay_diagnostic_poses.py',
        '--run-id',session,'--input-mode',config['pose_input_mode'],'--publisher',publisher,
        '--subscriber-ip',config['workstation']['ros_ip'],'--seconds',str(config['pose_duration_s']),
        '--output',base+'/poses.jsonl'])
    for slot,c in config['robots'].items():
        add(c['ssh'],'telemetry_'+slot,env+[robot+'/passive_v9_telemetry','--read-only-45s',c['private_telemetry_lcm']])
        add(c['ssh'],'policy_shadow_'+slot,env+[robot+'/.venv/bin/python','-B',robot+'/tools/run_onboard_relay.py',
            '--session',session,'--robot',slot,'--mode','shadow','--deploy-root',robot+'/vendor/deploy',
            '--torso-topic',prefix+'/'+c['ros_role']+'/torso_pose_origin',
            '--record-dir',robot+'/output/'+session,'--seconds',str(config['onboard_duration_s'])])
    add(config['workstation']['ssh'],'relay_shadow',env+[ws+'/.venv/bin/python','-B',ws+'/tools/run_runtime_relay.py',
        '--session',session,'--mode','shadow','--actor',str(config['actor']),
        '--seconds',str(config['client_duration_s']),'--shots',str(config['requested_shots']),
        '--subscriber-ip',config['workstation']['ros_ip'],'--output',base+'/relay.jsonl'])
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',required=True)
    p.add_argument('--publisher',required=True,help='current single Predictor ROS node name from the ROS graph')
    p.add_argument('--output-dir',type=Path,help='default: this release output/sessions/<session>')
    args=p.parse_args()
    config=json.loads((ROOT/'config/deployment.json').read_text())
    try:items=commands(config,args.session,args.publisher)
    except ValueError as exc:p.error(str(exc))
    dest=args.output_dir or ROOT/'output/sessions'/args.session
    dest.mkdir(parents=True,exist_ok=False)
    result={'session':args.session,'config':config,'commands':items,'processes_started':False,
        'active_available':False,'physical_execution_accepted':False,
        'requirements':['explicit device handoff and no other control process',
            'existing Predictor publishes both transformed torso topics and ball prediction',
            'launch poses, then each robot telemetry+shadow, then relay after both private states exist',
            'processes must overlap within their bounded durations; use a fresh session on expiry']}
    (dest/'session.json').write_text(json.dumps(result,indent=2)+'\n')
    (dest/'commands.txt').write_text('\n\n'.join(x['host']+' / '+x['label']+'\n'+x['shell'] for x in items)+'\n')
    print(json.dumps({'prepared':str(dest),'processes_started':False,'active_available':False}))


if __name__=='__main__':main()
