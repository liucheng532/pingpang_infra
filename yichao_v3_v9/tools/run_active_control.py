"""Launch the frozen private V9 driver only after explicit --start.

This driver can move to zero posture before policy R2, and repeats its last
command if policy exits. Software cleanup is not a physical emergency stop.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def validate(root,robot):
    m=json.loads((root/'config/active_control.json').read_text())
    if m.get('schema')!='yichao-active-control-assets-v1' or m.get('robot')!=robot:
        raise ValueError('active control manifest identity')
    expected='udpm://239.255.78.'+robot+':7867?ttl=0'
    if m.get('lcm_default_url')!=expected:raise ValueError('active LCM identity')
    if m.get('executable')!='control/bin/g1_control' or m.get('library_dir')!='control/lib':
        raise ValueError('private driver path required')
    if m['executable'] not in m['sha256']:raise ValueError('driver hash missing')
    for name,digest in m['sha256'].items():
        p=root/name;p.resolve().relative_to(root.resolve())
        if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise ValueError('active asset changed: '+name)
    return m


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot',choices=('66','198'),required=True)
    p.add_argument('--session',required=True)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--check',action='store_true')
    g.add_argument('--start',action='store_true')
    a=p.parse_args()
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',a.session):p.error('safe session required')
    m=validate(ROOT,a.robot)
    from yichao_v3_v9.onboard_isolation import control_processes
    conflicts=control_processes()
    if conflicts:raise RuntimeError('existing control/policy processes: '+str(conflicts))
    if a.check:
        print(json.dumps({'check_passed':True,'robot':a.robot,'motor_commands_sent':0}));return 0
    dest=ROOT/'output'/a.session;dest.mkdir(parents=True,exist_ok=True)
    stat=Path('/proc/self/stat').read_text().rsplit(')',1)[1].split()
    record={'session':a.session,'robot':a.robot,'pid':os.getpid(),'process_start':stat[19],
            'binary':str(ROOT/m['executable']),'binary_sha256':m['sha256'][m['executable']],
            'lcm_url':m['lcm_default_url'],'physical_stop_on_exit':False}
    with (dest/('control_'+a.robot+'.json')).open('x') as f:json.dump(record,f,indent=2)
    env=os.environ.copy();env['LCM_DEFAULT_URL']=m['lcm_default_url']
    env['LD_LIBRARY_PATH']=str(ROOT/m['library_dir'])+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
    print('Starting private V9 g1_control: zero-pose motion can precede R2; physical stop remains operator controlled.',flush=True)
    os.chdir(ROOT/'control')
    os.execve(record['binary'],[record['binary'],'eth0'],env)


if __name__=='__main__':raise SystemExit(main())
