"""Read-only V9 evidence capture. Credentials are read in memory from the authorized AGENTS file."""
import hashlib, json, re, shlex
from pathlib import Path
import paramiko
ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT.parent / 'doc/yichao_v3_v9'
DOC.mkdir(exist_ok=True)
credential_text = (ROOT.parent / 'AGENTS.md').read_text()
report = {}
for host in ('66', '198'):
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())  # memory only; no known_hosts write
    password = re.search(r'\| Unitree '+host+r' \|.*?\| `([^`]+)` \|', credential_text).group(1)
    try:
        client.connect('172.16.4.'+host, username='unitree', password=password, timeout=8, auth_timeout=8, banner_timeout=8)
        base = '/home/unitree/haoran/doubles-v9-real-runtime'
        command = 'git -C '+shlex.quote(base)+' rev-parse HEAD; git -C '+shlex.quote(base)+' status --porcelain; find '+shlex.quote(base+'/policy')+' -maxdepth 3 -type f; find '+shlex.quote(base+'/data')+' -maxdepth 2 -name dataindex.csv'
        _,out,err = client.exec_command(command, timeout=20)
        report[host] = {'evidence':'measured_read_only', 'inventory':out.read().decode(), 'stderr':err.read().decode()}
        sftp = client.open_sftp()
        dest = ROOT/'assets'/('robot_'+host)
        dest.mkdir(exist_ok=True)
        policy = base+'/policy/v9_model19000'
        files = []
        for item in sftp.listdir_attr(policy):
            if item.filename.endswith(('.json','.onnx')):
                target = dest/item.filename
                sftp.get(policy+'/'+item.filename, str(target))
                files.append({'path':policy+'/'+item.filename,'sha256':hashlib.sha256(target.read_bytes()).hexdigest(),'size':target.stat().st_size})
        report[host]['files'] = files
        for relative in ('g1_gym_deploy/utils/doubles_reference_scheduler.py','g1_gym_deploy/utils/mirrored_reference_scheduler.py','g1_gym_deploy/utils/left_right_mirror.py','g1_gym_deploy/utils/planner_ros_bridge.py','g1_gym_deploy/scripts/start_v9_policy.sh'):
            data = sftp.open(base+'/'+relative).read()
            target = dest/relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(data)
            report[host]['files'].append({'path':relative,'sha256':hashlib.sha256(data).hexdigest()})
        sftp.close()
    except Exception as error:
        report.setdefault(host,{})['error'] = type(error).__name__+': '+str(error)
    finally:
        client.close()
(DOC/'remote_assets.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
