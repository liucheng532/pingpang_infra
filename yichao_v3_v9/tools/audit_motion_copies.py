"""Stream remote motion data into controlled copies, then compare every named array locally."""
import hashlib, json, re
from pathlib import Path
import paramiko
import numpy as np
root=Path(__file__).resolve().parents[1]
auth=(root.parent/'AGENTS.md').read_text()
report={}
def digest(path):
    arrays={}
    with np.load(path,allow_pickle=False) as data:
        for key in sorted(data.files):
            a=data[key]
            if a.dtype.kind not in 'biufc':
                raise ValueError('non numeric motion field '+key)
            canonical=np.ascontiguousarray(a.astype(a.dtype.newbyteorder('<')))
            arrays[key]={'shape':list(a.shape),'dtype':canonical.dtype.str,'sha256':hashlib.sha256(canonical.tobytes()).hexdigest()}
    return {'file_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'arrays':arrays}
for host in ('66','198'):
    client=paramiko.SSHClient();client.load_system_host_keys();client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    password=re.search(r'\| Unitree '+host+r' \|.*?\| `([^`]+)` \|',auth).group(1)
    try:
        client.connect('172.16.4.'+host,username='unitree',password=password,timeout=8)
        sftp=client.open_sftp()
        records=[]
        for bank in ('0718-move-160-80hz','0302_combined'):
            local=root/'assets'/bank
            remote='/home/unitree/haoran/doubles-v9-real-runtime/data/'+bank
            for source in sorted(local.rglob('*')):
                if not source.is_file() or source.suffix not in ('.npz','.csv'):
                    continue
                relative=source.relative_to(local)
                target=root/'assets/robot_motion_copies'/host/bank/relative
                target.parent.mkdir(parents=True,exist_ok=True)
                if not target.exists():
                    sftp.get(remote+'/'+str(relative),str(target))
                record={'bank':bank,'path':str(relative),'local_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'remote_copy_sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
                if source.suffix=='.npz':
                    a,b=digest(source),digest(target)
                    record['array_summaries']={'local':a['arrays'],'remote_copy':b['arrays']}
                    with np.load(source) as x,np.load(target) as y:
                        record['arrays_equal']=set(x.files)==set(y.files) and all(np.array_equal(x[k],y[k],equal_nan=True) for k in x.files)
                records.append(record)
        report[host]={'evidence':'full_read_only_copy_numeric_comparison','files':records,'arrays_equal':all(r.get('arrays_equal',True) for r in records),'file_equal_count':sum(r['local_sha256']==r['remote_copy_sha256'] for r in records)}
        print(host,len(records),report[host]['arrays_equal'],flush=True)
    except Exception as error:
        report[host]={'error':type(error).__name__+': '+str(error)}
        print(host,report[host],flush=True)
    finally: client.close()
(root.parent/'doc/yichao_v3_v9/motion_array_audit.json').write_text(json.dumps(report,indent=2)+'\n')
