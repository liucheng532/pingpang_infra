"""Explicit release operation: regenerate reviewed local file identities; never called by runtime."""
from pathlib import Path
import json,sys
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'src'))
from yichao_v3_v9.manifest import sha
folders=['vendor','src','assets/0302_combined','assets/0718-move-160-80hz','assets/robot_66','assets/robot_198']
folders.append('tools')
folders.append('scripts')
files=[root/'requirements.lock',root/'run_offline.sh',root/'run_workstation_offline.sh',root/'run_workstation_observer.sh',root/'run_planner.sh',root/'tools/capture_existing_inputs.py',root/'tools/consume_joint_feedback.py',root/'tools/convert_pose_recording.py',root/'tools/assemble_base_recording.py']+[p for p in (root/'config').glob('*.json') if p.name!='assets.lock.json']
for folder in folders:
    files.extend(p for p in (root/folder).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
manifest={'version':'yichao-v3-v9-offline-release-v1','planner_commit':'e14fd5ba3daa6327ac842d53fe5c4292b04e5716','deploy_commit':'347890eb050a9b7f0856f311449d68f6411b6d92','evidence':'fixed git snapshots, local copied motion banks, read-only robot ONNX/sidecar snapshots','real_interface_accepted':False,'sha256':{str(p.relative_to(root)):sha(p) for p in sorted(files)}}
(root/'config/assets.lock.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('frozen',len(files),'files')
