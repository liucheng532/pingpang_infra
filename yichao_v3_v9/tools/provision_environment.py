"""Copy a minimal CPU dependency closure from the invoking Python into this release's venv.

Run using an existing read-only CPU inference environment. All writes stay inside
this release. Existing venvs are refused rather than upgraded in place.
"""
import importlib.metadata as metadata
from pathlib import Path
import shutil
import subprocess
import sys
import json
from packaging.requirements import Requirement
from packaging.markers import default_environment

root=Path(__file__).resolve().parents[1]
venv=root/'.venv'
if venv.exists(): raise SystemExit('Refusing to overwrite an existing release environment')
if '+cpu' not in metadata.version('torch'): raise SystemExit('Source environment must contain a CPU-only Torch build')
subprocess.run([sys.executable,'-B','-m','venv','--copies',str(venv)],check=True)
dest=venv/f'lib/python{sys.version_info.major}.{sys.version_info.minor}/site-packages'
pending=['torch','onnxruntime','numpy','packaging'];seen={};sources={}
markers=default_environment();markers['extra']=''
while pending:
    dist=metadata.distribution(pending.pop())
    name=dist.metadata['Name'].lower().replace('_','-')
    if name in seen:continue
    seen[name]=dist.version
    sources[name]={'version':dist.version,'source':str(dist.locate_file(''))}
    for item in dist.files or []:
        if '..' in item.parts or item.suffix=='.pyc':continue
        source=Path(dist.locate_file(item))
        if source.is_file():
            target=dest/item;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    for value in dist.requires or []:
        req=Requirement(value)
        if req.marker is None or req.marker.evaluate(markers):pending.append(req.name)
(root/'requirements.lock').write_text(''.join(f'{name}=={version}\n' for name,version in sorted(seen.items())))
(root/'config/environment_provenance.json').write_text(json.dumps({'python':sys.version,'base_executable':sys.executable,'packages':sources,'writes':'release-local venv only','shared_environment_modified':False},indent=2)+'\n')
print(json.dumps(seen,indent=2))
