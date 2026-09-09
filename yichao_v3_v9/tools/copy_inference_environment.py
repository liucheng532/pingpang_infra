"""Copy a minimal installed CPU dependency closure into an isolated venv; no shared writes."""
import importlib.metadata as md
from pathlib import Path
import shutil
import sys
from packaging.requirements import Requirement
from packaging.markers import default_environment

root = Path(__file__).resolve().parents[1]
dest = root / '.venv/lib/python3.11/site-packages'
pending = ['torch', 'onnxruntime', 'numpy', 'packaging']
seen = {}
environment = default_environment()
environment['extra'] = ''
while pending:
    name = pending.pop()
    dist = (next(d for d in md.distributions(path=['/home/lyz/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages']) if d.metadata['Name'].lower() == 'numpy') if name.lower() == 'numpy' else md.distribution(name))
    canonical = dist.metadata['Name'].lower().replace('_', '-')
    if canonical in seen:
        continue
    seen[canonical] = dist.version
    for item in dist.files or []:
        if '..' in item.parts or item.suffix == '.pyc':
            continue
        source = Path(dist.locate_file(item))
        if source.is_file():
            target = dest / item
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    for raw in dist.requires or []:
        requirement = Requirement(raw)
        if requirement.marker is None or requirement.marker.evaluate(environment):
            pending.append(requirement.name)
(root / 'requirements.lock').write_text(''.join(f'{k}=={v}\n' for k,v in sorted(seen.items())))
print(seen)
