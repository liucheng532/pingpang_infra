"""Copy ROS Python dependencies into this release. Run from the existing CPU env.

No pip/apt, no shared writes, no ROS node creation. Existing inference versions
must match. ROS imports run with bytecode disabled by the invoking python -B.
"""
import hashlib
import importlib.metadata as metadata
import json
from pathlib import Path
import shutil
import sys
from packaging.markers import default_environment
from packaging.requirements import Requirement

root = Path(__file__).resolve().parents[1]
ros_source = Path('/opt/ros/noetic/lib/python3/dist-packages')
ros_dest = root / 'vendor/ros1'
dest = root / '.venv' / ('lib/python%d.%d/site-packages' % sys.version_info[:2])
if not dest.is_dir() or ros_dest.exists():
    raise SystemExit('matching private venv required; existing ROS snapshot is refused')
sys.path.insert(0, str(ros_source))
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

top_ros, site_imports = set(), set()
for name, module in list(sys.modules.items()):
    path = getattr(module, '__file__', None)
    if not path:
        continue
    path = Path(path).resolve()
    if path.is_relative_to(ros_source):
        top_ros.add(path.relative_to(ros_source).parts[0])
    elif 'site-packages' in path.parts:
        site_imports.add(name.split('.')[0])
owners = metadata.packages_distributions()
pending = [name for module in site_imports for name in owners.get(module, [])]
markers = default_environment()
markers['extra'] = ''
closure = {}
while pending:
    dist = metadata.distribution(pending.pop())
    name = dist.metadata['Name'].lower().replace('_', '-')
    if name in closure:
        continue
    closure[name] = dist
    for text in dist.requires or []:
        req = Requirement(text)
        if req.marker is None or req.marker.evaluate(markers):
            pending.append(req.name)
versions = dict(line.strip().split('==') for line in (root/'requirements.lock').read_text().splitlines() if line.strip())
# venv bootstrap setuptools is already private and catkin-pkg accepts it without
# a version constraint. Keep it; don't overwrite it with the source env version.
private_dists = {d.metadata['Name'].lower().replace('_', '-'): d
                 for d in metadata.distributions(path=[str(dest)])}
reused = {}
if 'setuptools' in closure and 'setuptools' in private_dists:
    closure['setuptools'] = private_dists['setuptools']
    reused['setuptools'] = closure['setuptools'].version
    versions['setuptools'] = closure['setuptools'].version
for name, dist in closure.items():
    if name in versions and versions[name] != dist.version:
        raise SystemExit('refusing existing dependency version change: ' + name)
# Validate every destination before writing any additional file.
for name, dist in closure.items():
    if name in versions:
        continue
    for item in dist.files or []:
        if '..' in item.parts or item.suffix == '.pyc':
            continue
        source, target = Path(dist.locate_file(item)), dest/item
        if source.is_file() and target.exists() and target.read_bytes() != source.read_bytes():
            raise SystemExit('refusing to overwrite existing private dependency: ' + str(item))
copied = {}
for name, dist in sorted(closure.items()):
    if name in versions:
        continue
    copied[name] = {'version': dist.version, 'source': str(dist.locate_file(''))}
    for item in dist.files or []:
        if '..' in item.parts or item.suffix == '.pyc':
            continue
        source = Path(dist.locate_file(item))
        if source.is_file():
            target = dest/item
            if target.exists() and target.read_bytes() != source.read_bytes():
                raise SystemExit('refusing to overwrite existing private dependency: ' + str(item))
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
    versions[name] = dist.version
ros_dest.mkdir()
for top in sorted(top_ros):
    source = ros_source/top
    if source.is_dir():
        shutil.copytree(source, ros_dest/top, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    else:
        shutil.copy2(source, ros_dest/top)
hashes = {str(p.relative_to(ros_dest)): hashlib.sha256(p.read_bytes()).hexdigest()
          for p in ros_dest.rglob('*') if p.is_file()}
(root/'requirements.lock').write_text(''.join('%s==%s\n' % pair for pair in sorted(versions.items())))
(root/'config/observer_environment.json').write_text(json.dumps({
    'source_python': sys.executable, 'ros_source': str(ros_source), 'ros_packages': sorted(top_ros),
    'copied_distributions': copied, 'reused_private_distributions': reused, 'ros_sha256': hashes,
    'shared_environment_modified': False, 'node_created': False}, indent=2)+'\n')
print(json.dumps({'copied_distributions': copied, 'ros_packages': sorted(top_ros),
                  'ros_files': len(hashes)}, indent=2))
