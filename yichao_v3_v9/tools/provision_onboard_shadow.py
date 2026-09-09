"""Snapshot installed onboard dependencies into this release; no pip/apt or services.

Run with the robot's existing Python 3.8, -B. Source packages are only read;
the destination must be a new independent Yichao release directory.
"""
import hashlib
import argparse
import importlib.metadata as md
import importlib.util
import json
from pathlib import Path
import shutil
import sys
import venv
from packaging.requirements import Requirement
from packaging.markers import default_environment


def distribution_files(dist):
    """Debian egg-info can omit RECORD/SOURCES entirely; resolve its modules."""
    if dist.files:
        return [Path(dist.locate_file(item)).resolve() for item in dist.files]
    name = dist.metadata['Name'].lower().replace('_','-')
    names = (dist.read_text('top_level.txt') or '').split() or [name.replace('-', '_')]
    if name == 'pyyaml':
        names = ['yaml', '_yaml']
    if not names:
        raise RuntimeError('cannot identify installed distribution files: '+name)
    paths = [Path(dist._path)]  # installed metadata, needed for version checks
    for module in names:
        spec = importlib.util.find_spec(module)
        if spec is None:
            raise RuntimeError('installed module missing: '+module)
        if spec.submodule_search_locations:
            paths.extend(Path(p) for p in spec.submodule_search_locations)
        elif spec.origin:
            paths.append(Path(spec.origin))
    return [p.resolve() for path in paths for p in
            (path.rglob('*') if path.is_dir() else [path]) if p.is_file()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--complete-missing', action='store_true',
                        help='complete missing files in an already recorded private snapshot')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    env = root/'.venv'
    if not root.name.startswith('yichao_') or (env.exists() and not args.complete_missing):
        raise SystemExit('new independent release and absent private venv required')
    previous = None
    if args.complete_missing:
        previous = json.loads((root/'config/onboard_environment.json').read_text())
        for relative, digest in previous['sha256'].items():
            path = root/relative
            if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
                raise RuntimeError('recorded snapshot changed: '+relative)
    ros = Path('/opt/ros/noetic/lib/python3/dist-packages')
    sys.path.insert(0, str(ros))
    import rospy
    from std_msgs.msg import String, Float32
    from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
    ros_packages = set()
    for module in tuple(sys.modules.values()):
        path = getattr(module, '__file__', None)
        if path:
            try:
                ros_packages.add(Path(path).resolve().relative_to(ros).parts[0])
            except ValueError:
                pass
    pending = ['torch', 'onnx', 'onnxruntime', 'lcm', 'numpy', 'packaging',
               'rospkg', 'catkin-pkg', 'PyYAML', 'six', 'pyparsing', 'netifaces']
    markers = default_environment(); markers['extra'] = ''
    closure = {}
    while pending:
        dist = md.distribution(pending.pop())
        name = dist.metadata['Name'].lower().replace('_','-')
        if name in closure:
            continue
        closure[name] = dist
        for raw in dist.requires or []:
            req = Requirement(raw)
            if req.marker is None or req.marker.evaluate(markers):
                if req.specifier and md.version(req.name) not in req.specifier:
                    raise RuntimeError('installed dependency mismatch: '+str(req))
                pending.append(req.name)
    if previous:
        if {n:d.version for n,d in closure.items()} != previous['versions']:
            raise RuntimeError('source dependency versions changed')
    else:
        venv.EnvBuilder(with_pip=False, system_site_packages=False).create(env)
    dest = env/('lib/python%d.%d/site-packages' % sys.version_info[:2])
    versions, origins, hashes = {}, {}, {}
    for name, dist in sorted(closure.items()):
        versions[name] = dist.version
        origin = Path(dist.locate_file('')).resolve()
        origins[name] = str(origin)
        for source in distribution_files(dist):
            try:
                relative = source.relative_to(origin)
            except ValueError:
                continue  # no installed scripts/system paths copied
            if not source.is_file() or source.suffix == '.pyc':
                continue
            target = dest/relative
            digest = hashlib.sha256(source.read_bytes()).hexdigest()
            if target.exists() and hashlib.sha256(target.read_bytes()).hexdigest() != digest:
                raise RuntimeError('dependency destination conflict: '+str(relative))
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                shutil.copy2(source, target)
            hashes[str(target.relative_to(root))] = digest
    ros_dest = root/'vendor/ros1'
    if not previous:
        ros_dest.mkdir()
        for name in sorted(ros_packages):
            source, target = ros/name, ros_dest/name
            if source.is_dir():
                shutil.copytree(source,target,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
            else:
                shutil.copy2(source,target)
    for path in ros_dest.rglob('*'):
        if path.is_file():
            hashes[str(path.relative_to(root))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (root/'requirements.lock').write_text(''.join('%s==%s\n'%x for x in sorted(versions.items())))
    result = dict(source_python=sys.executable, versions=versions, origins=origins,
                  ros_packages=sorted(ros_packages), sha256=hashes,
                  shared_environment_modified=False, node_created=False)
    (root/'config/onboard_environment.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'versions':versions,'copied_files':len(hashes),'shared_environment_modified':False}))


if __name__ == '__main__': main()
