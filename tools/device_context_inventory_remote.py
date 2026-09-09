"""Run on a remote host over stdin, with CONFIG substituted by the local caller."""
import datetime
import fnmatch
import json
import os
import platform
import subprocess
from pathlib import Path


def excluded(relative, directory, rules):
    for rule in rules:
        only_dir = rule.endswith('/')
        pattern = rule.rstrip('/')
        if only_dir and not directory:
            continue
        if pattern.startswith('/'):
            match = relative == pattern[1:]
        else:
            match = fnmatch.fnmatch(Path(relative).name, pattern)
        if match:
            return rule
    return None


def run(args, cwd=None):
    env = dict(os.environ, GIT_OPTIONAL_LOCKS='0')
    p = subprocess.run(args, cwd=cwd, env=env, capture_output=True, text=True)
    return {'code': p.returncode, 'stdout': p.stdout, 'stderr': p.stderr}


result = {
    'captured_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'hostname': platform.node(), 'architecture': platform.machine(),
    'os_release': Path('/etc/os-release').read_text(),
    'python': platform.python_version(),
    'network': run(['ip', '-brief', 'address']),
    'roots': [], 'errors': [], 'environments': [],
}
for remote in CONFIG['roots'] + CONFIG.get('files', []):
    root = Path(remote)
    rules = CONFIG['common_excludes'] + CONFIG['root_excludes']['default'] + CONFIG['root_excludes'].get(root.name, [])
    entry = {'remote': remote, 'files': [], 'directories': [], 'excluded': [], 'git': None}
    if root.is_file():
        st = root.stat()
        entry['remote'] = str(root.parent)
        entry['selected_files'] = [root.name]
        entry['files'] = [{'path':root.name, 'kind':'file', 'size':st.st_size, 'mtime_ns':st.st_mtime_ns, 'mode':st.st_mode & 0o7777}]
        result['roots'].append(entry)
        continue
    if not root.is_dir():
        result['errors'].append('Missing root: ' + remote)
        continue
    if (root / '.git').exists():
        entry['git'] = {key: run(['git', *args], cwd=str(root)) for key, args in {
            'head': ['rev-parse', 'HEAD'],
            'branch': ['branch', '--show-current'],
            'status': ['status', '--porcelain=v1', '--untracked-files=all'],
            'refs': ['show-ref'],
            'git_dir': ['rev-parse', '--absolute-git-dir'],
            'common_dir': ['rev-parse', '--git-common-dir'],
        }.items()}
    def walk_error(error):
        result['errors'].append(str(error))
    for here, dirs, files in os.walk(root, onerror=walk_error, followlinks=False):
        for name in sorted(dirs + files):
            p = Path(here) / name
            rel = str(p.relative_to(root))
            is_dir = name in dirs
            rule = excluded(rel, is_dir, rules)
            if rule:
                entry['excluded'].append({'path': rel, 'rule': rule})
                if is_dir:
                    dirs.remove(name)
                continue
            try:
                st = p.lstat()
                if p.is_symlink():
                    entry['files'].append({'path': rel, 'kind': 'symlink', 'target': os.readlink(p), 'exists_on_remote': p.exists()})
                    if is_dir:
                        dirs.remove(name)
                elif is_dir:
                    entry['directories'].append(rel)
                elif p.is_file():
                    entry['files'].append({'path': rel, 'kind': 'file', 'size': st.st_size, 'mtime_ns': st.st_mtime_ns, 'mode': st.st_mode & 0o7777})
                else:
                    entry['files'].append({'path': rel, 'kind': 'special', 'mode': st.st_mode})
            except OSError as e:
                result['errors'].append(str(e))
    result['roots'].append(entry)
# Read installed distribution metadata, without activating/importing project environments.
home = Path.home()
for env in [*(Path(p)/'.venv' for p in CONFIG['roots']), *sorted((home/'miniconda3/envs').glob('*'))]:
    if not env.is_dir():
        continue
    packages = []
    for site in sorted((env/'lib').glob('python*/site-packages')):
        for dist in sorted(site.glob('*.dist-info/METADATA')):
            fields = {}
            try:
                with dist.open(errors='replace') as handle:
                    for line in handle:
                        if line.startswith(('Name:', 'Version:')):
                            k, v = line.split(':', 1); fields[k] = v.strip()
                        if not line.strip():
                            break
                packages.append(fields)
            except OSError as e:
                result['errors'].append(str(e))
    conda = []
    for p in sorted((env/'conda-meta').glob('*.json')):
        try:
            d = json.loads(p.read_text()); conda.append({k:d.get(k) for k in ['name','version','build']})
        except (OSError, ValueError) as e:
            result['errors'].append(str(e))
    result['environments'].append({'path': str(env), 'distributions': packages, 'conda_packages': conda})
print(json.dumps(result))
