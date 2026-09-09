#!/usr/bin/env python3
"""Seed missing snapshot files only from local bytes matching remote SHA-256.

Copies get independent inodes. Existing paths and symlink parents are never replaced.
This is an optional download accelerator, not a deployment command.
"""
import hashlib
import json
import os
import shutil
import tempfile
from collections import defaultdict
from pathlib import Path
from device_context_ssh import ROOT, HOSTS


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def safe_parent(path):
    rel = path.relative_to(ROOT)
    current = ROOT
    for part in rel.parts[:-1]:
        current /= part
        if current.is_symlink():
            return False
    return True


def main():
    wanted = defaultdict(list)
    for device, (_, folder, _) in HOSTS.items():
        p = ROOT / 'doc/device_context' / (device + '_sha256.json')
        if not p.exists():
            continue
        for record in json.loads(p.read_text())['files']:
            target = ROOT / folder / record['path'].lstrip('/')
            if not os.path.lexists(target):
                wanted[record['size']].append((target, record))
    by_hash = defaultdict(list)
    for records in wanted.values():
        for target, record in records:
            by_hash[record['sha256']].append((target, record))
    sources = [ROOT / 'yichao_v3_v9', ROOT / 'pingpang_deploy', ROOT / 'pingpang_assets']
    sources += [ROOT / host[1] for host in HOSTS.values()]
    seeded = []; errors = []
    for source in sources:
        for here, dirs, files in os.walk(source, followlinks=False):
            dirs[:] = [d for d in dirs if d not in ['.venv','__pycache__','.context-rsync-partial'] and not (Path(here)/d).is_symlink()]
            for name in files:
                path = Path(here) / name
                try:
                    if path.is_symlink() or not path.is_file() or path.stat().st_size not in wanted:
                        continue
                    sha = digest(path)
                    candidates = by_hash.pop(sha, [])
                    for target, record in candidates:
                        if os.path.lexists(target) or not safe_parent(target):
                            continue
                        target.parent.mkdir(parents=True, exist_ok=True)
                        fd, temporary = tempfile.mkstemp(prefix='.context-seed-', dir=target.parent)
                        os.close(fd)
                        temp = Path(temporary)
                        try:
                            shutil.copyfile(path, temp)
                            if digest(temp) != sha:
                                raise ValueError('Source changed: ' + str(path))
                            os.chmod(temp, record['mode'])
                            os.utime(temp, ns=(record['mtime_ns'], record['mtime_ns']))
                            try:
                                os.link(temp, target)  # atomic create-only; temp is removed below
                            except FileExistsError:
                                continue
                            seeded.append({'target':str(target),'sha256':sha,'bytes':record['size']})
                        finally:
                            temp.unlink(missing_ok=True)
                except (OSError, ValueError) as e:
                    errors.append(str(e))
    result = {'seeded':seeded,'errors':errors,'count':len(seeded),'bytes':sum(x['bytes'] for x in seeded)}
    (ROOT / 'doc/device_context/seed_result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k != 'seeded'}))


if __name__ == '__main__':
    main()
