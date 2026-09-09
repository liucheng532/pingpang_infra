#!/usr/bin/env python3
"""Verify downloaded content against captured remote SHA256 and symlinks.

No remote writes or project imports. This checks the captured baseline; use
`device_context_pull.py --verify` to compare against the remote current contents.
"""
import datetime
import hashlib
import json
import os
import stat
from pathlib import Path
from device_context_ssh import ROOT, HOSTS


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    report = {'captured_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(), 'devices':{}}
    for device, (_, folder, _) in HOSTS.items():
        manifest = json.loads((ROOT/'doc/device_context'/f'{device}_manifest.json').read_text())
        hashes = json.loads((ROOT/'doc/device_context'/f'{device}_sha256.json').read_text())
        errors = [*manifest['errors'], *hashes['errors']]
        checked = 0; total_bytes = 0; links = []; missing_dirs = []
        expected_regular = set()
        for record in hashes['files']:
            path = ROOT / folder / record['path'].lstrip('/')
            expected_regular.add(record['path'])
            try:
                st = path.lstat()
                if not stat.S_ISREG(st.st_mode):
                    raise ValueError('Expected regular file')
                if st.st_size != record['size'] or sha256(path) != record['sha256']:
                    raise ValueError('Content mismatch')
                if stat.S_IMODE(st.st_mode) != record['mode']:
                    raise ValueError('Mode mismatch')
                checked += 1; total_bytes += record['size']
            except (OSError, ValueError) as e:
                errors.append({'path':str(path), 'error':str(e)})
        for entry in manifest['roots']:
            base = ROOT / folder / entry['remote'].lstrip('/')
            for directory in entry['directories']:
                if not (base/directory).is_dir():
                    missing_dirs.append(str(base/directory))
            for record in entry['files']:
                path = base / record['path']
                if record['kind'] == 'file' and entry['remote']+'/'+record['path'] not in expected_regular:
                    errors.append({'path':str(path),'error':'Missing remote SHA256'})
                if record['kind'] == 'symlink':
                    try:
                        if not path.is_symlink() or os.readlink(path) != record['target']:
                            raise ValueError('Symlink mismatch')
                        target = record['target']
                        mapped = ROOT/folder/target.lstrip('/') if target.startswith('/') else path.parent/target
                        links.append({'local':str(path),'remote_target':target,'mapped_local':str(mapped),'target_present':mapped.exists()})
                    except (OSError, ValueError) as e:
                        errors.append({'path':str(path),'error':str(e)})
                elif record['kind'] == 'special':
                    errors.append({'path':str(path),'error':'Special file requires manual review'})
        report['devices'][device] = {'regular_files_checked':checked,'regular_bytes':total_bytes,
            'symlinks':links,'errors':errors,'missing_directories':missing_dirs,'roots':len(manifest['roots'])}
        print(device, checked, 'files,', len(links), 'links,', len(errors), 'errors,',len(missing_dirs),'missing dirs',flush=True)
    report['passed'] = all(not d['errors'] and not d['missing_directories'] for d in report['devices'].values())
    (ROOT/'doc/device_context/verification.json').write_text(json.dumps(report,indent=2))
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
