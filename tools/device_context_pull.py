#!/usr/bin/env python3
"""Read-only remote context download. Never uploads or deletes remote files.

Existing local files are protected by --ignore-existing. Use checksum verification
and a separately reviewed reconciliation for subsequent updates.
"""
import argparse
import concurrent.futures
import datetime
import json
import shlex
from pathlib import Path
from device_context_ssh import ROOT, HOSTS, SSH_OPTIONS, authenticated

CONFIG = json.loads((ROOT / 'tools/device_context_config.json').read_text())


def exclusions(remote):
    return CONFIG['common_excludes'] + CONFIG['root_excludes']['default'] + CONFIG['root_excludes'].get(Path(remote).name, [])


def transfer(device, output, verify=False):
    host, folder, _ = HOSTS[device]
    results = []
    for remote in CONFIG['roots'][device] + CONFIG.get('files', {}).get(device, []):
        local = ROOT / folder / remote.lstrip('/')
        is_file = remote in CONFIG.get('files', {}).get(device, [])
        (local.parent if is_file else local).mkdir(parents=True, exist_ok=True)
        stem = remote.strip('/').replace('/', '__')
        log = output / (device + '__' + stem + '.log')
        args = ['rsync', '-aH', '--no-owner', '--no-group', '--protect-args',
                '--rsync-path=nice -n 15 rsync', '-e', shlex.join(['ssh', *SSH_OPTIONS])]
        if verify:
            args += ['--dry-run', '--checksum', '--omit-dir-times', '--itemize-changes', '--out-format=%i %n%L']
        else:
            args += ['--compress', '--bwlimit=16384', '--partial-dir=.context-rsync-partial', '--ignore-existing', '--stats']
        args += ['--exclude=' + p for p in exclusions(remote)]
        args += [host + ':' + remote + ('' if is_file else '/'), str(local) + ('' if is_file else '/')]
        print(device + ': ' + ('verify ' if verify else 'copy ') + remote, flush=True)
        with log.open('w') as handle:
            result = authenticated(args, host=host, stdout=handle, stderr=handle)
        entry = dict(remote=remote, local=str(local), returncode=result.returncode, log=str(log),
                     finished_utc=datetime.datetime.now(datetime.timezone.utc).isoformat())
        if verify:
            entry['verification_clean'] = result.returncode == 0 and not log.read_text().strip()
        results.append(entry)
        (output / (device + '_results.json')).write_text(json.dumps(results, indent=2))
        print(device + ': ' + str(result.returncode) + ' ' + Path(remote).name, flush=True)
        if result.returncode not in (0, 24):
            print('Inspect ' + str(log), flush=True)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', choices=['all', *HOSTS], default='all')
    parser.add_argument('--verify', action='store_true')
    args = parser.parse_args()
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output = ROOT / 'doc/device_context' / (('verify_' if args.verify else 'pull_') + stamp)
    output.mkdir(parents=True)
    (output / 'config.json').write_text(json.dumps(CONFIG, indent=2))
    devices = list(HOSTS) if args.device == 'all' else [args.device]
    with concurrent.futures.ThreadPoolExecutor(max_workers=len(devices)) as pool:
        futures = [pool.submit(transfer, key, output, args.verify) for key in devices]
        all_results = [future.result() for future in futures]
    print('Evidence: ' + str(output), flush=True)
    raise SystemExit(int(any(x['returncode'] != 0 or (args.verify and not x['verification_clean']) for results in all_results for x in results)))


if __name__ == '__main__':
    main()
