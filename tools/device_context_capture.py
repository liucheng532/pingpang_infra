#!/usr/bin/env python3
"""Capture read-only inventory and SHA256 for selected device paths over SSH."""
import argparse
import json
from pathlib import Path
from device_context_ssh import ROOT, HOSTS, ssh


def capture(device, roots, files, merge=False):
    config=json.loads((ROOT/'tools/device_context_config.json').read_text())
    config.update(roots=roots,files=files)
    source='CONFIG = '+ascii(config)+'\n'+(ROOT/'tools/device_context_inventory_remote.py').read_text()
    r=ssh(device,'nice -n 19 python3 -',input=source,text=True,capture_output=True,timeout=300)
    if r.returncode:raise RuntimeError(r.stderr)
    manifest=json.loads(r.stdout)
    paths=[str(Path(entry['remote'])/f['path']) for entry in manifest['roots'] for f in entry['files'] if f['kind']=='file']
    source='ITEMS = '+ascii(paths)+'\n'+(ROOT/'tools/device_context_hash_remote.py').read_text()
    r=ssh(device,'nice -n 19 python3 -',input=source,text=True,capture_output=True,timeout=300)
    if r.returncode:raise RuntimeError(r.stderr)
    hashes=json.loads(r.stdout)
    destination=ROOT/'doc/device_context'
    if merge:
        old=json.loads((destination/f'{device}_manifest.json').read_text())
        def key(entry):return (entry['remote'],tuple(entry.get('selected_files',[])))
        replaced={key(entry) for entry in manifest['roots']}
        old['roots']=[entry for entry in old['roots'] if key(entry) not in replaced]+manifest['roots']
        old['errors']+=manifest['errors']
        old.setdefault('supplement_capture_utc',[]).append(manifest['captured_utc'])
        manifest=old
        old=json.loads((destination/f'{device}_sha256.json').read_text())
        replaced={entry['path'] for entry in hashes['files']}
        old['files']=[entry for entry in old['files'] if entry['path'] not in replaced and not any(entry['path'].startswith(root.rstrip('/')+'/') for root in roots) and entry['path'] not in files]+hashes['files']
        old['errors']+=hashes['errors'];hashes=old
    (destination/f'{device}_manifest.json').write_text(json.dumps(manifest,indent=2))
    (destination/f'{device}_sha256.json').write_text(json.dumps(hashes,indent=2))
    print(device,len(manifest['roots']),'scope entries,',len(hashes['files']),'hashes;',len(manifest['errors'])+len(hashes['errors']),'errors',flush=True)
    if manifest['errors'] or hashes['errors']:raise RuntimeError('Capture incomplete; inspect manifests')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--device',choices=HOSTS,required=True)
    p.add_argument('--supplement',type=Path,help='JSON roots/files subset; merges captured records')
    args=p.parse_args()
    c=json.loads((ROOT/'tools/device_context_config.json').read_text())
    subset=json.loads(args.supplement.read_text()) if args.supplement else {'roots':c['roots'][args.device],'files':c.get('files',{}).get(args.device,[])}
    capture(args.device,subset['roots'],subset.get('files',[]),bool(args.supplement))


if __name__=='__main__':main()
