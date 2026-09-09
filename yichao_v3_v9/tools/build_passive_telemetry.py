"""Build the read-only DDS/LCM helper in this independent release.

Copy SDK dependencies from a reviewed source. Existing destination files must
match; no source tree, shared library installation, or process is modified.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sdk-source',type=Path,required=True)
    args=p.parse_args()
    root=Path(__file__).resolve().parents[1]
    if not root.name.startswith('yichao_'):
        p.error('independent Yichao release required')
    arch=platform.machine()
    if arch not in ('aarch64','x86_64'): p.error('unsupported architecture')
    sdk=root/'sdk';origin=args.sdk_source.resolve()
    for folder in ['include','lcm_types','thirdparty/include','thirdparty/lib/'+arch,
                   'lib/'+arch,'local_lcm']:
        source=origin/folder
        if not source.is_dir(): raise RuntimeError('missing SDK directory: '+str(source))
        for file in source.rglob('*'):
            if not file.is_file():continue
            dest=sdk/file.relative_to(origin)
            if dest.exists():
                if digest(dest)!=digest(file):raise RuntimeError('SDK snapshot conflict: '+str(dest))
                continue
            dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(file,dest)
    output=root/'passive_v9_telemetry'
    if output.exists(): p.error('existing binary retained; review it before rebuilding')
    command=['nice','-n','19','g++','-std=c++17','-O1','-pthread']
    command += ['-I'+str(sdk/d) for d in ['include','lcm_types','thirdparty/include',
                                        'thirdparty/include/ddscxx','local_lcm/include']]
    command += [str(root/'tools/passive_v9_telemetry.cpp'),str(sdk/'lib'/arch/'libunitree_sdk2.a'),
                '-L'+str(sdk/'local_lcm/lib'),'-L'+str(sdk/'thirdparty/lib'/arch),
                '-Wl,-rpath,$ORIGIN/sdk/thirdparty/lib/'+arch+':$ORIGIN/sdk/local_lcm/lib',
                '-lddscxx','-lddsc','-llcm','-o',str(output)]
    result=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=120)
    evidence={'sdk_source':str(origin),'command':command,'exit':result.returncode,'build_output':result.stdout}
    if result.returncode==0:
        evidence['sha256']={str(f.relative_to(root)):digest(f) for f in [output]+list(sdk.rglob('*')) if f.is_file()}
        evidence['linked_libraries']=subprocess.check_output(['ldd',str(output)],text=True)
    (root/'config/native_build.json').write_text(json.dumps(evidence,indent=2)+'\n')
    print(json.dumps({'exit':result.returncode,'binary':str(output),'process_started':False}))
    raise SystemExit(result.returncode)


if __name__=='__main__':main()
