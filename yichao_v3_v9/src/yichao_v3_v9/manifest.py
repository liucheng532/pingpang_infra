import hashlib
import json
from . import ROOT, HANDOFF


def sha(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): h.update(chunk)
    return h.hexdigest()


def verify():
    import importlib.metadata as metadata
    for line in (ROOT/'requirements.lock').read_text().splitlines():
        name,version=line.split('==')
        if metadata.version(name)!=version:
            raise ValueError('dependency_version_mismatch:'+name)
    manifest=json.loads((ROOT/'config/assets.lock.json').read_text())
    for path,expected in manifest['sha256'].items():
        p=ROOT/path
        if not p.is_file() or sha(p)!=expected:
            raise ValueError('frozen_asset_mismatch:'+path)
    for line in (HANDOFF/'SHA256SUMS').read_text().splitlines():
        if not line.strip(): continue
        digest,path=line.split(maxsplit=1)
        if sha(HANDOFF/path.lstrip('*'))!=digest: raise ValueError('handoff_checksum:'+path)
    side=json.loads((ROOT/'assets/robot_66/student_v9_m14500_timedhandoff_1666_model19000.onnx.json').read_text())
    if (side['input_dim'],side['output_dim'],side['iteration'])!=(1666,29,19000): raise ValueError('low_level_ABI')
    if side['phase_order']!=['HIT','POST_DELAY','OUTWARD','OUTWARD_HOLD','RETURN','HOME_HOLD']: raise ValueError('phase_order')
    for bank,key in [('0302_combined','hit_motion_manifest_sha256'),('0718-move-160-80hz','move_motion_manifest_sha256')]:
        if sha(ROOT/'assets'/bank/'dataindex.csv')!=side[key]: raise ValueError('motion_manifest:'+bank)
    if sha(ROOT/'assets/robot_66/student_v9_m14500_timedhandoff_1666_model19000.onnx')!=side['onnx_sha256']: raise ValueError('student_ONNX')
    if sha(HANDOFF/'checkpoints/v9_student_iteration_19000.pt')!=side['checkpoint_sha256']: raise ValueError('student_checkpoint')
    return manifest
