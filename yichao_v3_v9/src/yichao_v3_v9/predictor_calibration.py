"""Capture the shared daily calibration once for each Predictor run."""
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
from pathlib import Path


def capture_calibration(snapshot, source, destination):
    """Validate with the Predictor's own reader, then save exactly those bytes.

    Code/assets remain frozen in ``snapshot``. Calibration comes from the
    shared source in the deployment manifest, never the old code snapshot.
    A running session keeps its copy; the next launch reads the source again.
    This function imports no SDK, ROS or robot controller.
    """
    snapshot = Path(snapshot).resolve()
    source = Path(source)
    if not source.is_absolute():
        raise ValueError('shared calibration source must be an absolute path')
    resolved = source.resolve(strict=True)
    if resolved == snapshot or snapshot in resolved.parents:
        raise ValueError('shared calibration source must be outside the code snapshot')
    validator_path = snapshot/'calib/runtime_calibration.py'
    spec = importlib.util.spec_from_file_location('predictor_calibration_reader', validator_path)
    reader = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(reader)
    blob = resolved.read_bytes()
    state = reader.validate_calibration_config_payload(json.loads(blob), str(source))
    table = reader.require_table(state)
    left, right = reader.require_doubles_trackers(state)
    target = Path(destination).resolve()/'calibration_config.json'
    record = {
        'source_path': str(source),
        'resolved_source_path': str(resolved),
        'snapshot_path': str(target),
        'sha256': hashlib.sha256(blob).hexdigest(),
        'updated_at': state['updated_at'],
        'captured_at_utc': datetime.now(timezone.utc).isoformat(),
        'table': table,
        'trackers': {'198': left, '66': right},
    }
    with target.open('xb') as stream:
        stream.write(blob)
    (target.parent/'calibration_provenance.json').write_text(
        json.dumps(record, indent=2)+'\n')
    return record
