from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys

import pytest


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
CHECKPOINT = Path(
    "/home/river777/Workspace/pingpang/artifacts/distill_v11_r2i299_commonhold/"
    "model_42500_teacherreset_resume.pt"
)
COMMON_HOLD = DEPLOY_ROOT / "assets/common_hold/source29_0368_upright_fk_v1.npz"


def _load_exporter():
    path = DEPLOY_ROOT / "scripts/export_v11_student_onnx.py"
    spec = importlib.util.spec_from_file_location("v11_deploy_exporter", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


EXPORTER = _load_exporter()


def test_v11_deploy_contract_is_strict_and_v10_path_remains_present() -> None:
    deploy = (DEPLOY_ROOT / "scripts/deploy_policy.py").read_text()
    assert EXPORTER.EXPECTED_CHECKPOINT_SHA256 in deploy
    assert EXPORTER.EXPECTED_MOVE_TEACHER_SHA256 in deploy
    assert EXPORTER.EXPECTED_COMMON_HOLD_SHA256 in deploy
    assert '"--startup-relative-x"' in deploy
    assert "V10_REFEND_CHECKPOINT_SHA256" in deploy
    assert "V11_EXCLUDED_MOVE_SOURCE_IDS" in deploy


@pytest.mark.skipif(not CHECKPOINT.is_file(), reason="V11 resume i42500 checkpoint unavailable")
def test_v11_export_parity_and_sidecar(tmp_path: Path) -> None:
    metadata = EXPORTER.export_student(CHECKPOINT, COMMON_HOLD, tmp_path)
    assert metadata["checkpoint_sha256"] == EXPORTER.EXPECTED_CHECKPOINT_SHA256
    assert metadata["move_teacher_sha256"] == EXPORTER.EXPECTED_MOVE_TEACHER_SHA256
    assert metadata["common_hold_pose_sha256"] == EXPORTER.EXPECTED_COMMON_HOLD_SHA256
    assert metadata["pytorch_onnx_max_action_error"] < 1.0e-5
    assert metadata["move_pool_contract"]["active_count"] == 147
    assert metadata["resume_source"]["iteration"] == 8000
    sidecar = json.loads((tmp_path / f"{EXPORTER.OUTPUT_BASENAME}.json").read_text())
    assert sidecar == metadata
