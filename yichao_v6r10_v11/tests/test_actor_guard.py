import json
from pathlib import Path

import numpy as np
import pytest

from yichao_v6r10_v11 import ROOT
from yichao_v6r10_v11.actor import OnnxActor, rebuild_torch_actor, sha256
from yichao_v6r10_v11.guard import project_actor_pair


def test_actor_artifact_identity_shape_determinism_and_pt_parity():
    import torch

    onnx_path = ROOT / "models/model630_actor.onnx"
    sidecar_path = ROOT / "models/model630_actor.onnx.json"
    fixtures = ROOT / "tests/fixtures"
    sidecar = json.loads(sidecar_path.read_text())
    assert sidecar["checkpoint_sha256"] == "81501373c08f6f4aa6bae3fdaccdd073939e5346d2b25a8e419ab35c6657ed0c"
    assert sidecar["onnx_sha256"] == sha256(onnx_path)
    assert sidecar["barrier_dependency"] is None
    actor = OnnxActor(onnx_path, sidecar_path)
    reference, _ = rebuild_torch_actor(fixtures / "model_630.pt")
    rng = np.random.default_rng(630)
    cases = [
        np.zeros((1, 36), np.float32),
        np.full((4, 36), 3.0, np.float32),
        np.full((4, 36), -3.0, np.float32),
        rng.uniform(-3, 3, (127, 36)).astype(np.float32),
    ]
    for values in cases:
        first = actor(values)
        second = actor(values)
        expected = reference(torch.from_numpy(values)).detach().numpy()
        assert first.shape == (len(values), 2)
        np.testing.assert_array_equal(first, second)
        np.testing.assert_allclose(first, expected, atol=2e-5, rtol=0)


def test_actor_rejects_bad_shape_and_nonfinite():
    actor = OnnxActor(ROOT / "models/model630_actor.onnx", ROOT / "models/model630_actor.onnx.json")
    with pytest.raises(ValueError):
        actor(np.zeros((2, 35), np.float32))
    values = np.zeros((1, 36), np.float32)
    values[0, 4] = np.nan
    with pytest.raises(ValueError):
        actor(values)


def test_actor_rejects_swapped_sidecar_mapping(tmp_path):
    source = ROOT / "models/model630_actor.onnx.json"
    sidecar = json.loads(source.read_text())
    sidecar["ros_mapping"] = {"left": "table_right", "right": "table_left"}
    changed = tmp_path / "swapped.json"
    changed.write_text(json.dumps(sidecar))
    with pytest.raises(ValueError, match="sidecar mismatch"):
        OnnxActor(ROOT / "models/model630_actor.onnx", changed)


def test_guard_passthrough_and_exact_boundary():
    direct = project_actor_pair(np.asarray([0.6, -0.2], np.float32), "left")
    assert direct.valid and not direct.intervened
    np.testing.assert_allclose(direct.projected, [0.54, -0.18], atol=1e-6)
    exact = project_actor_pair(np.asarray([0.25 / 0.9, -0.25 / 0.9]), "right")
    assert exact.valid and not exact.intervened
    assert exact.projected[0] - exact.projected[1] == pytest.approx(0.5, abs=2e-7)


def test_guard_hitter_priority_is_asymmetric():
    left = project_actor_pair(np.asarray([0.2 / 0.9, 0.1 / 0.9]), "left")
    assert left.valid and left.intervened
    np.testing.assert_allclose(left.projected, [0.2, -0.3], atol=1e-6)
    right = project_actor_pair(np.asarray([-0.1 / 0.9, -0.2 / 0.9]), "right")
    assert right.valid and right.intervened
    np.testing.assert_allclose(right.projected, [0.3, -0.2], atol=1e-6)


def test_guard_workspace_failure_and_nan_are_explicit():
    unavailable = project_actor_pair(np.asarray([-0.8 / 0.9, -0.7 / 0.9]), "left")
    assert not unavailable.valid and unavailable.reason == "guard_projection_unavailable"
    assert unavailable.projected is None
    invalid = project_actor_pair(np.asarray([np.nan, 0.0]), "right")
    assert not invalid.valid and invalid.reason == "actor_output_nonfinite"
    record = invalid.to_dict()
    assert record["barrier_risk"] == "not_evaluated"
    assert record["filter_mode"] == "off"
