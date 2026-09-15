import json
import time

import numpy as np
import pytest

from yichao_v7_v11 import ROOT
from yichao_v7_v11.actor import (
    OnnxActor,
    OnnxBarrier,
    rebuild_torch_actor,
    rebuild_torch_barrier,
    sha256,
)
from yichao_v7_v11.guard import apply_hard_guard, project_v7_pair


class ConstantBarrier:
    def __init__(self, risk):
        self.risk = risk

    def __call__(self, states, actions):
        actions = np.asarray(actions, dtype=np.float32)
        if actions.ndim == 1:
            actions = actions[None, :]
        risk = np.full(len(actions), self.risk, np.float32)
        return {
            "head_probabilities": np.repeat(risk[:, None, None], 9, axis=1).reshape(-1, 3, 3),
            "member_risk": np.repeat(risk[:, None], 3, axis=1),
            "conservative_risk": risk,
        }


def models():
    contract = ROOT / "models/model_contract.json"
    return (
        OnnxActor(ROOT / "models/v7_planner_actor.onnx", contract),
        OnnxBarrier(ROOT / "models/frozen_v7_barrier_risk.onnx", contract),
    )


def test_bundle_identity_dynamic_abi_and_strict_contract(tmp_path):
    actor, barrier = models()
    assert sha256(actor.model_path) == "c79054c4f0a64b6fc64ad44b2308c97e7fe8854a27feecb81845f2e97f323a7f"
    assert sha256(barrier.model_path) == "77f68d863abd4531cd63fbf31d8052d448e37309cfefb436439cb8264d505e54"
    assert actor(np.zeros((7, 29), np.float32)).shape == (7, 2)
    result = barrier(np.zeros((7, 87), np.float32), np.zeros((7, 2), np.float32))
    assert result["head_probabilities"].shape == (7, 3, 3)
    assert result["member_risk"].shape == (7, 3)
    assert result["conservative_risk"].shape == (7,)

    changed = json.loads((ROOT / "models/model_contract.json").read_text())
    changed["ros_mapping"] = {"left": "table_right", "right": "table_left"}
    path = tmp_path / "model_contract.json"
    path.write_text(json.dumps(changed))
    with pytest.raises(ValueError, match="contract mismatch"):
        OnnxActor(actor.model_path, path)


def test_pt_jit_onnx_actor_and_pt_onnx_cbf_parity():
    torch = pytest.importorskip("torch")
    actor, barrier = models()
    actor_pt, checkpoint = rebuild_torch_actor(ROOT / "models/model_190.pt")
    actor_jit = torch.jit.load(str(ROOT / "models/v7_planner_actor_jit.pt"), map_location="cpu").eval()
    barrier_pt, cbf_checkpoint = rebuild_torch_barrier(ROOT / "models/frozen_cbf_v7.pt")
    assert checkpoint["iter"] == 190
    assert cbf_checkpoint["calibrated_threshold"] == pytest.approx(0.3027352380752564)
    rng = np.random.default_rng(190)
    maximum = 0.0
    for batch in (1, 7, 32):
        observation = rng.normal(size=(batch, 29)).astype(np.float32)
        safe = rng.normal(size=(batch, 87)).astype(np.float32)
        action = rng.uniform(-1, 1, size=(batch, 2)).astype(np.float32)
        with torch.no_grad():
            pt_actor = actor_pt(torch.from_numpy(observation)).numpy()
            jit_actor = actor_jit(torch.from_numpy(observation)).numpy()
            pt_cbf = tuple(value.numpy() for value in barrier_pt(torch.from_numpy(safe), torch.from_numpy(action)))
        onnx_actor = actor(observation)
        onnx_cbf = barrier(safe, action)
        maximum = max(maximum, float(np.max(np.abs(pt_actor - jit_actor))), float(np.max(np.abs(pt_actor - onnx_actor))))
        for reference, name in zip(pt_cbf, ("head_probabilities", "member_risk", "conservative_risk")):
            maximum = max(maximum, float(np.max(np.abs(reference - onnx_cbf[name]))))
    assert maximum <= 2.0e-5


def test_actor_and_barrier_reject_bad_or_nonfinite_inputs():
    actor, barrier = models()
    with pytest.raises(ValueError):
        actor(np.zeros((2, 28), np.float32))
    bad = np.zeros((1, 29), np.float32)
    bad[0, 4] = np.nan
    with pytest.raises(ValueError):
        actor(bad)
    with pytest.raises(ValueError):
        barrier(np.zeros((1, 86)), np.zeros((1, 2)))


def test_nominal_passthrough_and_hitter_priority_projection():
    state = np.zeros(87, np.float32)
    direct = project_v7_pair(np.asarray([0.6, -0.2]), state, ConstantBarrier(0.1), "left")
    assert direct["valid"] and not direct["cbf_intervened"] and not direct["hard_guard_intervened"]
    np.testing.assert_allclose(direct["applied_pair_m"], [0.54, -0.18], atol=1e-6)

    left = project_v7_pair(np.asarray([0.2, 0.1]), state, ConstantBarrier(0.1), "left")
    right = project_v7_pair(np.asarray([-0.1, -0.2]), state, ConstantBarrier(0.1), "right")
    assert left["valid"] and right["valid"]
    assert left["cbf_candidate_count"] == right["cbf_candidate_count"] == 2603
    assert left["cbf_candidate_radius_normalized"] == 1.25
    assert abs(left["cbf_selected_normalized"][0] - 0.2) < 1.0e-6
    assert abs(right["cbf_selected_normalized"][1] + 0.2) < 1.0e-6
    assert left["applied_pair_m"][0] - left["applied_pair_m"][1] >= 0.5 - 1e-6
    assert right["applied_pair_m"][0] - right["applied_pair_m"][1] >= 0.5 - 1e-6


def test_no_safe_candidate_falls_home_and_marks_shot_invalid():
    result = project_v7_pair([0.0, 0.0], np.zeros(87), ConstantBarrier(0.9), "left")
    assert not result["valid"]
    assert result["reason"] == "cbf_no_safe_candidate"
    assert result["cbf_fallback"] is True
    np.testing.assert_allclose(result["cbf_projected_m"], [0.35, -0.35], atol=1e-6)


def test_independent_hard_guard_keeps_hitter_and_enforces_exact_gap():
    left, changed, error = apply_hard_guard([0.2, 0.1], "left")
    assert changed and error is None
    np.testing.assert_allclose(left, [0.2, -0.3], atol=1e-6)
    right, changed, error = apply_hard_guard([-0.1, -0.2], "right")
    assert changed and error is None
    np.testing.assert_allclose(right, [0.3, -0.2], atol=1e-6)


def test_real_full_2603_candidate_cpu_budget():
    _, barrier = models()
    state = np.zeros(87, np.float32)
    latencies = []
    for _ in range(35):
        started = time.perf_counter()
        result = project_v7_pair([0.0, 0.0], state, barrier, "left")
        latencies.append((time.perf_counter() - started) * 1000.0)
        assert result["cbf_candidate_count"] == 2603
    assert float(np.percentile(latencies[5:], 95)) <= 20.0
    assert max(latencies[5:]) <= 50.0
