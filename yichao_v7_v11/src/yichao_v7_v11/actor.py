"""Fail-closed ONNX loaders for the V7 actor and frozen CBF ensemble."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort

from . import (
    CBF_CANDIDATE_RADIUS_NORMALIZED,
    CBF_GRID_POINTS,
    CBF_HITTER_WEIGHT,
    CBF_NON_HITTER_WEIGHT,
    CBF_RISK_THRESHOLD,
    HANDOFF_COMMIT,
    MODEL_ORDER,
    MODEL_TO_ROS,
    PLANNER_IDENTITY,
    TRAINING_AUDIT_COMMIT,
    WORKSPACE_Y,
)
from .features import ACTOR_NAMES, SAFE_NAMES


def sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def session_options() -> ort.SessionOptions:
    options = ort.SessionOptions()
    # Four intra-op workers keep the 2603-candidate batch below budget on the
    # workstation while avoiding the unbounded pool multiplication caused by
    # one default-sized pool per short-lived validation session.
    options.intra_op_num_threads = 4
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    return options


def load_contract(path: str | Path) -> dict[str, Any]:
    contract_path = Path(path)
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    bundle_assets = {
        "v7_planner_actor.onnx": "c79054c4f0a64b6fc64ad44b2308c97e7fe8854a27feecb81845f2e97f323a7f",
        "v7_planner_actor_jit.pt": "3197926d5f4cd60fff0d5b3da11e7e273849c470572920a9da8dec36f6a5ff5c",
        "model_190.pt": "c9a568ee4ed60565578654b9999c3a22ff14d964fdb3ec994a1b8f075022088c",
        "frozen_v7_barrier_risk.onnx": "77f68d863abd4531cd63fbf31d8052d448e37309cfefb436439cb8264d505e54",
        "frozen_cbf_v7.pt": "1fa806cb071853f13aca25264c5c66934f3210db7020af901de6a8ce3359064d",
        "handoff_schema.json": "e72739b1d1157e6debbe9537d5fa740473e259e93b5ec485d061887ee7195e4a",
        "runner_config.json": "08dd7689f5a4fcb7454e0ff6db56c6bfe37d58a45d34656fe537206538787e80",
    }
    expected = {
        "format": "yichao-v7-model190-frozen-cbf-onnx-v1",
        "planner_identity": PLANNER_IDENTITY,
        "handoff_commit": HANDOFF_COMMIT,
        "training_audit_commit": TRAINING_AUDIT_COMMIT,
        "model_order": list(MODEL_ORDER),
        "ros_mapping": dict(MODEL_TO_ROS),
        "actor_input_dim": 29,
        "actor_output_dim": 2,
        "safe_input_dim": 87,
        "actor_observation_schema_hash": "6f635a3cc33ef827b820f40e27087868d8af60f081ab2f835c4a94a9a26a1cfc",
        "safe_observation_schema_hash": "29bcbcd34b8b41c9720381488d9b70615bce63a9e284dcdacba5786c4652cc2d",
        "action_schema_hash": "0c8a9f35a758e664d616af0d94a622c414fb3257c31585698d124c1c23838330",
        "action_names": ["left_base_target_y", "right_base_target_y"],
        "action_bounds_m": list(WORKSPACE_Y),
        "actor_onnx_sha256": "c79054c4f0a64b6fc64ad44b2308c97e7fe8854a27feecb81845f2e97f323a7f",
        "actor_jit_sha256": "3197926d5f4cd60fff0d5b3da11e7e273849c470572920a9da8dec36f6a5ff5c",
        "model190_sha256": "c9a568ee4ed60565578654b9999c3a22ff14d964fdb3ec994a1b8f075022088c",
        "barrier_onnx_sha256": "77f68d863abd4531cd63fbf31d8052d448e37309cfefb436439cb8264d505e54",
        "barrier_checkpoint_sha256": "1fa806cb071853f13aca25264c5c66934f3210db7020af901de6a8ce3359064d",
        "handoff_schema_sha256": "e72739b1d1157e6debbe9537d5fa740473e259e93b5ec485d061887ee7195e4a",
        "risk_threshold": CBF_RISK_THRESHOLD,
        "candidate_radius_normalized": CBF_CANDIDATE_RADIUS_NORMALIZED,
        "candidate_grid_points": CBF_GRID_POINTS,
        "candidate_count_full": CBF_GRID_POINTS * CBF_GRID_POINTS + 2,
        "projection_mode": "hitter-priority",
        "hitter_weight": CBF_HITTER_WEIGHT,
        "non_hitter_weight": CBF_NON_HITTER_WEIGHT,
        "batch_dynamic": True,
        "bundle_assets": bundle_assets,
    }
    wrong = [
        f"{key}={contract.get(key)!r}" for key, value in expected.items()
        if contract.get(key) != value
    ]
    parity = contract.get("parity") or {}
    if parity.get("threshold") != 2.0e-5 or parity.get("passed") is not True:
        wrong.append("parity")
    maximum = parity.get("maximum_errors") or {}
    if not maximum or any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not np.isfinite(value)
        or value > 2.0e-5
        for value in maximum.values()
    ):
        wrong.append("parity.maximum_errors")
    if wrong:
        raise ValueError("V7 model contract mismatch: " + ", ".join(wrong))
    for name, digest in bundle_assets.items():
        asset = contract_path.parent / name
        if not asset.is_file() or sha256(asset) != digest:
            raise ValueError(f"V7 bundle asset mismatch: {name}")
    schema = json.loads((contract_path.parent / "handoff_schema.json").read_text(encoding="utf-8"))
    if schema.get("observation_names") != list(ACTOR_NAMES):
        raise ValueError("V7 actor field order differs from handoff schema")
    if schema.get("cbf_input_names") != list(SAFE_NAMES):
        raise ValueError("V7 CBF field order differs from handoff schema")
    return contract


class OnnxActor:
    def __init__(self, model_path: str | Path, contract_path: str | Path) -> None:
        self.model_path = Path(model_path)
        self.contract = load_contract(contract_path)
        if sha256(self.model_path) != self.contract["actor_onnx_sha256"]:
            raise ValueError("V7 actor ONNX SHA256 mismatch")
        self.session = ort.InferenceSession(
            str(self.model_path), session_options(), providers=["CPUExecutionProvider"]
        )
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (
            len(inputs) != 1
            or inputs[0].name != "observation"
            or inputs[0].shape[-1] != 29
            or isinstance(inputs[0].shape[0], int)
            or len(outputs) != 1
            or outputs[0].name != "action"
            or outputs[0].shape[-1] != 2
            or isinstance(outputs[0].shape[0], int)
        ):
            raise ValueError("V7 actor ONNX I/O contract mismatch")

    def __call__(self, observation: Any) -> np.ndarray:
        value = np.asarray(observation, dtype=np.float32)
        if value.ndim == 1:
            value = value[None, :]
        if value.ndim != 2 or value.shape[1] != 29 or not np.isfinite(value).all():
            raise ValueError("observation must have finite shape [N,29]")
        result = self.session.run(["action"], {"observation": value})[0]
        if result.shape != (len(value), 2) or not np.isfinite(result).all():
            raise ValueError("V7 actor returned an invalid batch")
        return np.asarray(result, dtype=np.float32)


class OnnxBarrier:
    """Three-member, three-head risk scorer exported from frozen_cbf_v7.pt."""

    def __init__(self, model_path: str | Path, contract_path: str | Path) -> None:
        self.model_path = Path(model_path)
        self.contract = load_contract(contract_path)
        if sha256(self.model_path) != self.contract["barrier_onnx_sha256"]:
            raise ValueError("V7 barrier ONNX SHA256 mismatch")
        self.session = ort.InferenceSession(
            str(self.model_path), session_options(), providers=["CPUExecutionProvider"]
        )
        inputs = [(item.name, item.shape) for item in self.session.get_inputs()]
        outputs = [(item.name, item.shape) for item in self.session.get_outputs()]
        if (
            inputs != [
                ("safe_observation", ["batch", 87]),
                ("candidate_action", ["batch", 2]),
            ]
            or outputs != [
                ("head_probabilities", ["batch", 3, 3]),
                ("member_risk", ["batch", 3]),
                ("conservative_risk", ["batch"]),
            ]
        ):
            raise ValueError("V7 barrier ONNX I/O contract mismatch")

    def __call__(self, safe_observation: Any, candidate_action: Any) -> dict[str, np.ndarray]:
        state = np.asarray(safe_observation, dtype=np.float32)
        action = np.asarray(candidate_action, dtype=np.float32)
        if state.ndim == 1:
            state = state[None, :]
        if action.ndim == 1:
            action = action[None, :]
        if (
            state.ndim != 2
            or action.ndim != 2
            or state.shape != (len(action), 87)
            or action.shape[1] != 2
            or not np.isfinite(state).all()
            or not np.isfinite(action).all()
        ):
            raise ValueError("barrier inputs must have finite shapes [N,87] and [N,2]")
        head, member, conservative = self.session.run(
            ["head_probabilities", "member_risk", "conservative_risk"],
            {"safe_observation": state, "candidate_action": np.clip(action, -1.0, 1.0)},
        )
        if (
            head.shape != (len(action), 3, 3)
            or member.shape != (len(action), 3)
            or conservative.shape != (len(action),)
            or not all(np.isfinite(value).all() for value in (head, member, conservative))
        ):
            raise ValueError("V7 barrier returned invalid outputs")
        return {
            "head_probabilities": np.asarray(head, dtype=np.float32),
            "member_risk": np.asarray(member, dtype=np.float32),
            "conservative_risk": np.asarray(conservative, dtype=np.float32),
        }


def rebuild_torch_actor(checkpoint_path: str | Path):
    """Rebuild the deterministic mean network from model_190.pt for parity."""

    import torch

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    state = checkpoint.get("actor_state_dict") or {}
    expected = {
        "obs_normalizer._mean": (1, 29),
        "obs_normalizer._std": (1, 29),
        "mlp.0.weight": (256, 29),
        "mlp.0.bias": (256,),
        "mlp.2.weight": (256, 256),
        "mlp.2.bias": (256,),
        "mlp.4.weight": (2, 256),
        "mlp.4.bias": (2,),
    }
    if checkpoint.get("iter") != 190:
        raise ValueError("model_190 checkpoint update mismatch")
    if any(name not in state or tuple(state[name].shape) != shape for name, shape in expected.items()):
        raise ValueError("model_190 actor state_dict ABI mismatch")
    if any(not torch.isfinite(value).all() for value in state.values() if torch.is_tensor(value)):
        raise ValueError("model_190 actor state_dict contains non-finite values")

    class Actor(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.register_buffer("mean", state["obs_normalizer._mean"].clone())
            self.register_buffer("std", state["obs_normalizer._std"].clone())
            self.layers = torch.nn.Sequential(
                torch.nn.Linear(29, 256), torch.nn.ELU(),
                torch.nn.Linear(256, 256), torch.nn.ELU(),
                torch.nn.Linear(256, 2),
            )
            for source in (0, 2, 4):
                self.layers[source].weight.data.copy_(state[f"mlp.{source}.weight"])
                self.layers[source].bias.data.copy_(state[f"mlp.{source}.bias"])

        def forward(self, observation):
            return self.layers((observation - self.mean) / (self.std + 0.01))

    return Actor().eval().requires_grad_(False), checkpoint


def rebuild_torch_barrier(checkpoint_path: str | Path):
    """Rebuild the three-member frozen scorer from frozen_cbf_v7.pt."""

    import torch

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    expected = {
        "format": "doubles-v7-nominal-execution-safety-v1",
        "state_dim": 87,
        "action_dim": 2,
        "output_dim": 3,
        "hidden_dims": (256, 256),
        "calibrated_threshold": CBF_RISK_THRESHOLD,
    }
    if any(checkpoint.get(name) != value for name, value in expected.items()):
        raise ValueError("frozen_cbf_v7 checkpoint ABI mismatch")
    networks = []
    for state in checkpoint.get("model_state_dicts", ()):
        network = torch.nn.Sequential(
            torch.nn.Linear(89, 256), torch.nn.SiLU(),
            torch.nn.Linear(256, 256), torch.nn.SiLU(),
            torch.nn.Linear(256, 3),
        )
        network.load_state_dict(state, strict=True)
        networks.append(network.eval().requires_grad_(False))
    if len(networks) != 3:
        raise ValueError("frozen_cbf_v7 ensemble size mismatch")
    if any(
        not torch.isfinite(value).all()
        for state in checkpoint["model_state_dicts"]
        for value in state.values()
        if torch.is_tensor(value)
    ):
        raise ValueError("frozen_cbf_v7 state_dict contains non-finite values")

    class Barrier(torch.nn.Module):
        def __init__(self):
            super().__init__()
            self.networks = torch.nn.ModuleList(networks)

        def forward(self, safe_observation, candidate_action):
            features = torch.cat((safe_observation, candidate_action.clamp(-1.0, 1.0)), dim=1)
            head = torch.stack([torch.sigmoid(model(features)) for model in self.networks], dim=1)
            member = head.amax(dim=2)
            conservative = (member.mean(dim=1) + member.std(dim=1, unbiased=False)).clamp(0.0, 1.0)
            return head, member, conservative

    return Barrier().eval(), checkpoint


__all__ = [
    "OnnxActor", "OnnxBarrier", "load_contract", "rebuild_torch_actor",
    "rebuild_torch_barrier", "sha256",
]
