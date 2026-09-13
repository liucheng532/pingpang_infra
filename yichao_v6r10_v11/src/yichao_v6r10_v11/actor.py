"""Production ONNX actor and checkpoint reconstruction used by the exporter."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import onnxruntime as ort

from . import MODEL_ORDER, MODEL_TO_ROS, PLANNER_IDENTITY, WORKSPACE_Y
from .features import ACTOR_NAMES, AUDIT_NAMES


def sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class OnnxActor:
    def __init__(self, model_path: str | Path, sidecar_path: str | Path) -> None:
        self.model_path = Path(model_path)
        self.sidecar_path = Path(sidecar_path)
        self.sidecar = json.loads(self.sidecar_path.read_text(encoding="utf-8"))
        expected = {
            "format": "v6r10-model630-actor-onnx-v1",
            "planner_identity": PLANNER_IDENTITY,
            "input_name": "observation",
            "output_name": "action",
            "input_dim": 36,
            "output_dim": 2,
            "audit_dim": 87,
            "audit_schema_hash": "1eb855b26c4c7661d2385b34dc91d53a830b6ebfd01087d7cca2e2a53c7e82b2",
            "checkpoint_sha256": "81501373c08f6f4aa6bae3fdaccdd073939e5346d2b25a8e419ab35c6657ed0c",
            "schema_sha256": "f2bf9c51e37f675cdb6e063cab325c0bf0eb569bb46c0fb104c075c3e6ed7cee",
            "observation_version": "doubles-shot-observation-v1",
            "observation_schema_hash": "610b742594dca8d05bc0699a0573d3b5bad58b51c06996882778ef66dc7a47d6",
            "observation_names": list(ACTOR_NAMES),
            "audit_names": list(AUDIT_NAMES),
            "action_names": ["left_base_target_y", "right_base_target_y"],
            "action_bounds_m": list(WORKSPACE_Y),
            "model_order": list(MODEL_ORDER),
            "ros_mapping": dict(MODEL_TO_ROS),
            "network": {"hidden_dims": [256, 256], "activation": "elu"},
            "observation_normalization": {
                "frozen": True,
                "formula": "(observation - mean) / (std + 0.01)",
            },
            "barrier_dependency": None,
            "batch_dynamic": True,
            "filter_mode": "off",
        }
        wrong = [
            f"{key}={self.sidecar.get(key)!r}"
            for key, value in expected.items()
            if self.sidecar.get(key) != value
        ]
        if wrong:
            raise ValueError("actor sidecar mismatch: " + ", ".join(wrong))
        parity_threshold = self.sidecar.get("parity_threshold")
        parity_error = self.sidecar.get("parity_max_abs_error")
        if (
            parity_threshold != 2.0e-5
            or isinstance(parity_error, bool)
            or not isinstance(parity_error, (int, float))
            or not np.isfinite(parity_error)
            or parity_error > parity_threshold
        ):
            raise ValueError("actor sidecar parity contract mismatch")
        if sha256(self.model_path) != self.sidecar.get("onnx_sha256"):
            raise ValueError("actor ONNX SHA256 mismatch")
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(
            str(self.model_path), options, providers=["CPUExecutionProvider"]
        )
        input_meta = self.session.get_inputs()
        output_meta = self.session.get_outputs()
        if (
            len(input_meta) != 1
            or input_meta[0].name != "observation"
            or len(input_meta[0].shape) != 2
            or input_meta[0].shape[-1] != 36
            or isinstance(input_meta[0].shape[0], int)
            or len(output_meta) != 1
            or output_meta[0].name != "action"
            or len(output_meta[0].shape) != 2
            or output_meta[0].shape[-1] != 2
            or isinstance(output_meta[0].shape[0], int)
        ):
            raise ValueError("actor ONNX I/O contract mismatch")

    def __call__(self, observation: Any) -> np.ndarray:
        value = np.asarray(observation, dtype=np.float32)
        if value.ndim == 1:
            value = value[None, :]
        if value.ndim != 2 or value.shape[1] != 36 or not np.isfinite(value).all():
            raise ValueError("observation must have finite shape [N,36]")
        result = self.session.run(["action"], {"observation": value})[0]
        if result.shape != (value.shape[0], 2) or not np.isfinite(result).all():
            raise ValueError("actor returned an invalid batch")
        return np.asarray(result, dtype=np.float32)


def rebuild_torch_actor(checkpoint_path: str | Path):
    """Rebuild only the deterministic actor mean; critics/std are not loaded."""

    import torch

    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    state = checkpoint.get("actor_state_dict")
    if not isinstance(state, dict):
        raise ValueError("checkpoint has no actor_state_dict")
    expected_shapes = {
        "obs_normalizer._mean": (1, 36),
        "obs_normalizer._std": (1, 36),
        "mlp.0.weight": (256, 36),
        "mlp.0.bias": (256,),
        "mlp.2.weight": (256, 256),
        "mlp.2.bias": (256,),
        "mlp.4.weight": (2, 256),
        "mlp.4.bias": (2,),
    }
    for name, shape in expected_shapes.items():
        if name not in state or tuple(state[name].shape) != shape:
            raise ValueError(f"checkpoint actor tensor mismatch: {name}")

    class DeterministicActor(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.register_buffer("observation_mean", state["obs_normalizer._mean"].clone())
            self.register_buffer("observation_std", state["obs_normalizer._std"].clone())
            self.layers = torch.nn.Sequential(
                torch.nn.Linear(36, 256),
                torch.nn.ELU(),
                torch.nn.Linear(256, 256),
                torch.nn.ELU(),
                torch.nn.Linear(256, 2),
            )
            for source, destination in ((0, 0), (2, 2), (4, 4)):
                self.layers[destination].weight.data.copy_(state[f"mlp.{source}.weight"])
                self.layers[destination].bias.data.copy_(state[f"mlp.{source}.bias"])

        def forward(self, observation):
            normalized = (observation - self.observation_mean) / (
                self.observation_std + 0.01
            )
            return self.layers(normalized)

    return DeterministicActor().eval().requires_grad_(False), checkpoint
