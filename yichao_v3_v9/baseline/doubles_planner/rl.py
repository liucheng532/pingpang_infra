from __future__ import annotations

import hashlib
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np


BALL_Y_EDGES = np.linspace(-0.80, 0.80, 8, dtype=np.float32)
BASE_Y_EDGES = np.linspace(-0.75, 0.75, 7, dtype=np.float32)
TIME_TO_STRIKE_EDGES = np.asarray([0.30, 0.44, 0.58], dtype=np.float32)
RELATIVE_VELOCITY_EDGES = np.asarray([-0.90, -0.30, 0.30, 0.90], dtype=np.float32)
TEAMMATE_GOALS_Y = np.linspace(-0.84, 0.84, 9, dtype=np.float32)
DEFAULT_POLICY_PATH = (
    Path(__file__).resolve().parents[1]
    / "research_results"
    / "doubles_rl_policy.npz"
)
POLICY_HASH_KEY = "policy_sha256"


def _bin(value: float, edges: np.ndarray) -> int:
    if not np.isfinite(value):
        raise ValueError("RL state values must be finite")
    return int(np.digitize(float(value), edges, right=False))


def encode_state(
    hitter_index: int,
    ball_y: float,
    left_y: float,
    right_y: float,
    time_to_strike_s: float,
    relative_velocity_mps: float,
    *,
    ball_edges: np.ndarray = BALL_Y_EDGES,
    base_edges: np.ndarray = BASE_Y_EDGES,
    time_edges: np.ndarray = TIME_TO_STRIKE_EDGES,
    relative_velocity_edges: np.ndarray = RELATIVE_VELOCITY_EDGES,
) -> tuple[int, int, int, int, int, int]:
    if hitter_index not in {0, 1}:
        raise ValueError("hitter_index must be 0 or 1")
    return (
        hitter_index,
        _bin(ball_y, ball_edges),
        _bin(left_y, base_edges),
        _bin(right_y, base_edges),
        _bin(time_to_strike_s, time_edges),
        _bin(relative_velocity_mps, relative_velocity_edges),
    )


@dataclass(frozen=True)
class FrozenQPolicy:
    q_values: np.ndarray
    state_visits: np.ndarray
    action_goals_y: np.ndarray
    ball_edges: np.ndarray
    base_edges: np.ndarray
    time_edges: np.ndarray
    relative_velocity_edges: np.ndarray
    metadata: dict[str, object]

    def __post_init__(self) -> None:
        state_shape = (
            2,
            len(self.ball_edges) + 1,
            len(self.base_edges) + 1,
            len(self.base_edges) + 1,
            len(self.time_edges) + 1,
            len(self.relative_velocity_edges) + 1,
        )
        if self.state_visits.shape != state_shape:
            raise ValueError(
                f"state_visits shape must be {state_shape}, got {self.state_visits.shape}"
            )
        if self.q_values.shape != state_shape + (len(self.action_goals_y),):
            raise ValueError("q_values shape does not match state bins and action goals")
        if not np.isfinite(self.q_values).all() or not np.isfinite(self.action_goals_y).all():
            raise ValueError("RL policy arrays must be finite")

    def state_index(
        self,
        hitter_index: int,
        ball_y: float,
        left_y: float,
        right_y: float,
        time_to_strike_s: float,
        relative_velocity_mps: float,
    ) -> tuple[int, int, int, int, int, int]:
        return encode_state(
            hitter_index,
            ball_y,
            left_y,
            right_y,
            time_to_strike_s,
            relative_velocity_mps,
            ball_edges=self.ball_edges,
            base_edges=self.base_edges,
            time_edges=self.time_edges,
            relative_velocity_edges=self.relative_velocity_edges,
        )

    def choose_goal(
        self,
        hitter_index: int,
        ball_y: float,
        left_y: float,
        right_y: float,
        time_to_strike_s: float,
        relative_velocity_mps: float,
        minimum_visits: int = 1,
    ) -> tuple[float | None, dict[str, float | int | bool]]:
        state = self.state_index(
            hitter_index,
            ball_y,
            left_y,
            right_y,
            time_to_strike_s,
            relative_velocity_mps,
        )
        visits = int(self.state_visits[state])
        if visits < minimum_visits:
            return None, {
                "rl_state_visits": visits,
                "rl_state_covered": False,
                "rl_action_index": -1,
                "rl_q_value": 0.0,
            }
        values = self.q_values[state]
        action_index = int(np.argmax(values))
        return float(self.action_goals_y[action_index]), {
            "rl_state_visits": visits,
            "rl_state_covered": True,
            "rl_action_index": action_index,
            "rl_q_value": float(values[action_index]),
        }


def load_q_policy(path: str | os.PathLike[str] | None = None) -> FrozenQPolicy:
    if path is None:
        path = os.environ.get("PINGPANG_RL_POLICY", str(DEFAULT_POLICY_PATH))
    policy_path = Path(path).expanduser().resolve()
    if not policy_path.is_file():
        raise FileNotFoundError(
            f"frozen doubles RL policy is missing: {policy_path}; provide a validated "
            "policy artifact before enabling the RL strategy"
        )
    with np.load(policy_path, allow_pickle=False) as payload:
        metadata_text = str(payload["metadata_json"].item())
        metadata = json.loads(metadata_text)
        policy = FrozenQPolicy(
            q_values=np.asarray(payload["q_values"], dtype=np.float32),
            state_visits=np.asarray(payload["state_visits"], dtype=np.int64),
            action_goals_y=np.asarray(payload["action_goals_y"], dtype=np.float32),
            ball_edges=np.asarray(payload["ball_edges"], dtype=np.float32),
            base_edges=np.asarray(payload["base_edges"], dtype=np.float32),
            time_edges=np.asarray(payload["time_edges"], dtype=np.float32),
            relative_velocity_edges=np.asarray(
                payload["relative_velocity_edges"], dtype=np.float32
            ),
            metadata=metadata,
        )
    recorded_hash = policy.metadata.get(POLICY_HASH_KEY)
    if not isinstance(recorded_hash, str) or not recorded_hash:
        raise ValueError(
            f"RL policy metadata is missing required {POLICY_HASH_KEY}"
        )
    actual_hash = policy_content_sha256(
        policy.q_values,
        policy.state_visits,
        metadata=policy.metadata,
        action_goals_y=policy.action_goals_y,
        ball_edges=policy.ball_edges,
        base_edges=policy.base_edges,
        time_edges=policy.time_edges,
        relative_velocity_edges=policy.relative_velocity_edges,
    )
    if recorded_hash != actual_hash:
        raise ValueError(
            f"RL policy content hash mismatch: expected {recorded_hash}, got {actual_hash}"
        )
    return policy


def empty_policy_arrays() -> tuple[np.ndarray, np.ndarray]:
    state_shape = (
        2,
        len(BALL_Y_EDGES) + 1,
        len(BASE_Y_EDGES) + 1,
        len(BASE_Y_EDGES) + 1,
        len(TIME_TO_STRIKE_EDGES) + 1,
        len(RELATIVE_VELOCITY_EDGES) + 1,
    )
    return (
        np.zeros(state_shape + (len(TEAMMATE_GOALS_Y),), dtype=np.float32),
        np.zeros(state_shape, dtype=np.int64),
    )


def policy_content_sha256(
    q_values: np.ndarray,
    state_visits: np.ndarray,
    metadata: dict[str, object] | None = None,
    *,
    action_goals_y: np.ndarray = TEAMMATE_GOALS_Y,
    ball_edges: np.ndarray = BALL_Y_EDGES,
    base_edges: np.ndarray = BASE_Y_EDGES,
    time_edges: np.ndarray = TIME_TO_STRIKE_EDGES,
    relative_velocity_edges: np.ndarray = RELATIVE_VELOCITY_EDGES,
) -> str:
    digest = hashlib.sha256(b"pingpang-doubles-tabular-q-v1\0")
    arrays = (
        ("q_values", np.asarray(q_values, dtype="<f4")),
        ("state_visits", np.asarray(state_visits, dtype="<i8")),
        ("action_goals_y", np.asarray(action_goals_y, dtype="<f4")),
        ("ball_edges", np.asarray(ball_edges, dtype="<f4")),
        ("base_edges", np.asarray(base_edges, dtype="<f4")),
        ("time_edges", np.asarray(time_edges, dtype="<f4")),
        (
            "relative_velocity_edges",
            np.asarray(relative_velocity_edges, dtype="<f4"),
        ),
    )
    for name, array in arrays:
        contiguous = np.ascontiguousarray(array)
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(contiguous.shape).encode("ascii"))
        digest.update(b"\0")
        digest.update(contiguous.dtype.str.encode("ascii"))
        digest.update(b"\0")
        digest.update(contiguous.tobytes(order="C"))

    hash_metadata = dict(metadata or {})
    hash_metadata.pop(POLICY_HASH_KEY, None)
    digest.update(
        json.dumps(
            hash_metadata,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    )
    return digest.hexdigest()


def save_q_policy(
    path: str | os.PathLike[str],
    q_values: np.ndarray,
    state_visits: np.ndarray,
    metadata: dict[str, object],
) -> Path:
    policy_path = Path(path).expanduser().resolve()
    policy_path.parent.mkdir(parents=True, exist_ok=True)
    saved_metadata = dict(metadata)
    expected_hash = policy_content_sha256(q_values, state_visits, saved_metadata)
    supplied_hash = saved_metadata.get(POLICY_HASH_KEY)
    if supplied_hash is not None and supplied_hash != expected_hash:
        raise ValueError(
            f"supplied RL policy content hash is stale: {supplied_hash} != {expected_hash}"
        )
    saved_metadata[POLICY_HASH_KEY] = expected_hash
    np.savez_compressed(
        policy_path,
        q_values=np.asarray(q_values, dtype=np.float32),
        state_visits=np.asarray(state_visits, dtype=np.int64),
        action_goals_y=TEAMMATE_GOALS_Y,
        ball_edges=BALL_Y_EDGES,
        base_edges=BASE_Y_EDGES,
        time_edges=TIME_TO_STRIKE_EDGES,
        relative_velocity_edges=RELATIVE_VELOCITY_EDGES,
        metadata_json=np.asarray(
            json.dumps(saved_metadata, allow_nan=False, sort_keys=True)
        ),
    )
    load_q_policy(policy_path)
    return policy_path


__all__ = [
    "BALL_Y_EDGES",
    "BASE_Y_EDGES",
    "DEFAULT_POLICY_PATH",
    "FrozenQPolicy",
    "POLICY_HASH_KEY",
    "RELATIVE_VELOCITY_EDGES",
    "TEAMMATE_GOALS_Y",
    "TIME_TO_STRIKE_EDGES",
    "empty_policy_arrays",
    "encode_state",
    "load_q_policy",
    "policy_content_sha256",
    "save_q_policy",
]
