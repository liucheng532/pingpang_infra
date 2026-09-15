"""V7 learned-CBF candidate projection plus an independent 0.50 m guard."""

from __future__ import annotations

import time
from typing import Any

import numpy as np

from . import (
    CBF_CANDIDATE_RADIUS_NORMALIZED,
    CBF_GRID_POINTS,
    CBF_HITTER_WEIGHT,
    CBF_NON_HITTER_WEIGHT,
    CBF_RISK_THRESHOLD,
    HOME_PAIR,
    MINIMUM_GAP_M,
    MODEL_ORDER,
    WORKSPACE_Y,
)


def _list(value: np.ndarray | None) -> list[Any] | None:
    return None if value is None else np.asarray(value).astype(float).tolist()


def apply_hard_guard(pair_m: Any, hitter: str) -> tuple[np.ndarray | None, bool, str | None]:
    """Preserve the hitter target and move only its peer to restore the gap."""

    if hitter not in MODEL_ORDER:
        return None, False, "unknown_hitter"
    try:
        pair = np.asarray(pair_m, dtype=np.float32)
    except (TypeError, ValueError):
        return None, False, "hard_guard_input_not_numeric"
    if pair.shape != (2,) or not np.isfinite(pair).all():
        return None, False, "hard_guard_input_invalid"
    projected = np.clip(pair, *WORKSPACE_Y)
    intervened = bool(np.max(np.abs(projected - pair)) > 1.0e-7)
    if projected[0] - projected[1] < MINIMUM_GAP_M - 1.0e-7:
        intervened = True
        if hitter == "left":
            projected[1] = projected[0] - np.float32(MINIMUM_GAP_M)
        else:
            projected[0] = projected[1] + np.float32(MINIMUM_GAP_M)
    if (
        np.any(projected < WORKSPACE_Y[0] - 1.0e-7)
        or np.any(projected > WORKSPACE_Y[1] + 1.0e-7)
        or projected[0] - projected[1] < MINIMUM_GAP_M - 1.0e-6
    ):
        return None, intervened, "hard_guard_projection_unavailable"
    return projected.astype(np.float32), intervened, None


def project_v7_pair(raw_action: Any, safe_observation: Any, barrier: Any, hitter: str) -> dict[str, Any]:
    """Apply the exact normalized V7 grid and then the independent meter guard."""

    started = time.perf_counter()
    try:
        raw = np.asarray(raw_action, dtype=np.float32)
    except (TypeError, ValueError):
        raw = np.asarray([], dtype=np.float32)
    base = {
        "valid": False,
        "reason": None,
        "actor_raw": _list(raw.reshape(-1)),
        "actor_clipped_normalized": None,
        "actor_physical_m": None,
        "cbf_nominal_risk": None,
        "cbf_selected_risk": None,
        "cbf_head_probabilities": None,
        "cbf_member_risk": None,
        "cbf_selected_normalized": None,
        "cbf_projected_m": None,
        "cbf_intervened": False,
        "cbf_fallback": False,
        "cbf_candidate_count": 0,
        "cbf_candidate_radius_normalized": CBF_CANDIDATE_RADIUS_NORMALIZED,
        "cbf_risk_threshold": CBF_RISK_THRESHOLD,
        "cbf_projection_mode": "hitter-priority",
        "cbf_deviation_weights": None,
        "guard_projected_m": None,
        "hard_guard_intervened": False,
        "guard_intervened": False,
        "applied_pair_m": None,
        "filter_mode": "frozen_v7_cbf",
        "latency_ms": 0.0,
    }
    if hitter not in MODEL_ORDER:
        base["reason"] = "unknown_hitter"
        return base
    if raw.shape != (2,):
        base["reason"] = "actor_output_wrong_shape"
        return base
    if not np.isfinite(raw).all():
        base["reason"] = "actor_output_nonfinite"
        return base
    safe_state = np.asarray(safe_observation, dtype=np.float32)
    if safe_state.shape != (87,) or not np.isfinite(safe_state).all():
        base["reason"] = "safe_observation_invalid"
        return base

    clipped = np.clip(raw, -1.0, 1.0).astype(np.float32)
    physical = clipped * np.float32(WORKSPACE_Y[1])
    weights = np.asarray(
        (CBF_HITTER_WEIGHT, CBF_NON_HITTER_WEIGHT)
        if hitter == "left"
        else (CBF_NON_HITTER_WEIGHT, CBF_HITTER_WEIGHT),
        dtype=np.float32,
    )
    base.update(
        actor_clipped_normalized=_list(clipped),
        actor_physical_m=_list(physical),
        cbf_deviation_weights=_list(weights),
    )

    nominal_scores = barrier(safe_state, clipped)
    nominal_risk = float(nominal_scores["conservative_risk"][0])
    minimum_normalized_gap = MINIMUM_GAP_M / WORKSPACE_Y[1]
    nominal_safe = (
        nominal_risk <= CBF_RISK_THRESHOLD
        and clipped[0] - clipped[1] >= minimum_normalized_gap - 1.0e-7
    )
    if nominal_safe:
        selected = clipped
        selected_index = 0
        scores = nominal_scores
        base["cbf_candidate_count"] = 1
    else:
        offsets = np.linspace(
            -CBF_CANDIDATE_RADIUS_NORMALIZED,
            CBF_CANDIDATE_RADIUS_NORMALIZED,
            CBF_GRID_POINTS,
            dtype=np.float32,
        )
        first, second = np.meshgrid(offsets, offsets, indexing="ij")
        grid = np.stack((first.reshape(-1), second.reshape(-1)), axis=1)
        candidates = np.clip(clipped[None, :] + grid, -1.0, 1.0)
        home = np.asarray(HOME_PAIR, dtype=np.float32) / np.float32(WORKSPACE_Y[1])
        candidates = np.concatenate((clipped[None, :], candidates, home[None, :]), axis=0)
        repeated = np.repeat(safe_state[None, :], len(candidates), axis=0)
        scores = barrier(repeated, candidates)
        risks = scores["conservative_risk"]
        gap_valid = candidates[:, 0] - candidates[:, 1] >= minimum_normalized_gap - 1.0e-7
        radius_valid = (
            np.max(np.abs(candidates - clipped[None, :]), axis=1)
            <= CBF_CANDIDATE_RADIUS_NORMALIZED + 1.0e-6
        )
        safe = gap_valid & radius_valid & (risks <= CBF_RISK_THRESHOLD)
        deviation = np.sum(np.square(candidates - clipped[None, :]) * weights[None, :], axis=1)
        ranking = np.where(safe, deviation, np.inf)
        found = bool(np.any(safe))
        selected_index = int(np.argmin(ranking)) if found else len(candidates) - 1
        selected = candidates[selected_index]
        base["cbf_fallback"] = not found
        base["cbf_candidate_count"] = len(candidates)
        if not found:
            base.update(
                reason="cbf_no_safe_candidate",
                cbf_nominal_risk=nominal_risk,
                cbf_selected_risk=float(risks[selected_index]),
                cbf_head_probabilities=_list(scores["head_probabilities"][selected_index]),
                cbf_member_risk=_list(scores["member_risk"][selected_index]),
                cbf_selected_normalized=_list(selected),
                cbf_projected_m=_list(selected * np.float32(WORKSPACE_Y[1])),
                latency_ms=(time.perf_counter() - started) * 1000.0,
            )
            return base

    selected_risk = float(scores["conservative_risk"][selected_index])
    selected_m = selected * np.float32(WORKSPACE_Y[1])
    guarded, hard_intervened, error = apply_hard_guard(selected_m, hitter)
    cbf_intervened = bool(np.max(np.abs(selected - clipped)) > 1.0e-6)
    base.update(
        cbf_nominal_risk=nominal_risk,
        cbf_selected_risk=selected_risk,
        cbf_head_probabilities=_list(scores["head_probabilities"][selected_index]),
        cbf_member_risk=_list(scores["member_risk"][selected_index]),
        cbf_selected_normalized=_list(selected),
        cbf_projected_m=_list(selected_m),
        cbf_intervened=cbf_intervened,
        guard_projected_m=_list(guarded),
        hard_guard_intervened=hard_intervened,
        guard_intervened=cbf_intervened or hard_intervened,
        latency_ms=(time.perf_counter() - started) * 1000.0,
    )
    if error is not None:
        base["reason"] = error
        return base
    base.update(valid=True, reason=None, applied_pair_m=_list(guarded))
    return base


__all__ = ["apply_hard_guard", "project_v7_pair"]
