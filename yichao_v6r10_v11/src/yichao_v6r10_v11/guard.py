"""Deterministic actor-only deployment guard.

The learned barrier, risk ensemble, and candidate grid are intentionally absent
from this module and from the production dependency graph.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from . import MINIMUM_GAP_M, MODEL_ORDER, WORKSPACE_Y


@dataclass(frozen=True)
class GuardResult:
    valid: bool
    reason: str | None
    raw: list[float] | None
    clipped: list[float] | None
    physical: list[float] | None
    projected: list[float] | None
    intervened: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid": self.valid,
            "reason": self.reason,
            "actor_raw": self.raw,
            "actor_clipped": self.clipped,
            "actor_physical_m": self.physical,
            "guard_projected_m": self.projected,
            "applied_pair_m": self.projected if self.valid else None,
            "guard_intervened": self.intervened,
            "filter_mode": "off",
            "barrier_risk": "not_evaluated",
            "ensemble_risk": "not_evaluated",
        }


def _invalid(reason: str, raw: np.ndarray | None = None) -> GuardResult:
    values = None if raw is None else raw.reshape(-1).astype(float).tolist()
    return GuardResult(False, reason, values, None, None, None, False)


def project_actor_pair(raw_action: Any, hitter: str) -> GuardResult:
    """Keep the scheduled hitter target and move only its peer outward."""

    if hitter not in MODEL_ORDER:
        return _invalid("unknown_hitter")
    try:
        raw = np.asarray(raw_action, dtype=np.float32)
    except (TypeError, ValueError):
        return _invalid("actor_output_not_numeric")
    if raw.shape != (2,):
        return _invalid("actor_output_wrong_shape", raw)
    if not np.isfinite(raw).all():
        return _invalid("actor_output_nonfinite", raw)

    clipped = np.clip(raw, -1.0, 1.0)
    physical = clipped * np.float32(WORKSPACE_Y[1])
    projected = physical.copy()
    intervened = bool(physical[0] - physical[1] < MINIMUM_GAP_M - 1.0e-7)
    if intervened:
        if hitter == "left":
            projected[1] = projected[0] - np.float32(MINIMUM_GAP_M)
        else:
            projected[0] = projected[1] + np.float32(MINIMUM_GAP_M)

    low, high = WORKSPACE_Y
    if (
        not np.isfinite(projected).all()
        or np.any(projected < low - 1.0e-7)
        or np.any(projected > high + 1.0e-7)
    ):
        result = _invalid("guard_projection_unavailable", raw)
        return GuardResult(
            result.valid,
            result.reason,
            result.raw,
            clipped.astype(float).tolist(),
            physical.astype(float).tolist(),
            None,
            intervened,
        )
    if projected[0] - projected[1] < MINIMUM_GAP_M - 1.0e-6:
        return GuardResult(
            False,
            "guard_postcondition_gap",
            raw.astype(float).tolist(),
            clipped.astype(float).tolist(),
            physical.astype(float).tolist(),
            projected.astype(float).tolist(),
            intervened,
        )
    return GuardResult(
        True,
        None,
        raw.astype(float).tolist(),
        clipped.astype(float).tolist(),
        physical.astype(float).tolist(),
        projected.astype(float).tolist(),
        intervened,
    )

