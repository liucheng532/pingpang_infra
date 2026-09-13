"""Locate the unchanged Fixed runtime used for relay lifecycle semantics."""

from __future__ import annotations

import os
from pathlib import Path
import sys

from . import ROOT


def install_fixed_runtime(path: str | Path | None = None) -> Path:
    candidates = []
    if path is not None:
        candidates.append(Path(path))
    configured = os.environ.get("V6R10_FIXED_RUNTIME_ROOT")
    if configured:
        candidates.append(Path(configured))
    candidates.extend(
        (
            ROOT.parent / "yichao_v3_v9" / "baseline",
            Path("/home/odl/codebase/pingpang_doubles_v9_runtime"),
        )
    )
    for candidate in candidates:
        resolved = candidate.expanduser().resolve()
        if (resolved / "doubles_planner" / "fixed_relay.py").is_file():
            value = str(resolved)
            if value not in sys.path:
                sys.path.insert(0, value)
            return resolved
    raise RuntimeError(
        "unchanged Fixed runtime was not found; set V6R10_FIXED_RUNTIME_ROOT"
    )

