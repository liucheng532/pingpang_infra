"""V11 asset/process attestation consumed by non-read-only planner modes."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Mapping

from . import PLANNER_IDENTITY

ATTESTATION_SCHEMA = "v7-v11-process-attestation-v1"
DEPLOYMENT_CONTRACT_SHA256 = "59a64597f92d9b824615fa356eb0d05b9a6f3999dddff7d40a60b58646824eed"


def file_sha256(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_deployment_contract(deployment: Mapping[str, Any]) -> None:
    """Reject any semantic change to the frozen workstation/V11 contract."""

    encoded = json.dumps(
        deployment, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    if hashlib.sha256(encoded).hexdigest() != DEPLOYMENT_CONTRACT_SHA256:
        raise ValueError("V7/V11 deployment contract mismatch")


def verify_attestation(
    path: str | Path,
    *,
    expected_controller_mode: str,
    deployment: Mapping[str, Any],
    now: float | None = None,
    maximum_age_s: float = 120.0,
) -> dict[str, Any]:
    if expected_controller_mode not in {"shadow", "active"}:
        raise ValueError("expected controller mode must be shadow or active")
    validate_deployment_contract(deployment)
    record = json.loads(Path(path).read_text(encoding="utf-8"))
    expected = {
        "schema": ATTESTATION_SCHEMA,
        "planner_identity": PLANNER_IDENTITY,
        "controller_mode": expected_controller_mode,
        "profile": "normal",
        "residual_mode": "off",
        "residual_scale": 1.0,
        "residual_family": "v11-teacher",
        "live_processes_verified": True,
        "launcher": deployment.get("original_v11_launcher"),
        "launcher_sha256": deployment.get("original_v11_launcher_sha256"),
    }
    mismatches = [
        f"{key}={record.get(key)!r}"
        for key, value in expected.items()
        if record.get(key) != value
    ]
    created = record.get("created_at")
    current = time.time() if now is None else float(now)
    if (
        isinstance(created, bool)
        or not isinstance(created, (int, float))
        or not math.isfinite(created)
        or not 0.0 <= current - created <= maximum_age_s
    ):
        mismatches.append("attestation_age")
    robots = record.get("robots")
    expected_robots = deployment.get("robots", {})
    if not isinstance(robots, dict) or set(robots) != set(expected_robots):
        mismatches.append("robot_set")
    else:
        for robot, config in expected_robots.items():
            item = robots[robot]
            for key in ("boot_id", "g1_control", "policy"):
                if not item.get(key):
                    mismatches.append(f"{robot}.{key}")
            if item.get("host_label") != config.get("host_label"):
                mismatches.append(f"{robot}.host_label")
            if item.get("assets") != deployment.get("controller_assets"):
                mismatches.append(f"{robot}.assets")
            if item.get("external_assets") != deployment.get("external_controller_assets"):
                mismatches.append(f"{robot}.external_assets")
            contract = item.get("embedded_controller_contract")
            if contract != deployment.get("embedded_controller_contract"):
                mismatches.append(f"{robot}.embedded_controller_contract")
            policy = item.get("policy") or {}
            argv = policy.get("argv", "")
            if "--hit-arm7-residual-mode off" not in argv:
                mismatches.append(f"{robot}.residual_argv")
            has_shadow = "--shadow" in argv.split()
            if has_shadow != (expected_controller_mode == "shadow"):
                mismatches.append(f"{robot}.controller_mode_argv")
            for process_name in ("g1_control", "policy"):
                process = item.get(process_name) or {}
                if not isinstance(process.get("pid"), int) or process["pid"] <= 1:
                    mismatches.append(f"{robot}.{process_name}.pid")
                if not isinstance(process.get("starttime"), int) or process["starttime"] <= 0:
                    mismatches.append(f"{robot}.{process_name}.starttime")
                for field in ("argv", "cwd", "exe", "comm"):
                    if not process.get(field):
                        mismatches.append(f"{robot}.{process_name}.{field}")
            native = item.get("g1_control") or {}
            expected_lcm = (
                f"udpm://239.255.76.{config.get('host_label')}:7667?ttl=0"
            )
            if (native.get("environment") or {}).get("LCM_DEFAULT_URL") != expected_lcm:
                mismatches.append(f"{robot}.g1_control.LCM_DEFAULT_URL")
    if mismatches:
        raise ValueError("V11 attestation mismatch: " + ", ".join(mismatches))
    return record


__all__ = [
    "ATTESTATION_SCHEMA", "DEPLOYMENT_CONTRACT_SHA256", "file_sha256",
    "validate_deployment_contract", "verify_attestation",
]
