"""Planner-owned RETURN timing; no changes to the low-level robot protocol."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np


RETURN_TIMING_SCHEMA = "doubles-return-timing-v1"
RETURN_TIMING_TOPIC = "/doubles/return_timing_config"
TURNAROUND_HOLD_S = 0.20
PROFILE_PATH = Path(__file__).with_name("return_path_profile.json")


@dataclass(frozen=True)
class ReturnTimingConfig:
    robot_id: str
    mode: str = "strike_time"
    lead_ms: int = 100
    command_id: str | None = None
    published_at: float = 0.0

    def to_dict(self) -> dict[str, Any]:
        return {"schema": RETURN_TIMING_SCHEMA, **asdict(self)}


def parse_return_timing(payload: Mapping[str, Any], robots: tuple[str, ...]) -> ReturnTimingConfig:
    if not isinstance(payload, Mapping) or payload.get("schema") != RETURN_TIMING_SCHEMA:
        raise ValueError("wrong return timing schema")
    if payload.get("robot_id") not in robots:
        raise ValueError("unknown return timing robot_id")
    if payload.get("mode") not in ("strike_time", "peer_outward"):
        raise ValueError("mode must be strike_time or peer_outward")
    command_id = payload.get("command_id")
    if not isinstance(command_id, str) or not command_id.strip():
        raise ValueError("return timing command_id must be non-empty")
    numbers = {}
    for key in ("lead_ms", "published_at"):
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{key} must be a finite number")
        numbers[key] = value
    lead = numbers["lead_ms"]
    if not -200 <= lead <= 200 or lead % 20 != 0:
        raise ValueError("lead_ms must be -200..200 in 20 ms steps")
    if numbers["published_at"] < 0:
        raise ValueError("published_at must be non-negative")
    return ReturnTimingConfig(
        robot_id=payload["robot_id"], mode=payload["mode"], lead_ms=int(lead),
        command_id=command_id.strip(), published_at=float(numbers["published_at"]),
    )


class ReturnPathPreview:
    """Conservative quantile preview of the current V11 movement profile.

    Preview is admission-only. It never changes targets or promises to stop an
    accepted RETURN. Robot/ball physics and physical inter-robot collisions still
    require independent validation; this is not a certified safety controller.
    """

    def __init__(self) -> None:
        self.profile = json.loads(PROFILE_PATH.read_text())
        if self.profile["schema"] != "v11-return-path-preview-v1":
            raise ValueError("unsupported RETURN movement preview")

    def minimum_gap(self, *, hitter_state, peer_state, hitter_outward_y, peer_home_y,
                    tts: float, hitter_is_left: bool) -> float:
        times = np.asarray(self.profile["time_s"], dtype=float)
        horizon = times[times <= 1.6]
        # The hitter retains its complete existing 200 ms POST continuation.
        until_outward = max(0.0, tts + self.profile["post_delay_s"])
        out = np.interp(np.maximum(horizon - until_outward, 0.0), times,
                        self.profile["outward"]["p10"])
        ret = np.interp(horizon, times, self.profile["return"]["p90"])
        hy = float(hitter_state["base_position_xyz"][1])
        py = float(peer_state["base_position_xyz"][1])
        hv = float(hitter_state["base_linear_velocity_xyz"][1])
        pv = float(peer_state["base_linear_velocity_xyz"][1])
        sign = 1.0 if hitter_is_left else -1.0
        # Account for measured inward motion before the profiles take over.
        inward_hitter = min(0.0, sign * hv) * np.minimum(horizon, until_outward + .14)
        inward_peer = max(0.0, sign * pv) * np.minimum(horizon, .20)
        gap = sign * (hy - py) + abs(hitter_outward_y - hy) * out
        gap -= abs(peer_home_y - py) * ret
        gap += inward_hitter - inward_peer
        return float(np.min(gap))


class ReturnTiming:
    def __init__(self, robots: tuple[str, ...]) -> None:
        self.robots = robots
        self.active = {r: ReturnTimingConfig(r) for r in robots}
        self.pending: dict[str, ReturnTimingConfig | None] = {r: None for r in robots}
        self.received: dict[str, ReturnTimingConfig | None] = {r: None for r in robots}
        self.error = {r: "" for r in robots}
        self.cycles: dict[str, dict[str, Any] | None] = {r: None for r in robots}
        self.token: str | None = None
        self.profile = ReturnPathPreview()

    def stage(self, payload: Mapping[str, Any]) -> None:
        try:
            config = parse_return_timing(payload, self.robots)
            old = self.received[config.robot_id]
            if old is not None:
                if old.command_id == config.command_id:
                    if old != config:
                        raise ValueError("return timing command_id was reused")
                    self.error[config.robot_id] = ""
                    return
                if config.published_at < old.published_at:
                    raise ValueError("stale return timing configuration")
            self.pending[config.robot_id] = config
            self.received[config.robot_id] = config
            self.error[config.robot_id] = ""
        except ValueError as error:
            robot = payload.get("robot_id") if isinstance(payload, Mapping) else None
            if robot in self.robots:
                self.error[robot] = str(error)
            raise

    def begin(self, hitter: str, token: str, now: float) -> None:
        if token == self.token:
            return
        self.token = token
        peer = next(r for r in self.robots if r != hitter)
        if self.pending[peer] is not None:
            self.active[peer] = self.pending[peer]
            self.pending[peer] = None
        self.cycles[peer] = {
            "commit_token": token, "hitter": hitter, "returner": peer,
            "config": self.active[peer].to_dict(), "committed_at": now,
            "status": "waiting_hit_ack", "trigger_reason": None,
            "blocked_reason": None, "requested_at": None, "requested_sequence": None,
            "acknowledged_at": None, "request_to_return_ms": None,
            "observed_tts_s": None, "predicted_min_gap_m": None,
        }

    def status(self) -> dict[str, dict[str, Any]]:
        return {r: {
            "active": self.active[r].to_dict(),
            "pending": None if self.pending[r] is None else self.pending[r].to_dict(),
            "received_command_id": None if self.received[r] is None else self.received[r].command_id,
            "cycle": self.cycles[r], "error": self.error[r],
        } for r in self.robots}
