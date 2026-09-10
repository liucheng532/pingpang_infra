#!/usr/bin/env python3
"""ROS1 doubles dashboard, runtime-config publisher, and JSONL logger."""

from __future__ import annotations

import argparse
from collections import deque
import copy
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import threading
import time
from typing import Any, Mapping
from urllib.parse import parse_qs, urlparse
import uuid

from yichao_v3_v9 import ROOT
from yichao_v3_v9.monitor_recording import Library

from doubles_planner.runtime_motion_config import (
    RUNTIME_MOTION_CONFIG_SCHEMA,
    parse_runtime_motion_config,
)
from doubles_planner.return_timing import (
    RETURN_TIMING_SCHEMA, RETURN_TIMING_TOPIC, ReturnTiming, parse_return_timing,
)


DEFAULT_TOPICS = {
    "left_state": "/doubles/table_left/state",
    "right_state": "/doubles/table_right/state",
    "left_torso": "/doubles/table_left/torso_pose_origin",
    "right_torso": "/doubles/table_right/torso_pose_origin",
    "left_command": "/doubles/table_left/command",
    "right_command": "/doubles/table_right/command",
    "ball": "/doubles/ball_prediction",
    "fixed_status": "/doubles/yichao/status",
}
POSE_TOPIC_KEYS = {"left_torso", "right_torso"}

STATIC_INDEX = Path(__file__).with_name("static") / "index.html"
RUNTIME_MOTION_CONFIG_TOPIC = "/doubles/runtime_motion_config"
ROBOT_IDS = ("table_right", "table_left")


def _utc_text(epoch: float | None = None) -> str:
    value = time.time() if epoch is None else float(epoch)
    return datetime.fromtimestamp(value, timezone.utc).isoformat(timespec="milliseconds")


def _finite_json(value: Any) -> Any:
    """Return a browser-safe JSON value without NaN/Infinity or exotic objects."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): _finite_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_finite_json(item) for item in value]
    return str(value)


def _json_bytes(value: Any, *, pretty: bool = False) -> bytes:
    options = {"ensure_ascii": False, "allow_nan": False}
    if pretty:
        options.update(indent=2, sort_keys=True)
    else:
        options.update(separators=(",", ":"))
    return json.dumps(_finite_json(value), **options).encode("utf-8")


def quaternion_xyzw_to_rpy_deg(value: Any) -> list[float] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 4:
        return None
    try:
        x, y, z, w = (float(item) for item in value)
    except (TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in (x, y, z, w)):
        return None
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-12:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.degrees(item) for item in (roll, pitch, yaw)]


def pose_stamped_payload(message: Any) -> dict[str, Any]:
    orientation = [
        float(message.pose.orientation.x),
        float(message.pose.orientation.y),
        float(message.pose.orientation.z),
        float(message.pose.orientation.w),
    ]
    return {
        "position": [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ],
        "orientation_xyzw": orientation,
        "rpy_deg": quaternion_xyzw_to_rpy_deg(orientation),
        "source_stamp_s": float(message.header.stamp.to_sec()),
        "frame_id": str(message.header.frame_id),
    }


class RotatingJsonlLog:
    """Small dependency-free size-rotating JSONL writer."""

    def __init__(self, path: Path, max_bytes: int, backups: int) -> None:
        self.path = path.resolve()
        self.max_bytes = max(1, int(max_bytes))
        self.backups = max(1, int(backups))
        self._lock = threading.Lock()
        self._stream = None
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def _open(self) -> None:
        if self._stream is None:
            self._stream = self.path.open("ab", buffering=0)

    def _rotate(self) -> None:
        if self._stream is not None:
            self._stream.close()
            self._stream = None
        oldest = self.path.with_name(f"{self.path.name}.{self.backups}")
        if oldest.exists():
            oldest.unlink()
        for index in range(self.backups - 1, 0, -1):
            source = self.path.with_name(f"{self.path.name}.{index}")
            if source.exists():
                source.replace(self.path.with_name(f"{self.path.name}.{index + 1}"))
        if self.path.exists():
            self.path.replace(self.path.with_name(f"{self.path.name}.1"))

    def write(self, value: Any) -> None:
        line = _json_bytes(value) + b"\n"
        with self._lock:
            self._open()
            assert self._stream is not None
            if self._stream.tell() and self._stream.tell() + len(line) > self.max_bytes:
                self._rotate()
                self._open()
            assert self._stream is not None
            self._stream.write(line)

    def close(self) -> None:
        with self._lock:
            if self._stream is not None:
                self._stream.close()
                self._stream = None


class TelemetryStore:
    """Thread-safe latest-value store and bounded event broker."""

    def __init__(self, topics: Mapping[str, str], log: RotatingJsonlLog | None = None) -> None:
        self.topics = dict(topics)
        self.log = log
        self.session = {}
        self.recordings = None
        self.started_epoch = time.time()
        self.started_monotonic = time.monotonic()
        self._condition = threading.Condition()
        self._sequence = 0
        self._latest: dict[str, dict[str, Any]] = {}
        self._events: deque[dict[str, Any]] = deque(maxlen=4096)
        self._rate_samples: dict[str, deque[float]] = {}
        self._motion_config_sender = None
        self._return_timing_sender = None
        self._return_timing_submit = {
            robot: {"last_sent": None, "error": ""} for robot in ROBOT_IDS
        }
        self._motion_config_submit = {
            robot: {"last_sent": None, "error": ""} for robot in ROBOT_IDS
        }
        self._server: dict[str, Any] = {
            "mode": "starting",
            "ros_connected": False,
            "log_enabled": log is not None,
            "log_file": None if log is None else str(log.path),
        }

    @property
    def sequence(self) -> int:
        with self._condition:
            return self._sequence

    def set_server(self, **values: Any) -> None:
        with self._condition:
            self._server.update(_finite_json(values))

    def set_motion_config_sender(self, sender) -> None:
        with self._condition:
            self._motion_config_sender = sender

    def set_return_timing_sender(self, sender) -> None:
        with self._condition:
            self._return_timing_sender = sender

    def submit_return_timing(self, robot: str, values: Mapping[str, Any]) -> dict[str, Any]:
        if robot not in ROBOT_IDS:
            raise ValueError("unknown robot")
        payload = {
            **dict(values), "schema": RETURN_TIMING_SCHEMA,
            "robot_id": robot, "command_id": uuid.uuid4().hex, "published_at": time.time(),
        }
        try:
            payload = parse_return_timing(payload, ROBOT_IDS).to_dict()
            with self._condition:
                sender = self._return_timing_sender
            if sender is None:
                raise RuntimeError("return timing publisher is not ready")
            sender(payload)
        except Exception as error:
            with self._condition:
                self._return_timing_submit[robot]["error"] = str(error)
            raise
        with self._condition:
            self._return_timing_submit[robot] = {"last_sent": payload, "error": ""}
        if self.log is not None:
            self.log.write({"key": "return_timing_submit", "received_at": _utc_text(), "data": payload})
        return payload

    def submit_motion_config(self, robot: str, values: Mapping[str, Any]) -> dict[str, Any]:
        if robot not in ROBOT_IDS:
            raise ValueError("unknown robot")
        payload = {
            **dict(values),
            "schema": RUNTIME_MOTION_CONFIG_SCHEMA,
            "command_id": uuid.uuid4().hex,
            "published_at": time.time(),
            "robot_id": robot,
        }
        try:
            config = parse_runtime_motion_config(payload, expected_robots=ROBOT_IDS)
            with self._condition:
                sender = self._motion_config_sender
            if sender is None:
                raise RuntimeError("runtime motion config publisher is not ready")
            sender(config.command_dict())
        except Exception as error:
            with self._condition:
                self._motion_config_submit[robot]["error"] = str(error)
            raise
        sent = config.command_dict()
        with self._condition:
            self._motion_config_submit[robot] = {"last_sent": sent, "error": ""}
        return sent

    def ingest(self, key: str, raw: str | Any) -> dict[str, Any]:
        received_epoch = time.time()
        received_monotonic = time.monotonic()
        parse_error = None
        if isinstance(raw, str):
            try:
                data = json.loads(raw)
            except (json.JSONDecodeError, TypeError) as error:
                data = {"raw": raw}
                parse_error = str(error)
        else:
            data = raw
        data = _finite_json(data)

        with self._condition:
            previous = self._latest.get(key)
            samples = self._rate_samples.setdefault(key, deque(maxlen=200))
            samples.append(received_monotonic)
            rate_hz = 0.0
            if len(samples) > 1 and samples[-1] > samples[0]:
                # A windowed rate remains meaningful when rospy delivers a high-rate
                # topic in small TCP bursts. Instantaneous 1/dt greatly over-reports
                # those streams (for example 4 kHz for an actual 360 Hz source).
                rate_hz = (len(samples) - 1) / (samples[-1] - samples[0])
            self._sequence += 1
            record = {
                "id": self._sequence,
                "key": key,
                "topic": self.topics.get(key, key),
                "received_at": _utc_text(received_epoch),
                "received_epoch": received_epoch,
                "received_monotonic": received_monotonic,
                "messages": 1 if previous is None else int(previous.get("messages", 0)) + 1,
                "rate_hz": rate_hz,
                "parse_error": parse_error,
                "data": data,
            }
            self._latest[key] = record
            event = {name: value for name, value in record.items() if name != "received_monotonic"}
            self._events.append(event)
            self._condition.notify_all()

        if self.log is not None:
            self.log.write(event)
        if key == "fixed_status" and isinstance(data, dict) and data.get("session_id"):
            self.session = {"session": data["session_id"]}
        if self.recordings is not None:
            self.recordings.topic(key, data, data.get("session_id") if isinstance(data, dict) else None, received_epoch)
        return event

    def snapshot(self) -> dict[str, Any]:
        now_epoch = time.time()
        now_monotonic = time.monotonic()
        with self._condition:
            latest = copy.deepcopy(self._latest)
            server = copy.deepcopy(self._server)
            motion_config_submit = copy.deepcopy(self._motion_config_submit)
            return_timing_submit = copy.deepcopy(self._return_timing_submit)
            sequence = self._sequence
        for key, record in latest.items():
            record["age_s"] = max(0.0, now_monotonic - float(record.pop("received_monotonic")))
            record["configured_topic"] = self.topics.get(key, key)
        return {
            "sequence": sequence,
            "session": self.session.copy(),
            "server": {
                **server,
                "started_at": _utc_text(self.started_epoch),
                "now": _utc_text(now_epoch),
                "uptime_s": max(0.0, now_monotonic - self.started_monotonic),
            },
            "configured_topics": self.topics,
            "topics": latest,
            "motion_config_submit": motion_config_submit,
            "return_timing_submit": return_timing_submit,
        }

    def wait_after(self, cursor: int, timeout: float = 15.0) -> list[dict[str, Any]]:
        deadline = time.monotonic() + timeout
        with self._condition:
            while self._sequence <= cursor:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return []
                self._condition.wait(remaining)
            return [copy.deepcopy(event) for event in self._events if int(event["id"]) > cursor]


class MonitorHttpServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: tuple[str, int],
        store: TelemetryStore,
        web_rate_hz: float = 30.0,
    ) -> None:
        super().__init__(address, MonitorHandler)
        self.store = store
        self.recordings = store.recordings
        self.web_interval_s = 1.0 / max(1.0, float(web_rate_hz))


class MonitorHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "DoublesRosMonitor/1.0"

    @property
    def monitor(self) -> MonitorHttpServer:
        return self.server  # type: ignore[return-value]

    def log_message(self, _format: str, *_args: Any) -> None:
        return

    def _send_bytes(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, value: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
        self._send_bytes(_json_bytes(value), "application/json; charset=utf-8", status)

    def do_GET(self) -> None:  # noqa: N802 - stdlib handler API
        parsed = urlparse(self.path)
        if parsed.path == "/":
            try:
                body = STATIC_INDEX.read_bytes()
            except OSError as error:
                self._send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)
                return
            self._send_bytes(body, "text/html; charset=utf-8")
            return
        if parsed.path == "/api/recordings/download":
            try:
                name = parse_qs(parsed.query)["id"][0]
                path = self.monitor.recordings.path(name)/"recording.jsonl"
                with path.open("rb") as stream:
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "application/x-ndjson")
                    self.send_header("Content-Length", str(path.stat().st_size))
                    self.send_header("Content-Disposition", f'attachment; filename="{name}.jsonl"')
                    self.end_headers()
                    while True:
                        chunk = stream.read(1024*1024)
                        if not chunk: break
                        self.wfile.write(chunk)
            except (ValueError, KeyError, OSError) as error:
                self._send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            return
        if parsed.path == "/api/recordings":
            self._send_json(self.monitor.recordings.list())
            return
        if parsed.path == "/api/replay":
            try:
                query = parse_qs(parsed.query)
                result = self.monitor.recordings.frames(query["id"][0], float(query.get("start", [0])[0]), 35.)
                self._send_json(result)
            except (ValueError, KeyError, OSError) as error:
                self._send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            return
        if parsed.path == "/api/snapshot":
            self._send_json(self.monitor.store.snapshot())
            return
        if parsed.path == "/healthz":
            snapshot = self.monitor.store.snapshot()
            self._send_json(
                {
                    "ok": True,
                    "ros_connected": snapshot["server"]["ros_connected"],
                    "topic_count": len(snapshot["topics"]),
                    "sequence": snapshot["sequence"],
                }
            )
            return
        if parsed.path == "/api/events":
            self._serve_events(parsed.query)
            return
        if parsed.path == "/api/log/latest":
            self._serve_log()
            return
        self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:  # noqa: N802 - stdlib handler API
        parsed = urlparse(self.path)
        if parsed.path == "/api/recordings/import":
            try:
                result = self.monitor.recordings.upload(self.rfile, int(self.headers.get("Content-Length", "0")))
                self._send_json(result)
            except (ValueError, KeyError, OSError) as error:
                self._send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            return
        if parsed.path in {"/api/recordings/start", "/api/recordings/stop"}:
            try:
                result = (self.monitor.recordings.start("Yichao 测试") if parsed.path.endswith("start")
                          else self.monitor.recordings.stop())
                self._send_json(result)
            except (ValueError, RuntimeError) as error:
                self._send_json({"error": str(error)}, HTTPStatus.BAD_REQUEST)
            return
        routes = {
            "/api/runtime_motion_config/": self.monitor.store.submit_motion_config,
            "/api/return_timing/": self.monitor.store.submit_return_timing,
        }
        prefix = next((p for p in routes if parsed.path.startswith(p)), None)
        if prefix is None:
            self._send_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return
        robot = parsed.path[len(prefix):]
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 4096:
                raise ValueError("invalid request body length")
            values = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(values, dict):
                raise ValueError("request body must be a JSON object")
            payload = routes[prefix](robot, values)
        except (ValueError, RuntimeError, json.JSONDecodeError) as error:
            self._send_json({"ok": False, "error": str(error)}, HTTPStatus.BAD_REQUEST)
            return
        self._send_json({"ok": True, **payload})

    def _serve_log(self) -> None:
        log = self.monitor.store.log
        if log is None or not log.path.is_file():
            self._send_json({"error": "log is disabled or empty"}, HTTPStatus.NOT_FOUND)
            return
        try:
            body = log.path.read_bytes()
        except OSError as error:
            self._send_json({"error": str(error)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/x-ndjson")
        self.send_header("Content-Disposition", f'attachment; filename="{log.path.name}"')
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _serve_events(self, query: str) -> None:
        parameters = parse_qs(query)
        header_cursor = self.headers.get("Last-Event-ID", "0")
        try:
            cursor = int(parameters.get("last_id", [header_cursor])[0])
        except (TypeError, ValueError):
            cursor = 0
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache, no-transform")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        try:
            snapshot = self.monitor.store.snapshot()
            cursor = max(cursor, int(snapshot["sequence"]))
            self._write_sse("snapshot", snapshot, cursor)
            next_emit = time.monotonic()
            while True:
                events = self.monitor.store.wait_after(cursor, timeout=10.0)
                if not events:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    continue

                # Do not make the browser process a 360 Hz mocap stream one event
                # at a time. Wait for the configured display interval, then retain
                # only the newest event per topic. Logging remains full-rate.
                delay = next_emit - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                newest_id = max(int(event["id"]) for event in events)
                events.extend(self.monitor.store.wait_after(newest_id, timeout=0.0))
                latest_by_key: dict[str, dict[str, Any]] = {}
                for event in events:
                    latest_by_key[str(event["key"])] = event
                cursor = max(int(event["id"]) for event in events)
                for event in sorted(latest_by_key.values(), key=lambda item: int(item["id"])):
                    self._write_sse("topic", event, cursor)
                next_emit = time.monotonic() + self.monitor.web_interval_s
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            return

    def _write_sse(self, event_name: str, value: Any, event_id: int) -> None:
        body = _json_bytes(value)
        self.wfile.write(f"id: {event_id}\nevent: {event_name}\ndata: ".encode("ascii"))
        self.wfile.write(body)
        self.wfile.write(b"\n\n")
        self.wfile.flush()


def _parse_topics(values: list[str], include_defaults: bool) -> dict[str, str]:
    topics = dict(DEFAULT_TOPICS) if include_defaults else {}
    for value in values:
        if "=" not in value:
            raise ValueError(f"topic must be KEY=/ros/topic, got {value!r}")
        key, topic = (part.strip() for part in value.split("=", 1))
        if not key or not topic.startswith("/"):
            raise ValueError(f"topic must be KEY=/ros/topic, got {value!r}")
        topics[key] = topic
    if not topics:
        raise ValueError("at least one topic is required")
    return topics


def _start_demo(store: TelemetryStore, stop: threading.Event) -> threading.Thread:
    demo_timing = ReturnTiming(ROBOT_IDS)
    timing_lock = threading.RLock()

    def submit_timing(payload):
        with timing_lock:
            demo_timing.stage(payload)

    store.set_return_timing_sender(submit_timing)
    demo_motion_configs = {
        robot: {
            "schema": "v10-doubles-runtime-motion-config-status-v1",
            "robot_id": robot,
            "received_command_id": None,
            "robot_received_command_id": None,
            "error": "",
            "active": {
                "goal_x": 0.18,
                "outward_y": -0.9125 if robot == "table_right" else 0.9125,
                "home_y": -0.20 if robot == "table_right" else 0.20,
                "outward_hold_s": 3.0,
                "target_base_x_clip": 0.04,
            },
            "pending": None,
            "outward_applied_command_id": None,
            "home_applied_command_id": None,
        }
        for robot in ROBOT_IDS
    }

    def submit_demo(payload):
        robot = payload["robot_id"]
        command_id = payload["command_id"]
        demo_motion_configs[robot] = {
            **demo_motion_configs[robot],
            "received_command_id": command_id,
            "robot_received_command_id": command_id,
            "active": {
                name: payload[name]
                for name in ("goal_x", "outward_y", "home_y", "outward_hold_s")
            }
            | {"target_base_x_clip": 0.04},
            "pending": None,
            "outward_applied_command_id": command_id,
            "home_applied_command_id": command_id,
        }

    store.set_motion_config_sender(submit_demo)

    def loop() -> None:
        sequence = 0
        start = time.monotonic()
        while not stop.wait(0.05):
            sequence += 1
            elapsed = time.monotonic() - start
            shot_index = int(elapsed // 4)
            local = elapsed % 4
            hitter_left = shot_index % 2 == 0
            with timing_lock:
                demo_timing.begin("table_left" if hitter_left else "table_right", f"demo-{shot_index}", time.monotonic())
                timing_status = copy.deepcopy(demo_timing.status())
            left_y = 0.2 + 0.70 * (0.5 - 0.5 * math.cos(min(local, 2.0) * math.pi / 2.0))
            right_y = -0.91 + 0.70 * (0.5 - 0.5 * math.cos(min(local, 2.0) * math.pi / 2.0))
            if not hitter_left:
                left_y, right_y = 0.91 - (left_y - 0.2), -0.2 - (right_y + 0.91)
            for side, y, is_hitter in (
                ("left", left_y, hitter_left),
                ("right", right_y, not hitter_left),
            ):
                robot = f"table_{side}"
                phase = "HIT" if is_hitter and 0.6 < local < 1.3 else ("OUTWARD" if is_hitter else "HOME_HOLD")
                store.ingest(
                    f"{side}_state",
                    {
                        "schema_version": "v9-robot-state-v1",
                        "robot": robot,
                        "sequence": sequence,
                        "base_position_xyz": [0.0, y, 0.75],
                        "base_linear_velocity_xyz": [0.0, 0.1 * math.sin(elapsed), 0.0],
                        "phase": phase,
                        "ready": phase.endswith("HOLD"),
                        "valid": True,
                        "emergency_stop": False,
                        "state_elapsed_s": local,
                        "target_base_y": 0.9125 if is_hitter else (-0.2 if side == "right" else 0.2),
                    },
                )
                store.ingest(
                    f"{side}_torso",
                    {
                        "position": [0.02, y, 0.95],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "rpy_deg": [
                            2.0 * math.sin(elapsed),
                            1.0 * math.cos(elapsed),
                            (8.0 if side == "left" else -8.0) * math.sin(elapsed * 0.5),
                        ],
                        "source_stamp_s": time.time(),
                        "frame_id": "origin",
                    },
                )
                store.ingest(
                    f"{side}_command",
                    {
                        "valid": True,
                        "planned_valid": True,
                        "planned_active": is_hitter,
                        "relay_stage": "committed" if is_hitter else "clear",
                        "command": {
                            "active": is_hitter,
                            "role": "hit" if is_hitter else "hold",
                            "desired_base_position": [0.0, 0.2 if side == "left" else -0.2],
                        },
                        "fallbacks": [],
                    },
                )
            store.ingest(
                "ball",
                {
                    "schema_version": "v9-ball-prediction-v1",
                    "sequence": sequence,
                    "shot_id": f"demo-{shot_index + 1}",
                    "valid": True,
                    "position": [1.2 - 0.6 * local, 0.35 * math.sin(elapsed), 1.0 + 0.2 * math.sin(local * math.pi)],
                    "velocity": [-3.0, 0.0, -0.5],
                    "predicted_strike_position": [0.45, 0.2 if hitter_left else -0.2, 1.0],
                    "racket_velocity": [3.0, 0.0, 0.6],
                    "time_to_strike_s": max(-0.2, 1.3 - local),
                },
            )
            if sequence % 4 == 0:
                store.ingest(
                    "fixed_status",
                    {
                        "shadow": True,
                        "planner_mode": "demo",
                        "pending_shot_id": f"demo-{shot_index + 1}",
                        "cycle_hitter": "table_left" if hitter_left else "table_right",
                        "admission_reasons": [],
                        "runtime_motion_config": copy.deepcopy(demo_motion_configs),
                        "return_timing": timing_status,
                    },
                )

    thread = threading.Thread(target=loop, name="ros-monitor-demo", daemon=True)
    thread.start()
    return thread


def _connect_ros(store: TelemetryStore, topics: Mapping[str, str], enable_config=False) -> tuple[Any, list[Any]]:
    try:
        import rospy
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
    except ImportError as error:
        raise RuntimeError(
            "cannot import ROS1 rospy/std_msgs; source /opt/ros/noetic/setup.bash "
            "or run with --demo"
        ) from error

    rospy.init_node("yichao_fixed_web_monitor", anonymous=False)
    if enable_config:
        motion_config_publisher = rospy.Publisher(
            RUNTIME_MOTION_CONFIG_TOPIC,
            String,
            queue_size=10,
        )
        store.set_motion_config_sender(
            lambda payload: motion_config_publisher.publish(
                String(data=json.dumps(payload, allow_nan=False, separators=(",", ":")))
            )
        )
        return_timing_publisher = rospy.Publisher(RETURN_TIMING_TOPIC, String, queue_size=10)
        store.set_return_timing_sender(
            lambda payload: return_timing_publisher.publish(
                String(data=json.dumps(payload, allow_nan=False, separators=(",", ":")))
            )
        )
    subscribers = []
    for key, topic in topics.items():
        if key in POSE_TOPIC_KEYS:
            subscribers.append(
                rospy.Subscriber(
                    topic,
                    PoseStamped,
                    lambda message, event_key=key: store.ingest(
                        event_key, pose_stamped_payload(message)
                    ),
                    queue_size=20,
                )
            )
        else:
            subscribers.append(
                rospy.Subscriber(
                    topic,
                    String,
                    lambda message, event_key=key: store.ingest(event_key, message.data),
                    queue_size=20,
                )
            )
    store.set_server(mode="ros", ros_connected=True, ros_master=rospy.get_param("/rosdistro", "unknown"))
    if enable_config:
        subscribers.extend((motion_config_publisher, return_timing_publisher))
    return rospy, subscribers


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Read-only ROS1 doubles telemetry bridge and web dashboard"
    )
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address")
    parser.add_argument("--port", default=8089, type=int, help="HTTP port")
    parser.add_argument(
        "--web-rate",
        default=30.0,
        type=float,
        help="maximum browser update rate in Hz (ROS logging stays full-rate)",
    )
    parser.add_argument(
        "--topic",
        action="append",
        default=[],
        metavar="KEY=/ros/topic",
        help="add or override a std_msgs/String JSON topic",
    )
    parser.add_argument("--no-default-topics", action="store_true")
    parser.add_argument("--enable-config", action="store_true", help="Enable live motion/RETURN settings")
    parser.add_argument("--demo", action="store_true", help="generate demo data without ROS")
    parser.add_argument("--no-log", action="store_true", help="disable JSONL logging")
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=Path("outputs/ros_web_monitor_logs"),
        help="directory for monitor-only JSONL logs",
    )
    parser.add_argument("--max-log-mb", default=100.0, type=float)
    parser.add_argument("--log-backups", default=5, type=int)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        topics = _parse_topics(args.topic, not args.no_default_topics)
    except ValueError as error:
        raise SystemExit(str(error)) from error

    log = None
    if not args.no_log:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log = RotatingJsonlLog(
            args.log_dir / f"doubles_ros_{stamp}.jsonl",
            max_bytes=int(args.max_log_mb * 1024 * 1024),
            backups=args.log_backups,
        )
    store = TelemetryStore(topics, log)
    store.recordings = Library(ROOT, store)
    stop = threading.Event()
    rospy = None
    subscribers: list[Any] = []
    if args.demo:
        store.set_server(mode="demo", ros_connected=False)
        _start_demo(store, stop)
    else:
        rospy, subscribers = _connect_ros(store, topics, args.enable_config)

    server = MonitorHttpServer((args.host, args.port), store, args.web_rate)
    actual_port = int(server.server_address[1])
    store.set_server(bind=args.host, port=actual_port, web_rate_hz=max(1.0, args.web_rate))
    serving = threading.Thread(target=server.serve_forever, name="ros-monitor-http", daemon=True)
    serving.start()

    shown_host = "127.0.0.1" if args.host in {"0.0.0.0", "::"} else args.host
    print(f"[ros-web-monitor] dashboard: http://{shown_host}:{actual_port}", flush=True)
    print(f"[ros-web-monitor] mode: {'demo' if args.demo else 'ROS monitor + config'}", flush=True)
    print(f"[ros-web-monitor] log: {log.path if log else 'disabled'}", flush=True)

    def request_stop(*_unused: Any) -> None:
        stop.set()
        if rospy is not None and not rospy.is_shutdown():
            rospy.signal_shutdown("web monitor stopping")

    signal.signal(signal.SIGTERM, request_stop)
    try:
        if rospy is not None:
            rospy.spin()
        else:
            while not stop.wait(0.5):
                pass
    except KeyboardInterrupt:
        request_stop()
    finally:
        # Keep ROS handles alive until the bridge is fully stopped.
        _ = subscribers
        stop.set()
        store.set_server(ros_connected=False)
        server.shutdown()
        server.server_close()
        serving.join(timeout=2.0)
        if store.recordings.recorder is not None:
            store.recordings.stop()
        if log is not None:
            log.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
