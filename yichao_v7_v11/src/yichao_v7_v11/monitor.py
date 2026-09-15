"""Read-only V7 monitor with per-message recording and 10 Hz replay frames."""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import deque
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import queue
import re
import signal
import threading
import time
from urllib.parse import parse_qs, urlparse
import uuid

from . import ROOT, STATUS_TOPIC

ROBOTS = ("table_left", "table_right")
KNOWN_PHASES = ("HOME_HOLD", "HIT", "POST_DELAY", "OUTWARD", "OUTWARD_HOLD", "RETURN")
PHASE_STALE_S = 0.25
EXPECTED_NEXT = {
    "HIT": ("POST_DELAY", "HIT 段完成"),
    "POST_DELAY": ("OUTWARD", "0.2s post delay 完成"),
    "OUTWARD": ("OUTWARD_HOLD", "outward reference 完成"),
    "OUTWARD_HOLD": ("RETURN", "Planner return permission 生效"),
    "RETURN": ("HOME_HOLD", "return reference 完成"),
}
STATIC = Path(__file__).with_name("static")
RECORDING_ID = re.compile(r"^rec_[0-9]{8}_[0-9]{6}_[0-9a-f]{8}$")


def _fresh(topic, limit=0.25):
    return bool(topic) and float(topic.get("age_s", 1.0e9)) <= limit


def trace(snapshot):
    """Compact 10 Hz plotting trace without manufacturing missing positions."""
    result = {"t": snapshot["monotonic_s"], "robots": {}}
    topics = snapshot.get("topics", {})
    for robot in ROBOTS:
        state = topics.get(f"state/{robot}", {})
        torso = topics.get(f"torso/{robot}", {})
        command = topics.get(f"command/{robot}", {})
        state_payload = state.get("payload", {})
        torso_payload = torso.get("payload", {})
        command_payload = command.get("payload", {})
        state_position = state_payload.get("base_position_xyz")
        torso_position = torso_payload.get("position_xyz")
        position = (
            state_position
            if _fresh(state) and state_payload.get("valid") is True and state_position
            else torso_position if _fresh(torso) and torso_position else None
        )
        wire_command = command_payload.get("command") or {}
        desired = wire_command.get("desired_base_position")
        command_valid = _fresh(command) and command_payload.get("valid") is True
        result["robots"][robot] = {
            "y": position[1] if position else None,
            "target_y": desired[1] if command_valid and desired else None,
            "phase": state_payload.get("phase") if _fresh(state) else None,
            "role": wire_command.get("role") if command_valid else None,
        }
    return result


def pose_payload(message):
    stamp = message.header.stamp
    source_timestamp = (
        float(stamp.to_sec())
        if hasattr(stamp, "to_sec")
        else float(stamp.secs) + float(stamp.nsecs) * 1.0e-9
    )
    return {
        "frame_id": str(message.header.frame_id),
        "source_timestamp_s": source_timestamp,
        "position_xyz": [
            float(message.pose.position.x),
            float(message.pose.position.y),
            float(message.pose.position.z),
        ],
        "orientation_xyzw": [
            float(message.pose.orientation.x),
            float(message.pose.orientation.y),
            float(message.pose.orientation.z),
            float(message.pose.orientation.w),
        ],
    }


def _phase_name(value):
    return value if value in KNOWN_PHASES else "UNKNOWN"


def _state_context(payload, command=None, planner=None):
    command = command or {}
    planner = planner or {}
    wire = command.get("command") or {}
    staging = planner.get("staging") or {}
    robot = payload.get("robot") or command.get("robot")
    expected = (staging.get("sequence") or {}).get(robot)
    target = (staging.get("targets") or {}).get(robot)
    applied = payload.get("last_applied_sequence")
    session = payload.get("last_planner_session_id")
    session_match = session is not None and session == planner.get("session_id")
    sequence_ack = (
        session_match
        and type(applied) is int
        and type(expected) is int
        and applied >= expected
    )
    try:
        target_ack = abs(float(payload.get("target_base_y")) - float(target)) <= 1.0e-4
    except (TypeError, ValueError):
        target_ack = False
    transport_error = payload.get("transport_error")
    error_sequence = payload.get("transport_error_sequence")
    error_session = payload.get("transport_error_session_id")
    transport_error_relevant = bool(
        transport_error
        and (
            expected is None
            or (
                type(error_sequence) is int
                and error_sequence == expected
                and error_session == planner.get("session_id")
            )
        )
    )
    abnormal = (transport_error if transport_error_relevant else None) or (
        "emergency_stop" if payload.get("emergency_stop") else None
    ) or ("controller_state_invalid" if payload.get("valid") is False else None)
    return {
        "shot_id": command.get("shot_id", staging.get("shot_id")),
        "role": wire.get("role"),
        "session_id": session,
        "sequence": payload.get("sequence"),
        "last_applied_sequence": applied,
        "expected_sequence": expected,
        "ack": bool(sequence_ack and target_ack),
        "transport_error": transport_error,
        "transport_error_sequence": error_sequence,
        "transport_error_session_id": error_session,
        "transport_error_relevant": transport_error_relevant,
        "abnormal_reason": abnormal,
    }


def _next_expected(phase, context):
    if phase == "HOME_HOLD":
        if context.get("role") == "hit" and context.get("ack") is True:
            return "HIT", "本机 HIT 命令已受理"
        return "WAITING_SCHEDULE", "等待本机 HIT 命令受理"
    if phase in EXPECTED_NEXT:
        return EXPECTED_NEXT[phase]
    if phase == "STALE":
        return "UNKNOWN", "等待 Controller state 恢复"
    return "UNKNOWN", "未知 Controller phase"


def build_recorded_phase_index(events, start_monotonic_s, duration_s):
    """Build actual phase segments from recorded messages; never interpolate."""

    latest_planner = {}
    latest_commands = {robot: {} for robot in ROBOTS}
    state_events = {robot: [] for robot in ROBOTS}
    for event in events:
        topic = event.get("topic")
        payload = event.get("payload") or {}
        if topic == "planner":
            latest_planner = payload
        elif isinstance(topic, str) and topic.startswith("command/"):
            robot = topic.split("/", 1)[1]
            if robot in latest_commands:
                latest_commands[robot] = payload
        elif isinstance(topic, str) and topic.startswith("state/"):
            robot = topic.split("/", 1)[1]
            if robot in state_events:
                state_events[robot].append(
                    (
                        float(event["monotonic_s"]),
                        copy.deepcopy(payload),
                        copy.deepcopy(latest_commands[robot]),
                        copy.deepcopy(latest_planner),
                    )
                )
    end = float(start_monotonic_s) + max(0.0, float(duration_s))
    result = {}
    for robot, rows in state_events.items():
        segments = []
        previous_time = None
        current = None
        for stamp, payload, command, planner in rows:
            if previous_time is not None and stamp - previous_time > PHASE_STALE_S:
                if current is not None:
                    current["exit_monotonic_s"] = previous_time + PHASE_STALE_S
                    current["duration_s"] = current["exit_monotonic_s"] - current["enter_monotonic_s"]
                    current = None
                segments.append({
                    "robot": robot,
                    "phase": "STALE",
                    "raw_phase": None,
                    "enter_monotonic_s": previous_time + PHASE_STALE_S,
                    "exit_monotonic_s": stamp,
                    "duration_s": stamp - previous_time - PHASE_STALE_S,
                    "abnormal_reason": "state_age_exceeded_250ms",
                })
            phase = _phase_name(payload.get("phase"))
            context = _state_context(payload, command, planner)
            signature = (phase, payload.get("phase"), context.get("session_id"))
            if current is None or current.get("signature") != signature:
                if current is not None:
                    current["exit_monotonic_s"] = stamp
                    current["duration_s"] = stamp - current["enter_monotonic_s"]
                current = {
                    "robot": robot,
                    "phase": phase,
                    "raw_phase": payload.get("phase"),
                    "enter_monotonic_s": stamp,
                    "exit_monotonic_s": None,
                    "duration_s": 0.0,
                    "signature": signature,
                    **context,
                }
                if phase == "UNKNOWN" and not current.get("abnormal_reason"):
                    current["abnormal_reason"] = "unknown_phase:" + str(payload.get("phase"))
                segments.append(current)
            else:
                current.update(context)
            previous_time = stamp
        if current is not None and previous_time is not None:
            exit_stamp = min(end, previous_time + PHASE_STALE_S)
            current["exit_monotonic_s"] = exit_stamp
            current["duration_s"] = max(0.0, exit_stamp - current["enter_monotonic_s"])
            if end > previous_time + PHASE_STALE_S:
                segments.append({
                    "robot": robot,
                    "phase": "STALE",
                    "raw_phase": None,
                    "enter_monotonic_s": previous_time + PHASE_STALE_S,
                    "exit_monotonic_s": end,
                    "duration_s": end - previous_time - PHASE_STALE_S,
                    "abnormal_reason": "state_age_exceeded_250ms",
                })
        for index, segment in enumerate(segments):
            segment.pop("signature", None)
            following = next(
                (item for item in segments[index + 1 :] if item["phase"] != "STALE"),
                None,
            )
            segment["next_actual_state"] = None if following is None else following["phase"]
            segment["enter_time_s"] = max(0.0, segment["enter_monotonic_s"] - start_monotonic_s)
            segment["exit_time_s"] = max(0.0, segment["exit_monotonic_s"] - start_monotonic_s)
        result[robot] = segments
    return result


class RecordingStore:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.current = None
        self.pending = None
        self.worker = None

    def status(self):
        with self.lock:
            return copy.deepcopy(self.current) if self.current else {"active": False}

    def start(self):
        with self.lock:
            if self.current and self.current["active"]:
                return self.status()
            identifier = "rec_" + time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
            directory = self.root / identifier
            directory.mkdir()
            self.current = {
                "schema": "v7-monitor-recording-v1",
                "id": identifier,
                "active": True,
                "started_at": time.time(),
                "start_monotonic_s": time.monotonic(),
                "messages": 0,
                "frames": 0,
                "dropped": 0,
                "error": None,
                "duration_s": 0.0,
            }
            self.pending = queue.Queue(maxsize=20000)
            self.worker = threading.Thread(
                target=self._write, args=(directory, self.pending, self.current), daemon=True
            )
            self.worker.start()
            return self.status()

    def append(self, item):
        with self.lock:
            if not self.current or not self.current["active"]:
                return
            try:
                self.pending.put_nowait(copy.deepcopy(item))
            except queue.Full:
                self.current["dropped"] += 1

    def _write(self, directory, pending, metadata):
        try:
            with (directory / "events.jsonl").open("xb") as events, (
                directory / "frames.jsonl"
            ).open("xb") as frames, (directory / "frames.index.jsonl").open(
                "x", encoding="utf-8"
            ) as index:
                while True:
                    item = pending.get()
                    if item is None:
                        break
                    encoded = (
                        json.dumps(item, separators=(",", ":"), allow_nan=False) + "\n"
                    ).encode("utf-8")
                    if item["kind"] == "frame":
                        offset = frames.tell()
                        frames.write(encoded)
                        elapsed = max(
                            0.0,
                            item["snapshot"]["monotonic_s"] - metadata["start_monotonic_s"],
                        )
                        index.write(json.dumps({"time_s": elapsed, "offset": offset}) + "\n")
                        with self.lock:
                            metadata["frames"] += 1
                            metadata["duration_s"] = elapsed
                    else:
                        events.write(encoded)
                        with self.lock:
                            metadata["messages"] += 1
        except Exception as error:  # pragma: no cover - visible in UI/status
            with self.lock:
                metadata["error"] = str(error)

    def stop(self):
        with self.lock:
            if not self.current or not self.current["active"]:
                return self.status()
            self.current["active"] = False
            pending, worker, identifier = self.pending, self.worker, self.current["id"]
        pending.put(None)
        worker.join(timeout=20.0)
        if worker.is_alive():
            raise RuntimeError("recording writer did not finish")
        with self.lock:
            manifest = copy.deepcopy(self.current)
            (self.root / identifier / "manifest.json").write_text(
                json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
            )
            return manifest

    def list(self):
        result = []
        for path in sorted(self.root.glob("rec_*/manifest.json"), reverse=True):
            try:
                result.append(json.loads(path.read_text(encoding="utf-8")))
            except (OSError, ValueError):
                continue
        current = self.status()
        if current.get("active"):
            result.insert(0, current)
        return result

    def frame(self, identifier, requested):
        directory = self.directory(identifier)
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        entries = [json.loads(line) for line in (directory / "frames.index.jsonl").read_text().splitlines()]
        if not entries:
            raise ValueError("recording has no replay frames")
        times = [entry["time_s"] for entry in entries]
        position = max(0, min(len(entries) - 1, bisect_right(times, requested) - 1))
        with (directory / "frames.jsonl").open("rb") as stream:
            stream.seek(entries[position]["offset"])
            record = json.loads(stream.readline())
        return {
            **record["snapshot"],
            "playback": {
                "id": identifier,
                "time_s": times[position],
                "duration_s": manifest["duration_s"],
                "frame": position,
                "frame_count": len(entries),
            },
        }

    def phases(self, identifier):
        directory = self.directory(identifier)
        manifest = json.loads((directory / "manifest.json").read_text(encoding="utf-8"))
        events = [
            json.loads(line)
            for line in (directory / "events.jsonl").read_text(encoding="utf-8").splitlines()
            if line
        ]
        return {
            "schema": "v7-monitor-recorded-phases-v1",
            "id": identifier,
            "duration_s": float(manifest.get("duration_s", 0.0)),
            "robots": build_recorded_phase_index(
                events,
                float(manifest["start_monotonic_s"]),
                float(manifest.get("duration_s", 0.0)),
            ),
        }

    def directory(self, identifier):
        if not isinstance(identifier, str) or RECORDING_ID.fullmatch(identifier) is None:
            raise ValueError("invalid recording id")
        return self.root / identifier


class Collector:
    def __init__(self, recording: RecordingStore):
        self.recording = recording
        self.lock = threading.RLock()
        self.topics = {}
        self.event_id = 0
        self.transitions = deque(maxlen=120)
        self.decisions = deque(maxlen=24)
        self.timeline = deque(maxlen=300)
        self.signatures = {}
        self.last_decision_key = None
        self.phase_segments = {robot: deque(maxlen=4096) for robot in ROBOTS}
        self.last_state_received = {robot: None for robot in ROBOTS}
        self.ros = {"initialized": False, "parse_errors": 0, "error": None}

    def ingest(self, name, payload):
        if not isinstance(payload, dict):
            raise ValueError("payload must be a JSON object")
        now = time.monotonic()
        wall = time.time()
        with self.lock:
            self.event_id += 1
            item = self.topics.setdefault(
                name,
                {
                    "count": 0,
                    "samples": deque(maxlen=1000),
                    "sequence_gaps": 0,
                    "sequence_resets": 0,
                },
            )
            source_sequence = payload.get("sequence")
            source_session = payload.get(
                "session_id",
                payload.get("last_planner_session_id", payload.get("planner_session_id")),
            )
            previous = item.get("source_sequence")
            sequence_issue = None
            session_switch = (
                item.get("source_session") is not None
                and source_session != item.get("source_session")
            )
            if isinstance(source_sequence, int) and not isinstance(source_sequence, bool):
                if isinstance(previous, int) and item.get("source_session") == source_session:
                    if source_sequence > previous + 1:
                        item["sequence_gaps"] += source_sequence - previous - 1
                        sequence_issue = f"sequence_gap:{previous}->{source_sequence}"
                    elif source_sequence <= previous:
                        item["sequence_resets"] += 1
                        sequence_issue = f"sequence_reset:{previous}->{source_sequence}"
                if session_switch:
                    item["session_switches"] = item.get("session_switches", 0) + 1
                    sequence_issue = "session_switch"
                item["source_sequence"] = source_sequence
                item["source_session"] = source_session
            item.update(payload=copy.deepcopy(payload), received_monotonic_s=now, count=item["count"] + 1)
            item["samples"].append(now)
            if name.startswith("state/"):
                self._capture_phase(name.split("/", 1)[1], payload, now, wall, sequence_issue)
            self._capture_summary(name, payload, wall)
            self.recording.append(
                {
                    "kind": "message",
                    "event_id": self.event_id,
                    "topic": name,
                    "monotonic_s": now,
                    "wall_time_s": wall,
                    "payload": payload,
                }
            )

    def _capture_phase(self, robot, payload, now, wall, sequence_issue=None):
        if robot not in self.phase_segments:
            return
        rows = self.phase_segments[robot]
        previous_received = self.last_state_received[robot]
        if previous_received is not None and now - previous_received > PHASE_STALE_S:
            if rows and rows[-1].get("exit_monotonic_s") is None:
                rows[-1]["exit_monotonic_s"] = previous_received + PHASE_STALE_S
                rows[-1]["exit_wall_time_s"] = rows[-1]["enter_wall_time_s"] + (
                    rows[-1]["exit_monotonic_s"] - rows[-1]["enter_monotonic_s"]
                )
            rows.append({
                "robot": robot,
                "phase": "STALE",
                "raw_phase": None,
                "enter_monotonic_s": previous_received + PHASE_STALE_S,
                "enter_wall_time_s": wall - (now - previous_received - PHASE_STALE_S),
                "exit_monotonic_s": now,
                "exit_wall_time_s": wall,
                "abnormal_reason": "state_age_exceeded_250ms",
            })
        phase = _phase_name(payload.get("phase"))
        command = (self.topics.get(f"command/{robot}") or {}).get("payload") or {}
        planner = (self.topics.get("planner") or {}).get("payload") or {}
        context = _state_context(payload, command, planner)
        if sequence_issue:
            context["abnormal_reason"] = sequence_issue
        if phase == "UNKNOWN":
            unknown = "unknown_phase:" + str(payload.get("phase"))
            context["abnormal_reason"] = ";".join(
                value for value in (context.get("abnormal_reason"), unknown) if value
            )
        signature = (phase, payload.get("phase"), context.get("session_id"))
        active = rows[-1] if rows else None
        if active is None or active.get("signature") != signature or active.get("phase") == "STALE":
            if active is not None and active.get("exit_monotonic_s") is None:
                active["exit_monotonic_s"] = now
                active["exit_wall_time_s"] = wall
            active = {
                "robot": robot,
                "phase": phase,
                "raw_phase": payload.get("phase"),
                "enter_monotonic_s": now,
                "enter_wall_time_s": wall,
                "exit_monotonic_s": None,
                "exit_wall_time_s": None,
                "signature": signature,
                **context,
            }
            rows.append(active)
        else:
            active.update(context)
        self.last_state_received[robot] = now

    def _phase_flow(self, now, wall):
        window_start = now - 30.0
        result = {
            "schema": "v7-monitor-phase-flow-v1",
            "mode": "live",
            "cursor_monotonic_s": now,
            "cursor_wall_time_s": wall,
            "window_start_monotonic_s": window_start,
            "window_duration_s": 30.0,
            "stale_after_s": PHASE_STALE_S,
            "robots": {},
        }
        for robot in ROBOTS:
            segments = [copy.deepcopy(item) for item in self.phase_segments[robot]]
            last_received = self.last_state_received[robot]
            if segments and segments[-1].get("exit_monotonic_s") is None:
                actual_end = now if last_received is None else min(now, last_received + PHASE_STALE_S)
                segments[-1]["exit_monotonic_s"] = actual_end
                segments[-1]["exit_wall_time_s"] = segments[-1]["enter_wall_time_s"] + (
                    actual_end - segments[-1]["enter_monotonic_s"]
                )
            if last_received is None or now - last_received > PHASE_STALE_S:
                stale_start = window_start if last_received is None else last_received + PHASE_STALE_S
                segments.append({
                    "robot": robot,
                    "phase": "STALE",
                    "raw_phase": None,
                    "enter_monotonic_s": stale_start,
                    "enter_wall_time_s": wall - (now - stale_start),
                    "exit_monotonic_s": now,
                    "exit_wall_time_s": wall,
                    "abnormal_reason": (
                        "state_missing" if last_received is None else "state_age_exceeded_250ms"
                    ),
                })
            segments = [
                item for item in segments
                if float(item.get("exit_monotonic_s", now)) >= window_start
            ]
            for item in segments:
                item.pop("signature", None)
                item["duration_s"] = max(
                    0.0, float(item["exit_monotonic_s"]) - float(item["enter_monotonic_s"])
                )
            current = segments[-1] if segments else None
            phase = "STALE" if current is None else current["phase"]
            next_state, condition = _next_expected(phase, current or {})
            result["robots"][robot] = {
                "segments": segments,
                "current_state": phase,
                "current_duration_s": 0.0 if current is None else current["duration_s"],
                "next_state": next_state,
                "transition_condition": condition,
                "next_state_kind": "expected",
            }
        return result

    def _capture_summary(self, name, payload, wall):
        if name == "planner":
            decision = payload.get("rl_decision")
            if isinstance(decision, dict):
                key = (decision.get("session_id"), decision.get("sequence"))
                if key != self.last_decision_key:
                    self.last_decision_key = key
                    record = copy.deepcopy(decision)
                    record["observed_stamp"] = payload.get("stamp", wall)
                    self.decisions.append(record)
            staging = payload.get("staging") or {}
            signature = (
                payload.get("reason"), payload.get("cycle_hitter"),
                staging.get("shot_id"), staging.get("complete"),
                tuple(staging.get("acknowledged") or ()), staging.get("failure"),
            )
            if self.signatures.get(name) != signature:
                self.signatures[name] = signature
                self.transitions.append({
                    "event_id": self.event_id, "time": wall, "topic": name,
                    "label": payload.get("reason") or "planner",
                    "shot_id": staging.get("shot_id"),
                    "detail": (
                        "ACK " + ", ".join(staging.get("acknowledged") or ())
                        if staging.get("acknowledged") else staging.get("failure")
                    ),
                })
            return
        if name.startswith("state/"):
            signature = (
                payload.get("phase"), payload.get("valid"), payload.get("ready"),
                payload.get("emergency_stop"), payload.get("last_planner_session_id"),
                payload.get("transport_error"), payload.get("target_base_y"),
            )
            if self.signatures.get(name) != signature:
                self.signatures[name] = signature
                self.transitions.append({
                    "event_id": self.event_id, "time": wall, "topic": name,
                    "label": payload.get("phase") or "state",
                    "status": "ready" if payload.get("ready") else "not-ready",
                    "y": payload.get("target_base_y"),
                    "detail": payload.get("transport_error"),
                    "control_state": {
                        key: copy.deepcopy(payload.get(key))
                        for key in (
                            "phase", "valid", "ready", "emergency_stop",
                            "last_planner_session_id", "last_applied_sequence",
                            "target_base_y", "transport_error",
                        )
                    },
                })
            return
        if name.startswith("command/"):
            command = payload.get("command") or {}
            desired = command.get("desired_base_position") or []
            target_y = desired[1] if len(desired) > 1 else None
            signature = (
                command.get("role"), command.get("active"), payload.get("valid"),
                payload.get("planned_active"), payload.get("shot_id"), target_y,
                payload.get("commit_token"),
            )
            if self.signatures.get(name) != signature:
                self.signatures[name] = signature
                self.transitions.append({
                    "event_id": self.event_id, "time": wall, "topic": name,
                    "label": command.get("role") or "command",
                    "shot_id": payload.get("shot_id"), "y": target_y,
                    "status": "published", "detail": ", ".join(payload.get("admission_reasons") or ()),
                })

    def snapshot(self, sample=False):
        now = time.monotonic()
        with self.lock:
            topics = {}
            for name, item in self.topics.items():
                recent = [stamp for stamp in item["samples"] if now - stamp <= 2.0]
                hz = (
                    (len(recent) - 1) / (recent[-1] - recent[0])
                    if len(recent) > 1 and recent[-1] > recent[0]
                    else 0.0
                )
                topics[name] = {
                    "payload": copy.deepcopy(item["payload"]),
                    "count": item["count"],
                    "age_s": max(0.0, now - item["received_monotonic_s"]),
                    "hz": hz,
                    "sequence_gaps": item.get("sequence_gaps", 0),
                    "sequence_resets": item.get("sequence_resets", 0),
                    "session_switches": item.get("session_switches", 0),
                }
            wall = time.time()
            result = {
                "schema": "v7-monitor-snapshot-v1",
                "event_id": self.event_id,
                "monotonic_s": now,
                "wall_time_s": wall,
                "topics": topics,
                "ros": copy.deepcopy(self.ros),
                "recording": self.recording.status(),
                "robot_control_api": False,
                "transitions": list(self.transitions),
                "decisions": list(self.decisions),
                "phase_flow": self._phase_flow(now, wall),
            }
            if sample:
                self.timeline.append(trace(result))
            result["timeline"] = list(self.timeline)
            if sample:
                self.recording.append({"kind": "frame", "snapshot": result})
            return result


def handler(collector, recording):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def json(self, status, value):
            body = json.dumps(value, separators=(",", ":"), allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            parsed = urlparse(self.path)
            try:
                if parsed.path == "/api/snapshot":
                    return self.json(200, collector.snapshot())
                if parsed.path == "/api/recordings":
                    return self.json(200, {"recordings": recording.list()})
                pieces = parsed.path.strip("/").split("/")
                if len(pieces) == 4 and pieces[:2] == ["api", "recordings"]:
                    identifier, operation = pieces[2:]
                    if operation == "frame":
                        requested = float(parse_qs(parsed.query).get("time", ["0"])[0])
                        return self.json(200, recording.frame(identifier, requested))
                    if operation == "phases":
                        return self.json(200, recording.phases(identifier))
                    if operation == "messages":
                        path = recording.directory(identifier) / "events.jsonl"
                        body = path.read_bytes()
                        self.send_response(200)
                        self.send_header("Content-Type", "application/x-ndjson")
                        self.send_header("Content-Disposition", f'attachment; filename="{identifier}.jsonl"')
                        self.send_header("Content-Length", str(len(body)))
                        self.end_headers()
                        self.wfile.write(body)
                        return
                files = {
                    "/": ("monitor.html", "text/html; charset=utf-8"),
                    "/monitor.js": ("monitor.js", "application/javascript; charset=utf-8"),
                    "/monitor.css": ("monitor.css", "text/css; charset=utf-8"),
                }
                if parsed.path in files:
                    name, content_type = files[parsed.path]
                    body = (STATIC / name).read_bytes()
                    self.send_response(200)
                    self.send_header("Content-Type", content_type)
                    self.send_header("Content-Length", str(len(body)))
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("X-Content-Type-Options", "nosniff")
                    self.end_headers()
                    self.wfile.write(body)
                    return
                return self.json(404, {"error": "not found"})
            except (OSError, ValueError, KeyError) as error:
                return self.json(400, {"error": str(error)})

        def do_POST(self):
            if self.path not in {"/api/record/start", "/api/record/stop"}:
                return self.json(404, {"error": "no robot control API"})
            length = int(self.headers.get("Content-Length", "0"))
            if length > 4096:
                return self.json(413, {"error": "request too large"})
            self.rfile.read(length)
            value = recording.start() if self.path.endswith("start") else recording.stop()
            return self.json(200, value)

    return Handler


def bind_ros(collector):
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String

    rospy.init_node("yichao_v7_monitor", anonymous=True, disable_signals=True)

    def callback(message, name):
        try:
            collector.ingest(name, json.loads(message.data))
        except (TypeError, ValueError, json.JSONDecodeError) as error:
            collector.ros["parse_errors"] += 1
            collector.ros["error"] = str(error)

    def torso_callback(message, name):
        try:
            collector.ingest(name, pose_payload(message))
        except (AttributeError, TypeError, ValueError) as error:
            collector.ros["parse_errors"] += 1
            collector.ros["error"] = str(error)

    topics = [(STATUS_TOPIC, "planner"), ("/doubles/ball_prediction", "ball")]
    for robot in ROBOTS:
        topics.extend(
            (
                (f"/doubles/{robot}/state", f"state/{robot}"),
                (f"/doubles/{robot}/command", f"command/{robot}"),
            )
        )
    subscriptions = [
        rospy.Subscriber(topic, String, callback, callback_args=name, queue_size=1000, buff_size=16 * 1024 * 1024)
        for topic, name in topics
    ]
    subscriptions.extend(
        rospy.Subscriber(
            f"/doubles/{robot}/torso_pose_origin",
            PoseStamped,
            torso_callback,
            callback_args=f"torso/{robot}",
            queue_size=100,
            tcp_nodelay=True,
        )
        for robot in ROBOTS
    )
    collector.ros["initialized"] = True
    return subscriptions


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--recordings-root", type=Path, default=ROOT / "output/monitor_recordings")
    parser.add_argument("--offline", action="store_true")
    args = parser.parse_args()
    recording = RecordingStore(args.recordings_root)
    collector = Collector(recording)
    server = ThreadingHTTPServer((args.host, args.port), handler(collector, recording))
    stopped = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: stopped.set())
    subscriptions = []
    try:
        if not args.offline:
            subscriptions = bind_ros(collector)
            recording.start()
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f"V7 monitor port={args.port} robot_control=False", flush=True)
        while not stopped.wait(0.1):
            collector.snapshot(sample=True)
    finally:
        server.shutdown()
        server.server_close()
        if "thread" in locals():
            thread.join()
        for subscription in subscriptions:
            subscription.unregister()
        recording.stop()


if __name__ == "__main__":
    main()
