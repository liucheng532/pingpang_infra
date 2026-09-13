"""Read-only V6R10 monitor with per-message recording and 10 Hz replay frames."""

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
                "schema": "v6r10-monitor-recording-v1",
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
            if isinstance(source_sequence, int) and not isinstance(source_sequence, bool):
                if isinstance(previous, int) and item.get("source_session") == source_session:
                    if source_sequence > previous + 1:
                        item["sequence_gaps"] += source_sequence - previous - 1
                    elif source_sequence <= previous:
                        item["sequence_resets"] += 1
                item["source_sequence"] = source_sequence
                item["source_session"] = source_session
            item.update(payload=copy.deepcopy(payload), received_monotonic_s=now, count=item["count"] + 1)
            item["samples"].append(now)
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
                }
            result = {
                "schema": "v6r10-monitor-snapshot-v1",
                "event_id": self.event_id,
                "monotonic_s": now,
                "wall_time_s": time.time(),
                "topics": topics,
                "ros": copy.deepcopy(self.ros),
                "recording": self.recording.status(),
                "robot_control_api": False,
                "transitions": list(self.transitions),
                "decisions": list(self.decisions),
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

    rospy.init_node("yichao_v6r10_monitor", anonymous=True, disable_signals=True)

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
        print(f"V6R10 monitor port={args.port} robot_control=False", flush=True)
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
