#!/usr/bin/env python3
"""Keep Predictor alive for V11 preflight, then switch V6R10 to active."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
CURRENT = ROOT / "output/current_active_bootstrap.json"
TORSO_TOPICS = (
    "/doubles/table_left/torso_pose_origin",
    "/doubles/table_right/torso_pose_origin",
)
STATUS_TOPIC = "/doubles/yichao_v6r10/status"


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "status", "stop", "dry-run"), nargs="?", default="start")
    parser.add_argument("--config", type=Path, default=ROOT / "config/deployment.json")
    return parser.parse_args()


def process_start(pid):
    return int(Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19])


def same_process(record):
    try:
        return process_start(int(record["owner_pid"])) == int(record["owner_starttime"])
    except (KeyError, OSError, ValueError):
        return False


def read_record(path):
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def fresh_active_attestation(path, started_at):
    record = read_record(path)
    try:
        return (
            record["controller_mode"] == "active"
            and record["live_processes_verified"] is True
            and float(record["created_at"]) >= started_at
        )
    except (KeyError, TypeError, ValueError):
        return False


def publishers(master_uri, timeout_s=3.0):
    previous_timeout = socket.getdefaulttimeout()
    socket.setdefaulttimeout(timeout_s)
    try:
        code, message, state = xmlrpc.client.ServerProxy(master_uri).getSystemState(
            "/v6r10_active_bootstrap"
        )
    finally:
        socket.setdefaulttimeout(previous_timeout)
    if code != 1:
        raise RuntimeError(f"ROS master getSystemState failed: {message}")
    return {topic: tuple(nodes) for topic, nodes in state[0]}


def wait_for_publishers(master_uri, topics, child, stop_requested, timeout_s=30.0):
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline and not stop_requested():
        if child.poll() is not None:
            raise RuntimeError(f"workstation child exited with status {child.returncode}")
        try:
            found = publishers(master_uri)
            if all(found.get(topic) for topic in topics):
                return
        except Exception as error:  # Keep the final ROS error for an actionable failure.
            last_error = error
        time.sleep(0.1)
    if stop_requested():
        return
    suffix = f": {last_error}" if last_error else ""
    raise RuntimeError("timed out waiting for ROS publishers " + repr(tuple(topics)) + suffix)


def terminate(child):
    if child is None or child.poll() is not None:
        return
    child.send_signal(signal.SIGTERM)
    try:
        child.wait(timeout=30)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait(timeout=5)


def write_current(record):
    CURRENT.parent.mkdir(parents=True, exist_ok=True)
    CURRENT.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")


def main():
    args = arguments()
    deployment = json.loads(args.config.read_text(encoding="utf-8"))
    if args.action in {"status", "stop"}:
        record = read_record(CURRENT)
        alive = same_process(record)
        if args.action == "stop" and alive:
            os.kill(int(record["owner_pid"]), signal.SIGTERM)
        print(json.dumps({"owner_alive": alive, "stop_requested": args.action == "stop" and alive, "record": record}, indent=2))
        return 0
    if args.action == "dry-run":
        print(json.dumps({
            "workflow": "v6r10-v11-two-entry-active-v1",
            "phase_1": "workstation-shadow-until-fresh-active-v11-attestation",
            "phase_2": "automatic-workstation-active",
            "torso_topics": list(TORSO_TOPICS),
            "controller_files_modified": False,
        }, indent=2))
        return 0

    previous = read_record(CURRENT)
    if same_process(previous):
        raise RuntimeError(f"two-entry workstation already running as PID {previous['owner_pid']}")

    started_at = time.time()
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, request_stop)

    environment = os.environ.copy()
    environment["V6R10_ACTIVE_SITE_CONFIRMED"] = "YES"
    base = [str(ROOT / "scripts/start_v6r10_workstation.sh"), "start"]
    record = {
        "schema": "v6r10-v11-two-entry-active-v1",
        "owner_pid": os.getpid(),
        "owner_starttime": process_start(os.getpid()),
        "started_at": started_at,
        "phase": "starting_shadow",
        "child_pid": None,
    }
    write_current(record)
    child = None
    try:
        child = subprocess.Popen([*base, "shadow"], env=environment)
        record.update(phase="waiting_for_torso", child_pid=child.pid)
        write_current(record)
        wait_for_publishers(
            deployment["network"]["ROS_MASTER_URI"], TORSO_TOPICS, child,
            lambda: stop,
        )
        if stop:
            return 0
        record.update(phase="waiting_for_active_v11", torso_publishers_ready_at=time.time())
        write_current(record)
        print("WORKSTATION BOOTSTRAP READY: torso publishers are live.", flush=True)
        print("Now run: bash scripts/start_yichao_robots.sh", flush=True)

        attestation = ROOT / deployment["attestation"]
        while not stop and child.poll() is None:
            if fresh_active_attestation(attestation, started_at):
                break
            time.sleep(0.1)
        if stop:
            return 0
        if child.poll() is not None:
            raise RuntimeError(f"workstation shadow exited with status {child.returncode}")

        record.update(phase="switching_to_active", active_attestation_seen_at=time.time())
        write_current(record)
        print("Fresh V11 active attestation received; switching workstation automatically...", flush=True)
        terminate(child)
        if child.returncode != 0:
            raise RuntimeError(f"workstation shadow cleanup failed with status {child.returncode}")
        child = subprocess.Popen(
            [*base, "active", "--site-safety-confirmed"], env=environment
        )
        record.update(phase="starting_active", child_pid=child.pid)
        write_current(record)
        wait_for_publishers(
            deployment["network"]["ROS_MASTER_URI"], (STATUS_TOPIC,), child,
            lambda: stop,
        )
        if stop:
            return 0
        record.update(phase="active", active_ready_at=time.time())
        write_current(record)
        print("V6R10 WORKSTATION ACTIVE READY: verify Monitor 8090, then follow the normal R2 procedure.", flush=True)
        while not stop and child.poll() is None:
            time.sleep(0.1)
        if not stop and child.returncode:
            raise RuntimeError(f"workstation active exited with status {child.returncode}")
        return 0
    finally:
        terminate(child)
        record.update(phase="stopped", stopped_at=time.time(), child_exit=None if child is None else child.poll())
        write_current(record)


if __name__ == "__main__":
    raise SystemExit(main())
