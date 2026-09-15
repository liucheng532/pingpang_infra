#!/usr/bin/env python3
"""Session supervisor for the original Predictor, V7 Planner, and Monitor."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
import xmlrpc.client

ROOT = Path(__file__).resolve().parents[1]
COMMAND_TOPICS = (
    "/doubles/table_left/command",
    "/doubles/table_right/command",
)


@dataclass(frozen=True)
class Component:
    name: str
    cwd: Path
    command: tuple[str, ...]


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("dry-run", "check", "start", "status", "stop"), nargs="?", default="dry-run")
    parser.add_argument("--mode", choices=("shadow", "transport-shadow", "active"), default="shadow")
    parser.add_argument("--without-predictor", action="store_true")
    parser.add_argument("--without-monitor", action="store_true")
    parser.add_argument("--site-safety-confirmed", action="store_true")
    parser.add_argument("--config", type=Path, default=ROOT / "config/deployment.json")
    return parser.parse_args()


def process_start(pid):
    return int(Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[19])


def same_process(record):
    try:
        return process_start(int(record["owner_pid"])) == int(record["owner_starttime"])
    except (KeyError, OSError, ValueError):
        return False


def components(deployment, session, session_id, mode, predictor=True, monitor=True):
    runtime_python = Path(deployment["runtime_python"])
    if not runtime_python.is_file():
        runtime_python = Path(sys.executable)
    result = []
    if predictor:
        result.append(
            Component(
                "predictor",
                Path(deployment["predictor_root"]),
                (
                    deployment["predictor_python"], "-B", "-u",
                    str(Path(deployment["predictor_root"]) / deployment["predictor_entry"]),
                ),
            )
        )
    command = [
        str(runtime_python), "-B", "-u", str(ROOT / "scripts/run_v7_planner.py"),
        "--mode", mode, "--session-dir", str(session / "planner"),
        "--session-id", session_id, "--config", str(ROOT / "config/deployment.json"),
    ]
    if mode != "shadow":
        command.extend(("--attestation", str(ROOT / deployment["attestation"])))
    result.append(Component("planner", ROOT, tuple(command)))
    if monitor:
        result.append(
            Component(
                "monitor",
                ROOT,
                (
                    str(runtime_python), "-B", "-u", "-m", "yichao_v7_v11.monitor",
                    "--port", str(deployment["monitor_port"]),
                    "--recordings-root", str(ROOT / "output/monitor_recordings"),
                ),
            )
        )
    return result


def validate_model_bundle(deployment):
    """Load both ONNX sessions and verify every frozen bundle asset."""

    sys.path.insert(0, str(ROOT / "src"))
    from yichao_v7_v11.actor import OnnxActor, OnnxBarrier
    from yichao_v7_v11.attestation import validate_deployment_contract

    validate_deployment_contract(deployment)
    contract = ROOT / deployment["models"]["contract"]
    OnnxActor(ROOT / deployment["models"]["actor_onnx"], contract)
    OnnxBarrier(ROOT / deployment["models"]["barrier_onnx"], contract)


def blockers(process_table=None):
    table = process_table or subprocess.check_output(["ps", "-eo", "pid=,args="], text=True)
    markers = (
        "scripts/run_v7_planner.py", "yichao_v7_v11.monitor",
        "scripts/run_v6r10_planner.py", "yichao_v6r10_v11.monitor",
        "scripts/run_v9_real_fixed_relay.py", "scripts/run_yichao_planner.py",
        "scripts/run_predictor.py", "TableTennis.py",
    )
    own = os.getpid()
    return [
        line.strip()
        for line in table.splitlines()
        if len(line.strip().split(maxsplit=1)) == 2
        and line.strip().split(maxsplit=1)[0].isdigit()
        and int(line.strip().split(maxsplit=1)[0]) != own
        and any(marker in line for marker in markers)
    ]


def topic_publishers(master_uri, topics, timeout_s=3.0):
    """Return publishers for selected topics, failing if the ROS master is unavailable."""
    previous_timeout = socket.getdefaulttimeout()
    socket.setdefaulttimeout(timeout_s)
    try:
        code, message, state = xmlrpc.client.ServerProxy(master_uri).getSystemState(
            "/v7_workstation_preflight"
        )
    finally:
        socket.setdefaulttimeout(previous_timeout)
    if code != 1:
        raise RuntimeError(f"ROS master getSystemState failed: {message}")
    publishers = dict(state[0])
    return {
        topic: tuple(publishers.get(topic, ()))
        for topic in topics
        if publishers.get(topic)
    }


def command_publishers(master_uri, timeout_s=3.0):
    return topic_publishers(master_uri, COMMAND_TOPICS, timeout_s=timeout_s)


def wait_for_topic_publisher(master_uri, topic, timeout_s=10.0):
    """Ensure Monitor subscribes only after the Planner has advertised status."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        found = topic_publishers(master_uri, (topic,))
        if topic in found:
            return found[topic]
        time.sleep(0.05)
    raise RuntimeError(f"timed out waiting for publisher on {topic}")


def stop_owned(processes):
    for number, timeout_s in ((signal.SIGINT, 18.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        alive = [process for process in processes if process.poll() is None]
        if not alive:
            return
        for process in reversed(alive):
            try:
                os.killpg(process.pid, number)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and any(process.poll() is None for process in alive):
            time.sleep(0.05)


def main():
    args = arguments()
    deployment = json.loads(args.config.read_text(encoding="utf-8"))
    current = ROOT / "output/current_workstation.json"
    if args.action in {"status", "stop"}:
        record = json.loads(current.read_text()) if current.is_file() else {}
        alive = same_process(record)
        if args.action == "stop" and alive:
            os.kill(int(record["owner_pid"]), signal.SIGTERM)
        print(json.dumps({"owner_alive": alive, "stop_requested": args.action == "stop" and alive, "record": record}, indent=2))
        return 0
    session_id = "v7_" + time.strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:10]
    session = ROOT / "output/sessions" / session_id
    items = components(
        deployment, session, session_id, args.mode,
        predictor=not args.without_predictor, monitor=not args.without_monitor,
    )
    plan = {
        "start": args.action == "start",
        "mode": args.mode,
        "session_id": session_id,
        "network": deployment["network"],
        "status_topic": deployment["status_topic"],
        "monitor_port": deployment["monitor_port"],
        "active_site_confirmation": args.site_safety_confirmed,
        "components": [
            {"name": item.name, "cwd": str(item.cwd), "argv": list(item.command)} for item in items
        ],
    }
    if args.action == "dry-run":
        print(json.dumps(plan, indent=2))
        return 0
    problems = []
    for item in items:
        if item.name == "predictor" and (not item.cwd.is_dir() or not Path(item.command[0]).is_file() or not Path(item.command[3]).is_file()):
            problems.append("missing Predictor dependency")
    if not (Path(deployment["fixed_runtime_root"]) / "doubles_planner/fixed_relay.py").is_file():
        problems.append("missing unchanged Fixed runtime dependency")
    for relative in deployment["models"].values():
        if not (ROOT / relative).is_file():
            problems.append("missing V7 model artifact: " + relative)
    try:
        validate_model_bundle(deployment)
    except Exception as error:
        problems.append("V7 model/deployment contract failed: " + str(error))
    found = blockers()
    if found:
        problems.append("existing stack processes: " + repr(found))
    try:
        occupied_commands = command_publishers(deployment["network"]["ROS_MASTER_URI"])
        if occupied_commands:
            problems.append("command topic publishers already present: " + repr(occupied_commands))
    except Exception as error:
        problems.append("ROS master preflight failed: " + str(error))
    if not args.without_monitor:
        try:
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind(("0.0.0.0", int(deployment["monitor_port"])))
        except OSError as error:
            problems.append("monitor port unavailable: " + str(error))
    if args.mode == "active" and not args.site_safety_confirmed:
        problems.append("active requires a new explicit --site-safety-confirmed invocation")
    if args.mode == "active" and os.environ.get("V7_ACTIVE_SITE_CONFIRMED") != "YES":
        problems.append("active requires V7_ACTIVE_SITE_CONFIRMED=YES")
    if problems:
        raise RuntimeError("; ".join(problems))
    if args.mode != "shadow":
        controller_mode = "active" if args.mode == "active" else "shadow"
        audit = subprocess.run(
            [
                str(Path(sys.executable)),
                str(ROOT / "scripts/create_v11_attestation.py"),
                "--mode", controller_mode,
                "--config", str(args.config),
                "--output", str(ROOT / deployment["attestation"]),
                "--require-running",
            ],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        if audit.returncode:
            raise RuntimeError("V11 live attestation failed: " + audit.stdout.strip())
    if args.action == "check":
        print(json.dumps({**plan, "start": False, "check": "passed"}, indent=2))
        return 0

    environment = os.environ.copy()
    environment.update(deployment["network"])
    environment.pop("ROS_HOSTNAME", None)
    environment["V7_FIXED_RUNTIME_ROOT"] = deployment["fixed_runtime_root"]
    environment["PYTHONPATH"] = ":".join(
        filter(None, (str(ROOT / "src"), "/opt/ros/noetic/lib/python3/dist-packages", environment.get("PYTHONPATH")))
    )
    session.mkdir(parents=True, exist_ok=False)
    stop = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, lambda *_: stop.set())
    processes = []
    handles = []
    record = {
        **plan,
        "owner_pid": os.getpid(),
        "owner_starttime": process_start(os.getpid()),
        "started_at": time.time(),
        "components": [],
    }
    failed = None
    try:
        for item in items:
            if item.name == "monitor":
                wait_for_topic_publisher(
                    deployment["network"]["ROS_MASTER_URI"], deployment["status_topic"]
                )
            handle = (session / f"{item.name}.log").open("xb")
            handles.append(handle)
            process = subprocess.Popen(
                ["setpriv", "--pdeathsig", "KILL", "--", *item.command],
                cwd=item.cwd,
                env=environment,
                stdout=handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            processes.append(process)
            record["components"].append(
                {"name": item.name, "pid": process.pid, "starttime": process_start(process.pid), "argv": list(item.command)}
            )
        (session / "run.json").write_text(json.dumps(record, indent=2) + "\n")
        current.parent.mkdir(parents=True, exist_ok=True)
        current.write_text(json.dumps(record, indent=2) + "\n")
        print(json.dumps({"session_id": session_id, "session": str(session), "mode": args.mode}), flush=True)
        while not stop.wait(0.1):
            for item, process in zip(items, processes):
                if process.poll() is not None:
                    failed = item.name
                    stop.set()
                    break
    finally:
        stop_owned(processes)
        record.update(stopped_at=time.time(), failed_component=failed, exit_codes=[process.poll() for process in processes])
        (session / "run.json").write_text(json.dumps(record, indent=2) + "\n")
        current.write_text(json.dumps(record, indent=2) + "\n")
        for handle in handles:
            handle.close()
    return int(failed is not None)


if __name__ == "__main__":
    raise SystemExit(main())
