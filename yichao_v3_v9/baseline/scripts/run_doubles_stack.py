#!/usr/bin/env python3
"""Start the doubles predictor, monitor, and fixed planner as one unit."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Iterable, Sequence


RUNTIME_ROOT = Path(__file__).resolve().parents[1]
PREDICTOR_ROOT = Path("/home/odl/codebase/pingpang_predictor_dual_v9")
PREDICTOR_PYTHON = Path("/home/odl/miniconda3/envs/tabletennis/bin/python")
RUNTIME_PYTHON = Path("/home/odl/miniconda3/envs/tabletennis-active-v1/bin/python")
CALIBRATION_CONFIG = Path(
    "/home/odl/codebase/pingpang_planner_haoran_chingmu_mocap_g1/calib/calibration_config.json"
)
ROS_PYTHONPATH = "/opt/ros/noetic/lib/python3/dist-packages"
PROCESS_MARKERS = (
    "TableTennis.py",
    "scripts/run_predictor.py",
    "scripts/run_v9_real_fixed_relay.py",
    "ros_web_monitor.server",
)


@dataclass(frozen=True)
class Component:
    name: str
    cwd: Path
    command: tuple[str, ...]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Start the complete doubles stack")
    parser.add_argument(
        "--shadow",
        action="store_true",
        help="Start the fixed planner in shadow instead of the default active mode.",
    )
    return parser.parse_args()


def build_components(*, shadow: bool = False) -> tuple[Component, ...]:
    planner_command = [
        str(RUNTIME_PYTHON),
        "-B",
        "-u",
        str(RUNTIME_ROOT / "scripts" / "run_v9_real_fixed_relay.py"),
    ]
    if not shadow:
        planner_command.append("--active")
    return (
        Component(
            "predictor",
            PREDICTOR_ROOT,
            (
                str(PREDICTOR_PYTHON),
                "-B",
                "-u",
                str(PREDICTOR_ROOT / "scripts" / "run_predictor.py"),
            ),
        ),
        Component(
            "monitor",
            RUNTIME_ROOT,
            (
                str(RUNTIME_PYTHON),
                "-B",
                "-u",
                "-m",
                "ros_web_monitor.server",
                "--host",
                "0.0.0.0",
                "--port",
                "8088",
            ),
        ),
        Component(
            "fixed_planner_active" if not shadow else "fixed_planner_shadow",
            RUNTIME_ROOT,
            tuple(planner_command),
        ),
    )


def build_environment() -> dict[str, str]:
    env = os.environ.copy()
    env["ROS_MASTER_URI"] = "http://192.168.123.165:11311"
    env["ROS_IP"] = "192.168.123.165"
    env["PINGPANG_CALIBRATION_CONFIG"] = str(CALIBRATION_CONFIG)
    current_pythonpath = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = (
        f"{ROS_PYTHONPATH}:{current_pythonpath}" if current_pythonpath else ROS_PYTHONPATH
    )
    return env


def find_existing_processes(process_table: str | None = None) -> list[str]:
    if process_table is None:
        process_table = subprocess.check_output(
            ["ps", "-eo", "pid=,args="], text=True
        )
    own_pid = os.getpid()
    matches = []
    for line in process_table.splitlines():
        fields = line.strip().split(maxsplit=1)
        if len(fields) != 2 or not fields[0].isdigit() or int(fields[0]) == own_pid:
            continue
        if any(marker in fields[1] for marker in PROCESS_MARKERS):
            matches.append(line.strip())
    return matches


def supervised_command(command: Sequence[str]) -> list[str]:
    return ["setpriv", "--pdeathsig", "KILL", "--", *command]


def launch_components(
    components: Iterable[Component], env: dict[str, str]
) -> list[tuple[Component, subprocess.Popen[object]]]:
    started = []
    try:
        for component in components:
            proc = subprocess.Popen(
                supervised_command(component.command),
                cwd=component.cwd,
                env=env,
                start_new_session=True,
            )
            started.append((component, proc))
            print(f"started {component.name}: pid={proc.pid}", flush=True)
    except BaseException:
        stop_components(started)
        raise
    return started


def stop_components(
    started: Sequence[tuple[Component, subprocess.Popen[object]]],
    stages: Sequence[tuple[signal.Signals, float]] | None = None,
) -> None:
    shutdown_stages = stages or (
        (signal.SIGINT, 18.0),
        (signal.SIGTERM, 3.0),
        (signal.SIGKILL, 2.0),
    )
    for sig, timeout_s in shutdown_stages:
        alive = [(component, proc) for component, proc in started if proc.poll() is None]
        if not alive:
            break
        for _component, proc in reversed(alive):
            try:
                os.killpg(proc.pid, sig)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout_s
        while any(proc.poll() is None for _component, proc in alive):
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)

    lingering = [component.name for component, proc in started if proc.poll() is None]
    for _component, proc in started:
        if proc.poll() is not None:
            proc.wait()
    if lingering:
        raise RuntimeError(f"components did not exit after SIGKILL: {', '.join(lingering)}")


def main() -> int:
    args = parse_args()
    blockers = find_existing_processes()
    if blockers:
        print("Refusing to start: existing predictor/planner/monitor processes found:")
        for line in blockers:
            print(f"  {line}")
        return 2

    stop = threading.Event()

    def handle_signal(_signum: int, _frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGHUP, handle_signal)

    started: list[tuple[Component, subprocess.Popen[object]]] = []
    failed_component: str | None = None
    try:
        started = launch_components(build_components(shadow=args.shadow), build_environment())
        mode = "SHADOW" if args.shadow else "ACTIVE"
        print(f"Doubles stack is {mode}. Monitor: http://172.16.3.126:8088", flush=True)
        print("Ctrl+C stops predictor, planner, and monitor.", flush=True)
        while not stop.wait(0.1):
            for component, proc in started:
                if proc.poll() is not None:
                    failed_component = component.name
                    stop.set()
                    break
    finally:
        stop_components(started)

    if failed_component is not None:
        print(f"{failed_component} exited; stopped the remaining stack.", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
