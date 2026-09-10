from __future__ import annotations

import os
import signal
import sys

from scripts.run_doubles_stack import (
    Component,
    build_components,
    find_existing_processes,
    launch_components,
    stop_components,
    supervised_command,
)


def test_default_stack_is_predictor_monitor_and_active_planner() -> None:
    components = build_components()

    assert [component.name for component in components] == [
        "predictor",
        "monitor",
        "fixed_planner_active",
    ]
    assert components[0].command[-1].endswith("scripts/run_predictor.py")
    assert components[1].command[-2:] == ("--port", "8088")
    assert components[2].command[-2].endswith("scripts/run_v9_real_fixed_relay.py")
    assert components[2].command[-1] == "--active"


def test_shadow_must_be_explicit() -> None:
    planner = build_components(shadow=True)[-1]

    assert planner.name == "fixed_planner_shadow"
    assert "--active" not in planner.command


def test_preflight_reports_all_matching_existing_processes() -> None:
    process_table = """
       11 /env/python /repo/TableTennis.py
       12 /env/python scripts/run_predictor.py
       13 /env/python scripts/run_v9_real_fixed_relay.py
       14 /env/python -m ros_web_monitor.server --port 8088
       15 unrelated-service
    """

    matches = find_existing_processes(process_table)

    assert len(matches) == 4
    assert all("unrelated-service" not in match for match in matches)


def test_supervised_command_uses_parent_death_signal() -> None:
    assert supervised_command(("python", "worker.py")) == [
        "setpriv",
        "--pdeathsig",
        "KILL",
        "--",
        "python",
        "worker.py",
    ]


def test_stop_components_reaps_every_process_group(tmp_path) -> None:
    components = tuple(
        Component(
            f"worker_{index}",
            tmp_path,
            (
                sys.executable,
                "-u",
                "-c",
                "import time; time.sleep(60)",
            ),
        )
        for index in range(3)
    )
    started = launch_components(components, os.environ.copy())

    stop_components(started, stages=((signal.SIGINT, 1.0), (signal.SIGKILL, 1.0)))

    assert all(proc.poll() is not None for _component, proc in started)
