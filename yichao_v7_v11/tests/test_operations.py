import importlib.util
import html
from http.server import ThreadingHTTPServer
import json
import re
import shutil
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from urllib.request import Request, urlopen

import pytest

from yichao_v7_v11 import PLANNER_IDENTITY, ROOT
from yichao_v7_v11.attestation import ATTESTATION_SCHEMA, verify_attestation
import yichao_v7_v11.monitor as monitor
from yichao_v7_v11.monitor import (
    Collector,
    RecordingStore,
    build_recorded_phase_index,
    handler,
    pose_payload,
    trace,
)


def load_program(folder, name):
    spec = importlib.util.spec_from_file_location(
        "test_" + folder + "_" + name, ROOT / folder / name
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def script(name):
    return load_program("scripts", name)


def deployment():
    return json.loads((ROOT / "config/deployment.json").read_text())


def attestation_record(config, now=1000.0, mode="shadow"):
    argv_suffix = " --shadow" if mode == "shadow" else ""
    robots = {}
    for index, (robot, item) in enumerate(config["robots"].items(), start=1):
        robots[robot] = {
            "host_label": item["host_label"], "ssh_host": item["ssh_host"],
            "boot_id": f"boot-{index}", "assets": config["controller_assets"],
            "external_assets": config["external_controller_assets"],
            "embedded_controller_contract": config["embedded_controller_contract"],
            "g1_control": {
                "pid": 100 + index, "starttime": 200 + index,
                "argv": "/runtime/g1_control " + robot, "cwd": "/runtime",
                "exe": "/runtime/g1_control", "comm": "g1_control",
                "environment": {
                    "LCM_DEFAULT_URL":
                        f"udpm://239.255.76.{item['host_label']}:7667?ttl=0"
                },
            },
            "policy": {
                "pid": 300 + index, "starttime": 400 + index,
                "argv": "python3 scripts/deploy_policy.py --robot-id " + robot
                + " --external-planner --hit-policy-source teacher --hit-arm7-residual-mode off"
                + argv_suffix,
                "cwd": "/runtime/g1_gym_deploy", "exe": "/usr/bin/python3",
                "comm": "python3",
            },
        }
    return {
        "schema": ATTESTATION_SCHEMA, "planner_identity": PLANNER_IDENTITY,
        "created_at": now, "controller_mode": mode, "profile": "normal",
        "residual_mode": "off", "residual_scale": 1.0, "residual_family": "v11-teacher",
        "live_processes_verified": True,
        "launcher": config["original_v11_launcher"],
        "launcher_sha256": config["original_v11_launcher_sha256"],
        "robots": robots,
    }


def test_attestation_verifies_mode_assets_contract_and_process_identity(tmp_path):
    config = deployment()
    path = tmp_path / "attestation.json"
    path.write_text(json.dumps(attestation_record(config)))
    record = verify_attestation(path, expected_controller_mode="shadow", deployment=config, now=1050.0)
    assert record["robots"]["table_left"]["host_label"] == "198"
    broken = attestation_record(config)
    broken["robots"]["table_right"]["assets"] = {}
    path.write_text(json.dumps(broken))
    with pytest.raises(ValueError, match="assets"):
        verify_attestation(path, expected_controller_mode="shadow", deployment=config, now=1050.0)


def test_attestation_rejects_stale_wrong_mode_and_swapped_lcm(tmp_path):
    config = deployment()
    path = tmp_path / "attestation.json"
    path.write_text(json.dumps(attestation_record(config, mode="shadow")))
    with pytest.raises(ValueError):
        verify_attestation(path, expected_controller_mode="active", deployment=config, now=1050.0)
    with pytest.raises(ValueError, match="age"):
        verify_attestation(path, expected_controller_mode="shadow", deployment=config, now=2000.0)
    record = attestation_record(config)
    record["robots"]["table_left"]["g1_control"]["environment"]["LCM_DEFAULT_URL"] = (
        "udpm://239.255.76.66:7667?ttl=0"
    )
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match="LCM_DEFAULT_URL"):
        verify_attestation(path, expected_controller_mode="shadow", deployment=config, now=1050.0)


def test_deployment_contract_is_semantically_frozen(tmp_path):
    config = deployment()
    config["robots"]["table_left"]["host_label"] = "66"
    path = tmp_path / "attestation.json"
    path.write_text(json.dumps(attestation_record(deployment())))
    with pytest.raises(ValueError, match="deployment contract"):
        verify_attestation(path, expected_controller_mode="shadow", deployment=config, now=1050.0)


def test_deployment_pins_residual_ball_30300_runtime():
    config = deployment()
    path = (
        "/home/unitree/haoran/v01-arm7-residual-deploy-safe-20260810-50hz/"
        "g1_gym_deploy/utils/residual_policy.py"
    )
    assert config["external_controller_assets"][path] == (
        "85a68757e261621c5500a4fb3d2105557da38d5e50ca383b580232a66f62d5f5"
    )
    assert config["original_v11_launcher_sha256"] == (
        "27fb30e2075f8a67cc3e83dd52a0de9407e62246e7a38ec4527f39feeeb0a0da"
    )
    assert config["controller_assets"]["g1_gym_deploy/scripts/deploy_policy.py"] == (
        "1860b3bc195c661829b4336267410845ad637b759c66c3ed1642be1264097891"
    )
    assert config["controller_assets"]["g1_gym_deploy/utils/v11_home1000_contract.py"] == (
        "7f3e7549885f73a45b8619b9cbc00f27bd809e2654f6dec03fec338e6a78f2b5"
    )


def test_launch_plans_fix_network_modes_residual_and_no_remote_io():
    workstation_source = (ROOT / "scripts/start_v7_workstation.sh").read_text()
    assert "/home/odl/miniconda3/envs/tabletennis-active-v1/bin/python" in workstation_source
    assert "[[ -x $configured_python ]]" in workstation_source
    workstation = subprocess.run(
        ["bash", str(ROOT / "scripts/start_v7_workstation.sh"), "dry-run", "shadow", "--without-predictor", "--without-monitor"],
        text=True, capture_output=True, check=True,
    )
    plan = json.loads(workstation.stdout)
    assert plan["network"]["ROS_MASTER_URI"] == "http://192.168.123.165:11311"
    assert plan["status_topic"] == "/doubles/yichao_v7/status"
    assert [item["name"] for item in plan["components"]] == ["planner"]
    assert "--mode" in plan["components"][0]["argv"]
    robots = subprocess.run(
        ["bash", str(ROOT / "scripts/start_v11_robots.sh"), "dry-run", "shadow"],
        text=True, capture_output=True, check=True,
    )
    robot_plan = json.loads(robots.stdout)
    assert robot_plan["remote_contacted"] is False
    assert robot_plan["residual_mode"] == "off"
    assert robot_plan["residual_family"] == "v11-teacher"


def test_simple_two_entry_launchers_plan_automatic_active_handoff():
    workstation = subprocess.run(
        ["bash", str(ROOT / "scripts/start_yichao_workstation.sh"), "dry-run"],
        text=True, capture_output=True, check=True,
    )
    plan = json.loads(workstation.stdout)
    assert plan["phase_1"] == "workstation-shadow-until-fresh-active-v11-attestation"
    assert plan["phase_2"] == "automatic-workstation-active"
    assert not plan["controller_files_modified"]
    robots = subprocess.run(
        ["bash", str(ROOT / "scripts/start_yichao_robots.sh"), "dry-run"],
        text=True, capture_output=True, check=True,
    )
    robot_plan = json.loads(robots.stdout)
    assert robot_plan["controller_mode"] == "active"
    assert robot_plan["remote_contacted"] is False


def test_active_bootstrap_requires_fresh_live_active_attestation(tmp_path):
    module = script("run_active_bootstrap.py")
    path = tmp_path / "attestation.json"
    for record, expected in (
        ({"controller_mode": "shadow", "live_processes_verified": True, "created_at": 1001.0}, False),
        ({"controller_mode": "active", "live_processes_verified": False, "created_at": 1001.0}, False),
        ({"controller_mode": "active", "live_processes_verified": True, "created_at": 999.0}, False),
        ({"controller_mode": "active", "live_processes_verified": True, "created_at": 1001.0}, True),
    ):
        path.write_text(json.dumps(record))
        assert module.fresh_active_attestation(path, 1000.0) is expected


def test_workstation_conflict_detection_and_command_publishers(monkeypatch):
    module = script("run_workstation.py")
    table = "42 python3 scripts/run_v9_real_fixed_relay.py\n43 python3 unrelated.py\n"
    assert module.blockers(table) == ["42 python3 scripts/run_v9_real_fixed_relay.py"]

    class Master:
        def getSystemState(self, _caller):
            return 1, "ok", [[["/doubles/table_left/command", ["/old_planner"]], ["/unrelated", ["/other"]]], [], []]

    monkeypatch.setattr(module.xmlrpc.client, "ServerProxy", lambda _uri: Master())
    assert module.command_publishers("http://127.0.0.1:11311") == {
        "/doubles/table_left/command": ("/old_planner",)
    }


def test_transport_shadow_filter_and_active_site_gates():
    planner = script("run_v7_planner.py")
    stage = {"valid": True, "planned_active": False, "command": {"role": "stage"}}
    assert planner.should_publish("transport-shadow", stage)
    assert not planner.should_publish("transport-shadow", {**stage, "valid": False})
    assert not planner.should_publish("transport-shadow", {**stage, "command": {"role": "hold"}})
    assert not planner.should_publish("shadow", stage)
    assert planner.should_publish("active", {"valid": False, "command": {"role": "hold"}})
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/start_v7_workstation.sh"), "check", "active", "--without-predictor", "--without-monitor"],
        text=True, capture_output=True,
    )
    assert result.returncode != 0 and "site-safety-confirmed" in result.stderr
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/start_v11_robots.sh"), "check", "active"],
        text=True, capture_output=True,
    )
    assert result.returncode != 0 and "site-safety-confirmed" in result.stderr


def test_robot_wrapper_stops_new_session_if_live_attestation_fails():
    source = (ROOT / "scripts/start_v11_robots.sh").read_text()
    failure = source.index("V11 attestation failed")
    stop = source.index('"$launcher" stop || true', failure)
    exit_after_stop = source.index('exit "$status"', stop)
    assert failure < stop < exit_after_stop


def test_robot_wrapper_requires_actual_dual_torso_samples_before_delegate():
    source = (ROOT / "scripts/start_v11_robots.sh").read_text()
    ros_master = source.index("export ROS_MASTER_URI=http://192.168.123.165:11311")
    left = source.index("require_live_torso_sample /doubles/table_left/torso_pose_origin")
    right = source.index("require_live_torso_sample /doubles/table_right/torso_pose_origin")
    prestart = source.index("V11 pre-start asset attestation failed")
    delegate = source.index('"$launcher" "$action" "$mode" normal off 1.0 v11-teacher')
    assert "timeout 3 rostopic echo -n 1" in source
    assert "create_v11_attestation.py" in source
    assert ros_master < left < right < prestart < delegate


def phase_state(robot, phase, sequence, *, session="s", target=0.4, applied=7):
    return {
        "robot": robot, "phase": phase, "sequence": sequence, "valid": True,
        "ready": True, "base_position_xyz": [-0.4, target, 0.75],
        "target_base_y": target, "last_planner_session_id": session,
        "last_applied_sequence": applied, "transport_error": "", "emergency_stop": False,
    }


def test_monitor_ignores_transport_error_from_an_older_stage_sequence():
    planner = {
        "session_id": "s",
        "staging": {
            "sequence": {"table_left": 7},
            "targets": {"table_left": 0.4},
        },
    }
    payload = phase_state("table_left", "HOME_HOLD", 20)
    payload.update(
        transport_error="planner_command_invalid",
        transport_error_sequence=6,
        transport_error_session_id="s",
    )
    stale = monitor._state_context(payload, planner=planner)
    assert stale["ack"] is True
    assert stale["transport_error_relevant"] is False
    assert stale["abnormal_reason"] is None

    payload["transport_error_sequence"] = 7
    current = monitor._state_context(payload, planner=planner)
    assert current["transport_error_relevant"] is True
    assert current["abnormal_reason"] == "planner_command_invalid"


def test_live_phase_flow_expected_next_stale_unknown_and_data_quality(monkeypatch, tmp_path):
    clock = [100.0]
    monkeypatch.setattr(monitor.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(monitor.time, "time", lambda: 1_800_000_000.0 + clock[0])
    collector = Collector(RecordingStore(tmp_path))
    collector.ingest("planner", {
        "session_id": "s", "staging": {
            "shot_id": "shot-1", "sequence": {"table_left": 7},
            "targets": {"table_left": 0.4},
        },
    })
    collector.ingest("command/table_left", {
        "robot": "table_left", "shot_id": "shot-1", "command": {"role": "hit"},
    })
    expected = {
        "HOME_HOLD": "HIT", "HIT": "POST_DELAY", "POST_DELAY": "OUTWARD",
        "OUTWARD": "OUTWARD_HOLD", "OUTWARD_HOLD": "RETURN", "RETURN": "HOME_HOLD",
    }
    for sequence, (phase, following) in enumerate(expected.items(), start=10):
        clock[0] += 0.02
        collector.ingest("state/table_left", phase_state("table_left", phase, sequence))
        flow = collector.snapshot()["phase_flow"]["robots"]["table_left"]
        assert flow["current_state"] == phase
        assert flow["next_state"] == following
        assert flow["next_state_kind"] == "expected"
    clock[0] += 0.30
    assert collector.snapshot()["phase_flow"]["robots"]["table_left"]["current_state"] == "STALE"
    clock[0] += 0.01
    collector.ingest("state/table_left", phase_state("table_left", "CALIBRATING", 20))
    clock[0] += 0.01
    collector.ingest("state/table_left", phase_state("table_left", "HOME_HOLD", 19))
    clock[0] += 0.01
    collector.ingest("state/table_left", phase_state("table_left", "HOME_HOLD", 1, session="s2"))
    snapshot = collector.snapshot()
    quality = snapshot["topics"]["state/table_left"]
    assert quality["sequence_gaps"] >= 4
    assert quality["sequence_resets"] >= 1
    assert quality["session_switches"] == 1
    segments = snapshot["phase_flow"]["robots"]["table_left"]["segments"]
    assert any(item["phase"] == "STALE" for item in segments)
    assert any(item["phase"] == "UNKNOWN" and "unknown_phase" in item["abnormal_reason"] for item in segments)


def test_recorded_phase_index_six_stages_async_long_hold_stale_and_unknown():
    start = 10.0
    events = [
        {"topic": "planner", "monotonic_s": start, "payload": {"session_id": "s", "staging": {"shot_id": "shot", "sequence": {"table_left": 5, "table_right": 8}, "targets": {"table_left": 0.4, "table_right": -0.3}}}},
        {"topic": "command/table_left", "monotonic_s": start, "payload": {"robot": "table_left", "shot_id": "shot", "command": {"role": "hit"}}},
        {"topic": "command/table_right", "monotonic_s": start, "payload": {"robot": "table_right", "shot_id": "shot", "command": {"role": "outward"}}},
    ]
    left = [
        (0.00, "HOME_HOLD"), (0.10, "HIT"), (0.20, "POST_DELAY"),
        (0.30, "OUTWARD"), (0.40, "OUTWARD_HOLD"), (0.60, "OUTWARD_HOLD"),
        (0.80, "OUTWARD_HOLD"), (1.00, "OUTWARD_HOLD"), (1.10, "RETURN"),
        (1.20, "HOME_HOLD"), (1.90, "CALIBRATING"),
    ]
    right = [
        (0.05, "HOME_HOLD"), (0.15, "HIT"), (0.25, "POST_DELAY"),
        (0.35, "OUTWARD"), (0.45, "OUTWARD_HOLD"), (0.65, "OUTWARD_HOLD"),
        (0.85, "OUTWARD_HOLD"), (1.05, "RETURN"), (1.15, "HOME_HOLD"),
    ]
    for robot, rows, applied, target in (
        ("table_left", left, 5, 0.4), ("table_right", right, 8, -0.3),
    ):
        for sequence, (offset, phase) in enumerate(rows, start=1):
            events.append({
                "topic": "state/" + robot, "monotonic_s": start + offset,
                "payload": phase_state(robot, phase, sequence, target=target, applied=applied),
            })
    events.sort(key=lambda event: event["monotonic_s"])
    index = build_recorded_phase_index(events, start, 2.4)
    left_segments = index["table_left"]
    phases = [item["phase"] for item in left_segments]
    assert all(phase in phases for phase in monitor.KNOWN_PHASES)
    assert "STALE" in phases and "UNKNOWN" in phases
    outward_hold = next(item for item in left_segments if item["phase"] == "OUTWARD_HOLD")
    assert outward_hold["duration_s"] == pytest.approx(0.7)
    assert left_segments[0]["next_actual_state"] == "HIT"
    assert index["table_right"][0]["enter_time_s"] == pytest.approx(0.05)
    assert any(item["phase"] == "STALE" for item in index["table_right"])


def test_monitor_records_frames_phase_endpoint_and_has_no_control_api(tmp_path):
    recording = RecordingStore(tmp_path / "recordings")
    collector = Collector(recording)
    server = ThreadingHTTPServer(("127.0.0.1", 0), handler(collector, recording))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"

    def request(path, method="GET"):
        data = b"{}" if method == "POST" else None
        with urlopen(Request(base + path, data=data, method=method), timeout=3) as response:
            return json.loads(response.read())
    try:
        started = request("/api/record/start", "POST")
        collector.ingest("planner", {
            "planner_identity": PLANNER_IDENTITY, "filter_mode": "frozen_v7_cbf",
            "rl_decision": {"actor_observation": [0] * 29, "safe_observation": [0] * 87},
        })
        for robot, target in (("table_left", 0.4), ("table_right", -0.3)):
            collector.ingest("state/" + robot, phase_state(robot, "HOME_HOLD", 1, target=target))
        collector.snapshot(sample=True)
        stopped = request("/api/record/stop", "POST")
        assert stopped["messages"] == 3 and stopped["frames"] == 1 and stopped["dropped"] == 0
        replay = request(f"/api/recordings/{started['id']}/frame?time=0")
        assert replay["topics"]["planner"]["payload"]["filter_mode"] == "frozen_v7_cbf"
        phases = request(f"/api/recordings/{started['id']}/phases")
        assert phases["schema"] == "v7-monitor-recorded-phases-v1"
        assert phases["robots"]["table_left"][0]["phase"] == "HOME_HOLD"
        with urlopen(base + "/monitor.css") as response:
            assert response.status == 200 and b"phase-stale" in response.read()
        with pytest.raises(Exception):
            request("/api/robot/start", "POST")
        with pytest.raises(Exception):
            request("/api/recordings/../../etc/passwd/messages")
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_monitor_summarizes_ack_decisions_and_fresh_torso(tmp_path):
    collector = Collector(RecordingStore(tmp_path))
    collector.ingest("torso/table_left", {"position_xyz": [-0.5, 0.8, 0.75]})
    collector.ingest("planner", {
        "session_id": "s", "reason": "ready", "cycle_hitter": "table_right",
        "staging": {"shot_id": "shot-1", "complete": True, "acknowledged": ["table_left", "table_right"]},
        "rl_decision": {
            "session_id": "s", "sequence": 1, "shot_id": "shot-1",
            "actor_names": [f"a{i}" for i in range(29)], "actor_observation": [0.0] * 29,
            "safe_names": [f"v{i}" for i in range(87)], "safe_observation": [0.0] * 87,
            "decision": {"actor_raw": [0.2, -0.2], "actor_physical_m": [0.18, -0.18], "cbf_projected_m": [0.25, -0.25], "applied_pair_m": [0.25, -0.25], "valid": True},
        },
    })
    snapshot = collector.snapshot(sample=True)
    assert snapshot["decisions"][0]["shot_id"] == "shot-1"
    assert snapshot["transitions"][-1]["detail"] == "ACK table_left, table_right"
    assert snapshot["timeline"][-1]["robots"]["table_left"]["y"] == pytest.approx(0.8)
    assert trace(snapshot)["robots"]["table_right"]["y"] is None


def test_monitor_serializes_torso_pose_for_recording():
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 123.25), frame_id="origin"),
        pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            orientation=SimpleNamespace(x=0.1, y=0.2, z=0.3, w=0.9),
        ),
    )
    assert pose_payload(message) == {
        "frame_id": "origin", "source_timestamp_s": 123.25,
        "position_xyz": [1.0, 2.0, 3.0], "orientation_xyzw": [0.1, 0.2, 0.3, 0.9],
    }


def test_only_v7_actor_and_frozen_barrier_assets_are_bundled():
    expected = {
        "v7_planner_actor.onnx", "v7_planner_actor_jit.pt", "model_190.pt",
        "frozen_v7_barrier_risk.onnx", "frozen_cbf_v7.pt", "handoff_schema.json",
        "runner_config.json", "model_contract.json",
    }
    assert {path.name for path in (ROOT / "models").iterdir()} == expected
    names = [
        path.name.lower() for path in ROOT.rglob("*")
        if path.is_file() and "__pycache__" not in path.parts
    ]
    assert not any("safe_filter_v3" in name or "actor_only" in name for name in names)


def test_offline_two_ball_replay_latches_and_never_publishes(tmp_path):
    module = load_program("tools", "replay_v7.py")
    source = ROOT / "tests/fixtures/v7_two_ball.jsonl"
    output = tmp_path / "replay.jsonl"
    summary_path = tmp_path / "summary.json"
    summary = module.replay(source, output, summary_path, benchmark_iterations=60)
    assert summary["input_events"] == 4 and summary["distinct_shots"] == 2
    assert summary["decisions_executed"] == 2 and summary["duplicate_shot_events"] == 2
    assert summary["commands_published"] == 0 and summary["latched_targets_stable"]
    assert summary["benchmark"]["candidate_count"] == 2603
    assert summary["benchmark"]["passed"]
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert [item["inference_executed"] for item in records] == [True, False, True, False]
    assert all(len(item["actor_observation"]) == 29 for item in records)
    assert all(len(item["safe_observation"]) == 87 for item in records)
    assert all(item["decision"]["filter_mode"] == "frozen_v7_cbf" for item in records)
    assert records[0]["decision"] == records[1]["decision"]
    assert records[2]["decision"] == records[3]["decision"]


@pytest.mark.parametrize("width,height", [(1280, 900), (390, 844)])
def test_monitor_browser_desktop_and_narrow_phase_replay(tmp_path, width, height):
    chrome = shutil.which("google-chrome") or shutil.which("chromium")
    if not chrome:
        pytest.skip("headless Chrome unavailable")
    recording = RecordingStore(tmp_path / "recordings")
    collector = Collector(recording)
    started = recording.start()
    collector.ingest("planner", {
        "planner_identity": PLANNER_IDENTITY, "mode": "shadow", "session_id": "s",
        "filter_mode": "frozen_v7_cbf", "reason": "ok",
        "staging": {"shot_id": "shot-1", "complete": True, "acknowledged": ["table_left", "table_right"]},
        "rl_decision": {
            "session_id": "s", "sequence": 1, "shot_id": "shot-1",
            "actor_names": [f"a{i}" for i in range(29)], "actor_observation": [0.0] * 29,
            "safe_names": [f"s{i}" for i in range(87)], "safe_observation": [0.0] * 87,
            "decision": {
                "actor_raw": [0.2, -0.2], "actor_physical_m": [0.18, -0.18],
                "cbf_projected_m": [0.25, -0.25], "cbf_nominal_risk": 0.42,
                "cbf_selected_risk": 0.12, "cbf_member_risk": [0.1, 0.11, 0.12],
                "cbf_candidate_count": 2603, "cbf_candidate_radius_normalized": 1.25,
                "applied_pair_m": [0.25, -0.25], "cbf_intervened": True,
                "hard_guard_intervened": False, "guard_intervened": True,
                "filter_mode": "frozen_v7_cbf", "latency_ms": 7.1, "valid": True,
            },
        },
        "fixed_commands": {
            "table_left": {"role": "hold", "desired_base_position": [-0.4, 0.7]},
            "table_right": {"role": "reserved_hitter", "desired_base_position": [-0.4, -0.35]},
        },
        "robots": {"table_left": {"role": "stage"}, "table_right": {"role": "stage"}},
    })
    for name, y in (("table_left", 0.7), ("table_right", -0.6)):
        collector.ingest("state/" + name, phase_state(name, "HOME_HOLD", 1, target=y, applied=1))
    collector.snapshot(sample=True)
    time.sleep(0.11)
    for name, y in (("table_left", 0.7), ("table_right", -0.6)):
        collector.ingest("state/" + name, phase_state(name, "HIT", 2, target=y, applied=2))
    collector.snapshot(sample=True)
    recording.stop()
    base = handler(collector, recording)
    static = ROOT / "src/yichao_v7_v11/static/monitor.html"
    instrumentation = f"""
<script>
window.__errors=[];window.addEventListener('error',e=>window.__errors.push(e.message));
setTimeout(async()=>{{let result={{errors:window.__errors,width:innerWidth,overflow:document.documentElement.scrollWidth-innerWidth}};
try{{document.getElementById('recordings').value={json.dumps(started['id'])};await document.getElementById('load-recording').onclick();
result.title=document.querySelector('h1').textContent;result.replay=document.getElementById('view-mode').textContent;
result.fixed=document.getElementById('fixed-y-left').textContent;result.applied=document.getElementById('applied-y-left').textContent;
result.mainValues=document.querySelectorAll('.decision-panel .planner-values strong').length;
result.position=document.getElementById('robot-left').textContent.includes('+0.700');
result.feature29=document.getElementById('actor-features').textContent.includes('a28');result.feature87=document.getElementById('safe-features').textContent.includes('s86');
result.labels=document.querySelector('.decision-panel').textContent.includes('Rule-based command')&&document.querySelector('.decision-panel').textContent.includes('RL Planner command')&&document.querySelector('.decision-panel').textContent.includes('实时位置');
result.phaseSegments=document.querySelectorAll('.phase-segment').length;result.phaseMode=document.getElementById('phase-mode').textContent;
const scroll=document.querySelector('.phase-flow-scroll');result.phaseScroll=scroll.scrollWidth-scroll.clientWidth;result.phaseTab=scroll.getAttribute('tabindex');result.phaseAria=scroll.getAttribute('aria-label');
document.getElementById('phase-range').click();result.phaseFull=document.getElementById('phase-mode').textContent;
document.getElementById('scrub').value='0.1';await document.getElementById('scrub').oninput();result.scrub=document.getElementById('play-time').textContent;
}}catch(error){{result.exception=String(error)}}let marker=document.createElement('pre');marker.id='browser-result';marker.textContent=JSON.stringify(result);document.body.appendChild(marker)}},1200);
</script>
"""

    class BrowserHandler(base):
        def do_GET(self):
            if self.path != "/":
                return super().do_GET()
            body = static.read_text().replace("</html>", instrumentation + "</html>").encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server = ThreadingHTTPServer(("127.0.0.1", 0), BrowserHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        result = subprocess.run(
            [
                chrome, "--headless=new", "--no-sandbox", "--disable-gpu",
                "--disable-dev-shm-usage", "--disable-background-networking",
                "--disable-component-update", "--no-first-run", "--dump-dom",
                "--virtual-time-budget=4000", f"--window-size={width},{height}",
                f"--user-data-dir={tmp_path / ('chrome-' + str(width))}",
                f"http://127.0.0.1:{server.server_port}/",
            ],
            text=True, capture_output=True, timeout=20, check=True,
        )
        marker = re.search(r'<pre id="browser-result">(.*?)</pre>', result.stdout, re.S)
        assert marker, result.stderr[-1000:]
        report = json.loads(html.unescape(marker.group(1)))
        assert not report.get("errors") and not report.get("exception"), report
        assert report["overflow"] <= 1 and "Frozen CBF" in report["title"]
        assert "历史回放" in report["replay"] and report["feature29"] and report["feature87"]
        assert report["fixed"] == "+0.700" and report["applied"] == "+0.250"
        assert report["mainValues"] == 6 and report["position"] and report["labels"]
        assert report["phaseSegments"] >= 2 and "下一实际状态" in report["phaseMode"]
        assert "回放全程" in report["phaseFull"] and report["phaseTab"] == "0"
        assert report["phaseAria"] and "/" in report["scrub"]
        if width == 390:
            assert report["phaseScroll"] > 0
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
