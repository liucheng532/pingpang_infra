import importlib.util
import html
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import re
import shutil
import subprocess
import sys
import threading
import time
from types import SimpleNamespace
from urllib.request import Request, urlopen

import pytest

from yichao_v6r10_v11 import ROOT
from yichao_v6r10_v11.attestation import ATTESTATION_SCHEMA, verify_attestation
from yichao_v6r10_v11.monitor import Collector, RecordingStore, handler, pose_payload, trace


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
        "schema": ATTESTATION_SCHEMA, "planner_identity": "v6r10-model630-v11-i42500",
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


def test_attestation_rejects_stale_wrong_mode_and_active_shadow_argv(tmp_path):
    config = deployment()
    path = tmp_path / "attestation.json"
    path.write_text(json.dumps(attestation_record(config, mode="shadow")))
    with pytest.raises(ValueError):
        verify_attestation(path, expected_controller_mode="active", deployment=config, now=1050.0)
    with pytest.raises(ValueError, match="age"):
        verify_attestation(path, expected_controller_mode="shadow", deployment=config, now=2000.0)


def test_attestation_rejects_swapped_native_lcm_robot_identity(tmp_path):
    config = deployment()
    path = tmp_path / "attestation.json"
    record = attestation_record(config)
    record["robots"]["table_left"]["g1_control"]["environment"][
        "LCM_DEFAULT_URL"
    ] = "udpm://239.255.76.66:7667?ttl=0"
    path.write_text(json.dumps(record))
    with pytest.raises(ValueError, match="LCM_DEFAULT_URL"):
        verify_attestation(
            path, expected_controller_mode="shadow", deployment=config, now=1050.0
        )


def test_launch_plans_fix_network_modes_residual_and_no_remote_io():
    workstation = subprocess.run(
        ["bash", str(ROOT / "scripts/start_v6r10_workstation.sh"), "dry-run", "shadow", "--without-predictor", "--without-monitor"],
        text=True, capture_output=True, check=True,
    )
    plan = json.loads(workstation.stdout)
    assert plan["network"]["ROS_MASTER_URI"] == "http://192.168.123.165:11311"
    assert plan["status_topic"] == "/doubles/yichao_v6r10/status"
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
    path.write_text(json.dumps({
        "controller_mode": "shadow", "live_processes_verified": True,
        "created_at": 1001.0,
    }))
    assert not module.fresh_active_attestation(path, 1000.0)
    path.write_text(json.dumps({
        "controller_mode": "active", "live_processes_verified": False,
        "created_at": 1001.0,
    }))
    assert not module.fresh_active_attestation(path, 1000.0)
    path.write_text(json.dumps({
        "controller_mode": "active", "live_processes_verified": True,
        "created_at": 999.0,
    }))
    assert not module.fresh_active_attestation(path, 1000.0)
    path.write_text(json.dumps({
        "controller_mode": "active", "live_processes_verified": True,
        "created_at": 1001.0,
    }))
    assert module.fresh_active_attestation(path, 1000.0)


def test_workstation_detects_old_and_new_planner_conflicts():
    module = script("run_workstation.py")
    table = "42 python3 scripts/run_v9_real_fixed_relay.py\n43 python3 unrelated.py\n"
    assert module.blockers(table) == ["42 python3 scripts/run_v9_real_fixed_relay.py"]


def test_workstation_rejects_existing_command_publishers(monkeypatch):
    module = script("run_workstation.py")

    class Master:
        def getSystemState(self, _caller):
            return 1, "ok", [
                [
                    ["/doubles/table_left/command", ["/old_planner"]],
                    ["/unrelated", ["/other"]],
                ],
                [],
                [],
            ]

    monkeypatch.setattr(module.xmlrpc.client, "ServerProxy", lambda _uri: Master())
    assert module.command_publishers("http://127.0.0.1:11311") == {
        "/doubles/table_left/command": ("/old_planner",)
    }


def test_transport_shadow_publish_filter_allows_only_valid_inactive_stage():
    planner = script("run_v6r10_planner.py")
    stage = {"valid": True, "planned_active": False, "command": {"role": "stage"}}
    assert planner.should_publish("transport-shadow", stage)
    assert not planner.should_publish(
        "transport-shadow", {**stage, "valid": False}
    )
    assert not planner.should_publish(
        "transport-shadow", {**stage, "command": {"role": "hold"}}
    )
    assert not planner.should_publish("shadow", stage)
    assert planner.should_publish("active", {"valid": False, "command": {"role": "hold"}})


def test_active_launcher_has_two_explicit_site_gates():
    result = subprocess.run(
        ["bash", str(ROOT / "scripts/start_v6r10_workstation.sh"), "check", "active", "--without-predictor", "--without-monitor"],
        text=True, capture_output=True,
    )
    assert result.returncode != 0
    assert "site-safety-confirmed" in result.stderr
    robots = subprocess.run(
        ["bash", str(ROOT / "scripts/start_v11_robots.sh"), "check", "active"],
        text=True, capture_output=True,
    )
    assert robots.returncode != 0
    assert "site-safety-confirmed" in robots.stderr


def test_robot_wrapper_stops_new_session_if_live_attestation_fails():
    source = (ROOT / "scripts/start_v11_robots.sh").read_text()
    failure = source.index("V11 attestation failed")
    stop = source.index('"$launcher" stop || true', failure)
    exit_after_stop = source.index('exit "$status"', stop)
    assert failure < stop < exit_after_stop


def test_monitor_records_messages_frames_replays_and_has_no_control_api(tmp_path):
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
        collector.ingest(
            "planner",
            {
                "planner_identity": "v6r10-model630-v11-i42500", "filter_mode": "off",
                "barrier_risk": "not_evaluated", "rl_decision": {"actor_observation": [0] * 36, "audit_observation": [0] * 87},
            },
        )
        collector.snapshot(sample=True)
        stopped = request("/api/record/stop", "POST")
        assert stopped["messages"] == 1 and stopped["frames"] == 1 and stopped["dropped"] == 0
        replay = request(f"/api/recordings/{started['id']}/frame?time=0")
        assert replay["topics"]["planner"]["payload"]["filter_mode"] == "off"
        with urlopen(base + "/monitor.css") as response:
            assert response.status == 200 and b"--teal" in response.read()
        with pytest.raises(Exception):
            request("/api/robot/start", "POST")
        with pytest.raises(Exception):
            request("/api/recordings/../../etc/passwd/messages")
    finally:
        server.shutdown()
        server.server_close()
        thread.join()


def test_monitor_summarizes_v11_ack_events_decisions_and_fresh_torso(tmp_path):
    collector = Collector(RecordingStore(tmp_path))
    collector.ingest("torso/table_left", {"position_xyz": [-0.5, 0.8, 0.75]})
    collector.ingest("planner", {
        "session_id": "s", "reason": "ready", "cycle_hitter": "table_right",
        "staging": {"shot_id": "shot-1", "complete": True,
                    "acknowledged": ["table_left", "table_right"]},
        "rl_decision": {
            "session_id": "s", "sequence": 1, "shot_id": "shot-1",
            "actor_names": [f"a{i}" for i in range(36)],
            "actor_observation": [0.0] * 36,
            "audit_names": [f"v{i}" for i in range(87)],
            "audit_observation": [0.0] * 87,
            "decision": {"actor_raw": [0.2, -0.2],
                         "actor_physical_m": [0.18, -0.18],
                         "applied_pair_m": [0.25, -0.25], "valid": True},
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
        "frame_id": "origin",
        "source_timestamp_s": 123.25,
        "position_xyz": [1.0, 2.0, 3.0],
        "orientation_xyzw": [0.1, 0.2, 0.3, 0.9],
    }


def test_no_barrier_or_filter_artifact_in_production_tree():
    names = [path.name.lower() for path in ROOT.rglob("*") if path.is_file()]
    assert "frozen_v1_barrier.pt" not in names
    assert not any(name.endswith("barrier.onnx") or "safe_filter" in name for name in names)


def test_offline_replay_records_actor_audit_and_never_publishes(tmp_path):
    module = load_program("tools", "replay_actor_only.py")
    source = ROOT / "tests/fixtures/release_199.jsonl"
    output = tmp_path / "replay.jsonl"
    summary_path = tmp_path / "summary.json"
    summary = module.replay(source, output, summary_path, benchmark_iterations=25)
    assert summary["distinct_shots"] == 2
    assert summary["commands_published"] == 0
    assert summary["benchmark"]["passed"]
    records = [json.loads(line) for line in output.read_text().splitlines()]
    assert all(len(item["actor_observation"]) == 36 for item in records)
    assert all(len(item["audit_observation"]) == 87 for item in records)
    assert all(item["filter_mode"] == "off" for item in records)
    assert all(item["barrier_risk"] == "not_evaluated" for item in records)


@pytest.mark.parametrize("width,height", [(1280, 900), (390, 844)])
def test_monitor_browser_desktop_and_narrow_replay(tmp_path, width, height):
    chrome = shutil.which("google-chrome") or shutil.which("chromium")
    if not chrome:
        pytest.skip("headless Chrome unavailable")
    recording = RecordingStore(tmp_path / "recordings")
    collector = Collector(recording)
    started = recording.start()
    collector.ingest(
        "planner",
        {
            "planner_identity": "v6r10-model630-v11-i42500", "mode": "shadow", "session_id": "s",
            "filter_mode": "off", "barrier_risk": "not_evaluated", "reason": "ok",
            "rl_decision": {
                "session_id": "s", "sequence": 1, "shot_id": "shot-1",
                "actor_names": [f"a{i}" for i in range(36)],
                "actor_observation": [0.0] * 36,
                "audit_names": [f"s{i}" for i in range(87)],
                "audit_observation": [0.0] * 87,
                "decision": {
                    "actor_raw": [0.2, -0.2], "actor_physical_m": [0.18, -0.18],
                    "guard_projected_m": [0.25, -0.25], "applied_pair_m": [0.25, -0.25],
                    "guard_intervened": True, "filter_mode": "off",
                    "barrier_risk": "not_evaluated", "latency_ms": 0.3, "valid": True,
                },
            },
            "fixed_commands": {
                "table_left": {"role": "hold", "desired_base_position": [-0.4, 0.7]},
                "table_right": {"role": "reserved_hitter", "desired_base_position": [-0.4, -0.35]},
            },
            "robots": {"table_left": {"role": "stage"}, "table_right": {"role": "stage"}},
        },
    )
    for name, y in (("table_left", 0.7), ("table_right", -0.6)):
        collector.ingest("state/" + name, {
            "sequence": 1, "valid": True, "ready": True,
            "base_position_xyz": [-0.4, y, 0.75], "target_base_y": y,
            "phase": "HOME_HOLD", "last_planner_session_id": "s",
            "last_applied_sequence": 1, "transport_error": "", "emergency_stop": False,
        })
    collector.snapshot(sample=True)
    time.sleep(0.11)
    collector.snapshot(sample=True)
    recording.stop()
    base = handler(collector, recording)
    static = ROOT / "src/yichao_v6r10_v11/static/monitor.html"
    instrumentation = f"""
<script>
window.__errors=[];window.addEventListener('error',e=>window.__errors.push(e.message));
setTimeout(async()=>{{let result={{errors:window.__errors,width:innerWidth,overflow:document.documentElement.scrollWidth-innerWidth}};
try{{document.getElementById('recordings').value={json.dumps(started['id'])};await document.getElementById('load-recording').onclick();
result.title=document.querySelector('h1').textContent;result.replay=document.getElementById('view-mode').textContent;
result.fixed=document.getElementById('fixed-y-left').textContent;result.applied=document.getElementById('applied-y-left').textContent;
result.mainValues=document.querySelectorAll('.decision-panel .planner-values strong').length;
result.position=document.getElementById('robot-left').textContent.includes('+0.700');
result.feature36=document.getElementById('actor-features').textContent.includes('a35');result.labels=document.querySelector('.decision-panel').textContent.includes('Rule-based command')&&document.querySelector('.decision-panel').textContent.includes('RL Planner command')&&document.querySelector('.decision-panel').textContent.includes('实时位置');
document.getElementById('scrub').value='0.1';await document.getElementById('scrub').oninput();result.scrub=document.getElementById('play-time').textContent;
}}catch(error){{result.exception=String(error)}}let marker=document.createElement('pre');marker.id='browser-result';marker.textContent=JSON.stringify(result);document.body.appendChild(marker)}},900);
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
                "--virtual-time-budget=3500", f"--window-size={width},{height}",
                f"--user-data-dir={tmp_path / ('chrome-' + str(width))}",
                f"http://127.0.0.1:{server.server_port}/",
            ],
            text=True, capture_output=True, timeout=20, check=True,
        )
        marker = re.search(r'<pre id="browser-result">(.*?)</pre>', result.stdout, re.S)
        assert marker, result.stderr[-1000:]
        report = json.loads(html.unescape(marker.group(1)))
        assert not report.get("errors") and not report.get("exception"), report
        assert report["overflow"] <= 1 and "Actor-only" in report["title"]
        assert "历史回放" in report["replay"] and report["feature36"] and report["labels"]
        assert report["fixed"] == "+0.700" and report["applied"] == "+0.250"
        assert report["mainValues"] == 6
        assert report["position"] and "/" in report["scrub"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join()
