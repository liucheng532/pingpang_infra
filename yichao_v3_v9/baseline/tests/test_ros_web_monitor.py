from __future__ import annotations

import json
import math
import pytest
import threading
import urllib.request
from types import SimpleNamespace

from doubles_planner.real_ros_runtime import V9RealFixedRelayRuntime

from ros_web_monitor.server import (
    DEFAULT_TOPICS,
    MonitorHttpServer,
    RotatingJsonlLog,
    STATIC_INDEX,
    TelemetryStore,
    _parse_topics,
    pose_stamped_payload,
    quaternion_xyzw_to_rpy_deg,
)


def test_default_topics_cover_doubles_state_command_ball_and_status() -> None:
    assert DEFAULT_TOPICS["left_state"] == "/doubles/table_left/state"
    assert DEFAULT_TOPICS["right_state"] == "/doubles/table_right/state"
    assert DEFAULT_TOPICS["left_torso"] == "/doubles/table_left/torso_pose_origin"
    assert DEFAULT_TOPICS["right_torso"] == "/doubles/table_right/torso_pose_origin"
    assert DEFAULT_TOPICS["left_command"] == "/doubles/table_left/command"
    assert DEFAULT_TOPICS["right_command"] == "/doubles/table_right/command"
    assert DEFAULT_TOPICS["ball"] == "/doubles/ball_prediction"
    assert {"fixed_status", "cbf_status"} <= set(DEFAULT_TOPICS)


def test_torso_pose_payload_exposes_xyz_and_rpy_degrees() -> None:
    half = math.pi / 4.0
    message = SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=0.1, y=-0.2, z=0.95),
            orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half)),
        ),
        header=SimpleNamespace(
            stamp=SimpleNamespace(to_sec=lambda: 123.5),
            frame_id="origin",
        ),
    )
    payload = pose_stamped_payload(message)
    assert payload["position"] == [0.1, -0.2, 0.95]
    assert payload["frame_id"] == "origin"
    assert payload["source_stamp_s"] == 123.5
    assert payload["rpy_deg"] == pytest.approx([0.0, 0.0, 90.0])
    assert quaternion_xyzw_to_rpy_deg([0.0, 0.0, 0.0, 0.0]) is None


def test_store_parses_string_json_and_exposes_snapshot() -> None:
    store = TelemetryStore({"left_state": "/doubles/table_left/state"})
    event = store.ingest(
        "left_state",
        json.dumps({"base_position_xyz": [0.0, 0.2, 0.75], "phase": "HOME_HOLD"}),
    )
    snapshot = store.snapshot()

    assert event["id"] == 1
    assert snapshot["sequence"] == 1
    assert snapshot["topics"]["left_state"]["data"]["phase"] == "HOME_HOLD"
    assert snapshot["topics"]["left_state"]["age_s"] >= 0.0


def test_store_submits_independent_runtime_motion_config() -> None:
    sent = []
    store = TelemetryStore(DEFAULT_TOPICS)
    store.set_motion_config_sender(sent.append)
    payload = store.submit_motion_config(
        "table_right",
        {
            "goal_x": 4.2,
            "outward_y": -0.90,
            "home_y": -0.20,
            "outward_hold_s": 5.0,
        },
    )
    assert payload["robot_id"] == "table_right"
    assert payload["goal_x"] == 4.2
    assert sent == [payload]
    snapshot = store.snapshot()
    assert snapshot["motion_config_submit"]["table_right"]["last_sent"] == payload
    assert snapshot["motion_config_submit"]["table_left"]["last_sent"] is None


def test_store_keeps_runtime_motion_config_error_until_next_submit() -> None:
    store = TelemetryStore(DEFAULT_TOPICS)
    store.set_motion_config_sender(lambda _payload: None)
    with pytest.raises(ValueError):
        store.submit_motion_config(
            "table_left",
            {
                "goal_x": 0.2,
                "outward_y": 0.3,
                "home_y": 0.2,
                "outward_hold_s": 3.0,
            },
        )
    assert store.snapshot()["motion_config_submit"]["table_left"]["error"]


def test_http_runtime_motion_config_routes_by_robot() -> None:
    sent = []
    store = TelemetryStore(DEFAULT_TOPICS)
    store.set_motion_config_sender(sent.append)
    server = MonitorHttpServer(("127.0.0.1", 0), store)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        body = json.dumps(
            {
                "goal_x": 0.31,
                "outward_y": -0.91,
                "home_y": -0.20,
                "outward_hold_s": 4.0,
            }
        ).encode()
        request = urllib.request.Request(
            f"http://127.0.0.1:{server.server_address[1]}/api/runtime_motion_config/table_right",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        result = json.load(urllib.request.urlopen(request, timeout=2.0))
        assert result["ok"] is True
        assert result["robot_id"] == "table_right"
        assert sent[0]["command_id"] == result["command_id"]
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


def test_monitor_payload_stages_in_fixed_planner() -> None:
    runtime = V9RealFixedRelayRuntime(shadow=True)
    store = TelemetryStore(DEFAULT_TOPICS)
    store.set_motion_config_sender(runtime.stage_runtime_motion_config)
    payload = store.submit_motion_config(
        "table_left",
        {
            "goal_x": 0.44,
            "outward_y": 0.91,
            "home_y": 0.20,
            "outward_hold_s": 6.0,
        },
    )
    status = runtime.runtime_motion_config_status()["table_left"]
    assert status["received_command_id"] == payload["command_id"]
    assert status["pending"]["goal_x"] == 0.44
    assert runtime.runtime_motion_config_status()["table_right"]["pending"] is None


def test_jsonl_log_writes_and_rotates(tmp_path) -> None:
    path = tmp_path / "monitor.jsonl"
    log = RotatingJsonlLog(path, max_bytes=80, backups=2)
    store = TelemetryStore({"ball": "/doubles/ball_prediction"}, log)
    for sequence in range(8):
        store.ingest("ball", {"sequence": sequence, "position": [1.0, 0.0, 1.0]})
    log.close()

    assert path.is_file()
    assert path.with_name("monitor.jsonl.1").is_file()
    latest = json.loads(path.read_text(encoding="utf-8").splitlines()[-1])
    assert latest["data"]["sequence"] == 7


def test_topic_overrides_are_incremental_and_dashboard_is_bundled() -> None:
    topics = _parse_topics(["ball=/custom/ball", "extra=/custom/status"], True)
    assert topics["left_state"] == DEFAULT_TOPICS["left_state"]
    assert topics["ball"] == "/custom/ball"
    assert topics["extra"] == "/custom/status"
    page = STATIC_INDEX.read_text(encoding="utf-8")
    assert "Doubles Planner Monitor" in page
    assert "Apply next move · 66" in page
    assert "Apply next move · 198" in page
    assert "goal x (canonical)" in page
    assert "startupX+canonicalGoal-startupCanonicalX" in page
    assert "/api/runtime_motion_config/" in page
    assert "outward target" in page
    assert "home target" in page
    assert "torso xyz" in page
    assert "torso RPY deg" in page
    assert 'id="right-torso"' in page
    assert 'id="left-rpy"' in page
    assert "backX: 0.7 + 2.74" in page
    assert "minY: -1.525 / 2" in page
    assert "ctx.fillRect(left, top, width, height)" in page
    assert "const plane=py(.7)" not in page
