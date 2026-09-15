from __future__ import annotations

import sys
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_ROOT))

import utils.planner_ros_bridge as planner_ros_bridge_module
from utils.planner_ros_bridge import COMMAND_SCHEMA, PlannerRosBridge


class FakeScheduler:
    def __init__(self):
        self.calls = []
        self.pending_outward_target = None
        self.strike_time = 0.4
        self.state = SimpleNamespace(name="HOME_HOLD")
        self.motion_config_calls = []

    def set_external_outward_target(self, target_y):
        self.pending_outward_target = float(target_y)
        self.calls.append(("outward", float(target_y)))

    def set_external_home_target(self, target_y):
        self.calls.append(("home", float(target_y)))

    def set_external_hit(self, target, velocity, tts, ball_velocity):
        self.pending_outward_target = None
        self.state = SimpleNamespace(name="HIT")
        self.calls.append(
            (
                "hit",
                np.asarray(target).copy(),
                np.asarray(velocity).copy(),
                float(tts),
                np.asarray(ball_velocity).copy(),
            )
        )

    def set_external_base_target(
        self,
        target,
        return_target_y=None,
        safety_override=False,
        semantic_phase=None,
    ):
        self.calls.append(
            (
                "base",
                np.asarray(target).copy(),
                return_target_y,
                bool(safety_override),
                semantic_phase,
            )
        )

    def stage_runtime_motion_config(self, **kwargs):
        self.motion_config_calls.append(kwargs)

    def runtime_motion_config_status(self):
        return {
            "active": {},
            "pending": None,
            "outward_applied_command_id": None,
            "home_applied_command_id": None,
        }


def _bridge(payload, external_hit_command_mode="frozen"):
    bridge = PlannerRosBridge.__new__(PlannerRosBridge)
    bridge.robot_id = "table_left"
    bridge.command_timeout_s = 0.25
    bridge.external_hit_command_mode = external_hit_command_mode
    bridge._lock = threading.Lock()
    bridge._latest_command = payload
    bridge._latest_receive_monotonic = time.monotonic()
    bridge._last_applied_sequence = -1
    bridge._last_commit_token = None
    bridge._last_session_id = None
    bridge._last_error = ""
    bridge._last_error_sequence = None
    bridge._last_error_session_id = None
    bridge._last_command_sequence = -1
    bridge._last_command_age_s = np.nan
    bridge._last_raw_tts = np.nan
    bridge._last_streamed_tts = np.nan
    bridge._last_stream_update = False
    bridge._last_prediction_source_timestamp_s = np.nan
    bridge._last_prediction_age_s = np.nan
    bridge._last_motion_config_command_id = None
    bridge._last_motion_config_payload = None
    bridge._motion_config_received_id = None
    bridge._motion_config_error = ""
    return bridge


def _command(sequence=1, role="hit", active=True, planner_mode="cbf"):
    return {
        "schema_version": COMMAND_SCHEMA,
        "planner_mode": planner_mode,
        "robot": "table_left",
        "sequence": sequence,
        "session_id": "planner-session-1",
        "valid": True,
        "commit_token": "relay-1",
        "safety_active": False,
        "safety_override": False,
        "return_target_y": 0.35,
        "post_hit_outward_y": 0.9125,
        "prediction_source_timestamp_s": time.time(),
        "command": {
            "role": role,
            "active": active,
            "predicted_ball_position": [0.45, 0.2, 1.0],
            "predicted_ball_velocity": [-3.0, 0.1, -0.5],
            "predicted_ball_predict_time": 0.4,
            "predicted_racket_velocity": [3.0, 0.0, 0.5],
            "desired_base_position": [0.0, 0.7],
            "trajectory_base_position": [0.0, 0.8],
        },
    }


def _runtime_motion_config(command_id="cfg-1"):
    return {
        "schema": "v10-runtime-motion-config-v1",
        "command_id": command_id,
        "published_at": 123.0,
        "robot_id": "table_left",
        "goal_x": 0.42,
        "outward_y": 0.90,
        "home_y": 0.20,
        "outward_hold_s": 5.0,
    }


def _consume_hit_start(bridge, scheduler, pelvis):
    update = bridge.apply_pending(scheduler, pelvis)
    assert update is not None and update.starts_hit
    scheduler.state = SimpleNamespace(name="HIT")
    scheduler.strike_time = update.streamed_tts
    return update


def test_hit_commit_is_exactly_once_and_returns_handoff_targets():
    scheduler = FakeScheduler()
    bridge = _bridge(_command())
    pelvis = np.array([0.1, 0.35, 0.75], dtype=np.float32)
    update = _consume_hit_start(bridge, scheduler, pelvis)
    assert bridge.apply_pending(scheduler, pelvis) is None
    assert np.isclose(update.post_hit_outward_y, 0.9125)
    assert np.isclose(update.return_target_y, 0.35)
    assert scheduler.calls == []
    assert bridge.last_commit_token == "relay-1"


def test_runtime_motion_config_is_staged_once_before_command_application():
    payload = _command()
    payload["runtime_motion_config"] = _runtime_motion_config()
    scheduler = FakeScheduler()
    bridge = _bridge(payload)
    pelvis = np.array([0.1, 0.35, 0.75], dtype=np.float32)
    _consume_hit_start(bridge, scheduler, pelvis)
    assert scheduler.motion_config_calls == [
        {
            "command_id": "cfg-1",
            "goal_x": 0.42,
            "outward_y": 0.90,
            "home_y": 0.20,
            "outward_hold_s": 5.0,
        }
    ]
    payload = _command(sequence=2)
    payload["runtime_motion_config"] = _runtime_motion_config()
    bridge._latest_command = payload
    bridge._latest_receive_monotonic = time.monotonic()
    bridge.apply_pending(scheduler, pelvis)
    assert len(scheduler.motion_config_calls) == 1

    payload = _command(sequence=3)
    payload["runtime_motion_config"] = {
        **_runtime_motion_config(),
        "goal_x": 0.99,
    }
    bridge._latest_command = payload
    bridge._latest_receive_monotonic = time.monotonic()
    bridge.apply_pending(scheduler, pelvis)
    assert "command_id was reused" in bridge._motion_config_error


def test_runtime_motion_config_is_staged_even_for_shadow_invalid_command():
    payload = _command(role="hold", active=False, planner_mode="fixed_relay")
    payload["valid"] = False
    payload["runtime_motion_config"] = _runtime_motion_config("shadow-config")
    scheduler = FakeScheduler()
    bridge = _bridge(payload)
    bridge.apply_pending(scheduler, np.array([0.1, 0.35, 0.75], dtype=np.float32))
    assert scheduler.motion_config_calls[0]["command_id"] == "shadow-config"
    assert bridge._motion_config_received_id == "shadow-config"


def test_frozen_mode_ignores_new_sequences_for_same_commit_token():
    scheduler = FakeScheduler()
    bridge = _bridge(_command(), external_hit_command_mode="frozen")
    pelvis = np.array([0.1, 0.35, 0.75], dtype=np.float32)
    _consume_hit_start(bridge, scheduler, pelvis)
    update = _command(sequence=2)
    update["command"]["predicted_ball_position"] = [0.45, -0.3, 1.1]
    bridge._latest_command = update
    bridge._latest_receive_monotonic = time.monotonic()
    assert bridge.apply_pending(scheduler, pelvis) is None
    assert scheduler.calls == []
    assert not bridge.record_snapshot_fields()["planner_stream_update"]


def test_stream_mode_updates_same_token_without_restarting_hit():
    scheduler = FakeScheduler()
    bridge = _bridge(_command(), external_hit_command_mode="stream")
    pelvis = np.array([0.1, 0.35, 0.75], dtype=np.float32)
    _consume_hit_start(bridge, scheduler, pelvis)
    initial_calls = list(scheduler.calls)

    updates = []
    for sequence in range(2, 102):
        payload = _command(sequence=sequence)
        payload["command"]["predicted_ball_position"] = [
            0.45,
            -0.2 + sequence * 0.001,
            1.05,
        ]
        payload["command"]["predicted_racket_velocity"] = [
            3.0,
            0.1,
            0.4 + sequence * 0.001,
        ]
        payload["command"]["predicted_ball_predict_time"] = 0.4 - sequence * 0.002
        bridge._latest_command = payload
        bridge._latest_receive_monotonic = time.monotonic()
        update = bridge.apply_pending(scheduler, pelvis)
        assert update is not None
        updates.append(update)

    assert scheduler.calls == initial_calls
    assert len(updates) == 100
    assert updates[-1].sequence == 101
    assert np.allclose(updates[-1].racket_target, [0.45, -0.099, 1.05])
    assert np.allclose(updates[-1].target_velocity, [3.0, 0.1, 0.501])
    assert all(np.isfinite(update.streamed_tts) for update in updates)
    fields = bridge.record_snapshot_fields()
    assert fields["external_hit_command_mode"] == 1
    assert fields["planner_command_sequence"] == 101
    assert fields["planner_stream_update"] == 1
    assert fields["planner_commit_token"] == b"relay-1"


def test_stream_mode_accepts_new_predictor_tts_revision():
    scheduler = FakeScheduler()
    bridge = _bridge(_command(), external_hit_command_mode="stream")
    pelvis = np.array([0.1, 0.35, 0.75], dtype=np.float32)
    _consume_hit_start(bridge, scheduler, pelvis)

    scheduler.strike_time = 0.18
    payload = _command(sequence=2)
    payload["command"]["predicted_ball_predict_time"] = 0.35
    bridge._latest_command = payload
    bridge._latest_receive_monotonic = time.monotonic()
    update = bridge.apply_pending(scheduler, pelvis)

    assert update is not None
    assert update.streamed_tts > 0.30


def test_tts_subtracts_predictor_source_age_exactly_once(monkeypatch):
    monkeypatch.setattr(planner_ros_bridge_module.time, "time", lambda: 100.04)
    payload = _command()
    payload["prediction_source_timestamp_s"] = 100.0
    scheduler = FakeScheduler()
    bridge = _bridge(payload, external_hit_command_mode="stream")
    bridge._latest_receive_monotonic -= 0.02

    update = _consume_hit_start(
        bridge,
        scheduler,
        np.array([0.1, 0.35, 0.75], dtype=np.float32),
    )

    assert np.isclose(update.prediction_age_s, 0.04)
    assert np.isclose(update.streamed_tts, 0.36)
    assert update.command_age_s >= 0.02
    fields = bridge.record_snapshot_fields()
    assert np.isclose(fields["planner_prediction_age_ms"], 40.0)


def test_stream_mode_rejects_nonfinite_update_and_keeps_last_command():
    scheduler = FakeScheduler()
    bridge = _bridge(_command(), external_hit_command_mode="stream")
    pelvis = np.array([0.1, 0.35, 0.75], dtype=np.float32)
    _consume_hit_start(bridge, scheduler, pelvis)
    payload = _command(sequence=2)
    payload["command"]["predicted_racket_velocity"][1] = float("nan")
    bridge._latest_command = payload
    bridge._latest_receive_monotonic = time.monotonic()

    assert bridge.apply_pending(scheduler, pelvis) is None
    assert bridge._last_error.startswith("command_rejected:")
    assert not bridge.record_snapshot_fields()["planner_stream_update"]


def test_stream_mode_freezes_after_hit_phase():
    scheduler = FakeScheduler()
    bridge = _bridge(_command(), external_hit_command_mode="stream")
    pelvis = np.array([0.1, 0.35, 0.75], dtype=np.float32)
    _consume_hit_start(bridge, scheduler, pelvis)
    scheduler.state = SimpleNamespace(name="POST_DELAY")
    bridge._latest_command = _command(sequence=2)
    bridge._latest_receive_monotonic = time.monotonic()
    assert bridge.apply_pending(scheduler, pelvis) is None
    assert not bridge.record_snapshot_fields()["planner_stream_update"]


def test_clear_uses_terminal_goal_and_current_pelvis_x():
    scheduler = FakeScheduler()
    bridge = _bridge(_command(role="clear", active=False))
    bridge.apply_pending(scheduler, np.array([0.12, 0.35, 0.75], dtype=np.float32))
    assert scheduler.calls[0][0] == "base"
    assert np.allclose(scheduler.calls[0][1], [0.12, 0.8])
    assert scheduler.calls[0][2] == 0.35


def test_fixed_hold_does_not_change_scheduler():
    scheduler = FakeScheduler()
    bridge = _bridge(
        _command(role="hold", active=False, planner_mode="fixed_relay")
    )
    bridge.apply_pending(scheduler, np.array([0.12, 0.35, 0.75], dtype=np.float32))
    assert scheduler.calls == []
    assert bridge._last_error == ""


def test_fixed_return_uses_only_fixed_home_target():
    scheduler = FakeScheduler()
    payload = _command(role="return", active=False, planner_mode="fixed_relay")
    payload["return_target_y"] = 0.20
    payload["command"]["desired_base_position"] = [9.0, 8.0]
    payload["command"]["trajectory_base_position"] = [7.0, 6.0]
    bridge = _bridge(payload)
    bridge.apply_pending(scheduler, np.array([0.12, 0.35, 0.75], dtype=np.float32))
    assert len(scheduler.calls) == 1
    assert scheduler.calls[0][0] == "base"
    assert np.allclose(scheduler.calls[0][1], [0.12, 0.20])
    assert scheduler.calls[0][2] == 0.20
    assert not scheduler.calls[0][3]


def test_fixed_outward_uses_fixed_outward_target_and_keeps_home_target():
    scheduler = FakeScheduler()
    payload = _command(role="outward", active=False, planner_mode="fixed_relay")
    payload["return_target_y"] = 0.20
    payload["post_hit_outward_y"] = 0.70
    bridge = _bridge(payload)
    bridge.apply_pending(scheduler, np.array([0.12, 0.35, 0.75], dtype=np.float32))
    assert len(scheduler.calls) == 1
    assert scheduler.calls[0][0] == "base"
    assert np.allclose(scheduler.calls[0][1], [0.12, 0.70])
    assert scheduler.calls[0][2] == 0.20
    assert bridge._last_error == ""


def test_native_fixed_return_passes_explicit_return_semantics():
    scheduler = FakeScheduler()
    scheduler.native_no_mirror = True
    payload = _command(role="return", active=False, planner_mode="fixed_relay")
    payload["return_target_y"] = -0.20
    bridge = _bridge(payload)
    bridge.apply_pending(scheduler, np.array([0.12, -0.80, 0.75], dtype=np.float32))
    assert len(scheduler.calls) == 1
    assert scheduler.calls[0][0] == "base"
    assert scheduler.calls[0][4] == "RETURN"


def test_fixed_mode_rejects_legacy_motion_roles():
    for role in ("clear", "exit", "stage", "reserved_hitter"):
        scheduler = FakeScheduler()
        bridge = _bridge(
            _command(role=role, active=False, planner_mode="fixed_relay")
        )
        bridge.apply_pending(
            scheduler, np.array([0.12, 0.35, 0.75], dtype=np.float32)
        )
        assert scheduler.calls == [], role
        assert bridge._last_error.startswith("command_rejected:"), role


def test_stale_command_never_reaches_scheduler():
    scheduler = FakeScheduler()
    bridge = _bridge(_command())
    bridge._latest_receive_monotonic -= 1.0
    bridge.apply_pending(scheduler, np.array([0.0, 0.35, 0.75], dtype=np.float32))
    assert scheduler.calls == []
    assert bridge._last_error == "stale_command"
    assert bridge._last_error_sequence == 1
    assert bridge._last_error_session_id == "planner-session-1"


def test_invalid_command_error_is_sequence_correlated_and_next_valid_clears_it():
    payload = _command(role="hold", active=False, planner_mode="fixed_relay")
    payload["valid"] = False
    scheduler = FakeScheduler()
    bridge = _bridge(payload)
    pelvis = np.array([0.0, 0.35, 0.75], dtype=np.float32)
    bridge.apply_pending(scheduler, pelvis)
    assert bridge._last_error == "planner_command_invalid"
    assert bridge._last_error_sequence == 1
    assert bridge._last_error_session_id == "planner-session-1"

    bridge._latest_command = _command(
        sequence=2, role="hold", active=False, planner_mode="fixed_relay"
    )
    bridge._latest_receive_monotonic = time.monotonic()
    bridge.apply_pending(scheduler, pelvis)
    assert bridge._last_error == ""
    assert bridge._last_error_sequence is None
    assert bridge._last_error_session_id is None


def test_new_planner_session_resets_sequence_and_commit_latch():
    scheduler = FakeScheduler()
    bridge = _bridge(_command(sequence=100))
    pelvis = np.array([0.0, 0.35, 0.75], dtype=np.float32)
    bridge.apply_pending(scheduler, pelvis)
    assert bridge.last_applied_sequence == 100

    restarted = _command(sequence=1)
    restarted["session_id"] = "planner-session-2"
    restarted["commit_token"] = "relay-after-restart"
    bridge._latest_command = restarted
    bridge._latest_receive_monotonic = time.monotonic()
    bridge.apply_pending(scheduler, pelvis)
    assert bridge.last_applied_sequence == 1
    assert bridge.last_commit_token == "relay-after-restart"
