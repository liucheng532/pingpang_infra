from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request

import pytest

from doubles_planner.real_ros_runtime import ROBOT_ORDER, V9RealFixedRelayRuntime
from doubles_planner.return_timing import RETURN_TIMING_SCHEMA, ReturnTiming, parse_return_timing
from ros_web_monitor.server import DEFAULT_TOPICS, MonitorHttpServer, TelemetryStore
from test_real_ros_runtime import ball_prediction, robot_state


def timing(robot="table_left", lead=100, mode="strike_time", command_id="timing-1", stamp=1.0):
    return dict(schema=RETURN_TIMING_SCHEMA, robot_id=robot, mode=mode, lead_ms=lead,
                command_id=command_id, published_at=stamp)


def start_cycle(hitter="table_right", lead=100, mode="strike_time", *, shadow=False):
    runtime = V9RealFixedRelayRuntime(shadow=shadow, session_id="return-test")
    peer = next(r for r in ROBOT_ORDER if r != hitter)
    runtime.stage_return_timing_config(timing(peer, lead, mode))
    for robot in ROBOT_ORDER:
        sign = 1 if robot == "table_left" else -1
        state = robot_state(robot, sign * (.35 if robot == hitter else .9))
        if robot == peer:
            state["phase"] = "OUTWARD_HOLD"
        runtime.update_robot_state(robot, state, 1.0)
    runtime.update_ball(ball_prediction(), 1.0)
    runtime.tick(now=1.0)
    assert runtime._swap_hitter == hitter
    return runtime, hitter, peer


def step(runtime, hitter, peer, tts, *, now=1.1, phase="HIT", peer_phase="OUTWARD_HOLD",
         token=None, fresh=True, valid=True, elapsed=1.0, peer_vy=0.0, sequence=2):
    for robot in (hitter, peer):
        prior = runtime._states[robot]
        state = robot_state(robot, prior["base_position_xyz"][1], sequence=sequence)
        state["phase"] = phase if robot == hitter else peer_phase
        state["state_elapsed_s"] = elapsed
        state["time_to_strike_s"] = tts if robot == hitter else -.5
        state["last_commit_token"] = (runtime._swap_commit_token if token is None else token)
        state["valid"] = valid
        state["last_planner_session_id"] = runtime.session_id
        state["base_linear_velocity_xyz"][1] = peer_vy if robot == peer else 0.0
        cycle = runtime.return_timing_status()[peer]["cycle"]
        state["last_applied_sequence"] = cycle["requested_sequence"] or 0
        runtime.update_robot_state(robot, state, now if fresh else now-.3)
    # Prediction may disappear after contact; cycle timing uses acknowledged robot TTS.
    ball = ball_prediction(sequence=sequence)
    ball.update(valid=False, shot_id=None, time_to_strike_s=-.5)
    runtime.update_ball(ball, now)
    return runtime.tick(now=now)


@pytest.mark.parametrize("lead", [-200, -100, 0, 100, 200])
@pytest.mark.parametrize("hitter", ROBOT_ORDER)
def test_thresholds_and_sides(lead, hitter):
    runtime, hitter, peer = start_cycle(hitter, lead)
    runtime._return_timing.profile.minimum_gap = lambda **kw: 1.0
    threshold = lead / 1000
    before = step(runtime, hitter, peer, threshold+.02, phase="HIT" if lead >= 0 else "POST_DELAY")
    assert before[peer]["command"]["role"] == "hold"
    after = step(runtime, hitter, peer, threshold, now=1.12, sequence=3,
                 phase="HIT" if lead >= 0 else "POST_DELAY")
    assert after[peer]["command"]["role"] == "return"
    assert not after[hitter]["command"]["active"]  # invalid ball cannot synthesize another HIT
    cycle = runtime.return_timing_status()[peer]["cycle"]
    assert cycle["trigger_reason"] == "strike_time"
    assert cycle["requested_at"] == 1.12
    assert after[peer]["post_hit_outward_y"] == runtime.config.outward_y[ROBOT_ORDER.index(peer)]


def test_pending_config_is_latched_only_on_returners_next_cycle():
    runtime, hitter, peer = start_cycle()
    runtime._return_timing.profile.minimum_gap = lambda **kw: 1.0
    runtime.stage_return_timing_config(timing(peer, -100, command_id="timing-2", stamp=2))
    runtime.stage_return_timing_config(timing(hitter, 200, command_id="other", stamp=2))
    result = step(runtime, hitter, peer, .1)
    assert result[peer]["command"]["role"] == "return"
    status = runtime.return_timing_status()
    assert status[peer]["active"]["lead_ms"] == 100
    assert status[peer]["pending"]["lead_ms"] == -100
    runtime._on_cycle_commit(hitter, runtime._swap_commit_token, 1.2)
    assert runtime.return_timing_status()[peer]["active"]["lead_ms"] == 100
    runtime._on_cycle_commit(hitter, "next-token", 2)
    assert runtime.return_timing_status()[peer]["active"]["lead_ms"] == -100
    assert runtime.return_timing_status()[hitter]["pending"]["lead_ms"] == 200


@pytest.mark.parametrize("mode", ["peer_outward", "strike_time"])
def test_legacy_trigger_and_path_rejection_fallback(mode):
    runtime, hitter, peer = start_cycle(mode=mode)
    runtime._return_timing.profile.minimum_gap = lambda **kw: .3
    early = step(runtime, hitter, peer, .08)
    assert early[peer]["command"]["role"] == "hold"
    outward = step(runtime, hitter, peer, -.2, phase="OUTWARD", now=1.3, sequence=3)
    assert outward[peer]["command"]["role"] == "return"
    assert runtime.return_timing_status()[peer]["cycle"]["trigger_reason"] == "peer_outward"


@pytest.mark.parametrize("kwargs,reason", [
    ({"token":"old-shot"}, "waiting_hit_ack"),
    ({"fresh":False}, "stale_or_invalid_feedback"),
    ({"valid":False}, "stale_or_invalid_feedback"),
    ({"elapsed":.1}, "waiting_turnaround"),
    ({"peer_vy":.5}, "waiting_turnaround"),
    ({"tts":-.5}, "invalid_hitter_tts"),
])
def test_early_admission_rejects_unusable_state(kwargs, reason):
    runtime, hitter, peer = start_cycle()
    runtime._return_timing.profile.minimum_gap = lambda **kw: 1.0
    options = dict(tts=.08, **{k:v for k,v in kwargs.items() if k != "tts"})
    options["tts"] = kwargs.get("tts", .08)
    result = step(runtime, hitter, peer, **options)
    assert result[peer]["command"]["role"] != "return"
    assert runtime.return_timing_status()[peer]["cycle"]["blocked_reason"] == reason


def test_stale_feedback_cannot_be_overridden_even_by_legacy_path():
    runtime, hitter, peer = start_cycle(mode="peer_outward")
    result = step(runtime, hitter, peer, -.2, phase="OUTWARD", fresh=False)
    assert not result[peer]["valid"]
    assert result[peer]["command"]["role"] != "return"


def test_nan_tts_rejected_before_it_can_break_the_timer():
    runtime, hitter, _ = start_cycle()
    state = robot_state(hitter, -.35, sequence=2)
    state["time_to_strike_s"] = float("nan")
    with pytest.raises(ValueError, match="time_to_strike_s"):
        runtime.update_robot_state(hitter, state, 1.1)
    assert runtime._states[hitter]["sequence"] == 1


def test_request_retries_keep_first_timestamp_then_stop_after_ack():
    runtime, hitter, peer = start_cycle()
    runtime._return_timing.profile.minimum_gap = lambda **kw: 1.0
    first = step(runtime, hitter, peer, .1)
    seq = first[peer]["sequence"]
    again = step(runtime, hitter, peer, .08, now=1.12, sequence=3)
    assert again[peer]["command"]["role"] == "return"
    assert runtime.return_timing_status()[peer]["cycle"]["requested_sequence"] == seq
    ack = step(runtime, hitter, peer, .06, peer_phase="RETURN", now=1.14, sequence=4)
    assert ack[peer]["command"]["role"] == "hold"  # no new RETURN reference starts
    cycle = runtime.return_timing_status()[peer]["cycle"]
    assert cycle["request_to_return_ms"] == pytest.approx(40)
    assert cycle["status"] == "returning"
    step(runtime, hitter, peer, -.2, phase="OUTWARD", peer_phase="HOME_HOLD", now=1.4, sequence=5)
    assert runtime._swap_commit_token is None
    assert runtime.return_timing_status()[peer]["cycle"]["status"] == "complete"


def test_shadow_never_sends_active_return():
    runtime, hitter, peer = start_cycle(shadow=True)
    runtime._return_timing.profile.minimum_gap = lambda **kw: 1.0
    result = step(runtime, hitter, peer, .1)
    assert result[peer]["planned_valid"]
    assert not result[peer]["valid"]
    assert not result[peer]["command"]["active"]


@pytest.mark.parametrize("lead", [True, None, "100", float("nan"), float("inf"), 201, -201, 110, 100.1])
def test_invalid_values(lead):
    with pytest.raises(ValueError):
        parse_return_timing(timing(lead=lead), ROBOT_ORDER)


def test_config_idempotence_and_stale_publication():
    config = ReturnTiming(ROBOT_ORDER)
    config.stage(timing())
    config.stage(timing())
    with pytest.raises(ValueError, match="reused"):
        config.stage(timing(lead=80))
    with pytest.raises(ValueError, match="stale"):
        config.stage(timing(command_id="old", stamp=.5))
    assert config.pending["table_left"].lead_ms == 100


def test_http_to_planner_status_and_error_round_trip():
    runtime = V9RealFixedRelayRuntime(shadow=True)
    store = TelemetryStore(DEFAULT_TOPICS)
    messages = []
    def publish(payload):
        messages.append(json.loads(json.dumps(payload)))
        runtime.stage_return_timing_config(messages[-1])
        store.ingest("fixed_status", {"return_timing": runtime.return_timing_status()})
    store.set_return_timing_sender(publish)
    server = MonitorHttpServer(("127.0.0.1", 0), store)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        def post(body):
            request = urllib.request.Request(base+"/api/return_timing/table_left", data=json.dumps(body).encode(),
                                             headers={"Content-Type":"application/json"})
            return json.load(urllib.request.urlopen(request, timeout=2))
        response = post(dict(mode="strike_time", lead_ms=-100))
        assert response["ok"]
        assert len(messages) == 1
        snapshot = json.load(urllib.request.urlopen(base+"/api/snapshot", timeout=2))
        status = snapshot["topics"]["fixed_status"]["data"]["return_timing"]
        assert status["table_left"]["pending"]["command_id"] == response["command_id"]
        assert status["table_right"]["pending"] is None
        with pytest.raises(urllib.error.HTTPError) as exc:
            post(dict(mode="strike_time", lead_ms=110))
        assert exc.value.code == 400
        assert len(messages) == 1
        runtime._on_cycle_commit("table_right", "new-ball", 1)
        assert runtime.return_timing_status()["table_left"]["active"]["lead_ms"] == -100
    finally:
        server.shutdown()
        server.server_close()
        thread.join(2)
