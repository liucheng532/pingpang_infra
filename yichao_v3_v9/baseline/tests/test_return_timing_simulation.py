"""Two-controller closed-loop timing simulation driven by Isaac movement traces.

This exercises the production planner and message/acknowledgement flow. It is
not a two-humanoid rigid-body/contact simulation or a real-robot safety result.
"""
from __future__ import annotations

import numpy as np
import pytest

from test_return_timing import start_cycle
from test_real_ros_runtime import ball_prediction, robot_state


def simulate_pair(hitter, mode, lead, command_delay, hitter_y=.4):
    runtime, hitter, peer = start_cycle(hitter, lead, mode)
    profile = runtime._return_timing.profile.profile
    times = np.asarray(profile["time_s"])
    outward = np.asarray(profile["outward"]["p50"])
    returning = np.asarray(profile["return"]["p50"])
    outward_v = np.gradient(outward, times)
    return_v = np.gradient(returning, times)
    sign = 1 if hitter == "table_left" else -1
    token = runtime._swap_commit_token
    return_start = None
    accepted_sequence = 0
    requests = 0
    ack = None
    minimum_gap = float("inf")
    actor_trajectory = []
    for tick in range(1, 131):
        now = 1.0 + tick * .02
        tts = 1.5 - now
        out_t = max(0., now-1.7)
        hy = sign * (hitter_y + (.7-hitter_y) * np.interp(out_t, times, outward))
        hv = sign * (.7-hitter_y) * np.interp(out_t, times, outward_v) if out_t > 0 else 0.
        actor_trajectory.append(hy)
        peer_moving = return_start is not None and now+1e-9 >= return_start
        rt = max(0., now-return_start) if peer_moving else 0.
        py = -sign * (.9 - .7 * np.interp(rt, times, returning))
        pv = sign * .7 * np.interp(rt, times, return_v) if peer_moving else 0.
        minimum_gap = min(minimum_gap, sign*(hy-py))
        hp = "HIT" if now < 1.5-1e-9 else "POST_DELAY" if now < 1.7-1e-9 else "OUTWARD" if out_t < .86 else "OUTWARD_HOLD"
        pp = "OUTWARD_HOLD" if not peer_moving else "RETURN" if rt < .6 else "HOME_HOLD"
        for robot, y, velocity, phase in ((hitter,hy,hv,hp),(peer,py,pv,pp)):
            state = robot_state(robot, y, sequence=tick+1)
            state.update(phase=phase, last_commit_token=token, state_elapsed_s=10.0,
                         time_to_strike_s=tts if robot == hitter and hp in {"HIT","POST_DELAY"} else -.5,
                         last_applied_sequence=accepted_sequence if robot == peer else 1,
                         last_planner_session_id=runtime.session_id)
            state["base_linear_velocity_xyz"][1] = velocity
            runtime.update_robot_state(robot, state, now)
        ball = ball_prediction(sequence=tick+1)
        ball.update(valid=False, shot_id=None, time_to_strike_s=-.5)
        runtime.update_ball(ball, now)
        outputs = runtime.tick(now=now)
        assert not outputs[hitter]["command"]["active"]
        if outputs[peer]["command"]["role"] == "return":
            assert outputs[peer]["valid"]
            requests += 1
            if return_start is None:
                return_start = now + command_delay + .02
                accepted_sequence = outputs[peer]["sequence"]
        if pp == "RETURN" and ack is None:
            ack = now
    cycle = runtime.return_timing_status()[peer]["cycle"]
    assert return_start is not None
    assert cycle["status"] == "complete"
    assert cycle["acknowledged_at"] == pytest.approx(ack)
    assert cycle["request_to_return_ms"] == pytest.approx((command_delay+.02)*1000)
    assert minimum_gap >= runtime.config.min_separation
    return dict(hitter=hitter, mode=mode, lead_ms=lead, command_delay_ms=command_delay*1000,
                hitter_start_y=hitter_y, trigger_reason=cycle["trigger_reason"],
                return_request_offset_ms=(cycle["requested_at"]-1.5)*1000,
                request_to_return_ms=cycle["request_to_return_ms"], minimum_gap_m=minimum_gap,
                requests=requests, actor_trajectory=actor_trajectory)


@pytest.mark.parametrize("hitter", ["table_right", "table_left"])
@pytest.mark.parametrize("delay", [0., .02, .04])
@pytest.mark.parametrize("lead", [-200, 0, 100, 200])
def test_two_controllers_with_command_delay(hitter, delay, lead):
    candidate = simulate_pair(hitter, "strike_time", lead, delay)
    baseline = simulate_pair(hitter, "peer_outward", lead, delay)
    assert candidate["actor_trajectory"] == baseline["actor_trajectory"]
    assert candidate["return_request_offset_ms"] <= baseline["return_request_offset_ms"]+1e-7
    if lead == 100:
        assert candidate["trigger_reason"] == "strike_time"
        assert candidate["return_request_offset_ms"] == pytest.approx(-100)


@pytest.mark.parametrize("hitter", ["table_right", "table_left"])
def test_close_hitter_waits_for_clearance(hitter):
    result = simulate_pair(hitter, "strike_time", 100, .02, hitter_y=.2)
    assert result["return_request_offset_ms"] > 0
    assert result["return_request_offset_ms"] <= 200+1e-7
