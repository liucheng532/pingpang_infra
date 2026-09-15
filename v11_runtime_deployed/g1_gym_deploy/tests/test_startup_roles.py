from __future__ import annotations

import sys
from pathlib import Path
import types
from types import SimpleNamespace

import numpy as np
import pytest
import torch


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
if str(DEPLOY_ROOT) not in sys.path:
    sys.path.insert(0, str(DEPLOY_ROOT))

geometry_msgs = types.ModuleType("geometry_msgs")
geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
geometry_msgs_msg.PoseStamped = type("PoseStamped", (), {})
geometry_msgs_msg.PointStamped = type("PointStamped", (), {})
geometry_msgs_msg.TwistStamped = type("TwistStamped", (), {})
geometry_msgs.msg = geometry_msgs_msg
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
std_msgs_msg.Float32 = type("Float32", (), {})
std_msgs.msg = std_msgs_msg
sys.modules.setdefault("geometry_msgs", geometry_msgs)
sys.modules.setdefault("geometry_msgs.msg", geometry_msgs_msg)
sys.modules.setdefault("std_msgs", std_msgs)
sys.modules.setdefault("std_msgs.msg", std_msgs_msg)
sys.modules.setdefault("lcm", types.ModuleType("lcm"))

from envs.lcm_agent import LCMAgent
from utils.doubles_reference_scheduler import DoublesPhase
from utils.planner_ros_bridge import PlannerHitUpdate


class FakeScheduler:
    def __init__(self):
        self.calls = []

    def set_external_home_target(self, value):
        self.calls.append(("home", float(value)))

    def bootstrap_outward_hold(self, home_y, outward_y):
        self.calls.append(("bootstrap", float(home_y), float(outward_y)))


def agent(home, bootstrap, startup_home_current, pelvis_y):
    value = LCMAgent.__new__(LCMAgent)
    value.reference_scheduler = FakeScheduler()
    value.planner_home_y = home
    value.bootstrap_outward_hold = bootstrap
    value.startup_home_current = startup_home_current
    value.pelvis_pos = np.array([0.0, pelvis_y, 0.75], dtype=np.float32)
    return value


def test_right_startup_keeps_physical_current_home_until_planner_command():
    value = agent(-0.20, False, True, -0.18)
    value._apply_startup_scheduler_state()
    assert value.reference_scheduler.calls == []


def test_left_startup_bootstraps_current_outward_with_ideal_return_home():
    value = agent(0.20, True, False, 0.98)
    value._apply_startup_scheduler_state()
    assert value.reference_scheduler.calls == [
        ("home", 0.20),
        ("bootstrap", 0.20, np.float32(0.98)),
    ]


def test_stationary_hit_home_skips_all_startup_locomotion():
    value = agent(0.20, True, False, 0.98)
    value.stationary_hit_test = "hit_home"
    value._apply_startup_scheduler_state()
    assert value.reference_scheduler.calls == []


@pytest.mark.parametrize("starts_hit", (False, True))
def test_lcm_agent_uses_external_hit_command_without_duplicate_advance(starts_hit):
    class Scheduler:
        def __init__(self, reference):
            self.reference = reference
            self.calls = []
            self.handoff_calls = []

        def update(self, tts, target, velocity, *args, **kwargs):
            self.calls.append(
                (
                    float(tts),
                    np.asarray(target).copy(),
                    np.asarray(velocity).copy(),
                    bool(kwargs.get("start_external_hit", False)),
                )
            )
            return self.reference

        def set_external_outward_target(self, target_y):
            self.handoff_calls.append(("outward", float(target_y)))

        def set_external_home_target(self, target_y):
            self.handoff_calls.append(("home", float(target_y)))

        def output(self):
            return self.reference

    update = PlannerHitUpdate(
        sequence=2,
        commit_token="token",
        command_age_s=0.012,
        raw_tts=0.40,
        streamed_tts=0.388,
        racket_target=np.array([0.45, -0.2, 1.05], dtype=np.float32),
        target_velocity=np.array([3.1, 0.2, 0.4], dtype=np.float32),
        prediction_source_timestamp_s=10.0,
        prediction_age_s=0.012,
        starts_hit=starts_hit,
        post_hit_outward_y=0.9125 if starts_hit else None,
        return_target_y=0.20 if starts_hit else None,
    )
    reference = SimpleNamespace(
        state=DoublesPhase.HIT,
        strike_time=0.388,
        racket_target=update.racket_target.copy(),
        target_velocity=update.target_velocity.copy(),
        hit_motion_index=7,
        move_motion_index=8,
        reference_step=9,
        command=np.zeros(58, dtype=np.float32),
    )
    scheduler = Scheduler(reference)
    bridge = SimpleNamespace(
        apply_pending=lambda *_args: update,
        publish_state=lambda _agent: None,
    )
    agent = LCMAgent.__new__(LCMAgent)
    agent._update_robot_state = lambda **_kwargs: None
    agent.planner_bridge = bridge
    agent.reference_scheduler = scheduler
    agent.reference = reference
    agent.recvtarget = np.zeros(3, dtype=np.float32)
    agent.recvvel = np.zeros(3, dtype=np.float32)
    agent.handle_time = -0.5
    agent.delay_time = 0.0
    agent._corrected_time_to_strike = lambda: -0.5
    agent.pelvis_pos = np.zeros(3, dtype=np.float32)
    agent.torso_pos = np.zeros(3, dtype=np.float32)
    agent.pelvis_rpy = np.zeros(3, dtype=np.float32)
    agent.pelvis_angular_velocity_w = np.zeros(3, dtype=np.float32)
    agent.dt = 0.02
    agent.timestep = 0
    agent.get_obs = lambda refresh_state=False: "obs"

    assert LCMAgent.observe(agent) == "obs"
    assert np.isclose(scheduler.calls[0][0], 0.388)
    assert np.array_equal(scheduler.calls[0][1], update.racket_target)
    assert np.array_equal(scheduler.calls[0][2], update.target_velocity)
    assert scheduler.calls[0][3] is starts_hit
    assert scheduler.handoff_calls == (
        [("outward", 0.9125), ("home", 0.20)] if starts_hit else []
    )
    assert agent.handle_time == 0.40
    assert agent.delay_time == 0.012


def test_v10_action_clip_preserves_v9_apply_path():
    agent = LCMAgent.__new__(LCMAgent)
    agent.action_clip = 10.0
    agent.publish_action = lambda hard_reset=False: None
    action = torch.zeros(1, 29)
    action[0, 3] = 12.0
    action[0, 4] = -11.0
    agent.apply_action(action)
    assert agent.actions[0, 3] == 10.0
    assert agent.actions[0, 4] == -10.0
