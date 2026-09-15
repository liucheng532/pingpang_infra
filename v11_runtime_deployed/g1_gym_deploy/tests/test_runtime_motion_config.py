import json
import math
import threading
from types import SimpleNamespace

import pytest

from utils.runtime_motion_config import parse_runtime_motion_config
from envs.lcm_agent import LCMAgent


def valid_payload():
    return {
        "schema": "v10-runtime-motion-config-v1",
        "command_id": "cmd-1",
        "published_at": 123.0,
        "robot_id": "table_right",
        "goal_x": 0.20,
        "outward_y": -0.90,
        "home_y": -0.10,
        "outward_hold_s": 4.0,
    }


def test_runtime_motion_config_parses_json_and_fields():
    config = parse_runtime_motion_config(json.dumps(valid_payload()))
    assert config.command_id == "cmd-1"
    assert config.goal_x == 0.20
    assert config.outward_y == -0.90
    assert config.home_y == -0.10
    assert config.outward_hold_s == 4.0


@pytest.mark.parametrize(
    "field,value,error",
    [
        ("schema", "wrong", "schema"),
        ("robot_id", "table_left", "robot_id"),
        ("outward_y", 1.31, "outward_y"),
        ("home_y", -1.31, "home_y"),
        ("outward_hold_s", 31.0, "outward_hold_s"),
        ("goal_x", math.nan, "goal_x"),
    ],
)
def test_runtime_motion_config_rejects_invalid_values(field, value, error):
    payload = valid_payload()
    payload[field] = value
    with pytest.raises(ValueError, match=error):
        parse_runtime_motion_config(payload)


def test_runtime_motion_config_rejects_unsupported_y_displacement():
    payload = valid_payload()
    payload["home_y"] = -0.80
    with pytest.raises(ValueError, match="outward_y-home_y"):
        parse_runtime_motion_config(payload)


def test_lcm_agent_callback_hands_latest_config_to_control_thread():
    class Scheduler:
        def __init__(self):
            self.calls = []

        def stage_runtime_motion_config(self, **kwargs):
            self.calls.append(kwargs)

    agent = LCMAgent.__new__(LCMAgent)
    agent._runtime_motion_config_lock = threading.Lock()
    agent._runtime_motion_config_inbox = None
    agent._runtime_motion_config_received_id = None
    agent._runtime_motion_config_error = ""
    agent.reference_scheduler = Scheduler()
    agent._runtime_motion_config_callback(
        SimpleNamespace(data=json.dumps(valid_payload()))
    )
    assert agent._runtime_motion_config_received_id == "cmd-1"
    assert agent.reference_scheduler.calls == []
    agent._apply_pending_runtime_motion_config()
    assert len(agent.reference_scheduler.calls) == 1
    assert agent.reference_scheduler.calls[0]["goal_x"] == 0.20
    assert agent.reference_scheduler.calls[0]["home_y"] == -0.10


def test_lcm_agent_callback_rejects_invalid_message_without_replacing_inbox():
    agent = LCMAgent.__new__(LCMAgent)
    agent._runtime_motion_config_lock = threading.Lock()
    agent._runtime_motion_config_inbox = "existing"
    agent._runtime_motion_config_received_id = "existing"
    agent._runtime_motion_config_error = ""
    agent._runtime_motion_config_callback(SimpleNamespace(data="{}"))
    assert agent._runtime_motion_config_inbox == "existing"
    assert agent._runtime_motion_config_received_id == "existing"
    assert "schema" in agent._runtime_motion_config_error
