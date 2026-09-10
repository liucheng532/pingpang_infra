#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import threading
from functools import wraps


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from doubles_planner.real_ros_runtime import ROBOT_ORDER, V9RealFixedRelayRuntime
from doubles_planner.return_timing import RETURN_TIMING_TOPIC


def parse_args():
    parser = argparse.ArgumentParser(description="Real-robot V9 fixed relay runtime")
    parser.add_argument("--active", action="store_true")
    return parser.parse_args()


def main() -> None:
    import rospy
    from std_msgs.msg import String

    args = parse_args()
    rospy.init_node("doubles_v9_real_fixed_relay", anonymous=False)
    runtime = V9RealFixedRelayRuntime(shadow=not args.active)
    runtime_lock = threading.RLock()

    def serialized(callback):
        @wraps(callback)
        def wrapped(*args, **kwargs):
            with runtime_lock:
                return callback(*args, **kwargs)
        return wrapped
    publishers = {
        name: rospy.Publisher(f"/doubles/{name}/command", String, queue_size=1)
        for name in ROBOT_ORDER
    }
    status_publisher = rospy.Publisher(
        "/doubles/v9_fixed_relay/status", String, queue_size=1
    )

    @serialized
    def state_callback(message, robot):
        try:
            runtime.update_robot_state(robot, json.loads(message.data))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, f"invalid {robot} state: {error}")

    @serialized
    def ball_callback(message):
        try:
            runtime.update_ball(json.loads(message.data))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, f"invalid doubles ball prediction: {error}")

    @serialized
    def motion_config_callback(message):
        try:
            runtime.stage_runtime_motion_config(json.loads(message.data))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, f"invalid runtime motion config: {error}")

    @serialized
    def return_timing_callback(message):
        try:
            runtime.stage_return_timing_config(json.loads(message.data))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, f"invalid return timing config: {error}")

    for name in ROBOT_ORDER:
        rospy.Subscriber(
            f"/doubles/{name}/state",
            String,
            state_callback,
            callback_args=name,
            queue_size=1,
        )
    rospy.Subscriber("/doubles/ball_prediction", String, ball_callback, queue_size=1)
    rospy.Subscriber(
        "/doubles/runtime_motion_config",
        String,
        motion_config_callback,
        queue_size=10,
    )
    rospy.Subscriber(RETURN_TIMING_TOPIC, String, return_timing_callback, queue_size=10)

    last_status = 0.0

    @serialized
    def tick(_event):
        nonlocal last_status
        outputs = runtime.tick()
        for name, payload in outputs.items():
            publishers[name].publish(
                String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True))
            )
        now = rospy.Time.now().to_sec()
        if now - last_status >= 0.2:
            status_publisher.publish(
                String(
                    data=json.dumps(
                        {
                            "stamp": now,
                            "shadow": runtime.shadow,
                            "planner_mode": runtime.planner_mode,
                            "pending_shot_id": outputs[ROBOT_ORDER[0]][
                                "pending_shot_id"
                            ],
                            "admission_reasons": outputs[ROBOT_ORDER[0]][
                                "admission_reasons"
                            ],
                            "cycle_hitter": runtime._swap_hitter,
                            "cycle_commit_token": runtime._swap_commit_token,
                            "cycle_hit_acknowledged": runtime._swap_observed_hit_phase,
                            "cycle_return_acknowledged": runtime._return_dispatched,
                            "runtime_motion_config": runtime.runtime_motion_config_status(),
                            "return_timing": runtime.return_timing_status(),
                            "robots": {
                                name: {
                                    "valid": payload["valid"],
                                    "planned_valid": payload["planned_valid"],
                                    "planned_active": payload["planned_active"],
                                    "active": payload["command"]["active"],
                                    "role": payload["command"]["role"],
                                    "relay_stage": payload["relay_stage"],
                                    "commit_token": payload["commit_token"],
                                    "fallbacks": payload["fallbacks"],
                                }
                                for name, payload in outputs.items()
                            },
                        },
                        separators=(",", ":"),
                        sort_keys=True,
                    )
                )
            )
            last_status = now

    rospy.Timer(rospy.Duration(0.02), tick)
    rospy.loginfo(
        "V9 real fixed relay runtime started in %s",
        "ACTIVE" if args.active else "SHADOW",
    )
    rospy.spin()


if __name__ == "__main__":
    main()
