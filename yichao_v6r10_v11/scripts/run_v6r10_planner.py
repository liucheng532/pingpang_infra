#!/usr/bin/env python3
"""ROS adapter for V6R10 shadow, transport-shadow, and active modes."""

from __future__ import annotations

import argparse
from functools import wraps
import json
import os
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from yichao_v6r10_v11 import PLANNER_IDENTITY, STATUS_TOPIC
from yichao_v6r10_v11.actor import OnnxActor
from yichao_v6r10_v11.attestation import verify_attestation
from yichao_v6r10_v11.bootstrap import install_fixed_runtime
from yichao_v6r10_v11.journal import SessionJournal


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("shadow", "transport-shadow", "active"), default="shadow")
    parser.add_argument("--session-dir", type=Path, required=True)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "config/deployment.json")
    parser.add_argument("--attestation", type=Path)
    return parser.parse_args()


def command_publishers(rospy, robot_order):
    code, message, state = rospy.get_master().getSystemState()
    if code != 1:
        raise RuntimeError("cannot inspect command publishers: " + str(message))
    topics = {f"/doubles/{robot}/command" for robot in robot_order}
    return {topic: nodes for topic, nodes in state[0] if topic in topics and nodes}


def should_publish(mode, payload):
    """Transport shadow emits only valid, inactive staging messages."""

    if mode == "shadow":
        return False
    if mode == "transport-shadow":
        return bool(
            payload.get("valid")
            and not payload.get("planned_active")
            and payload.get("command", {}).get("role") == "stage"
        )
    return True


def main() -> None:
    args = parse_args()
    deployment = json.loads(args.config.read_text(encoding="utf-8"))
    os.environ["V6R10_FIXED_RUNTIME_ROOT"] = deployment["fixed_runtime_root"]
    install_fixed_runtime(deployment["fixed_runtime_root"])
    from doubles_planner.real_ros_runtime import ROBOT_ORDER
    from yichao_v6r10_v11.v6r10_runtime import ActorOnlyPipeline, V6R10Runtime

    attestation = None
    if args.mode != "shadow":
        attestation_path = args.attestation or ROOT / deployment["attestation"]
        attestation = verify_attestation(
            attestation_path,
            expected_controller_mode="active" if args.mode == "active" else "shadow",
            deployment=deployment,
        )
    actor = OnnxActor(ROOT / deployment["actor"]["onnx"], ROOT / deployment["actor"]["sidecar"])

    import rospy
    from std_msgs.msg import String

    rospy.init_node("doubles_yichao_v6r10", anonymous=True)
    if args.mode != "shadow":
        occupied = command_publishers(rospy, ROBOT_ORDER)
        if occupied:
            raise RuntimeError("existing command publishers: " + json.dumps(occupied, sort_keys=True))
    runtime = V6R10Runtime(
        mode=args.mode,
        session_id=args.session_id,
        pipeline=ActorOnlyPipeline(actor),
        attestation_verified=attestation is not None,
    )
    journal = SessionJournal(args.session_dir)
    journal.write(
        "start",
        session_id=runtime.session_id,
        mode=args.mode,
        planner_identity=PLANNER_IDENTITY,
        attestation=attestation,
    )
    rospy.on_shutdown(journal.close)
    lock = threading.RLock()

    def serialized(callback):
        @wraps(callback)
        def wrapped(*values, **keywords):
            with lock:
                return callback(*values, **keywords)
        return wrapped

    publishers = (
        {
            robot: rospy.Publisher(f"/doubles/{robot}/command", String, queue_size=1)
            for robot in ROBOT_ORDER
        }
        if runtime.command_publish_enabled
        else {}
    )
    status_publisher = rospy.Publisher(STATUS_TOPIC, String, queue_size=1)

    @serialized
    def state_callback(message, robot):
        try:
            payload = json.loads(message.data)
            journal.write("state", robot=robot, payload=payload)
            runtime.update_robot_state(robot, payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, f"invalid {robot} state: {error}")

    @serialized
    def ball_callback(message):
        try:
            payload = json.loads(message.data)
            journal.write("ball", payload=payload)
            runtime.update_ball(payload)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            rospy.logwarn_throttle(1.0, f"invalid ball prediction: {error}")

    for robot in ROBOT_ORDER:
        rospy.Subscriber(
            f"/doubles/{robot}/state", String, state_callback, callback_args=robot, queue_size=1
        )
    rospy.Subscriber("/doubles/ball_prediction", String, ball_callback, queue_size=1)

    last_status = 0.0
    decision_sequence = 0
    latest_decision = None

    @serialized
    def tick(_event):
        nonlocal last_status, decision_sequence, latest_decision
        outputs = runtime.tick()
        if runtime.last_inference is not None:
            decision_sequence += 1
            latest_decision = {
                "sequence": decision_sequence,
                "session_id": runtime.session_id,
                "shot_id": None if runtime._ball is None else runtime._ball.get("shot_id"),
                "decision": runtime.last_inference,
                "feature_sources": None if runtime.last_features is None else runtime.last_features.record,
                "actor_observation": None if runtime.last_features is None else runtime.last_features.actor.tolist(),
                "audit_observation": None if runtime.last_features is None else runtime.last_features.audit.tolist(),
                "actor_names": None if runtime.last_features is None else list(runtime.last_features.actor_names),
                "audit_names": None if runtime.last_features is None else list(runtime.last_features.audit_names),
            }
        journal.write(
            "tick",
            outputs=outputs,
            rl_decision=latest_decision,
            actor=None if runtime.last_features is None else runtime.last_features.actor.tolist(),
            audit=None if runtime.last_features is None else runtime.last_features.audit.tolist(),
        )
        published = {}
        for robot, publisher in publishers.items():
            payload = outputs[robot]
            publish = should_publish(args.mode, payload)
            published[robot] = publish
            if publish:
                publisher.publish(String(data=json.dumps(payload, separators=(",", ":"), sort_keys=True)))
        now = rospy.Time.now().to_sec()
        if now - last_status >= 0.2:
            status = {
                "schema": "v6r10-monitor-status-v1",
                "stamp": now,
                "session_id": runtime.session_id,
                "mode": runtime.mode,
                "planner_identity": PLANNER_IDENTITY,
                "filter_mode": "off",
                "barrier_risk": "not_evaluated",
                "ensemble_risk": "not_evaluated",
                "reason": runtime.reason,
                "rl_decision": latest_decision,
                "staging": runtime._staging,
                "previous_fully_acked_targets_m": runtime.features.previous_targets.tolist(),
                "failed_shots": dict(runtime.failed_shots),
                "cycle_hitter": runtime._swap_hitter,
                "cycle_commit_token": runtime._swap_commit_token,
                "cycle_hit_acknowledged": runtime._swap_observed_hit_phase,
                "cycle_return_requested": runtime._return_dispatched,
                "journal": {"dropped": journal.dropped, "error": journal.error},
                "fixed_commands": {
                    robot: payload.get("fixed_command")
                    for robot, payload in outputs.items()
                },
                "robots": {
                    robot: {
                        "role": payload["command"]["role"],
                        "valid": payload["valid"],
                        "planned_valid": payload["planned_valid"],
                        "planned_active": payload["planned_active"],
                        "sequence": payload["sequence"],
                        "phase": runtime._states.get(robot, {}).get("phase"),
                        "ack": None if runtime._staging is None else robot in runtime._staging.get("acknowledged", ()),
                        "target_y": payload["command"]["desired_base_position"][1],
                        "transport_error": runtime._states.get(robot, {}).get("transport_error"),
                        "command_published": published.get(robot, False),
                    }
                    for robot, payload in outputs.items()
                },
            }
            status_publisher.publish(
                String(data=json.dumps(status, separators=(",", ":"), sort_keys=True))
            )
            last_status = now

    rospy.Timer(rospy.Duration(0.02), tick)
    rospy.loginfo("V6R10 actor-only Planner started mode=%s", args.mode)
    rospy.spin()


if __name__ == "__main__":
    main()
