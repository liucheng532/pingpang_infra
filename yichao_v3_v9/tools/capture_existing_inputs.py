"""Bounded diagnostics of existing single/doubles ROS inputs; no control output."""
import argparse
import json
import math
import os
from pathlib import Path
import queue
import sys
import threading
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from yichao_v3_v9.observer import graph, load_ros, Writer, configure_subscriber_ip

ALLOW = {
    '/torso_pose_origin': 'geometry_msgs/PoseStamped',
    '/table_tennis_planner_monitor': 'std_msgs/String',
    '/debug/racket_rigid1_pose_origin': 'std_msgs/Float64MultiArray',
    '/predicted_ball_position': 'geometry_msgs/PointStamped',
    '/predicted_ball_velocity': 'geometry_msgs/TwistStamped',
    '/predicted_ball_predict_time': 'geometry_msgs/PointStamped',
    '/residual/mocap_ball_state': 'std_msgs/Float64MultiArray',
    '/doubles/ball_prediction': 'std_msgs/String',
    '/doubles/table_left/torso_pose_origin': 'geometry_msgs/PoseStamped',
    '/doubles/table_right/torso_pose_origin': 'geometry_msgs/PoseStamped',
    '/doubles/table_left/state': 'std_msgs/String',
    '/doubles/table_right/state': 'std_msgs/String',
}


def serialize(value):
    if isinstance(value, float) and not math.isfinite(value):
        return {'invalid_number': str(value)}
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, (list, tuple)):
        return [serialize(x) for x in value]
    if hasattr(value, '__slots__'):
        return {name: serialize(getattr(value, name)) for name in value.__slots__}
    raise ValueError('unsupported ROS field type: ' + type(value).__name__)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seconds', type=float, default=20.)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--subscriber-ip', required=True)
    p.add_argument('--topic', action='append', choices=sorted(ALLOW),
                   help='inspect only these existing inputs; repeat for multiple topics')
    p.add_argument('--learned-shadow', action='store_true',
                   help='diagnostic V3 inference and invalid movement previews; creates no publishers')
    p.add_argument('--actor', type=int, choices=(190, 199), default=199)
    args = p.parse_args()
    if not 0 < args.seconds <= 30:
        p.error('duration must be in (0,30] seconds')
    publishers, types = graph()
    shadow = None
    if args.learned_shadow:
        if args.topic:
            p.error('--learned-shadow selects its own three input topics')
        from yichao_v3_v9.runtime_shadow import RuntimeShadow, STATE_TOPICS, MONITOR, DOUBLES_BALL
        from yichao_v3_v9.inference import Pipeline
        from yichao_v3_v9.inputs import Features
        import numpy as np
        pipeline = Pipeline(args.actor)
        # Warm CPU sessions before subscribing. This is only a model warmup.
        pipeline.decide(Features(np.zeros(36, np.float32), np.zeros(85, np.float32), {}, 'synthetic'))
        ball_topic = DOUBLES_BALL if publishers.get(DOUBLES_BALL) and types.get(DOUBLES_BALL) == ALLOW[DOUBLES_BALL] else MONITOR
        shadow = RuntimeShadow('shadow-'+uuid.uuid4().hex, pipeline, provenance='live_ros', ball_topic=ball_topic)
        requested = {t: ALLOW[t] for t in (*STATE_TOPICS, ball_topic)}
    else:
        requested = {t: ALLOW[t] for t in args.topic} if args.topic else ALLOW
    selected = {t: sorted(publishers[t]) for t, typ in requested.items()
                if publishers.get(t) and types.get(t) == typ}
    if not selected:
        raise SystemExit('No allowlisted input publishers; no node started')
    configure_subscriber_ip(args.subscriber_ip)
    rospy, _, _ = load_ros()
    from roslib.message import get_message_class
    if rospy.get_param('/use_sim_time', False):
        raise SystemExit('simulated ROS clock is not supported')
    pending = queue.Queue(maxsize=128)
    lock = threading.Lock()
    stats = {t: {'count': 0, 'wrong_publisher': 0, 'queue_drops': 0, 'last_raw': -1.,
                 'max_receive_gap_s': 0., 'source_stamp_duplicates': 0,
                 'source_stamp_backwards': 0, 'nonpositive_source_stamp': 0}
             for t in selected}
    def callback(message, topic):
        mono, wall = time.monotonic(), time.time()
        caller = getattr(message, '_connection_header', {}).get('callerid')
        with lock:
            s = stats[topic]
            if caller not in selected[topic]:
                s['wrong_publisher'] += 1
                return
            if s['count']:
                s['max_receive_gap_s'] = max(s['max_receive_gap_s'], mono-s['last'])
            else:
                s['first'] = mono
            s['last'] = mono
            s['count'] += 1
            stamp = None
            if hasattr(message, 'header'):
                stamp = message.header.stamp.to_sec()
                if stamp <= 0:
                    s['nonpositive_source_stamp'] += 1
                else:
                    age = wall-stamp
                    s['candidate_source_age_min_s'] = min(age, s.get('candidate_source_age_min_s', age))
                    s['candidate_source_age_max_s'] = max(age, s.get('candidate_source_age_max_s', age))
                    if 'previous_stamp' in s:
                        s['source_stamp_duplicates'] += stamp == s['previous_stamp']
                        s['source_stamp_backwards'] += stamp < s['previous_stamp']
                    s['previous_stamp'] = stamp
            if mono - s['last_raw'] >= .02:  # retain <= 50 Hz, count every delivered callback
                s['last_raw'] = mono
                record = {'kind': 'raw_input', 'topic': topic, 'publisher': caller,
                          'receive_monotonic_s': mono, 'receive_wall_s': wall,
                          'source_ros_stamp_s': stamp, 'sensor_age_verified': False,
                          'payload': serialize(message), 'real_input_accepted': False}
                try:
                    pending.put_nowait(record)
                except queue.Full:
                    s['queue_drops'] += 1
    subscribers = []
    with args.output.open('x') as stream:
        write = Writer(stream)
        last_shadow_write = -1.
        def consume(record):
            nonlocal last_shadow_write
            write(record)
            if shadow is not None:
                shadow.ingest(record)
                result = shadow.tick(time.monotonic())
                # Persist every decision, throttle repeated missing/idle reports.
                if 'decision' in result or time.monotonic()-last_shadow_write >= .2:
                    write(result)
                    last_shadow_write = time.monotonic()
        write({'kind': 'start', 'selected_publishers': selected,
               'missing_topics': [t for t in requested if t not in selected],
               'source_identity_verified': False, 'real_input_accepted': False,
               'diagnostic_learned_shadow': args.learned_shadow, 'actor': args.actor if shadow else None})
        rospy.init_node('yichao_input_diagnostic_' + uuid.uuid4().hex,
                        disable_signals=True, disable_rosout=True, disable_rostime=True)
        reason = 'duration_complete'
        try:
            for topic in selected:
                cls = get_message_class(ALLOW[topic])
                if cls is None:
                    raise ValueError('private message type missing: ' + ALLOW[topic])
                subscribers.append(rospy.Subscriber(topic, cls, callback, callback_args=topic,
                                                    queue_size=1, buff_size=65536))
            print('CAPTURING', args.seconds, 'seconds;', len(selected), 'input topics', flush=True)
            deadline, next_graph = time.monotonic()+args.seconds, time.monotonic()+2.
            while time.monotonic() < deadline and not rospy.is_shutdown():
                try:
                    consume(pending.get(timeout=.02))
                except queue.Empty:
                    pass
                if time.monotonic() >= next_graph:
                    pubs, typs = graph()
                    if any(sorted(pubs.get(t, [])) != selected[t] or typs.get(t) != ALLOW[t]
                           for t in selected):
                        reason = 'publisher_graph_changed'
                        break
                    next_graph = time.monotonic()+2.
        finally:
            for sub in subscribers:
                sub.unregister()
            rospy.signal_shutdown('bounded input diagnostic finished')
        while not pending.empty():
            # Do not infer from the shutdown backlog.
            write(pending.get_nowait())
        if shadow is not None:
            write(shadow.summary())
        with lock:
            for s in stats.values():
                span = s.get('last', 0)-s.get('first', 0)
                s['delivered_callback_hz'] = (s['count']-1)/span if span > 0 else 0.
                for key in ('last_raw', 'previous_stamp', 'first', 'last'):
                    s.pop(key, None)
        summary = {'kind': 'summary', 'reason': reason, 'topics': stats,
                   'control_commands_sent': 0, 'real_input_accepted': False,
                   'note': 'Callback rate is not sensor rate; ROS age does not certify sensor freshness.'}
        write(summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
