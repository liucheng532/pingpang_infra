"""Bounded existing mocap -> private diagnostic ROS poses; no control topics."""
import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import queue
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'tools'))
from yichao_v3_v9.observer import graph, configure_subscriber_ip, load_ros, Writer, SegmentedWriter
from yichao_v3_v9.pose_feedback import PoseFeedback, DoublesTorsoFeedback, DOUBLES_TORSO, PRIMARY, SECONDARY
from capture_existing_inputs import serialize


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-id', required=True)
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--until-interrupted',action='store_true',help='persistent private input relay for operator-controlled active session')
    p.add_argument('--subscriber-ip', required=True)
    p.add_argument('--publisher', required=True)
    p.add_argument('--input-mode', choices=('legacy-rigid-debug','doubles-torso'), default='legacy-rigid-debug')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', args.run_id) or not 0 < args.seconds <= 60:
        p.error('run-id must be a safe ROS name; duration must be <=60s')
    prefix = '/yichao_v3_v9/'+args.run_id
    pubs, types = graph()
    expected = ({t:'geometry_msgs/PoseStamped' for t in DOUBLES_TORSO} if args.input_mode == 'doubles-torso'
                else {PRIMARY: 'geometry_msgs/PoseStamped', SECONDARY: 'std_msgs/Float64MultiArray'})
    for topic, typ in expected.items():
        if pubs.get(topic) != [args.publisher] or types.get(topic) != typ:
            raise SystemExit('input publisher/type mismatch: '+topic)
    if any(t.startswith(prefix+'/') for t in pubs):
        raise SystemExit('private diagnostic namespace is already in use')
    configure_subscriber_ip(args.subscriber_ip)
    rospy, String, PoseStamped = load_ros()
    from std_msgs.msg import Float64MultiArray
    if rospy.get_param('/use_sim_time', False):
        raise SystemExit('simulated ROS clock unsupported')
    if args.input_mode == 'doubles-torso':
        adapter = DoublesTorsoFeedback(args.publisher)
    else:
        blob = (ROOT/'config/pose_calibration_source.json').read_bytes()
        adapter = PoseFeedback(json.loads(blob), hashlib.sha256(blob).hexdigest(), args.publisher)
    pending = queue.Queue(maxsize=128)
    counts = {'66': 0, '198': 0, 'queue_drops': 0, 'rejected': 0}
    subscribers, outputs, metadata = [], {}, None
    with args.output.open('x') as stream, ExitStack() as cleanup:
        write = SegmentedWriter(stream) if args.until_interrupted else Writer(stream)
        if args.until_interrupted:
            cleanup.callback(write.close)
        rospy.init_node('yichao_pose_'+args.run_id, disable_signals=True,
                        disable_rosout=True, disable_rostime=True)
        def receive(message, topic):
            record = {'kind': 'raw_input', 'topic': topic,
                'publisher': getattr(message, '_connection_header', {}).get('callerid'),
                'receive_monotonic_s': time.monotonic(), 'payload': serialize(message)}
            try:
                pending.put_nowait(record)
            except queue.Full:
                counts['queue_drops'] += 1
        try:
            for role in ('table_left', 'table_right'):
                outputs[role] = rospy.Publisher(prefix+'/'+role+'/torso_pose_origin', PoseStamped,
                                                queue_size=1, latch=False)
            metadata = rospy.Publisher(prefix+'/pose_evidence', String, queue_size=1, latch=False)
            for topic, typ in expected.items():
                cls = PoseStamped if typ == 'geometry_msgs/PoseStamped' else Float64MultiArray
                subscribers.append(rospy.Subscriber(topic, cls, receive, callback_args=topic,
                                                    queue_size=1, buff_size=65536))
            write({'kind': 'start', 'prefix': prefix, 'real_input_accepted': False,
                   'clock_conversion_applied': False, 'control_commands_sent': 0,
                   'input_mode':args.input_mode,'publisher':args.publisher})
            print(json.dumps({'started': True, 'prefix': prefix}), flush=True)
            deadline, check_at = (float('inf') if args.until_interrupted else time.monotonic()+args.seconds), time.monotonic()+2
            last_published = {}
            while time.monotonic() < deadline and not rospy.is_shutdown():
                try:
                    rec = pending.get(timeout=.02)
                except queue.Empty:
                    continue
                now = time.monotonic()
                if now >= check_at:
                    current, current_types = graph()
                    if any(current.get(t) != [args.publisher] or current_types.get(t) != expected[t] for t in expected):
                        raise RuntimeError('source_publisher_changed')
                    check_at = now+2
                try:
                    if now-rec['receive_monotonic_s'] > .05:
                        raise ValueError('queued_pose_expired')
                    pose = adapter.parse(rec)
                    slot = pose['robot_id']
                    if now-last_published.get(slot, -1.) < .02:
                        continue
                    last_published[slot] = now
                    msg = PoseStamped()
                    # Preserve each source clock, including the mocap clock's
                    # offset. This header must not be treated as aligned ROS time.
                    source = next(iter(pose['source_clock'].values()))
                    msg.header.stamp = rospy.Time.from_sec(source)
                    msg.header.seq = counts[slot]
                    msg.header.frame_id = 'mocap_origin'
                    for key, value in zip(('x', 'y', 'z'), pose['torso_position_xyz']):
                        setattr(msg.pose.position, key, value)
                    for key, value in zip(('x', 'y', 'z', 'w'), pose['torso_orientation_xyzw']):
                        setattr(msg.pose.orientation, key, value)
                    outputs[pose['role']].publish(msg)
                    metadata.publish(String(data=json.dumps(pose, allow_nan=False)))
                    write(pose)
                    counts[slot] += 1
                except (KeyError, TypeError, ValueError) as exc:
                    counts['rejected'] += 1
                    write({'kind': 'pose_rejected', 'reason': str(exc)})
        finally:
            for sub in subscribers:
                sub.unregister()
            for pub in outputs.values():
                pub.unregister()
            if metadata is not None:
                metadata.unregister()
            rospy.signal_shutdown('bounded diagnostic relay finished')
        summary = {'kind': 'summary', 'counts': counts, 'control_commands_sent': 0,
                   'real_input_accepted': False, 'source_clock_alignment_verified': False}
        write(summary)
        print(json.dumps(summary), flush=True)
    return 0 if counts['66'] and counts['198'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
