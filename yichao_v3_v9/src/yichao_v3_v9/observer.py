"""Bounded, input-only ROS diagnostics. No inference or command transport imports."""
import argparse
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import queue
import socket
import struct
import sys
import time
import uuid
import xmlrpc.client

from .telemetry import TOPICS, TelemetryParser, strict_json

MASTER = 'http://127.0.0.1:11311'


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = 2.0
        return connection


def graph():
    with xmlrpc.client.ServerProxy(MASTER, transport=TimeoutTransport()) as master:
        state = master.getSystemState('/yichao_v3_v9_preflight')
        types = master.getTopicTypes('/yichao_v3_v9_preflight')
    if state[0] != 1 or types[0] != 1:
        raise RuntimeError('ROS master rejected graph inspection')
    return dict(state[2][0]), dict(types[2])


def preflight(read_graph=graph):
    result = {'kind': 'preflight', 'master': MASTER, 'ready_to_subscribe': False,
              'real_input_accepted': False, 'control_publishers_created': 0}
    try:
        publishers, types = read_graph()
        result['inputs'] = {topic: {'publishers': sorted(publishers.get(topic, [])),
                                    'type': types.get(topic), 'expected_type': typ}
                            for topic, (_, typ) in TOPICS.items()}
        result['missing_publishers'] = [t for t in TOPICS if not publishers.get(t)]
        result['wrong_types'] = [t for t, (_, typ) in TOPICS.items() if types.get(t) != typ]
        result['ready_to_subscribe'] = not result['missing_publishers'] and not result['wrong_types']
    except Exception as exc:
        result['reason'] = '%s: %s' % (type(exc).__name__, exc)
    return result


def pose_payload(message):
    p, q, h = message.pose.position, message.pose.orientation, message.header
    return {'header': {'seq': h.seq, 'stamp': {'secs': h.stamp.secs, 'nsecs': h.stamp.nsecs},
                       'frame_id': h.frame_id},
            'position_xyz': [p.x, p.y, p.z], 'orientation_xyzw': [q.x, q.y, q.z, q.w]}


def load_ros():
    # Pure ROS Python modules are copied into this release; no live checkout imports.
    private_ros = Path(__file__).resolve().parents[2] / 'vendor' / 'ros1'
    if not private_ros.is_dir():
        raise RuntimeError('private ROS dependencies missing; use preflight/replay only')
    root = private_ros.parent.parent
    evidence = json.loads((root/'config/observer_environment.json').read_text())
    for relative, expected in evidence['ros_sha256'].items():
        if hashlib.sha256((private_ros/relative).read_bytes()).hexdigest() != expected:
            raise ValueError('private ROS snapshot mismatch: ' + relative)
    import importlib.metadata as metadata
    for line in (root/'requirements.lock').read_text().splitlines():
        name, version = line.split('==')
        if metadata.version(name) != version:
            raise ValueError('private dependency version mismatch: ' + name)
    sys.path.insert(0, str(private_ros))
    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    return rospy, String, PoseStamped


def configure_subscriber_ip(address):
    """Explicitly select an already assigned address; never change network config."""
    ip = ipaddress.IPv4Address(address)
    if ip.is_loopback or ip.is_unspecified or ip.is_multicast:
        raise ValueError('capture requires an assigned, reachable workstation IPv4 address')
    local = set()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for _, interface in socket.if_nameindex():
            try:
                value = fcntl.ioctl(sock.fileno(), 0x8915, struct.pack('256s', interface.encode()[:15]))
                local.add(socket.inet_ntoa(value[20:24]))
            except OSError:
                pass
    if str(ip) not in local:
        raise ValueError('subscriber IP is not assigned to this workstation')
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_IP'] = str(ip)
    os.environ['ROS_MASTER_URI'] = MASTER


def capture(seconds, write, initial, read_graph=graph, ros_loader=load_ros):
    """Call only after graph readiness; hard-bounded diagnostic subscriptions."""
    if not initial.get('ready_to_subscribe'):
        raise ValueError('required publishers unavailable; node creation forbidden')
    if not 0 < seconds <= 60:
        raise ValueError('capture duration must be in (0, 60] seconds')
    current = preflight(read_graph)
    if not current['ready_to_subscribe'] or current['inputs'] != initial['inputs']:
        raise ValueError('input publisher identities changed before node creation')
    rospy, String, PoseStamped = ros_loader()
    parser, pending = TelemetryParser(), queue.Queue(maxsize=32)
    counters = {'received': 0, 'dropped': 0, 'wire_valid': 0, 'invalid': 0}
    subscribers = []
    node_name = 'yichao_v3_v9_observer_' + uuid.uuid4().hex
    # Disables rosout publisher and implicit /clock subscriber; ROS clock is
    # recorded only in wall-time mode. Simulated ROS time requires another contract.
    if rospy.get_param('/use_sim_time', False):
        raise ValueError('simulated ROS clock unsupported in live diagnostic observer')
    rospy.init_node(node_name, anonymous=False, disable_signals=True,
                    disable_rosout=True, disable_rostime=True)
    try:
        def callback(message, topic):
            receive = {'monotonic_s': time.monotonic(), 'wall_s': time.time(),
                       'ros_s': rospy.Time.now().to_sec()}
            counters['received'] += 1
            caller = getattr(message, '_connection_header', {}).get('callerid')
            try:
                if caller not in initial['inputs'][topic]['publishers']:
                    raise ValueError('message publisher does not match preflight identity')
                payload = (strict_json(message.data) if TOPICS[topic][0] != 'torso'
                           else pose_payload(message))
                record = {'topic': topic, 'payload': payload, 'receive': receive,
                          'provenance': 'live_ros', 'publisher': caller}
            except Exception as exc:
                record = {'kind': 'parse_error', 'topic': topic, 'receive': receive,
                          'reason': str(exc), 'publisher': caller, 'real_input_accepted': False}
            try:
                pending.put_nowait(record)
            except queue.Full:
                counters['dropped'] += 1
        for topic, (_, typ) in TOPICS.items():
            subscribers.append(rospy.Subscriber(topic, String if typ == 'std_msgs/String' else PoseStamped,
                                                callback, callback_args=topic, queue_size=1,
                                                buff_size=65536))
        write({'kind': 'capture_start', 'node_name': node_name, 'inputs': initial['inputs'],
               'real_input_accepted': False, 'control_publishers_created': 0})
        deadline, check_at = time.monotonic() + seconds, time.monotonic()
        reason = 'duration_complete'
        last_receive = {t: time.monotonic() for t in TOPICS}
        while time.monotonic() < deadline and not rospy.is_shutdown():
            now = time.monotonic()
            if now >= check_at:
                check = preflight(read_graph)
                if not check['ready_to_subscribe'] or check['inputs'] != initial['inputs']:
                    write(check)
                    reason = 'publisher_graph_changed'
                    break
                check_at = time.monotonic() + 1
            try:
                record = pending.get(timeout=.02)
            except queue.Empty:
                record = None
            if record is not None:
                last_receive[record['topic']] = record['receive']['monotonic_s']
                if record.get('kind') != 'parse_error':
                    record['diagnostic'] = parser.parse(record['topic'], record['payload'], record['receive'])
                    counters['wire_valid' if record['diagnostic']['wire_valid'] else 'invalid'] += 1
                else:
                    counters['invalid'] += 1
                write(record)
            # Transport silence is a diagnostic fault, not a physical stop command.
            stale = [t for t, seen in last_receive.items()
                     if time.monotonic() - seen > (.30 if TOPICS[t][0] == 'ball' else .25)]
            if stale:
                reason = 'input_stream_gap'
                write({'kind': reason, 'topics': stale, 'real_input_accepted': False})
                break
        if rospy.is_shutdown():
            reason = 'ros_shutdown'
        write({'kind': 'capture_end', 'reason': reason, **counters,
               'real_input_accepted': False, 'control_publishers_created': 0})
    finally:
        for sub in subscribers:
            sub.unregister()
        rospy.signal_shutdown('Yichao diagnostic observer finished')


class Writer:
    def __init__(self, stream):
        self.stream, self.total = stream, 0

    def __call__(self, record):
        text = json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n'
        self.total += len(text.encode('utf-8'))
        if self.total > 16 * 1024 * 1024:
            raise ValueError('16 MiB diagnostic output budget exceeded')
        self.stream.write(text)
        self.stream.flush()


class SegmentedWriter:
    """Persistent JSONL logs roll into new files without stopping live input.

    Keep every segment; never overwrite or delete earlier trial evidence.
    The caller owns the initial stream and must close this writer on exit.
    """
    def __init__(self, stream, segment_bytes=16 * 1024 * 1024):
        if type(segment_bytes) is not int or segment_bytes < 1:
            raise ValueError('positive log segment budget required')
        self.stream = self.initial_stream = stream
        self.path = Path(stream.name)
        self.segment_bytes = segment_bytes
        self.segment = self.segment_total = self.total = 0

    def __call__(self, record):
        text = json.dumps(record, ensure_ascii=False, allow_nan=False) + '\n'
        size = len(text.encode('utf-8'))
        if self.segment_total and self.segment_total + size > self.segment_bytes:
            next_path = self.path.with_name(self.path.name + '.%04d' % (self.segment + 1))
            next_stream = next_path.open('x')
            if self.stream is not self.initial_stream:
                self.stream.close()
            self.stream = next_stream
            self.segment += 1
            self.segment_total = 0
        self.stream.write(text)
        self.stream.flush()
        self.segment_total += size
        self.total += size

    def close(self):
        if self.stream is not self.initial_stream:
            self.stream.close()


def replay(records, write):
    parser = TelemetryParser()
    metadata_kinds = {'preflight', 'capture_start', 'capture_end', 'parse_error', 'input_stream_gap'}
    while True:
        line = records.readline(65537)
        if not line:
            break
        event = strict_json(line)
        if event.get('kind') in metadata_kinds:
            write({'kind': 'replayed_capture_metadata', 'event': event, 'real_input_accepted': False})
            continue
        result = parser.parse(event['topic'], event['payload'], event['receive'],
                              provenance=event.get('provenance', 'unverified_replay'))
        write({'kind': 'replay_diagnostic', 'diagnostic': result})


def main():
    p = argparse.ArgumentParser(description='Yichao input-only diagnostics; default: graph preflight')
    p.add_argument('--output', type=Path, required=True, help='new JSONL file')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--capture-seconds', type=float, help='at most 60 seconds; all publishers required')
    mode.add_argument('--replay', type=Path, help='parse recorded input envelopes without ROS')
    p.add_argument('--subscriber-ip', help='capture only: existing workstation address reachable by input publishers')
    args = p.parse_args()
    if args.capture_seconds is not None and not 0 < args.capture_seconds <= 60:
        p.error('--capture-seconds must be in (0,60]')
    if args.capture_seconds is not None and not args.subscriber_ip:
        p.error('--capture-seconds requires --subscriber-ip; never infer the control network')
    # Ensure an existing output/log can never be overwritten.
    with args.output.open('x') as stream:
        write = Writer(stream)
        if args.replay:
            if not args.replay.is_file():
                raise ValueError('replay input must be an existing regular file')
            with args.replay.open() as records:
                replay(records, write)
            return 0
        check = preflight()
        write(check)
        if not check['ready_to_subscribe']:
            print('Required input publishers unavailable; no ROS node created.', file=sys.stderr)
            return 2
        if args.capture_seconds is not None:
            configure_subscriber_ip(args.subscriber_ip)
            capture(args.capture_seconds, write, check)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
