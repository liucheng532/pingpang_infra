"""Bounded, subscription-only capture of private onboard shadow states and ball.

Single-robot capture diagnoses that endpoint; V3 requires both robot streams.
No command or clock-request publishers are constructed here.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import queue
import re
import sys
import time
import uuid
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from yichao_v3_v9.observer import graph, load_ros, configure_subscriber_ip, Writer
from yichao_v3_v9.runtime_shadow import RuntimeShadow, MONITOR, DOUBLES_BALL, STATE_TOPICS
from yichao_v3_v9.inputs import Features
from yichao_v3_v9.inference import Pipeline


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True)
    p.add_argument('--robots', nargs='+', choices=('66','198'), default=['66','198'])
    p.add_argument('--seconds', type=float, default=20.)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',args.session) or not 0 < args.seconds <= 60:
        p.error('safe session name and duration <=60s required')
    configure_subscriber_ip('172.16.3.126')
    pubs, types = graph()
    prefix = '/yichao_v3_v9/'+args.session
    mapping = {prefix+'/'+t.split('/')[2]+'/state':t for t,s in STATE_TOPICS.items() if s in args.robots}
    ball = DOUBLES_BALL if pubs.get(DOUBLES_BALL) else MONITOR
    mapping[ball] = ball
    if any(len(pubs.get(t,[])) != 1 or types.get(t) != 'std_msgs/String' for t in mapping):
        raise SystemExit('one matching publisher required for every selected input')
    owners = {t:pubs[t][0] for t in mapping}
    model = Pipeline(199)
    model.decide(Features(np.zeros(36,np.float32),np.zeros(85,np.float32),{},'synthetic'))
    shadow = RuntimeShadow(args.session,model,provenance='live_ros',ball_topic=ball)
    rospy, String, _ = load_ros()
    rospy.init_node('yichao_capture_'+uuid.uuid4().hex,disable_signals=True,
                    disable_rosout=True,disable_rostime=True)
    pending = queue.Queue(maxsize=128)
    counts, reasons = Counter(), Counter()
    def receive(message, topic):
        record = {'kind':'raw_input','topic':mapping[topic], 'wire_topic':topic,
                  'publisher':getattr(message,'_connection_header',{}).get('callerid'),
                  'receive_monotonic_s':time.monotonic(),'receive_wall_s':time.time(),
                  'payload':{'data':message.data}}
        try: pending.put_nowait(record)
        except queue.Full: counts['queue_drops'] += 1
    with args.output.open('x') as stream:
        write = Writer(stream)
        subs = [rospy.Subscriber(t,String,receive,callback_args=t,queue_size=1) for t in mapping]
        deadline, next_graph, next_tick, next_log = time.monotonic()+args.seconds, 0., 0., 0.
        try:
            while time.monotonic() < deadline and not rospy.is_shutdown():
                if counts['queue_drops']: raise RuntimeError('input_queue_overflow')
                now = time.monotonic()
                if now >= next_graph:
                    current, typs = graph()
                    if any(current.get(t) != [owners[t]] or typs.get(t) != types[t] for t in mapping):
                        raise RuntimeError('input_publisher_changed')
                    next_graph = now+1.
                try:
                    record = pending.get(timeout=.005)
                    if record['publisher'] != owners[record['wire_topic']]:
                        raise RuntimeError('input_publisher_identity')
                    counts[record['wire_topic']] += 1
                    write(record); shadow.ingest(record)
                except queue.Empty:
                    pass
                if time.monotonic() >= next_tick:
                    result = shadow.tick(time.monotonic())
                    reasons[result['reason'] or 'decision'] += 1
                    if result.get('decision') or time.monotonic() >= next_log:
                        write(result)
                        next_log = time.monotonic()+.2
                    next_tick = time.monotonic()+.02
        finally:
            for sub in subs: sub.unregister()
            rospy.signal_shutdown('bounded private state capture finished')
        result = dict(kind='summary',session=args.session,robots=args.robots,counts=dict(counts),
                      reasons=dict(reasons),decisions=shadow.counts['decisions'],
                      real_input_accepted=False,control_commands_sent=0)
        write(result); print(json.dumps(result),flush=True)


if __name__ == '__main__': main()
