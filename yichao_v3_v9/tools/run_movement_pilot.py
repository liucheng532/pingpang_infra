"""Private V9 movement/relay client with automatic per-run recording."""
import argparse
import hashlib
import json
from pathlib import Path
import queue
import re
import signal
import sys
import time
import uuid
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from yichao_v3_v9.runtime_shadow import RuntimeShadow, MONITOR, DOUBLES_BALL, STATE_TOPICS
from yichao_v3_v9.movement_client import MovementClient
from yichao_v3_v9.inference import Pipeline
from yichao_v3_v9.inputs import Features
from yichao_v3_v9.observer import graph, load_ros, configure_subscriber_ip
from yichao_v3_v9.telemetry import strict_json
from yichao_v3_v9.run_log import RunLog


def parse_args(argv=None, default_protocol='movement'):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True)
    p.add_argument('--robot', choices=('66', '198'), default='198')
    p.add_argument('--mode', choices=('shadow', 'active'), default='shadow')
    p.add_argument('--subscriber-ip', default='172.16.3.126')
    p.add_argument('--seconds', type=float, help='0 means continuous active; default active=0, shadow=30')
    p.add_argument('--actor', type=int, choices=(190, 199), default=199)
    p.add_argument('--output', type=Path, help='new JSONL path; defaults to a unique session log')
    p.add_argument('--protocol', choices=('movement','relay'), default=default_protocol)
    p.add_argument('--shots',type=int,help='0 means continuous active; default active=0, shadow=2')
    p.add_argument('--experimental-active',action='store_true',help='operator-controlled active trial; does not certify the real input contract')
    args = p.parse_args(argv)
    if args.seconds is None:args.seconds=0. if args.mode=='active' else 30.
    if args.shots is None:args.shots=0 if args.mode=='active' else 2
    if args.mode=='active' and not (args.experimental_active and args.protocol=='relay'):
        p.error('active requires the explicit experimental relay entry')
    if args.experimental_active and args.mode!='active':p.error('experimental-active requires active mode')
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', args.session) or not (
            0<args.seconds<=60 or args.mode=='active' and np.isfinite(args.seconds) and args.seconds>=0):
        p.error('session must be an identifier; shadow duration <=60s; active duration >=0')
    if args.shots<0 or args.shots==0 and args.mode!='active':p.error('shots must be positive, or 0 for active')
    if args.output is None:
        run=time.strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]
        args.output=ROOT/'output'/'sessions'/args.session/run/'planner.jsonl'
    return args


def main(default_protocol='movement', argv=None):
    args=parse_args(argv, default_protocol)
    metadata={'mode':args.mode,'protocol':args.protocol,'session':args.session,'actor':args.actor,
              'seconds':args.seconds,'shots':args.shots,'experimental_active':args.experimental_active,
              'first_hitter':'66' if args.protocol=='relay' else None}
    metadata['source_sha256']={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [*sorted((ROOT/'src/yichao_v3_v9').glob('*.py')),Path(__file__),ROOT/'config/assets.lock.json']}
    def terminate(signum, frame):raise SystemExit(128+signum)
    previous=signal.signal(signal.SIGTERM,terminate)
    try:
        with RunLog(args.output,metadata) as write:
            print(json.dumps({'planner_log':str(args.output.resolve()),**metadata}),flush=True)
            return run(args,write)
    finally:
        signal.signal(signal.SIGTERM,previous)


def run(args,write):
    prefix = '/yichao_v3_v9/'+args.session
    pubs, types = graph()
    wire_states = {prefix+'/'+t.split('/')[2]+'/state': t for t in STATE_TOPICS}
    ball_topic = DOUBLES_BALL if pubs.get(DOUBLES_BALL) else MONITOR
    for t in (*wire_states, ball_topic):
        if len(pubs.get(t, [])) != 1 or types.get(t) != 'std_msgs/String':
            raise SystemExit('one matching input publisher required: '+t)
    for role in ('table_left', 'table_right'):
        if pubs.get(prefix+'/'+role+'/command') or pubs.get(prefix+'/'+role+'/clock_request'):
            raise SystemExit('another client owns the private movement session')
    identities = {t: pubs[t][0] for t in (*wire_states, ball_topic)}
    configure_subscriber_ip(args.subscriber_ip)
    rospy, String, _ = load_ros()
    if rospy.get_param('/use_sim_time', False):
        raise SystemExit('simulated ROS clock unsupported')
    model = Pipeline(args.actor)
    model.decide(Features(np.zeros(36, np.float32), np.zeros(85, np.float32), {}, 'synthetic'))
    inputs = RuntimeShadow(args.session, model, provenance='live_ros', ball_topic=ball_topic,
                           continuous=args.experimental_active)
    if args.protocol=='relay':
        from yichao_v3_v9.runtime_relay import RuntimeRelay
        client=RuntimeRelay(args.session,model,inputs,mode=args.mode,shots=args.shots,
                            experimental_active=args.experimental_active)
    else:client = MovementClient(args.session, model, args.robot, mode=args.mode)
    pending = queue.Queue(maxsize=128)
    counters = {'queue_drops': 0, 'commands_sent': 0, 'retransmissions': 0}
    sent_tokens=set();feedback_keys={};last_step_key=None;last_queue_drops=0
    rospy.init_node('yichao_move_'+uuid.uuid4().hex, disable_signals=True,
                    disable_rosout=True, disable_rostime=True)
    subscriptions, publishers, states, nonces, command_publishers = [], {}, {}, {}, {}
    def receive(message, topic):
        item = (topic, getattr(message, '_connection_header', {}).get('callerid'),
                message.data, time.monotonic(), time.time())
        try:
            pending.put_nowait(item)
        except queue.Full:
            counters['queue_drops'] += 1
    try:
        for slot, role in (('66', 'table_right'), ('198', 'table_left')):
            publishers[slot] = rospy.Publisher(prefix+'/'+role+'/clock_request', String, queue_size=1, tcp_nodelay=True)
            subscriptions.append(rospy.Subscriber(prefix+'/'+role+'/clock_reply', String,
                receive, callback_args='clock:'+slot, queue_size=1, tcp_nodelay=True))
        for slot in (('66','198') if args.protocol=='relay' else (args.robot,)):
            role = 'table_left' if slot == '198' else 'table_right'
            command_publishers[slot]=rospy.Publisher(prefix+'/'+role+'/command', String, queue_size=1, tcp_nodelay=True)
        for topic in identities:
            # State-history checks use callback receipt times. Request
            # immediate TCP delivery instead of allowing small packets
            # to wait for Nagle/delayed-ACK batching; retain age/gap limits.
            subscriptions.append(rospy.Subscriber(topic, String, receive, callback_args=topic,
                                                 queue_size=1, tcp_nodelay=True))
        deadline, next_probe, next_graph = (float('inf') if args.seconds==0 else time.monotonic()+args.seconds), 0., 0.
        next_step, next_log = 0., 0.
        print(json.dumps({'started': True, 'mode': args.mode, 'session': args.session,
                          'hit_enabled': args.protocol=='relay', 'protocol':args.protocol,
                          'maximum_step_m':None if args.protocol=='relay' else .05}), flush=True)
        while time.monotonic() < deadline and not rospy.is_shutdown():
            now = time.monotonic()
            if counters['queue_drops']!=last_queue_drops:
                if not args.experimental_active:raise RuntimeError('input_queue_overflow')
                write({'kind':'input_queue_overflow','queue_drops':counters['queue_drops']})
                last_queue_drops=counters['queue_drops']
            if now >= next_graph:
                try:
                    current, typs = graph()
                except (OSError,RuntimeError) as exc:
                    if not args.experimental_active:raise
                    write({'kind':'graph_wait','reason':str(exc)})
                else:
                    changed=[t for t,owner in identities.items() if current.get(t)
                             and (current[t]!=[owner] or typs.get(t)!='std_msgs/String')]
                    missing=[t for t in identities if not current.get(t)]
                    if changed or missing and not args.experimental_active:
                        raise RuntimeError('input_publisher_changed')
                    if missing:write({'kind':'input_publishers_wait','topics':missing})
                next_graph = now+1.
            if now >= next_probe:
                for slot in publishers:
                    nonce = uuid.uuid4().hex
                    probe = {'kind': 'probe', 'session': args.session, 'robot': slot,
                             'nonce': nonce, 'workstation_send': time.monotonic()}
                    nonces[slot] = probe
                    publishers[slot].publish(String(data=json.dumps(probe)))
                    write({'kind': 'clock_probe', 'payload': probe,
                           'request_connections': publishers[slot].get_num_connections()})
                next_probe = now+.5
            try:
                topic, caller, text, mono, wall = pending.get(timeout=.005)
                payload = strict_json(text)
                if topic.startswith('clock:'):
                    slot = topic.split(':')[1]
                    expected = nonces.get(slot)
                    role = 'table_left' if slot == '198' else 'table_right'
                    if caller != identities[prefix+'/'+role+'/state']:
                        raise ValueError('clock_publisher_identity')
                    matched = bool(expected and all(payload.get(k) == expected[k] for k in ('session', 'robot', 'nonce', 'workstation_send')))
                    write({'kind': 'clock_reply', 'robot': slot, 'publisher': caller,
                           'receive_monotonic_s': mono, 'processed_monotonic_s': time.monotonic(),
                           'matched_probe': matched, 'payload': payload})
                    if matched:
                        payload.update(kind='commit', workstation_receive=mono)
                        publishers[slot].publish(String(data=json.dumps(payload)))
                        nonces.pop(slot, None)
                else:
                    if caller != identities[topic]:
                        raise ValueError('input_publisher_identity')
                    canonical = wire_states.get(topic, topic)
                    record = {'kind': 'raw_input', 'topic': canonical, 'publisher': caller,
                        'receive_monotonic_s': mono, 'receive_wall_s': wall, 'payload': {'data': text}}
                    write(record)
                    inputs.ingest(record)
                    if topic in wire_states:
                        slot = STATE_TOPICS[canonical]
                        states[slot] = payload
                        if args.protocol=='relay' or slot == args.robot:
                            client.observe(payload)
                        ack=payload.get('yichao_relay',{}).get('feedback')
                        if ack:
                            key=tuple(ack.get(k) for k in ('sequence','token','status','phase','reason','completed'))
                            if feedback_keys.get(slot)!=key:
                                write({'kind':'command_feedback','receive_monotonic_s':mono,'feedback':ack})
                                feedback_keys[slot]=key
            except queue.Empty:
                pass
            if time.monotonic() >= next_step:
                next_step = time.monotonic()+.02
                try:
                    if args.protocol=='relay':result=client.tick(time.monotonic())
                    else:
                        result = client.poll(time.monotonic())
                        if result is None:
                            features = inputs.features(time.monotonic())
                            result = client.step(features, states, time.monotonic())
                except (KeyError, TypeError, ValueError) as exc:
                    result = {'reason': str(exc), 'command': None}
                packets=result.get('commands',[]) if args.protocol=='relay' else ([result['command']] if result.get('command') is not None else [])
                for command in packets:
                    identity=(command['robot'],command['session'],command['sequence'],command['token'])
                    retransmission=identity in sent_tokens
                    if time.monotonic()>command['expires_at']:
                        if not args.experimental_active:raise RuntimeError('command_expired_before_publish')
                        # An identical old packet can retrieve the receiver's
                        # cached ACK; its original expiry prevents first-time
                        # execution. Never refresh an uncertain HIT's time/token.
                        if not retransmission:
                            client.relay.undelivered(command,'missed_shot:command_expired_before_publish')
                            write({'kind':'command_not_sent','reason':'expired_before_publish','command':command})
                            result=client._result('command_expired_before_publish',[])
                            break
                    command_publishers[command['robot']].publish(String(data=json.dumps(command, allow_nan=False)))
                    counters['retransmissions']+=retransmission
                    sent_tokens.add(identity)
                    counters['commands_sent'] += 1
                    write({'kind':'command_tx','command':command,'retransmission':retransmission})
                for event in result.pop('events',[]):
                    write(event)
                    if event['kind']=='planner_input':
                        write({'kind':'planner_decision','session':args.session,'shot':event['shot'],
                               'decision':result.get('decision'),'hitter':client.relay.hitter})
                step_key=(result.get('reason'),client.fault,
                          result.get('relay',{}).get('state'),result.get('relay',{}).get('shot'),
                          result.get('relay',{}).get('skipped_count'))
                if packets or step_key!=last_step_key or time.monotonic() >= next_log:
                    write({'kind': args.protocol+'_step', **result, 'mode': args.mode,
                           'state_changed':step_key!=last_step_key,
                           'log_dropped_records':write.dropped,'log_write_errors':write.write_errors,
                           'feedback_fault': client.fault, 'real_input_accepted': False})
                    last_step_key=step_key
                    next_log = time.monotonic()+.2
                if client.completed or (client.fault and args.mode!='active'):
                    break
    finally:
        for sub in subscriptions:
            sub.unregister()
        for pub in publishers.values():
            pub.unregister()
        for pub in command_publishers.values():pub.unregister()
        write.summary.update(**counters,completed=client.completed,fault=client.fault,
            relay=client.relay.snapshot() if args.protocol=='relay' else None,
            completed_shots=sorted(client.finished) if args.protocol=='relay' else [],
            real_input_accepted=False,existing_motion_stops_on_exit=False)
        rospy.signal_shutdown('planner client finished')
    return 0 if client.completed or args.mode=='active' and not client.fault else 2


if __name__ == '__main__':
    raise SystemExit(main())
