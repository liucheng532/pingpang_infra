"""Read-only wait for two fresh policy-stage samples from each active robot."""
import argparse
from collections import deque
import json
from pathlib import Path
import re
import sys
import threading
import time

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from yichao_v3_v9.observer import load_ros,configure_subscriber_ip
from yichao_v3_v9.telemetry import strict_json


def ready(rows,session,robot,now):
    if len(rows)!=2:return False
    for received,payload in rows:
        e=payload.get('yichao_relay',{})
        if (not 0<=now-received<=.25 or payload.get('valid') is not True
                or e.get('session')!=session or e.get('robot')!=robot
                or e.get('execution_mode')!='active' or e.get('experimental_active') is not True
                or e.get('control_stage')!='policy' or e.get('control_session_invalidated') is True
                or e.get('sensors_recent') is not True):return False
    sequences=[row[1].get('sequence') for row in rows]
    return all(type(s) is int for s in sequences) and sequences[1]>sequences[0]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--session',required=True)
    p.add_argument('--timeout-seconds',type=float,default=0.,
        help='0 keeps waiting for the operator; positive values bound a diagnostic wait')
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.timeout_seconds < 0:p.error('timeout must be nonnegative')
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}',a.session):p.error('safe session required')
    configure_subscriber_ip('172.16.3.126');rospy,String,_=load_ros()
    rospy.init_node('yichao_active_ready_'+a.session,disable_signals=True,disable_rosout=True,disable_rostime=True)
    rows={s:deque(maxlen=2) for s in ('66','198')};stages={};subs=[];lock=threading.Lock()
    def receive(message,slot):
        try:payload=strict_json(message.data)
        except (ValueError,TypeError):return
        if not isinstance(payload,dict):return
        e=payload.get('yichao_relay',{})
        if not isinstance(e,dict):return
        if getattr(message,'_connection_header',{}).get('callerid')!='/yichao_active_'+a.session+'_'+slot:
            return
        with lock:rows[slot].append((time.monotonic(),payload))
        stage=e.get('control_stage')
        if stage!=stages.get(slot):
            stages[slot]=stage;print(slot+' stage='+str(stage),flush=True)
    try:
        for slot,role in [('66','table_right'),('198','table_left')]:
            subs.append(rospy.Subscriber('/yichao_v3_v9/'+a.session+'/'+role+'/state',String,
                receive,callback_args=slot,queue_size=1,tcp_nodelay=True))
        deadline=float('inf') if a.timeout_seconds==0 else time.monotonic()+a.timeout_seconds
        while time.monotonic()<deadline and not rospy.is_shutdown():
            with lock:snapshot={s:list(v) for s,v in rows.items()}
            if all(ready(snapshot[s],a.session,s,time.monotonic()) for s in snapshot):
                a.output.write_text(json.dumps({'session':a.session,'both_policy_ready':True,
                    'observed_monotonic':time.monotonic(),'real_input_accepted':False})+'\n')
                print('Both robots completed R2 startup and report fresh POLICY state.',flush=True);return 0
            time.sleep(.05)
        raise RuntimeError('both-policy readiness timeout; motor processes may still be running')
    finally:
        for sub in subs:sub.unregister()
        rospy.signal_shutdown('readiness observer finished')


if __name__=='__main__':raise SystemExit(main())
