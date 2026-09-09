"""File-only integration: frozen actor/filter, wire inputs, dedicated receivers.

Positions follow a prescribed synthetic plant. This is NOT robot dynamics,
student closed-loop evidence, real sensor acceptance or physical HIT success.
"""
import argparse
import copy
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from yichao_v3_v9.isolation import install_guard
install_guard()
from yichao_v3_v9.inference import Pipeline
from yichao_v3_v9.inputs import Features
from yichao_v3_v9.executor import OfflineExecutor
from yichao_v3_v9.command_receiver import CommandReceiver
from yichao_v3_v9.runtime_shadow import RuntimeShadow,STATE_TOPICS,DOUBLES_BALL
from yichao_v3_v9.runtime_relay import RuntimeRelay
from yichao_v3_v9.manifest import verify


def run(output, *, mode='shadow', shots=2, handoff_delay_s=0., outward_undershoot_m=0.):
    if mode not in ('shadow', 'active'):
        raise ValueError('replay mode must be shadow or active')
    verify()
    examples={r['topic']:r['payload'] for r in map(json.loads,(ROOT/'fixtures/telemetry_synthetic.jsonl').read_text().splitlines())}
    model=Pipeline(199)
    model.decide(Features(np.zeros(36,np.float32),np.zeros(85,np.float32),{},'synthetic'))
    session='synthetic_full_relay'
    inputs=RuntimeShadow(session,model,provenance='synthetic_wire_fixture',ball_topic=DOUBLES_BALL)
    client=RuntimeRelay(session,model,inputs,shots=shots,mode=mode,experimental_active=mode=='active')
    executor=OfflineExecutor(session)
    receivers={s:CommandReceiver(session,s,v,allow_prepare_preemption=mode=='active')
               for s,v in executor.schedulers.items()}
    positions={s:p.copy() for s,p in executor.positions.items()}
    count=0;shot_start=None;shot_number=0;last_complete=None;feedbacks={s:None for s in receivers}
    kinds={};records=0;hit_order=[]
    def wire(topic,payload,now):
        return dict(kind='raw_input',topic=topic,publisher='/synthetic_'+topic.split('/')[2],
                    receive_monotonic_s=now,receive_wall_s=1000.+now,payload={'data':json.dumps(payload)})
    with Path(output).open('x') as log:
        for i in range(int(shots*(450+handoff_delay_s/.02))):
            now=10.+i*.02
            for topic,slot in STATE_TOPICS.items():
                receiver=receivers[slot];scheduler=executor.schedulers[slot]
                before=positions[slot].copy()
                if receiver.active:
                    # HIT stays at the strike location until the frozen
                    # scheduler actually transitions into locomotion.
                    if scheduler.state.name not in ('HIT','POST_DELAY'):
                        goal=receiver.active['goal_y']
                        if receiver.active['kind'] in ('hit','clear'):
                            goal-=np.sign(goal)*outward_undershoot_m
                        positions[slot][1]+=np.clip(goal-positions[slot][1],-.018,.018)
                pos=positions[slot];speed=float((pos[1]-before[1])/.02)
                ref=scheduler.update(-.5,np.zeros(3),np.zeros(3),pos,pos,np.zeros(3),np.zeros(3),dt=.02)
                ack=receiver.observe(now=now,position=pos,velocity_y=speed,sensors_recent=True)
                if ack and (feedbacks[slot] is None or feedbacks[slot].get('status')!='rejected'):
                    feedbacks[slot]={**ack,'execution_mode':mode}
                state=copy.deepcopy(examples['/doubles/table_left/state'])
                state.update(robot=topic.split('/')[2],sequence=i+1,source_monotonic_ns=int((1000+now)*1e9),
                    base_position_xyz=pos.tolist(),base_linear_velocity_xyz=[0,speed,0],phase=ref.state.name,
                    ready=ref.state.name in ('HOME_HOLD','OUTWARD_HOLD'),target_base_y=float(pos[1]+ref.target_base[1]),
                    last_planner_session_id=session,last_applied_sequence=receiver.sequence,
                    transport_error='',q=np.linspace(-.2,.2,29).tolist())
                state['yichao_relay']=dict(schema='yichao-onboard-relay-state-v1',session=session,robot=slot,
                    execution_mode=mode,hit_enabled=True,sensors_recent=True,
                    clock_workstation_interval=[now,now+.001],feedback=feedbacks[slot],input_contract_accepted=False)
                if (handoff_delay_s and client.relay.state=='COMMITTED'
                        and now-client.relay.last_hit_at<handoff_delay_s):
                    # Inject a slow outward phase in synthetic telemetry while
                    # retaining real scheduler application and receiver ACKs.
                    state['phase']='OUTWARD'
                    if state['yichao_relay']['feedback']:
                        state['yichao_relay']['feedback']=copy.deepcopy(state['yichao_relay']['feedback'])
                        state['yichao_relay']['feedback'].update(completed=False,status='accepted',phase='OUTWARD')
                if mode=='active':
                    state['yichao_relay'].update(experimental_active=True,control_stage='policy',
                        control_session_invalidated=False)
                raw=wire(topic,state,now);inputs.ingest(raw);client.observe(state)
            if client.relay.state=='COMPLETE' and last_complete!=client.relay.shot:
                shot_start=None;last_complete=client.relay.shot
            if shot_start is None and i>=6 and not client.completed:
                shot_number+=1;shot_start=now
            ball=copy.deepcopy(examples[DOUBLES_BALL])
            tts=.5 if shot_start is None else .5-(now-shot_start)
            hitter=client.relay.next_hitter
            y=-.2 if hitter=='66' else .2
            ball.update(sequence=i+1,source_timestamp=1000+now,shot_id='shot-'+str(shot_number),
                valid=shot_start is not None and tts>0,position=[1.,y,1.],velocity=[-3.,0.,-.2],
                predicted_strike_position=[.45,y,1.],predicted_strike_velocity=[-2.9,0.,-.3],
                racket_velocity=[2.,0.,.5],time_to_strike_s=max(tts,.001))
            inputs.ingest(wire(DOUBLES_BALL,ball,now))
            result=client.tick(now);acks=[]
            for command in result['commands']:
                kinds[command['kind']]=kinds.get(command['kind'],0)+1
                ack=receivers[command['robot']].apply(command,workstation_time_interval=[now,now+.001],
                    position=positions[command['robot']],sensors_recent=True)
                acks.append(ack);feedbacks[command['robot']]={**ack,'execution_mode':mode}
                if command['kind']=='hit' and ack['status']=='accepted':
                    if not hit_order or hit_order[-1]['shot']!=command['shot']:
                        hit_order.append({'shot':command['shot'],'robot':command['robot']})
            log.write(json.dumps({'provenance':'synthetic_full_relay','now':now,'result':result,
                                 'application_feedback':acks,'positions':{s:p.tolist() for s,p in positions.items()}},allow_nan=False)+'\n')
            records+=1
            if client.completed or client.fault:break
    result=dict(records=records,completed=client.completed,fault=client.fault,
                hit_count={s:len(r.hit_shots) for s,r in receivers.items()},command_counts=kinds,
                first_hitter=client.relay.first_hitter,hit_order=hit_order,
                final_state=client.relay.state,completed_shots=sorted(client.finished),
                evidence='synthetic plant; real actor/filter and frozen scheduler; no physical execution',
                real_input_accepted=False,control_network_access=False,execution_mode=mode,
                policy_stage_evidence='synthetic fixture, not observed hardware readiness',
                injected_handoff_delay_s=handoff_delay_s,injected_outward_undershoot_m=outward_undershoot_m)
    Path(str(output)+'.summary.json').write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--mode',choices=('shadow','active'),default='shadow',
        help='exercise admission logic in a file-only replay; never starts hardware')
    p.add_argument('--shots',type=int,default=2)
    p.add_argument('--handoff-delay-seconds',type=float,default=0.)
    p.add_argument('--outward-undershoot-m',type=float,default=0.)
    args=p.parse_args();result=run(args.output,mode=args.mode,shots=args.shots,
        handoff_delay_s=args.handoff_delay_seconds,outward_undershoot_m=args.outward_undershoot_m)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['completed'] else 2)
