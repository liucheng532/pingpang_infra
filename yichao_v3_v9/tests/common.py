import copy,json
import numpy as np
from yichao_v3_v9 import ROOT
from yichao_v3_v9.inputs import InputAdapter
CONTRACT=json.loads((ROOT/'config/interface.json').read_text())

def record(now=10.,shot='shot-1',positions=None,phases=None):
    positions=positions or {'66':[0,-.9125,.75],'198':[0,.2,.75]}
    stamp={'domain':'replay_monotonic','source':now,'received':now,'uncertainty_s':0.}
    r={'provenance':'synthetic','frame':'training_common_fixture','clock_domain':'replay_monotonic',
        'now':now,'shot_id':shot,'emergency_stop':False,'base_history_times':(now-np.arange(4,-1,-1)*.02).tolist(),'robots':{}}
    for s in ('66','198'):
        r['robots'][s]={'robot_id':s,'role':'table_right' if s=='66' else 'table_left','timestamps':{k:copy.deepcopy(stamp) for k in ('low','base','controller')},
            'base_position':list(positions[s]),'base_history':[list(positions[s]) for _ in range(5)],
            'base_quaternion':[1,0,0,0],'imu_quaternion':[1,0,0,0],'base_velocity':[0,0,0],
            'base_angular_velocity':[0,0,0],'gyro':[0,0,0],'q':np.linspace(-.3,.3,29).tolist(),'dq':[0.]*29,
            'joint_order':CONTRACT['fixture_joint_order'],'phase':(phases or {}).get(s,'OUTWARD_HOLD' if s=='66' else 'HOME_HOLD'),
            'ready':True,'valid':True,'phase_elapsed_s':.2,'stable_elapsed_s':.2}
    r['ball']={'position':[1.,.1,1.2],'velocity':[-2.,.1,-.2],'acceleration':[0,0,0],
        'strike_position':[.4,.2,1.1],'strike_velocity':[-2.,.1,-.2],
        'racket_velocity':[1.,.2,.3],'racket_normal':[1,0,0],'time_to_strike_s':.5,'timestamp':copy.deepcopy(stamp),'valid':True}
    return r

def features(r=None,previous=(-.9125,.2)):
    return InputAdapter(CONTRACT).build(record() if r is None else r,previous)

def command(robot='198',kind='move',target=.5,seq=1,session='test',now=10.,token=None):
    hit={'position':[.4,.2,1.1],'racket_velocity':[1,.2,.3],'ball_velocity':[-2,0,0],'tts':.5,'outward_y':.9125 if robot=='198' else -.9125,'return_y':.2 if robot=='198' else -.2} if kind=='hit' else None
    return {'schema':'yichao-v3-v9-command-v1','session':session,'sequence':seq,'shot':'s1','token':token or f'{robot}:{kind}:{seq}','robot':robot,'kind':kind,'target_y':target,'issued_at':now,'expires_at':now+.25,'hit':hit}
