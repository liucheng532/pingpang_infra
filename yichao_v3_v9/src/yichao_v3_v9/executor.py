"""Frozen V9 scheduler harness. All feedback is simulated, never robot acknowledgement."""
import copy
import hashlib
import json
from dataclasses import dataclass
import numpy as np
from . import ROOT
from .inputs import SLOTS, array, finite
from utils.data_utils import MotionCommand, MoveMotionBank
from utils.doubles_reference_scheduler import DoublesReferenceScheduler
from utils.mirrored_reference_scheduler import MirroredReferenceScheduler


def clone_scheduler(scheduler):
    """Copy state, share immutable banks (avoids copying the entire motion library per tick)."""
    mirrored=isinstance(scheduler,MirroredReferenceScheduler)
    original=scheduler.scheduler if mirrored else scheduler
    result=copy.copy(original)
    shared={'hit_motions','move_motions','_hit_q','_hit_qd','_hit_targets','_hit_lengths'}
    for key,value in vars(original).items():
        if key not in shared:
            setattr(result,key,copy.deepcopy(value))
    return MirroredReferenceScheduler(result) if mirrored else result


class OfflineExecutor:
    def __init__(self, session, banks=None):
        if not isinstance(session,str) or not session:
            raise ValueError('session must be explicit')
        self.session=session
        if banks is None:
            banks=(MotionCommand(str(ROOT/'assets/0302_combined')),MoveMotionBank(str(ROOT/'assets/0718-move-160-80hz')))
        self.schedulers={}
        self.positions={'66':np.array([0,-.9125,.75],np.float32),'198':np.array([0,.2,.75],np.float32)}
        self.accepted={}
        self.sequences={s:-1 for s in SLOTS}
        self.tokens=set()
        self.hit_shots=set()
        self.history={}
        self.stable={s:0. for s in SLOTS}
        self.hit_count={s:0 for s in SLOTS}
        self.last_tick=None
        for slot in SLOTS:
            canonical=DoublesReferenceScheduler(*banks,external_control=True,transition_s=.1,target_hold_reference_transition_s=.3)
            scheduler=MirroredReferenceScheduler(canonical) if slot=='66' else canonical
            scheduler.reset(self.positions[slot],self.positions[slot])
            if slot=='66': scheduler.bootstrap_outward_hold(-.2,-.9125)
            self.schedulers[slot]=scheduler

    def _validate(self,c,now):
        if not isinstance(c,dict): raise ValueError('command_must_be_object')
        expected={'schema','session','sequence','shot','token','robot','kind','target_y','issued_at','expires_at','hit'}
        if set(c) not in (expected,expected|{'return_y'}) or c['schema']!='yichao-v3-v9-command-v1': raise ValueError('command_schema')
        if c['session']!=self.session: raise ValueError('unregistered_or_old_session')
        if c['robot'] not in SLOTS or c['kind'] not in ('move','clear','return','hit'): raise ValueError('robot_or_kind')
        if type(c['sequence']) is not int or c['sequence']<0: raise ValueError('sequence')
        for key in ('shot','token'):
            if not isinstance(c[key],str) or not c[key]: raise ValueError(key)
        target=finite(c['target_y'],'target_y')
        if not -1.2<=target<=1.2: raise ValueError('target_workspace')
        issued=finite(c['issued_at'],'issued_at');expires=finite(c['expires_at'],'expires_at')
        if not issued<=now<=expires or not 0<expires-issued<=.251: raise ValueError('expired_or_future_command')
        if c.get('return_y') is not None and abs(finite(c['return_y'],'return_y'))>1.2: raise ValueError('return_workspace')
        if c['kind']=='hit':
            h=c['hit']
            if not isinstance(h,dict) or set(h)!={'position','racket_velocity','ball_velocity','tts','outward_y','return_y'}: raise ValueError('hit_schema')
            for key in ('position','racket_velocity','ball_velocity'): array(h[key],(3,),key)
            for key in ('outward_y','return_y'):
                if abs(finite(h[key],key))>1.2: raise ValueError('hit_target_workspace')
            tts=finite(h['tts'],'tts')-(now-issued)
            if not .12<=tts<=.55: raise ValueError('hit_tts_window')
        elif c['hit'] is not None: raise ValueError('unexpected_hit_fields')
        return json.dumps(c,sort_keys=True,allow_nan=False)

    def apply_batch(self,commands,now):
        """Atomic *offline* preview. Not a claim that distributed robot commits are atomic."""
        if not isinstance(commands,(list,tuple)):
            return [{'evidence':'simulated_scheduler','status':'rejected','reason':'batch_must_be_array','session':self.session}]
        try:
            finite(now,'now')
            if not commands or len(commands)>2 or len({c.get('robot') for c in commands if isinstance(c,dict)})!=len(commands): raise ValueError('batch_robots')
            encoded=[self._validate(c,now) for c in commands]  # entire batch before any mutation
            cached=[]
            for c,text in zip(commands,encoded):
                identity=(c['robot'],c['sequence'])
                if identity in self.history:
                    old_text,ack=self.history[identity]
                    if old_text!=text: raise ValueError('sequence_payload_conflict')
                    cached.append(copy.deepcopy(ack))
            if cached:
                if len(cached)!=len(commands): raise ValueError('mixed_duplicate_batch')
                return cached
            previews={}
            acks=[]
            if sum(c['kind']=='hit' for c in commands)>1: raise ValueError('simultaneous_hit')
            for c in commands:
                slot=c['robot'];identity=(slot,c['kind'],c['token'])
                if c['sequence']<=self.sequences[slot]: raise ValueError('old_sequence')
                if identity in self.tokens: raise ValueError('reused_token')
                if c['kind']=='hit' and c['shot'] in self.hit_shots: raise ValueError('shot_already_hit')
                scheduler=clone_scheduler(self.schedulers[slot]);before=scheduler.output()
                if c['kind']=='hit':
                    peer=self.schedulers[SLOTS[1-SLOTS.index(slot)]]
                    if peer.state.name in ('HIT','POST_DELAY'): raise ValueError('peer_hit_in_progress')
                    h=c['hit']
                    scheduler.set_external_hit(h['position'],h['racket_velocity'],h['tts']-(now-c['issued_at']),h['ball_velocity'])
                    scheduler.set_external_outward_target(h['outward_y'])
                    scheduler.set_external_home_target(h['return_y'])
                    goal=h['outward_y']
                else:
                    scheduler.set_external_base_target(np.array([self.positions[slot][0],c['target_y']],np.float32),return_target_y=c.get('return_y') if c.get('return_y') is not None else c['target_y'],safety_override=False)
                    goal=c['target_y']
                    actual=float(self.positions[slot][1]+scheduler.output().target_base[1])
                    if abs(actual-goal)>1e-4:
                        raise ValueError('scheduler_did_not_latch_target')
                    if c['kind']=='return' and scheduler.state.name not in ('RETURN','HOME_HOLD'):
                        raise ValueError('v9_cannot_express_return_for_target_direction')
                previews[slot]=scheduler
                after=scheduler.output()
                acks.append({'schema':'yichao-v3-v9-feedback-v1','evidence':'simulated_scheduler',
                    'session':self.session,'robot':slot,'sequence':c['sequence'],'shot':c['shot'],'token':c['token'],
                    'kind':c['kind'],'status':'accepted','goal_y':float(goal),'applied_target_y':float(c['target_y']),
                    'scheduler_strike_time_s':float(after.strike_time),
                    'phase':after.state.name,'reference_changed':before.move_motion_index!=after.move_motion_index,
                    'move_motion_index':after.move_motion_index,'hit_motion_index':after.hit_motion_index,'completed':False})
            for c,text,ack in zip(commands,encoded,acks):
                slot=c['robot'];self.schedulers[slot]=previews[slot]
                self.sequences[slot]=c['sequence'];self.tokens.add((slot,c['kind'],c['token']))
                self.accepted[slot]=copy.deepcopy(ack);self.stable[slot]=0.
                self.history[(slot,c['sequence'])]=(text,copy.deepcopy(ack))
                self.hit_count[slot]+=int(c['kind']=='hit')
                if c['kind']=='hit': self.hit_shots.add(c['shot'])
            return acks
        except (KeyError,TypeError,ValueError,RuntimeError) as error:
            return [{'evidence':'simulated_scheduler','status':'rejected','reason':str(error),'session':self.session,'robot':c.get('robot'),'sequence':c.get('sequence'),'token':c.get('token'),'shot':c.get('shot')} for c in (item if isinstance(item,dict) else {} for item in commands)]

    def tick(self,now,positions):
        now=finite(now,'tick_time')
        dt=.02 if self.last_tick is None else now-self.last_tick
        if not 0<dt<=.05: raise ValueError('scheduler_tick_gap_or_clock_jump')
        validated={s:array(positions[s],(3,),'pelvis') for s in SLOTS}
        self.last_tick=now
        feedback={}
        for slot in SLOTS:
            scheduler=self.schedulers[slot];position=validated[slot]
            speed=float(np.linalg.norm((position-self.positions[slot])[:2])/dt)
            self.positions[slot]=position.copy()
            ref=scheduler.update(-.5,np.zeros(3),np.zeros(3),position,position,np.zeros(3),np.zeros(3),dt=dt)
            ack=copy.deepcopy(self.accepted.get(slot))
            if ack is not None:
                at_goal=abs(float(position[1])-ack['goal_y'])<=.06
                hold=ref.state.name in ('HOME_HOLD','OUTWARD_HOLD')
                self.stable[slot]=self.stable[slot]+dt if at_goal and speed<=.15 and hold else 0.
                ack.update(phase=ref.state.name,stable_elapsed_s=self.stable[slot],position_y=float(position[1]),lateral_speed=speed,
                    completed=self.stable[slot]>=.1-1e-8,status='completed' if self.stable[slot]>=.1-1e-8 else 'accepted')
                feedback[slot]=ack
        return feedback
