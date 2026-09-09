"""Dedicated V9 MOVE/CLEAR/HIT/RETURN receiver; transport is owned by the caller.

Application feedback is produced only after committing a fully validated
scheduler preview. Completion is separately measured from phase and pose.
"""
import copy
import numpy as np
from .executor import OfflineExecutor, clone_scheduler
from .inputs import array, finite
from utils.mirrored_reference_scheduler import MirroredReferenceScheduler
from utils.left_right_mirror import mirror_polar_vectors

EVIDENCE = 'dedicated_v9_receiver'


class CommandReceiver:
    def __init__(self, session, robot, scheduler, *, allow_prepare_preemption=False):
        if not isinstance(session,str) or not session or robot not in ('66','198'):
            raise ValueError('receiver_identity')
        self.session,self.robot,self.scheduler=session,robot,scheduler
        self.sequence=-1; self.history={}; self.tokens=set(); self.prepared=set(); self.hit_shots=set()
        self.active=None; self.stable_since=None; self.last_observation=None
        self.allow_prepare_preemption=allow_prepare_preemption

    def apply(self, command, *, workstation_time_interval, position, sensors_recent):
        result={'schema':'yichao-v3-v9-feedback-v1','evidence':EVIDENCE,
                'session':self.session,'robot':self.robot,'status':'rejected','completed':False}
        if isinstance(command,dict):
            result.update({k:command.get(k) for k in ('sequence','shot','token','kind')})
        try:
            c=command
            # Validate shape and all fields independently of transit age first,
            # so an identical retry can report prior application after expiry.
            issued=finite(c['issued_at'],'issued_at')
            encoded=OfflineExecutor._validate(self,c,issued)
            if c['robot']!=self.robot: raise ValueError('wrong_robot')
            cached=self.history.get(c['sequence'])
            if cached:
                if encoded!=cached[0]:raise ValueError('sequence_payload_conflict')
                return copy.deepcopy(cached[1])
            if not isinstance(workstation_time_interval,(tuple,list)) or len(workstation_time_interval)!=2:
                raise ValueError('clock_interval')
            early,late=[finite(x,'clock_bound') for x in workstation_time_interval]
            if early>late or late-early>.02:raise ValueError('clock_uncertainty')
            OfflineExecutor._validate(self,c,early)
            OfflineExecutor._validate(self,c,late)
            if sensors_recent is not True:raise ValueError('sensor_freshness_required')
            if c['sequence']<=self.sequence:raise ValueError('old_sequence')
            if c['token'] in self.tokens:raise ValueError('reused_token')
            if self.scheduler.state.name in ('HIT','POST_DELAY'):raise ValueError('hit_locked')
            current=array(position,(3,),'current_pelvis')
            if c['kind']=='move':
                if c['shot'] in self.prepared:raise ValueError('shot_already_prepared')
                abandoned_prepare=(self.allow_prepare_preemption and self.active
                    and self.active['kind'] in ('move','clear'))
                settled_return=(self.allow_prepare_preemption and self.active
                    and self.active['kind']=='return'
                    and self.scheduler.state.name in ('HOME_HOLD','OUTWARD_HOLD'))
                if self.active and not self.active['completed'] and not (abandoned_prepare or settled_return):
                    raise ValueError('previous_relay_not_completed')
            elif c['kind'] in ('hit','clear'):
                if not self.active or self.active['kind']!='move' or self.active['shot']!=c['shot']:
                    raise ValueError('matching_prepare_required')
                if c['kind']=='hit':
                    if c['shot'] in self.hit_shots:raise ValueError('shot_already_hit')
                    if abs(self.active['applied_target_y']-c['target_y'])>1e-4:
                        raise ValueError('prepared_target_changed_before_hit')
            elif c['kind']=='return':
                if not self.active or self.active['kind'] not in ('hit','clear') or self.active['shot']!=c['shot']:
                    raise ValueError('matching_commit_required_for_return')
            preview=clone_scheduler(self.scheduler)
            if c['kind']=='hit':
                h=c['hit']
                preview.set_external_hit(h['position'],h['racket_velocity'],h['tts']-(late-issued),h['ball_velocity'])
                preview.set_external_outward_target(h['outward_y'])
                preview.set_external_home_target(h['return_y'])
                if preview.state.name!='HIT':raise ValueError('scheduler_did_not_enter_hit')
                goal=h['outward_y']
            else:
                goal=c['target_y']
                # LCMAgent applies pending commands before the scheduler's next
                # update. Its reference offsets still use the preceding pose;
                # validate/latch against that same frame, not mixed-frame X/Y.
                canonical=getattr(preview,'scheduler',preview)
                reference_position=np.asarray(canonical._pelvis_position,np.float32)
                if isinstance(preview,MirroredReferenceScheduler):
                    reference_position=mirror_polar_vectors(reference_position)
                preview.set_external_base_target(np.array([reference_position[0],goal],np.float32),
                    return_target_y=c['return_y'] if c.get('return_y') is not None else goal,
                    safety_override=False)
                actual=float(reference_position[1]+preview.output().target_base[1])
                if abs(actual-goal)>1e-4:raise ValueError('scheduler_did_not_latch_target')
                if c['kind']=='return' and preview.state.name not in ('RETURN','HOME_HOLD'):
                    raise ValueError('v9_cannot_express_return_for_target_direction')
            ref=preview.output()
            result.update(status='accepted',reason=None,goal_y=float(goal),
                applied_target_y=float(c['target_y']),phase=ref.state.name,
                scheduler_strike_time_s=float(ref.strike_time),
                move_motion_index=int(ref.move_motion_index),hit_motion_index=int(ref.hit_motion_index))
            # Preserve the scheduler object held by LCMAgent; all rejection
            # paths above leave it untouched, including malformed HIT fields.
            destination=getattr(self.scheduler,'scheduler',self.scheduler)
            source=getattr(preview,'scheduler',preview)
            destination.__dict__.update(source.__dict__)
            self.sequence=c['sequence'];self.tokens.add(c['token'])
            if c['kind']=='move':self.prepared.add(c['shot'])
            if c['kind']=='hit':self.hit_shots.add(c['shot'])
            self.active=copy.deepcopy(result)
            self.history[self.sequence]=(encoded,copy.deepcopy(result))
            self.stable_since=self.last_observation=None
            return result
        except (KeyError,TypeError,ValueError,RuntimeError) as exc:
            return {**result,'reason':str(exc)}

    def observe(self, *, now, position, velocity_y, sensors_recent):
        if self.active is None:return None
        if self.active['completed']:return copy.deepcopy(self.active)
        now=finite(now,'observation_time');p=array(position,(3,),'pelvis')
        speed=abs(finite(velocity_y,'velocity_y'))
        continuous=self.last_observation is None or 0<now-self.last_observation<=.05+1e-9
        self.last_observation=now
        hold=self.scheduler.state.name in ('HOME_HOLD','OUTWARD_HOLD')
        if not (sensors_recent is True and continuous and hold and speed<=.15 and
                abs(float(p[1])-self.active['goal_y'])<=.06):
            self.stable_since=None
        elif self.stable_since is None:self.stable_since=now
        stable=0. if self.stable_since is None else now-self.stable_since
        completed=stable>=.1-1e-8
        self.active.update(phase=self.scheduler.state.name,position_y=float(p[1]),
            lateral_speed=speed,stable_elapsed_s=stable,completed=completed,
            status='completed' if completed else 'accepted')
        encoded,_=self.history[self.sequence]
        self.history[self.sequence]=(encoded,copy.deepcopy(self.active))
        return copy.deepcopy(self.active)
