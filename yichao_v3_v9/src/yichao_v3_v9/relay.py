"""Conservative offline prepare/commit relay; telemetry phase alone is never ACK."""
import copy
import time
import math
from types import SimpleNamespace
from .inputs import SLOTS
from .protocol import V3Protocol


class Relay:
    POLL_STATES = frozenset(('PREPARING','CLEARING','COMMITTED','RETURNING'))
    def __init__(self,session,initial_targets,*,feedback_evidence='simulated_scheduler',training_timing=False,handoff_timeout_s=5.,first_hitter=None):
        if first_hitter not in (None,*SLOTS):raise ValueError('unknown_first_hitter')
        self.session=session
        self.first_hitter=first_hitter
        self.previous_targets=list(initial_targets)
        self.state='IDLE';self.shot=None;self.hitter=first_hitter or '198';self.next_hitter=self.hitter
        self.sequence=0;self.seen=set();self.pending={};self.accepted=set();self.completed=set()
        self.decision=None;self.started=None;self.last_hit_at=None;self.fault=None
        self.protocol=V3Protocol();self.commit_plan=None;self.protocol_checks=[]
        self.feedback_evidence=feedback_evidence
        self.training_timing=training_timing
        self.handoff_timeout_s=handoff_timeout_s
        self.continuous=handoff_timeout_s is None
        self.last_skipped=None;self.skipped_count=0;self.renew=set()

    def skip(self,reason):
        """A missed pre-HIT opportunity does not end a continuous session."""
        self.last_skipped={'shot':self.shot,'reason':reason,'state':self.state}
        self.skipped_count+=1
        self.pending={};self.accepted=set();self.completed=set();self.renew=set()
        self.state='IDLE';self.fault=None
        return []

    def fail(self,reason):
        if self.continuous and self.state in ('IDLE','PREPARING','PREPARED','CLEARING') and (
                reason in ('prepare_timeout','prepare_ack_timeout','clear_ack_timeout',
                           'commit_tts_expired','decision_deadline_exceeded','no_safe_candidate')
                or reason.startswith('missed_shot:')):
            return self.skip(reason)
        self.state='FAULT';self.fault=reason
        return []

    def undelivered(self,command,reason):
        """Handle a packet known not to have executed, without inventing an ACK."""
        if not self.continuous:return self.fail(reason)
        if command['kind']=='return':
            self.renew.add(command['robot'])
            return []
        return self.skip(reason)

    def command(self,slot,kind,now,ball=None,target=None,return_y=None):
        self.sequence+=1
        hit=None
        if kind=='hit':
            hit={'position':ball['strike_position'],'racket_velocity':ball['racket_velocity'],
                'ball_velocity':ball['strike_velocity'],'tts':ball['time_to_strike_s'],
                'outward_y':self.commit_plan['hitter_outward_y'],'return_y':self.commit_plan['return_pair'][SLOTS.index(slot)]}
        return {'schema':'yichao-v3-v9-command-v1','session':self.session,'sequence':self.sequence,
            'shot':self.shot,'token':f'{self.session}:{self.shot}:{slot}:{kind}', 'robot':slot,'kind':kind,
            'target_y':self.decision['target_y'][SLOTS.index(slot)] if target is None else float(target),'return_y':return_y,'issued_at':now,'expires_at':now+.25,'hit':hit}

    def observe(self,feedback):
        if self.state=='FAULT': return
        for ack in feedback:
            slot=ack.get('robot');expected=self.pending.get(slot)
            if expected is None: continue
            if any(ack.get(key)!=expected[key] for key in ('session','sequence','shot','token','robot')):
                continue
            if (self.continuous and ack.get('evidence')=='active_stage_admission'
                    and ack.get('status')=='rejected'
                    and ack.get('reason')=='control_stage_not_policy:waiting_inputs'):
                self.undelivered(expected,'missed_shot:waiting_inputs');return
            if ack.get('evidence')!=self.feedback_evidence: continue
            if ack.get('status')=='rejected':
                if self.continuous and ack.get('reason') in (
                        'expired_or_future_command','sensor_freshness_required','hit_locked'):
                    self.undelivered(expected,'missed_shot:'+ack['reason']);return
                self.fail('executor_rejected:'+str(ack.get('reason')));return
            if ack.get('status') not in ('accepted','completed') or ack.get('kind')!=expected['kind']:
                continue
            if any(not isinstance(ack.get(k),(int,float)) or isinstance(ack.get(k),bool) or not math.isfinite(ack[k]) for k in ('goal_y','applied_target_y')):
                self.fail('nonfinite_ack_target');return
            expected_goal=expected['hit']['outward_y'] if expected['kind']=='hit' else expected['target_y']
            if abs(ack.get('goal_y',float('inf'))-expected_goal)>1e-5 or abs(ack.get('applied_target_y',float('inf'))-expected['target_y'])>1e-5:
                self.fail('ack_target_mismatch');return
            self.accepted.add(slot)
            if self.state=='PREPARING':
                self.previous_targets[SLOTS.index(slot)]=ack['applied_target_y']
            if ack.get('completed') is True and all(isinstance(ack.get(k),(int,float)) and not isinstance(ack.get(k),bool) and math.isfinite(ack[k]) and ack[k]>=0 for k in ('stable_elapsed_s','lateral_speed')) and isinstance(ack.get('position_y'),(int,float)) and math.isfinite(ack['position_y']) and ack['status']=='completed' and ack.get('phase') in ('HOME_HOLD','OUTWARD_HOLD') and ack.get('stable_elapsed_s',0)>=.1-1e-8 and abs(ack.get('position_y',float('inf'))-expected_goal)<=.06 and ack.get('lateral_speed',float('inf'))<=.15:
                self.completed.add(slot)
        prepared=self.accepted if self.training_timing and self.protocol.locomotion_preemption else self.completed
        if self.state=='PREPARING' and prepared==set(SLOTS): self.state='PREPARED'
        if self.state=='COMMITTED' and self.completed==set(SLOTS):self.state='NEED_RETURN'
        if self.state=='RETURNING' and self.completed==set(SLOTS):
            self.state='COMPLETE';self.next_hitter=SLOTS[1-SLOTS.index(self.hitter)]

    def poll(self,record):
        """Advance an existing transaction without constructing model inputs.

        These states only use current telemetry, aged shot context and matching
        feedback. No actor/filter may run here: there are deliberately no
        feature arrays or pipeline available. CLEAR and RETURN selection still
        require step() with freshly constructed safety features.
        """
        if self.state not in self.POLL_STATES:
            raise ValueError('relay_state_requires_features:'+self.state)
        return self.step(SimpleNamespace(record=record),None)

    def step(self,features,pipeline):
        r=features.record;now=r['now'];shot=r['shot_id']
        if self.state=='FAULT': return []
        if not self.continuous and self.state in ('PREPARING','PREPARED','CLEARING') and now-self.started>2:
            return self.fail('prepare_timeout')
        if (self.state in ('COMMITTED','NEED_RETURN','RETURNING') and self.handoff_timeout_s is not None
                and now-self.last_hit_at>self.handoff_timeout_s):
            return self.fail('handoff_timeout')
        if self.state in ('IDLE','COMPLETE'):
            if shot in self.seen: return []
            self.seen.add(shot);self.shot=shot;self.started=now
            self.protocol_checks=[]
            if self.last_hit_at is not None and now-self.last_hit_at>10:
                self.hitter=self.first_hitter or min(SLOTS,key=lambda s:abs(r['robots'][s]['base_position'][1]))
            else: self.hitter=self.next_hitter
            start=time.perf_counter()
            self.decision=None
            self.decision=pipeline.decide(features)
            if not self.continuous and time.perf_counter()-start>.02: return self.fail('decision_deadline_exceeded')
            if not self.decision['valid']:
                return self.skip(self.decision['reason']) if self.continuous else self.fail(self.decision['reason'])
            commands=[self.command(s,'move',now) for s in SLOTS]
            self.pending={c['robot']:c for c in commands};self.accepted=set();self.completed=set();self.state='PREPARING'
            return commands
        if shot!=self.shot and self.state in ('PREPARING','PREPARED','CLEARING'): return []  # no HIT from another shot; finish existing handoff independently
        if self.state=='PREPARING':
            waiting=[c for s,c in self.pending.items() if s not in self.accepted]
            if any(now>c['expires_at'] for c in waiting): return self.fail('prepare_ack_timeout')
            return copy.deepcopy(waiting)
        if self.state=='CLEARING':
            peer=SLOTS[1-SLOTS.index(self.hitter)]
            if peer not in self.accepted:
                if now>self.pending[peer]['expires_at']:return self.fail('clear_ack_timeout')
                return [copy.deepcopy(self.pending[peer])]
            if not .12<=r['ball']['time_to_strike_s']<=.55:return self.fail('commit_tts_expired')
            if any(r['robots'][s]['phase'] in ('HIT','POST_DELAY') for s in SLOTS):
                return self.skip('commit_phase_locked') if self.continuous else self.fail('commit_phase_locked')
            if r['robots']['198']['base_position'][1]-r['robots']['66']['base_position'][1]<.45:
                return self.skip('actual_base_gap') if self.continuous else self.fail('actual_base_gap')
            try:self.commit_plan=self.protocol.commit(self.hitter,r)
            except (ValueError,RuntimeError) as error:return self.fail('commit_protocol:'+str(error))
            hit=self.command(self.hitter,'hit',now,r['ball'])
            self.pending[self.hitter]=hit
            self.state='COMMITTED';self.last_hit_at=now
            return [hit]
        if self.state=='PREPARED':
            # Frozen V3 permits locomotion preemption after explicit prepare
            # application and its reservation interval. Phase alone is no ACK.
            if self.training_timing and now-self.started<self.protocol.reservation_duration_s-1e-9:return []
            if r['ball']['time_to_strike_s']>.55: return []
            if r['ball']['time_to_strike_s']<.12: return self.fail('commit_tts_expired')
            phases=('OUTWARD','RETURN','HOME_HOLD','OUTWARD_HOLD') if self.training_timing and self.protocol.locomotion_preemption else ('HOME_HOLD','OUTWARD_HOLD')
            if any(r['robots'][s]['phase'] not in phases for s in SLOTS): return []
            if r['robots']['198']['base_position'][1]-r['robots']['66']['base_position'][1]<.45:
                return self.skip('actual_base_gap') if self.continuous else self.fail('actual_base_gap')
            try:self.commit_plan=self.protocol.commit(self.hitter,r)
            except (ValueError,RuntimeError) as error:return self.fail('commit_protocol:'+str(error))
            peer=SLOTS[1-SLOTS.index(self.hitter)]
            targets=[r['robots'][s]['base_position'][1] for s in SLOTS]
            targets[SLOTS.index(peer)]=self.commit_plan['peer_clear_y']
            if not self._protocol_safe(features,pipeline,targets,'peer_clear'):return []
            clear=self.command(peer,'clear',now,target=self.commit_plan['peer_clear_y'],return_y=self.commit_plan['return_pair'][SLOTS.index(peer)])
            if self.training_timing:
                # Two robots cannot share the simulation's atomic commit. Get
                # explicit peer CLEAR application before emitting any HIT.
                self.pending={peer:clear};self.accepted=set();self.completed=set();self.state='CLEARING'
                return [clear]
            commands=[clear,self.command(self.hitter,'hit',now,r['ball'])]
            self.pending={c['robot']:c for c in commands};self.accepted=set();self.completed=set()
            self.state='COMMITTED';self.last_hit_at=now
            return commands
        if self.state=='NEED_RETURN':
            try:targets=self.protocol.return_targets(self.commit_plan['return_pair'],r)
            except (ValueError,RuntimeError) as error:return self.fail('return_protocol:'+str(error))
            if not self._protocol_safe(features,pipeline,targets,'return'):return []
            commands=[self.command(s,'return',now,target=targets[i],return_y=targets[i]) for i,s in enumerate(SLOTS)]
            self.pending={c['robot']:c for c in commands};self.accepted=set();self.completed=set();self.state='RETURNING'
            return commands
        if self.state in ('COMMITTED','RETURNING'):
            for slot in list(self.renew):
                old=self.pending[slot]
                fresh=self.command(slot,'return',now,target=old['target_y'],return_y=old['return_y'])
                fresh['token']+=':retry:'+str(fresh['sequence'])
                self.pending[slot]=fresh;self.accepted.discard(slot);self.renew.remove(slot)
            waiting=[c for s,c in self.pending.items() if s not in self.accepted]
            if not self.continuous and any(now>c['expires_at'] for c in waiting):return self.fail('protocol_ack_timeout')
            return copy.deepcopy(waiting)
        return []

    def _protocol_safe(self,features,pipeline,targets,stage):
        import numpy as np
        pair=np.asarray(targets,dtype=np.float32)
        if not np.isfinite(pair).all() or np.any(np.abs(pair)>1.2) or pair[1]-pair[0]<.45:
            self.fail('protocol_target_gap_or_workspace:'+stage);return False
        risk=float(pipeline.risks(features.safe,(pair/1.2)[None,:])[0])
        self.protocol_checks.append({'stage':stage,'target_y':pair.tolist(),'risk':risk,'threshold':pipeline.threshold,
            'safe_observation':features.safe.tolist(),
            'sample_time':features.record.get('sample_time',features.record['now']),
            'safe_observation_basis':features.record.get('safe_observation_basis','provided step features'),
            'risk_evidence':'diagnostic only: V3 filter is calibrated at shot decision, not mid-shot protocol stages',
            'guard':'finite workspace/gap plus frozen V3 motion feasibility; live geometry gate remains closed'})
        return True

    def snapshot(self):
        return {'session':self.session,'state':self.state,'shot':self.shot,'hitter':self.hitter,
            'first_hitter':self.first_hitter,'next_hitter':self.next_hitter,
            'previous_applied_targets':self.previous_targets,'accepted':sorted(self.accepted),
            'completed':sorted(self.completed),'fault':self.fault,'decision':self.decision,
            'commit_plan':self.commit_plan,'protocol_checks':self.protocol_checks,
            'last_skipped':self.last_skipped,'skipped_count':self.skipped_count,
            'started_at':self.started,'last_hit_at':self.last_hit_at,
            'continuous':self.continuous}
