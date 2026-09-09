"""Workstation supervisor for the dedicated full V9 relay protocol.

Formal input acceptance remains separate from explicitly enabled operator
active trials. No ROS or LCM is imported by this module.
"""
import copy
import time
import numpy as np
from .inputs import SLOTS, finite
from .relay import Relay
from .command_receiver import EVIDENCE


class RuntimeRelay:
    def __init__(self, session, pipeline, inputs, *, mode='shadow', shots=2, experimental_active=False,
                 first_hitter='66'):
        if mode not in ('shadow','active') or type(shots) is not int or shots<0 or (shots==0 and mode!='active'):
            raise ValueError('runtime_relay_configuration')
        self.session,self.pipeline,self.inputs,self.mode=session,pipeline,inputs,mode
        if type(experimental_active) is not bool or (experimental_active and mode!='active'):
            raise ValueError('experimental_active_requires_active_mode')
        self.experimental_active=experimental_active
        # Frozen V3 reset() initializes memory from home_y, even though the
        # non-hitter can start outward. This is reset memory, not a fake ACK.
        home=(np.asarray(pipeline.home,dtype=np.float32)*np.float32(1.2)).tolist()
        self.relay=Relay(session,home,feedback_evidence=EVIDENCE,training_timing=True,
                         handoff_timeout_s=None if experimental_active else 5.,first_hitter=first_hitter)
        self.shot_ball=None;self.finished=set();self.shots=shots
        self.resuming=False;self.release_since=None;self.outward_release=None
        self.return_release=None;self.events=[]

    def _event(self,kind,**fields):
        self.events.append({'kind':kind,'session':self.session,'shot':self.relay.shot,**fields})

    @property
    def fault(self):return self.relay.fault

    @property
    def completed(self):return self.shots>0 and len(self.finished)>=self.shots

    def observe(self, state):
        if self.inputs.fault or self.fault:return
        try:
            e=state['yichao_relay'];slot=e['robot']
            if (e['session']!=self.session or slot not in SLOTS or e['execution_mode']!=self.mode
                    or e['hit_enabled'] is not True):
                raise ValueError('dedicated_feedback_identity_or_validity')
            if self.experimental_active and (e.get('control_stage')=='waiting_inputs'
                    or e.get('sensors_recent') is not True or state.get('valid') is not True):
                return
            if state['valid'] is not True or e['sensors_recent'] is not True:
                raise ValueError('dedicated_feedback_identity_or_validity')
            if (self.experimental_active and self.relay.sequence==0
                    and e.get('control_stage')!='policy'
                    and e.get('control_session_invalidated') is not True):
                return  # R2 calibration before the first command: keep waiting.
            if self.mode=='active' and (e.get('control_stage')!='policy'
                    or e.get('control_session_invalidated') is True):
                raise ValueError('active_policy_stage_not_ready')
            ack=e['feedback']
            if ack is not None:
                if ack.get('execution_mode')!=self.mode or ack.get('robot')!=slot:
                    raise ValueError('feedback_mode_or_robot')
                self.relay.observe([ack])
            if self.relay.state=='COMPLETE':self.finished.add(self.relay.shot)
        except (KeyError,TypeError,ValueError) as exc:
            self.relay.fail('feedback_invalid:'+str(exc))

    def _robots(self,now):
        robots={}
        for slot in SLOTS:
            if not self.inputs.states[slot]:raise ValueError('missing_private_robot_state')
            row=self.inputs.states[slot][-1];s=row['payload'];e=s['yichao_relay']
            if not 0<=now-row['receipt']<=.25:raise ValueError('robot_state_expired')
            if (e['session']!=self.session or e['robot']!=slot or e['execution_mode']!=self.mode
                    or e['hit_enabled'] is not True):
                raise ValueError('dedicated_receiver_or_clock_not_ready')
            if e['sensors_recent'] is not True or e['clock_workstation_interval'] is None:
                raise ValueError('waiting_robot_inputs_or_clock')
            if self.mode=='active':
                if e.get('control_stage')=='waiting_inputs' and not e.get('control_session_invalidated'):
                    raise ValueError('waiting_robot_inputs_or_clock')
                rows=list(self.inputs.states[slot])[-2:]
                if len(rows)<2:
                    raise ValueError('active_policy_stage_not_ready')
                for recent in rows:
                    ready=recent['payload']['yichao_relay']
                    if (not 0<=now-recent['receipt']<=.25 or ready.get('control_stage')!='policy'
                            or ready.get('control_session_invalidated') is True
                            or ready.get('session')!=self.session or ready.get('robot')!=slot
                            or ready.get('execution_mode')!='active'):
                        raise ValueError('active_policy_stage_not_ready')
                if e.get('input_contract_accepted') is not True and not (
                        self.experimental_active and e.get('experimental_active') is True):
                    raise ValueError('real_input_contract_not_accepted')
            robots[slot]={**copy.deepcopy(s),'base_position':copy.deepcopy(s['base_position_xyz']),
                          'base_velocity':copy.deepcopy(s['base_linear_velocity_xyz'])}
        return robots

    def _shot_context(self,now,*,before_hit):
        # Only newer predictions for this exact shot can refresh strike data.
        # Another incoming shot must never replace the latched transaction.
        current=self.inputs.ball
        if (self.relay.state in ('PREPARING','PREPARED','CLEARING')
                and current is not None and current.get('prediction_fields_valid') is True
                and current.get('diagnostic_shot_id')==self.relay.shot
                and self.shot_ball is not None
                and current['received_workstation_monotonic_s']>=self.shot_ball['received_workstation_monotonic_s']):
            self.shot_ball=copy.deepcopy(current)
        if self.shot_ball is None:
            raise ValueError('missing_latched_shot_context')
        ball=copy.deepcopy(self.shot_ball)
        age=now-ball['received_workstation_monotonic_s']
        if not 0<=age<=(.30 if before_hit else (float('inf') if self.experimental_active else 5.)):
            raise ValueError('latched_shot_prediction_expired')
        ball['time_to_strike_s']-=age
        return ball

    def _outward_release(self,robots,now):
        # Frozen V9 enters OUTWARD_HOLD on reference_end as well as stable
        # arrival. Release that motion from fresh measured geometry, without
        # labelling an undershot target as completed.
        if not self.experimental_active or self.relay.state not in ('COMMITTED','RETURNING'):
            self.release_since=None
            return
        returning=self.relay.state=='RETURNING'
        phases=('HOME_HOLD','OUTWARD_HOLD') if returning else ('OUTWARD_HOLD',)
        positions={s:float(robots[s]['base_position'][1]) for s in SLOTS}
        valid=(self.relay.accepted==set(SLOTS) and positions['66']<0<positions['198']
            and positions['198']-positions['66']>=.45
            and all(robots[s]['phase'] in phases
                    and abs(float(robots[s]['base_velocity'][1]))<=.15
                    and abs(positions[s])<=1.2 for s in SLOTS))
        if not valid:self.release_since=None;return
        if self.release_since is None:self.release_since=now;return
        if now-self.release_since<.1:return
        release={'basis':'V9 reference hold and measured stable separation',
            'position_y':positions,'stable_duration_s':now-self.release_since,
            'outward_target_completed':sorted(self.relay.completed) if not returning else None,
            'target_completed':sorted(self.relay.completed),'shot':self.relay.shot,
            'target_y':{s:(c['hit']['outward_y'] if c['kind']=='hit' else c['target_y'])
                        for s,c in self.relay.pending.items()}}
        self._event('handoff_release',stage=self.relay.state,**release)
        if returning:
            self.return_release=release
            self.relay.state='COMPLETE'
            self.relay.next_hitter=SLOTS[1-SLOTS.index(self.relay.hitter)]
            self.finished.add(self.relay.shot)
        else:
            self.outward_release=release
            self.relay.state='NEED_RETURN'
        self.release_since=None

    def tick(self,now):
        now=finite(now,'runtime_now');start=time.perf_counter()
        self.tick_started=start
        commands=[];reason=None
        try:
            if self.inputs.fault:raise ValueError('input_fault:'+self.inputs.fault)
            if self.fault:raise ValueError(self.fault)
            if self.completed:return self._result('requested_relays_completed',[])
            robots=self._robots(now)
            self.resuming=False
            self._outward_release(robots,now)
            if self.experimental_active and self.relay.state in ('PREPARING','PREPARED','CLEARING'):
                ball=self._shot_context(now,before_hit=True)
                if ball['time_to_strike_s']<.12:
                    self.relay.skip('missed_shot:commit_tts_expired')
                    return self._result('missed_shot:commit_tts_expired',[])
            if self.relay.state in self.relay.POLL_STATES:
                # Polling ACKs does not invoke either network or choose a new
                # target. Continue checking current robot/clock/sensor state;
                # validate ball freshness before a pending HIT can be sent.
                record=dict(now=now,robots=robots,shot_id=self.relay.shot)
                if self.relay.state=='CLEARING':
                    record['ball']=self._shot_context(now,before_hit=True)
                # After HIT, completion is established by measured robot
                # feedback and the handoff deadline, not incoming-ball age.
                commands=self.relay.poll(record)
            elif self.relay.state in ('PREPARED','NEED_RETURN'):
                # Retain only this shot's strike context. Robot histories and
                # joints must be rebuilt before selecting CLEAR or RETURN.
                ball=self._shot_context(now,before_hit=self.relay.state=='PREPARED')
                features=(self.inputs.handoff_features(now,self.shot_ball,context_max_age_s=None)
                    if self.experimental_active and self.relay.state=='NEED_RETURN'
                    else self.inputs.handoff_features(now,self.shot_ball))
                features.record.update(now=now,robots=robots,shot_id=self.relay.shot,ball=ball)
                commands=self.relay.step(features,self.pipeline)
            else:
                features=self.inputs.features(now)
                if self.mode=='active' and not self.experimental_active and features.record.get('real_input_accepted') is not True:
                    raise ValueError('real_feature_contract_not_accepted')
                features=copy.deepcopy(features)
                features.actor[-2:]=np.asarray(self.relay.previous_targets,np.float32)/np.float32(1.2)
                ball=copy.deepcopy(self.inputs.ball)
                ball['time_to_strike_s']-=now-ball['received_workstation_monotonic_s']
                features.record.update(now=now,robots=robots,ball=ball,
                    previous_target_basis='frozen V3 reset home_y, then dedicated prepare application feedback')
                self.shot_ball=copy.deepcopy(self.inputs.ball)
                if features.record['shot_id'] not in self.relay.seen:
                    self._event('planner_input',shot=features.record['shot_id'],
                        actor_observation=features.actor.tolist(),safe_observation=features.safe.tolist(),
                        input_evidence=features.record)
                commands=self.relay.step(features,self.pipeline)
            if commands and time.perf_counter()-start>.02:
                if self.experimental_active:
                    self._event('slow_planner_step',elapsed_s=time.perf_counter()-start)
                else:commands=self.relay.fail('end_to_end_decision_deadline_exceeded')
            reason=self.fault
        except (KeyError,TypeError,ValueError,RuntimeError) as exc:
            reason=str(exc)
            # An idle observer may warm up or wait for a ball. Once a relay has
            # begun, invalid inputs stop new commands and require a new session.
            transient=reason in ('missing_private_robot_state','robot_state_expired',
                'waiting_robot_inputs_or_clock','base_history_warming_up','base_history_gap',
                'robot_receive_age_or_pair_skew','ball_state_receipt_skew','ball_receive_age',
                'no_valid_incoming_shot','robot_hit_locked','expired_tts')
            if self.experimental_active and reason=='latched_shot_prediction_expired':
                self.relay.skip('missed_shot:'+reason)
            elif self.experimental_active and transient:
                self.release_since=None
            elif not self.resuming and self.relay.state not in ('IDLE','COMPLETE','FAULT'):
                self.relay.fail('runtime_input_or_execution:'+reason)
        return self._result(reason,commands)

    def _result(self,reason,commands):
        events,self.events=self.events,[]
        return {'reason':reason,'commands':commands,'relay':self.relay.snapshot(),'events':events,
                'processing_s':time.perf_counter()-getattr(self,'tick_started',time.perf_counter()),
                'decision':self.relay.decision,'completed_shots':sorted(self.finished),
                'execution_mode':self.mode,'real_input_accepted':False,'experimental_active':self.experimental_active,
                'outward_release':self.outward_release,'return_release':self.return_release,'resuming':self.resuming,
                'existing_motion_stops_on_exit':False}
