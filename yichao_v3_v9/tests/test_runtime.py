import sys
import time
from pathlib import Path
import numpy as np
import pytest
from yichao_v3_v9 import ROOT
from yichao_v3_v9.runtime import YichaoRuntime
from doubles_planner.real_ros_runtime import ROBOT_ORDER
sys.path.insert(0, str(ROOT/'baseline/tests'))
from test_real_ros_runtime import robot_state, ball_prediction
sys.path.insert(0, str(ROOT/'baseline/controller_reference/g1_gym_deploy/tests'))
from test_planner_ros_bridge import _bridge, FakeScheduler


class Model:
    def __init__(self):
        self.calls = 0
    def decide(self, features):
        self.calls += 1
        assert features.actor.shape == (36,)
        assert features.safe.shape == (85,)
        return {'valid': True, 'target_y': [-.4, .4], 'reason': None}


def warm(runtime, start=1.):
    for i in range(6):
        for name, y in zip(ROBOT_ORDER, (-.2, .7)):
            runtime.update_robot_state(name, robot_state(name, y, i+1), start+i*.02)


def incoming(runtime, now=1.1, sequence=1, shot='shot-1', tts=.5):
    ball = ball_prediction(sequence)
    ball.update(shot_id=shot, time_to_strike_s=tts, source_timestamp=time.time())
    runtime.update_ball(ball, now)
    return ball


def prepared(runtime, now=1.12, sequence=7):
    """Prescribed feedback for protocol replay; this is not simulated physics."""
    decision = runtime.position_strategy.decision
    assert decision and decision['valid']
    for name,y in zip(ROBOT_ORDER, decision['target_y']):
        state=robot_state(name,y,sequence);state['target_base_y']=y
        runtime.update_robot_state(name,state,now)
    return runtime.tick(now)


def test_no_ball_is_idle_without_inference_or_movement():
    model = Model(); runtime = YichaoRuntime(pipeline=model, shadow=False)
    warm(runtime)
    output = runtime.tick(1.1)
    assert model.calls == 0
    assert all(not p['valid'] and p['command']['role'] == 'hold' for p in output.values())


@pytest.mark.parametrize('positions', [(-.4, .7), (-.2, .4), (-.2, .7)])
def test_hit_does_not_wait_for_either_robot_to_reach_model_target(positions):
    runtime = YichaoRuntime(pipeline=Model(), shadow=False)
    for i in range(6):
        for name, y in zip(ROBOT_ORDER, positions):
            runtime.update_robot_state(name, robot_state(name, y, i+1), 1.+i*.02)
    incoming(runtime)
    output = runtime.tick(1.1)
    assert output['table_right']['command']['role'] == 'hit'
    assert output['table_right']['command']['active']
    assert not output['table_left']['command']['active']
    assert all('moving_to_model_targets' not in p['admission_reasons'] for p in output.values())


def test_model_targets_reach_existing_hit_and_movement_bridge():
    runtime = YichaoRuntime(pipeline=Model(), shadow=False)
    warm(runtime); incoming(runtime, tts=.6)
    initial = runtime.tick(1.1)
    assert all(p['command']['role'] == 'stage' for p in initial.values())
    incoming(runtime, now=1.12, sequence=2)
    output = prepared(runtime)
    hitter = next(name for name,p in output.items() if p['command']['active'])
    peer = next(name for name in ROBOT_ORDER if name != hitter)
    assert output[hitter]['return_target_y'] == runtime._desired_runtime_motion_config(hitter).home_y
    for name in ROBOT_ORDER:
        bridge = _bridge(output[name] if name == hitter else initial[name]); bridge.robot_id = name
        scheduler = FakeScheduler()
        update = bridge.apply_pending(scheduler, np.array([0., -.2 if name == 'table_right' else .7, .75]))
        assert bridge.last_error == '' if hasattr(bridge, 'last_error') else bridge._last_error == ''
        if name == hitter:
            assert update.starts_hit
            assert update.return_target_y == output[name]['return_target_y']
        else:
            assert scheduler.calls[0][0] == 'base'
            assert np.isclose(scheduler.calls[0][1][1], .4 if name == 'table_left' else -.4)


def test_history_warmup_recovers_without_restart():
    runtime = YichaoRuntime(pipeline=Model(), shadow=False)
    incoming(runtime, 1.)
    runtime.tick(1.)
    warm(runtime)
    output = runtime.tick(1.1)
    assert output['table_right']['command']['role'] == 'hit'
    assert output['table_right']['command']['active']


def test_shadow_never_produces_executable_commands():
    runtime = YichaoRuntime(pipeline=Model())
    warm(runtime); incoming(runtime)
    runtime.tick(1.1)
    output = prepared(runtime)
    assert any(p['planned_active'] for p in output.values())
    assert all(not p['valid'] and not p['command']['active'] for p in output.values())


def test_existing_return_path_accepts_yichao_wire_mode():
    from test_planner_ros_bridge import _command
    payload = _command(role='return', active=False, planner_mode='fixed_relay')
    bridge = _bridge(payload); scheduler = FakeScheduler()
    bridge.apply_pending(scheduler, np.array([.1,.7,.75]))
    assert bridge._last_error == ''
    assert np.isclose(scheduler.calls[0][1][1], .35)


def test_source_tree_keeps_fixed_control_unchanged():
    import hashlib,json
    onboard = ROOT/'baseline/controller_reference'
    manifest = json.loads((onboard/'SOURCE_MANIFEST.json').read_text())
    changed = [name for name, meta in manifest.items()
               if hashlib.sha256((onboard/name).read_bytes()).hexdigest() != meta['sha256']]
    assert changed == []
    baseline = ROOT/'baseline'
    manifest = json.loads((baseline/'SOURCE_MANIFEST.json').read_text())
    assert all(hashlib.sha256((baseline/name).read_bytes()).hexdigest() == meta['sha256']
               for name,meta in manifest.items())


@pytest.mark.parametrize("real_model", [False, True])
def test_model_targets_reach_original_observe_and_real_reference_assets(real_model):
    from test_return_pose_sync import observe_method, agent_for
    from utils.data_utils import MotionCommand, MoveMotionBank
    from utils.doubles_reference_scheduler import DoublesReferenceScheduler
    from utils.mirrored_reference_scheduler import MirroredReferenceScheduler
    import json
    reference = ROOT/'baseline/controller_reference'
    policy = reference/'policy/v11_resume_i42500'
    sidecar = json.loads((policy/'student_v11_r2i299_commonhold_1666_resume42500.onnx.json').read_text())
    hit = MotionCommand(str(reference/'data/0302_combined'), device='cpu')
    move = MoveMotionBank(str(reference/'data/0718-move-160-80hz'),
                          excluded_source_ids=sidecar['move_pool_contract']['excluded_source_ids'],
                          expected_active_count=147)
    runtime = YichaoRuntime(pipeline=None if real_model else Model(), shadow=False)
    warm(runtime);incoming(runtime, tts=.6)
    output = runtime.tick(1.1)
    assert all(p['command']['role']=='stage' for p in output.values())
    for name, y in zip(ROBOT_ORDER, (-.2, .7)):
        canonical = DoublesReferenceScheduler(hit, move, external_control=True,
            v10_relative_x=True, reference_end_forces_hold=True, latch_hold_x_on_entry=True,
            common_hold_pose_path=str(policy/'source29_0368_upright_fk_v1.npz'),
            common_hold_pose_sha256=sidecar['common_hold_pose_sha256'])
        scheduler = MirroredReferenceScheduler(canonical) if name == 'table_right' else canonical
        pelvis = np.array([.18, y, .75], np.float32)
        torso = pelvis+np.array([.02,0.,.2],np.float32)
        scheduler.reset(pelvis, torso)
        bridge = _bridge(output[name]);bridge.robot_id = name
        agent = agent_for(scheduler, pelvis, torso, bridge)
        observe_method()(agent)
        assert bridge._last_error == ''
        actual_y = pelvis[1]+agent.reference.target_base[1]
        assert np.isclose(actual_y, output[name]['command']['desired_base_position'][1], atol=1e-6)
        assert np.isfinite(agent.reference.command).all()


def test_return_is_retried_using_original_protocol_after_ball_disappears():
    runtime = YichaoRuntime(pipeline=Model(), shadow=False)
    warm(runtime);incoming(runtime)
    runtime.tick(1.1)
    output = prepared(runtime)
    hitter = next(name for name,p in output.items() if p['command']['active'])
    peer = next(n for n in ROBOT_ORDER if n != hitter)
    token = output[hitter]['commit_token']
    ball = ball_prediction(2);ball['valid'] = False;runtime.update_ball(ball,1.14)
    for seq,t in [(8,1.14),(9,1.16)]:
        hs=robot_state(hitter,-.3,seq);hs.update(phase='OUTWARD', last_commit_token=token)
        ps=robot_state(peer,.7,seq);ps.update(phase='OUTWARD_HOLD')
        runtime.update_robot_state(hitter,hs,t);runtime.update_robot_state(peer,ps,t)
        outputs=runtime.tick(t)
        p=outputs[peer]
        assert runtime.last_features is None
        assert p['command']['role']=='return' and p['valid']
        assert p['planner_mode']=='fixed_relay'
        bridge=_bridge(p);bridge.robot_id=peer;scheduler=FakeScheduler()
        bridge.apply_pending(scheduler,np.array([.1,.7,.75]))
        assert bridge._last_error=='' and scheduler.calls[0][0]=='base'


def test_six_shots_with_real_actor_filter_and_prescribed_controller_feedback():
    runtime=YichaoRuntime(shadow=False)
    seq=0;ball_seq=0;positions=dict(zip(ROBOT_ORDER,(-.2,.7)));hitters=[]
    def states(t,phases=None,token=None,hitter=None,ack=None):
        nonlocal seq
        seq+=1
        for name in ROBOT_ORDER:
            s=robot_state(name,positions[name],seq)
            s.update(phase=(phases or {}).get(name,'HOME_HOLD'),target_base_y=positions[name])
            if name==hitter:s['last_commit_token']=token
            if ack is not None:s.update(last_applied_sequence=ack,last_planner_session_id=runtime.session_id)
            runtime.update_robot_state(name,s,t)
    for turn in range(6):
        base=1.+turn*.5
        for step in range(6):states(base+step*.02)
        t=base+.1;ball_seq+=1;incoming(runtime,t,ball_seq,f'shot-{turn}')
        outputs=runtime.tick(t)
        decision=runtime.position_strategy.decision
        assert decision['valid'], decision
        positions.update(zip(ROBOT_ORDER,decision['target_y']))
        states(t+.02);outputs=runtime.tick(t+.02)
        hitter=next(n for n,p in outputs.items() if p['command']['active'])
        peer=next(n for n in ROBOT_ORDER if n!=hitter)
        hitters.append(hitter);token=outputs[hitter]['commit_token']
        states(t+.04,{hitter:'HIT',peer:'OUTWARD_HOLD'},token,hitter);runtime.tick(t+.04)
        ball_seq+=1;invalid=ball_prediction(ball_seq);invalid['valid']=False;runtime.update_ball(invalid,t+.06)
        states(t+.06,{hitter:'OUTWARD',peer:'OUTWARD_HOLD'},token,hitter)
        outputs=runtime.tick(t+.06)
        assert outputs[peer]['command']['role']=='return'
        ack=outputs[peer]['sequence']
        states(t+.08,{hitter:'OUTWARD',peer:'RETURN'},token,hitter,ack);runtime.tick(t+.08)
        positions[peer]=runtime._desired_runtime_motion_config(peer).home_y
        states(t+.10,{hitter:'OUTWARD_HOLD',peer:'HOME_HOLD'},token,hitter,ack);runtime.tick(t+.10)
    assert hitters==['table_right','table_left']*3
