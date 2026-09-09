import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from yichao_v3_v9.runtime_shadow import RuntimeShadow, MONITOR, STATE_TOPICS, DOUBLES_BALL
from yichao_v3_v9.inference import Pipeline
from doubles_planner.v0 import IsaacV1ShotPlannerEnv
from test_observer import fixture, STATE
from test_ball_feedback import message
from test_selector_wire import stock_bridge

torch.set_num_threads(1)


def raw(topic, payload, now):
    return {'kind':'raw_input', 'topic':topic, 'publisher':'/fixture-'+topic.split('/')[-2],
            'receive_monotonic_s':now, 'receive_wall_s':990.+now,
            'payload':{'data':json.dumps(payload)}}


def state(topic, seq, now):
    p=fixture(STATE); p['robot']=topic.split('/')[2]
    sign=-1 if p['robot']=='table_right' else 1
    p.update(sequence=seq, source_monotonic_ns=1000000000+int(now*1e9),
             base_position_xyz=[.1*(now-10), sign*.6+.02*(now-10), .75],
             target_base_y=sign*.65, q=(np.linspace(-.2,.2,29)*sign).tolist())
    return raw(topic,p,now)


def ball(seq, now, phase):
    p=message(seq,phase);p['timestamp']=990.+now
    return raw(MONITOR,p,now)


def ready(pipeline=None):
    if pipeline is None:
        pipeline=Mock()
        pipeline.decide.return_value={'valid':True,'target_y':[-.5,.5]}
    runtime=RuntimeShadow('synthetic',pipeline,provenance='synthetic_wire_fixture')
    runtime.ingest(ball(1,9.99,'waiting'))
    for seq,t in enumerate((10.,10.017,10.041,10.062,10.084,10.10),1):
        for topic in STATE_TOPICS:
            runtime.ingest(state(topic,seq,t))
    runtime.ingest(ball(2,10.10,'tracking'))
    return runtime


class RuntimeTests(unittest.TestCase):
    def test_private_onboard_stale_sensors_and_wrong_identity_are_rejected(self):
        for patch, reason in [({'sensors_recent':False},'onboard_sensor_freshness'),
                              ({'robot':'198'},'onboard_state_identity'),
                              ({'session':'old'},'onboard_state_identity')]:
            r=ready()
            record=state('/doubles/table_right/state',7,10.12)
            payload=json.loads(record['payload']['data'])
            payload['yichao_movement']={'schema':'yichao-onboard-movement-state-v1',
                'session':'synthetic','robot':'66','sensors_recent':True,**patch}
            record['payload']['data']=json.dumps(payload)
            r.ingest(record)
            self.assertEqual(r.fault,reason)
            self.assertIn(reason,r.tick(10.12)['reason'])
            r.pipeline.decide.assert_not_called()

    def test_vectors_against_frozen_training_methods_with_designated_urdf(self):
        r=ready();f=r.features(10.10);e=f.record
        h={n:torch.tensor([e['base_histories'][i]]) for i,n in enumerate(('left','right'))}
        b=r.ball
        env=NS(num_envs=1,device='cpu',config=NS(observation=NS(planner=NS(workspace_y=(-1.2,1.2)))),
            ball_position=torch.tensor([b['position']]),ball_velocity=torch.tensor([b['velocity']]),
            ball_acceleration=torch.zeros(1,3),ball_position_history=torch.zeros(1,5,3),
            ball_history_valid=torch.ones(1,5,dtype=torch.bool),
            base_history_valid={n:torch.ones(1,5,dtype=torch.bool) for n in h},base_position_history=h,
            ball_target=torch.tensor([b['strike_position']]),target_racket_velocity=torch.tensor([b['racket_velocity']]),
            time_to_strike=torch.tensor([b['time_to_strike_s']]),ball_initialized=torch.tensor([True]))
        env._step_dt=lambda:.02
        env._base_positions=lambda:(h['left'][:,-1],h['right'][:,-1],None,None)
        env.robots={n:NS(data=NS(joint_pos=torch.tensor([e['robots'][s]['q']]),
            soft_joint_pos_limits=torch.tensor(r.normalizer.soft))) for n,s in zip(h,('66','198'))}
        w=object.__new__(IsaacV1ShotPlannerEnv);w.env=env;w.config=env.config;w.num_envs=1;w.device='cpu'
        w.previous_applied_targets=torch.tensor([e['observed_targets_y']])
        np.testing.assert_allclose(f.actor,w._build_observation().numpy()[0],atol=2e-7,rtol=0)
        np.testing.assert_allclose(f.safe,w._build_safe_observation().numpy()[0],atol=2e-7,rtol=0)

    def test_irregular_callbacks_are_interpolated_on_20ms_grid(self):
        r=ready();f=r.features(10.10)
        grid=np.array(f.record['base_history_times'])
        np.testing.assert_allclose(np.diff(grid),.02,atol=1e-9)
        for h in f.record['base_histories']:
            np.testing.assert_allclose(np.asarray(h)[:,0],.1*(grid-10),atol=1e-8)
        self.assertNotEqual(grid.tolist(),[10.017,10.041,10.062,10.084,10.10])

    def test_handoff_refreshes_robot_features_after_ball_expires(self):
        r=ready();before=r.features(10.1);shot_ball=copy.deepcopy(r.ball)
        r.ball=None
        for seq,t in enumerate(np.arange(11.1,11.221,.02),7):
            for topic in STATE_TOPICS:
                row=state(topic,seq,float(t));payload=json.loads(row['payload']['data'])
                payload['q']=[.7]*29
                row['payload']['data']=json.dumps(payload)
                r.ingest(row)
        f=r.handoff_features(11.22,shot_ball)
        self.assertTrue(f.record['handoff_only'])
        self.assertFalse(np.array_equal(before.safe[:78],f.safe[:78]))
        np.testing.assert_allclose(f.safe[20:49],r.normalizer.normalize([.7]*29,r.normalizer.profile['joint_names']))
        np.testing.assert_array_equal(f.safe[78:84],before.safe[78:84])
        self.assertAlmostEqual(float(f.safe[84]),shot_ball['time_to_strike_s']-(f.record['sample_time']-10.1),places=6)
        with self.assertRaisesRegex(ValueError,'no_valid_incoming_shot'):r.features(11.22)
        with self.assertRaisesRegex(ValueError,'robot_receive_age'):r.handoff_features(11.6,shot_ball)

    def test_missing_history_and_state_are_not_padded(self):
        r=ready();r.states['66'].clear()
        self.assertEqual(r.tick(10.10)['reason'],'missing_v9_robot_state')
        r=ready()
        while len(r.states['66'])>2:r.states['66'].popleft()
        self.assertEqual(r.tick(10.10)['reason'],'base_history_warming_up')
        r.pipeline.decide.assert_not_called()

    def test_same_shot_is_latched_and_external_memory_not_overwritten(self):
        r=ready();before=r.features(10.10).actor[-2:].copy()
        first=r.tick(10.10); second=r.tick(10.101)
        self.assertEqual(len(first['movement_preview']),2)
        self.assertEqual(second['reason'],'shot_already_decided')
        r.pipeline.decide.assert_called_once()
        np.testing.assert_array_equal(r.features(10.10).actor[-2:],before)
        self.assertFalse(first['command_ack'])

    def test_preview_is_invalid_but_stage_geometry_is_consumed_by_stock_bridge(self):
        out=ready().tick(10.10)
        for preview in out['movement_preview']:
            self.assertFalse(preview['valid']);self.assertFalse(preview['command']['active'])
            self.assertEqual(preview['command']['role'],'stage')
            scheduler=Mock();stock_bridge(preview).apply_pending(scheduler,[0,0,.75])
            scheduler.set_external_base_target.assert_not_called()
            # Pure isolated bridge acceptance test: never publish this copy.
            test_wire=copy.deepcopy(preview);test_wire['valid']=True
            bridge=stock_bridge(test_wire);bridge.apply_pending(scheduler,[0,0,.75])
            scheduler.set_external_base_target.assert_called_once()
            scheduler.set_external_hit.assert_not_called()
            goal=scheduler.set_external_base_target.call_args.args[0]
            self.assertAlmostEqual(goal[1],preview['command']['desired_base_position'][1])

    def test_unsafe_and_slow_decisions_produce_no_preview(self):
        r=ready();r.pipeline.decide.return_value={'valid':False,'reason':'no_safe_candidate'}
        self.assertEqual(r.tick(10.1)['movement_preview'],[])
        self.assertEqual(r.tick(10.101)['reason'],'shot_already_decided')
        r=ready()
        with patch('yichao_v3_v9.runtime_shadow.time.perf_counter',side_effect=[0.,.021]):
            out=r.tick(10.1)
        self.assertEqual(out['reason'],'decision_deadline_exceeded')
        self.assertEqual(out['movement_preview'],[])

    def test_stale_future_locked_and_history_gap(self):
        for now in (9.9,10.5):
            r=ready();self.assertEqual(r.tick(now)['movement_preview'],[])
            r.pipeline.decide.assert_not_called()
        r=ready();r.states['66'][-1]['payload']['phase']='HIT'
        self.assertEqual(r.tick(10.1)['reason'],'robot_hit_locked')
        r=ready();r.states['66']=type(r.states['66'])([r.states['66'][0],r.states['66'][-1]])
        self.assertEqual(r.tick(10.1)['reason'],'base_history_gap')

    def test_duplicate_restart_publisher_and_invalid_quaternion_latch(self):
        topic=next(iter(STATE_TOPICS))
        for mutation in ('duplicate','restart','publisher','quaternion','stop','transport'):
            r=ready();rec=state(topic,7,10.12)
            p=json.loads(rec['payload']['data'])
            if mutation=='duplicate':p['sequence']=6
            if mutation=='restart':p['source_monotonic_ns']=1
            if mutation=='publisher':rec['publisher']='/another_controller'
            if mutation=='quaternion':p['base_orientation_wxyz']=[0,0,0,0]
            if mutation=='stop':p['emergency_stop']=True
            if mutation=='transport':p['transport_error']='command_rejected'
            rec['payload']['data']=json.dumps(p);r.ingest(rec)
            self.assertIn('input_fault_latched',r.tick(10.12)['reason'])
            r.pipeline.decide.assert_not_called()

    def test_waiting_monitor_and_fresh_state_do_not_infer(self):
        r=ready();r.ingest(ball(3,10.11,'waiting'))
        self.assertEqual(r.tick(10.11)['reason'],'no_valid_incoming_shot')
        r.pipeline.decide.assert_not_called()

    def test_real_models_on_synthetic_wire_input(self):
        pipeline=Pipeline();r=ready(pipeline)
        # Warmup before checking the budget, as done by both entry points.
        pipeline.decide(r.features(10.1))
        out=r.tick(10.1)
        self.assertIn('decision',out)
        self.assertEqual(len(out['actor']),36);self.assertEqual(len(out['safe']),85)
        self.assertEqual(out['decision']['actor_iteration'],199)
        self.assertFalse(out['real_input_accepted'])
        self.assertEqual(out['provenance'],'synthetic_wire_fixture')

    def test_original_doubles_ball_and_monitor_are_not_mixed(self):
        r=ready();r.ball_topic=DOUBLES_BALL;r.ball=None
        p=fixture(DOUBLES_BALL);p['source_timestamp']=1000.1
        r.ingest(raw(DOUBLES_BALL,p,10.1))
        self.assertEqual(r.tick(10.1)['shot_id'],'synthetic/mocap-1')
        r.ingest(ball(3,10.11,'waiting'))
        self.assertIsNotNone(r.ball)
        p['valid']=False;r.ingest(raw(DOUBLES_BALL,p,10.12))
        self.assertIsNone(r.ball)


if __name__=='__main__':unittest.main(verbosity=2)
