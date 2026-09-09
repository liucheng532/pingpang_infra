import copy,json,sys,time,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import torch
from yichao_v3_v9 import ROOT,HANDOFF
from yichao_v3_v9.inputs import InputAdapter
from yichao_v3_v9.inference import Pipeline,project
from yichao_v3_v9.executor import OfflineExecutor
from yichao_v3_v9.relay import Relay
from yichao_v3_v9.manifest import verify,sha
from doubles_planner.v0 import FrozenV0SafetyFilter,FrozenV0SafetyConfig,IsaacV1ShotPlannerEnv
from common import CONTRACT,record,features,command

torch.set_num_threads(1)
RESULTS={}

class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pipeline=Pipeline()
        cls.reference=FrozenV0SafetyFilter(FrozenV0SafetyConfig(checkpoint=HANDOFF/'checkpoints/safe_filter_v3.pt',enabled=True,candidate_radius=.75,candidate_grid_points=7),device='cpu')

    def test_manifest_and_whole_v9_identity(self):
        self.assertGreater(len(verify()['sha256']),600)
        for slot in ('66','198'):
            base=ROOT/'assets'/('robot_'+slot)
            self.assertEqual(sha(base/'student_v9_m14500_timedhandoff_1666_model19000.onnx'),'7e7e26d71895eb900c7b95fa2cd2fb69cbe8207586869390138c6f758848cae7')
            for filename in ('doubles_reference_scheduler.py','mirrored_reference_scheduler.py','left_right_mirror.py','planner_ros_bridge.py'):
                self.assertEqual(sha(base/'g1_gym_deploy/utils'/filename),sha(ROOT/'vendor/deploy/g1_gym_deploy/utils'/filename))

    def test_low_level_onnx_abi_only(self):
        import onnxruntime as ort
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        model=ort.InferenceSession(str(ROOT/'assets/robot_66/student_v9_m14500_timedhandoff_1666_model19000.onnx'),options,providers=['CPUExecutionProvider'])
        output=model.run(None,{model.get_inputs()[0].name:np.zeros((1,1666),np.float32)})[0]
        self.assertEqual(output.shape,(1,29));self.assertTrue(np.isfinite(output).all())
        RESULTS['low_level_model']={'input':[1,1666],'output':[1,29],'evidence':'ABI smoke only; zero input is not a valid robot state'}

    def test_actor_190_199_and_three_filter_outputs(self):
        errors={}
        for iteration in (190,199):
            pipeline=Pipeline(iteration)
            reference=torch.jit.load(str(HANDOFF/f'onnx/rl_planner_actor_model_{iteration}.pt')).eval()
            rng=np.random.default_rng(19000)
            for batch in (1,7,32):
                obs=rng.normal(0,.5,(batch,36)).astype(np.float32)
                state=rng.normal(0,.5,(batch,85)).astype(np.float32)
                act=rng.uniform(-1,1,(batch,2)).astype(np.float32)
                actual=pipeline.actor.run(None,{'observation':obs})[0]
                with torch.no_grad(): expected=reference(torch.tensor(obs)).numpy()
                error=float(np.max(np.abs(actual-expected)))
                self.assertLessEqual(error,2e-5);errors[f'actor_{iteration}_batch_{batch}']=error
                with torch.no_grad():
                    inputs=torch.cat((torch.tensor(state),torch.tensor(act)),dim=1)
                    heads=torch.stack([torch.sigmoid(m(inputs)) for m in self.reference.models],dim=1)
                    members=heads.amax(dim=2)
                    conservative=(members.mean(1)+members.std(1,unbiased=False)).clamp(0,1)
                actuals=pipeline.scorer.run(['head_probabilities','member_risk','conservative_risk'],{'safe_observation':state,'candidate_action':act})
                for name,actual,expected in zip(('heads','members','risk'),actuals,(heads,members,conservative)):
                    error=float(np.max(np.abs(actual-expected.numpy())))
                    self.assertLessEqual(error,2e-5);errors[f'filter_{name}_batch_{batch}']=error
        self.assertEqual(self.pipeline.threshold,self.reference.threshold)
        RESULTS['onnx_parity_max_abs_error']=errors

    def test_feature_vectors_against_frozen_source(self):
        r=record()
        for j,s in enumerate(('66','198')):
            h=np.array(r['robots'][s]['base_history'])
            h[:,0]=np.arange(5)*.01+j*.05
            h[:,1]+=np.arange(5)*.003
            r['robots'][s]['base_history']=h.tolist();r['robots'][s]['base_position']=h[-1].tolist()
        r['ball']['acceleration']=[.1,-.2,-9.81]
        built=features(r,(-.6,.7))
        env=NS(num_envs=1,device='cpu',config=NS(observation=NS(planner=NS(workspace_y=(-1.2,1.2)))),
            ball_position=torch.tensor([r['ball']['position']]),ball_velocity=torch.tensor([r['ball']['velocity']]),
            ball_acceleration=torch.tensor([r['ball']['acceleration']]),ball_position_history=torch.zeros(1,5,3),
            ball_history_valid=torch.ones(1,5,dtype=torch.bool),base_history_valid={n:torch.ones(1,5,dtype=torch.bool) for n in ('left','right')},
            base_position_history={n:torch.tensor([r['robots'][s]['base_history']],dtype=torch.float32) for n,s in zip(('left','right'),('66','198'))},
            ball_target=torch.tensor([r['ball']['strike_position']]),target_racket_velocity=torch.tensor([r['ball']['racket_velocity']]),
            time_to_strike=torch.tensor([.5]),ball_initialized=torch.tensor([True]))
        env._step_dt=lambda:.02
        env._base_positions=lambda:(env.base_position_history['left'][:,-1],env.base_position_history['right'][:,-1],None,None)
        env.robots={n:NS(data=NS(joint_pos=torch.tensor([r['robots'][s]['q']]),soft_joint_pos_limits=torch.tensor(CONTRACT['fixture_joint_limits']))) for n,s in zip(('left','right'),('66','198'))}
        wrapper=object.__new__(IsaacV1ShotPlannerEnv);wrapper.env=env;wrapper.config=env.config;wrapper.num_envs=1;wrapper.device='cpu';wrapper.previous_applied_targets=torch.tensor([[-.6,.7]])
        obs=wrapper._build_observation().numpy()[0];safe=wrapper._build_safe_observation().numpy()[0]
        np.testing.assert_allclose(built.actor,obs,atol=2e-7,rtol=0)
        np.testing.assert_allclose(built.safe,safe,atol=2e-7,rtol=0)
        # Save reference outputs produced by the frozen training feature methods.
        artifact={'provenance':'synthetic inputs + frozen V3 pure feature methods; no Isaac simulation',
            'input':r,'previous_targets':[-.6,.7],'actor_reference':obs.tolist(),'safe_reference':safe.tolist()}
        (ROOT/'fixtures/reference_vectors.json').write_text(json.dumps(artifact,indent=2)+'\n')

    def test_selection_matches_reference(self):
        rng=np.random.default_rng(11)
        for _ in range(12):
            state=rng.normal(0,.4,85).astype(np.float32);action=rng.uniform(-1,1,2).astype(np.float32)
            actual=self.pipeline.select(state,action)
            expected=self.reference.filter(torch.tensor(state[None]),torch.tensor(action[None]),home_action=torch.tensor(self.pipeline.home[None]),minimum_normalized_gap=.375)
            np.testing.assert_allclose(actual['filtered'],expected.action.numpy()[0],atol=2e-6,rtol=0)
            self.assertEqual(actual['fallback'],bool(expected.fallback[0]))

    def test_threshold_grid_fallback_and_projection(self):
        state=np.zeros(85,np.float32)
        for label,risk,valid in [('boundary',self.pipeline.threshold,True),('above',self.pipeline.threshold+1e-6,False)]:
            with patch.object(self.pipeline,'risks',side_effect=lambda s,a:np.full(len(a),risk,np.float64)):
                result=self.pipeline.select(state,np.array([-.5,.5]))
            self.assertEqual(result['valid'],valid,label)
        with patch.object(self.pipeline,'risks',side_effect=lambda s,a:np.zeros(len(a))):
            nominal=self.pipeline.select(state,np.array([-.5,.5]));grid=self.pipeline.select(state,np.array([.8,-.8]))
            self.assertEqual(nominal['candidate_index'],0);self.assertTrue(grid['filter_intervened'])
        with patch.object(self.pipeline,'risks',side_effect=lambda s,a:np.ones(len(a))):
            self.assertFalse(self.pipeline.select(state,np.array([-.5,.5]))['valid'])
        for act in ([4,-4],[.5,.5],[-2,2]):
            p,changed=project(act)
            self.assertGreaterEqual(float((p[1]-p[0])*1.2),.45-1e-6)
            self.assertTrue(np.all(np.abs(p)<=1))
        with self.assertRaises(ValueError):self.pipeline.select(state,np.array([np.nan,0]))

class ProtocolTests(unittest.TestCase):
    def test_next_hitter_return_targets_and_mirrored_projection(self):
        from yichao_v3_v9.protocol import V3Protocol
        p=V3Protocol();r=record()
        r['robots']['198']['base_position'][1]=.98
        r['robots']['66']['base_position'][1]=-.98
        right=p.commit('198',r);left=p.commit('66',r)
        self.assertGreater(right['hitter_outward_y'],.98)
        self.assertAlmostEqual(right['hitter_outward_y'],-left['hitter_outward_y'],places=5)
        np.testing.assert_allclose(right['return_pair'],[-.1,.8],atol=1e-6)
        np.testing.assert_allclose(left['return_pair'],[-.8,.1],atol=1e-6)
        self.assertEqual(right['next_hitter'],'66')
        self.assertEqual(left['next_hitter'],'198')

    def test_runway_rejection_and_return_projection(self):
        from yichao_v3_v9.protocol import V3Protocol
        p=V3Protocol();r=record()
        r['robots']['198']['base_position'][1]=1.19
        with self.assertRaises(ValueError):p.commit('198',r)
        r['robots']['198']['base_position'][1]=.8
        r['robots']['66']['base_position'][1]=-.8
        projected=p.return_targets([-.9,.9],r)
        self.assertGreater(projected[0],-.8)
        self.assertLess(projected[1],.8)
        self.assertGreater(projected[1]-projected[0],.45)

class InputTests(unittest.TestCase):
    def test_reject_invalid_input_table(self):
        changes={
            'missing_q':lambda r:r['robots']['66'].pop('q'),
            'q_shape':lambda r:r['robots']['198'].update(q=[0]*28),
            'nan':lambda r:r['ball'].update(velocity=[np.nan,0,0]),
            'inf':lambda r:r['robots']['66'].update(dq=[float('inf')]*29),
            'quaternion':lambda r:r['robots']['66'].update(imu_quaternion=[0]*4),
            'phase':lambda r:r['robots']['66'].update(phase='IDLE'),
            'stale':lambda r:r['ball']['timestamp'].update(source=9.),
            'future':lambda r:r['ball']['timestamp'].update(source=10.1),
            'cross_clock':lambda r:r['ball']['timestamp'].update(domain='robot66_monotonic'),
            'missing_history':lambda r:r.pop('base_history_times'),
            'callback_history':lambda r:r.update(base_history_times=[9.90,9.96,9.97,9.99,10.]),
            'joint_order':lambda r:r['robots']['66'].update(joint_order=list(reversed(CONTRACT['fixture_joint_order']))),
            'mapping':lambda r:r['robots']['66'].update(role='table_left'),
            'emergency':lambda r:r.update(emergency_stop=True),
            'real_gate':lambda r:r.update(provenance='real'),
            'invalid_source':lambda r:r['robots']['66'].update(valid=False),
            'missing_racket_speed':lambda r:r['ball'].pop('racket_velocity'),
        }
        for name,change in changes.items():
            r=record();change(r)
            with self.subTest(name=name),self.assertRaises((ValueError,KeyError,TypeError)):features(r)
        RESULTS['invalid_input_cases']=list(changes)

    def test_clocks_and_source_preservation(self):
        adapter=InputAdapter(CONTRACT);adapter.build(record(),[-.9,.2])
        with self.assertRaises(ValueError):adapter.build(record(9.99),[-.9,.2])
        adapter=InputAdapter(CONTRACT);adapter.build(record(),[-.9,.2])
        r=record(10.02);r['ball']['timestamp']['source']=9.99
        with self.assertRaises(ValueError):adapter.build(r,[-.9,.2])
        r=record();r['ball']['timestamp']['source']=9.9
        f=features(r)
        self.assertAlmostEqual(f.safe[-1],.4,places=6)
        self.assertEqual(f.record['ball']['timestamp']['source'],9.9)

class ExecutorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from utils.data_utils import MotionCommand,MoveMotionBank
        cls.banks=MotionCommand(str(ROOT/'assets/0302_combined')),MoveMotionBank(str(ROOT/'assets/0718-move-160-80hz'))
    def setUp(self):self.e=OfflineExecutor('test',self.banks)

    def test_malformed_command_envelopes(self):
        for payload in (None,'bad',[None],[{}],[{'robot':'198'}]):
            self.assertTrue(all(a['status']=='rejected' for a in self.e.apply_batch(payload,10)))
        self.assertEqual(self.e.hit_count,{'66':0,'198':0})
        self.assertEqual(self.e.sequences,{'66':-1,'198':-1})

    def test_full_validation_before_hit_and_partial_pair_rejection(self):
        c=command(kind='hit');c['hit']['outward_y']=float('nan')
        self.assertEqual(self.e.apply_batch([c],10)[0]['status'],'rejected');self.assertEqual(self.e.hit_count['198'],0)
        c=command();bad=command(robot='66',target=-2)
        self.assertTrue(all(a['status']=='rejected' for a in self.e.apply_batch([c,bad],10)))
        self.assertEqual(self.e.schedulers['198'].state.name,'HOME_HOLD');self.assertEqual(self.e.sequences['198'],-1)

    def test_hit_dedup_stale_tokens_sessions(self):
        c=command(kind='hit');self.assertEqual(self.e.apply_batch([c],10)[0]['status'],'accepted')
        self.assertEqual(self.e.apply_batch([c],10.01)[0]['status'],'accepted');self.assertEqual(self.e.hit_count['198'],1)
        for mutate in (lambda c:c.update(sequence=2),lambda c:c.update(session='old'),lambda c:c.update(target_y=.8)):
            bad=copy.deepcopy(c);mutate(bad)
            self.assertEqual(self.e.apply_batch([bad],10.01)[0]['status'],'rejected')
        self.assertEqual(self.e.apply_batch([c],11.)[0]['status'],'rejected')
        restarted=OfflineExecutor('new',self.banks)
        self.assertEqual(restarted.apply_batch([c],10)[0]['status'],'rejected')

    def test_no_parallel_hits_protected_phase_and_motion_continues(self):
        self.e.apply_batch([command(kind='hit')],10)
        for c in (command(target=.3,seq=2),command(robot='66',kind='hit',target=-.3)):
            self.assertEqual(self.e.apply_batch([c],10.01)[0]['status'],'rejected')
        initial=self.e.schedulers['198'].hit_step
        self.e.tick(10.,self.e.positions)
        self.assertGreater(self.e.schedulers['198'].hit_step,initial)
        # No further commands: frozen scheduler still moves through POST_DELAY and OUTWARD.
        for i in range(1,45):self.e.tick(10+i*.02,self.e.positions)
        self.assertIn(self.e.schedulers['198'].state.name,('OUTWARD','OUTWARD_HOLD'))
        new=command(kind='hit',seq=3,now=11.)
        self.assertEqual(self.e.apply_batch([new],11.)[0]['status'],'rejected')
        self.assertEqual(self.e.hit_count['198'],1)
        RESULTS['invalid_input_does_not_stop_existing_motion']=True

    def test_move_small_reversal_same_phase_and_mirror(self):
        small=command(target=.20005)
        self.assertEqual(self.e.apply_batch([small],10)[0]['status'],'accepted')
        self.assertEqual(self.e.schedulers['198'].state.name,'HOME_HOLD')
        c=command(target=.5,seq=2);self.assertEqual(self.e.apply_batch([c],10)[0]['status'],'accepted')
        self.e.tick(10,self.e.positions)
        index=self.e.schedulers['198'].move_motion_index;step=self.e.schedulers['198'].move_step
        c=command(target=.8,seq=3);self.assertEqual(self.e.apply_batch([c],10.01)[0]['status'],'accepted')
        self.assertEqual(self.e.schedulers['198'].move_motion_index,index)
        self.assertEqual(self.e.schedulers['198'].move_step,step)
        reverse=command(target=-.1,seq=4)
        self.assertEqual(self.e.apply_batch([reverse],10.01)[0]['status'],'accepted')
        self.assertEqual(self.e.schedulers['198'].state.name,'RETURN')
        from utils.mirrored_reference_scheduler import MirroredReferenceScheduler
        from utils.doubles_reference_scheduler import DoublesReferenceScheduler
        from utils.left_right_mirror import mirror_joint_values
        from utils.joint_mapping import LAB_JOINT_NAMES
        a=DoublesReferenceScheduler(*self.banks,external_control=True);b=MirroredReferenceScheduler(DoublesReferenceScheduler(*self.banks,external_control=True))
        a.reset([0,.2,.75],[0,.2,.75]);b.reset([0,-.2,.75],[0,-.2,.75])
        a.set_external_base_target([0,.7]);b.set_external_base_target([0,-.7])
        np.testing.assert_allclose(a.output().target_base,-b.output().target_base,atol=1e-6)
        np.testing.assert_allclose(mirror_joint_values(a.output().joint_pos,LAB_JOINT_NAMES),b.output().joint_pos,atol=1e-6)
        self.assertEqual(a.move_motion_index,b.move_motion_index)
        self.assertNotIn(index,(41,47))
        RESULTS['scheduler_same_phase_target_change_restarts_reference']=False

    def test_return_requires_position_and_stability(self):
        c=command(robot='66',kind='return',target=-.2)
        ack=self.e.apply_batch([c],10)[0]
        self.assertEqual(ack['status'],'accepted');self.assertEqual(ack['phase'],'RETURN');self.assertFalse(ack['completed'])
        feedback=self.e.tick(10,self.e.positions)['66'];self.assertFalse(feedback['completed'])
        positions=copy.deepcopy(self.e.positions);positions['66'][1]=-.2
        completed=False
        for i in range(1,30):completed=self.e.tick(10+i*.02,positions)['66']['completed']
        self.assertTrue(completed)

    def test_relay_latch_partial_ack_and_no_phase_ack(self):
        relay=Relay('test',[-.9125,.2]);pipeline=Pipeline()
        f=features();commands=relay.step(f,pipeline)
        self.assertEqual(len(commands),2)
        acks=self.e.apply_batch(commands,10);self.assertTrue(all(a['status']=='accepted' for a in acks))
        previous=list(relay.previous_targets)
        wrong=copy.deepcopy(acks[0]);wrong['session']='foreign';relay.observe([wrong]);self.assertEqual(relay.previous_targets,previous)
        relay.observe(acks[:1]);self.assertEqual(relay.state,'PREPARING')
        self.assertEqual(len(relay.step(features(record(10.02)),pipeline)),1)
        relay.observe(acks[1:]);self.assertEqual(relay.state,'PREPARING')
        with patch.object(pipeline,'decide',side_effect=AssertionError('duplicate inference')):
            self.assertEqual(relay.step(features(record(10.04)),pipeline),[])
        rejected=copy.deepcopy(acks[0]);rejected.update(status='rejected',reason='test')
        relay.observe([rejected]);self.assertEqual(relay.state,'FAULT')
        self.assertEqual(relay.step(features(record(10.06,'newshot')),pipeline),[])

class LifecycleTests(unittest.TestCase):
    def test_file_replay_and_network_isolation(self):
        import tempfile,socket
        from yichao_v3_v9.offline import run
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)/'run.jsonl'
            result=run(ROOT/'fixtures/two_shots.jsonl',output)
            self.assertEqual(result['hit_count'],{'66':1,'198':1})
            self.assertEqual(result['final_state'],'COMPLETE')
            self.assertEqual(result['invalid'],0)
            with self.assertRaises(FileExistsError):run(ROOT/'fixtures/two_shots.jsonl',output)
            rows=[json.loads(line) for line in output.read_text().splitlines()]
            kinds=[c['kind'] for row in rows for c in row['commands']]
            self.assertEqual(kinds.count('hit'),2)
            self.assertEqual(kinds.count('clear'),2)
            self.assertEqual(kinds.count('return'),4)
            self.assertTrue(any(row['relay']['state']=='RETURNING' for row in rows))
            RESULTS['two_shot_replay']=result
            RESULTS['protocol_commands']={kind:kinds.count(kind) for kind in sorted(set(kinds))}
        with socket.socket() as sock:
            with self.assertRaisesRegex(RuntimeError,'offline isolation'):sock.connect(('127.0.0.1',11311))
        self.assertFalse(any(m in sys.modules for m in ('rospy','lcm','unitree_sdk2py')))

    def test_invalid_record_cannot_hit_or_fabricate_success(self):
        import tempfile
        from yichao_v3_v9.offline import run
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'input.jsonl';output=Path(tmp)/'out.jsonl'
            r=record();r['ball']['valid']=False
            source.write_text(json.dumps(r)+'\n'+json.dumps(record(10.02))+'\n')
            result=run(source,output)
            self.assertEqual(result['hit_count'],{'66':0,'198':0})
            self.assertEqual(result['final_state'],'FAULT')
            self.assertEqual(result['invalid'],1)
            self.assertTrue(all(not json.loads(line)['commands'] for line in output.read_text().splitlines()))

    def test_ack_nan_wrong_goal_and_stable_phase_only(self):
        for mutation in ('nan','wrong_goal','phase_only'):
            relay=Relay('test',[-.9125,.2]);pipeline=Pipeline();commands=relay.step(features(),pipeline)
            c=commands[0]
            ack={k:c[k] for k in ('session','robot','sequence','shot','token','kind')}
            ack.update(evidence='simulated_scheduler',status='accepted',goal_y=c['target_y'],applied_target_y=c['target_y'],phase='HOME_HOLD',completed=True)
            if mutation=='nan':ack['applied_target_y']=float('nan')
            if mutation=='wrong_goal':ack['goal_y']+=.2
            relay.observe([ack])
            if mutation=='phase_only':self.assertEqual(relay.state,'PREPARING');self.assertFalse(relay.completed)
            else:self.assertEqual(relay.state,'FAULT')

    def test_decision_deadline_and_ack_timeout(self):
        relay=Relay('test',[-.9125,.2]);pipeline=Pipeline()
        original=pipeline.decide
        def delayed(f):
            time.sleep(.025)
            return original(f)
        with patch.object(pipeline,'decide',side_effect=delayed):commands=relay.step(features(),pipeline)
        self.assertEqual(commands,[]);self.assertEqual(relay.fault,'decision_deadline_exceeded')
        relay=Relay('test',[-.9125,.2]);relay.step(features(),pipeline)
        self.assertEqual(relay.step(features(record(10.3)),pipeline),[])
        self.assertEqual(relay.fault,'prepare_ack_timeout')

if __name__=='__main__':unittest.main()
