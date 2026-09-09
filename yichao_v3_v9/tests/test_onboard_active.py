"""No hardware/bus tests of active ownership and command stage admission."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import time
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np
from yichao_v3_v9 import ROOT
from yichao_v3_v9.executor import OfflineExecutor, clone_scheduler
from yichao_v3_v9 import onboard_movement
import utils.planner_ros_bridge as stock_bridge
from common import command

spec = importlib.util.spec_from_file_location('onboard_active_entry', ROOT/'tools/run_onboard_active.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


class InitialInputWaitTests(unittest.TestCase):
    def install(self):
        class Agent:
            def __init__(self):
                self._torso_receive_monotonic_ns = 0
                self.se = types.SimpleNamespace(last_body_receive_monotonic_ns=0, spin=Mock())
        module = types.SimpleNamespace(LCMAgent=Agent)
        with patch.dict('sys.modules', {'envs': types.SimpleNamespace(lcm_agent=module),
                                       'envs.lcm_agent': module}):
            stage=entry.ControlStage();ros=types.SimpleNamespace(is_shutdown=Mock(return_value=False))
            entry.install_initial_input_wait(stage,ros)
        return Agent(),stage,ros

    def test_late_initial_torso_resumes_without_deadline_or_motor_action(self):
        agent,stage,ros=self.install();ticks=[]
        def delay(seconds):
            ticks.append(seconds)
            agent.se.last_body_receive_monotonic_ns=1
            if len(ticks)==240:agent._torso_receive_monotonic_ns=2
        with patch.object(entry.time,'sleep',side_effect=delay):agent.se.spin()
        self.assertGreater(sum(ticks),10.)
        self.assertEqual(stage(),'starting')
        self.assertEqual(stage.reason,'initial_inputs_received')

    def test_initial_wait_can_be_interrupted(self):
        agent,stage,ros=self.install();ros.is_shutdown.return_value=True
        with self.assertRaises(KeyboardInterrupt):agent.se.spin()


class String:
    def __init__(self, data):
        self.data = data


def ros_stub():
    return types.SimpleNamespace(
        Publisher=Mock(side_effect=lambda *a, **kw: Mock()), Subscriber=Mock(),
        core=types.SimpleNamespace(is_initialized=lambda: True),
        resolve_name=lambda name: name.replace('/doubles/', '/yichao_v3_v9/test/'),
        get_node_uri=lambda: 'test', is_shutdown=Mock(return_value=False))


class ActiveBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.baseline = OfflineExecutor('test')

    def setUp(self):
        self.ros = ros_stub()
        self.patches = [patch.dict('sys.modules', {
            'rospy': self.ros, 'std_msgs': types.SimpleNamespace(),
            'std_msgs.msg': types.SimpleNamespace(String=String)}),
            patch.object(stock_bridge, 'rospy', self.ros),
            patch.object(stock_bridge, 'String', String)]
        for p in self.patches:
            p.start()
        self.addCleanup(lambda: [p.stop() for p in reversed(self.patches)])
        self.stage = entry.ControlStage()
        self.bridge = onboard_movement.bridge_class(
            'test', shadow=False, protocol='relay', active_stage=self.stage)('table_left')
        self.scheduler = clone_scheduler(self.baseline.schedulers['198'])
        self.bridge.clock = Mock()
        self.bridge.clock.workstation_interval.return_value = [10., 10.001]
        self.bridge.sensors_recent = True
        self.bridge._agent = Mock()
        self.bridge._current_sensor_ages = Mock(return_value={'body': .01, 'imu': .01, 'torso': .01})

    def receive(self):
        self.bridge._command_callback(String(json.dumps(command(target=.2))))

    def apply(self):
        self.bridge.apply_pending(self.scheduler, self.baseline.positions['198'])

    def test_no_command_applies_before_actual_policy_stage(self):
        for stage in ('starting', 'waiting_r2', 'calibrating', 'waiting_policy_r2'):
            self.stage.set(stage)
            self.receive()
            self.apply()
            self.assertEqual(self.bridge.feedback['status'], 'rejected')
            self.assertIsNone(self.bridge._latest_command)
            self.assertEqual(self.scheduler.state.name, 'HOME_HOLD')
        self.stage.set('policy')
        self.receive()
        self.apply()
        self.assertEqual(self.bridge.feedback['status'], 'accepted')
        self.assertEqual(self.bridge.feedback['execution_mode'], 'active')

    def test_queued_command_cannot_survive_recalibration_or_resume(self):
        self.stage.set('policy')
        self.receive()
        self.stage.set('calibrating')
        self.apply()
        self.assertTrue(self.bridge._control_session_invalidated)
        self.assertEqual(self.bridge.feedback['status'], 'rejected')
        self.stage.set('policy')
        self.receive()
        self.apply()
        self.assertEqual(self.bridge.feedback['reason'], 'control_recalibrated_restart_session_required')
        self.assertEqual(self.scheduler.state.name, 'HOME_HOLD')

    def test_recalibration_before_first_command_does_not_latch_invalid_session(self):
        self.stage.set('policy')
        self.assertIsNone(self.bridge._admission_reason())
        # The receiver may already exist even though nothing has been applied.
        self.apply()
        self.assertEqual(self.bridge.receiver.sequence,-1)
        self.stage.set('calibrating')
        self.assertEqual(self.bridge._admission_reason(),'control_stage_not_policy:calibrating')
        self.stage.set('waiting_policy_r2')
        self.bridge._admission_reason()
        self.assertFalse(self.bridge._control_session_invalidated)
        self.stage.set('policy')
        self.receive()
        self.apply()
        self.assertEqual(self.bridge.feedback['status'],'accepted')

    def test_freshness_is_recomputed_at_application_time(self):
        self.stage.set('policy')
        self.receive()
        self.bridge._current_sensor_ages.return_value = {'body': .251, 'imu': .01, 'torso': .01}
        self.apply()
        self.assertEqual(self.bridge.feedback['reason'], 'sensor_freshness_required')
        self.assertEqual(self.scheduler.state.name, 'HOME_HOLD')

    def test_short_input_wait_does_not_invalidate_calibration_or_resume_queued_command(self):
        self.stage.set('policy')
        self.receive()
        generation=self.stage.calibration_generation
        self.stage.set('waiting_inputs')
        self.apply()
        self.assertIsNone(self.bridge._latest_command)
        self.assertFalse(self.bridge._control_session_invalidated)
        self.stage.set('policy')
        self.assertEqual(self.stage.calibration_generation,generation)
        self.receive()
        self.apply()
        self.assertEqual(self.bridge.feedback['status'],'accepted')

    def test_shadow_default_keeps_existing_command_semantics(self):
        bridge = onboard_movement.bridge_class('test', shadow=True, protocol='relay')('table_left')
        bridge.clock = self.bridge.clock
        bridge.sensors_recent = True
        bridge._command_callback(String(json.dumps(command(target=.2))))
        bridge.apply_pending(self.scheduler, self.baseline.positions['198'])
        self.assertEqual(bridge.feedback['status'], 'accepted')
        self.assertEqual(bridge.feedback['execution_mode'], 'shadow')

    def test_active_cannot_omit_its_stage_guard(self):
        with self.assertRaisesRegex(ValueError, 'requires_control_stage'):
            onboard_movement.bridge_class('test', shadow=False, protocol='relay')


class ActiveEntryTests(unittest.TestCase):
    def test_manifest_rejects_wrong_robot_and_changed_binary(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root/'config').mkdir()
            (root/'control/bin').mkdir(parents=True)
            (root/'control/lib').mkdir()
            (root/'control/bin/g1_control').write_bytes(b'private-binary')
            (root/'control/bin/g1_control').chmod(0o755)
            (root/'control/lib/lcm.so').write_bytes(b'private-lib')
            hashes = {str(p.relative_to(root)): entry.hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in (root/'control').rglob('*') if p.is_file()}
            manifest = {'schema': 'yichao-active-control-assets-v1', 'robot': '66',
                'executable': 'control/bin/g1_control', 'library_dir': 'control/lib',
                'lcm_default_url': 'udpm://239.255.78.66:7867?ttl=0', 'sha256': hashes}
            (root/'config/active_control.json').write_text(json.dumps(manifest))
            self.assertEqual(entry.verify_control_manifest(root, '66'), manifest)
            with self.assertRaisesRegex(RuntimeError, 'identity'):
                entry.verify_control_manifest(root, '198')
            (root/'control/bin/g1_control').write_bytes(b'changed')
            with self.assertRaisesRegex(RuntimeError, 'asset_changed'):
                entry.verify_control_manifest(root, '66')

    def test_only_exact_own_native_process_is_allowed(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            binary = root/'control/bin/g1_control'
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b'not executed')
            proc = root/'proc'
            own = proc/'901000'
            own.mkdir(parents=True)
            (own/'exe').symlink_to(binary)
            (own/'cmdline').write_bytes(str(binary).encode()+b'\0eth0\0')
            (own/'stat').write_text('901000 (g1_control) S '+'0 '*18+'123 0\n')
            (own/'environ').write_bytes(b'LCM_DEFAULT_URL=udpm://239.255.78.66:7867?ttl=0\0'
                +b'LD_LIBRARY_PATH='+str(root/'control/lib').encode()+b'\0')
            self.assertEqual(entry.verify_control_process(root, '66', 901000, proc_root=proc), '123')
            other = proc/'901001'
            other.mkdir()
            (other/'cmdline').write_bytes(b'python\0/some/other/run_onboard_active.py\0')
            with self.assertRaisesRegex(RuntimeError, 'other_controller'):
                entry.verify_control_process(root, '66', 901000, proc_root=proc)

    def _runner(self):
        ros = ros_stub()
        self.addCleanup(patch.stopall)
        patch.dict('sys.modules', {'rospy': ros}).start()
        import utils.deployment_runner as stock_runner
        patch.object(stock_runner, 'rospy', ros).start()
        native_publish = Mock()
        fake_agent = types.SimpleNamespace(LCMAgent=types.SimpleNamespace(publish_action=native_publish))
        patch.dict('sys.modules', {'envs': types.SimpleNamespace(lcm_agent=fake_agent),
                                  'envs.lcm_agent': fake_agent}).start()
        patch.object(stock_runner, 'DeploymentRunner', stock_runner.DeploymentRunner).start()
        stage = entry.ControlStage()
        cls = entry.install_active_runner(stage, ros, Mock(from_numpy=lambda a: a))
        runner = cls()
        return runner, stage, ros, fake_agent, native_publish

    def test_shutdown_calibration_is_a_noop(self):
        runner, stage, ros, _, _ = self._runner()
        ros.is_shutdown.return_value = True
        self.assertIsNone(runner.calibrate(wait=False))
        self.assertEqual(stage(), 'starting')

    def test_reference_option_preserves_original_default_and_left_robot(self):
        self.assertEqual(entry.startup_options('66'),
                         ['--mirror-left-hand', '--bootstrap-outward-hold'])
        self.assertEqual(entry.startup_options('66', 'home-hold'),
                         ['--mirror-left-hand', '--startup-home-current'])
        for mode in ('outward-hold', 'home-hold'):
            self.assertEqual(entry.startup_options('198', mode), ['--startup-home-current'])
        with self.assertRaises(ValueError):entry.startup_options('66', 'static-joints')

    def test_home_wait_runs_every_policy_tick_and_latches_world_target(self):
        # Exercise the native runner, rather than testing a replacement wait loop.
        # Prescribed observations are not a physical balance simulation.
        from yichao_v3_v9.command_receiver import CommandReceiver
        runner, stage, ros, _, _ = self._runner()
        import utils.deployment_runner as stock
        scheduler = clone_scheduler(OfflineExecutor('test').schedulers['66'])
        initial = np.array([.1, -.2, .75], np.float32)
        scheduler.reset(initial, initial)  # native --startup-home-current
        receiver = CommandReceiver('test', '66', scheduler, allow_prepare_preemption=True)
        expired = command('66', 'move', -.7, now=9.)
        self.assertEqual(receiver.apply(expired, workstation_time_interval=[10., 10.001],
            position=initial, sensors_recent=True)['status'], 'rejected')
        agent = Mock();agent.from_lab_to_gym = list(range(29))
        runner.agents = {'robot': agent};runner.control_agent_name = 'robot'
        runner.command_profile = types.SimpleNamespace(state_estimator=types.SimpleNamespace(
            right_lower_right_switch_pressed=False))
        ticks=[];references=[];positions=[]
        def observation():
            p=initial.copy();p[1]+=.03*np.sin(len(ticks)*.1)
            ref=scheduler.update(.3, np.zeros(3), np.zeros(3), p, p,
                                 np.zeros(3), np.zeros(3), dt=.02)
            references.append(ref);positions.append(p)
            return {'obs':p.copy(), 'obs_history':p.copy()}
        agent.reset.side_effect=observation;agent.observe.side_effect=observation
        def calibrate(wait=True):
            if ros.is_shutdown():return None
            stage.set('waiting_policy_r2')
            return agent.reset()
        runner.calibrate=calibrate
        runner._publish_exit_stage=Mock()
        policy=Mock(side_effect=lambda obs:np.full((1,29),obs[1],np.float32))
        runner.add_policy(policy)
        def tick():
            ticks.append(.02)
            if len(ticks)==600:ros.is_shutdown.return_value=True
        with patch.object(stock.os,'makedirs'), \
                patch.object(stock.FixedRate,'sleep',side_effect=tick):
            runner.run()
        self.assertEqual(policy.call_count,600)
        self.assertEqual(agent.apply_action.call_count,600)
        self.assertGreater(np.ptp([c.args[0][0,0] for c in agent.apply_action.call_args_list]),.05)
        self.assertFalse(receiver.prepared)
        for ref,p in zip(references,positions):
            self.assertEqual(ref.state.name,'HOME_HOLD')
            np.testing.assert_allclose(p[:2]+ref.target_base,initial[:2],atol=1e-6)
        # A valid first MOVE can still request a real lateral displacement.
        move=command('66','move',-.65,now=10.)
        ack=receiver.apply(move,workstation_time_interval=[10.,10.001],
                           position=positions[-1],sensors_recent=True)
        self.assertEqual(ack['status'],'accepted',ack)
        self.assertAlmostEqual(ack['applied_target_y'],-.65)
        hit=command('66','hit',-.65,seq=2,now=10.05)
        self.assertEqual(receiver.apply(hit,workstation_time_interval=[10.05,10.051],
            position=positions[-1],sensors_recent=True)['status'],'accepted')
        self.assertEqual(scheduler.state.name,'HIT')

    def test_two_r2_calibration_publishes_waiting_states_before_policy(self):
        runner, stage, ros, _, _ = self._runner()
        agent = Mock()
        goal = np.array([-.312, 0., 0., .669, -.363, 0.,
                         -.312, 0., 0., .669, -.363, 0.,
                         0., 0., 0., -.2, .2, 0., -.2, 0., 0., 0.,
                         -.2, -.2, 0., -.2, 0., 0., 0.])
        agent.dof_pos = goal+.02
        agent.action_scale = np.ones(29)
        agent.num_envs, agent.num_dofs = 1, 29
        agent.reset.return_value = 'fresh reset observation'
        runner.agents = {'robot': agent}
        runner.control_agent_name = 'robot'
        state = types.SimpleNamespace(right_lower_right_switch_pressed=False)
        runner.command_profile = types.SimpleNamespace(state_estimator=state)
        seen = []

        def observe():
            seen.append(stage())
            # Emulate separate operator R2 edges during each wait.
            if stage() in ('waiting_r2', 'waiting_policy_r2'):
                state.right_lower_right_switch_pressed = True

        agent.planner_bridge.publish_state.side_effect = lambda a: observe()
        with patch.object(entry.time, 'sleep'), \
                patch.object(entry, 'source_ages', return_value={'body': .01, 'imu': .01, 'torso': .01}):
            result = runner.calibrate()
        self.assertEqual(result, 'fresh reset observation')
        self.assertEqual(seen, ['waiting_r2', 'calibrating', 'waiting_policy_r2'])
        self.assertGreater(agent.step.call_count, 0)
        self.assertEqual(stage(), 'waiting_policy_r2')
        self.assertFalse(state.right_lower_right_switch_pressed)

    def test_check_never_creates_control_or_bus_publishers(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            ros = ros_stub()
            ros.init_node = Mock(side_effect=AssertionError('check opened ROS node'))
            lcm = types.SimpleNamespace(LCM=Mock(side_effect=AssertionError('check opened LCM')))
            torch = types.SimpleNamespace(set_num_threads=Mock(), set_num_interop_threads=Mock(),
                                          cuda=types.SimpleNamespace(is_available=lambda: True))
            ort = types.SimpleNamespace(InferenceSession=Mock(), SessionOptions=Mock())
            load = Mock()
            entrypoints = {'load_onnx_policy': load, '_validate_motion_manifest': Mock(),
                           'EXPECTED_HIT_MANIFEST_SHA256': 'hit',
                           'EXPECTED_MOVE_MANIFEST_SHA256': 'move'}
            with patch.object(entry, 'ROOT', root), patch.object(entry.sys, 'prefix', str(root/'.venv')), \
                    patch.object(entry.sys, 'argv', ['entry', '--session', 'test', '--robot', '66', '--check']), \
                    patch.dict('sys.modules', {'rospy': ros, 'lcm': lcm, 'torch': torch, 'onnxruntime': ort}), \
                    patch.dict(os.environ), \
                    patch('yichao_v3_v9.onboard_isolation.verify_release'), \
                    patch.object(entry, 'verify_control_manifest', return_value={}), \
                    patch.object(entry.runpy, 'run_path', return_value=entrypoints):
                entry.main()
            load.assert_called_once()
            lcm.LCM.assert_not_called()
            ros.init_node.assert_not_called()

    def test_policy_stage_changes_only_after_successful_inference(self):
        runner, stage, _, _, _ = self._runner()
        stage.set('waiting_policy_r2')
        policy = Mock(side_effect=RuntimeError('bad inference'))
        runner.add_policy(policy)
        with self.assertRaises(RuntimeError):
            runner.policy(None)
        self.assertEqual(stage(), 'waiting_policy_r2')
        policy.side_effect = None
        policy.return_value = [0]*29
        self.assertEqual(runner.policy(None), [0]*29)
        self.assertEqual(stage(), 'policy')

    def test_stale_sources_or_inactive_stage_cannot_publish_q_des(self):
        _, stage, _, fake_agent, native_publish = self._runner()
        agent = Mock()
        agent.actions.reshape.return_value.detach.return_value.cpu.return_value.numpy.return_value = np.zeros(29)
        agent.action_scale = np.ones(29)
        agent.default_dof_pos = np.zeros(29)
        with patch.object(entry, 'source_ages', return_value={'body': .01, 'imu': .01, 'torso': .01}) as ages:
            with self.assertRaisesRegex(RuntimeError, 'outside'):
                fake_agent.LCMAgent.publish_action(agent)
            stage.set('policy')
            fake_agent.LCMAgent.publish_action(agent)
            native_publish.assert_called_once()
            ages.side_effect = [{'body': .01, 'imu': .01, 'torso': .251},
                                {'body': .01, 'imu': .01, 'torso': .01}]
            with patch.object(entry.time, 'sleep'):
                fake_agent.LCMAgent.publish_action(agent)
            native_publish.assert_called_once()
            self.assertEqual(stage(), 'policy')
            agent.planner_bridge.publish_state.assert_called_once_with(agent)
            self.assertEqual(stage.generation, 3)

    def test_waiting_for_sources_preserves_operator_shutdown(self):
        _, stage, ros, fake_agent, native_publish = self._runner()
        stage.set('calibrating')
        ros.is_shutdown.side_effect = [False, True]
        with patch.object(entry, 'source_ages', return_value={'torso': 2.}):
            with self.assertRaises(KeyboardInterrupt):
                fake_agent.LCMAgent.publish_action(Mock())
        native_publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
