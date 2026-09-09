"""Operator-controlled experimental V9 active entry, in the isolated release.

The native controller can move before R2 and holds its last command on Python
exit. Exiting this program is NOT a physical stop. Disarm at the robot first.
--check only verifies files, imports and CPU inference ABI; it opens no bus.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import runpy
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


class ControlStage:
    def __init__(self):
        self.value = 'starting'
        self.generation = 0
        self.calibration_generation = 0
        self.reason = None

    def __call__(self):
        return self.value

    def set(self, value, reason=None):
        if value != self.value:
            self.generation += 1
            if value == 'calibrating':
                self.calibration_generation += 1
        self.value, self.reason = value, reason
        print('YICHAO_CONTROL_STAGE '+json.dumps({
            'control_stage': value, 'generation': self.generation,
            'calibration_generation': self.calibration_generation,
            'reason': reason, 'experimental_active': True,
            'real_input_accepted': False}), flush=True)


def verify_control_manifest(root, robot):
    root = root.resolve()
    manifest = json.loads((root/'config/active_control.json').read_text())
    binary = 'control/bin/g1_control'
    bus = 'udpm://239.255.78.'+robot+':7867?ttl=0'
    if (manifest.get('schema') != 'yichao-active-control-assets-v1'
            or manifest.get('robot') != robot
            or manifest.get('executable') != binary
            or manifest.get('library_dir') != 'control/lib'
            or manifest.get('lcm_default_url') != bus):
        raise RuntimeError('active_control_identity_or_bus_mismatch')
    hashes = manifest.get('sha256')
    if not isinstance(hashes, dict) or binary not in hashes or len(hashes) < 2:
        raise RuntimeError('active_control_hashes_required')
    for relative, expected in hashes.items():
        path = root/relative
        path.resolve().relative_to(root)
        if not relative.startswith('control/') or not path.is_file():
            raise RuntimeError('active_control_path:'+relative)
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('active_control_asset_changed:'+relative)
    if not os.access(str(root/binary), os.X_OK):
        raise RuntimeError('active_control_not_executable')
    return manifest


def verify_control_process(root, robot, pid, *, proc_root=Path('/proc')):
    """Allow only the exact binary PID already started by our own launcher."""
    expected = (root/'control/bin/g1_control').resolve()
    if not isinstance(pid, int) or pid <= 0:
        raise RuntimeError('own_g1_pid_required')
    process = proc_root/str(pid)
    try:
        if (process/'exe').resolve(strict=True) != expected:
            raise RuntimeError('g1_pid_executable_mismatch')
        environ = dict(part.split(b'=', 1) for part in
                       (process/'environ').read_bytes().split(b'\0') if b'=' in part)
        bus = ('udpm://239.255.78.'+robot+':7867?ttl=0').encode()
        if environ.get(b'LCM_DEFAULT_URL') != bus:
            raise RuntimeError('g1_pid_lcm_bus_mismatch')
        libs = environ.get(b'LD_LIBRARY_PATH', b'').split(b':')
        if not libs or libs[0] != str(root/'control/lib').encode():
            raise RuntimeError('g1_pid_private_libraries_required')
        stat = (process/'stat').read_text().rsplit(')', 1)[1].split()
        if stat[0] in ('Z', 'T', 't', 'X'):
            raise RuntimeError('g1_pid_not_running')
    except OSError as exc:
        raise RuntimeError('g1_pid_unavailable') from exc
    markers = {'g1_control', 'deploy_policy.py', 'run_onboard_movement.py',
               'run_onboard_relay.py', 'run_onboard_active.py', 'run_active_control.py',
               'passive_v9_telemetry'}
    conflicts = []
    for path in proc_root.glob('[0-9]*/cmdline'):
        if int(path.parent.name) in (pid, os.getpid()):
            continue
        try:
            args = path.read_bytes().decode().split('\0')
        except (OSError, UnicodeError):
            continue
        if any(Path(arg).name in markers for arg in args if arg):
            conflicts.append(int(path.parent.name))
    if conflicts:
        raise RuntimeError('other_controller_or_policy:'+str(sorted(conflicts)))
    return stat[19]


def source_ages(agent):
    raw = time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW)
    return {'body': (raw-agent.se.body_source_monotonic_raw_ns)/1e9,
            'imu': (raw-agent.se.imu_source_monotonic_raw_ns)/1e9,
            'torso': time.monotonic()-agent._torso_receive_monotonic_ns/1e9}


def install_initial_input_wait(stage, rospy):
    """Wait for actual initial samples before the frozen startup deadline begins."""
    import envs.lcm_agent as agent_module
    original_init = agent_module.LCMAgent.__init__

    def initialize(agent, *args, **kwargs):
        original_init(agent, *args, **kwargs)
        original_spin = agent.se.spin

        def spin():
            original_spin()
            waiting = False
            while (agent._torso_receive_monotonic_ns <= 0
                   or agent.se.last_body_receive_monotonic_ns <= 0):
                if rospy.is_shutdown():
                    raise KeyboardInterrupt()
                if not waiting:
                    stage.set('waiting_initial_inputs', 'waiting_for_initial_torso_and_joint_state')
                    waiting = True
                time.sleep(.05)
            if waiting:
                stage.set('starting', 'initial_inputs_received')

        agent.se.spin = spin

    agent_module.LCMAgent.__init__ = initialize


def startup_options(robot, right_startup='outward-hold'):
    """Select only the native reference; both choices run the full V9 loop."""
    if robot not in ('66', '198') or right_startup not in ('outward-hold', 'home-hold'):
        raise ValueError('invalid_startup_reference')
    if robot == '198':
        return ['--startup-home-current']
    return ['--mirror-left-hand',
            '--startup-home-current' if right_startup == 'home-hold' else '--bootstrap-outward-hold']


def install_active_runner(stage, rospy, torch):
    """Keep the frozen loop/recorder; override only calibration and admission."""
    import numpy as np
    import utils.deployment_runner as stock
    import envs.lcm_agent as agent_module
    original_publish = agent_module.LCMAgent.publish_action

    def wait_sources(agent):
        previous = stage()
        paused = False
        while not all(0 <= age <= .25 for age in source_ages(agent).values()):
            if rospy.is_shutdown():
                raise KeyboardInterrupt()
            if not paused:
                stage.set('waiting_inputs', 'waiting_for_fresh_body_imu_and_torso')
                paused = True
            # Publish actual freshness while retaining the native controller.
            # A pause after POLICY invalidates the former Planner transaction.
            agent._update_robot_state(require_fresh=False)
            if agent.planner_bridge is not None:
                agent.planner_bridge.publish_state(agent)
            time.sleep(.02)
        if paused:
            stage.set(previous, 'fresh_inputs_restored')
        return paused

    def guarded_publish(agent, hard_reset=False):
        if rospy.is_shutdown():
            raise KeyboardInterrupt()
        if stage() not in ('calibrating', 'policy'):
            raise RuntimeError('q_des_outside_calibration_or_policy')
        if wait_sources(agent):
            # Discard the action computed before the gap. The next loop reads
            # current observations and computes a new action before publishing.
            return
        target = agent.actions.reshape(-1).detach().cpu().numpy()*agent.action_scale+agent.default_dof_pos
        if target.shape != (29,) or not np.isfinite(target).all():
            raise RuntimeError('nonfinite_active_q_des')
        return original_publish(agent, hard_reset=hard_reset)

    agent_module.LCMAgent.publish_action = guarded_publish
    original_runner = stock.DeploymentRunner

    class ActiveRunner(original_runner):
        def add_policy(self, policy):
            class WithStage:
                def __call__(self, observation):
                    result = policy(observation)
                    if stage() != 'policy':
                        stage.set('policy')
                    return result

                def __getattr__(self, name):
                    return getattr(policy, name)
            super().add_policy(WithStage())

        def _heartbeat(self, agent):
            # Read actual telemetry and publish it without advancing scheduler,
            # observation history or synthesizing a physical completion.
            agent._update_robot_state(require_fresh=True)
            if agent.planner_bridge is not None:
                agent.planner_bridge.publish_state(agent)

        def _wait_r2(self, agent, value, message):
            stage.set(value)
            print(message, flush=True)
            while not rospy.is_shutdown():
                self._heartbeat(agent)
                if wait_sources(agent):
                    self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                    print(message, flush=True)
                    continue
                if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                    self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                    return
                time.sleep(.02)
            raise KeyboardInterrupt()

        def calibrate(self, wait=True):
            # The frozen runner invokes calibrate after normal loop shutdown.
            # Suppress that call: exiting software must not initiate motion.
            if rospy.is_shutdown():
                return None
            agent = self.agents[self.control_agent_name]
            if wait:
                self._wait_r2(agent, 'waiting_r2',
                    'About to calibrate [Press R2 to move to the V9 default pose]')
            stage.set('calibrating')
            self._heartbeat(agent)  # invalidates all pre-reset pending receipts
            agent.get_obs()
            # Exact nominal joint values and 0.01 rad / 50 ms increments from
            # the matching frozen V9 DeploymentRunner.calibrate implementation.
            final_goal = np.array([
                -.312, 0., 0., .669, -.363, 0.,
                -.312, 0., 0., .669, -.363, 0.,
                0., 0., 0., -.200, .200, 0., -.200, 0., 0., 0.,
                -.200, -.200, 0., -.200, 0., 0., 0.], dtype=float)
            target = np.asarray(agent.dof_pos, dtype=float).copy()
            if target.shape != (29,) or not np.isfinite(target).all():
                raise RuntimeError('invalid_calibration_joint_position')
            while np.max(np.abs(target-final_goal)) > .01:
                if rospy.is_shutdown():
                    raise KeyboardInterrupt()
                target -= np.clip(target-final_goal, -.01, .01)
                action = np.zeros((agent.num_envs, agent.num_dofs))
                action[:, :] = (target-final_goal)/agent.action_scale
                agent.step(torch.from_numpy(action))
                agent.get_obs()
                time.sleep(.05)
            self._wait_r2(agent, 'waiting_policy_r2',
                'Starting pose calibrated [Press R2 again to start V9 policy]')
            # Stock resets history and scheduler after the second R2.
            result = None
            for name, candidate in self.agents.items():
                obs = candidate.reset()
                if name == self.control_agent_name:
                    result = obs
            return result

        def run(self, *args, **kwargs):
            try:
                return super().run(*args, **kwargs)
            except KeyboardInterrupt:
                stage.set('stopped', 'software_exit_is_not_physical_stop')
            except BaseException as exc:
                stage.set('fault', type(exc).__name__+':'+str(exc))
                self._publish_exit_stage()
                raise
            finally:
                if stage() != 'fault':
                    stage.set('stopped', 'software_exit_is_not_physical_stop')
                    self._publish_exit_stage()

        def _publish_exit_stage(self):
            # Best effort evidence only; a stale or disconnected ROS channel is
            # never taken as proof that the native controller has stopped.
            for agent in self.agents.values():
                try:
                    if agent.planner_bridge is not None:
                        agent.planner_bridge.publish_state(agent)
                except Exception:
                    pass

    stock.DeploymentRunner = ActiveRunner
    return ActiveRunner


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True)
    parser.add_argument('--robot', required=True, choices=('66', '198'))
    parser.add_argument('--deploy-root', type=Path, default=ROOT/'vendor/deploy')
    parser.add_argument('--torso-topic')
    parser.add_argument('--record-dir', type=Path)
    parser.add_argument('--g1-pid', type=int)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--right-startup', choices=('outward-hold', 'home-hold'),
                        default='outward-hold',
                        help='66 reference after R2; home-hold is an unvalidated dynamic idle candidate')
    args = parser.parse_args()
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', args.session):
        parser.error('session must be a safe private ROS name')
    deploy = args.deploy_root.resolve()
    if deploy != (ROOT/'vendor/deploy').resolve():
        parser.error('deploy-root must be this release frozen vendor/deploy')
    if Path(sys.prefix).resolve() != (ROOT/'.venv').resolve():
        parser.error('run with this release .venv/bin/python')
    role = 'table_right' if args.robot == '66' else 'table_left'
    prefix = '/yichao_v3_v9/'+args.session+'/'+role
    torso_topic = args.torso_topic or prefix+'/torso_pose_origin'
    if torso_topic != prefix+'/torso_pose_origin':
        parser.error('torso topic must be private to this robot and session')
    sys.path[:0] = [str(ROOT/'src'), str(deploy/'g1_gym_deploy'), str(ROOT/'vendor/ros1')]
    os.environ.update(ROS_MASTER_URI='http://172.16.3.126:11311', ROS_IP='172.16.4.'+args.robot,
                      OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
                      CUDA_VISIBLE_DEVICES='')
    os.environ.pop('ROS_HOSTNAME', None)
    from yichao_v3_v9.onboard_isolation import verify_release
    verify_release(ROOT)
    control = verify_control_manifest(ROOT, args.robot)
    import rospy
    import lcm
    import torch
    import onnxruntime as ort
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    # Both physical roles use CPU tensors, even on the Jetson.
    torch.cuda.is_available = lambda: False
    session_factory = ort.InferenceSession

    def cpu_session(*positional, **kwargs):
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        kwargs.update(sess_options=options, providers=['CPUExecutionProvider'])
        return session_factory(*positional, **kwargs)

    ort.InferenceSession = cpu_session
    policy_path = ROOT/'assets/robot_66/student_v9_m14500_timedhandoff_1666_model19000.onnx'
    if args.check:
        # Loading the deploy module does not run its __main__ or open LCM.
        spec = runpy.run_path(str(deploy/'g1_gym_deploy/scripts/deploy_policy.py'),
                              run_name='yichao_active_check')
        spec['load_onnx_policy'](str(policy_path), device='cpu')
        spec['_validate_motion_manifest'](ROOT/'assets/0302_combined',
            spec['EXPECTED_HIT_MANIFEST_SHA256'], 'hit')
        spec['_validate_motion_manifest'](ROOT/'assets/0718-move-160-80hz',
            spec['EXPECTED_MOVE_MANIFEST_SHA256'], 'move')
        print(json.dumps({'check': 'passed', 'robot': args.robot, 'execution_mode': 'active',
            'experimental_active': True, 'input_contract_accepted': False,
            'real_input_accepted': False, 'device_or_bus_opened': False}), flush=True)
        return
    if args.g1_pid is None or args.record_dir is None:
        parser.error('active requires --g1-pid and --record-dir')
    record = args.record_dir.resolve()
    record.relative_to((ROOT/'output').resolve())
    verify_control_process(ROOT, args.robot, args.g1_pid)
    lock = (ROOT/'onboard_shadow.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('another onboard entry owns this release')
    lcm_factory = lcm.LCM

    def private_lcm(url):
        if url != control['lcm_default_url']:
            raise RuntimeError('active_lcm_must_use_private_robot_group')
        return lcm_factory(url)

    lcm.LCM = private_lcm
    remaps = [sys.argv[0]]+['/doubles/'+role+'/'+name+':='+prefix+'/'+name
                           for name in ('state', 'command')]
    rospy.init_node('yichao_active_'+args.session+'_'+args.robot, argv=remaps,
                    disable_rosout=True, disable_rostime=True)
    stage = ControlStage()
    from yichao_v3_v9.onboard_movement import bridge_class
    import utils.planner_ros_bridge as stock_bridge
    stock_bridge.PlannerRosBridge = bridge_class(args.session, shadow=False,
                                               protocol='relay', active_stage=stage)
    install_initial_input_wait(stage, rospy)
    install_active_runner(stage, rospy, torch)
    record.mkdir(parents=True, exist_ok=False)
    entry = deploy/'g1_gym_deploy/scripts/deploy_policy.py'
    sys.argv = [str(entry), '--external-planner', '--robot-id', role,
        '--torso-topic', torso_topic, '--planner-home-y', '-0.20' if args.robot == '66' else '0.20',
        '--lcm-url', control['lcm_default_url'], '--policy', str(policy_path),
        '--hit-motion-data', str(ROOT/'assets/0302_combined'),
        '--move-motion-data', str(ROOT/'assets/0718-move-160-80hz'),
        '--record-dir', str(record), '--record-mode', 'bounded', '--record-duration', '600']
    sys.argv += startup_options(args.robot, args.right_startup)
    print('V9 startup reference: robot='+args.robot+' options='
          +str(startup_options(args.robot, args.right_startup))
          +'; second R2 starts continuous policy inference.', flush=True)
    os.chdir(deploy/'g1_gym_deploy')

    def stop(signum, frame):
        stage.set('stopped', 'software_signal_'+str(signum)+'_is_not_physical_stop')
        rospy.signal_shutdown('operator stopped experimental active software')
        raise KeyboardInterrupt()

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, stop)
    print('EXPERIMENTAL ACTIVE: native g1 may already move; software exit does not disarm.', flush=True)
    try:
        runpy.run_path(str(entry), run_name='__main__')
    finally:
        rospy.signal_shutdown('experimental active entry exited; physical disarm required')


if __name__ == '__main__':
    main()
