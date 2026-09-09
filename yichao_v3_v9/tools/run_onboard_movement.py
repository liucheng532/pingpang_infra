"""Run the frozen V9 policy with a private movement-only bridge. No HIT hook."""
import argparse
import fcntl
import os
from pathlib import Path
import re
import runpy
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]


def main(default_protocol='movement'):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session', required=True)
    p.add_argument('--robot', choices=('66', '198'), required=True)
    p.add_argument('--deploy-root', type=Path, required=True, help='independent frozen V9 release')
    p.add_argument('--mode', choices=('shadow', 'active'), default='shadow')
    p.add_argument('--torso-topic', required=True)
    p.add_argument('--record-dir', type=Path, required=True)
    p.add_argument('--seconds', type=float, default=30.)
    p.add_argument('--protocol', choices=('movement','relay'), default=default_protocol)
    args = p.parse_args()
    if args.mode != 'shadow':
        p.error('active unavailable: real input contract and physical control acceptance are pending')
    if not 0 < args.seconds <= 60:
        p.error('bounded shadow requires duration in (0, 60] seconds')
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', args.session):
        p.error('session must be a safe ROS name')
    deploy = args.deploy_root.resolve()
    if deploy != (ROOT/'vendor/deploy').resolve():
        p.error('deploy-root must be this release\'s frozen vendor/deploy')
    role = 'table_right' if args.robot == '66' else 'table_left'
    prefix = '/yichao_v3_v9/'+args.session+'/'+role
    if args.torso_topic != prefix+'/torso_pose_origin':
        p.error('torso topic must belong to this session and robot role')
    sys.path[:0] = [str(ROOT/'src'), str(deploy/'g1_gym_deploy'),
                   str(ROOT/'vendor/ros1')]
    os.environ.update(ROS_MASTER_URI='http://172.16.3.126:11311', ROS_IP='172.16.4.'+args.robot,
                      OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1',
                      CUDA_VISIBLE_DEVICES='')
    os.environ.pop('ROS_HOSTNAME', None)
    from yichao_v3_v9.onboard_isolation import ReadOnlyLCM, verify_release, control_processes
    conflicts = control_processes()
    if conflicts:
        p.error('existing control/policy process; no onboard shadow alongside it: '+str(conflicts))
    lock = (ROOT/'onboard_shadow.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        p.error('another onboard shadow owns this release')
    verify_release(ROOT)
    import rospy
    import lcm
    import torch
    import onnxruntime as ort
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    session_factory = ort.InferenceSession
    def cpu_session(*a, **kw):
        options = ort.SessionOptions()
        options.intra_op_num_threads = options.inter_op_num_threads = 1
        kw.update(sess_options=options, providers=['CPUExecutionProvider'])
        return session_factory(*a, **kw)
    ort.InferenceSession = cpu_session
    lcm_factory = lcm.LCM
    telemetry_url = 'udpm://239.255.77.'+args.robot+':7767?ttl=0'
    def readonly_lcm(url):
        if url != telemetry_url:
            raise RuntimeError('shadow LCM URL must be the private passive telemetry group')
        return ReadOnlyLCM(lcm_factory(url))
    lcm.LCM = readonly_lcm
    # Initialize explicit remappings before the stock CLI parser is invoked.
    remaps = [sys.argv[0]] + ['/doubles/'+role+'/'+name+':='+prefix+'/'+name for name in ('state', 'command')]
    rospy.init_node('yichao_movement_'+args.session+'_'+args.robot, argv=remaps,
                    disable_rosout=True, disable_rostime=True)
    stop = threading.Timer(args.seconds, lambda: rospy.signal_shutdown('bounded onboard shadow finished'))
    stop.daemon = True
    stop.start()
    from yichao_v3_v9.onboard_movement import bridge_class
    import utils.planner_ros_bridge as stock
    stock.PlannerRosBridge = bridge_class(args.session, shadow=args.mode == 'shadow',protocol=args.protocol)
    entry = deploy/'g1_gym_deploy/scripts/deploy_policy.py'
    args.record_dir.mkdir(parents=True, exist_ok=False)
    sys.argv = [str(entry), '--external-planner', '--robot-id', role,
        '--torso-topic', args.torso_topic, '--planner-home-y', '-0.20' if args.robot == '66' else '0.20',
        '--lcm-url', telemetry_url,
        '--policy', str(ROOT/'assets/robot_66/student_v9_m14500_timedhandoff_1666_model19000.onnx'),
        '--hit-motion-data', str(ROOT/'assets/0302_combined'),
        '--move-motion-data', str(ROOT/'assets/0718-move-160-80hz'),
        '--record-dir', str(args.record_dir)]
    sys.argv += ['--mirror-left-hand', '--bootstrap-outward-hold'] if args.robot == '66' else ['--startup-home-current']
    if args.mode == 'shadow':
        sys.argv.append('--shadow')
    os.chdir(deploy/'g1_gym_deploy')
    try:
        runpy.run_path(str(entry), run_name='__main__')
    finally:
        stop.cancel()
        rospy.signal_shutdown('onboard shadow exited')


if __name__ == '__main__':
    main()
