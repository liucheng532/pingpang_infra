"""Private Predictor extension: retain camera identity and expire cached samples.

Callback receipt age is a transport check, never a camera capture-age proof.
The original calibration, ball estimator and published pose frame are retained.
"""
import json
import math
import threading
import time


class SourceFreshness:
    def __init__(self, max_age=.25):
        self.latest = {}
        self.max_age = max_age

    def accept(self, sensor, frame, stamp, receipt, position, quaternion):
        if (type(sensor) is not int or type(frame) is not int or frame < 0 or
                not all(math.isfinite(x) for x in (stamp, receipt, *position, *quaternion)) or
                stamp <= 0 or receipt < 0 or len(position) != 3 or len(quaternion) != 4 or
                sum(x*x for x in quaternion) < 1e-12):
            return False
        old = self.latest.get(sensor)
        if old and (frame <= old['frame'] or stamp <= old['source_stamp_s'] or
                    receipt < old['callback_monotonic_s']):
            return False
        self.latest[sensor] = {'sensor': sensor, 'frame': frame,
            'source_stamp_s': stamp, 'callback_monotonic_s': receipt}
        return True

    def fresh(self, evidence, now):
        return bool(evidence and 0 <= now-evidence['callback_monotonic_s'] <= self.max_age)


def install(module, log_path):
    """Call before importing TableTennis, only in the private snapshot launcher."""
    base = module.Mocap

    class FreshMocap(base):
        def __init__(self, *args, **kwargs):
            self._source_lock = threading.RLock()
            self._source_freshness = SourceFreshness()
            self._last_source_emitted = {}
            self._source_log = open(log_path, 'x', buffering=1)
            super().__init__(*args, **kwargs)

        def _make_tracker_callback(self):
            self._original_source_callback = super()._make_tracker_callback()
            def callback(userdata, data):
                try:
                    sensor = int(data.sensor)
                    if sensor not in (module.DOUBLES_TABLE_LEFT_TRACKER_RIGID_ID,
                            module.DOUBLES_TABLE_RIGHT_TRACKER_RIGID_ID, self._ball_rigid_body_id):
                        return
                    receipt = time.monotonic()
                    stamp = data.msg_time.tv_sec + data.msg_time.tv_usec/1e6
                    pos = [float(x)*module.CHINGMU_POSITION_SCALE for x in data.pos]
                    quat = [float(x) for x in data.quat]
                    if any(abs(x) > module.CHINGMU_INVALID_POS_M for x in pos):
                        return
                    with self._source_lock:
                        if self._source_freshness.accept(sensor, int(data.frameCounter), stamp, receipt, pos, quat):
                            self._original_source_callback(userdata, data)
                except (TypeError, ValueError, OverflowError):
                    return
            return module.CFUNCTYPE(None, module.c_char_p, module.VrpnTracker)(callback)

        def _get_latest_tracker_callback_poses(self):
            with self._source_lock:
                poses = super()._get_latest_tracker_callback_poses()
                return {sensor: (*pose, dict(self._source_freshness.latest[sensor]))
                        for sensor, pose in poses.items()
                        if sensor in self._source_freshness.latest}

        def _run_torso_frame(self, job):
            # Never fall back to the cached legacy primary tracker. The evidence
            # travels with the copied pose through the existing worker queue.
            with self._mocap_lock:
                self.tracker_list = {}
                for sensor, item in sorted((job.tracker_poses or {}).items()):
                    position, quaternion, stamp_ms, evidence = item
                    now = time.monotonic()
                    if (not self._source_freshness.fresh(evidence, now) or
                            self._last_source_emitted.get(sensor, -1) >= evidence['frame']):
                        continue
                    self._apply_torso_branch(position, quaternion, job.frame_ros_time_sec,
                                             rigid_body_id=sensor)
                    self._last_source_emitted[sensor] = evidence['frame']
                    self._source_log.write(json.dumps({'kind':'mocap_torso_source', **evidence,
                        'processing_monotonic_s':now, 'published_ros_stamp_s':job.frame_ros_time_sec,
                        'tracker_world_position':position.tolist(),
                        'tracker_world_quaternion_xyzw':quaternion.tolist(),
                        'camera_capture_age_verified':False}, allow_nan=False)+'\n')

        def _get_latest_ball_callback_pose(self):
            with self._source_lock:
                evidence = self._source_freshness.latest.get(self._ball_rigid_body_id)
                if not self._source_freshness.fresh(evidence, time.monotonic()):
                    return None
                pose = super()._get_latest_ball_callback_pose()
                if pose is not None:
                    pose['yichao_source_evidence'] = dict(evidence)
                return pose

        def _run_ball_planner_frame(self, job):
            pose = job.ball_pose
            if pose is not None and not self._source_freshness.fresh(
                    pose.get('yichao_source_evidence'), time.monotonic()):
                job = job._replace(ball_pose=None)
            return super()._run_ball_planner_frame(job)

    module.Mocap = FreshMocap
