import json
from collections import namedtuple
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch
import threading
import numpy as np
from yichao_v3_v9.predictor_freshness import SourceFreshness, install


class SourceTests(TestCase):
    def test_stale_duplicate_and_reordered_source_does_not_refresh_receipt(self):
        f=SourceFreshness()
        self.assertTrue(f.accept(0,7,100.,10.,[0,0,1],[0,0,0,1]))
        for frame,stamp in [(7,100.01),(8,99.),(6,101.)]:
            self.assertFalse(f.accept(0,frame,stamp,10.2,[0,0,1],[0,0,0,1]))
        self.assertTrue(f.fresh(f.latest[0],10.25))
        self.assertFalse(f.fresh(f.latest[0],10.251))
        self.assertFalse(f.fresh(f.latest[0],9.9))

    def test_invalid_timestamp_and_quaternion_are_not_repaired(self):
        f=SourceFreshness()
        for stamp,quat in [(0.,[0,0,0,1]),(float('nan'),[0,0,0,1]),(10.,[0,0,0,0])]:
            self.assertFalse(f.accept(1,1,stamp,1.,[0,0,1],quat))
        self.assertEqual(f.latest,{})

    def test_independent_rigid_counters(self):
        f=SourceFreshness()
        for sensor in [0,1,30300]:
            self.assertTrue(f.accept(sensor,1,100.,10.,[0,0,1],[0,0,0,1]))
        self.assertEqual(len(f.latest),3)


class FakeBase:
    def __init__(self):
        self._mocap_lock=threading.Lock();self.calls=[]
    def _apply_torso_branch(self,*args,**kw):self.calls.append((args,kw))
    def _run_ball_planner_frame(self,job):self.ball_job=job


class ExtensionTests(TestCase):
    def test_queue_stall_and_duplicate_do_not_republish_torso(self):
        with TemporaryDirectory() as tmp:
            m=SimpleNamespace(Mocap=FakeBase);install(m,Path(tmp)/'source.jsonl');obj=m.Mocap()
            Job=namedtuple('Job','tracker_poses frame_ros_time_sec ball_pose')
            e={'sensor':0,'frame':8,'source_stamp_s':100.,'callback_monotonic_s':10.}
            job=Job({0:(np.zeros(3),np.array([0,0,0,1]),100000,e)},200.,None)
            with patch('yichao_v3_v9.predictor_freshness.time.monotonic',return_value=10.1):
                obj._run_torso_frame(job);obj._run_torso_frame(job)
            self.assertEqual(len(obj.calls),1)
            self.assertEqual(obj.calls[0][0][2],200.)
            later=job._replace(tracker_poses={0:(np.zeros(3),np.array([0,0,0,1]),100001,{**e,'frame':9})})
            with patch('yichao_v3_v9.predictor_freshness.time.monotonic',return_value=10.3):
                obj._run_torso_frame(later)
                obj._run_ball_planner_frame(job._replace(ball_pose={'yichao_source_evidence':e}))
            self.assertEqual(len(obj.calls),1)
            self.assertIsNone(obj.ball_job.ball_pose)
            obj._source_log.close()
            record=json.loads((Path(tmp)/'source.jsonl').read_text())
            self.assertEqual(record['source_stamp_s'],100.)
            self.assertFalse(record['camera_capture_age_verified'])
