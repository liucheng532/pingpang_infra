import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from yichao_v3_v9.ball_feedback import BallFeedback


def message(seq=1, state='waiting'):
    return {'version':1,'sequence':seq,'timestamp':1000.+seq*.02,
            'planner':{'state':state,'planning_armed':True,'hold_post_hit_output':False,'time_to_strike':.4},
            'ball':{'age_s':0.,'current_origin_filtered':[1.,.2,1.],'current_velocity_origin':[-3.,.1,-.2]},
            'prediction':{'position_origin':[.45,.2,1.],'velocity_origin':[-2.9,.1,-.3]},
            'racket':{'normal_origin':[1.,0.,0.],'velocity_origin':[2.,.2,.5]}}


class BallTests(unittest.TestCase):
    def test_waiting_then_tracking_latches_one_shot(self):
        a=BallFeedback('session')
        self.assertFalse(a.parse(message(),1.)['prediction_fields_valid'])
        first=a.parse(message(2,'tracking'),1.02)
        second=a.parse(message(3,'tracking'),1.04)
        self.assertTrue(first['prediction_fields_valid'])
        self.assertEqual(first['diagnostic_shot_id'],second['diagnostic_shot_id'])
        self.assertFalse(first['real_input_accepted'])
        self.assertFalse(first['command_ack'])

    def test_start_midshot_and_draining_do_not_start_new_shot(self):
        a=BallFeedback('session')
        self.assertFalse(a.parse(message(1,'tracking'),1.)['prediction_fields_valid'])
        a.parse(message(2,'waiting'),2.)
        first=a.parse(message(3,'tracking'),3.)
        self.assertFalse(a.parse(message(4,'draining'),4.)['prediction_fields_valid'])
        self.assertFalse(a.parse(message(5,'tracking'),5.)['prediction_fields_valid'])
        a.parse(message(6,'waiting'),6.)
        second=a.parse(message(7,'tracking'),7.)
        self.assertNotEqual(first['diagnostic_shot_id'],second['diagnostic_shot_id'])

    def test_duplicate_and_restart_latch_fault(self):
        a=BallFeedback('session');a.parse(message(),1.)
        with self.assertRaises(ValueError):a.parse(message(),2.)
        with self.assertRaisesRegex(ValueError,'ball_stream_fault_latched'):
            a.parse(message(3,'tracking'),3.)

    def test_expired_stale_disarmed_and_held(self):
        for field,value in [('time_to_strike',-.52),('planning_armed',False),('hold_post_hit_output',True)]:
            a=BallFeedback('session');a.parse(message(),1.)
            msg=message(2,'tracking');msg['planner'][field]=value
            self.assertFalse(a.parse(msg,2.)['prediction_fields_valid'])
        a=BallFeedback('session');a.parse(message(),1.)
        msg=message(2,'tracking');msg['ball']['age_s']=.31
        self.assertFalse(a.parse(msg,2.)['prediction_fields_valid'])

    def test_current_ball_strike_and_racket_velocities_not_confused(self):
        a=BallFeedback('session');a.parse(message(),1.)
        msg=message(2,'tracking');before=copy.deepcopy(msg);out=a.parse(msg,2.)
        self.assertEqual(out['velocity'],[-3.,.1,-.2])
        self.assertEqual(out['strike_velocity'],[-2.9,.1,-.3])
        self.assertEqual(out['racket_velocity'],[2.,.2,.5])
        self.assertEqual(msg,before)
        self.assertFalse(out['sensor_source_timestamp_available'])

    def test_invalid_dimensions_and_nonfinite_are_rejected(self):
        for value in ([1.,2.], [1.,float('nan'),2.]):
            a=BallFeedback('session');a.parse(message(),1.)
            msg=message(2,'tracking');msg['racket']['velocity_origin']=value
            with self.assertRaises(ValueError):a.parse(msg,2.)


if __name__=='__main__':
    unittest.main()
