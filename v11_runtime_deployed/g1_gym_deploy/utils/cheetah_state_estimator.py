import math
import select
import struct
import threading
import time

import numpy as np

from lcm_types.body_control_data_lcmt import body_control_data_lcmt
from lcm_types.rc_command_lcmt import rc_command_lcmt
from lcm_types.state_estimator_lcmt import state_estimator_lcmt


_LATENCY_COMMAND_TRACE = struct.Struct(">QQQQ")


def monotonic_raw_ns():
    clock_id = getattr(time, "CLOCK_MONOTONIC_RAW", time.CLOCK_MONOTONIC)
    return time.clock_gettime_ns(clock_id)


def get_rpy_from_quaternion(q):
    w, x, y, z = q
    r = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
    p = np.arcsin(2 * (w * y - z * x))
    y = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    return np.array([r, p, y])


def get_rotation_matrix_from_rpy(rpy):
    """
    Get rotation matrix from the given quaternion.
    Args:
        q (np.array[float[4]]): quaternion [w,x,y,z]
    Returns:
        np.array[float[3,3]]: rotation matrix.
    """
    r, p, y = rpy
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(r), -math.sin(r)],
                    [0, math.sin(r), math.cos(r)]
                    ])

    R_y = np.array([[math.cos(p), 0, math.sin(p)],
                    [0, 1, 0],
                    [-math.sin(p), 0, math.cos(p)]
                    ])

    R_z = np.array([[math.cos(y), -math.sin(y), 0],
                    [math.sin(y), math.cos(y), 0],
                    [0, 0, 1]
                    ])

    rot = np.dot(R_z, np.dot(R_y, R_x))
    return rot


class StateEstimator:
    def __init__(self, lc):

        # The C++ publisher already emits the 29 hardware joints in Gym order.
        self.joint_idxs = list(range(29))
        if len(self.joint_idxs) != len(set(self.joint_idxs)):
            raise RuntimeError(f"Hardware joint indices must be unique: {self.joint_idxs}")

        self.lc = lc
        self.num_dofs = 29
        self.joint_pos = np.zeros(29)
        self.joint_vel = np.zeros(29)
        self.euler = np.zeros(3)
        self.R = np.eye(3)
        self.buf_idx = 0
        self.imu_ang_vel = np.zeros(3)
        
        self.left_stick = [0, 0]
        self.right_stick = [0, 0]
        self.right_lower_right_switch = 0
        self.right_lower_right_switch_pressed = 0

        self.right_upper_switch = 0
        self.right_upper_switch_pressed = 0

        self.left_upper_switch = 0


        self.init_time = time.time()
        self.received_first_bodydate = False
        self.last_body_receive_monotonic_ns = 0
        self.last_imu_receive_monotonic_ns = 0
        self.last_body_receive_monotonic_raw_ns = 0
        self.last_imu_receive_monotonic_raw_ns = 0
        self.body_source_monotonic_raw_ns = 0
        self.imu_source_monotonic_raw_ns = 0
        self.trace_command_seq = 0
        self.command_lcm_receive_monotonic_raw_ns = 0
        self.command_control_monotonic_raw_ns = 0
        self.command_dds_write_monotonic_raw_ns = 0

        self.imu_subscription = self.lc.subscribe("state_estimator_data", self._imu_cb)
        self.bodydate_state_subscription = self.lc.subscribe("body_control_data", self._bodydata_cb)
        self.rc_command_subscription = self.lc.subscribe("rc_command", self._rc_command_cb)
        self.latency_trace_subscription = self.lc.subscribe(
            "latency_command_trace", self._latency_trace_cb
        )
        self.body_quat = np.array([0, 0, 0, 1])
        self.smoothing_ratio = 0.2
        self.body_ang_vel = np.zeros(3)
        self.body_lin_vel = np.zeros(3)
        self.smoothing_length = 12
        self.dt_history = np.zeros((self.smoothing_length, 1))
        self.euler_prev = np.zeros(3)
        self.deuler_history = np.zeros((self.smoothing_length, 3))
        self.timeuprev = time.time()
        self.world_vel = np.zeros(3)
        self.world_acc = np.zeros(3)
        

    def get_gravity_vector(self):
        grav = np.dot(self.R.T, np.array([0, 0, -1]))
        return grav


    def get_rpy(self):
        return self.euler


    def get_command(self):
        # always in use
        lin_v = 1.6 * self.left_stick[1]
        target_yaw = 0.4 * self.right_stick[0]
        return np.array([lin_v, 0, -target_yaw])

    def get_aside_command(self):
        # always in use
        vel = 1.5 * self.left_stick[0]
        vel = np.clip(vel,a_min=-1.0, a_max=1.0)
        return np.array([vel])

    def get_buttons(self):
        return self.right_lower_right_switch
        
    def get_dof_pos(self):
        return self.joint_pos[self.joint_idxs]

    def get_dof_vel(self):
        return self.joint_vel[self.joint_idxs]

    def get_yaw(self):
        return self.euler[2]

    def get_body_angular_vel(self):
        # self.body_ang_vel = self.smoothing_ratio * np.mean(self.deuler_history / self.dt_history, axis=0) + (1 - self.smoothing_ratio) * self.body_ang_vel
        # return self.body_ang_vel
        return self.body_ang_vel

    def get_body_linear_vel(self):
        self.body_lin_vel += 0.02 * self.world_acc
        self.body_lin_vel = np.dot(self.R.T, self.world_vel)
        return self.body_lin_vel

    def _bodydata_cb(self, channel, data):
        self.last_body_receive_monotonic_ns = time.monotonic_ns()
        self.last_body_receive_monotonic_raw_ns = monotonic_raw_ns()
        if not self.received_first_bodydate:
            self.received_first_bodydate = True
            print(f"First body data: {time.time() - self.init_time}")

        msg = body_control_data_lcmt.decode(data)
        self.joint_pos = np.array(msg.q)
        self.joint_vel = np.array(msg.qd)
        self.body_source_monotonic_raw_ns = int(msg.timestamp_us) * 1000

    def _imu_cb(self, channel, data):
        self.last_imu_receive_monotonic_ns = time.monotonic_ns()
        self.last_imu_receive_monotonic_raw_ns = monotonic_raw_ns()
        msg = state_estimator_lcmt.decode(data)
        self.imu_source_monotonic_raw_ns = int(msg.timestamp_us) * 1000

        self.euler = np.array(msg.rpy)

        self.R = get_rotation_matrix_from_rpy(self.euler)
        self.body_quat = np.array(msg.quat)
        self.body_ang_vel = np.array(msg.omegaBody)

        self.deuler_history[self.buf_idx % self.smoothing_length, :] = msg.rpy - self.euler_prev
        self.dt_history[self.buf_idx % self.smoothing_length] = time.time() - self.timeuprev
        self.timeuprev = time.time()
        self.buf_idx += 1
        self.euler_prev = np.array(msg.rpy)
        # self.world_vel += np.array(msg.aBody) * 0.02
        self.world_acc = np.array(msg.aBody)

    def _latency_trace_cb(self, channel, data):
        if len(data) != _LATENCY_COMMAND_TRACE.size:
            return
        (
            self.trace_command_seq,
            self.command_lcm_receive_monotonic_raw_ns,
            self.command_control_monotonic_raw_ns,
            self.command_dds_write_monotonic_raw_ns,
        ) = _LATENCY_COMMAND_TRACE.unpack(data)
        
    def _imu_torso(self, channel, data):
        msg = state_estimator_lcmt.decode(data)
        self.torsoeuler = np.array(msg.rpy)

    def _rc_command_cb(self, channel, data):

        msg = rc_command_lcmt.decode(data)

        self.right_lower_right_switch_pressed = ((msg.right_lower_right_switch and not self.right_lower_right_switch) or self.right_lower_right_switch_pressed)
        self.right_upper_switch_pressed = ((msg.right_upper_switch and not self.right_upper_switch) or self.right_upper_switch)

        self.right_stick = msg.right_stick
        self.left_stick = msg.left_stick
        self.right_lower_right_switch = msg.right_lower_right_switch
        self.right_upper_switch = msg.right_upper_switch
        self.left_upper_switch = msg.left_upper_switch

    def poll(self, cb=None):
        t = time.time()
        try:
            while True:
                timeout = 0.01
                rfds, wfds, efds = select.select([self.lc.fileno()], [], [], timeout)
                if rfds:
                    # print("message received!")
                    self.lc.handle()
                    # print(f'Freq {1. / (time.time() - t)} Hz'); t = time.time()
                else:
                    continue

        except KeyboardInterrupt:
            pass

    def spin(self):
        self.run_thread = threading.Thread(target=self.poll, daemon=True)
        self.run_thread.start()

    def close(self):
        self.lc.unsubscribe(self.bodydate_state_subscription)


if __name__ == "__main__":
    import lcm

    lc = lcm.LCM("udpm://239.255.76.198:7667?ttl=255")
    se = StateEstimator(lc)
    se.poll()
