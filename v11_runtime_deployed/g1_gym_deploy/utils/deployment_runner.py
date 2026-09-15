import copy
import time
import os
import sys
import rospy
import numpy as np
import logging
from datetime import datetime
import json
import torch
import signal


class FixedRate:
    def __init__(self, hz):
        self.period = 1.0 / float(hz)
        self.reset()

    def reset(self):
        self.next_time = time.monotonic() + self.period

    def sleep(self):
        now = time.monotonic()
        if self.next_time > now:
            time.sleep(self.next_time - now)
        self.next_time += self.period
        if self.next_time < time.monotonic():
            self.next_time = time.monotonic() + self.period



class DeploymentRunner:
    def __init__(self, experiment_name="unnamed", se=None, shadow=False, recorder=None):
        # Ensure ROS node is initialized before any rospy.Rate/Timer usage
        if not rospy.get_node_uri():
            try:
                rospy.init_node('g1_deploy_runner', anonymous=True)
            except Exception:
                # If another part already initialized or master not ready, we'll try again on first use
                pass
        self.agents = {}
        self.policy = None
        self.command_profile = None
        self.se = se
        self.shadow = bool(shadow)
        self.recorder = recorder

        self.control_agent_name = None
        self.command_agent_name = None

    def sigint_handler(self, sig, frame):
        rospy.loginfo("caught ctrl+c")
        rospy.signal_shutdown("user requested shutdown")



    def add_open_loop_agent(self, agent, name):
        self.agents[name] = agent
        self.logger.add_robot(name, agent.env.cfg)

    def add_control_agent(self, agent, name):
        self.control_agent_name = name
        self.agents[name] = agent

    def set_command_agents(self, name):
        self.command_agent = name

    def add_policy(self, policy):
        self.policy = policy

    def add_command_profile(self, command_profile):
        self.command_profile = command_profile

    def _reset_policy_state(self):
        if hasattr(self.policy, "reset"):
            self.policy.reset()


    def calibrate(self, wait=True):
        if self.shadow:
            control_obs = None
            for agent_name, agent in self.agents.items():
                obs = agent.reset()
                if agent_name == self.control_agent_name:
                    control_obs = obs
            if control_obs is None:
                raise RuntimeError("Shadow runner has no control agent observation.")
            return control_obs
        # first, if the robot is not in nominal pose, move slowly to the nominal pose
        for agent_name in self.agents.keys():
            if hasattr(self.agents[agent_name], "get_obs"):
                agent = self.agents[agent_name]
                agent.get_obs()
                joint_pos = agent.dof_pos[:]

                final_goal = np.array([-0.312,  0.000,  0.000,  0.669, -0.363,  0.000, 
                                         -0.312,  0.000,  0.000,  0.669, -0.363,  0.000,
                                         0.0, 0.0, 0.0,
                                         -0.200,  0.200, 0.000, -0.200, 0.00, 0.0, 0.0, 
                                         -0.200, -0.200, 0.000, -0.200, 0.00, 0.0, 0.0], dtype=float)
                                         

                custom_default = np.array([-0.312,  0.000,  0.000,  0.669, -0.363,  0.000, 
                                         -0.312,  0.000,  0.000,  0.669, -0.363,  0.000,
                                         0.0, 0.0, 0.0,
                                         -0.200,  0.200, 0.000, -0.200, 0.00, 0.0, 0.0, 
                                         -0.200, -0.200, 0.000, -0.200, 0.00, 0.0, 0.0])


                print(f"About to calibrate; the robot will stand [Press R2 to calibrate]")
                while wait and not rospy.is_shutdown():
                    if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                        break
                    rospy.sleep(0.01)
                target = joint_pos
                cal_action = np.zeros((agent.num_envs, agent.num_dofs))
                target_sequence = []
                while np.max(np.abs(target - final_goal)) > 0.01:
                    target -= np.clip((target - final_goal), -0.01, 0.01)
                    target_sequence += [copy.deepcopy(target)]
                for target in target_sequence:
                    next_target = target
                    custom_target = next_target  - custom_default
                    action_scale = agent.action_scale
                    custom_target = custom_target / action_scale
                    cal_action[:, :] = custom_target
                    agent.step(torch.from_numpy(cal_action))
                    agent.get_obs()
                    rospy.sleep(0.05)

                print("Starting pose calibrated [Press R2 to start controller]")

                while True and not rospy.is_shutdown():
                    if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                        break
                    rospy.sleep(0.01)

                for agent_name in self.agents.keys():
                    # print(agent_name)
                    obs = self.agents[agent_name].reset()
                    # obs = self.agents[agent_name].get_obs()
                    # print("!!!!!!!!!!!", obs['obs_history'].shape)
                    if agent_name == self.control_agent_name:
                        control_obs = obs

        return control_obs


    def run(self, num_log_steps=1000000000, max_steps=100000000):

        assert self.control_agent_name is not None, "cannot deploy, runner has no control agent!"
        assert self.command_profile is not None, "cannot deploy, runner has no command profile!"

        # configure logging once at the top of your script
        log_dir = "logs"
        os.makedirs(log_dir, exist_ok=True)

        for agent_name in self.agents.keys():
            obs = self.agents[agent_name].reset()
            if agent_name == self.control_agent_name:
                control_obs = obs
            
        
        try:
            if self.shadow:
                logging.info("shadow mode: calibration and q_des publication are disabled")
            else:
                control_obs = self.calibrate(wait=True)
                logging.info("finish calibrate")

            use_existing_control_obs = True
            rate = FixedRate(50)
            last_loop_start_ns = None
            last_recorder_summary = 0.0
            while not rospy.is_shutdown():
                loop_start_ns = time.perf_counter_ns()
                loop_period_s = (
                    0.0 if last_loop_start_ns is None else (loop_start_ns - last_loop_start_ns) * 1.0e-9
                )
                last_loop_start_ns = loop_start_ns

                if use_existing_control_obs:
                    use_existing_control_obs = False
                else:
                    for agent_name in self.agents.keys():
                        obs = self.agents[agent_name].observe()
                        if agent_name == self.control_agent_name:
                            control_obs = obs

                inference_start_ns = time.perf_counter_ns()
                if hasattr(self.policy, "infer_control"):
                    action = self.policy.infer_control(control_obs)
                else:
                    action = self.policy(control_obs['obs_history'])
                inference_s = (time.perf_counter_ns() - inference_start_ns) * 1.0e-9
                policy_obs_history = getattr(
                    self.policy, "last_policy_observation", control_obs["obs_history"]
                )
                canonical_action = getattr(self.policy, "last_canonical_action", action)
                record_snapshot = None

                for agent_name in self.agents.keys():
                    agent_action = action[:, self.agents[agent_name].from_lab_to_gym]
                    if agent_name == self.control_agent_name and self.recorder is not None:
                        record_snapshot = self.agents[agent_name].make_record_snapshot(
                            control_obs["obs"],
                            control_obs["obs_history"],
                            action,
                            agent_action,
                            loop_period_s,
                            inference_s,
                            policy_obs_history=policy_obs_history,
                            canonical_action_lab=canonical_action,
                        )
                        if "teacher_obs_history" in control_obs:
                            record_snapshot["teacher_obs_history"] = (
                                control_obs["teacher_obs_history"]
                                .detach()
                                .reshape(-1)
                                .cpu()
                                .numpy()
                            )
                        if hasattr(self.policy, "record_snapshot_fields"):
                            record_snapshot.update(self.policy.record_snapshot_fields())
                    apply_start_ns = time.perf_counter_ns()
                    self.agents[agent_name].apply_action(agent_action)
                    apply_publish_s = (time.perf_counter_ns() - apply_start_ns) * 1.0e-9
                    if agent_name == self.control_agent_name and hasattr(
                        self.policy, "record_applied_command"
                    ):
                        self.policy.record_applied_command(
                            agent_action,
                            self.agents[agent_name].joint_pos_target,
                            self.agents[agent_name].last_command_publish_monotonic_raw_ns,
                            loop_period_s,
                            policy_call_s=inference_s,
                            apply_publish_s=apply_publish_s,
                        )
                    if agent_name == self.control_agent_name and record_snapshot is not None:
                        self.agents[agent_name].append_command_timing(record_snapshot)

                if self.recorder is not None and record_snapshot is not None:
                    self.recorder.enqueue(record_snapshot)
                    now = time.monotonic()
                    if now - last_recorder_summary >= 1.0:
                        print(
                            f"[RECORDER] session={self.recorder.session_dir.name} "
                            f"frames={self.recorder.sequence} dropped={self.recorder.dropped_frames} "
                            f"enqueue_ms={self.recorder.last_enqueue_s * 1.0e3:.3f} "
                            f"failed={self.recorder.failed}"
                        )
                        last_recorder_summary = now



                if self.command_profile.state_estimator.right_lower_right_switch_pressed:
                    self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                    if self.shadow:
                        control_obs = self.agents[self.control_agent_name].reset()
                    else:
                        control_obs = self.calibrate(wait=False)
                        rospy.sleep(1)
                        while not self.command_profile.state_estimator.right_lower_right_switch_pressed:
                            rospy.sleep(0.01)
                        self.command_profile.state_estimator.right_lower_right_switch_pressed = False
                    self._reset_policy_state()
                    use_existing_control_obs = True
                    last_loop_start_ns = None
                    rate.reset()
                    continue

                rate.sleep()

            # finally, return to the nominal pose
            if not self.shadow:
                control_obs = self.calibrate(wait=False)

        except KeyboardInterrupt:
            logging.info("Interrupted by user, shutting down.")
        finally:
            if self.recorder is not None:
                self.recorder.close()
            if hasattr(self.policy, "close"):
                self.policy.close()
