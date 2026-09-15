import os
import sys
import tempfile
from types import SimpleNamespace
from unittest import mock

import torch


fake_rospy = SimpleNamespace(
    get_node_uri=lambda: "mock://node",
    init_node=lambda *args, **kwargs: None,
    is_shutdown=lambda: fake_rospy.shutdown_requested,
    sleep=lambda duration: None,
    shutdown_requested=False,
)
sys.modules.setdefault("rospy", fake_rospy)

from utils import deployment_runner as deployment_runner_module


class FakeRunnerAgent:
    from_lab_to_gym = list(range(29))

    def __init__(self, events):
        self.events = events
        self.observation_index = 0

    def _obs(self):
        return {
            "obs": torch.full((1, 172), float(self.observation_index)),
            "obs_history": torch.full((1, 1666), float(self.observation_index)),
        }

    def reset(self):
        self.events.append("reset")
        self.observation_index = 0
        return self._obs()

    def observe(self):
        self.events.append("observe")
        self.observation_index += 1
        return self._obs()

    def apply_action(self, action):
        self.events.append("apply")


class TestFixedRate:
    def test_absolute_deadline_and_reset(self):
        now = [10.0]
        sleeps = []

        def fake_sleep(duration):
            sleeps.append(duration)
            now[0] += duration

        with mock.patch.object(
            deployment_runner_module.time, "monotonic", side_effect=lambda: now[0]
        ), mock.patch.object(deployment_runner_module.time, "sleep", side_effect=fake_sleep):
            rate = deployment_runner_module.FixedRate(50)
            now[0] = 10.005
            rate.sleep()
            assert abs(sleeps[-1] - 0.015) < 1.0e-9
            assert abs(rate.next_time - 10.04) < 1.0e-9

            now[0] = 10.1
            rate.sleep()
            assert abs(rate.next_time - 10.12) < 1.0e-9

            now[0] = 20.0
            rate.reset()
            assert abs(rate.next_time - 20.02) < 1.0e-9


class TestFreshObservationRunner:
    def test_first_tick_reset_then_steady_tick_observes_before_policy(self):
        events = []

        class FakeRate:
            def __init__(self, hz):
                events.append("rate_init")

            def reset(self):
                events.append("rate_reset")

            def sleep(self):
                events.append("sleep")

        runner = deployment_runner_module.DeploymentRunner()
        agent = FakeRunnerAgent(events)
        runner.add_control_agent(agent, "control")
        runner.add_command_profile(
            SimpleNamespace(
                state_estimator=SimpleNamespace(right_lower_right_switch_pressed=False)
            )
        )

        def calibrate(wait=True):
            events.append(f"calibrate_{wait}")
            return agent.reset()

        runner.calibrate = calibrate
        policy_calls = 0

        def policy(obs_history):
            nonlocal policy_calls
            events.append("policy")
            policy_calls += 1
            if policy_calls == 2:
                fake_rospy.shutdown_requested = True
            return torch.zeros(1, 29)

        runner.add_policy(policy)
        fake_rospy.shutdown_requested = False

        previous_cwd = os.getcwd()
        try:
            with tempfile.TemporaryDirectory() as temp_dir:
                os.chdir(temp_dir)
                with mock.patch.object(deployment_runner_module, "FixedRate", FakeRate):
                    runner.run()
        finally:
            os.chdir(previous_cwd)
            fake_rospy.shutdown_requested = False

        control_events = [
            event for event in events if event in ("observe", "policy", "apply", "sleep")
        ]
        assert control_events == [
            "policy",
            "apply",
            "sleep",
            "observe",
            "policy",
            "apply",
            "sleep",
        ]
        assert events.index("calibrate_True") < events.index("rate_init")
