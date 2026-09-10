from __future__ import annotations

import unittest

import numpy as np

from doubles_planner.centralized import (
    OBSERVATION_NAMES_V2,
    BallPolicyState,
    CentralizedRelaySupervisor,
    DoublesCommandAction,
    DoublesPolicyObservation,
    RobotPolicyState,
    guard_doubles_action,
)


def _state(name: str, y: float, *, phase: str = "HOME_HOLD", timestamp: float = 1.0, ready: bool = True) -> RobotPolicyState:
    return RobotPolicyState(
        name=name,
        base_position=[0.0, y, 0.8],
        base_linear_velocity=[0.0, 0.0, 0.0],
        joint_position=np.linspace(-0.1, 0.1, 29),
        joint_velocity=np.zeros(29),
        imu=np.zeros(6),
        contact=np.ones(4),
        controller_phase=phase,
        ready=ready,
        timestamp=timestamp,
    )


def _observation(**overrides: object) -> DoublesPolicyObservation:
    values: dict[str, object] = {
        "robots": (_state("left", -0.35), _state("right", 0.35)),
        "ball": BallPolicyState(
            position=[0.4, 0.0, 1.0],
            velocity=[-2.0, 0.2, 0.0],
            predicted_strike_position=[0.4, -0.2, 1.0],
            predicted_strike_velocity=[-2.0, -0.1, 0.0],
            time_to_strike_s=0.4,
            timestamp=1.0,
        ),
        "relay_stage": "PREPARED",
        "timestamp": 1.0,
        "next_hitter_index": 0,
    }
    values.update(overrides)
    return DoublesPolicyObservation(**values)


class CentralizedObservationTests(unittest.TestCase):
    def test_joint_vector_is_fixed_finite_and_contains_both_robot_inputs(self) -> None:
        observation = _observation()
        vector = observation.vector_v2()

        self.assertEqual(vector.shape, (len(OBSERVATION_NAMES_V2),))
        self.assertTrue(np.isfinite(vector).all())
        for feature in (
            "left_base_position_y",
            "right_base_position_y",
            "left_joint_position_0",
            "right_joint_velocity_28",
            "ball_position_y",
            "ball_time_to_strike",
            "left_skill_hit_legal",
            "right_skill_hit_legal",
        ):
            self.assertIn(feature, OBSERVATION_NAMES_V2)

    def test_stale_prediction_forces_both_commands_to_hold(self) -> None:
        observation = _observation(
            ball=BallPolicyState(time_to_strike_s=0.4, prediction_age_s=1.0, timestamp=1.0)
        )
        action = np.zeros(DoublesCommandAction.num_actions)
        action[4] = 10.0
        batch = guard_doubles_action(action, observation)

        self.assertTrue(batch.stale_input)
        self.assertEqual([command.skill for command in batch.commands], ["HOLD", "HOLD"])
        self.assertFalse(any(command.command_valid for command in batch.commands))

    def test_stale_prediction_timestamp_cannot_authorize_hit(self) -> None:
        observation = _observation(
            ball=BallPolicyState(time_to_strike_s=0.4, timestamp=-10.0)
        )
        batch = guard_doubles_action(CentralizedActionGuardTests._hit_action(), observation)
        self.assertTrue(batch.stale_input)
        self.assertFalse(batch.commit_requested)


class CentralizedActionGuardTests(unittest.TestCase):
    @staticmethod
    def _hit_action(left: bool = True, right: bool = False) -> np.ndarray:
        action = np.zeros(DoublesCommandAction.num_actions, dtype=np.float32)
        if left:
            action[4] = 10.0
        if right:
            action[10] = 10.0
        return action

    def test_one_accepted_hit_starts_peer_clear_atomically(self) -> None:
        batch = guard_doubles_action(self._hit_action(), _observation())

        self.assertTrue(batch.commit_requested)
        self.assertEqual(batch.accepted_hitter_index, 0)
        self.assertEqual([command.skill for command in batch.commands], ["HIT", "CLEAR"])
        self.assertIsNotNone(batch.commands[0].hit_request)
        self.assertIn("commit_accepted", batch.reasons)

    def test_simultaneous_hit_is_rejected(self) -> None:
        batch = guard_doubles_action(self._hit_action(left=True, right=True), _observation())

        self.assertFalse(batch.commit_requested)
        self.assertTrue(batch.simultaneous_hit)
        self.assertEqual([command.skill for command in batch.commands], ["HOLD", "HOLD"])
        self.assertIn("simultaneous_hit_rejected", batch.reasons)

    def test_late_or_wrong_robot_hit_is_masked(self) -> None:
        observation = _observation(
            relay_stage="COMMITTED",
            active_hitter_index=0,
            next_hitter_index=0,
        )
        batch = guard_doubles_action(self._hit_action(), observation)

        self.assertFalse(batch.commit_requested)
        self.assertEqual([command.skill for command in batch.commands], ["HOLD", "HOLD"])
        self.assertIn("hit_not_legal", batch.reasons)

    def test_supervisor_latches_commit_and_ignores_repeat(self) -> None:
        supervisor = CentralizedRelaySupervisor()
        first = supervisor.step(_observation(), self._hit_action())
        committed_observation = _observation(relay_stage="COMMITTED", active_hitter_index=0)
        repeated = supervisor.step(committed_observation, self._hit_action())

        self.assertEqual(first.accepted_hitter_index, 0)
        self.assertFalse(repeated.commit_requested)
        self.assertIn("commit_already_latched", repeated.reasons)
        self.assertIsNotNone(first.commit_token)

    def test_supervisor_releases_latch_without_shot_id_on_new_stage_cycle(self) -> None:
        supervisor = CentralizedRelaySupervisor()
        first = supervisor.step(_observation(), self._hit_action())
        supervisor.step(_observation(relay_stage="COMMITTED", active_hitter_index=0), np.zeros(DoublesCommandAction.num_actions))
        supervisor.step(
            _observation(relay_stage="RESERVED", next_hitter_index=1, ball=BallPolicyState(time_to_strike_s=0.4, timestamp=1.0)),
            np.zeros(DoublesCommandAction.num_actions),
        )
        next_shot = supervisor.step(
            _observation(relay_stage="PREPARED", next_hitter_index=1, ball=BallPolicyState(time_to_strike_s=0.4, timestamp=1.0)),
            self._hit_action(left=False, right=True),
        )
        self.assertTrue(first.commit_requested)
        self.assertTrue(next_shot.commit_requested)
        self.assertEqual(next_shot.accepted_hitter_index, 1)


if __name__ == "__main__":
    unittest.main()
