from __future__ import annotations

import unittest

import numpy as np

from doubles_planner import BallPrediction, PlannerConfig, RelayPlanner, RobotFeedback
from doubles_planner.protocol import OUTPUT_SUFFIXES, output_topics
from doubles_planner.reachability import reachability_waypoints, viability_value
from doubles_planner.safety import cbf_waypoints
from doubles_planner.safety import _bounded_halfspace_projection
from doubles_planner.strategies import project_ordered_pair


def prediction(shot_id: int, time_to_strike: float, timestamp: float, y: float = 0.0) -> BallPrediction:
    return BallPrediction(
        position=[0.45, y, 1.0],
        velocity=[-3.5, 0.0, 0.0],
        time_to_strike=time_to_strike,
        timestamp=timestamp,
        racket_normal=[0.93, 0.0, 0.36],
        racket_velocity=[1.9, 0.0, 0.7],
        shot_id=shot_id,
    )


def feedback(timestamp: float, left_y: float = -0.35, right_y: float = 0.35):
    return {
        "left": RobotFeedback("left", [0.0, left_y], timestamp=timestamp),
        "right": RobotFeedback("right", [0.0, right_y], timestamp=timestamp),
    }


class RelayPlannerTests(unittest.TestCase):
    def test_stage_supervisor_reserves_early_and_commits_once(self) -> None:
        planner = RelayPlanner(
            PlannerConfig(initial_hitter="left"),
            strategy="relay_heuristic",
        )

        reserved = planner.plan(prediction(1, 0.9, 1.0), feedback(1.0), 1.0)
        updated = planner.plan(prediction(1, 0.7, 1.1, 0.4), feedback(1.1), 1.1)
        committed = planner.plan(prediction(1, 0.5, 1.2), feedback(1.2), 1.2)
        repeated = planner.plan(prediction(1, 0.4, 1.3), feedback(1.3), 1.3)

        self.assertEqual(reserved.relay_stage, "prepared")
        self.assertEqual(reserved.phase, "reservation")
        self.assertEqual(updated.hitter, "left")
        self.assertFalse(any(command.active for command in reserved.commands.values()))
        self.assertEqual(reserved.commands["left"].role, "reserved_hitter")
        self.assertTrue(committed.commit_requested)
        self.assertEqual(committed.relay_stage, "committed")
        self.assertEqual(committed.commands["left"].role, "hit")
        self.assertEqual(committed.commands["right"].role, "clear")
        self.assertFalse(repeated.commit_requested)
        self.assertEqual(repeated.commit_token, committed.commit_token)

    def test_turn_token_flips_only_after_controller_handoff(self) -> None:
        planner = RelayPlanner(
            PlannerConfig(initial_hitter="left"),
            strategy="relay_heuristic",
        )

        def staged_feedback(timestamp: float, left_phase: str, right_phase: str):
            return {
                "left": RobotFeedback(
                    "left",
                    [0.0, -0.35],
                    timestamp=timestamp,
                    controller_phase=left_phase,
                    ready=left_phase == "HOME_HOLD",
                ),
                "right": RobotFeedback(
                    "right",
                    [0.0, 0.35],
                    timestamp=timestamp,
                    controller_phase=right_phase,
                    ready=right_phase == "HOME_HOLD",
                ),
            }

        planner.plan(
            prediction(1, 0.9, 1.0),
            staged_feedback(1.0, "HOME_HOLD", "HOME_HOLD"),
            1.0,
        )
        planner.plan(
            prediction(1, 0.5, 1.1),
            staged_feedback(1.1, "HOME_HOLD", "HOME_HOLD"),
            1.1,
        )
        planner.plan(
            prediction(1, 0.4, 1.2),
            staged_feedback(1.2, "HIT", "HOME_HOLD"),
            1.2,
        )
        pending = planner.plan(
            prediction(2, 0.8, 1.3),
            staged_feedback(1.3, "HIT", "HOME_HOLD"),
            1.3,
        )
        second_commit = planner.plan(
            prediction(2, 0.5, 1.4),
            staged_feedback(1.4, "HOME_HOLD", "HOME_HOLD"),
            1.4,
        )

        self.assertEqual(pending.hitter, "right")
        self.assertFalse(pending.commit_requested)
        self.assertTrue(pending.diagnostics["commit_blocked_by_active_shot"])
        self.assertEqual(second_commit.hitter, "right")
        self.assertTrue(second_commit.commit_requested)

    def test_unsafe_commit_is_deferred_then_aborted_not_forced(self) -> None:
        planner = RelayPlanner(
            PlannerConfig(initial_hitter="left"),
            strategy="relay_heuristic",
        )
        close = feedback(1.0, left_y=-0.20, right_y=0.20)

        deferred = planner.plan(prediction(1, 0.5, 1.0), close, 1.0)
        aborted = planner.plan(
            prediction(1, 0.05, 1.1),
            feedback(1.1, left_y=-0.20, right_y=0.20),
            1.1,
        )

        self.assertFalse(deferred.commit_requested)
        self.assertIn("base_hard_margin", deferred.diagnostics["reasons"])
        self.assertEqual(aborted.relay_stage, "aborted")
        self.assertFalse(any(command.active for command in aborted.commands.values()))
        self.assertIn("late_commit_aborted", aborted.fallbacks)

    def test_config_requires_velocity_preserving_cbf_waypoint_horizon(self) -> None:
        with self.assertRaisesRegex(
            ValueError,
            r"cbf_horizon \* base_position_gain must equal 1",
        ):
            PlannerConfig(base_position_gain=10.0)

    def test_deploy_topic_contract_is_unchanged(self) -> None:
        topics = output_topics("left")
        self.assertEqual(set(topics), set(OUTPUT_SUFFIXES))
        self.assertEqual(topics["desired_base_position"], "/left/desired_base_position")

    def test_strict_alternation_and_shot_latch(self) -> None:
        planner = RelayPlanner(strategy="relay_heuristic")
        first = planner.plan(prediction(1, 0.5, 1.0, -0.4), feedback(1.0), 1.0)
        repeated = planner.plan(prediction(1, 0.4, 1.1, 0.4), feedback(1.1), 1.1)
        self.assertEqual(first.hitter, "left")
        self.assertEqual(repeated.hitter, "left")
        planner.plan(prediction(1, -0.6, 1.2), feedback(1.2), 1.2)
        second = planner.plan(prediction(2, 0.5, 1.3, -0.5), feedback(1.3), 1.3)
        self.assertEqual(second.hitter, "right")
        self.assertEqual(second.next_hitter, "left")

    def test_first_hitter_selection_accounts_for_mirrored_racket_reach(self) -> None:
        planner = RelayPlanner(strategy="relay_heuristic")
        states = feedback(1.0, left_y=-0.1, right_y=0.1)
        result = planner.plan(prediction(1, 0.5, 1.0, 0.2), states, 1.0)
        self.assertEqual(result.hitter, "left")

    def test_configured_initial_hitter_starts_strict_relay(self) -> None:
        planner = RelayPlanner(
            PlannerConfig(initial_hitter="left"),
            strategy="cbf_mpc",
        )
        first = planner.plan(prediction(1, 0.5, 1.0, 0.6), feedback(1.0), 1.0)
        planner.plan(prediction(1, -0.6, 1.1), feedback(1.1), 1.1)
        second = planner.plan(prediction(2, 0.5, 1.2, -0.6), feedback(1.2), 1.2)
        self.assertEqual(first.hitter, "left")
        self.assertEqual(second.hitter, "right")

    def test_only_hitter_receives_active_policy_command(self) -> None:
        planner = RelayPlanner(strategy="cbf_mpc")
        result = planner.plan(prediction(1, 0.5, 1.0, -0.2), feedback(1.0), 1.0)
        active = [name for name, command in result.commands.items() if command.active]
        self.assertEqual(active, [result.hitter])
        inactive = result.commands[result.next_hitter]
        self.assertEqual(inactive.predicted_ball_predict_time, -0.5)
        np.testing.assert_allclose(inactive.predicted_racket_velocity, np.zeros(3))

    def test_relay_base_goal_compensates_racket_reach(self) -> None:
        planner = RelayPlanner(strategy="relay_heuristic")
        result = planner.plan(prediction(1, 0.5, 1.0, -0.4), feedback(1.0), 1.0)
        self.assertEqual(result.hitter, "left")
        expected_base_y = -0.4 - planner.config.racket_reach_y[0]
        self.assertAlmostEqual(result.diagnostics["nominal_base_y"], expected_base_y)
        self.assertAlmostEqual(result.commands["left"].predicted_ball_position[1], -0.4)

    def test_pre_hit_teammate_moves_to_own_outward_lane(self) -> None:
        self.assertEqual(PlannerConfig().teammate_avoidance_y, (-0.70, 0.70))
        cases = (
            ("left", -0.2, "right", 1),
            ("right", 0.2, "left", 0),
        )
        for hitter, ball_y, teammate, teammate_index in cases:
            with self.subTest(hitter=hitter):
                planner = RelayPlanner(
                    PlannerConfig(initial_hitter=hitter),
                    strategy="relay_heuristic",
                )
                result = planner.plan(
                    prediction(1, 0.5, 1.0, ball_y),
                    feedback(1.0),
                    1.0,
                )

                self.assertEqual(result.phase, "pre_hit")
                self.assertEqual(result.next_hitter, teammate)
                self.assertEqual(result.diagnostics["teammate_stage_mode"], "outward")
                self.assertAlmostEqual(
                    result.commands[teammate].trajectory_base_position[1],
                    planner.config.teammate_avoidance_y[teammate_index],
                )
                self.assertLess(
                    abs(planner.config.home_y[teammate_index]),
                    abs(result.commands[teammate].trajectory_base_position[1]),
                )
                self.assertLess(
                    abs(result.commands[teammate].trajectory_base_position[1]),
                    abs(planner.config.outward_y[teammate_index]),
                )

    def test_follow_through_keeps_next_hitter_in_clear_lane(self) -> None:
        planner = RelayPlanner(
            PlannerConfig(initial_hitter="left"),
            strategy="relay_heuristic",
        )
        states = feedback(1.0)
        planner.plan(prediction(1, 0.5, 1.0, 0.0), states, 1.0)
        result = planner.plan(prediction(1, -0.1, 1.1, 0.0), feedback(1.1), 1.1)

        self.assertEqual(result.phase, "post_hit")
        self.assertEqual(result.next_hitter, "right")
        self.assertEqual(result.diagnostics["teammate_stage_mode"], "clear_hold")
        self.assertAlmostEqual(
            result.commands["left"].trajectory_base_position[1],
            planner.config.outward_y[0],
        )
        self.assertAlmostEqual(
            result.commands["right"].trajectory_base_position[1],
            planner.config.teammate_avoidance_y[1],
        )

    def test_teammate_avoidance_lane_must_be_outward_and_separated(self) -> None:
        invalid_lanes = (
            (-0.30, 0.65),
            (-0.80, 0.65),
            (-0.20, 0.20),
        )
        for lanes in invalid_lanes:
            with self.subTest(lanes=lanes):
                with self.assertRaises(ValueError):
                    PlannerConfig(teammate_avoidance_y=lanes)

    def test_cbf_command_exposes_trajectory_goal_and_safe_waypoint(self) -> None:
        planner = RelayPlanner(
            PlannerConfig(initial_hitter="left"),
            strategy="cbf_mpc",
        )
        states = feedback(1.0)
        result = planner.plan(prediction(1, 0.5, 1.0, -0.4), states, 1.0)
        command = result.commands[result.next_hitter]

        self.assertNotAlmostEqual(
            command.trajectory_base_position[1],
            command.desired_base_position[1],
        )
        self.assertGreaterEqual(
            command.desired_base_position[1],
            planner.config.workspace_y[0],
        )
        self.assertLessEqual(
            command.desired_base_position[1],
            planner.config.workspace_y[1],
        )
        self.assertAlmostEqual(
            planner.config.cbf_horizon * planner.config.base_position_gain,
            1.0,
        )
        self.assertEqual(result.diagnostics["cbf_controlled_robot"], result.next_hitter)

    def test_stale_prediction_causes_safe_hold(self) -> None:
        planner = RelayPlanner(strategy="cbf_mpc")
        result = planner.plan(prediction(1, 0.5, 0.0), feedback(1.0), 1.0)
        self.assertEqual(result.phase, "safe_hold")
        self.assertIn("stale_prediction", result.fallbacks)
        self.assertFalse(any(command.active for command in result.commands.values()))

    def test_missing_feedback_preserves_legacy_fallback_and_holds_motion(self) -> None:
        planner = RelayPlanner(strategy="cbf_mpc")
        initial = planner.plan(prediction(1, 0.5, 1.0), feedback(1.0), 1.0)
        partial = {"left": RobotFeedback("left", [0.0, -0.3], timestamp=1.1)}
        result = planner.plan(prediction(1, 0.4, 1.1), partial, 1.1)
        self.assertIn("right_feedback_last_feedback", result.fallbacks)
        self.assertTrue(result.diagnostics["feedback_safety_hold"])
        self.assertAlmostEqual(result.commands["right"].desired_base_position[1], 0.35)
        self.assertIsNotNone(initial.hitter)

    def test_ordered_projection_respects_workspace_and_gap(self) -> None:
        left, right, projected = project_ordered_pair(
            0.2,
            -0.2,
            0.61,
            (-0.95, 0.95),
            left_weight=20.0,
            right_weight=1.0,
        )
        self.assertTrue(projected)
        self.assertGreaterEqual(right - left, 0.61 - 1e-12)
        self.assertGreaterEqual(left, -0.95)
        self.assertLessEqual(right, 0.95)

    def test_cbf_waypoint_keeps_safe_set_forward_invariant(self) -> None:
        config = PlannerConfig()
        states = feedback(0.0, left_y=-0.35, right_y=0.35)
        goals = {"left": np.asarray([0.0, 0.2]), "right": np.asarray([0.0, -0.2])}
        waypoints, diagnostics = cbf_waypoints(states, goals, config, hitter="left")
        next_gap = waypoints["right"][1] - waypoints["left"][1]
        self.assertGreaterEqual(next_gap, config.min_separation - 1e-12)
        self.assertTrue(diagnostics["cbf_active"])

    def test_cbf_accounts_for_closing_speed(self) -> None:
        config = PlannerConfig()
        states = {
            "left": RobotFeedback("left", [0.0, -0.35], [0.0, 1.0], timestamp=0.0),
            "right": RobotFeedback("right", [0.0, 0.35], [0.0, -1.0], timestamp=0.0),
        }
        goals = {"left": np.asarray([0.0, 0.2]), "right": np.asarray([0.0, -0.2])}
        _, diagnostics = cbf_waypoints(states, goals, config, hitter="left")
        self.assertGreater(diagnostics["braking_distance"], 0.0)
        self.assertEqual(
            diagnostics["barrier"] + config.min_separation + config.separation_margin,
            0.7,
        )
        self.assertEqual(
            diagnostics["control_barrier_distance"],
            config.min_separation + config.separation_margin,
        )
        self.assertGreater(diagnostics["safe_right_acceleration"], 0.0)
        self.assertLess(diagnostics["safe_left_acceleration"], 0.0)

    def test_phase_locked_cbf_allocates_braking_to_controlled_robot(self) -> None:
        config = PlannerConfig()
        states = {
            "left": RobotFeedback("left", [0.0, -0.25], [0.0, 0.8], timestamp=0.0),
            "right": RobotFeedback("right", [0.0, 0.25], [0.0, -0.8], timestamp=0.0),
        }
        goals = {
            "left": np.asarray([0.0, -0.2]),
            "right": np.asarray([0.0, 0.2]),
        }

        _, diagnostics = cbf_waypoints(
            states,
            goals,
            config,
            hitter="left",
            controlled_robot="left",
        )

        self.assertEqual(diagnostics["safe_right_acceleration"], 0.0)
        self.assertLessEqual(diagnostics["safe_left_acceleration"], 0.0)
        self.assertTrue(diagnostics["cbf_uncontrolled_nominal_suppressed"])

    def test_cbf_emergency_commands_maximum_separation(self) -> None:
        config = PlannerConfig()
        states = {
            "left": RobotFeedback("left", [0.0, -0.31], [0.0, 0.9], timestamp=0.0),
            "right": RobotFeedback("right", [0.0, 0.31], [0.0, -0.9], timestamp=0.0),
        }
        goals = {"left": np.asarray([0.0, 0.1]), "right": np.asarray([0.0, -0.1])}
        waypoints, diagnostics = cbf_waypoints(states, goals, config, hitter="left")
        self.assertTrue(diagnostics["cbf_emergency"])
        self.assertEqual(waypoints["left"][1], config.workspace_y[0])
        self.assertEqual(waypoints["right"][1], config.workspace_y[1])

    def test_phase_locked_emergency_preserves_uncontrolled_hitter(self) -> None:
        config = PlannerConfig()
        states = {
            "left": RobotFeedback("left", [0.0, -0.31], [0.0, 0.9], timestamp=0.0),
            "right": RobotFeedback("right", [0.0, 0.31], [0.0, -0.9], timestamp=0.0),
        }
        goals = {"left": np.asarray([0.0, 0.1]), "right": np.asarray([0.0, -0.1])}

        waypoints, diagnostics = cbf_waypoints(
            states,
            goals,
            config,
            hitter="left",
            controlled_robot="right",
        )

        self.assertTrue(diagnostics["cbf_emergency"])
        self.assertEqual(waypoints["left"][1], states["left"].base_xy[1])
        self.assertEqual(waypoints["right"][1], config.workspace_y[1])

    def test_cbf_name_uses_heuristic_nominal_and_generic_safety_diagnostics(self) -> None:
        planner = RelayPlanner(PlannerConfig(initial_hitter="left"), strategy="cbf")
        result = planner.plan(prediction(1, 0.5, 1.0), feedback(1.0), 1.0)
        self.assertEqual(result.strategy, "cbf")
        self.assertEqual(result.diagnostics["nominal_planner"], "relay_heuristic")
        self.assertEqual(result.diagnostics["safety_filter"], "cbf")
        self.assertIn("safety_active", result.diagnostics)

    def test_reachability_strategy_exposes_viability_diagnostics(self) -> None:
        planner = RelayPlanner(
            PlannerConfig(initial_hitter="left"),
            strategy="reachability",
        )
        result = planner.plan(prediction(1, 0.5, 1.0), feedback(1.0), 1.0)
        self.assertEqual(result.strategy, "reachability")
        self.assertEqual(result.diagnostics["safety_filter"], "reachability")
        self.assertIn("reachability_value", result.diagnostics)

    def test_viability_value_accounts_for_latency_and_stopping_distance(self) -> None:
        value = viability_value(
            gap_m=0.70,
            relative_velocity_mps=-1.0,
            safety_distance_m=0.47,
            maximum_evasive_relative_acceleration_mps2=4.0,
            latency_s=0.08,
        )
        self.assertAlmostEqual(value, 0.025)

    def test_reachability_filter_uses_outer_rail_in_unsafe_state(self) -> None:
        config = PlannerConfig()
        states = {
            "left": RobotFeedback(
                "left", [0.0, -0.20], velocity_xy=[0.0, 0.6], timestamp=0.0
            ),
            "right": RobotFeedback(
                "right", [0.0, 0.20], velocity_xy=[0.0, -0.6], timestamp=0.0
            ),
        }
        goals = {
            "left": np.asarray([0.0, -0.10]),
            "right": np.asarray([0.0, 0.10]),
        }
        waypoints, diagnostics = reachability_waypoints(
            states,
            goals,
            config,
            hitter="left",
        )
        self.assertTrue(diagnostics["reachability_emergency"])
        self.assertEqual(waypoints["left"][1], config.workspace_y[0])
        self.assertEqual(waypoints["right"][1], config.workspace_y[1])

    def test_bounded_projection_matches_brute_force_optimum(self) -> None:
        generator = np.random.default_rng(19)
        grid = np.linspace(-4.0, 4.0, 801)
        for _ in range(40):
            nominal_left, nominal_right = generator.uniform(-5.0, 5.0, size=2)
            lower_bound = float(generator.uniform(-3.0, 7.0))
            left_weight, right_weight = generator.uniform(0.2, 8.0, size=2)
            left, right, _ = _bounded_halfspace_projection(
                nominal_left,
                nominal_right,
                lower_bound,
                4.0,
                left_weight,
                right_weight,
            )
            feasible = grid[None, :] - grid[:, None] >= lower_bound - 1.0e-12
            if not feasible.any():
                feasible = grid[None, :] - grid[:, None] >= 8.0 - 1.0e-12
            costs = (
                left_weight * np.square(grid[:, None] - nominal_left)
                + right_weight * np.square(grid[None, :] - nominal_right)
            )
            brute_cost = float(np.min(costs[feasible]))
            analytic_cost = (
                left_weight * (left - nominal_left) ** 2
                + right_weight * (right - nominal_right) ** 2
            )
            self.assertLessEqual(analytic_cost, brute_cost + 0.02)

    def test_checkpoint_abi_is_not_part_of_planner_config(self) -> None:
        names = set(PlannerConfig.__dataclass_fields__)
        self.assertNotIn("observation_dim", names)
        self.assertNotIn("action_dim", names)


if __name__ == "__main__":
    unittest.main()
