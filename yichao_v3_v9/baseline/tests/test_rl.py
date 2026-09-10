from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from doubles_planner.config import PlannerConfig
from doubles_planner.models import BallPrediction, RobotFeedback
from doubles_planner.rl import (
    BALL_Y_EDGES,
    BASE_Y_EDGES,
    POLICY_HASH_KEY,
    RELATIVE_VELOCITY_EDGES,
    TEAMMATE_GOALS_Y,
    TIME_TO_STRIKE_EDGES,
    FrozenQPolicy,
    empty_policy_arrays,
    encode_state,
    load_q_policy,
    save_q_policy,
)
from doubles_planner.strategies import (
    RLStrategy,
    RelayHeuristicStrategy,
    StrategyContext,
)
def _context(
    *,
    ball_y: float = 0.0,
    time_to_strike: float = 0.50,
    left_y: float = -0.35,
    right_y: float = 0.35,
    phase: str | None = None,
) -> StrategyContext:
    config = PlannerConfig(initial_hitter="left")
    feedback = {
        "left": RobotFeedback("left", [0.0, left_y], [0.0, 0.0], timestamp=1.0),
        "right": RobotFeedback(
            "right", [0.0, right_y], [0.0, 0.0], timestamp=1.0
        ),
    }
    prediction = BallPrediction(
        position=[0.45, ball_y, 1.0],
        velocity=[-3.5, 0.0, -0.3],
        time_to_strike=time_to_strike,
        timestamp=1.0,
        racket_normal=[0.93, 0.0, 0.36],
        racket_velocity=[1.89, 0.0, 0.73],
        shot_id=1,
    )
    return StrategyContext(
        config=config,
        phase=phase or ("pre_hit" if time_to_strike >= 0.0 else "post_hit"),
        hitter="left",
        next_hitter="right",
        prediction=prediction,
        feedback=feedback,
        previous_goals={name: state.base_xy for name, state in feedback.items()},
    )


def _policy(
    covered_actions: list[tuple[StrategyContext, int]] | None = None,
) -> FrozenQPolicy:
    q_values, state_visits = empty_policy_arrays()
    for context, action_index in covered_actions or []:
        left = context.feedback["left"]
        right = context.feedback["right"]
        state = encode_state(
            0,
            float(context.prediction.position[1]),
            float(left.base_xy[1]),
            float(right.base_xy[1]),
            float(context.prediction.time_to_strike),
            float(right.velocity_xy[1] - left.velocity_xy[1]),
        )
        state_visits[state] = 10
        q_values[state + (action_index,)] = 5.0
    return FrozenQPolicy(
        q_values=q_values,
        state_visits=state_visits,
        action_goals_y=TEAMMATE_GOALS_Y,
        ball_edges=BALL_Y_EDGES,
        base_edges=BASE_Y_EDGES,
        time_edges=TIME_TO_STRIKE_EDGES,
        relative_velocity_edges=RELATIVE_VELOCITY_EDGES,
        metadata={"algorithm": "test"},
    )


class FrozenQStrategyTests(unittest.TestCase):
    def test_covered_state_uses_highest_value_action(self) -> None:
        context = _context()
        action_index = 7
        decision = RLStrategy(_policy([(context, action_index)])).decide(context)

        self.assertTrue(decision.diagnostics["rl_state_covered"])
        self.assertEqual(decision.diagnostics["rl_action_index"], action_index)
        self.assertAlmostEqual(
            decision.goals["right"][1],
            float(TEAMMATE_GOALS_Y[action_index]),
        )
        self.assertNotIn("rl_unseen_state", decision.fallbacks)

    def test_uncovered_state_falls_back_to_relay_heuristic(self) -> None:
        context = _context()
        expected = RelayHeuristicStrategy().decide(context)
        decision = RLStrategy(_policy()).decide(context)

        self.assertFalse(decision.diagnostics["rl_state_covered"])
        self.assertIn("rl_unseen_state", decision.fallbacks)
        for name in context.config.robots:
            np.testing.assert_allclose(decision.goals[name], expected.goals[name])

    def test_runtime_fallback_switches_from_avoidance_to_preview(self) -> None:
        pre_hit = _context(ball_y=0.20, time_to_strike=0.50)
        post_hit = _context(ball_y=0.20, time_to_strike=-0.10)
        strategy = RLStrategy(_policy())

        pre_hit_decision = strategy.decide(pre_hit)
        post_hit_decision = strategy.decide(post_hit)

        self.assertEqual(pre_hit_decision.diagnostics["teammate_stage_mode"], "outward")
        self.assertAlmostEqual(
            pre_hit_decision.goals["right"][1],
            pre_hit.config.teammate_avoidance_y[1],
        )
        self.assertEqual(post_hit_decision.diagnostics["teammate_stage_mode"], "preview")
        np.testing.assert_allclose(
            post_hit_decision.goals["right"],
            RelayHeuristicStrategy().decide(post_hit).goals["right"],
        )

    def test_policy_metadata_sets_deployment_coverage_threshold(self) -> None:
        context = _context()
        policy = _policy([(context, 7)])
        policy.metadata["minimum_deployment_visits"] = 11
        decision = RLStrategy(policy).decide(context)

        self.assertEqual(decision.diagnostics["rl_state_visits"], 10)
        self.assertEqual(decision.diagnostics["rl_minimum_visits"], 11)
        self.assertFalse(decision.diagnostics["rl_state_covered"])
        self.assertIn("rl_unseen_state", decision.fallbacks)

    def test_action_is_cached_for_shot_and_reset_clears_cache(self) -> None:
        first = _context(ball_y=0.0, time_to_strike=0.50)
        later = _context(ball_y=0.72, time_to_strike=0.38)
        strategy = RLStrategy(_policy([(first, 7), (later, 1)]))

        initial = strategy.decide(first)
        cached = strategy.decide(later)
        strategy.reset()
        after_reset = strategy.decide(later)

        self.assertEqual(initial.diagnostics["rl_action_index"], 7)
        self.assertEqual(cached.diagnostics["rl_action_index"], 7)
        self.assertEqual(after_reset.diagnostics["rl_action_index"], 1)


class QPolicyArtifactTests(unittest.TestCase):
    def test_saved_policy_has_validated_semantic_hash(self) -> None:
        q_values, state_visits = empty_policy_arrays()
        state_visits[(0, 0, 0, 0, 0, 0)] = 1
        q_values[(0, 0, 0, 0, 0, 0, 2)] = 1.25
        with tempfile.TemporaryDirectory() as directory:
            path = save_q_policy(
                Path(directory) / "policy.npz",
                q_values,
                state_visits,
                {"algorithm": "test", "seed": 10000},
            )
            loaded = load_q_policy(path)

        self.assertEqual(len(loaded.metadata[POLICY_HASH_KEY]), 64)
        np.testing.assert_array_equal(loaded.q_values, q_values)
        np.testing.assert_array_equal(loaded.state_visits, state_visits)

    def test_loader_rejects_hashless_policy(self) -> None:
        q_values, state_visits = empty_policy_arrays()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hashless.npz"
            np.savez_compressed(
                path,
                q_values=q_values,
                state_visits=state_visits,
                action_goals_y=TEAMMATE_GOALS_Y,
                ball_edges=BALL_Y_EDGES,
                base_edges=BASE_Y_EDGES,
                time_edges=TIME_TO_STRIKE_EDGES,
                relative_velocity_edges=RELATIVE_VELOCITY_EDGES,
                metadata_json=np.asarray('{"algorithm": "legacy"}'),
            )
            with self.assertRaisesRegex(ValueError, "missing required policy_sha256"):
                load_q_policy(path)

    def test_loader_rejects_policy_with_tampered_values(self) -> None:
        q_values, state_visits = empty_policy_arrays()
        with tempfile.TemporaryDirectory() as directory:
            path = save_q_policy(
                Path(directory) / "policy.npz",
                q_values,
                state_visits,
                {"algorithm": "test"},
            )
            with np.load(path, allow_pickle=False) as stored:
                payload = {name: stored[name].copy() for name in stored.files}
            payload["q_values"].flat[0] = 1.0
            np.savez_compressed(path, **payload)

            with self.assertRaisesRegex(ValueError, "content hash mismatch"):
                load_q_policy(path)

if __name__ == "__main__":
    unittest.main()
