from __future__ import annotations

import unittest


class IsaacRewardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        try:
            import torch
        except ImportError as error:
            raise unittest.SkipTest("torch is required") from error
        cls.torch = torch

    def _reward(self, **overrides):
        from doubles_planner.isaac_rl import compute_isaac_relay_reward

        torch = self.torch
        values = {
            "strike_mask": torch.tensor([True, True]),
            "position_error_m": torch.tensor([0.0, 0.0]),
            "velocity_error_mps": torch.tensor([0.0, 0.0]),
            "orientation_error_rad": torch.tensor([0.0, 0.0]),
            "timing_error_s": torch.tensor([0.0, 0.0]),
            "base_distance_m": torch.tensor([1.0, 1.0]),
            "hand_distance_m": torch.tensor([1.0, 1.0]),
            "racket_distance_m": torch.tensor([1.0, 1.0]),
            "base_ttc_s": torch.tensor([10.0, 10.0]),
            "hand_ttc_s": torch.tensor([10.0, 10.0]),
            "racket_ttc_s": torch.tensor([10.0, 10.0]),
            "simultaneous_hit": torch.tensor([False, False]),
            "phase_interruption": torch.tensor([False, False]),
            "safety_projected": torch.tensor([False, False]),
            "masked_commit": torch.tensor([False, False]),
            "stale_input": torch.tensor([False, False]),
            "commit_started": torch.tensor([False, False]),
            "peer_clear_at_commit": torch.tensor([False, False]),
            "ready_at_commit": torch.tensor([False, False]),
            "handoff_completed": torch.tensor([False, False]),
            "timeout": torch.tensor([False, False]),
        }
        values.update(overrides)
        return compute_isaac_relay_reward(**values)

    def test_strike_quality_preserves_original_accuracy_ordering(self) -> None:
        torch = self.torch
        reward, terms = self._reward(
            position_error_m=torch.tensor([0.0, 0.20]),
            velocity_error_mps=torch.tensor([0.0, 2.0]),
            orientation_error_rad=torch.tensor([0.0, 0.50]),
            timing_error_s=torch.tensor([0.0, 0.20]),
        )
        self.assertGreater(float(reward[0]), float(reward[1]))
        self.assertGreater(float(terms["strike_position_quality"][0]), float(terms["strike_position_quality"][1]))

    def test_clearance_and_ttc_penalize_closing_pairs(self) -> None:
        torch = self.torch
        reward, terms = self._reward(
            base_distance_m=torch.tensor([1.0, 0.30]),
            hand_distance_m=torch.tensor([1.0, 0.10]),
            racket_distance_m=torch.tensor([1.0, 0.10]),
            base_ttc_s=torch.tensor([10.0, 0.05]),
            hand_ttc_s=torch.tensor([10.0, 0.02]),
            racket_ttc_s=torch.tensor([10.0, 0.02]),
        )
        self.assertLess(float(reward[1]), float(reward[0]))
        for name in ("base_clearance", "hand_clearance", "racket_clearance", "base_ttc", "hand_ttc", "racket_ttc"):
            self.assertLess(float(terms[name][1]), float(terms[name][0]))

    def test_simultaneous_hit_penalty_dominates_commit_bonus(self) -> None:
        torch = self.torch
        reward, _ = self._reward(
            simultaneous_hit=torch.tensor([False, True]),
            commit_started=torch.tensor([True, True]),
            peer_clear_at_commit=torch.tensor([True, True]),
            ready_at_commit=torch.tensor([True, True]),
        )
        self.assertLess(float(reward[1]), float(reward[0]) - 20.0)

    def test_reward_rejects_inconsistent_batch_shapes(self) -> None:
        torch = self.torch
        with self.assertRaises(ValueError):
            self._reward(base_distance_m=torch.tensor([1.0, 1.0, 1.0]))


if __name__ == "__main__":
    unittest.main()
