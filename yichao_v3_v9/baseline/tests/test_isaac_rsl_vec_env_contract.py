from __future__ import annotations

import pytest


torch = pytest.importorskip("torch")
pytest.importorskip("tensordict")
pytest.importorskip("rsl_rl")

from doubles_planner.isaac_rl import IsaacPPORunnerConfig, make_rsl_rl_vec_env


class _FakeIsaacOuterEnv:
    def __init__(self) -> None:
        self.num_envs = 2
        self.num_actions = 12
        self.max_episode_length = 17
        self.device = torch.device("cpu")
        self.episode_length_buf = torch.zeros(2, dtype=torch.long)
        self.closed = False
        self._observation = torch.arange(2 * 5, dtype=torch.float32).reshape(2, 5)

    def _build_observation(self):
        return self._observation.clone()

    def step(self, actions):
        assert tuple(actions.shape) == (2, 12)
        self.episode_length_buf += 1
        observations = self._observation + 1.0
        rewards = torch.tensor([1.0, -1.0])
        dones = torch.tensor([False, True])
        return observations, rewards, dones, {
            "time_outs": torch.tensor([False, True]),
            "reward_terms": {"base_clearance": torch.tensor([0.5, 0.25])},
            "commit_started": torch.tensor([True, False]),
            "simultaneous_hit": torch.tensor([False, False]),
            "masked_commit": torch.tensor([False, True]),
            "safety_projected": torch.tensor([False, True]),
        }

    def close(self):
        self.closed = True


def test_vec_env_exposes_tensordict_and_logging_contract() -> None:
    outer = _FakeIsaacOuterEnv()
    vector_env = make_rsl_rl_vec_env(outer, device="cpu")
    observations = vector_env.get_observations()
    assert tuple(observations.batch_size) == (2,)
    assert tuple(observations["policy"].shape) == (2, 5)

    next_observations, rewards, dones, extras = vector_env.step(torch.zeros((2, 12)))
    assert tuple(next_observations.batch_size) == (2,)
    assert tuple(next_observations["policy"].shape) == (2, 5)
    assert rewards.shape == (2,)
    assert dones.dtype == torch.bool
    assert extras["time_outs"].tolist() == [False, True]
    assert "/reward/base_clearance" in extras["log"]
    assert extras["log"]["/relay/commit_rate"].item() == 0.5
    assert extras["log"]["/relay/masked_commit_rate"].item() == 0.5
    assert vector_env.episode_length_buf.tolist() == [1, 1]

    vector_env.close()
    assert outer.closed


def test_rsl_runner_constructs_from_the_adapter_contract() -> None:
    from rsl_rl.runners import OnPolicyRunner

    outer = _FakeIsaacOuterEnv()
    vector_env = make_rsl_rl_vec_env(outer, device="cpu")
    config = IsaacPPORunnerConfig(
        num_steps_per_env=2,
        max_iterations=1,
        actor_hidden_dims=(16,),
        critic_hidden_dims=(16,),
    ).to_dict()
    config["algorithm"]["num_mini_batches"] = 2
    runner = OnPolicyRunner(vector_env, config, log_dir=None, device="cpu")
    assert runner.env.num_actions == 12
    assert runner.env.cfg["action_version"] == "doubles-centralized-v2"
    vector_env.close()
