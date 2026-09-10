from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest


torch = pytest.importorskip("torch")

from doubles_planner.isaac_rl import IsaacDoublesHighLevelEnv, IsaacDoublesPPOConfig


class _Policy:
    def eval(self):
        return self

    def requires_grad_(self, value):
        return self

    def __call__(self, observations):
        return torch.zeros((observations.shape[0], 29), device=observations.device)


class _Scene(SimpleNamespace):
    def __getitem__(self, key):
        raise KeyError(key)


class _Robot:
    def __init__(self, local_y, origins):
        num_envs = origins.shape[0]
        positions = torch.zeros((num_envs, 4, 3))
        positions[:, :, 0] = origins[:, 0, None]
        positions[:, :, 1] = origins[:, 1, None] + local_y
        positions[:, :, 2] = origins[:, 2, None] + 0.8
        positions[:, 1, 1] += 0.10
        positions[:, 2, 1] -= 0.10
        positions[:, 3, 1] += 0.05
        self.data = SimpleNamespace(
            body_pos_w=positions,
            body_lin_vel_w=torch.zeros_like(positions),
            body_ang_vel_w=torch.zeros_like(positions),
            body_quat_w=torch.tensor([1.0, 0.0, 0.0, 0.0]).repeat(num_envs, 4, 1),
            joint_pos=torch.zeros((num_envs, 29)),
            joint_vel=torch.zeros((num_envs, 29)),
            projected_gravity_b=torch.tensor([0.0, 0.0, -1.0]).repeat(num_envs, 1),
        )


class _Command:
    HIT, POST_DELAY, OUTWARD, OUTWARD_HOLD, RETURN, HOME_HOLD = range(6)

    def __init__(self, num_envs, home_y):
        outward_y = 0.78 if home_y > 0.0 else -0.78
        self.cfg = SimpleNamespace(
            outward_target_y=outward_y,
            external_target_deadband_m=0.01,
            post_delay_range_s=(0.20, 0.20),
        )
        return_reference = -0.42 if home_y > 0.0 else 0.42
        outward_reference = -return_reference
        self.motion_targets_t = torch.tensor(
            [[0.0, return_reference, 0.0], [0.0, outward_reference, 0.0]]
        )
        self.motion_labels_t = torch.tensor([False, True])
        self._motion_pelvis_displacement_y = torch.tensor([return_reference, outward_reference])
        self._outbound_motion_indices = torch.tensor([0])
        self._return_motion_indices = torch.tensor([1])
        self.doubles_state = torch.full((num_envs,), self.HOME_HOLD, dtype=torch.long)
        self.state_elapsed_s = torch.zeros(num_envs)
        self.stable_elapsed_s = torch.ones(num_envs)
        self.home_y = torch.full((num_envs,), home_y)
        self.target_y = torch.full((num_envs,), home_y)
        self.home_target_y = torch.full((num_envs,), home_y)
        self.move_distance = torch.zeros(num_envs)
        self.outbound_motion_index = torch.ones(num_envs, dtype=torch.long)
        self.return_motion_index = torch.zeros(num_envs, dtype=torch.long)
        self.outbound_y_scale = torch.zeros(num_envs)
        self.return_y_scale = torch.zeros(num_envs)
        self.pending_outward_target_y = torch.full((num_envs,), outward_y)
        self.pending_outward_target_valid = torch.zeros(num_envs, dtype=torch.bool)
        self.hit = SimpleNamespace(
            time_to_strike_s=torch.ones(num_envs),
            racket_target=torch.zeros((num_envs, 3)),
            target_velocity=torch.zeros((num_envs, 3)),
            ball_velocity=torch.zeros((num_envs, 3)),
        )
        self.base_calls = []
        self.hit_calls = []
        self.outward_calls = []

    def _handoff_ready(self):
        return torch.ones_like(self.doubles_state, dtype=torch.bool)

    def clear_external_control(self, env_ids):
        return None

    def set_external_base_target(self, *, env_ids, target_xy, return_target_y, safety_override):
        self.base_calls.append((env_ids.clone(), target_xy.clone(), return_target_y.clone(), safety_override))
        self.target_y[env_ids] = target_xy[:, 1]
        self.home_target_y[env_ids] = return_target_y
        self.doubles_state[env_ids] = self.OUTWARD

    def set_external_hit(self, *, env_ids, racket_target, target_velocity, time_to_strike_s, ball_velocity):
        self.hit_calls.append((env_ids.clone(), racket_target.clone(), target_velocity.clone(), time_to_strike_s.clone(), ball_velocity.clone()))
        self.hit.racket_target[env_ids] = racket_target
        self.hit.target_velocity[env_ids] = target_velocity
        self.hit.ball_velocity[env_ids] = ball_velocity
        self.hit.time_to_strike_s[env_ids] = time_to_strike_s
        self.pending_outward_target_valid[env_ids] = False
        self.doubles_state[env_ids] = self.HIT

    def set_external_outward_target(self, *, env_ids, target_y):
        self.outward_calls.append((env_ids.clone(), target_y.clone()))
        self.pending_outward_target_y[env_ids] = target_y
        self.pending_outward_target_valid[env_ids] = True


class _Underlying:
    def __init__(self, num_envs):
        self.num_envs = num_envs
        self.device = "cpu"
        self.step_dt = 0.02
        self.scene = _Scene(env_origins=torch.tensor([[0.0, 0.0, 0.0], [4.0, 7.0, 0.0]]))
        self.reset_ids = []

    def reset(self, *, env_ids):
        self.reset_ids.append(env_ids.clone())
        return {}, {}


class _Env:
    def __init__(self, num_envs):
        self.unwrapped = _Underlying(num_envs)
        self.last_actions = None
        self.action_history = []

    def step(self, actions):
        self.last_actions = actions.clone()
        self.action_history.append(actions.clone())
        num_envs = actions.shape[0]
        observations = {
            "policy": torch.zeros((num_envs, 1666)),
            "policy_left": torch.zeros((num_envs, 1666)),
        }
        done = torch.zeros(num_envs, dtype=torch.bool)
        return observations, torch.zeros(num_envs), done, done.clone(), {}

    def close(self):
        return None


@pytest.fixture
def high_level_env():
    num_envs = 2
    env = _Env(num_envs)
    origins = env.unwrapped.scene.env_origins
    commands = {"left": _Command(num_envs, -0.35), "right": _Command(num_envs, 0.35)}
    robots = {"left": _Robot(-0.35, origins), "right": _Robot(0.35, origins)}
    terms = {
        "left": SimpleNamespace(_offset=torch.zeros(29), _scale=torch.ones(29)),
        "right": SimpleNamespace(_offset=torch.zeros(29), _scale=torch.ones(29)),
    }
    wrapped = IsaacDoublesHighLevelEnv(
        env,
        _Policy(),
        config=IsaacDoublesPPOConfig(num_envs=num_envs, warmup_steps=0, device="cpu"),
        commands=commands,
        robots=robots,
        action_terms=terms,
        joint_names=tuple(f"joint_{index}" for index in range(29)),
        body_indices={
            "left": {"pelvis": 0, "hands": (1, 2), "racket": 3},
            "right": {"pelvis": 0, "hands": (1, 2), "racket": 3},
        },
        mirror_observation=lambda observation, *args: observation,
        mirror_action=lambda action, *args: action,
        distill_hit_errors=lambda command: (torch.zeros(num_envs), torch.zeros(num_envs), torch.zeros(num_envs)),
        auto_reset=False,
    )
    wrapped.stage[:] = wrapped._PREPARED
    wrapped.time_to_strike[:] = 0.40
    return wrapped


def _hit_action(left=False, right=False):
    action = torch.zeros((2, 12))
    if left:
        action[:, 4] = 10.0
    if right:
        action[:, 10] = 10.0
    return action


def test_commit_starts_one_hit_and_peer_clear_in_same_tick(high_level_env):
    events = high_level_env._apply_high_level_action(_hit_action(left=True))
    left = high_level_env.commands["left"]
    right = high_level_env.commands["right"]
    assert events["commit_started"].tolist() == [True, False]
    assert events["peer_clear_at_commit"].tolist() == [True, False]
    assert len(left.hit_calls) == 1
    assert len(left.outward_calls) == 1
    assert len(right.base_calls) == 1
    assert torch.allclose(left.hit_calls[0][2], torch.tensor([[1.9, 0.0, 0.7]]))
    assert high_level_env.stage.tolist() == [high_level_env._COMMITTED, high_level_env._PREPARED]


def test_simultaneous_hit_is_masked_before_controller_hooks(high_level_env):
    events = high_level_env._apply_high_level_action(_hit_action(left=True, right=True))
    assert events["simultaneous_hit_request"].all()
    assert not events["simultaneous_hit"].any()
    assert not events["commit_started"].any()
    assert not high_level_env.commands["left"].hit_calls
    assert not high_level_env.commands["right"].hit_calls


def test_prepare_targets_stay_on_each_robot_outward_corridor(high_level_env):
    action = torch.zeros((2, 12))
    action[:, 0] = 1.0
    action[:, 6] = -1.0
    action[:, 2] = 10.0
    action[:, 8] = 10.0
    high_level_env.stage[:] = high_level_env._RESERVED
    events = high_level_env._apply_high_level_action(action)
    assert not events["stale_input"].any()
    left_target = high_level_env.commands["left"].base_calls[-1][1][:, 1]
    right_target = high_level_env.commands["right"].base_calls[-1][1][:, 1]
    assert torch.all(left_target <= torch.tensor(-0.35))
    assert torch.all(right_target >= torch.tensor(0.35))


def test_stage_machine_reaches_handoff_without_changing_hitter(high_level_env):
    high_level_env.active_hitter[:] = 0
    high_level_env.stage[:] = high_level_env._COMMITTED
    high_level_env.commands["left"].doubles_state[:] = high_level_env.commands["left"].POST_DELAY
    high_level_env._advance_stage()
    assert (high_level_env.stage == high_level_env._STRIKE).all()
    high_level_env.commands["left"].doubles_state[:] = high_level_env.commands["left"].OUTWARD
    high_level_env._advance_stage()
    assert (high_level_env.stage == high_level_env._FOLLOW_THROUGH).all()
    high_level_env.commands["left"].doubles_state[:] = high_level_env.commands["left"].HOME_HOLD
    high_level_env.commands["right"].doubles_state[:] = high_level_env.commands["right"].HOME_HOLD
    high_level_env._advance_stage()
    assert (high_level_env.stage == high_level_env._HANDOFF).all()
    assert (high_level_env.active_hitter == 0).all()


def test_v9_can_handoff_and_commit_during_locomotion(high_level_env):
    high_level_env.config = replace(high_level_env.config, locomotion_preemption=True)
    high_level_env.commands["left"].doubles_state[:] = high_level_env.commands["left"].RETURN
    high_level_env.commands["right"].doubles_state[:] = high_level_env.commands["right"].OUTWARD_HOLD
    high_level_env.stage[:] = high_level_env._FOLLOW_THROUGH

    high_level_env._advance_stage()

    assert (high_level_env.stage == high_level_env._HANDOFF).all()
    high_level_env.stage[:] = high_level_env._PREPARED
    events = high_level_env._apply_high_level_action(_hit_action(left=True))
    assert events["commit_started"].tolist() == [True, False]
    assert high_level_env.commands["left"].doubles_state.tolist() == [0, 4]


def test_v6_still_waits_for_home_before_commit(high_level_env):
    high_level_env.commands["left"].doubles_state[:] = high_level_env.commands["left"].RETURN
    high_level_env.commands["right"].doubles_state[:] = high_level_env.commands["right"].OUTWARD_HOLD

    events = high_level_env._apply_high_level_action(_hit_action(left=True))

    assert not events["commit_started"].any()
    assert not high_level_env.commands["left"].hit_calls


def test_observation_is_origin_relative_and_finite(high_level_env):
    observation = high_level_env._build_observation()
    left_y = high_level_env._name_to_index["left_base_position_y"]
    right_y = high_level_env._name_to_index["right_base_position_y"]
    assert observation.shape == (2, high_level_env.observation_size)
    assert torch.isfinite(observation).all()
    assert torch.allclose(observation[:, left_y], torch.tensor([-0.35, -0.35]))
    assert torch.allclose(observation[:, right_y], torch.tensor([0.35, 0.35]))


def test_return_guard_projects_overshot_targets_to_the_inner_side(high_level_env):
    left = high_level_env.commands["left"]
    right = high_level_env.commands["right"]
    left.doubles_state[:] = left.OUTWARD_HOLD
    right.doubles_state[:] = right.OUTWARD_HOLD
    left.home_target_y[:] = -0.45
    right.home_target_y[:] = 0.45

    high_level_env._guard_controller_return_targets()

    assert torch.allclose(left.home_target_y, torch.full_like(left.home_target_y, -0.35))
    assert torch.allclose(right.home_target_y, torch.full_like(right.home_target_y, 0.35))


def test_return_guard_does_not_shift_home_during_initial_outward_motion(high_level_env):
    left = high_level_env.commands["left"]
    right = high_level_env.commands["right"]
    left.doubles_state[:] = left.OUTWARD
    right.doubles_state[:] = right.OUTWARD
    left.target_y[:] = -0.78
    right.target_y[:] = 0.78
    left.home_target_y[:] = -0.35
    right.home_target_y[:] = 0.35

    high_level_env._guard_controller_return_targets()

    assert torch.allclose(left.home_target_y, torch.full_like(left.home_target_y, -0.35))
    assert torch.allclose(right.home_target_y, torch.full_like(right.home_target_y, 0.35))


def test_pending_outward_guard_projects_each_robot_to_physical_outward_side(high_level_env):
    left = high_level_env.commands["left"]
    right = high_level_env.commands["right"]
    left.pending_outward_target_valid[:] = True
    right.pending_outward_target_valid[:] = True
    left.pending_outward_target_y[:] = 0.10
    right.pending_outward_target_y[:] = 0.60

    high_level_env._guard_controller_pending_outward_targets()

    assert torch.allclose(left.pending_outward_target_y, torch.full_like(left.pending_outward_target_y, -0.78))
    assert torch.allclose(right.pending_outward_target_y, torch.full_like(right.pending_outward_target_y, 0.60))


def test_locomotion_preempted_hits_queue_outward_targets_for_both_mirrors(high_level_env):
    high_level_env.config = replace(high_level_env.config, locomotion_preemption=True)
    origins = high_level_env.unwrapped.scene.env_origins
    high_level_env.robots["left"].data.body_pos_w[0, :, 1] = origins[0, 1] - 0.84
    high_level_env.robots["right"].data.body_pos_w[1, :, 1] = origins[1, 1] + 0.84
    for command in high_level_env.commands.values():
        command.doubles_state[:] = command.RETURN
    action = torch.zeros((2, 12))
    action[0, 4] = 10.0
    action[1, 10] = 10.0

    events = high_level_env._apply_high_level_action(action)

    assert events["commit_started"].all()
    left_target = high_level_env.commands["left"].outward_calls[-1][1]
    right_target = high_level_env.commands["right"].outward_calls[-1][1]
    assert left_target.item() < -0.84
    assert right_target.item() > 0.84


def test_hit_commit_rejects_insufficient_outward_runway(high_level_env):
    origins = high_level_env.unwrapped.scene.env_origins
    high_level_env.robots["right"].data.body_pos_w[0, :, 1] = origins[0, 1] + 0.93

    with pytest.raises(ValueError, match="insufficient outward runway"):
        high_level_env._physical_outward_target(1, torch.tensor([0.93]))


def test_outward_projection_checks_actual_motion_displacement(high_level_env):
    left = high_level_env.commands["left"]
    left._motion_pelvis_displacement_y[1] = 0.42

    with pytest.raises(ValueError, match="insufficient outward runway"):
        high_level_env._physical_outward_target(0, torch.tensor([-0.35]))


def test_same_phase_retarget_failure_restores_controller_fields(high_level_env):
    right = high_level_env.commands["right"]
    right.doubles_state[0] = right.OUTWARD
    original = {
        name: getattr(right, name).clone()
        for name in ("home_y", "target_y", "home_target_y", "move_distance")
    }

    def fail_after_partial_mutation(**arguments):
        ids = arguments["env_ids"]
        right.target_y[ids] = -0.10
        right.home_target_y[ids] = -0.20
        right.move_distance[ids] = 9.0
        raise ValueError("External base target would reverse the selected motion reference")

    right.set_external_base_target = fail_after_partial_mutation

    with pytest.raises(ValueError, match="would reverse"):
        high_level_env._command_base_target(
            1,
            torch.tensor([0]),
            torch.tensor([0.78]),
            torch.tensor([0.35]),
        )

    for name, value in original.items():
        torch.testing.assert_close(getattr(right, name), value)


def test_infeasible_pending_outward_synthetically_truncates_only_affected_env(high_level_env):
    class _OnesPolicy:
        def __call__(self, observations):
            return torch.ones((observations.shape[0], 29))

    high_level_env.policy = _OnesPolicy()
    right = high_level_env.commands["right"]
    origins = high_level_env.unwrapped.scene.env_origins
    high_level_env.robots["right"].data.body_pos_w[1, :, 1] = origins[1, 1] + 0.94
    right.pending_outward_target_valid[1] = True
    observations = {
        "policy": torch.zeros((2, 1666)),
        "policy_left": torch.zeros((2, 1666)),
    }

    _observations, terminated, truncated, _info = high_level_env._low_level_step(observations)

    assert not terminated.any()
    assert truncated.tolist() == [False, True]
    torch.testing.assert_close(high_level_env.env.last_actions[0], torch.ones(58))
    torch.testing.assert_close(high_level_env.env.last_actions[1], torch.zeros(58))
    assert high_level_env.unwrapped.reset_ids[-1].tolist() == [1]
    assert not right.pending_outward_target_valid[1]


def test_synthetic_truncation_resets_episode_once_after_transition(high_level_env):
    class _OnesPolicy:
        def __call__(self, observations):
            return torch.ones((observations.shape[0], 29))

    high_level_env.policy = _OnesPolicy()
    high_level_env._last_observations = {
        "policy": torch.zeros((2, 1666)),
        "policy_left": torch.zeros((2, 1666)),
    }
    high_level_env.episode_length_buf[1] = 7
    high_level_env.episode_return[1] = 3.0
    previous_shot_id = high_level_env.shot_id.clone()
    right = high_level_env.commands["right"]
    origins = high_level_env.unwrapped.scene.env_origins
    high_level_env.robots["right"].data.body_pos_w[1, :, 1] = origins[1, 1] + 0.94
    right.pending_outward_target_valid[1] = True
    original_reset = high_level_env.unwrapped.reset

    def reset_to_controller_default(*, env_ids):
        result = original_reset(env_ids=env_ids)
        for command in high_level_env.commands.values():
            command.doubles_state[env_ids] = command.HIT
        return result

    high_level_env.unwrapped.reset = reset_to_controller_default

    _observation, reward, done, info = high_level_env.step(torch.zeros((2, 12)))

    assert done.tolist() == [False, True]
    assert info["episode_length"][1].item() == 8
    torch.testing.assert_close(info["episode_reward"][1], reward[1] + 3.0)
    assert info["base_distance_m"].shape == (2,)
    assert info["hand_distance_m"].shape == (2,)
    assert info["racket_distance_m"].shape == (2,)
    assert high_level_env.shot_id[1].item() == previous_shot_id[1].item() + 1
    assert high_level_env.episode_length_buf[1].item() == 0
    assert len(high_level_env.unwrapped.reset_ids) == 1
    assert high_level_env.unwrapped.reset_ids[0].tolist() == [1]
    assert high_level_env.commands["left"].doubles_state[1].item() == high_level_env.commands["left"].HOME_HOLD
    assert high_level_env.commands["right"].doubles_state[1].item() == high_level_env.commands["right"].HOME_HOLD
    assert len(high_level_env.env.action_history) == high_level_env.config.low_level_steps
    torch.testing.assert_close(high_level_env.env.action_history[0][0], torch.ones(58))
    torch.testing.assert_close(high_level_env.env.action_history[0][1], torch.zeros(58))
    torch.testing.assert_close(high_level_env.env.action_history[1][0], torch.ones(58))
    torch.testing.assert_close(high_level_env.env.action_history[1][1], torch.zeros(58))


def test_failed_commit_restores_both_controller_states(high_level_env):
    ids = torch.tensor([0])
    hitter = high_level_env.commands["left"]
    peer = high_level_env.commands["right"]
    before = {
        "hitter": high_level_env._snapshot_command_state(hitter, ids),
        "peer": high_level_env._snapshot_command_state(peer, ids),
    }

    def fail_outward_target(*, env_ids, target_y):
        hitter.pending_outward_target_y[env_ids] = target_y
        hitter.pending_outward_target_valid[env_ids] = True
        raise ValueError("injected outward target failure")

    hitter.set_external_outward_target = fail_outward_target

    assert not high_level_env._commit_one(0, 0)
    assert high_level_env.active_hitter[0].item() == -1
    assert high_level_env.stage[0].item() == high_level_env._PREPARED
    for label, command in (("hitter", hitter), ("peer", peer)):
        for scope, fields in before[label].items():
            owner = command if scope == "command" else getattr(command, scope)
            for name, value in fields.items():
                torch.testing.assert_close(getattr(owner, name)[ids], value)


def test_hit_skill_mask_rejects_insufficient_runway(high_level_env):
    origins = high_level_env.unwrapped.scene.env_origins
    high_level_env.next_hitter[0] = 1
    high_level_env.robots["right"].data.body_pos_w[0, :, 1] = origins[0, 1] + 0.94

    mask = high_level_env._skill_mask()
    action = torch.zeros((2, 12))
    action[0, 10] = 10.0
    events = high_level_env._apply_high_level_action(action)

    assert not mask[0, 1, 3]
    assert not events["commit_started"][0]
    assert not high_level_env.commands["right"].hit_calls


def test_hit_skill_mask_rejects_nonfinite_outward_velocity(high_level_env):
    high_level_env.robots["left"].data.body_lin_vel_w[0, 0, 1] = float("nan")

    mask = high_level_env._skill_mask()

    assert not mask[0, 0, 3]


@pytest.mark.parametrize(
    ("robust_readiness", "expected_post_delay"),
    ((True, 0.12), (False, 0.80)),
)
def test_commit_runway_horizon_uses_active_post_delay_mode(
    high_level_env,
    monkeypatch,
    robust_readiness,
    expected_post_delay,
):
    hitter = high_level_env.commands["left"]
    hitter.cfg.robust_handoff_readiness = robust_readiness
    hitter.cfg.readiness_min_post_delay_s = 0.12
    hitter.cfg.post_delay_range_s = (0.20, 0.80)
    high_level_env.time_to_strike[0] = 0.40
    captured = {}

    def capture_runway(_robot_index, current, *, outward_speed, transition_horizon_s):
        captured["horizon"] = transition_horizon_s.clone()
        return current - 0.10

    monkeypatch.setattr(high_level_env, "_physical_outward_target", capture_runway)

    assert high_level_env._commit_one(0, 0)
    torch.testing.assert_close(
        captured["horizon"],
        torch.tensor([0.40 + expected_post_delay]),
    )
