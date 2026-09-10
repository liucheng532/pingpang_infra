from __future__ import annotations

import numpy as np

from doubles_planner import (
    ACTIVE_STUDENT_PROFILE,
    REAL_PLANNER_NAMES,
    V6_PHASE_ORDER,
    V6_STUDENT_PROFILE,
    V9_STUDENT_PROFILE,
    PlannerConfig,
    make_real_robot_planner,
    make_real_robot_planners,
)
from doubles_planner.rl import empty_policy_arrays, load_q_policy, save_q_policy


def test_active_delivery_profile_is_v6_six_phase_1666_abi() -> None:
    assert ACTIVE_STUDENT_PROFILE == V6_STUDENT_PROFILE
    assert V6_STUDENT_PROFILE.observation_dim == 1666
    assert V6_STUDENT_PROFILE.action_dim == 29
    assert V6_STUDENT_PROFILE.phase_order == V6_PHASE_ORDER
    assert V9_STUDENT_PROFILE.phase_order == V6_PHASE_ORDER


def test_factory_constructs_all_real_robot_planners() -> None:
    planners = make_real_robot_planners(PlannerConfig())

    assert tuple(planners) == REAL_PLANNER_NAMES
    assert tuple(planner.strategy.name for planner in planners.values()) == (
        "relay_heuristic",
        "cbf",
        "reachability",
        "rl",
    )


def test_factory_accepts_q_policy_without_environment_lookup(tmp_path) -> None:
    q_values, state_visits = empty_policy_arrays()
    state_visits[(0, 0, 0, 0, 0, 0)] = 1
    policy_path = save_q_policy(
        tmp_path / "rl_policy.npz",
        q_values,
        state_visits,
        {"minimum_deployment_visits": 1},
    )
    policy = load_q_policy(policy_path)
    planner = make_real_robot_planner("rl", q_policy=policy)

    assert planner.strategy.name == "rl"
    assert np.isfinite(policy.q_values).all()


def test_factory_rejects_legacy_mpc_as_real_delivery_strategy() -> None:
    try:
        make_real_robot_planner("cbf_mpc")
    except ValueError as error:
        assert "heuristic" in str(error)
    else:
        raise AssertionError("legacy cbf_mpc must not be a real-robot delivery name")
