"""Exercise the production observe method, bridge and scheduler without robot I/O."""
from __future__ import annotations

import ast
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

DEPLOY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEPLOY_ROOT))

from utils.doubles_reference_scheduler import DoublesPhase, DoublesReferenceScheduler
from utils.mirrored_reference_scheduler import MirroredReferenceScheduler
from tests.test_planner_ros_bridge import _bridge, _command
from tests.test_student_deploy import motion_banks


def observe_method():
    if "envs.lcm_agent" in sys.modules:
        return sys.modules["envs.lcm_agent"].LCMAgent.observe
    # Compile the actual method, avoiding hardware-only imports in lcm_agent.py.
    path = DEPLOY_ROOT / "envs/lcm_agent.py"
    module = ast.parse(path.read_text())
    cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == "LCMAgent")
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "observe")
    scope = {"np": np, "DoublesPhase": DoublesPhase}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), scope)
    return scope["observe"]


def setup_scheduler(motion_banks, mirrored):
    hit, move = motion_banks
    canonical = DoublesReferenceScheduler(
        hit, move, external_control=True, v10_relative_x=True,
        reference_end_forces_hold=True, latch_hold_x_on_entry=True,
    )
    scheduler = MirroredReferenceScheduler(canonical) if mirrored else canonical
    sign = -1.0 if mirrored else 1.0
    pelvis = np.array([.18, sign*.8, .75], dtype=np.float32)
    torso = pelvis + np.array([.02, 0., .20], dtype=np.float32)
    scheduler.reset(pelvis, torso)
    canonical._configured_goal_x = .21
    scheduler.bootstrap_outward_hold(sign*.2, outward_y=sign*.8)
    return scheduler, canonical, pelvis, torso


def agent_for(scheduler, pelvis, torso, bridge):
    agent = SimpleNamespace(
        reference_scheduler=scheduler, reference=scheduler.output(),
        pelvis_pos=pelvis.copy(), torso_pos=torso.copy(), planner_bridge=bridge,
        pelvis_rpy=np.zeros(3), pelvis_angular_velocity_w=np.zeros(3),
        recvtarget=np.zeros(3), recvvel=np.zeros(3), dt=.02, timestep=0,
        _update_robot_state=lambda **kwargs: None,
        _apply_pending_runtime_motion_config=lambda: None,
        _corrected_time_to_strike=lambda: -.5,
        _publish_runtime_motion_config_status=lambda **kwargs: None,
    )
    agent.get_obs = lambda refresh_state: agent.reference.target_base.copy()
    if bridge is not None:
        bridge.publish_state = lambda agent: None
    return agent


@pytest.mark.parametrize("mirrored", [False, True])
@pytest.mark.parametrize("dx", [-.005, -.001, -.0002, .0002, .001, .005])
def test_observe_accepts_return_with_fresh_x_and_y(motion_banks, mirrored, dx):
    scheduler, canonical, pelvis, torso = setup_scheduler(motion_banks, mirrored)
    sign = -1 if mirrored else 1
    pelvis += np.array([dx, -sign*.08, 0.], dtype=np.float32)
    torso += np.array([dx, -sign*.08, 0.], dtype=np.float32)
    payload = _command(role="return", active=False, planner_mode="fixed_relay")
    payload["return_target_y"] = sign*.2
    bridge = _bridge(payload)
    agent = agent_for(scheduler, pelvis, torso, bridge)
    clock_before = canonical._clock_s
    samples_before = len(canonical._base_samples)
    with mock.patch.object(canonical, "_external_motion_index", wraps=canonical._external_motion_index) as select:
        result = observe_method()(agent)
    assert bridge._last_error == ""
    assert scheduler.state == DoublesPhase.RETURN
    assert select.call_args.args[0] == pytest.approx(-.52)
    assert canonical.goal_x == pytest.approx(.21)
    assert result[0] == pytest.approx(np.clip(.21-pelvis[0], -.04, .04))
    assert result[1] == pytest.approx(sign*.2-pelvis[1])
    assert canonical._clock_s-clock_before == pytest.approx(.02)
    assert canonical.state_elapsed_s == pytest.approx(.02)
    assert len(canonical._base_samples) == samples_before+1
    assert agent.timestep == 1


@pytest.mark.parametrize("mirrored", [False, True])
def test_sync_only_changes_pose_not_time_reference_or_x_goal(motion_banks, mirrored):
    scheduler, canonical, pelvis, torso = setup_scheduler(motion_banks, mirrored)
    before = (canonical.state, canonical._clock_s, canonical.state_elapsed_s,
              canonical.segment_elapsed_s, canonical.hit_step, canonical.move_step,
              canonical.goal_x, len(canonical._base_samples))
    pelvis[0] += .005
    scheduler.sync_robot_pose(pelvis, torso)
    after = (canonical.state, canonical._clock_s, canonical.state_elapsed_s,
             canonical.segment_elapsed_s, canonical.hit_step, canonical.move_step,
             canonical.goal_x, len(canonical._base_samples))
    assert before == after
    expected = pelvis.copy()
    if mirrored:
        expected[1] *= -1
    np.testing.assert_allclose(canonical._pelvis_position, expected)
    np.testing.assert_allclose(canonical._torso_position, torso*[1, -1 if mirrored else 1, 1])
    pelvis[:] = 123
    np.testing.assert_allclose(canonical._pelvis_position, expected)


@pytest.mark.parametrize("mirrored", [False, True])
def test_illegal_x_request_still_rejected(motion_banks, mirrored):
    scheduler, canonical, pelvis, torso = setup_scheduler(motion_banks, mirrored)
    scheduler.sync_robot_pose(pelvis, torso)
    with pytest.raises(ValueError, match="lateral-Y"):
        scheduler.set_external_base_target(np.array([pelvis[0]+.01, -.2 if mirrored else .2]))
    assert canonical.state == DoublesPhase.OUTWARD_HOLD


@pytest.mark.parametrize("mirrored", [False, True])
def test_hit_reference_advances_once_per_observe(motion_banks, mirrored):
    scheduler, canonical, pelvis, torso = setup_scheduler(motion_banks, mirrored)
    target = np.array([.45, -.2 if mirrored else .2, 1.0])
    velocity = np.array([2., 0., .5])
    scheduler.set_external_hit(target, velocity, .4)
    agent = agent_for(scheduler, pelvis, torso, None)
    agent.recvtarget[:] = target
    agent.recvvel[:] = velocity
    agent._corrected_time_to_strike = lambda: .38
    step_before = canonical.hit_step
    clock_before = canonical._clock_s
    observe_method()(agent)
    assert canonical.hit_step == step_before+1
    assert canonical._clock_s == pytest.approx(clock_before+.02)
    assert canonical.strike_time == pytest.approx(.38)


def test_invalid_sync_is_atomic(motion_banks):
    scheduler, canonical, pelvis, torso = setup_scheduler(motion_banks, False)
    before = canonical._pelvis_position.copy()
    pelvis[0] += .005
    torso[1] = np.nan
    with pytest.raises(ValueError, match="finite"):
        scheduler.sync_robot_pose(pelvis, torso)
    np.testing.assert_array_equal(canonical._pelvis_position, before)
