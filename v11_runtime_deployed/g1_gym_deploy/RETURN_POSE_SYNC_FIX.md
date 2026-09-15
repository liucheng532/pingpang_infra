# RETURN pose synchronization fix — 2026-09-09

The planner bridge constructs a lateral RETURN target with the freshly measured
pelvis X. Previously the scheduler compared that X against its previous-tick pose
before `update()` refreshed it. Normal X recovery exceeding 0.1 mm per tick could
therefore reject a legal RETURN.

`LCMAgent.observe()` now captures pelvis/torso positions once, synchronizes the
scheduler pose, processes the planner command and advances the scheduler once
using the same positions. `sync_robot_pose()` only copies validated positions;
it does not advance clocks, references, phase transitions or velocity history.
The mirrored scheduler explicitly transforms the pose before synchronizing its
canonical scheduler. Existing `goal_x`, X command clipping, and rejection of
actual non-lateral external targets are retained.

Only three runtime files change: `envs/lcm_agent.py`,
`utils/doubles_reference_scheduler.py`, `utils/mirrored_reference_scheduler.py`.
No policy weights, planner timing settings or raw-ball configuration are changed.

Regression coverage includes both mirrored/canonical robots; signed X changes
of 0.2/1/5 mm; fresh Y for motion selection; preserved nonzero X recovery command;
illegal X rejection; pose-copy isolation; atomic rejection of invalid poses;
one scheduler tick/velocity-history sample per observe; and one HIT reference
advance. The old observe implementation fails the RETURN regression with the
recorded `lateral-Y` error. Tests use fake robot I/O and real bridge/scheduler code;
in a ROS environment the actual imported `LCMAgent.observe` is exercised.

From `g1_gym_deploy`, with ROS Python packages available:

```bash
PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:. python3 -m pytest -q \
  tests/test_return_pose_sync.py tests/test_planner_ros_bridge.py \
  tests/test_runtime_motion_config.py
```

Deployment backs up the original three files and verifies pre/post SHA256 values.
The fix loads on the next policy start; deployment does not start or restart robot
control processes. Real request-to-RETURN latency still requires a subsequent
robot run and is not inferred from these offline tests.
