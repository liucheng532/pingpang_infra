# Raw ball diagnostic ID — 2026-09-09

The live `/residual/mocap_ball_state` packet on 126 reports rigid ID **30300**.
The diagnostic buffer previously required 30301, so valid incoming samples were
discarded as `wrong_rigid`. The default is now 30300.

`deploy_policy.py` accepts `--raw-ball-rigid-body-id <integer>` when a different
ball ID is configured. The chosen value is saved as `raw_ball_rigid_body_id` in
the recording metadata. On an ID mismatch, the diagnostic `rigid_body_id` field
reports the received ID while `valid` remains false. Wrong rigid bodies, stale
samples, malformed packets and out-of-order frames are still rejected.

This only changes the raw-ball diagnostic subscriber and its argument/metadata
wiring. It does not change the hit teacher, student, RETURN synchronization,
planner or Arm7 residual control path.

The existing V11 launcher needs no new argument: its next policy start uses
30300 automatically. Existing Python policy processes retain the old imported
code until restarted. Deployment does not stop or restart robot control.

Validation: literal 30300 packets produce valid position/velocity samples;
configurable old IDs, wrong-ID reporting, freshness, duplicate rejection and
invalid configuration are covered by tests. An independent read-only subscriber
can validate the new module against the live ROS stream without controlling a
robot.
