# Doubles Student 1666 deployment

This entry point deploys only the frozen six-state student. It does not load either teacher.

## Required bundle

- `student_m73000_hold6_1666_model23000.onnx` and its required `.onnx.json` sidecar
- hit reference directory `0302_combined` (50 Hz, strike frame 27)
- move reference directory `0718-move-160-80hz` (155 motions at 80 Hz)

Export the fixed `model_23000.pt` checkpoint:

```bash
python scripts/export_student_onnx.py \
  --checkpoint /path/to/model_23000.pt \
  --output-dir /path/to/deploy_bundle
```

The exporter enforces the checkpoint, teacher, phase, and input ABI hashes, copies the source
checkpoint to an immutable iteration-named file, and verifies PyTorch/ONNX action parity.

## Shadow validation

Run from `g1_gym_deploy/`:

```bash
python scripts/deploy_policy.py \
  --policy /home/unitree/haoran/doubles-student-1666-deploy/policy/student_m73000_hold6_1666_model23000.onnx \
  --hit-motion-data /home/unitree/haoran/doubles-student-1666-deploy/data/0302_combined \
  --move-motion-data /home/unitree/haoran/doubles-student-1666-deploy/data/0718-move-160-80hz \
  --shadow
```

Shadow mode reads LCM/ROS sensors and runs the scheduler and ONNX at 50 Hz, but it skips
calibration and never publishes `pd_plustau_targets`.

Add `--record-dir /home/unitree/haoran/doubles-student-1666-deploy/logs/records` to enable the
non-blocking recorder. It records the six-state phase, HOLD timing/reason, exact policy input,
reference, action, `q_des`, robot state, and receive timestamps in a separate writer process.

## Observation contract

The current frame is 166 history dimensions plus 6 current soft-phase dimensions. Ten frames
are stored independently per term, oldest to newest, and flattened term-major:

```text
strike        [0:10]
target_vel    [10:40]
racket        [40:70]
target_base   [70:90]
task_anchor   [90:120]
orientation   [120:180]
base_ang_vel  [180:210]
joint_pos     [210:500]
joint_vel     [500:790]
actions       [790:1080]
command       [1080:1660]
phase         [1660:1666]
```

Phase order is `HIT/POST_DELAY/OUTWARD/OUTWARD_HOLD/RETURN/HOME_HOLD`. POST_DELAY is 0.20 s.
OUTWARD enters HOLD after 0.10 s of training-threshold stability or when the outbound reference
ends. The deployment HOLD is fixed at 1.00 s and then transitions directly to RETURN. Only
HOME_HOLD accepts a new hit.

The deploy process reconstructs the pelvis world pose at 50 Hz from `/torso_pose_origin` and
Gym-order waist yaw/roll/pitch joints 12/13/14. `task_anchor` is
`[pelvis_x, pelvis_y, torso_z]`; target-base, home locking, movement distance, and stability use
pelvis XY. HIT motion selection uses torso position. Reset is rejected until both torso and joint
state have been received.

The active ChingMu inputs remain `/predicted_ball_position`, `/predicted_racket_velocity`,
`/predicted_ball_predict_time`, and `/torso_pose_origin`. This deployment does not use or modify
the ZMQ camera backend.
