from __future__ import annotations

import numpy as np

from utils.joint_mapping import FROM_GYM_TO_LAB, LAB_JOINT_NAMES
from utils.left_right_mirror import mirror_joint_values


ARM7 = (12, 16, 20, 22, 24, 26, 28)
ORIGINS_XYZ = np.asarray(
    (
        (0.0039563, -0.10021, 0.24778),
        (0.0, -0.038, -0.013831),
        (0.0, -0.00624, -0.1032),
        (0.015783, 0.0, -0.080518),
        (0.100, -0.00188791, -0.010),
        (0.038, 0.0, 0.0),
        (0.046, 0.0, 0.0),
    ),
    dtype=np.float64,
)
ORIGINS_RPY = np.asarray(
    ((-0.27931, 5.4949e-05, 0.00019159), (0.27925, 0.0, 0.0), *((0.0, 0.0, 0.0),) * 5),
    dtype=np.float64,
)
AXES = np.asarray(
    ((0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
    dtype=np.float64,
)
PALM_XYZ = np.asarray((0.041, 0.0, 0.0), dtype=np.float64)
RACKET_XYZ = np.asarray((0.12305, 0.023806, 0.13196), dtype=np.float64)
RACKET_RPY = np.asarray((-0.058906, 0.52058, -0.11806), dtype=np.float64)
MIRROR = np.diag((1.0, -1.0, 1.0))
RACKET_HALF_X = 0.085
RACKET_HALF_Z = 0.085
RACKET_FACE_Z_MIN = -0.070
CONTACT_PLANE_BAND = 0.02 + 0.00725 + 0.006


def rpy_matrix(rpy):
    roll, pitch, yaw = np.asarray(rpy, dtype=np.float64)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.asarray(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        ),
        dtype=np.float64,
    )


def axis_matrix(axis, angle):
    x, y, z = np.asarray(axis, dtype=np.float64) / np.linalg.norm(axis)
    cosine, sine = np.cos(angle), np.sin(angle)
    one = 1.0 - cosine
    return np.asarray(
        (
            (cosine + x * x * one, x * y * one - z * sine, x * z * one + y * sine),
            (y * x * one + z * sine, cosine + y * y * one, y * z * one - x * sine),
            (z * x * one - y * sine, z * y * one + x * sine, cosine + z * z * one),
        ),
        dtype=np.float64,
    )


def right_racket_pose(q_lab):
    rotation = np.eye(3, dtype=np.float64)
    position = np.zeros(3, dtype=np.float64)
    for index, joint_index in enumerate(ARM7):
        position += rotation @ ORIGINS_XYZ[index]
        rotation = rotation @ rpy_matrix(ORIGINS_RPY[index]) @ axis_matrix(AXES[index], q_lab[joint_index])
    position += rotation @ PALM_XYZ
    position += rotation @ RACKET_XYZ
    rotation = rotation @ rpy_matrix(RACKET_RPY)
    return position, rotation


def orientation_from_six(values):
    first_two = np.asarray(values, dtype=np.float64).reshape(3, 2)
    return np.column_stack((first_two, np.cross(first_two[:, 0], first_two[:, 1])))


def canonical_racket_pose(frame):
    q_lab = np.asarray(frame["joint_pos"], dtype=np.float64)[list(FROM_GYM_TO_LAB)]
    mirrored = bool(frame["mirror_left_hand"])
    if mirrored:
        q_lab = np.asarray(mirror_joint_values(q_lab, LAB_JOINT_NAMES), dtype=np.float64)
    relative_position, relative_rotation = right_racket_pose(q_lab)
    observation = np.asarray(frame["teacher_policy_obs_history"], dtype=np.float64)
    torso_position = observation[127:130]
    torso_rotation = orientation_from_six(observation[184:190])
    return (
        torso_position + torso_rotation @ relative_position,
        torso_rotation @ relative_rotation,
    )


def canonical_ball_state(frame):
    age_s = max(
        0.0,
        (int(frame["monotonic_time_ns"]) - int(frame["raw_ball_receive_monotonic_ns"])) * 1.0e-9,
    )
    position = np.asarray(frame["raw_ball_position"], dtype=np.float64)
    velocity = np.asarray(frame["raw_ball_velocity"], dtype=np.float64)
    position = position + velocity * min(age_s, 0.03)
    if bool(frame["mirror_left_hand"]):
        position[1] *= -1.0
        velocity[1] *= -1.0
    return position, velocity


def _finite_velocity(positions, times):
    velocity = np.zeros_like(positions)
    if len(positions) < 2:
        return velocity
    delta_t = np.diff(times)
    valid = delta_t > 1.0e-6
    indices = np.flatnonzero(valid) + 1
    velocity[indices] = np.diff(positions, axis=0)[valid] / delta_t[valid, None]
    velocity[0] = velocity[1]
    return velocity


def analyze_hit_frames(frames, strike_plane_x=0.45):
    frames = np.asarray(frames)
    mask = np.isin(frames["phase_id"], (0, 1))
    frames = frames[mask]
    if not len(frames):
        raise ValueError("Clip contains no HIT/POST_DELAY frames")
    valid = np.asarray(frames["raw_ball_valid"], dtype=bool)
    valid &= np.isfinite(np.asarray(frames["raw_ball_position"])).all(axis=1)
    valid &= np.isfinite(np.asarray(frames["raw_ball_velocity"])).all(axis=1)
    if not np.any(valid):
        return {"classification": "raw_ball_missing", "valid_fraction": 0.0}

    racket_position = []
    racket_rotation = []
    ball_position = []
    ball_velocity = []
    for frame in frames:
        position, rotation = canonical_racket_pose(frame)
        racket_position.append(position)
        racket_rotation.append(rotation)
        ball_pos, ball_vel = canonical_ball_state(frame)
        ball_position.append(ball_pos)
        ball_velocity.append(ball_vel)
    racket_position = np.asarray(racket_position)
    racket_rotation = np.asarray(racket_rotation)
    ball_position = np.asarray(ball_position)
    ball_velocity = np.asarray(ball_velocity)
    times = np.asarray(frames["monotonic_time_ns"], dtype=np.float64) * 1.0e-9
    racket_velocity = _finite_velocity(racket_position, times)
    tts = np.asarray(frames["corrected_tts"], dtype=np.float64)

    delta = ball_position - racket_position
    local = np.einsum("nji,nj->ni", racket_rotation, delta)
    center_distance = np.linalg.norm(delta, axis=1)
    center_distance[~valid] = np.inf
    closest = int(np.argmin(center_distance))
    crossings = []
    for index in range(1, len(frames)):
        if not valid[index - 1] or not valid[index]:
            continue
        previous_normal = local[index - 1, 1]
        current_normal = local[index, 1]
        if previous_normal * current_normal > 0.0 or abs(current_normal - previous_normal) <= 1.0e-9:
            continue
        alpha = -previous_normal / (current_normal - previous_normal)
        crossing_local = local[index - 1] + alpha * (local[index] - local[index - 1])
        crossing_tts = tts[index - 1] + alpha * (tts[index] - tts[index - 1])
        in_plane_radius = float(np.linalg.norm(crossing_local[[0, 2]]))
        if abs(crossing_tts) <= 0.12 and in_plane_radius <= 0.30:
            crossings.append((in_plane_radius, index, alpha, crossing_local, crossing_tts))
    if crossings:
        _, plane_index, plane_alpha, local_plane, plane_tts = min(crossings, key=lambda value: value[0])
    else:
        plane_index = closest
        plane_alpha = 1.0
        local_plane = local[closest]
        plane_tts = tts[closest]
    ellipse_value = (local_plane[0] / RACKET_HALF_X) ** 2 + (local_plane[2] / RACKET_HALF_Z) ** 2
    inside_face = ellipse_value <= 1.0 and local_plane[2] >= RACKET_FACE_Z_MIN
    geometric_contact = inside_face and abs(local_plane[1]) <= CONTACT_PLANE_BAND

    plane_crossing = None
    for index in range(1, len(frames)):
        x0, x1 = ball_position[index - 1, 0], ball_position[index, 0]
        if (x0 - strike_plane_x) * (x1 - strike_plane_x) <= 0.0 and abs(x1 - x0) > 1.0e-6:
            alpha = (strike_plane_x - x0) / (x1 - x0)
            crossing = ball_position[index - 1] + alpha * (ball_position[index] - ball_position[index - 1])
            plane_crossing = crossing.tolist()
            break

    closest_tts = float(tts[closest])
    closest_local = local[closest]
    if geometric_contact:
        classification = "geometric_contact"
    elif abs(closest_local[0]) > RACKET_HALF_X:
        classification = "lateral_miss"
    elif closest_local[2] < RACKET_FACE_Z_MIN or abs(closest_local[2]) > RACKET_HALF_Z:
        classification = "vertical_miss"
    elif closest_tts > 0.01:
        classification = "timing_early"
    elif closest_tts < -0.01:
        classification = "timing_late"
    else:
        classification = "normal_gap"

    target = np.asarray(frames[closest]["obs"], dtype=np.float64)[4:7]
    if bool(frames[closest]["mirror_left_hand"]):
        target[1] *= -1.0
    return {
        "classification": classification,
        "valid_fraction": float(np.mean(valid)),
        "closest_index": closest,
        "closest_tts_s": closest_tts,
        "center_distance_m": float(center_distance[closest]),
        "delta_canonical_m": delta[closest].tolist(),
        "local_delta_m": local[closest].tolist(),
        "plane_index": plane_index,
        "plane_alpha": float(plane_alpha),
        "plane_crossing_tts_s": float(plane_tts),
        "plane_local_delta_m": local_plane.tolist(),
        "ellipse_value": float(ellipse_value),
        "inside_face": bool(inside_face),
        "geometric_contact": bool(geometric_contact),
        "racket_position_canonical_m": racket_position[closest].tolist(),
        "racket_velocity_canonical_mps": racket_velocity[closest].tolist(),
        "ball_position_canonical_m": ball_position[closest].tolist(),
        "ball_velocity_canonical_mps": ball_velocity[closest].tolist(),
        "target_canonical_m": target.tolist(),
        "ball_strike_plane_position_canonical_m": plane_crossing,
    }
