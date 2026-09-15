import numpy as np

try:
    import torch  # type: ignore
except ImportError:  # Fallback when torch is unavailable
    torch = None

def quaternion_to_rpy(quaternion):
    """
    将四元数转换为RPY角（Roll-Pitch-Yaw）
    
    Args:
        quaternion: 四元数 [x, y, z, w] 格式
    
    Returns:
        rpy: [roll, pitch, yaw] 弧度制，单位：弧度
    """
    x, y, z, w = quaternion
    
    # Roll (绕X轴旋转)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    
    # Pitch (绕Y轴旋转)
    sin_pitch = 2.0 * (w * y - z * x)
    # 限制在[-1, 1]范围内，避免asin域错误
    sin_pitch = np.clip(sin_pitch, -1.0, 1.0)
    pitch = np.arcsin(sin_pitch)
    
    # Yaw (绕Z轴旋转)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    
    return np.array([roll, pitch, yaw])


def matrix_from_quat_wxyz(quaternions):
    """Convert (w, x, y, z) quaternions to rotation matrices using NumPy."""
    q = np.asarray(quaternions, dtype=float)
    if q.shape[-1] != 4:
        raise ValueError("Expected quaternions with shape (..., 4) in (w, x, y, z) order")

    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    norm = np.clip(norm, 1e-9, None)
    q = q / norm

    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    two_s = 2.0

    m00 = 1.0 - two_s * (y * y + z * z)
    m01 = two_s * (x * y - z * w)
    m02 = two_s * (x * z + y * w)

    m10 = two_s * (x * y + z * w)
    m11 = 1.0 - two_s * (x * x + z * z)
    m12 = two_s * (y * z - x * w)

    m20 = two_s * (x * z - y * w)
    m21 = two_s * (y * z + x * w)
    m22 = 1.0 - two_s * (x * x + y * y)

    rot = np.stack(
        [m00, m01, m02, m10, m11, m12, m20, m21, m22],
        axis=-1,
    )
    return rot.reshape(q.shape[:-1] + (3, 3))


def quat_from_rpy_wxyz(roll, pitch, yaw):
    """Compute a (w, x, y, z) quaternion from roll, pitch, yaw (XYZ intrinsic)."""
    hr = 0.5 * roll
    hp = 0.5 * pitch
    hy = 0.5 * yaw

    cr, sr = np.cos(hr), np.sin(hr)
    cp, sp = np.cos(hp), np.sin(hp)
    cy, sy = np.cos(hy), np.sin(hy)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    quat = np.array([w, x, y, z], dtype=float)
    norm = np.linalg.norm(quat)
    return quat / max(norm, 1e-9)

def quat_normalize_xyzw(q):
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


if torch is not None:

    @torch.jit.script
    def matrix_from_quat(quaternions):
        """Convert rotations given as quaternions to rotation matrices."""
        r, i, j, k = torch.unbind(quaternions, -1)
        two_s = 2.0 / (quaternions * quaternions).sum(-1)

        o = torch.stack(
            (
                1 - two_s * (j * j + k * k),
                two_s * (i * j - k * r),
                two_s * (i * k + j * r),
                two_s * (i * j + k * r),
                1 - two_s * (i * i + k * k),
                two_s * (j * k - i * r),
                two_s * (i * k - j * r),
                two_s * (j * k + i * r),
                1 - two_s * (i * i + j * j),
            ),
            -1,
        )
        return o.reshape(quaternions.shape[:-1] + (3, 3))
else:

    def matrix_from_quat(quaternions):
        raise ImportError("PyTorch is required for matrix_from_quat")

def matrix_from_quat_xyzw(quaternions):
    """Convert XYZW quaternions to rotation matrices using NumPy."""
    q = np.asarray(quaternions, dtype=float)
    if q.shape[-1] != 4:
        raise ValueError("Expected quaternions with shape (..., 4)")

    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    norm = np.clip(norm, 1e-9, None)
    q = q / norm

    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    m00 = 1.0 - 2.0 * (yy + zz)
    m01 = 2.0 * (xy - wz)
    m02 = 2.0 * (xz + wy)

    m10 = 2.0 * (xy + wz)
    m11 = 1.0 - 2.0 * (xx + zz)
    m12 = 2.0 * (yz - wx)

    m20 = 2.0 * (xz - wy)
    m21 = 2.0 * (yz + wx)
    m22 = 1.0 - 2.0 * (xx + yy)

    rot = np.stack(
        [m00, m01, m02, m10, m11, m12, m20, m21, m22],
        axis=-1,
    )
    return rot.reshape(q.shape[:-1] + (3, 3))

def axis_after_roll_pitch(roll, pitch):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)

    # new x axis
    x_new = np.array([cp, 0.0, -sp])

    # new y axis
    y_new = np.array([sp*sr, cr, cp*sr])

    return x_new, y_new


def quat_to_rotmat_xyzw(q):
    """q: [x, y, z, w] -> 3x3 R"""
    x, y, z, w = quat_normalize_xyzw(q)
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy+zz), 2*(xy - wz),     2*(xz + wy)],
        [2*(xy + wz),   1 - 2*(xx+zz),   2*(yz - wx)],
        [2*(xz - wy),   2*(yz + wx),     1 - 2*(xx+yy)]
    ], dtype=float)
    return R

def rotmat_to_quat_xyzw(R):
    """3x3 R -> [x, y, z, w]（数值稳定）"""
    R = np.asarray(R, dtype=float)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2,1] - R[1,2]) / s
        y = (R[0,2] - R[2,0]) / s
        z = (R[1,0] - R[0,1]) / s
    else:
        i = np.argmax([R[0,0], R[1,1], R[2,2]])
        if i == 0:
            s = np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2.0
            x = 0.25 * s
            y = (R[0,1] + R[1,0]) / s
            z = (R[0,2] + R[2,0]) / s
            w = (R[2,1] - R[1,2]) / s
        elif i == 1:
            s = np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2.0
            x = (R[0,1] + R[1,0]) / s
            y = 0.25 * s
            z = (R[1,2] + R[2,1]) / s
            w = (R[0,2] - R[2,0]) / s
        else:
            s = np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2.0
            x = (R[0,2] + R[2,0]) / s
            y = (R[1,2] + R[2,1]) / s
            z = 0.25 * s
            w = (R[1,0] - R[0,1]) / s
    return quat_normalize_xyzw([x, y, z, w])

def quat_inv_xyzw(q):
    """单位四元数的逆：共轭"""
    x, y, z, w = quat_normalize_xyzw(q)
    return np.array([-x, -y, -z,  w], dtype=float)

# ------------------ 1) 估计 R<-M 的相对位姿（一次性标定） ------------------

def solve_relative_pose_M_wrt_R(points_R, points_W, T_WM, q_WM_xyzw):
    """
    根据 R/W 对应点求 ^W T_R，再与 ^W T_M 组合得到 ^R T_M。

    Args:
        points_R: (N,3)   N>=4，点在机器人坐标系 R 下
        points_W: (N,3)   对应点在世界坐标系 W 下
        T_WM: (3,)        坐标系 M 在 W 下的平移
        q_WM_xyzw: (4,)   坐标系 M 在 W 下的四元数 [x,y,z,w]

    Returns:
        translation_M: (3,)  M 在 R 下的平移（^R t_M）
        rotation_M: (4,)     M 在 R 下的四元数 [x,y,z,w]（^R R_M）
        also returns (optional) 可自行需要：
            dict 包含 ^W T_R (R_WR, t_WR) 便于验证
    """
    X = np.asarray(points_R, dtype=float)
    Y = np.asarray(points_W, dtype=float)
    assert X.shape == Y.shape and X.shape[1] == 3 and X.shape[0] >= 3

    # Kabsch: 求解 x_W ≈ R_WR x_R + t_WR
    mu_X = X.mean(axis=0)
    mu_Y = Y.mean(axis=0)
    Xc = X - mu_X
    Yc = Y - mu_Y
    H = Xc.T @ Yc
    U, S, Vt = np.linalg.svd(H)
    R_WR = Vt.T @ U.T
    # 处理反射
    if np.linalg.det(R_WR) < 0:
        Vt[-1, :] *= -1
        R_WR = Vt.T @ U.T
    t_WR = mu_Y - R_WR @ mu_X

    # 反求 ^R T_W
    R_RW = R_WR.T
    t_RW = - R_RW @ t_WR

    # ^W T_M
    R_WM = quat_to_rotmat_xyzw(q_WM_xyzw)
    t_WM = np.asarray(T_WM, dtype=float)

    # 组合：^R T_M = ^R T_W * ^W T_M
    R_RM = R_RW @ R_WM
    t_RM = R_RW @ t_WM + t_RW

    q_RM_xyzw = rotmat_to_quat_xyzw(R_RM)

    aux = {"R_WR": R_WR, "t_WR": t_WR, "R_RW": R_RW, "t_RW": t_RW}
    return t_RM, q_RM_xyzw, aux

# ------------------ 2) 在线变换：p_W -> p_R，利用固定的 ^R T_M 和当前 ^W T_M ------------------

def world_point_to_robot_via_M(p_W, T_WM_now, q_WM_now_xyzw, translation_M, rotation_M_xyzw):
    """
    已知：
      - 当前时刻的 ^W T_M（T_WM_now, q_WM_now_xyzw）
      - 事先标定得到的 ^R T_M（translation_M, rotation_M_xyzw）
      - 某点 p_W 在世界坐标系 W 下坐标
    直接输出该点在机器人坐标系 R 下坐标 p_R。

    Args:
        p_W: (3,)
        T_WM_now: (3,)
        q_WM_now_xyzw: (4,)
        translation_M: (3,)       ^R t_M
        rotation_M_xyzw: (4,)     ^R R_M（[x,y,z,w]）

    Returns:
        p_R: (3,)
    """
    p_W = np.asarray(p_W, dtype=float)
    t_WM = np.asarray(T_WM_now, dtype=float)
    q_WM = quat_normalize_xyzw(q_WM_now_xyzw)

    # ^M T_W = (^W T_M)^{-1}
    R_MW = quat_to_rotmat_xyzw(quat_inv_xyzw(q_WM))
    t_MW = - R_MW @ t_WM

    # ^R T_M
    R_RM = quat_to_rotmat_xyzw(rotation_M_xyzw)
    t_RM = np.asarray(translation_M, dtype=float)

    # p_R = ^R T_M * ^M T_W * p_W
    p_M = R_MW @ p_W + t_MW
    p_R = R_RM @ p_M + t_RM
    return p_R