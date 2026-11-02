"""Quaternion operations, Hamilton convention, q = [w, x, y, z] (sec. 2)."""
import numpy as np

def quat_product(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product q1 ⊗ q2 (sec. 2.2).

    Args:
        q1: Left quaternion [w, x, y, z].
        q2: Right quaternion [w, x, y, z].

    Returns:
        q1 ⊗ q2, shape (4,).
    """
    result_w = np.array([q1[0] * q2[0] - q1[1:].T @ q2[1:]],)
    result_v = [q1[0] * q2[1:] + q2[0] * q1[1:] + np.cross(q1[1:], q2[1:])]
    result = np.append(result_w, result_v)
    return result

def rotation_matrix_hamilton(q: np.ndarray) -> np.ndarray:
    """Rotation matrix R(q) of a unit quaternion (sec. 2.3).

    R(q) v = q ⊗ v ⊗ q*: rotates a vector from the local (body) frame to the
    global (NED) frame.

    Args:
        q: Unit quaternion [w, x, y, z].

    Returns:
        R, shape (3, 3).

    Raises:
        ValueError: If ‖q‖ differs from 1 by more than 1e-9.
    """

    if not np.isclose(np.linalg.norm(q), 1.0,atol=1e-9):
        raise ValueError("the quaternion must be unitary")

    R1 = [1 - 2 *  (q[2] ** 2 + q[3] ** 2),
        2 * (q[1] * q[2] - q[0] * q[3]),
        2 * (q[1] * q[3] + q[0] * q[2])]
    R2 = [2 * (q[1] * q[2] + q[0] * q[3]),
        1 - 2 *  (q[1] ** 2 + q[3] ** 2),
        2 * (q[2] * q[3] - q[0] * q[1])]
    R3 = [2 * (q[1] * q[3] - q[0] * q[2]),
        2 * (q[2] * q[3] + q[0] * q[1]),
        1 - 2 *  (q[1] ** 2 + q[2] ** 2)]

    return np.array([R1, R2, R3])

def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate q* = [w, -x, -y, -z]; the inverse rotation for a unit quaternion.

    Args:
        q: Quaternion [w, x, y, z].

    Returns:
        q*, shape (4,).
    """
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quat_normalize(q:np.ndarray) -> np.ndarray:
    """Scale q to unit norm.

    Args:
        q: Non-zero quaternion [w, x, y, z].

    Returns:
        q / ‖q‖, shape (4,).
    """
    return q / np.linalg.norm(q)

def quat_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """Quaternion of a rotation vector, q{θ} = [cos(‖θ‖/2), θ/‖θ‖ sin(‖θ‖/2)] (sec. 2.1).

    Below 1e-8 rad uses the first-order form [1, θ/2], normalized, to avoid
    dividing by ‖θ‖ ≈ 0.

    Args:
        rotvec: Rotation vector θ = angle·axis (rad), shape (3,).

    Returns:
        Unit quaternion [w, x, y, z].
    """
    theta = np.linalg.norm(rotvec)

    if theta < 1e-8:
        return quat_normalize(np.append(1.0, rotvec / 2))

    u = rotvec / theta
    q_w = np.cos(theta / 2)
    q_v = u * np.sin(theta / 2)

    return np.append(q_w, q_v)

def rotvec_from_quat(q: np.ndarray) -> np.ndarray:
    """Rotation vector of a unit quaternion; inverse of quat_from_rotvec.

    q and -q are the same rotation (double cover): q is taken with w >= 0, so the
    angle is in [0, π].

    Args:
        q: Unit quaternion [w, x, y, z].

    Returns:
        Rotation vector θ = angle·axis (rad), shape (3,).
    """

    if q[0] < 0:
        q = -q

    q_v = q[1:]
    s = np.linalg.norm(q_v)

    if s < 1e-12:
        return 2 * q_v

    theta = 2 * np.arctan2(s, q[0])
    u = q_v / s

    return u * theta

def jacobian_q_dtheta(q:np.ndarray) -> np.ndarray:
    """Jacobian of q ⊗ δq{δθ} w.r.t. a local angle error δθ at δθ = 0 (Q_δθ, eq. 47).

    Args:
        q: Nominal quaternion [w, x, y, z].

    Returns:
        Q_δθ, shape (4, 3).
    """
    R1 = [-q[1], -q[2], -q[3]]
    R2 = [q[0], -q[3], q[2]]
    R3 = [q[3], q[0], -q[1]]
    R4 = [-q[2], q[1], q[0]]
    return 0.5 * np.array([R1, R2, R3, R4])

def quat_to_euler(q: np.ndarray) -> np.ndarray:
    """Euler angles of a unit quaternion, ZYX sequence (yaw, then pitch, then roll).

    Pitch is clipped to [-π/2, π/2]; at gimbal lock roll and yaw are not unique.

    Args:
        q: Unit quaternion [w, x, y, z].

    Returns:
        [roll, pitch, yaw] (rad).
    """
    # ZYX (yaw-pitch-roll), returns [roll, pitch, yaw] in rad
    w, x, y, z = q

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))

    return np.array([roll, pitch, yaw])
def jacobian_rotT_vec_q(q: np.ndarray, r: np.ndarray) -> np.ndarray:
    """Jacobian of R(q)ᵀ r w.r.t. the quaternion components (eq. 56).

    Used by the magnetometer (r = mag_ref) and accelerometer (r = g, eq. 63)
    updates. It differentiates the homogeneous form of R(q), so it differs from
    other valid forms only along q itself, which X_δx projects out.

    Args:
        q: Quaternion [w, x, y, z].
        r: Reference vector in the global frame, shape (3,).

    Returns:
        ∂(R(q)ᵀ r)/∂q, shape (3, 4).
    """
    # d(R(q)^T r)/dq, 3x4 (eq. 56; eq. 63 is the case r = [0, 0, g])
    w, x, y, z = q
    rx, ry, rz = r
    R1 = [rx * w + ry * z - rz * y, rx * x + ry * y + rz * z, -rx * y + ry * x - rz * w, -rx * z + ry * w + rz * x]
    R2 = [-rx * z + ry * w + rz * x, rx * y - ry * x + rz * w, rx * x + ry * y + rz * z, -rx * w - ry * z + rz * y]
    R3 = [rx * y - ry * x + rz * w, rx * z - ry * w - rz * x, rx * w + ry * z - rz * y, rx * x + ry * y + rz * z]
    return 2 * np.array([R1, R2, R3])
