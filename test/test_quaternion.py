"""Tests for the quaternion operations: known values, algebraic properties, and
cross-checks against scipy and finite differences.
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from quaternion import (
    quat_product,
    rotation_matrix_hamilton,
    quat_conjugate,
    quat_normalize,
    quat_from_rotvec,
    rotvec_from_quat,
    jacobian_q_dtheta,
    quat_to_euler,
    jacobian_rotT_vec_q,
)

S2 = np.sqrt(2) / 2
IDENTITY = np.array([1.0, 0, 0, 0])
Q_Z90 = np.array([S2, 0, 0, S2])
Q_Y90 = np.array([S2, 0, S2, 0])
Q_X90 = np.array([S2, S2, 0, 0])
Q_111 = np.array([0.5, 0.5, 0.5, 0.5])


def random_quats(n=50, seed=0):
    """Random unit quaternions, reproducible from the seed."""
    rng = np.random.default_rng(seed)
    return [quat_normalize(rng.normal(size=4)) for _ in range(n)]


# --- quat_product ---

def test_product_integers():
    """Hamilton product of non-unit integer quaternions, both orders.

    q1 ⊗ q2 = [w1·w2 - v1·v2, w1·v2 + w2·v1 + v1 × v2]. Swapping the factors flips the sign of
    v1 × v2, so the vector part changes (the product does not commute) while the scalar part
    (-60) does not.
    """
    q1 = np.array([1, 2, 3, 4])
    q2 = np.array([5, 6, 7, 8])
    np.testing.assert_allclose(quat_product(q1, q2), [-60, 12, 30, 24])
    np.testing.assert_allclose(quat_product(q2, q1), [-60, 20, 14, 32])


def test_product_rotations():
    """Product of two 90° rotations about different axes, both orders.

    Rotations about different axes do not commute: q_z ⊗ q_x is the 120° rotation about
    (1, 1, 1)/√3, while q_x ⊗ q_z is a different one.
    """
    np.testing.assert_allclose(quat_product(Q_Z90, Q_X90), [0.5, 0.5, 0.5, 0.5], atol=1e-12)
    np.testing.assert_allclose(quat_product(Q_X90, Q_Z90), [0.5, 0.5, -0.5, 0.5], atol=1e-12)


@pytest.mark.parametrize("q", random_quats(10))
def test_product_identity(q):
    """The identity quaternion is neutral on both sides.

    With w = 1 and v = 0 the product formula reduces to the other factor.
    """
    np.testing.assert_allclose(quat_product(q, IDENTITY), q)
    np.testing.assert_allclose(quat_product(IDENTITY, q), q)


@pytest.mark.parametrize("q", random_quats(10))
def test_product_inverse(q):
    """q ⊗ q* = identity for a unit quaternion.

    q ⊗ q* = [w² + ‖v‖², w·(-v) + w·v + v × (-v)] = [‖q‖², 0], which is the identity when ‖q‖ = 1:
    the conjugate undoes the rotation.
    """
    np.testing.assert_allclose(quat_product(q, quat_conjugate(q)), IDENTITY, atol=1e-12)


# --- rotation_matrix_hamilton ---

@pytest.mark.parametrize("q, R_expected", [
    (IDENTITY, np.eye(3)),
    (Q_Z90, [[0, -1, 0], [1, 0, 0], [0, 0, 1]]),
    (Q_Y90, [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]),
    (Q_111, [[0, 0, 1], [1, 0, 0], [0, 1, 0]]),
])
def test_rotation_matrix_known(q, R_expected):
    """R(q) for rotations with known matrices.

    The columns of R are the images of the body axes: 90° about z sends x → y and y → -x, 90°
    about y sends z → x, and 120° about (1, 1, 1)/√3 permutes x → y → z → x.
    """
    np.testing.assert_allclose(rotation_matrix_hamilton(q), R_expected, atol=1e-12)


@pytest.mark.parametrize("q", random_quats())
def test_rotation_matrix_is_rotation(q):
    """R(q) is orthogonal with determinant +1.

    R ∈ SO(3): R Rᵀ = I preserves lengths and angles, and det R = +1 rules out a reflection.
    """
    R = rotation_matrix_hamilton(q)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)


@pytest.mark.parametrize("q", random_quats())
def test_rotation_matrix_vs_scipy(q):
    """R(q) matches scipy, an independent implementation.

    scipy orders quaternions scalar-last, [x, y, z, w], so the input is reordered.
    """
    R_scipy = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    np.testing.assert_allclose(rotation_matrix_hamilton(q), R_scipy, atol=1e-12)


def test_rotation_matrix_composition():
    """R(q1 ⊗ q2) = R(q1) R(q2): composition order of the Hamilton convention.

    q1 ⊗ q2 ⊗ v ⊗ q2* ⊗ q1* rotates v by q2 first and then by q1, exactly like the matrix
    product R(q1) R(q2) applied to v.
    """
    q1, q2 = random_quats(2, seed=1)
    np.testing.assert_allclose(
        rotation_matrix_hamilton(quat_normalize(quat_product(q1, q2))),
        rotation_matrix_hamilton(q1) @ rotation_matrix_hamilton(q2),
        atol=1e-12,
    )


@pytest.mark.parametrize("q", random_quats(10))
def test_rotation_matrix_matches_sandwich(q):
    """R(q) v equals the vector part of q ⊗ [0, v] ⊗ q*.

    The sandwich product is the definition of rotating a vector with a quaternion; R(q) is
    just its matrix form, so both must agree for any q and v.
    """
    v = np.array([0.3, -1.2, 2.0])
    rotated = quat_product(quat_product(q, np.append(0.0, v)), quat_conjugate(q))
    np.testing.assert_allclose(rotation_matrix_hamilton(q) @ v, rotated[1:], atol=1e-12)


def test_rotation_matrix_rejects_non_unit():
    """A non-unit quaternion raises ValueError.

    The 1 - 2(y² + z²) form of R(q) is only a rotation when ‖q‖ = 1; for other q it silently
    returns a non-orthogonal matrix that would scale and shear vectors.
    """
    with pytest.raises(ValueError):
        rotation_matrix_hamilton(np.array([1.0, 2, 3, 4]))


# --- quat_conjugate / quat_normalize ---

def test_conjugate():
    """The conjugate negates the vector part.

    q* = [w, -v]: for a unit quaternion it is the inverse, the rotation by the same angle
    about the same axis in the opposite direction.
    """
    np.testing.assert_allclose(quat_conjugate(np.array([1, 2, 3, 4])), [1, -2, -3, -4])


def test_normalize():
    """quat_normalize returns the unit quaternion in the same direction.

    q/‖q‖, with ‖[1, 2, 3, 4]‖ = √30. Scaling does not change the rotation axis or angle,
    only removes the norm error.
    """
    q = quat_normalize(np.array([1.0, 2, 3, 4]))
    assert np.linalg.norm(q) == pytest.approx(1.0)
    np.testing.assert_allclose(q, np.array([1, 2, 3, 4]) / np.sqrt(30))


# --- quat_from_rotvec / rotvec_from_quat ---

def test_from_rotvec_known():
    """π/2 about z gives the 90° yaw quaternion.

    q{θ} = [cos(θ/2), u·sin(θ/2)]: quaternions use the half angle, so 90° gives
    [cos 45°, 0, 0, sin 45°] = [√2/2, 0, 0, √2/2].
    """
    np.testing.assert_allclose(quat_from_rotvec(np.array([0, 0, np.pi / 2])), Q_Z90, atol=1e-12)


def test_from_rotvec_zero():
    """A zero rotation vector gives the identity without NaN.

    The axis u = θ/‖θ‖ is undefined at θ = 0. Since sin(θ/2)/θ → 1/2, the small-angle branch
    uses q ≈ [1, θ/2], which is exact in the limit.
    """
    q = quat_from_rotvec(np.zeros(3))
    assert not np.any(np.isnan(q))
    np.testing.assert_allclose(q, IDENTITY)


def test_rotvec_from_identity():
    """The identity gives a zero rotation vector without NaN.

    With v = 0 the axis v/‖v‖ is undefined; the limit of θ = 2·u·atan2(‖v‖, w) is 2v = 0.
    """
    rv = rotvec_from_quat(IDENTITY)
    assert not np.any(np.isnan(rv))
    np.testing.assert_allclose(rv, np.zeros(3))


@pytest.mark.parametrize("theta", [
    np.zeros(3),
    np.array([1e-10, -2e-10, 3e-10]),
    np.array([0.01, -0.02, 0.005]),
    np.array([0, 0, np.pi / 2]),
    np.array([0.3, -1.1, 0.7]),
    np.array([0, 0, np.pi - 1e-6]),
])
def test_rotvec_roundtrip(theta):
    """rotvec → quaternion → rotvec returns the input, from 0 rad to almost π.

    The inverse uses θ = 2·atan2(‖v‖, w), accurate over the whole range: acos(w) would lose
    precision near 0 and asin(‖v‖) near π. The cases cover the small-angle branch (1e-10 rad)
    and the edge before the double cover flips the axis (π - 1e-6).
    """
    q = quat_from_rotvec(theta)
    assert np.linalg.norm(q) == pytest.approx(1.0)
    np.testing.assert_allclose(rotvec_from_quat(q), theta, atol=1e-12)


@pytest.mark.parametrize("q", random_quats(10))
def test_rotvec_double_cover(q):
    """q and -q give the same rotation vector.

    R(q) is quadratic in q, so q and -q are the same rotation (double cover). Taking w >= 0
    picks the representative with angle in [0, π].
    """
    np.testing.assert_allclose(rotvec_from_quat(-q), rotvec_from_quat(q), atol=1e-12)


@pytest.mark.parametrize("q", random_quats(10))
def test_rotvec_vs_scipy(q):
    """rotvec_from_quat matches scipy, an independent implementation.

    Both return the angle in [0, π] for the same rotation.
    """
    rv_scipy = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_rotvec()
    np.testing.assert_allclose(rotvec_from_quat(q), rv_scipy, atol=1e-12)


# --- jacobian_q_dtheta ---

@pytest.mark.parametrize("q", random_quats(10))
def test_jacobian_finite_differences(q):
    """jacobian_q_dtheta matches finite differences of q ⊗ q{δθ} at δθ = 0.

    For small δθ, q{δθ} ≈ [1, δθ/2], so q ⊗ q{δθ} ≈ q + ½·q ⊗ [0, δθ]: linear in δθ, with matrix
    Q_δθ (eq. 47).
    """
    eps = 1e-7
    J_num = np.zeros((4, 3))
    for i in range(3):
        d = np.zeros(3)
        d[i] = eps
        J_num[:, i] = (quat_product(q, quat_from_rotvec(d)) - q) / eps
    np.testing.assert_allclose(jacobian_q_dtheta(q), J_num, atol=1e-6)


# --- quat_to_euler ---

@pytest.mark.parametrize("q, euler_expected", [
    (IDENTITY, [0, 0, 0]),
    (Q_X90, [np.pi / 2, 0, 0]),
    (quat_from_rotvec(np.array([0, 0.5, 0])), [0, 0.5, 0]),
    (Q_Z90, [0, 0, np.pi / 2]),
])
def test_euler_known(q, euler_expected):
    """Euler angles of pure roll, pitch and yaw rotations.

    ZYX convention: R = Rz(yaw)·Ry(pitch)·Rx(roll). A rotation about a single axis must give
    that angle and zero for the other two.
    """
    np.testing.assert_allclose(quat_to_euler(q), euler_expected, atol=1e-12)


def test_euler_gimbal_lock():
    """At 90° of pitch roll and yaw are not unique: only pitch is checked, and no NaN.

    With pitch = 90° the body x axis is vertical, so roll and yaw rotate about the same axis
    and only their difference is defined. arcsin is clipped because rounding can push its
    argument slightly above 1.
    """
    euler = quat_to_euler(Q_Y90)
    assert not np.any(np.isnan(euler))
    assert euler[1] == pytest.approx(np.pi / 2)


@pytest.mark.parametrize("q", random_quats())
def test_euler_vs_scipy(q):
    """quat_to_euler matches scipy's intrinsic ZYX, reordered to [roll, pitch, yaw].

    Intrinsic ZYX applies yaw, then pitch about the new y, then roll about the new x; scipy
    returns them in that order, [yaw, pitch, roll].
    """
    euler_scipy = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_euler("ZYX")[::-1]
    np.testing.assert_allclose(quat_to_euler(q), euler_scipy, atol=1e-9)


# --- jacobian_rotT_vec_q ---

@pytest.mark.parametrize("q", random_quats(10))
def test_jacobian_rotT_vec_finite_differences(q):
    """jacobian_rotT_vec_q matches finite differences of R(q)ᵀ r.

    q + ε is not unit, so R(q)ᵀ r is computed with the homogeneous formula, valid for any q,
    which is the form eq. 56 differentiates.
    """
    def rotT_vec(q, r):
        w, x, y, z = q
        R = np.array([
            [w*w + x*x - y*y - z*z, 2 * (x*y - w*z), 2 * (x*z + w*y)],
            [2 * (x*y + w*z), w*w - x*x + y*y - z*z, 2 * (y*z - w*x)],
            [2 * (x*z - w*y), 2 * (y*z + w*x), w*w - x*x - y*y + z*z],
        ])
        return R.T @ r

    r = np.array([26.0, -0.5, 37.0])
    eps = 1e-7
    J_num = np.zeros((3, 4))
    for i in range(4):
        dq = np.zeros(4)
        dq[i] = eps
        J_num[:, i] = (rotT_vec(q + dq, r) - rotT_vec(q, r)) / eps
    np.testing.assert_allclose(jacobian_rotT_vec_q(q, r), J_num, atol=1e-4)
