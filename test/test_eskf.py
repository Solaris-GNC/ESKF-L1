"""Tests for the ESKF: propagation, covariance, measurement updates and observability.

Most physical scenarios put the true vehicle at rest and level (p = v = 0, q = 1), start
the estimate with an error in one state, and check that the filter corrects what the
sensor observes and leaves alone what it does not.
"""
import numpy as np
import pytest
from pytest import approx
from eskf import ESKF
from linalg_utils import skew
from quaternion import (quat_to_euler, quat_product, quat_conjugate, quat_from_rotvec, rotvec_from_quat,
                        rotation_matrix_hamilton, jacobian_rotT_vec_q)

A_REST = np.array([0, 0, -9.81])


def make_eskf(sigma_a=0.0, sigma_w=0.0, sigma_aw=0.0, sigma_ww=0.0):
    """ESKF with the given process noise, all zero by default (deterministic propagation)."""
    return ESKF(sigma_a=sigma_a, sigma_w=sigma_w, sigma_aw=sigma_aw, sigma_ww=sigma_ww)


# nominal propagation 

def test_rest():
    """Level IMU at rest: position, velocity and attitude stay constant.

    At rest the accelerometer reads the gravity reaction g = [0, 0, -9.81], so with q = 1
    v̇ = R(q)·a_m - g = 0 (eq. 19), and ω_m = 0 leaves q unchanged (eq. 24).
    """
    eskf = make_eskf()
    dt = 0.01
    T = 10

    for _ in range(round(T/dt)):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=dt)

    assert eskf.p == approx(np.zeros(3))
    assert eskf.v == approx(np.zeros(3))
    assert eskf.q == approx((1.0, 0.0, 0.0, 0.0))

def test_rotation():
    """A constant yaw rate of 0.5 rad/s for 1 s gives 0.5 rad of yaw.

    Each step composes q ⊗ q{ω·dt} (eq. 24). Rotations about a fixed axis commute, so N steps
    give exactly q{ω·N·dt}: there is no integration error to tolerate.
    """
    eskf = make_eskf()
    w_meas = np.array([0, 0, 0.5])
    dt = 0.01
    T = 1

    for _ in range(round(T/dt)):
        eskf._predict(a_meas=A_REST, w_meas=w_meas, dt=dt)

    assert quat_to_euler(eskf.q) == approx((0.0, 0.0, 0.5))

def test_quat_norm():
    """The quaternion stays unit after 10 000 steps of arbitrary rotation.

    The product of unit quaternions is unit in exact arithmetic, but rounding makes ‖q‖ drift
    a little every step; the renormalization of eq. 25 removes it. A non-unit q would no
    longer be a rotation (R(q) stops being orthogonal).
    """
    eskf = make_eskf()
    w_meas = np.array([0.3, -0.2, 0.5])
    dt = 0.01
    T = 100

    for _ in range(round(T/dt)):
        eskf._predict(a_meas=A_REST, w_meas=w_meas, dt=dt)

    assert np.linalg.norm(eskf.q) == approx(1.0)

def test_rest_tilted():
    """A tilted IMU at rest stays still.

    At rest the accelerometer reads Rᵀg for any attitude, so R Rᵀg - g = 0. Fixes the sign
    of g and the direction of R with q ≠ identity (test_rest only checks q = 1, where R = Rᵀ).
    """
    eskf = make_eskf()
    eskf.q = quat_from_rotvec(np.array([0.4, -0.3, 1.0]))
    q0 = eskf.q.copy()
    a_meas = rotation_matrix_hamilton(eskf.q).T @ eskf.g

    for _ in range(1000):
        eskf._predict(a_meas=a_meas, w_meas=np.zeros(3), dt=0.01)

    assert eskf.p == approx(np.zeros(3), abs=1e-10)
    assert eskf.v == approx(np.zeros(3), abs=1e-10)
    assert eskf.q == approx(q0)

def test_constant_acceleration():
    """Constant acceleration integrates with forward Euler (eq. 21-22).

    The accelerometer reads a plus the gravity reaction. v_N = a·N·dt exactly and
    p_N = Σ v_k·dt = a·dt²·N(N-1)/2: 0.495 m, not the continuous 0.5 m, for 1 m/s² over 1 s.
    """
    a = np.array([1.0, -0.5, 2.0])
    dt, N = 0.01, 100
    eskf = make_eskf()

    for _ in range(N):
        eskf._predict(a_meas=a + A_REST, w_meas=np.zeros(3), dt=dt)

    assert eskf.v == approx(a * N * dt)
    assert eskf.p == approx(a * dt ** 2 * N * (N - 1) / 2)

def test_known_bias_is_compensated():
    """With the right biases in the state, a biased IMU at rest stays still.

    a_m = a + b_a and ω_m = ω + b_g (eq. 15-16), so a_m - b_a and ω_m - b_g are the true values.
    """
    b_a, b_g = np.array([0.1, -0.2, 0.05]), np.array([0.01, 0.02, -0.03])
    eskf = make_eskf()
    eskf.b_a, eskf.b_g = b_a.copy(), b_g.copy()

    for _ in range(1000):
        eskf._predict(a_meas=A_REST + b_a, w_meas=b_g, dt=0.01)

    assert eskf.p == approx(np.zeros(3), abs=1e-10)
    assert eskf.v == approx(np.zeros(3), abs=1e-10)
    assert eskf.q == approx((1.0, 0.0, 0.0, 0.0))


#covariance propagation

def run_noisy(steps=500, dt=0.01):
    """Propagate with noisy IMU readings and every process noise on.

    Returns:
        The filter and the trace of P after each step.
    """
    eskf = make_eskf(sigma_a=0.05, sigma_w=0.01, sigma_aw=1e-3, sigma_ww=1e-4)
    eskf.P = np.eye(15) * 1e-4
    rng = np.random.default_rng(0)
    traces = [np.trace(eskf.P)]

    for _ in range(steps):
        eskf._predict(a_meas=A_REST + rng.normal(size=3), w_meas=rng.normal(size=3) * 0.3, dt=dt)
        traces.append(np.trace(eskf.P))

    return eskf, np.array(traces)

def test_covariance_symmetric():
    """P stays symmetric after noisy propagation.

    Φ P Φᵀ + F_i Q_i F_iᵀ is symmetric whenever P and Q_i are, but floating point does not
    guarantee it; an asymmetric P would give complex eigenvalues and meaningless variances.
    """
    eskf, _ = run_noisy()
    np.testing.assert_allclose(eskf.P, eskf.P.T, atol=1e-12)

def test_covariance_positive_semidefinite():
    """P stays positive semidefinite after noisy propagation.

    Φ P Φᵀ is a congruence (keeps xᵀPx >= 0 for every x) and F_i Q_i F_iᵀ is PSD, so their sum
    is PSD: no direction can get a negative variance.
    """
    eskf, _ = run_noisy()
    assert np.all(np.linalg.eigvalsh(eskf.P) >= -1e-12)

def test_covariance_grows():
    """Without measurements uncertainty only grows: trace(P) increases every step.

    Every prediction adds process noise F_i Q_i F_iᵀ ⪰ 0 and only measurements remove
    information. Φ P Φᵀ alone could lower the trace for some correlated P (e.g. a negative
    p-v correlation with p⁺ = p + v·dt); from this small diagonal P0 the added noise dominates.
    """
    _, traces = run_noisy()
    assert np.all(np.diff(traces) > 0)

def test_covariance_gyro_noise_only():
    """Gyro white noise alone gives var(δθ) = N·σ_w²·dt² from P0 = 0.

    Each step adds an angle impulse of variance σ_w²·dt² (white noise integrated over one step).
    """
    sigma_w = 0.01
    dt = 0.01
    N = 1000
    eskf = make_eskf(sigma_w=sigma_w)
    eskf.P = np.zeros((15, 15))

    for _ in range(N):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=dt)

    np.testing.assert_allclose(np.diag(eskf.P)[6:9], N * sigma_w ** 2 * dt ** 2)

def test_covariance_accel_noise_only():
    """Accel white noise alone gives var(δv) = N·σ_a²·dt² from P0 = 0.

    Each step adds a velocity impulse of variance σ_a²·dt².
    """
    sigma_a = 0.05
    dt = 0.01
    N = 1000
    eskf = make_eskf(sigma_a=sigma_a)
    eskf.P = np.zeros((15, 15))

    for _ in range(N):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=dt)

    np.testing.assert_allclose(np.diag(eskf.P)[3:6], N * sigma_a ** 2 * dt ** 2)

def test_covariance_bias_random_walk():
    """The bias random walk gives var(b) = N·σ²·dt from P0 = 0.

    ḃ = η with η ~ N(0, K²). Integrated over dt its variance is K²·dt, linear in dt (white
    noise integrated once), not dt².
    """
    sigma_aw, sigma_ww = 1e-3, 1e-4
    dt = 0.01
    N = 1000
    eskf = make_eskf(sigma_aw=sigma_aw, sigma_ww=sigma_ww)
    eskf.P = np.zeros((15, 15))

    for _ in range(N):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=dt)

    np.testing.assert_allclose(np.diag(eskf.P)[9:12], N * sigma_aw ** 2 * dt)
    np.testing.assert_allclose(np.diag(eskf.P)[12:15], N * sigma_ww ** 2 * dt)


#transition matrix vs finite differences

def set_state(eskf, p, v, q, b_a, b_g):
    """Set the nominal state of the filter (copies the arrays)."""
    eskf.p, eskf.v, eskf.q, eskf.b_a, eskf.b_g = p.copy(), v.copy(), q.copy(), b_a.copy(), b_g.copy()

def inject(p, v, q, b_a, b_g, dx):
    """True state x ⊕ δx: add the error to the nominal, locally on the attitude (q ⊗ δq{δθ}).

    Returns:
        (p, v, q, b_a, b_g) of the perturbed state.
    """
    return (p + dx[0:3], v + dx[3:6], quat_product(q, quat_from_rotvec(dx[6:9])),
            b_a + dx[9:12], b_g + dx[12:15])

def error_between(nominal, perturbed):
    """Error state from nominal to perturbed; inverse of inject (attitude error in local axes).

    Returns:
        δx, shape (15,).
    """
    return np.concatenate([
        perturbed.p - nominal.p,
        perturbed.v - nominal.v,
        rotvec_from_quat(quat_product(quat_conjugate(nominal.q), perturbed.q)),
        perturbed.b_a - nominal.b_a,
        perturbed.b_g - nominal.b_g,
    ])

def test_transition_matrix_finite_differences():
    """Φ is the Jacobian of the discrete propagation (sec. 3.5.2).

    Perturbs each of the 15 error components by ε, propagates nominal and perturbed one step
    and checks that the propagated error equals Φ·δx. Arbitrary state and IMU input, so every
    block of Φ is non-zero.
    """
    rng = np.random.default_rng(1)
    dt = 0.01
    state = (rng.normal(size=3), rng.normal(size=3), quat_from_rotvec(rng.normal(size=3)),
             rng.normal(size=3) * 0.1, rng.normal(size=3) * 0.01)
    a_meas = A_REST + rng.normal(size=3)
    w_meas = rng.normal(size=3)

    nominal = make_eskf()
    set_state(nominal, *state)
    nominal.R = rotation_matrix_hamilton(nominal.q)
    Phi = nominal._transition_matrix(a_meas - nominal.b_a, (w_meas - nominal.b_g) * dt, dt)
    nominal._predict(a_meas, w_meas, dt)

    eps = 1e-6
    for i in range(15):
        dx = np.zeros(15)
        dx[i] = eps
        perturbed = make_eskf()
        set_state(perturbed, *inject(*state, dx))
        perturbed._predict(a_meas, w_meas, dt)

        np.testing.assert_allclose(error_between(nominal, perturbed), Phi @ dx, atol=1e-10,
                                   err_msg=f"column {i} of Phi")


# magnetometer update

MAG_REF = np.array([26.0, -0.5, 37.0])

def test_mag_observation_jacobian_finite_differences():
    """The attitude columns of the magnetometer H match finite differences.

    H = H_x·X_δx (eq. 44): the derivative of h = R(q)ᵀ mag_ref w.r.t. the quaternion, chained
    with ∂q/∂δθ. Perturbing the true attitude locally, q ⊗ q{ε·e_i}, and dividing the change
    of h by ε must give column i. h only depends on q, so only the δθ columns are checked.
    """
    rng = np.random.default_rng(2)
    eskf = make_eskf()
    eskf.mag_ref = MAG_REF
    eskf.q = quat_from_rotvec(rng.normal(size=3))

    H_x = np.zeros((3, 16))
    H_x[:, 6:10] = jacobian_rotT_vec_q(eskf.q, MAG_REF)
    H = eskf._observation_jacobian(H_x)

    h = lambda q: rotation_matrix_hamilton(q).T @ MAG_REF
    eps = 1e-7
    for i in range(3):
        dtheta = np.zeros(3)
        dtheta[i] = eps
        dh = (h(quat_product(eskf.q, quat_from_rotvec(dtheta))) - h(eskf.q)) / eps
        np.testing.assert_allclose(H[:, 6 + i], dh, atol=1e-5)

def test_mag_corrects_yaw():
    """Magnetometer updates remove a 0.3 rad yaw error.

    Truth at rest with identity attitude. Roll and pitch are assumed known (small P): a single
    field vector only fixes 2 of the 3 angles.
    """
    eskf = make_eskf(sigma_w=1e-3, sigma_ww=1e-5)
    eskf.mag_ref = MAG_REF
    eskf.q = quat_from_rotvec(np.array([0, 0, 0.3]))
    eskf.P = np.diag([1e-2] * 6 + [1e-6, 1e-6, 0.5 ** 2] + [1e-4] * 3 + [1e-6] * 3)

    for k in range(500):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=0.01)
        if k % 10 == 0:
            eskf.update_mag(MAG_REF, sigma_mag=0.5)

    assert quat_to_euler(eskf.q)[2] == approx(0.0, abs=1e-2)

def test_mag_requires_reference():
    """update_mag raises if mag_ref is not set.

    The model h = R(q)ᵀ mag_ref needs the Earth field in NED at the launch site (WMM/IGRF):
    without it there is nothing to compare the measurement with, and an explicit error is
    better than silently using None.
    """
    eskf = make_eskf()
    with pytest.raises(ValueError):
        eskf.update_mag(MAG_REF, sigma_mag=0.5)


# barometer update

def test_baro_corrects_only_z():
    """The barometer corrects z and leaves x, y alone.

    H = [0 0 1 0 …], so K = P Hᵀ / S is the z column of P scaled: with a diagonal P only z gets
    gain. With P_zz = 100 and σ = 0.5, K = 100/100.25 ≈ 0.998, so z moves almost all the way
    to the measurement and P_zz⁺ = (1 - K)·P_zz ≈ 0.249 < σ².
    """
    eskf = make_eskf()
    eskf.p = np.array([5.0, -3.0, 10.0])
    eskf.P = np.eye(15) * 1e-2
    eskf.P[2, 2] = 100.0

    eskf.update_baro(z_meas=0.0, sigma_baro=0.5)

    assert eskf.p[0:2] == approx([5.0, -3.0])
    assert abs(eskf.p[2]) < 0.5
    assert eskf.P[2, 2] < 0.5 ** 2

def test_baro_accepts_array_measurement():
    """z_meas can be a scalar or a 1-element array.

    h(x) = p_z has shape (1,), so the innovation y - h(x) needs y with shape (1,) as well;
    update_baro promotes a scalar so drivers can pass either.
    """
    eskf = make_eskf()
    eskf.update_baro(z_meas=np.array([1.0]), sigma_baro=0.5)
    assert eskf.p.shape == (3,)


# gps update

def test_gps_linear_kf():
    """The GPS update equals the classic linear Kalman filter computed by hand.

    h(x) = [p; v] is linear and its H is exact, so the ESKF must give K = P Hᵀ(H P Hᵀ + V)⁻¹,
    the Joseph-form P and then the reset G on the attitude block. A full random P makes the
    correction reach all 15 error states.
    """
    rng = np.random.default_rng(9)
    eskf = make_eskf()
    eskf.p = rng.normal(size=3)
    eskf.v = rng.normal(size=3)
    A = rng.normal(size=(15, 15))
    eskf.P = A @ A.T + np.eye(15)

    p_meas = eskf.p + rng.normal(size=3)
    v_meas = eskf.v + rng.normal(size=3)
    sigma_p, sigma_v = 0.7, 0.2

    x0 = np.concatenate([eskf.p, eskf.v])
    P0 = eskf.P.copy()
    y = np.concatenate([p_meas, v_meas])

    H = np.zeros((6,15))
    H[:, 0:6] = np.eye(6)
    V = np.diag([sigma_p**2] * 3 + [sigma_v**2] * 3)

    K = P0 @ H.T @ np.linalg.inv(H @ P0 @ H.T + V)
    dx = K @ (y - x0)
    I_KH = np.eye(15) - K @ H
    P_kf = I_KH @ P0 @ I_KH.T + K @ V @ K.T

    eskf.update_gps(p_meas, v_meas, sigma_p, sigma_v)

    np.testing.assert_allclose(eskf.p, x0[0:3] + dx[0:3], atol=1e-12)
    np.testing.assert_allclose(eskf.v, x0[3:6] + dx[3:6], atol=1e-12)

    G = np.eye(15)
    G[6:9, 6:9] = np.eye(3) - skew(0.5 * dx[6:9])
    np.testing.assert_allclose(eskf.P, G @ P_kf @ G.T, atol=1e-12)

def test_gps_sigma_v_large_ignores_velocity():
    """A huge σ_v makes K ≈ 0 for the velocity: v stays and p jumps to the measurement.

    P = I (no p-v correlation), so v can only change through its own measurement.
    """
    eskf = make_eskf()
    eskf.P = np.eye(15)
    eskf.update_gps(p_meas=np.full(3, 1.0), v_meas=np.full(3, 1.0),
                     sigma_p=0.01, sigma_v=1e4)

    assert eskf.p == approx(np.full(3, 1.0), abs=1e-3)
    assert eskf.v == approx(np.zeros(3), abs=1e-6)

def test_gps_sigma_p_large_ignores_position():
    """A huge σ_p makes K ≈ 0 for the position: p stays and v jumps to the measurement.

    Mirror of the σ_v test: K_p = P/(P + σ_p²) = 1/(1 + 10⁸) ≈ 10⁻⁸ while K_v = 1/(1 + 10⁻⁴) ≈ 1.
    """
    eskf = make_eskf()
    eskf.P = np.eye(15)
    eskf.update_gps(p_meas=np.full(3, 1.0), v_meas=np.full(3, 1.0), sigma_p=1e4, sigma_v=0.01)

    assert eskf.p == approx(np.zeros(3), abs=1e-6)
    assert eskf.v == approx(np.full(3, 1.0), abs=1e-3)

def test_gps_scalar_and_per_axis_sigma_match():
      """Scalar sigmas and constant per-axis sigmas give the same update.

      Isotropic noise is V = σ²·I. Passing σ or [σ, σ, σ] must build the same V (eq. 69), and
      then K, δx and P are identical.
      """
      rng = np.random.default_rng(3)
      p_meas, v_meas = rng.normal(size=3), rng.normal(size=3)
      a, b = make_eskf(), make_eskf()

      a.update_gps(p_meas, v_meas, sigma_p=0.5, sigma_v=0.1)
      b.update_gps(p_meas, v_meas, sigma_p=np.full(3, 0.5), sigma_v=np.full(3, 0.1))

      assert a.p == approx(b.p)
      assert a.v == approx(b.v)
      np.testing.assert_allclose(a.P, b.P)

def test_gps_velocity_corrects_tilt():
    """GPS velocity corrects roll and pitch although it does not measure attitude.

    A 0.1 rad roll error rotates gravity into ~1 m/s² of fake horizontal acceleration, so v
    drifts. The v-θ correlation built by Φ (block -R[a]×·dt) lets the velocity innovation
    correct the roll.
    """
    eskf = make_eskf(sigma_a=1e-2, sigma_w=1e-3)
    eskf.q = quat_from_rotvec(np.array([0.1, 0.0, 0.0]))
    eskf.P = np.diag([1e-4] * 6 + [0.2 ** 2] * 3 + [1e-6] * 6)

    for k in range(1000):
          eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=0.01)
          if k % 10 == 0:
              eskf.update_gps(np.zeros(3), np.zeros(3), sigma_p=0.5, sigma_v=0.05)

    roll, pitch, _ = quat_to_euler(eskf.q)
    assert roll == approx(0.0, abs=1e-2)
    assert pitch == approx(0.0, abs=1e-2)

def test_gps_at_rest_does_not_observe_yaw():
    """At rest GPS cannot see a yaw error, and P_yaw does not shrink.

    Rotating about the vertical does not change R·g, so a yaw error never produces velocity
    drift (a × δθ = 0 with both vertical). Yaw needs horizontal acceleration or the magnetometer.
    """
    eskf = make_eskf(sigma_a=1e-2, sigma_w=1e-3)
    eskf.q = quat_from_rotvec(np.array([0.0, 0.0, 0.3]))
    eskf.P = np.diag([1e-4] * 6 + [0.2 ** 2] * 3 + [1e-6] * 6)
    P_yaw0 = eskf.P[8, 8]

    for k in range(1000):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=0.01)
        if k % 10 == 0:
            eskf.update_gps(np.zeros(3), np.zeros(3), sigma_p=0.5, sigma_v=0.05)

    assert quat_to_euler(eskf.q)[2] == approx(0.3, abs=1e-3)
    assert eskf.P[8, 8] >= P_yaw0


# accel update

def test_accel_finite_differences():
    """All 15 columns of the accelerometer H match finite differences of R(q)ᵀg + b_a.

    Covers the attitude columns, the I₃ of b_a (missing in eq. 64) and the zeros elsewhere.
    """
    rng = np.random.default_rng(4)
    eskf = make_eskf()
    state = (rng.normal(size=3), rng.normal(size=3), quat_from_rotvec(rng.normal(size=3)),
               rng.normal(size=3) * 0.1, rng.normal(size=3) * 0.01)
    set_state(eskf, *state)

    H_x = np.zeros((3, 16))
    H_x[:, 6:10] = jacobian_rotT_vec_q(eskf.q, eskf.g)
    H_x[:, 10:13] = np.eye(3)
    H = eskf._observation_jacobian(H_x)

    h = lambda p, v, q, b_a, b_g: rotation_matrix_hamilton(q).T @ eskf.g + b_a
    eps = 1e-7
    for i in range(15):
        dx = np.zeros(15)
        dx[i] = eps
        dh = (h(*inject(*state, dx)) - h(*state)) / eps
        np.testing.assert_allclose(H[:, i], dh, atol=1e-5, err_msg=f"column {i} of H")

def test_accel_corrects_pitch_roll():
    """Accelerometer updates remove a 0.1 rad roll error.

    The filter expects gravity tilted in body axes; the measured vertical gravity tells it
    directly that the vehicle is level.
    """
    eskf = make_eskf()
    eskf.q = quat_from_rotvec(np.array([0.1, 0.0, 0.0]))
    eskf.P = np.diag([1e-4] * 6 + [0.2 ** 2] * 3 + [1e-6] * 6)

    for i in range(1000):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=0.01)
        if i % 10 == 0:
            eskf.update_accel(a_meas=A_REST, sigma_a=0.05)

    roll, pitch, _ = quat_to_euler(eskf.q)
    assert roll == approx(0.0, abs=1e-2)
    assert pitch == approx(0.0, abs=1e-2)

def test_accel_at_rest_does_not_observe_yaw():
    """The accelerometer cannot see a yaw error, and P_yaw does not shrink.

    Rz(ψ)ᵀ[0, 0, g] = [0, 0, g]: the yaw column of H is zero.
    """
    eskf = make_eskf()
    eskf.q = quat_from_rotvec(np.array([0.0, 0.0, 0.3]))
    eskf.P = np.diag([1e-4] * 6 + [0.2 ** 2] * 3 + [1e-6] * 6)
    P_yaw0 = eskf.P[8, 8]

    for i in range(1000):
        eskf._predict(a_meas=A_REST, w_meas=np.zeros(3), dt=0.01)
        if i % 10 == 0:
            eskf.update_accel(a_meas=A_REST, sigma_a=0.05)

    assert quat_to_euler(eskf.q)[2] == approx(0.3, abs=1e-3)
    assert eskf.P[8, 8] >= P_yaw0

def test_accel_tilt_and_horizontal_bias_are_indistinguishable():
      """At rest a roll error and a b_a,y bias have parallel H columns (rank 1).

      Both shift the measurement along body y, so a reading cannot tell them apart and the
      filter splits the innovation according to P. Rotation in flight separates them.
      """
      eskf = make_eskf()
      H_x = np.zeros((3, 16))
      H_x[:, 6:10] = jacobian_rotT_vec_q(eskf.q, eskf.g)
      H_x[:, 10:13] = np.eye(3)
      H = eskf._observation_jacobian(H_x)

      roll_col, b_ay_col = H[:, 6], H[:, 10]
      assert np.linalg.matrix_rank(np.column_stack([roll_col, b_ay_col])) == 1

@pytest.mark.parametrize("a_meas", [
    A_REST * 6,                          # boost: gravity + 5 g of thrust along z
    np.zeros(3),                         # coast: free fall, the accel reads ~0
    A_REST + np.array([4.0, 0.0, 0.0]),  # horizontal acceleration: ||a|| = 10.59
])
def test_accel_rejected_with_linear_acceleration(a_meas):
    """The accel update is rejected under linear acceleration and nothing changes (sec. 4.5.2).

    The accelerometer measures specific force: with linear acceleration it is no longer just
    gravity, and h = Rᵀg + b_a would turn that acceleration into tilt or bias. Cases: boost,
    coast (free fall) and strong horizontal acceleration.
    """
    eskf = make_eskf()
    eskf.P = np.diag([1e-4] * 6 + [0.2 ** 2] * 3 + [0.1 ** 2] * 3 + [1e-6] * 3)
    q0, b_a0, P0 = eskf.q.copy(), eskf.b_a.copy(), eskf.P.copy()

    assert eskf.update_accel(a_meas, sigma_a=0.05) is False
    assert eskf.q == approx(q0)
    assert eskf.b_a == approx(b_a0)
    np.testing.assert_array_equal(eskf.P, P0)

def test_accel_used_near_g():
    """A measurement with norm close to g is used and reduces P.

    | ‖a_m‖ - g | below the threshold means the accelerometer sees (almost) only gravity, so
    h = Rᵀg + b_a holds and the update removes K S Kᵀ from P (information on roll and pitch).
    """
    eskf = make_eskf()
    eskf.P = np.diag([1e-4] * 6 + [0.2 ** 2] * 3 + [1e-6] * 6)
    P0 = eskf.P.copy()

    assert eskf.update_accel(A_REST + np.array([0.1, -0.1, 0.2]), sigma_a=0.05) is True
    assert np.trace(eskf.P) < np.trace(P0)

def test_accel_threshold_is_configurable():
    """accel_threshold decides which measurements pass.

    2 m/s² forward only changes the norm by 0.2 (√(9.81² + 2²) = 10.01) but tilts the vector
    by 11.5°: the threshold is necessary, not sufficient, so it has to be tuned.
    """
    a_meas = A_REST + np.array([2.0, 0.0, 0.0])
    loose = ESKF(0, 0, 0, 0, accel_threshold=0.5)
    tight = ESKF(0, 0, 0, 0, accel_threshold=0.1)

    assert loose.update_accel(a_meas, sigma_a=0.05) is True
    assert tight.update_accel(a_meas, sigma_a=0.05) is False

# generic update

def random_filter(seed):
    """Filter with an arbitrary state and a full, correlated P.

    Every update then touches all 15 error states.

    Returns:
        The filter and its random generator.
    """
    rng = np.random.default_rng(seed)
    eskf = make_eskf()
    eskf.mag_ref = MAG_REF
    set_state(eskf, rng.normal(size=3), rng.normal(size=3), quat_from_rotvec(rng.normal(size=3)),
              rng.normal(size=3) * 0.1, rng.normal(size=3) * 0.01)
    A = rng.normal(size=(15, 15)) * 0.1
    eskf.P = A @ A.T + np.eye(15) * 1e-3
    return eskf, rng

UPDATES = {
    "gps": lambda f, rng: f.update_gps(f.p + rng.normal(size=3), f.v + rng.normal(size=3), 0.5, 0.1),
    "mag": lambda f, rng: f.update_mag(MAG_REF + rng.normal(size=3), 0.5),
    "accel": lambda f, rng: f.update_accel(A_REST + rng.normal(size=3) * 0.1, 0.05),
    "baro": lambda f, rng: f.update_baro(f.p[2] + rng.normal(), 0.5),
}

@pytest.mark.parametrize("kind", UPDATES)
def test_update_keeps_P_symmetric_positive_definite(kind):
    """P stays symmetric and positive definite after each kind of update.

    The Joseph form (I-KH)P(I-KH)ᵀ + KVKᵀ is a sum of quadratic forms: PD by construction.
    """
    eskf, rng = random_filter(5)
    UPDATES[kind](eskf, rng)

    np.testing.assert_allclose(eskf.P, eskf.P.T, atol=1e-12)
    assert np.all(np.linalg.eigvalsh(eskf.P) > 0)

NO_INNOVATION = {
    "gps": lambda f: f.update_gps(f.p, f.v, 0.5, 0.1),
    "mag": lambda f: f.update_mag(rotation_matrix_hamilton(f.q).T @ MAG_REF, 0.5),
    "accel": lambda f: f.update_accel(rotation_matrix_hamilton(f.q).T @ f.g + f.b_a, 0.05),
    "baro": lambda f: f.update_baro(f.p[2], 0.5),
}

@pytest.mark.parametrize("kind", NO_INNOVATION)
def test_update_does_not_increase_uncertainty(kind):
    """A measurement never adds uncertainty: P⁻ - P⁺ is PSD and trace(P) decreases.

    P⁺ = P⁻ - K S Kᵀ. P⁺ does not depend on y, so y = h(x) gives δx = 0 and G = I: this
    isolates the Kalman step (with a large injected δθ, G P Gᵀ can grow the attitude block
    slightly).
    """
    eskf, _ = random_filter(6)
    P0 = eskf.P.copy()
    NO_INNOVATION[kind](eskf)

    assert np.all(np.linalg.eigvalsh(P0 - eskf.P) > -1e-12)
    assert np.trace(eskf.P) < np.trace(P0)

@pytest.mark.parametrize("kind", UPDATES)
def test_update_keeps_quaternion_unit(kind):
    """The quaternion stays unit after repeated updates with large attitude corrections.

    The injection q ⊗ q{δθ} (eq. 48) uses the exact exponential map, which is unit for any δθ;
    the first-order form [1, δθ/2] would not be. Large P_θ forces large δθ to exercise it.
    """
    eskf, rng = random_filter(7)
    eskf.P[6:9, 6:9] += np.eye(3)          # large attitude uncertainty -> large injected dtheta
    for _ in range(20):
        UPDATES[kind](eskf, rng)

    assert np.linalg.norm(eskf.q) == approx(1.0, abs=1e-12)

def test_reset_jacobian_finite_differences():
    """The reset Jacobian G = I - [½δθ̂]× (eq. 51) by finite differences.

    After injecting δθ̂, the remaining error is measured from the new nominal:
    δθ⁺ = Log(δq(δθ̂)⁻¹ ⊗ δq(δθ⁻)). Its derivative at δθ⁻ = δθ̂ is G up to O(‖δθ̂‖²).
    """
    dtheta_hat = np.array([0.01, -0.02, 0.015])
    G = np.eye(3) - skew(0.5 * dtheta_hat)

    after = lambda dtheta: rotvec_from_quat(quat_product(quat_conjugate(quat_from_rotvec(dtheta_hat)),
                                                         quat_from_rotvec(dtheta)))
    eps = 1e-7
    for i in range(3):
        d = np.zeros(3)
        d[i] = eps
        column = (after(dtheta_hat + d) - after(dtheta_hat)) / eps
        np.testing.assert_allclose(column, G[:, i], atol=np.linalg.norm(dtheta_hat) ** 2)

def test_sequential_updates_equal_stacked():
    """Updating GPS then baro equals one stacked update.

    With independent noises (block-diagonal V) the second update starts from the P already
    reduced by the first, so nothing is lost (contradicts the justification in sec. 4.5.3).
    GPS and baro are linear, so every state but θ matches exactly; θ differs only through the
    reset G between the two updates (second order).
    """
    eskf, rng = random_filter(8)
    x0 = np.concatenate([eskf.p, eskf.v, np.zeros(3), eskf.b_a, eskf.b_g])
    P0 = eskf.P.copy()
    p_meas, v_meas, z_meas = eskf.p + rng.normal(size=3), eskf.v + rng.normal(size=3), eskf.p[2] + 0.3

    H = np.zeros((7, 15))
    H[0:6, 0:6] = np.eye(6)
    H[6, 2] = 1.0
    V = np.diag([0.5 ** 2] * 3 + [0.1 ** 2] * 3 + [0.5 ** 2])
    y = np.concatenate([p_meas, v_meas, [z_meas]])
    K = P0 @ H.T @ np.linalg.inv(H @ P0 @ H.T + V)
    dx = K @ (y - H @ x0)

    eskf.update_gps(p_meas, v_meas, 0.5, 0.1)
    eskf.update_baro(z_meas, 0.5)

    np.testing.assert_allclose(eskf.p, x0[0:3] + dx[0:3], atol=1e-12)
    np.testing.assert_allclose(eskf.v, x0[3:6] + dx[3:6], atol=1e-12)
    np.testing.assert_allclose(eskf.b_a, x0[9:12] + dx[9:12], atol=1e-12)
    np.testing.assert_allclose(eskf.b_g, x0[12:15] + dx[12:15], atol=1e-12)


def run_bg(updates, steps=6000):
      """60 s at rest with an unknown gyro bias, fusing the given updates every 0.1 s.

      The attitude starts right (small P_θ), P_bg covers the true bias, and P_ba is small so a
      tilt is not blamed on the accelerometer bias.

      Args:
          updates: Set with "accel" and/or "mag".

      Returns:
          The filter and the true gyro bias.
      """
      b_g_true = np.array([0.01, -0.02, 0.015])   
      eskf = make_eskf(sigma_w=1e-3, sigma_ww=1e-5)
      eskf.mag_ref = MAG_REF
      eskf.P = np.diag([1e-4] * 6 + [0.01 ** 2] * 3 + [1e-6] * 3 + [0.05 ** 2] * 3)
      for k in range(steps):
          eskf._predict(a_meas=A_REST, w_meas=b_g_true, dt=0.01)
          if k % 10 == 0:
              if "accel" in updates: eskf.update_accel(A_REST, sigma_a=0.05)
              if "mag" in updates: eskf.update_mag(MAG_REF, sigma_mag=0.5)
      return eskf, b_g_true

def test_gyro_bias_converges_with_accel_and_mag():
    """With accel + mag the full attitude is observed and all of b_g converges.

    The bias makes the attitude drift, the updates keep correcting it, and the θ-b_g
    correlation (block -I·dt of Φ) attributes the drift to the bias.
    """
    eskf, b_g_true = run_bg({"accel", "mag"})
    assert eskf.b_g == approx(b_g_true, abs=1e-3)

def test_gyro_bias_z_unobservable_with_accel_only():
    """Without magnetometer b_g,z does not converge.

    A z bias drifts the yaw, which gravity cannot see (sec. 4.5.3).
    """
    eskf, b_g_true = run_bg({"accel"})
    assert eskf.b_g[0:2] == approx(b_g_true[0:2], abs=1e-3)
    assert abs(eskf.b_g[2] - b_g_true[2]) > 5e-3


    
def run_ba(updates, steps=3000):
    """30 s at rest with an unknown accelerometer bias, fusing the given updates every 0.1 s.

    Truth level and at rest; the attitude is known (small P_θ).

    Args:
        updates: Set with "baro" and/or "gps".

    Returns:
        The filter and the true accelerometer bias.
    """
    b_a_true = np.array([0.05, -0.04, 0.2])
    eskf = make_eskf(sigma_a=1e-2, sigma_aw=1e-5)
    eskf.P = np.diag([1e-4] * 6 + [1e-6] * 3 + [0.5 ** 2] * 3 + [1e-6] * 3)
    for k in range(steps):
        eskf._predict(a_meas=A_REST + b_a_true, w_meas=np.zeros(3), dt=0.01)
        if k % 10 == 0:
            if "baro" in updates: eskf.update_baro(0.0, sigma_baro=0.5)
            if "gps" in updates: eskf.update_gps(np.zeros(3), np.zeros(3), sigma_p=0.5, sigma_v=0.05)
    return eskf, b_a_true

def test_accel_bias_z_converges_with_baro():
    """The baro makes b_a,z converge.

    An unknown b_a,z is integrated twice into z (½·b·t²). The baro sees z drift away and the
    p-v-b_a correlation built by Φ (blocks I·dt and -R·dt) attributes it to the bias.
    """
    eskf, b_a_true = run_ba({"baro"})
    assert eskf.b_a[2] == approx(b_a_true[2], abs=1e-2)

def test_accel_bias_horizontal_unobservable_with_baro_only():
    """With only the baro the horizontal b_a stays unestimated.

    A horizontal bias drifts x and y, which the baro never sees.
    """
    eskf, _ = run_ba({"baro"})
    assert eskf.b_a[0:2] == approx(np.zeros(2), abs=1e-6)

def test_accel_bias_converges_with_gps():
    """GPS makes all of b_a converge.

    GPS sees the drift in every axis (position and velocity). Horizontally only because P_θ is
    small: at rest a tilt and a horizontal b_a are indistinguishable (see
    test_accel_tilt_and_horizontal_bias_are_indistinguishable).
    """
    eskf, b_a_true = run_ba({"gps"})
    assert eskf.b_a == approx(b_a_true, abs=1e-2)
