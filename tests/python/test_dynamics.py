"""C++ 动力学层测试：物理自洽性（结果即 Pinocchio，故等价验证绑定正确）。"""
import numpy as np
import pytest

m = pytest.importorskip("rebotarm_control_rt._math")
from rebotarm_control_rt.kinematics import _URDF


def _rm():
    return m.RobotModel(_URDF)


def test_mass_matrix_spd():
    rm = _rm()
    q = np.array([0.1, -0.3, -0.5, 0.2, -0.4, 0.6])
    M = m.mass_matrix(rm, q)
    assert M.shape == (6, 6)
    assert np.allclose(M, M.T, atol=1e-9), "质量矩阵应对称"
    assert np.all(np.linalg.eigvalsh(M) > 0), "质量矩阵应正定"


def test_gravity_equals_nle_at_zero_velocity():
    rm = _rm()
    q = np.array([0.2, -0.5, -0.7, 0.1, 0.3, -0.2])
    g = m.gravity_vector(rm, q)
    n = m.nle(rm, q, np.zeros(6))
    idtorque = m.inverse_dynamics(rm, q, np.zeros(6), np.zeros(6))
    assert np.allclose(g, n, atol=1e-9)
    assert np.allclose(g, idtorque, atol=1e-9), "rnea(q,0,0) 应等于广义重力"


def test_rnea_aba_roundtrip():
    rm = _rm()
    rng = np.random.default_rng(1)
    for _ in range(10):
        q = rng.uniform(-0.8, 0.0, size=6)
        v = rng.uniform(-1, 1, size=6)
        a = rng.uniform(-1, 1, size=6)
        tau = m.inverse_dynamics(rm, q, v, a)        # RNEA
        a_back = m.forward_dynamics(rm, q, v, tau)   # ABA
        assert np.allclose(a, a_back, atol=1e-6), "ABA∘RNEA 应还原加速度"


def test_all_terms_and_dtau_da_equal_mass_matrix():
    rm = _rm()
    q = np.array([0.0, -0.4, -0.6, 0.0, 0.2, 0.0])
    v = np.array([0.1, 0.2, -0.1, 0.0, 0.3, -0.2])
    M, C, g = m.all_terms(rm, q, v)
    assert np.allclose(M, m.mass_matrix(rm, q), atol=1e-9)
    _, _, dtau_da = m.rnea_derivatives(rm, q, v, np.zeros(6))
    assert np.allclose(dtau_da, m.mass_matrix(rm, q), atol=1e-6), "∂τ/∂q̈ == M(q)"


def test_kinetic_energy_formula():
    rm = _rm()
    q = np.array([0.1, -0.3, -0.5, 0.2, -0.4, 0.6])
    v = np.array([0.5, -0.2, 0.3, 0.1, -0.4, 0.2])
    M = m.mass_matrix(rm, q)
    T = m.kinetic_energy(rm, q, v)
    assert abs(T - 0.5 * v @ M @ v) < 1e-9


def test_gravity_toggle():
    rm = _rm()
    q = np.array([0.0, -0.6, -0.8, 0.0, 0.3, 0.0])
    assert np.allclose(rm.get_gravity(), [0, 0, -9.81], atol=1e-6)
    g_earth = m.gravity_vector(rm, q)
    rm.set_gravity(np.array([0.0, 0.0, 0.0]))
    g_zero = m.gravity_vector(rm, q)
    assert np.allclose(g_zero, 0.0, atol=1e-9), "零重力下重力项应为 0"
    assert not np.allclose(g_earth, 0.0)


def test_centroidal_and_com():
    rm = _rm()
    q = np.array([0.1, -0.3, -0.5, 0.2, -0.4, 0.6])
    v = np.zeros(6)
    com = m.center_of_mass(rm, q)
    assert com.shape == (3,)
    Ag = m.centroidal_matrix(rm, q, v)
    assert Ag.shape == (6, 6)
    h = m.centroidal_momentum(rm, q, v)
    assert h.shape == (6,) and np.allclose(h, 0.0, atol=1e-9)  # v=0 → 动量为 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([str(Path(__file__)), "-v"]))
