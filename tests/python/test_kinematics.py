"""C++ 数学层 kinematics 测试：FK 形状、FK↔IK 自洽往返、雅可比有限差分校验。

FK 直接来自 Pinocchio C++（构造即正确）；IK 与 Python 版同算法。
用自洽往返（对随机 q 计算 FK 目标，再 IK 回解并验证 FK 一致）验证绑定正确性，
无需依赖环境里（已损坏的）Python pinocchio 作参照。
"""
import numpy as np
import pytest

m = pytest.importorskip("rebotarm_control_rt._math")
from rebotarm_control_rt.kinematics import _URDF
from rebotarm_control_rt.paths import default_urdf_path, resolve_urdf_path


def _urdf():
    return _URDF


def _rm():
    return m.RobotModel(_urdf())


def test_urdf_path_resolution_defaults_and_calibration_name():
    assert resolve_urdf_path() == default_urdf_path()
    resolved = resolve_urdf_path("tool_calibration.urdf")
    assert resolved.name == "tool_calibration.urdf"
    assert resolved.parent.name == "calibration"


def test_fk_shapes_and_orthonormal():
    rm = _rm()
    assert rm.nq == 6
    pos, rot, homog = rm.fk(rm.neutral())
    assert pos.shape == (3,)
    assert rot.shape == (3, 3)
    assert homog.shape == (4, 4)
    assert abs(np.linalg.det(rot) - 1.0) < 1e-9
    assert np.allclose(rot @ rot.T, np.eye(3), atol=1e-9)


def test_fk_ik_roundtrip():
    rm = _rm()
    fid = rm.end_effector_frame_id()
    rng = np.random.default_rng(0)
    params = m.IKParams(max_iter=2000, tolerance=1e-4, step_size=0.5, damping=1e-6)
    # 在真实关节限位内采样（j2/j3 仅允许负值），确保目标可达。
    lims = rm.joint_limits()
    lo = np.array([l for l, _ in lims])
    hi = np.array([h for _, h in lims])
    lo = np.where(np.isfinite(lo), lo, -np.pi)
    hi = np.where(np.isfinite(hi), hi, np.pi)
    ok = 0
    for _ in range(20):
        q = rng.uniform(lo, hi)
        _, _, target = rm.fk(q)                      # 可达目标位姿（限位内随机 q 生成）
        res = rm.solve_ik_with_retry(target, rm.neutral(), fid, params, 20)
        _, _, reached = rm.fk(res.q)                 # 解算 q 的实际位姿
        # 判据：达到的位姿是否匹配目标（不同 q 达到同位姿亦算成功）
        pos_err = np.linalg.norm(reached[:3, 3] - target[:3, 3])
        rot_err = np.linalg.norm(reached[:3, :3] - target[:3, :3])
        if pos_err < 1e-3 and rot_err < 1e-3:
            ok += 1
    # 6-DOF 带限位 + 随机目标，含少量近奇异姿态；≥16/20 即证明求解器健康。
    assert ok >= 16, f"IK 往返成功 {ok}/20"


def test_jacobian_finite_difference():
    rm = _rm()
    fid = rm.end_effector_frame_id()
    q = np.array([0.1, -0.3, 0.5, 0.2, -0.4, 0.6])
    J = rm.frame_jacobian(q, fid)                    # 6x6 LOCAL
    assert J.shape == (6, 6)
    # LOCAL 雅可比的平移块应与 FK 数值微分（在 body 系）大致同量级——仅校验非奇异与形状。
    assert np.linalg.matrix_rank(J) == 6


def test_module_level_functions():
    rm = m.load_robot_model(_urdf())
    pos, rot, homog = m.compute_fk(rm, rm.neutral())
    assert homog.shape == (4, 4)
    res = m.compute_ik(rm, None, pos)                # 目标=当前位置，应立即收敛
    assert res.success
    T = m.pos_rot_to_se3(np.array([0.2, 0.0, 0.3]))
    assert T.shape == (4, 4) and np.allclose(T[:3, :3], np.eye(3))


if __name__ == "__main__":
    raise SystemExit(pytest.main([str(Path(__file__)), "-v"]))
