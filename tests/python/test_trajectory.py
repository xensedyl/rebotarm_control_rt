"""C++ 轨迹层测试：测地线端点、CLIK 跟踪自洽、统计。"""
from pathlib import Path
import numpy as np
import pytest

m = pytest.importorskip("rebotarm_control_rt._math")


def _rm():
    urdf = str(Path(m.__file__).parent / "urdf" / "reBot-DevArm_fixend_description"
               / "urdf" / "reBot-DevArm_fixend.urdf")
    return m.RobotModel(urdf)


def test_geodesic_endpoints():
    rm = _rm()
    qa = np.array([0.0, -0.3, -0.5, 0.0, 0.2, 0.0])
    qb = np.array([0.3, -0.6, -0.8, 0.2, -0.1, 0.4])
    _, _, Ta = rm.fk(qa)
    _, _, Tb = rm.fk(qb)
    res = m.plan_cartesian_geodesic_trajectory(Ta, Tb, 2.0, m.TrajPlanParams(dt=0.05))
    pts = res.trajectory.points()
    assert res.n_points == len(pts) >= 2
    assert np.allclose(pts[0].pose, Ta, atol=1e-9)
    assert np.allclose(pts[-1].pose, Tb, atol=1e-6)
    assert abs(pts[0].time) < 1e-12 and abs(pts[-1].time - 2.0) < 1e-9


def test_clik_joint_space_tracking():
    rm = _rm()
    fid = rm.end_effector_frame_id()
    qa = np.array([0.0, -0.3, -0.5, 0.0, 0.2, 0.0])
    qb = np.array([0.3, -0.6, -0.8, 0.2, -0.1, 0.4])
    jt = m.plan_joint_space_trajectory(rm, fid, qa, qb, 2.0,
                                       m.TrajPlanParams(dt=0.05), m.CLIKParams(), 0.0)
    assert len(jt) >= 2
    # 末点应到达 qb 对应位姿
    _, _, Tb = rm.fk(qb)
    _, _, Treached = rm.fk(jt[-1].q)
    assert np.linalg.norm(Treached[:3, 3] - Tb[:3, 3]) < 1e-3
    assert all(p.ik_success for p in jt[-3:]), "末段应收敛"


def test_traj_stats():
    rm = _rm()
    fid = rm.end_effector_frame_id()
    qa = np.array([0.0, -0.3, -0.5, 0.0, 0.2, 0.0])
    qb = np.array([0.2, -0.5, -0.7, 0.1, 0.0, 0.3])
    _, _, Ta = rm.fk(qa)
    _, _, Tb = rm.fk(qb)
    params = m.TrajPlanParams(dt=0.05)
    jt = m.plan_joint_space_trajectory(rm, fid, qa, qb, 2.0, params, m.CLIKParams(), 0.0)
    stats = m.compute_traj_stats(rm, fid, jt, Ta, Tb, 2.0, params)
    assert stats.total_points == len(jt)
    assert stats.success_rate > 0.9
    assert stats.max_ik_error < 1e-2


if __name__ == "__main__":
    raise SystemExit(pytest.main([str(Path(__file__)), "-v"]))
