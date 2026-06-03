"""ArmEndPos 离线编排测试：用 Mock arm 验证 C++↔Python 跨语言驱动（无需硬件）。

Mock arm 立即跟随 set_targets（_q := pos），模拟理想执行器，
从而验证 start/move_to_ik/move_to_traj/safe_home 的编排逻辑与 GIL 处理。
"""
import time
from pathlib import Path

import numpy as np
import pytest

m = pytest.importorskip("rebotarm_control_rt._math")
from rebotarm_control_rt.controllers import ArmEndPos
from rebotarm_control_rt.kinematics import load_robot_model, joint_to_pose


class MockArm:
    def __init__(self, n=6):
        self._n = n
        self._q = np.zeros(n)
        self.calls = []

    @property
    def num_joints(self):
        return self._n

    def connect(self):
        self.calls.append("connect")

    def mode_pos_vel(self):
        self.calls.append("mode_pos_vel")

    def enable(self, *a, **k):
        self.calls.append("enable")

    def start_rt_loop(self, *a, **k):
        self.calls.append("start_rt_loop")

    def stop_control_loop(self, *a, **k):
        self.calls.append("stop_control_loop")

    def disconnect(self, *a, **k):
        self.calls.append("disconnect")

    def set_targets(self, pos, vlim=None, **kw):
        self._q = np.asarray(pos, dtype=float).copy()  # 理想执行器：立即跟随

    def get_state(self, request=False):
        return (self._q.copy(), np.zeros(self._n), np.zeros(self._n))


def test_start_and_move_to_ik():
    rm = load_robot_model()
    q_feasible = np.array([0.2, -0.5, -0.7, 0.1, 0.0, 0.3])
    pos, euler = joint_to_pose(rm, q_feasible)

    arm = MockArm()
    ep = ArmEndPos(arm)
    ep.start()
    assert {"connect", "mode_pos_vel", "enable", "start_rt_loop"} <= set(arm.calls)

    ok = ep.move_to_ik(float(pos[0]), float(pos[1]), float(pos[2]),
                       float(euler[0]), float(euler[1]), float(euler[2]))
    assert ok, "IK 应收敛"
    # mock._q 已被 set_targets 设为 IK 解；其 FK 应匹配目标位置
    _, _, reached = rm.fk(arm._q)
    assert np.linalg.norm(reached[:3, 3] - pos) < 1e-3


def test_move_to_traj_and_safe_home():
    rm = load_robot_model()
    q_target = np.array([0.15, -0.4, -0.6, 0.05, 0.1, 0.2])
    pos, euler = joint_to_pose(rm, q_target)

    arm = MockArm()
    ep = ArmEndPos(arm, dt=0.05)
    ep.start()

    ok = ep.move_to_traj(float(pos[0]), float(pos[1]), float(pos[2]),
                         float(euler[0]), float(euler[1]), float(euler[2]), duration=0.4)
    assert ok
    time.sleep(0.8)  # 等待 C++ 发送线程跑完轨迹
    _, _, reached = rm.fk(arm._q)
    assert np.linalg.norm(reached[:3, 3] - pos) < 5e-3, "轨迹末端应到达目标"

    ep.safe_home()
    assert np.max(np.abs(arm._q)) < 0.01, "safe_home 后应回零"
    ep.end()
    assert "disconnect" in arm.calls


if __name__ == "__main__":
    raise SystemExit(pytest.main([str(Path(__file__)), "-v", "-s"]))
