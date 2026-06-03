"""冒烟：C++ _math 加载 URDF、FK at neutral、关节名。验证 Pinocchio C++ 链路。"""
from pathlib import Path
import numpy as np
from rebotarm_control_rt import _math

URDF = (
    Path(_math.__file__).parent
    / "urdf" / "reBot-DevArm_fixend_description" / "urdf" / "reBot-DevArm_fixend.urdf"
)
rm = _math.RobotModel(str(URDF))
print("nq =", rm.nq())
print("joints =", rm.joint_names())
q = rm.neutral()
print("neutral q =", np.asarray(q))
T = rm.fk(q)
print("FK(neutral) end_link 4x4 =\n", np.asarray(T))
assert np.asarray(T).shape == (4, 4)
assert abs(np.linalg.det(np.asarray(T)[:3, :3]) - 1.0) < 1e-9, "旋转块应为正交阵"
print("OK: Pinocchio C++ link + FK works")
