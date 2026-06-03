"""Kinematics 运动学库 —— 由 C++ (_math, Pinocchio) 实现，Python 仅 re-export。

接口与 reBotArm_control_py.kinematics 对齐。位姿在 Python 边界统一为 4×4 numpy 齐次矩阵
（不再暴露 pinocchio.SE3）。
"""
from pathlib import Path

from rebotarm_control_rt._math import (
    RobotModel,
    IKParams,
    IKResult,
    compute_fk,
    joint_to_pose,
    compute_ik,
    pos_rot_to_se3,
    get_joint_names,
    get_joint_limits,
    get_frame_id,
    get_end_effector_frame_id,
    get_all_frame_names,
    load_robot_model as _load_robot_model_native,
)

# 与 inverse_kinematics.py 的别名一致。
IKSolverParams = IKParams

_URDF = str(
    Path(__file__).resolve().parent.parent
    / "urdf" / "reBot-DevArm_fixend_description" / "urdf" / "reBot-DevArm_fixend.urdf"
)


def load_robot_model(urdf_path: str | None = None) -> RobotModel:
    """构建 reBot-DevArm 模型；urdf_path 为空时用内置 URDF。"""
    return _load_robot_model_native(urdf_path if urdf_path else _URDF)


__all__ = [
    "RobotModel",
    "load_robot_model",
    "get_joint_names",
    "get_joint_limits",
    "get_frame_id",
    "get_end_effector_frame_id",
    "get_all_frame_names",
    "compute_fk",
    "joint_to_pose",
    "compute_ik",
    "pos_rot_to_se3",
    "IKParams",
    "IKSolverParams",
    "IKResult",
]
