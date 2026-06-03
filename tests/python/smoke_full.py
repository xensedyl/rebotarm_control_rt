"""端到端导入冒烟：全部公共 API 可用，且不依赖 Python pinocchio。"""
import numpy as np
import rebotarm_control_rt as pkg
from rebotarm_control_rt.actuator import RobotArm, Gripper, JointCfg, GripperCfg, load_cfg
from rebotarm_control_rt.kinematics import (
    RobotModel, load_robot_model, compute_fk, compute_ik, pos_rot_to_se3,
    get_end_effector_frame_id, IKParams,
)
from rebotarm_control_rt.dynamics import (
    load_dynamics_model, compute_mass_matrix, compute_generalized_gravity,
    compute_inverse_dynamics, get_default_gravity,
)
from rebotarm_control_rt.trajectory import (
    TrajProfile, TrajPlanParams, plan_cartesian_geodesic_trajectory,
    track_trajectory, plan_joint_space_trajectory,
)
from rebotarm_control_rt.controllers import ArmEndPos

print("package:", pkg.__version__, "| subpackages:", pkg.__all__)

rm = load_robot_model()
q = rm.neutral()
print("nq:", rm.nq, "| ee frame:", get_end_effector_frame_id(rm))
print("FK ok:", compute_fk(rm, q)[2].shape)
print("M ok:", compute_mass_matrix(rm, q).shape, "| g ok:", compute_generalized_gravity(rm, q).shape)
print("default gravity:", get_default_gravity())
cfg = load_cfg()
print("arm cfg joints:", len(cfg["joints"]), "| j1 kp:", cfg["joints"][0].kp)
print("ALL IMPORTS + NATIVE CALLS OK (no python pinocchio)")
