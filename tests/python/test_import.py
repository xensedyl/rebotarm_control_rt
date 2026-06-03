"""离线测试：导入 + 配置解析 + API 表面一致性（无需硬件）。

不构造 RobotArm/Gripper（会打开串口），仅验证：
  - 包及子包可导入
  - 编译扩展 _native 提供应有的类与函数
  - load_cfg / load_gripper_cfg 解析包内 yaml 的字段与默认值
  - RobotArm/Gripper 暴露与 reBotArm_control_py 一致的方法名
"""
import importlib
from pathlib import Path

import pytest

PKG = "rebotarm_control_rt"


def test_import_all_subpackages():
    """全部子包均为原生实现（_native / _math），不依赖 Python pinocchio。"""
    importlib.import_module(PKG)
    importlib.import_module(f"{PKG}.actuator")
    importlib.import_module(f"{PKG}.kinematics")
    importlib.import_module(f"{PKG}.dynamics")
    importlib.import_module(f"{PKG}.trajectory")
    importlib.import_module(f"{PKG}.controllers")


def test_no_python_pinocchio_dependency(monkeypatch):
    """确保导入数学层不依赖 Python pinocchio（C++ 直连）。"""
    import sys
    monkeypatch.setitem(sys.modules, "pinocchio", None)  # 模拟 pinocchio 不可用
    for sub in ("kinematics", "dynamics", "trajectory", "controllers"):
        mod = importlib.reload(importlib.import_module(f"{PKG}.{sub}"))
        assert mod is not None


def test_native_exports():
    native = importlib.import_module(f"{PKG}._native")
    for name in ("RobotArm", "Gripper", "JointCfg", "GripperCfg",
                 "load_cfg", "load_gripper_cfg"):
        assert hasattr(native, name), f"_native 缺少 {name}"


def test_actuator_reexports():
    act = importlib.import_module(f"{PKG}.actuator")
    for name in ("RobotArm", "Gripper", "JointCfg", "GripperCfg",
                 "load_cfg", "load_gripper_cfg"):
        assert hasattr(act, name), f"actuator 缺少 {name}"


def test_load_arm_cfg_defaults():
    act = importlib.import_module(f"{PKG}.actuator")
    cfg = act.load_cfg()  # 包内 config/arm.yaml
    assert cfg["channel"] == "/dev/ttyACM0"
    assert cfg["rate"] == 500.0
    joints = cfg["joints"]
    assert len(joints) == 6
    j1 = joints[0]
    assert j1.name == "joint1"
    assert j1.motor_id == 0x01
    assert j1.feedback_id == 0x11
    assert j1.vendor == "damiao"
    assert j1.kp == 120.0
    assert j1.kd == 8.0
    assert j1.vel_kp == 0.0125
    assert j1.vlim == 5.0


def test_load_gripper_cfg_defaults():
    act = importlib.import_module(f"{PKG}.actuator")
    cfg = act.load_gripper_cfg()
    g = cfg["gripper"]
    assert g.name == "gripper"
    assert g.motor_id == 0x07
    assert g.vendor == "damiao"
    assert g.kp == 8.0


def test_robotarm_api_surface():
    act = importlib.import_module(f"{PKG}.actuator")
    expected = [
        "connect", "disconnect", "reconnect", "enable", "disable",
        "set_zero", "set_zero_single", "get_state", "get_positions",
        "get_velocities", "get_torques", "mode_mit", "mode_pos_vel",
        "mode_vel", "mit", "pos_vel", "set_vel", "estop",
        "start_control_loop", "stop_control_loop", "start_rt_loop",
        "set_targets",
    ]
    for name in expected:
        assert hasattr(act.RobotArm, name), f"RobotArm 缺少方法 {name}"


if __name__ == "__main__":
    raise SystemExit(pytest.main([str(Path(__file__)), "-v"]))
