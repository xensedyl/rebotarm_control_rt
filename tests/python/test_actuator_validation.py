"""输入长度校验测试。需要可构造的 RobotArm（有硬件）；无硬件则跳过。"""
import numpy as np
import pytest

from rebotarm_control_rt.actuator import RobotArm


@pytest.fixture
def arm():
    try:
        a = RobotArm()
    except Exception as e:  # 无串口设备
        pytest.skip(f"无法构造 RobotArm（无硬件）: {e}")
    yield a
    try:
        a.disconnect()
    except Exception:
        pass


def test_mit_rejects_bad_lengths(arm):
    n = arm.num_joints
    with pytest.raises(ValueError):
        arm.mit(np.zeros(n - 1))                    # pos 太短
    with pytest.raises(ValueError):
        arm.mit(np.zeros(n), kp=np.zeros(n - 1))    # kp 长度错
    with pytest.raises(ValueError):
        arm.mit(np.zeros(n), vel=np.zeros(n + 1))   # vel 长度错


def test_set_targets_rejects_bad_lengths(arm):
    n = arm.num_joints
    with pytest.raises(ValueError):
        arm.set_targets(np.zeros(n), kd=np.zeros(2))
    with pytest.raises(ValueError):
        arm.set_targets(np.zeros(n + 1))


def test_pos_vel_set_vel_validation(arm):
    n = arm.num_joints
    with pytest.raises(ValueError):
        arm.pos_vel(np.zeros(n), vlim=np.zeros(n - 1))
    with pytest.raises(ValueError):
        arm.set_vel(np.zeros(n - 1))


def test_rt_overruns_getter_exists(arm):
    assert isinstance(arm.rt_overruns, int)
    assert isinstance(arm.rt_send_overruns, int)
    assert isinstance(arm.rt_read_overruns, int)


def test_mode_switch_rejected_while_loop_running(arm):
    import numpy as np
    arm.enable()
    arm.mode_mit()
    arm.set_targets(np.zeros(arm.num_joints))
    arm.start_rt_loop()
    try:
        with pytest.raises(RuntimeError):
            arm.mode_pos_vel()          # 循环运行中切模式应被拒绝
    finally:
        arm.stop_control_loop()


def test_can_timeout_watchdog(arm):
    # Damiao/RobStride 支持；返回成功设置的电机数
    n_set = arm.set_can_timeout_ms(200)
    assert isinstance(n_set, int) and 0 <= n_set <= arm.num_joints
    arm.set_can_timeout_ms(0)           # 禁用看门狗


def test_bad_rate_rejected(arm):
    for bad in (0.0, -1.0, float("nan"), float("inf"), 1e9):
        with pytest.raises(ValueError):
            arm.start_rt_loop(rate=bad)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
