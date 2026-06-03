#!/usr/bin/env python3
"""reBotArm RT-native MIT 控制示例（控制循环跑在 Rust 原生线程，全程释放 GIL）。

与兼容模式（start_control_loop + Python 回调）不同，这里 Python 只负责
用 set_targets() 原子更新目标，Rust 后台线程以配置频率持续下发 MIT 指令并请求反馈。

用法:
    python example/rt_mit_control.py
输入: n 个关节角度（度），空格分隔；q 退出，state 查看状态。
"""
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from rebotarm_control_rt.actuator import RobotArm

arm = RobotArm()
arm.connect()
print("--- 连接成功 ---")
arm.enable()
print("--- 使能成功 ---")
arm.mode_mit()
print("--- MIT 模式 ---")

n = arm.num_joints
# 安全：不预设目标，start_rt_loop 会读取当前关节位置作为 hold 目标（不会拉向 0 位姿）。
# 反馈按 100Hz 抽取以降低总线压力；如有 root 权限可加 rt_priority=80 申请 SCHED_FIFO。
arm.start_rt_loop(feedback_rate=100.0)   # Rust 原生 RT 线程启动（hold 当前位置）
print(f"关节数: {n} | {arm._rate}Hz RT 循环已启动（保持当前位置）")
print("输入 n 个角度(度)  q退出  state查看状态\n")

try:
    while True:
        line = input("> ").strip()
        if not line:
            continue
        if line.lower() in ("q", "quit", "exit"):
            break
        if line.lower() == "state":
            pos = arm.get_positions()
            vel = arm.get_velocities()
            print(f"  pos: {[f'{x:+.2f}' for x in np.degrees(pos)]}")
            print(f"  vel: {[f'{x:+.2f}' for x in np.degrees(vel)]}")
            continue
        tokens = line.split()
        if len(tokens) < n:
            print(f"需要 {n} 个值")
            continue
        target = np.radians([float(x) for x in tokens[:n]])
        arm.set_targets(pos=target)      # 仅更新目标，RT 线程自动跟踪
        print(f"  -> {[f'{x:+.1f}' for x in np.degrees(target)]}")
except (EOFError, KeyboardInterrupt):
    pass

arm.disconnect()
