#!/usr/bin/env python3
"""灵足（RobStride）电机底层控制测试 —— 通过 rebotarm_control_rt RobotArm。

Single LingZu (RobStride) motor test through rebotarm_control_rt RobotArm.

对照 reBotArm_control_py 的 example/0x01rs06_test.py（直接用 motorbridge SDK），
本示例使用 RT RobotArm API，控制选定关节并保持其余关节。

注意 / Note:
    灵足电机使用 CAN 总线通信（如 can0），不是串口。
    LingZu (RobStride) uses CAN bus (e.g. can0), not serial.
    运行前确认 CAN 接口已配置:
        sudo ip link set can0 up type can bitrate 1000000

用法 / Usage:
    python example/python/0x03robstride_test.py [--config ...] [--port can0] [--joint 0]

交互命令 / Interactive commands:
    enable / disable                       — 使能 / 去使能
    ping                                   — ping 电机获取响应
    clear_error                            — 清除电机错误
    set_zero                               — 电机零位设置
    mode <mit|posvel|vel>                  — 切换控制模式
    mit <pos_deg> [vel kp kd tau]          — MIT 模式指令
    posvel <pos_deg> [vlim_rad_s]          — POS_VEL 模式指令
    csp <pos_deg> [vlim_rad_s]             — 灵足原生 CSP 位置模式（单关节）
    vel <vel_rad_s>                        — 纯速度模式指令
    state                                  — 打印当前状态
    report <on|off>                        — 开/关主动状态上报
    read_param <param_id> [type]           — 读取参数（默认 f32）
    write_param <param_id> <value> [type]  — 写入参数（默认 f32）
    save_params                            — 保存参数（断电保持）
    q / quit                               — 退出
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

SOURCE_PYTHON = Path(__file__).resolve().parents[2] / "python"
if SOURCE_PYTHON.exists() and str(SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(SOURCE_PYTHON))

from rebotarm_control_rt.actuator import RobotArm
from _example_config import add_port_argument, config_with_port

_CONFIG_DIR = Path(__file__).resolve().parents[2] / "python" / "rebotarm_control_rt" / "config"
DEFAULT_CONFIG = str(_CONFIG_DIR / "arm_rs.yaml")


def joint_index(names: list[str], joint: str) -> int:
    try:
        idx = int(joint)
    except ValueError:
        if joint not in names:
            raise ValueError(f"unknown joint {joint!r}; available: {names}") from None
        idx = names.index(joint)
    if idx < 0 or idx >= len(names):
        raise ValueError(f"joint index {idx} out of range 0..{len(names) - 1}")
    return idx


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config", "-c", default=DEFAULT_CONFIG,
        help="Path to arm YAML config. Default: packaged arm_rs.yaml (LingZu / RobStride).",
    )
    add_port_argument(parser)
    parser.add_argument("--joint", default="0", help="Joint index or name to control. Default: 0.")
    parser.add_argument("--rate", type=float, default=150.0, help="RT loop rate for target-cache modes.")
    parser.add_argument("--rt-priority", type=int, default=0, help="Best-effort SCHED_FIFO priority.")
    parser.add_argument("--cpu", type=int, default=None, help="Optional CPU affinity.")
    args = parser.parse_args()

    arm = RobotArm(config_with_port(args.config, args.port))
    try:
        arm.connect()
        names = list(arm.joint_names)
        idx = joint_index(names, args.joint)
        name = names[idx]
        print(f"connected: {arm.name}")
        print(f"joint: {idx} ({name}); all joints: {names}")

        # 灵足电机没有单帧状态查询命令，开启主动上报后
        # motorbridge 后台轮询会持续更新状态缓存。
        for jn in names:
            try:
                arm.robstride_set_active_report(jn, True)
            except Exception as exc:
                print(f"[active_report/{jn}] {exc}")

        target_pos = np.asarray(arm.get_positions(request=True), dtype=float)
        mit_kp = np.array([j.kp for j in arm._joints], dtype=float)
        mit_kd = np.array([j.kd for j in arm._joints], dtype=float)
        pv_vlim = np.array([j.vlim for j in arm._joints], dtype=float)
        mode = "mit"
        rt_started = False

        def start_rt_if_needed() -> None:
            nonlocal rt_started
            if rt_started:
                return
            arm.start_rt_loop(rate=args.rate, rt_priority=args.rt_priority, cpu=args.cpu)
            rt_started = True
            print(f"RT loop started @ {args.rate} Hz")

        def stop_rt_if_running() -> None:
            nonlocal rt_started
            if rt_started:
                arm.stop_control_loop()
                rt_started = False

        def do_enable() -> None:
            arm.enable()
            print("enabled")

        def do_disable() -> None:
            stop_rt_if_running()
            arm.disable()
            print("disabled")

        def do_ping() -> None:
            try:
                device_id, responder_id = arm.robstride_ping(name)
                print(f"ping OK: device_id={device_id} responder_id={responder_id:#04x}")
            except Exception as exc:
                print(f"ping failed: {exc}")

        def do_clear_error() -> None:
            arm.clear_error(name)
            print("error cleared")

        def do_set_zero() -> None:
            answer = input(f"Set current {name} as zero? Type YES: ").strip()
            if answer != "YES":
                print("aborted")
                return
            stop_rt_if_running()
            arm.set_zero_single(name)
            print("zero set")

        def do_mode(values: list[str]) -> None:
            nonlocal mode
            if not values:
                print("usage: mode <mit|posvel|vel>")
                return
            stop_rt_if_running()
            m = values[0].lower()
            if m == "mit":
                arm.mode_mit(kp=mit_kp.tolist(), kd=mit_kd.tolist())
                mode = "mit"
            elif m == "posvel":
                # 灵足：mode_pos_vel 内部把 vlim/vel_kp/vel_ki/pos_kp 写入
                # 0x7017/0x701F/0x7020/0x701E 参数表后切 run_mode=1。
                arm.mode_pos_vel(vlim=pv_vlim.tolist())
                mode = "posvel"
            elif m == "vel":
                arm.mode_vel()
                mode = "vel"
            else:
                print("available modes: mit / posvel / vel")
                return
            print(f"mode: {mode}")

        def do_state() -> None:
            pos, vel, torq = arm.get_state(request=True)
            print(
                f"{name}: pos={math.degrees(float(pos[idx])):+.4f}deg  "
                f"vel={math.degrees(float(vel[idx])):+.4f}deg/s  "
                f"torq={float(torq[idx]):+.4f}  mode={mode}"
            )

        def do_mit(values: list[str]) -> None:
            nonlocal mode, target_pos
            if not values:
                print("usage: mit <pos_deg> [vel_rad_s kp kd tau]")
                return
            if mode != "mit":
                do_mode(["mit"])
            target_pos[idx] = math.radians(float(values[0]))
            vel = np.zeros(arm.num_joints, dtype=float)
            tau = np.zeros(arm.num_joints, dtype=float)
            if len(values) > 1:
                vel[idx] = float(values[1])
            if len(values) > 2:
                mit_kp[idx] = float(values[2])
            if len(values) > 3:
                mit_kd[idx] = float(values[3])
            if len(values) > 4:
                tau[idx] = float(values[4])
            arm.set_targets(
                pos=target_pos.tolist(),
                vel=vel.tolist(),
                kp=mit_kp.tolist(),
                kd=mit_kd.tolist(),
                tau=tau.tolist(),
            )
            start_rt_if_needed()
            print(f"target {name}: {float(values[0]):+.2f} deg  kp={mit_kp[idx]:.2f} kd={mit_kd[idx]:.2f}")

        def do_posvel(values: list[str]) -> None:
            nonlocal mode, target_pos
            if not values:
                print("usage: posvel <pos_deg> [vlim_rad_s]")
                return
            if mode != "posvel":
                do_mode(["posvel"])
            target_pos[idx] = math.radians(float(values[0]))
            if len(values) > 1:
                pv_vlim[idx] = float(values[1])
            arm.set_targets(pos=target_pos.tolist(), vlim=pv_vlim.tolist())
            start_rt_if_needed()
            print(f"target {name}: {float(values[0]):+.2f} deg  vlim={pv_vlim[idx]:.3f} rad/s")

        def do_csp(values: list[str]) -> None:
            if not values:
                print("usage: csp <pos_deg> [vlim_rad_s]")
                return
            stop_rt_if_running()
            pos = math.radians(float(values[0]))
            vlim = float(values[1]) if len(values) > 1 else 1.0
            # 灵足原生 CSP 位置模式（run_mode=5），仅驱动选定关节。
            arm.robstride_pos_vel_csp(name, pos, vlim)
            print(f"CSP {name}: pos={pos:.4f}rad vlim={vlim}")

        def do_vel(values: list[str]) -> None:
            nonlocal mode
            if not values:
                print("usage: vel <vel_rad_s>")
                return
            if mode != "vel":
                do_mode(["vel"])
            vel = np.zeros(arm.num_joints, dtype=float)
            vel[idx] = float(values[0])
            arm.set_vel(vel.tolist())
            print(f"velocity {name}: {vel[idx]:+.3f} rad/s")

        def do_report(values: list[str]) -> None:
            if not values or values[0].lower() not in {"on", "off"}:
                print("usage: report <on|off>")
                return
            enabled = values[0].lower() == "on"
            arm.robstride_set_active_report(name, enabled)
            print(f"active report: {'on' if enabled else 'off'}")

        def do_read_param(values: list[str]) -> None:
            if not values:
                print("usage: read_param <param_id> [u8|u16|u32|f32]  (e.g. read_param 0x7019)")
                return
            param_id = int(values[0], 0)
            ptype = values[1] if len(values) > 1 else "f32"
            try:
                if ptype == "u8":
                    value = arm.robstride_get_param_u8(name, param_id)
                elif ptype == "u16":
                    value = arm.robstride_get_param_u16(name, param_id)
                elif ptype == "u32":
                    value = arm.robstride_get_param_u32(name, param_id)
                else:
                    value = arm.robstride_get_param_f32(name, param_id)
                print(f"param 0x{param_id:04X} = {value}")
            except Exception as exc:
                print(f"read param failed: {exc}")

        def do_write_param(values: list[str]) -> None:
            if len(values) < 2:
                print("usage: write_param <param_id> <value> [u8|u16|u32|f32]")
                return
            param_id = int(values[0], 0)
            raw = values[1]
            ptype = values[2] if len(values) > 2 else "f32"
            try:
                if ptype == "u8":
                    arm.robstride_write_param_u8(name, param_id, int(raw, 0))
                elif ptype == "u16":
                    arm.robstride_write_param_u16(name, param_id, int(raw, 0))
                elif ptype == "u32":
                    arm.robstride_write_param_u32(name, param_id, int(raw, 0))
                else:
                    arm.robstride_write_param_f32(name, param_id, float(raw))
                print(f"param 0x{param_id:04X} written: {raw}")
            except Exception as exc:
                print(f"write param failed: {exc}")

        def do_save_params() -> None:
            arm.robstride_save_parameters(name)
            print("parameters saved (persist after power cycle)")

        commands = {
            "enable": do_enable,
            "disable": do_disable,
            "ping": do_ping,
            "clear_error": do_clear_error,
            "set_zero": do_set_zero,
            "mode": do_mode,
            "state": do_state,
            "mit": do_mit,
            "posvel": do_posvel,
            "csp": do_csp,
            "vel": do_vel,
            "report": do_report,
            "read_param": do_read_param,
            "write_param": do_write_param,
            "save_params": do_save_params,
        }
        no_arg = {"enable", "disable", "ping", "clear_error", "set_zero", "state", "save_params"}

        print(
            "commands: enable / disable / ping / clear_error / set_zero / mode / "
            "mit / posvel / csp / vel / state / report / read_param / write_param / save_params / q"
        )
        print("examples: mit 10 0 50 3 0 | posvel 10 1.0 | csp 10 1.0 | vel 0.2 | read_param 0x7019")
        while True:
            try:
                line = input("> ").strip()
            except (KeyboardInterrupt, EOFError):
                print("\n[exit]")
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()
            values = parts[1:]
            if cmd in {"q", "quit", "exit"}:
                break
            fn = commands.get(cmd)
            if fn is None:
                print(f"unknown command: {cmd}")
                continue
            try:
                if cmd in no_arg:
                    fn()
                else:
                    fn(values)
            except Exception as exc:
                print(f"error: {exc}")
    finally:
        try:
            arm.stop_control_loop()
            arm.disconnect()
        except Exception as exc:
            print(f"disconnect error: {exc}", file=sys.stderr)


if __name__ == "__main__":
    main()
