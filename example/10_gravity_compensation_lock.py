#!/usr/bin/env python3
"""Gravity compensation with end-effector velocity lock.

This mirrors reBotArm_control_py's lock demo, but uses the RT package's C++
Pinocchio bindings instead of Python pinocchio.

When the end-effector velocity is below thresholds, q_target stays locked.
When the arm is pushed fast enough, q_target is updated to the current pose.
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from pathlib import Path

import numpy as np

SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if SOURCE_PYTHON.exists() and str(SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(SOURCE_PYTHON))

from rebotarm_control_rt.actuator import RobotArm
from rebotarm_control_rt.dynamics import (
    compute_generalized_gravity,
    get_default_gravity,
    load_dynamics_model,
)
from rebotarm_control_rt.kinematics import load_robot_model


_running = True


def _sigint_handler(signum, frame) -> None:
    global _running
    print("\n[gravity_lock] stopping...")
    _running = False


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", "-c", default=None, help="Path to arm YAML config.")
    parser.add_argument("--rate", type=float, default=200.0, help="Python control rate in Hz.")
    parser.add_argument("--vel-threshold", type=float, default=0.04, help="Linear velocity threshold.")
    parser.add_argument("--w-threshold", type=float, default=0.08, help="Angular velocity threshold.")
    parser.add_argument("--kp", type=float, default=8.0, help="MIT lock stiffness.")
    parser.add_argument("--kd", type=float, default=1.0, help="MIT lock damping.")
    parser.add_argument("--integral-limit", type=float, default=0.5, help="Integral torque clamp.")
    args = parser.parse_args()

    signal.signal(signal.SIGINT, _sigint_handler)

    kin_model = load_robot_model()
    dyn_model = load_dynamics_model()
    frame_id = kin_model.end_effector_frame_id()

    print("=" * 65)
    print("  reBotArm RT gravity compensation with velocity lock")
    print("=" * 65)
    print(f"[model] nq={dyn_model.nq}, nv={dyn_model.nv}")
    print(f"[gravity] {get_default_gravity()} m/s^2")
    print(f"[thresholds] v={args.vel_threshold} m/s, w={args.w_threshold} rad/s")
    print("Ctrl+C to stop and disconnect.")
    print("-" * 65)

    arm = RobotArm(args.config)
    q_target: np.ndarray | None = None
    integral: np.ndarray | None = None
    counter = 0
    lock_counter = 0
    dt = 1.0 / args.rate

    try:
        arm.connect()
        print("[connect] OK")
        arm.enable()
        print("[enable] OK")

        q_target = np.asarray(arm.get_positions(request=True), dtype=float)
        integral = np.zeros_like(q_target)
        print(f"[target] initial lock: {np.degrees(q_target).round(2)} deg")

        kp = np.full(arm.num_joints, args.kp, dtype=float).tolist()
        kd = np.full(arm.num_joints, args.kd, dtype=float).tolist()
        arm.mode_mit(kp=kp, kd=kd)
        print(f"[MIT mode] kp={args.kp}, kd={args.kd}")

        while _running:
            t0 = time.perf_counter()
            q = np.asarray(arm.get_positions(request=True), dtype=float)
            qd = np.asarray(arm.get_velocities(), dtype=float)
            tau_g = np.asarray(compute_generalized_gravity(dyn_model, q), dtype=float)

            q_error = q_target - q
            integral += q_error * dt
            np.clip(integral, -args.integral_limit, args.integral_limit, out=integral)

            jacobian = np.asarray(kin_model.frame_jacobian(q, frame_id), dtype=float)
            v_spatial = jacobian @ qd
            v_norm = float(np.linalg.norm(v_spatial[:3]))
            w_norm = float(np.linalg.norm(v_spatial[3:]))

            if v_norm > args.vel_threshold or w_norm > args.w_threshold:
                q_target = q.copy()
                lock_counter = 0
                integral *= 0.9
            else:
                lock_counter += 1

            arm.mit(
                pos=q_target.tolist(),
                vel=np.zeros(arm.num_joints).tolist(),
                kp=kp,
                kd=kd,
                tau=(tau_g + integral).tolist(),
                request_feedback=False,
            )

            counter += 1
            if counter % 20 == 0:
                state = "LOCKED" if lock_counter > 0 else "UPDATE"
                print(
                    f"[{counter:4d}] {state} "
                    f"err_max={float(np.max(np.abs(q_error))):.4f}rad "
                    f"v={v_norm:.4f}m/s w={w_norm:.4f}rad/s "
                    f"tau_g=" + " ".join(f"{float(t):+.3f}" for t in tau_g)
                )

            elapsed = time.perf_counter() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)
    finally:
        print("\n[stop] disconnecting...")
        arm.disconnect()
        print("[done] disconnected")


if __name__ == "__main__":
    main()
