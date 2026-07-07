#!/usr/bin/env python3
"""Replay a recorded trajectory and collect dynamics identification data.

The output CSV is directly consumable by ``13_identify_dynamics.py``.
By default this script only previews the recorded trajectory range. Add
``--execute`` to move the real arm.
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

SOURCE_PYTHON = Path(__file__).resolve().parents[2] / "python"
if SOURCE_PYTHON.exists() and str(SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(SOURCE_PYTHON))

from rebotarm_control_rt.actuator import RobotArm
from rebotarm_control_rt.identification.trajectory import (
    load_recorded_trajectory_csv,
    trajectory_summary,
)
from _example_config import add_port_argument, config_with_port


def _print_summary(traj) -> None:
    summary = trajectory_summary(traj)
    print(f"samples={traj.samples} dof={traj.dof} duration={traj.time[-1]:.3f}s dt={traj.dt:.4f}s")
    print(f"q min/max [deg]:")
    for i, (lo, hi) in enumerate(zip(summary["q_min"], summary["q_max"]), start=1):
        print(f"  joint{i}: {np.degrees(lo):+.2f} .. {np.degrees(hi):+.2f}")
    print("max |dq| [deg/s]:", [round(float(v), 2) for v in np.degrees(summary["dq_abs_max"])])


def _resample_rows(rows: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]], fps: float | None):
    if fps is None or fps <= 0 or len(rows) < 2:
        return rows
    t0 = rows[0][0]
    t_end = rows[-1][0]
    target_t = np.arange(t0, t_end + 1e-12, 1.0 / fps)
    out = []
    idx = 0
    for t in target_t:
        while idx + 1 < len(rows) and rows[idx + 1][0] <= t:
            idx += 1
        out.append(rows[idx])
    return out


def _differentiate_velocity(time_s: np.ndarray, vel: np.ndarray) -> np.ndarray:
    ddq = np.zeros_like(vel)
    if len(time_s) < 3:
        return ddq
    for j in range(vel.shape[1]):
        ddq[:, j] = np.gradient(vel[:, j], time_s, edge_order=1)
    return ddq


def _write_identification_csv(
    path: str | Path,
    rows: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]],
    *,
    output_fps: float | None,
) -> Path:
    rows = _resample_rows(rows, output_fps)
    if not rows:
        raise ValueError("no samples collected")
    time_s = np.array([row[0] for row in rows], dtype=float)
    q = np.vstack([row[1] for row in rows])
    dq = np.vstack([row[2] for row in rows])
    tau = np.vstack([row[3] for row in rows])
    ddq = _differentiate_velocity(time_s, dq)

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    dof = q.shape[1]
    header = ["time"]
    for prefix in ("q", "dq", "ddq", "tau"):
        header.extend(f"{prefix}{i}" for i in range(1, dof + 1))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        t0 = time_s[0]
        for i in range(len(time_s)):
            row = [float(time_s[i] - t0)]
            row.extend(float(v) for v in q[i])
            row.extend(float(v) for v in dq[i])
            row.extend(float(v) for v in ddq[i])
            row.extend(float(v) for v in tau[i])
            writer.writerow(row)
    return path


def _move_to_start(arm, q_start: np.ndarray, vlim: np.ndarray, rate_hz: float, threshold_rad: float) -> None:
    q_cur = np.asarray(arm.get_positions(request=True), dtype=float)
    max_delta = float(np.max(np.abs(q_start - q_cur)))
    if max_delta <= threshold_rad:
        arm.set_targets(pos=q_start.tolist(), vlim=vlim.tolist())
        return
    duration = max(max_delta / max(float(np.min(vlim)), 1e-6), 1.0)
    steps = max(2, int(np.ceil(duration * rate_hz)))
    print(f"[pre] moving to trajectory start in {duration:.2f}s ({steps} steps)")
    for i in range(steps + 1):
        alpha = i / steps
        # Smoothstep interpolation.
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        target = (1.0 - alpha) * q_cur + alpha * q_start
        arm.set_targets(pos=target.tolist(), vlim=vlim.tolist())
        time.sleep(1.0 / rate_hz)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", required=True, help="Recorded trajectory CSV from 11_record_gravity_trajectory.py.")
    parser.add_argument("--output", default="calibration/id_data_train.csv", help="Output identification CSV.")
    parser.add_argument("--config", "-c", default=None, help="Path to arm YAML config.")
    add_port_argument(parser)
    parser.add_argument("--rate", type=float, default=150.0, help="RT command loop rate in Hz.")
    parser.add_argument("--feedback-rate", type=float, default=300.0, help="Python sampling rate in Hz.")
    parser.add_argument("--output-fps", type=float, default=None, help="Optional downsampled output rate.")
    parser.add_argument("--rt-priority", type=int, default=0, help="Best-effort SCHED_FIFO priority.")
    parser.add_argument("--cpu", type=int, default=None, help="Optional CPU affinity.")
    parser.add_argument("--vlim", type=float, default=0.8, help="POS_VEL command velocity limit in rad/s.")
    parser.add_argument("--start-threshold-deg", type=float, default=2.0, help="Skip pre-move if start error is below this.")
    parser.add_argument("--settle-s", type=float, default=1.0, help="Hold start pose before recording.")
    parser.add_argument("--execute", action="store_true", help="Actually move the arm. Without this, only preview.")
    args = parser.parse_args()

    traj = load_recorded_trajectory_csv(args.trajectory)
    print("=" * 72)
    print("  reBotArm recorded trajectory replay and identification data collection")
    print("=" * 72)
    _print_summary(traj)
    print(f"output: {args.output}")
    if not args.execute:
        print("\n[dry-run] Add --execute to move the real arm and collect data.")
        return

    arm = RobotArm(config_with_port(args.config, args.port))
    rows: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []
    vlim = np.full(traj.dof, float(args.vlim), dtype=float)
    try:
        arm.connect()
        print("[connect] OK")
        arm.enable()
        print("[enable] OK")
        arm.mode_pos_vel(vlim=vlim.tolist())
        print("[mode] POS_VEL")
        arm.start_rt_loop(rate=args.rate, rt_priority=args.rt_priority, cpu=args.cpu, request_feedback=False)
        print(f"[rt] started at {args.rate:.1f} Hz")

        _move_to_start(arm, traj.q[0], vlim, args.rate, np.radians(args.start_threshold_deg))
        if args.settle_s > 0:
            print(f"[pre] settling {args.settle_s:.2f}s")
            end = time.monotonic() + args.settle_s
            while time.monotonic() < end:
                arm.set_targets(pos=traj.q[0].tolist(), vlim=vlim.tolist())
                time.sleep(1.0 / args.rate)

        print("[record] executing trajectory...")
        start = time.monotonic()
        next_sample = start
        sample_period = 1.0 / args.feedback_rate
        target_index = 0
        while target_index < traj.samples:
            now = time.monotonic()
            elapsed = now - start
            while target_index + 1 < traj.samples and traj.time[target_index + 1] <= elapsed:
                target_index += 1
            arm.set_targets(pos=traj.q[target_index].tolist(), vlim=vlim.tolist())

            if now >= next_sample:
                pos, vel, torque = arm.get_state(request=True)
                rows.append((now, np.asarray(pos, dtype=float), np.asarray(vel, dtype=float), np.asarray(torque, dtype=float)))
                next_sample += sample_period

            if elapsed >= traj.time[-1]:
                break
            time.sleep(max(0.0, min(0.002, traj.time[target_index] - elapsed)))

        arm.set_targets(pos=traj.q[-1].tolist(), vlim=vlim.tolist())
        out = _write_identification_csv(args.output, rows, output_fps=args.output_fps)
        print(f"[saved] {out}")
        print(f"[record] collected {len(rows)} raw samples")
    except KeyboardInterrupt:
        if rows:
            out = _write_identification_csv(args.output, rows, output_fps=args.output_fps)
            print(f"\n[interrupted] saved partial data: {out} ({len(rows)} raw samples)")
        raise
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
