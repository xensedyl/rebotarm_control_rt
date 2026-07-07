#!/usr/bin/env python3
"""Record a hand-guided trajectory under gravity compensation.

This is the preferred way to create a safe identification replay trajectory:
the operator physically drags the real arm through free space, so the path has
already been checked against the local scene.
"""
from __future__ import annotations

import argparse
import signal
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

SOURCE_PYTHON = Path(__file__).resolve().parents[2] / "python"
if SOURCE_PYTHON.exists() and str(SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(SOURCE_PYTHON))

from rebotarm_control_rt.actuator import RobotArm
from rebotarm_control_rt.calibration.free_drive import FreeDrive
from rebotarm_control_rt.dynamics import get_default_gravity, load_dynamics_model
from rebotarm_control_rt.identification.trajectory import (
    RecordedTrajectory,
    save_recorded_trajectory_csv,
    trajectory_summary,
)
from rebotarm_control_rt.paths import resolve_urdf_path
from _example_config import add_port_argument, config_with_port, model_urdf_for_config


END_LINK_LOAD_SCALE_WITH_GRIPPER = 0.7


def _str_to_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    value = value.strip().lower()
    if value in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if value in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def _end_link_load_urdf(urdf_path: str | Path | None, scale: float) -> str:
    if scale < 0.0:
        raise ValueError("--end-link-load-scale must be >= 0")
    tree = ET.parse(resolve_urdf_path(urdf_path))
    root = tree.getroot()
    end_link = root.find("./link[@name='end_link']")
    inertial = end_link.find("inertial") if end_link is not None else None
    if end_link is None or inertial is None:
        raise RuntimeError("URDF does not contain end_link inertial to scale")
    if scale == 0.0:
        end_link.remove(inertial)
    else:
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is None or inertia is None:
            raise RuntimeError("URDF end_link inertial is missing mass or inertia")
        mass.set("value", str(float(mass.attrib["value"]) * scale))
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            inertia.set(key, str(float(inertia.attrib[key]) * scale))
    tmp = tempfile.NamedTemporaryFile("wb", suffix=".urdf", delete=False)
    with tmp:
        tree.write(tmp, encoding="utf-8", xml_declaration=True)
    return tmp.name


def _has_end_link_inertial(urdf_path: str | Path | None) -> bool:
    root = ET.parse(resolve_urdf_path(urdf_path)).getroot()
    end_link = root.find("./link[@name='end_link']")
    return end_link is not None and end_link.find("inertial") is not None


def _default_end_link_load_scale(use_gripper: bool, urdf_path: str | Path | None) -> float:
    if not use_gripper:
        return 0.0
    # The built-in B601 URDF keeps the historical tested load scale. Explicit
    # URDFs are assumed to already contain the user's calibrated/identified load.
    return 1.0 if urdf_path else END_LINK_LOAD_SCALE_WITH_GRIPPER


def _load_gravity_model(urdf_path: str | Path | None, use_gripper: bool, end_link_load_scale: float | None):
    scale = _default_end_link_load_scale(use_gripper, urdf_path) if end_link_load_scale is None else end_link_load_scale
    if scale == 1.0:
        return load_dynamics_model(None if urdf_path is None else str(urdf_path)), scale
    if not _has_end_link_inertial(urdf_path):
        return load_dynamics_model(None if urdf_path is None else str(urdf_path)), 1.0
    tmp_urdf = _end_link_load_urdf(urdf_path, scale)
    try:
        return load_dynamics_model(tmp_urdf), scale
    finally:
        Path(tmp_urdf).unlink(missing_ok=True)


def _input_worker(start_event: threading.Event, record_stop_event: threading.Event) -> None:
    try:
        input("Press Enter to START recording after gravity compensation feels stable...\n")
        start_event.set()
        input("Recording. Press Enter to STOP recording and save. Gravity compensation will keep running...\n")
        record_stop_event.set()
    except EOFError:
        record_stop_event.set()


def _print_summary(traj: RecordedTrajectory) -> None:
    summary = trajectory_summary(traj)
    print(f"samples={traj.samples} dof={traj.dof} duration={traj.time[-1]:.3f}s dt~={traj.dt:.4f}s")
    print("q min/max [deg]:")
    for i, (lo, hi) in enumerate(zip(summary["q_min"], summary["q_max"]), start=1):
        print(f"  joint{i}: {np.degrees(lo):+.2f} .. {np.degrees(hi):+.2f}")
    print("max |dq| [deg/s]:", [round(float(v), 2) for v in np.degrees(summary["dq_abs_max"])])


def _release_mit_torque_hold(arm, frames: int = 10, dt_s: float = 0.02) -> None:
    q_all = np.asarray(arm.get_positions(request=True), dtype=float)
    n = arm.num_joints
    if q_all.size != n:
        q_all = np.resize(q_all, n)
    zeros = np.zeros(n, dtype=float).tolist()
    pos = q_all.tolist()
    for _ in range(frames):
        arm.mit(pos=pos, vel=zeros, kp=zeros, kd=zeros, tau=zeros, request_feedback=False)
        time.sleep(dt_s)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="calibration/recorded_trajectory.csv", help="Output replay trajectory CSV.")
    parser.add_argument("--config", "-c", default=None, help="Path to arm YAML config.")
    parser.add_argument(
        "--urdf",
        default=None,
        help="Dynamics URDF used for gravity compensation. Defaults to the config URDF or SDK URDF.",
    )
    add_port_argument(parser)
    parser.add_argument("--rate", type=float, default=200.0, help="Gravity compensation loop rate in Hz.")
    parser.add_argument("--sample-rate", type=float, default=100.0, help="Trajectory recording sample rate in Hz.")
    parser.add_argument("--kd", type=float, default=1.0, help="MIT damping during free-drive.")
    parser.add_argument("--gravity-scale", type=float, default=1.0, help="Gravity feed-forward torque multiplier.")
    parser.add_argument(
        "--use_gripper",
        "--use-gripper",
        dest="use_gripper",
        type=_str_to_bool,
        default=True,
        metavar="{true,false}",
        help="Whether to include the fixed end_link gripper load in the gravity model.",
    )
    parser.add_argument(
        "--end-link-load-scale",
        type=float,
        default=None,
        help=(
            "Scale end_link inertial for gravity compensation. Default: 0.7 for the built-in "
            "URDF with --use_gripper=true, 1.0 for an explicit --urdf, and 0.0 when "
            "--use_gripper=false."
        ),
    )
    parser.add_argument("--max-duration-s", type=float, default=180.0, help="Safety stop for recording duration.")
    args = parser.parse_args()

    if args.sample_rate <= 0.0:
        raise ValueError("--sample-rate must be positive")
    if args.max_duration_s <= 0.0:
        raise ValueError("--max-duration-s must be positive")

    model_urdf = model_urdf_for_config(args.config, args.urdf)
    model, end_link_scale = _load_gravity_model(model_urdf, args.use_gripper, args.end_link_load_scale)
    print("=" * 72)
    print("  reBotArm hand-guided trajectory recording")
    print("=" * 72)
    print(f"[urdf] {resolve_urdf_path(model_urdf)}")
    print(f"[model] nq={model.nq}, nv={model.nv}")
    print(f"[gravity] {get_default_gravity()} m/s^2")
    print(f"[gripper/end_link load] scale={end_link_scale:.3f}")
    print(f"[free-drive] rate={args.rate:.1f} Hz, kd={args.kd}, gravity_scale={args.gravity_scale}")
    print(f"[record] sample_rate={args.sample_rate:.1f} Hz, max_duration={args.max_duration_s:.1f}s")
    print("After saving, gravity compensation keeps running. Drag the arm back to zero, then Ctrl+C to disconnect.")
    print("-" * 72)

    arm = RobotArm(config_with_port(args.config, args.port))
    rows: list[tuple[float, np.ndarray, np.ndarray, np.ndarray]] = []
    start_event = threading.Event()
    record_stop_event = threading.Event()
    exit_event = threading.Event()
    connected = False

    def sigint_handler(signum, frame) -> None:
        print("\n[exit] Ctrl+C received. Stopping gravity compensation and disconnecting...")
        exit_event.set()

    old_handler = signal.signal(signal.SIGINT, sigint_handler)
    try:
        arm.connect()
        connected = True
        print("[connect] OK")
        arm.enable()
        print("[enable] OK")

        if arm.num_joints < model.nq:
            raise ValueError(f"arm config has {arm.num_joints} joints, dynamics model requires {model.nq}")

        with FreeDrive(
            arm,
            model,
            rate=args.rate,
            kd=args.kd,
            gravity_scale=args.gravity_scale,
            model_joints=model.nq,
            request_feedback=True,
        ):
            print("[free-drive] started. Drag the arm by hand.")
            thread = threading.Thread(target=_input_worker, args=(start_event, record_stop_event), daemon=True)
            thread.start()

            while not start_event.is_set() and not exit_event.is_set():
                time.sleep(0.02)
            if exit_event.is_set():
                return

            print("[record] started.")
            t0 = time.monotonic()
            next_sample = t0
            period = 1.0 / args.sample_rate
            while not record_stop_event.is_set() and not exit_event.is_set():
                now = time.monotonic()
                if now - t0 >= args.max_duration_s:
                    print("[record] max duration reached.")
                    break
                if now >= next_sample:
                    pos, vel, torque = arm.get_state(request=True)
                    rows.append(
                        (
                            now - t0,
                            np.asarray(pos, dtype=float),
                            np.asarray(vel, dtype=float),
                            np.asarray(torque, dtype=float),
                        )
                    )
                    next_sample += period
                time.sleep(0.001)

            if rows:
                time_s = np.array([row[0] for row in rows], dtype=float)
                q = np.vstack([row[1] for row in rows])
                dq = np.vstack([row[2] for row in rows])
                torque = np.vstack([row[3] for row in rows])
                traj = RecordedTrajectory(time=time_s, q=q, dq=dq, torque=torque)
                out = save_recorded_trajectory_csv(args.output, traj)
                print(f"[saved] {out}")
                _print_summary(traj)
            else:
                print("[record] no samples captured.")

            if not exit_event.is_set():
                print("[free-drive] still running. Drag the arm back to zero, then press Ctrl+C to exit.")
            while not exit_event.is_set():
                time.sleep(0.05)
    finally:
        signal.signal(signal.SIGINT, old_handler)
        print("\n[stop] disconnecting...")
        if connected:
            try:
                arm.stop_control_loop()
                _release_mit_torque_hold(arm)
            except Exception as exc:
                print(f"[stop] failed to release MIT torque cleanly: {exc}")
            arm.disconnect()
        print("[done] disconnected")


if __name__ == "__main__":
    main()
