#!/usr/bin/env python3
"""Read Damiao POS_VEL gains from motor registers.

Damiao POS_VEL gain registers:
  25: vel_kp / KP_ASR
  26: vel_ki / KI_ASR
  27: pos_kp / KP_APR
  28: pos_ki / KI_APR
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if SOURCE_PYTHON.exists() and str(SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(SOURCE_PYTHON))

from rebotarm_control_rt.actuator import RobotArm


DEFAULT_BI_CONFIGS = [
    Path.home()
    / ".cache/huggingface/lerobot/calibration/robots/bi_seeed_b601_rt_follower/left_follower_rt_arm.yaml",
    Path.home()
    / ".cache/huggingface/lerobot/calibration/robots/bi_seeed_b601_rt_follower/right_follower_rt_arm.yaml",
]


def read_one(config: str | None, timeout_ms: int) -> None:
    label = config if config else "package default arm.yaml"
    arm = RobotArm(config)
    try:
        arm.connect()
        print(f"\n[{arm.name}] {label}")
        print(f"{'joint':<16} {'vel_kp':>10} {'vel_ki':>10} {'pos_kp':>10} {'pos_ki':>10}")
        gains = arm.get_pos_vel_gains(timeout_ms=timeout_ms)
        for name in arm.joint_names:
            vel_kp, vel_ki, pos_kp, pos_ki = gains[name]
            print(f"{name:<16} {vel_kp:10.6g} {vel_ki:10.6g} {pos_kp:10.6g} {pos_ki:10.6g}")
    finally:
        arm.disconnect(disable=False)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        "-c",
        action="append",
        default=None,
        help="Arm YAML config. Can be passed multiple times for left/right arms.",
    )
    parser.add_argument("--timeout-ms", type=int, default=300, help="Per-register read timeout.")
    parser.add_argument(
        "--default-bi",
        action="store_true",
        help="Read the generated left/right B601 RT configs from the LeRobot cache.",
    )
    args = parser.parse_args()

    configs = args.config
    if args.default_bi:
        missing = [str(path) for path in DEFAULT_BI_CONFIGS if not path.exists()]
        if missing:
            raise FileNotFoundError(
                "Generated bi B601 configs not found: "
                + ", ".join(missing)
                + ". Run lerobot-teleoperate-pico4 once or pass --config explicitly."
            )
        configs = [str(path) for path in DEFAULT_BI_CONFIGS]
    if not configs:
        configs = [None]

    for config in configs:
        read_one(config, args.timeout_ms)


if __name__ == "__main__":
    main()
