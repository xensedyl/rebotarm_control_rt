"""Recorded joint trajectory utilities for dynamics identification."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv

import numpy as np


@dataclass
class RecordedTrajectory:
    time: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    torque: np.ndarray | None = None

    @property
    def samples(self) -> int:
        return int(self.q.shape[0])

    @property
    def dof(self) -> int:
        return int(self.q.shape[1])

    @property
    def dt(self) -> float:
        if self.samples < 2:
            return 0.0
        return float(np.median(np.diff(self.time)))


def trajectory_summary(traj: RecordedTrajectory) -> dict[str, np.ndarray | float]:
    return {
        "q_min": np.min(traj.q, axis=0),
        "q_max": np.max(traj.q, axis=0),
        "dq_abs_max": np.max(np.abs(traj.dq), axis=0),
    }


def save_recorded_trajectory_csv(path: str | Path, traj: RecordedTrajectory) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ["time"]
    for prefix in ("q", "dq"):
        header.extend(f"{prefix}{i}" for i in range(1, traj.dof + 1))
    if traj.torque is not None:
        header.extend(f"tau{i}" for i in range(1, traj.dof + 1))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(traj.samples):
            row = [float(traj.time[i])]
            row.extend(float(v) for v in traj.q[i])
            row.extend(float(v) for v in traj.dq[i])
            if traj.torque is not None:
                row.extend(float(v) for v in traj.torque[i])
            writer.writerow(row)
    return path


def load_recorded_trajectory_csv(path: str | Path, dof: int | None = None) -> RecordedTrajectory:
    path = Path(path)
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = [cell.strip() for cell in next(reader)]
        rows = [[float(cell) for cell in row] for row in reader if row]
    if not rows:
        raise ValueError(f"{path} contains no samples")
    data = np.asarray(rows, dtype=float)
    if dof is None:
        dof = len([name for name in header if name.startswith("q") and name[1:].isdigit()])
    cols = {}
    for prefix in ("q", "dq"):
        indices = []
        for i in range(1, dof + 1):
            name = f"{prefix}{i}"
            if name not in header:
                raise ValueError(f"missing column {name}")
            indices.append(header.index(name))
        cols[prefix] = indices
    tau_cols = []
    for i in range(1, dof + 1):
        name = f"tau{i}"
        if name in header:
            tau_cols.append(header.index(name))
    torque = data[:, tau_cols] if len(tau_cols) == dof else None
    return RecordedTrajectory(
        time=data[:, header.index("time")] if "time" in header else np.arange(data.shape[0], dtype=float),
        q=data[:, cols["q"]],
        dq=data[:, cols["dq"]],
        torque=torque,
    )


__all__ = [
    "RecordedTrajectory",
    "trajectory_summary",
    "save_recorded_trajectory_csv",
    "load_recorded_trajectory_csv",
]
