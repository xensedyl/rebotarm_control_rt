#!/usr/bin/env python3
"""Verify identified dynamics parameters on a CSV dataset."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from rebotarm_control_rt import _math
from rebotarm_control_rt.identification import (
    build_regression_matrix,
    load_identification_csv,
    load_identification_result,
    load_model_for_identification,
    stack_tau_samples,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, help="Verification CSV.")
    parser.add_argument("--params", required=True, help="YAML produced by 13_identify_dynamics.py.")
    parser.add_argument("--urdf", default=None, help="URDF used to build the regressor. Defaults to YAML input_urdf.")
    args = parser.parse_args()

    result = load_identification_result(args.params)
    urdf = args.urdf or result.get("input_urdf")
    model = load_model_for_identification(urdf)
    dataset = load_identification_csv(args.data)
    Y = build_regression_matrix(
        model,
        dataset,
        include_friction=bool(result.get("include_friction", True)),
        coulomb_eps=float(result.get("coulomb_eps", 1e-3)),
    )
    tau = stack_tau_samples(dataset)

    if result["mode"] == "full":
        beta = np.asarray(result["beta"], dtype=float)
        tau_pred = Y @ beta
    elif result["mode"] == "base":
        selected = np.asarray(result["selected_columns"], dtype=int)
        beta = np.asarray(result["beta_base"], dtype=float)
        tau_pred = Y[:, selected] @ beta
    else:
        raise ValueError(f"unknown identification mode: {result['mode']}")

    metrics = _math.regression_metrics(tau, tau_pred, dataset.dof)
    print(
        "verify: "
        f"samples={dataset.samples} rmse={metrics.rmse:.6g} mae={metrics.mae:.6g} "
        f"max_abs={metrics.max_abs:.6g} r2={metrics.r2:.6g}"
    )
    print(f"per-joint RMSE: {np.asarray(metrics.per_joint_rmse)}")
    print(f"per-joint MAE:  {np.asarray(metrics.per_joint_mae)}")


if __name__ == "__main__":
    main()
