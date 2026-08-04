#!/usr/bin/env python3
"""Fit dynamics parameters from an identification CSV dataset."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "python"))

from rebotarm_control_rt.identification import (
    apply_dynamic_parameters_to_urdf,
    apply_payload_parameters_to_urdf,
    fit_dynamics,
    fit_payload_dynamics,
    load_identification_csv,
    load_model_for_identification,
    save_identification_result,
    write_urdf_without_link_inertial,
)
from rebotarm_control_rt.paths import resolve_urdf_path
from _example_config import model_urdf_for_config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", required=True, help="CSV with time,q1..q6,dq1..dq6,ddq1..ddq6,tau1..tau6.")
    parser.add_argument("--config", "-c", default=None, help="Path to arm YAML config.")
    parser.add_argument("--urdf", default=None, help="Input URDF. Defaults to the config URDF or SDK URDF.")
    parser.add_argument(
        "--mode",
        choices=["full", "base", "payload"],
        default="full",
        help="full/base identify the whole model; payload keeps the arm fixed and identifies one fixed payload link.",
    )
    parser.add_argument("--output", default="calibration/identified_dynamics.yaml")
    parser.add_argument("--urdf-output", default=None, help="Write identified inertial parameters to this URDF.")
    parser.add_argument("--no-friction", action="store_true", help="Do not include viscous/Coulomb friction columns.")
    parser.add_argument(
        "--no-model-prior",
        action="store_true",
        help="For full mode, do not keep the URDF null-space parameters close to the input model.",
    )
    parser.add_argument(
        "--ignore-payload-link",
        default=None,
        help="Remove this link's inertial from a temporary URDF before full/base identification, e.g. end_link.",
    )
    parser.add_argument(
        "--fold-fixed-child-inertials",
        action="store_true",
        help="When writing --mode full URDF, fold fixed child inertials into their parent and remove the child inertial.",
    )
    parser.add_argument("--payload-link", default="end_link", help="Payload link used by --mode payload.")
    parser.add_argument(
        "--payload-parameters",
        type=int,
        choices=[4, 10],
        default=4,
        help="Payload parameters to identify: 4 for mass+first moments, 10 for full inertial block.",
    )
    parser.add_argument(
        "--payload-default-mass",
        type=float,
        default=0.5,
        help="Fallback mass if the payload link has no inertial in the input URDF.",
    )
    parser.add_argument(
        "--payload-fd-eps",
        type=float,
        default=1e-5,
        help="Finite difference step ratio for payload regressors.",
    )
    parser.add_argument("--coulomb-eps", type=float, default=1e-3)
    parser.add_argument("--rcond", type=float, default=1e-12)
    args = parser.parse_args()

    dataset = load_identification_csv(args.data)
    input_urdf = resolve_urdf_path(model_urdf_for_config(args.config, args.urdf))
    model_urdf = input_urdf
    temp_urdf_to_remove: Path | None = None

    if args.mode in {"full", "base"} and args.ignore_payload_link:
        temp_urdf_to_remove = Path(args.output).with_suffix(f".no_{args.ignore_payload_link}.urdf")
        model_urdf = write_urdf_without_link_inertial(
            input_urdf,
            temp_urdf_to_remove,
            link_name=args.ignore_payload_link,
        )

    if args.mode == "payload":
        if not args.no_friction:
            print("[warn] --mode payload keeps arm/friction fixed; --no-friction is implied.")
        result = fit_payload_dynamics(
            input_urdf,
            dataset,
            link_name=args.payload_link,
            parameter_count=args.payload_parameters,
            default_mass=args.payload_default_mass,
            finite_difference_eps=args.payload_fd_eps,
            rcond=args.rcond,
        )
    else:
        model = load_model_for_identification(model_urdf)
        result = fit_dynamics(
            model,
            dataset,
            mode=args.mode,
            include_friction=not args.no_friction,
            coulomb_eps=args.coulomb_eps,
            rcond=args.rcond,
            use_model_prior=not args.no_model_prior,
        )
    result["input_data"] = str(Path(args.data))
    result["input_urdf"] = str(input_urdf)
    result["identification_urdf"] = str(model_urdf)
    if args.ignore_payload_link:
        result["ignored_payload_link"] = args.ignore_payload_link

    output = save_identification_result(args.output, result)
    print(f"[saved] {output}")
    print(
        "fit: "
        f"mode={result['mode']} samples={result['samples']} rank={result['rank']} "
        f"cond={result['condition']:.3g} rmse={result['metrics']['rmse']:.6g} "
        f"mae={result['metrics']['mae']:.6g} r2={result['metrics']['r2']:.6g}"
    )
    print(f"per-joint RMSE: {np.array(result['metrics']['per_joint_rmse'])}")

    if args.urdf_output:
        if args.mode == "base":
            raise ValueError("--urdf-output requires --mode full or --mode payload; base parameters cannot be uniquely written to URDF.")
        if args.mode == "payload":
            urdf_out = apply_payload_parameters_to_urdf(
                result["input_urdf"],
                np.asarray(result["payload_dynamic_parameters"], dtype=float),
                args.urdf_output,
                link_name=args.payload_link,
                default_mass=args.payload_default_mass,
            )
        else:
            urdf_out = apply_dynamic_parameters_to_urdf(
                result["identification_urdf"],
                np.asarray(result["dynamic_parameters"], dtype=float),
                args.urdf_output,
                remove_fixed_child_inertials=args.fold_fixed_child_inertials,
                preserve_fixed_child_inertials=not args.fold_fixed_child_inertials,
            )
        result["output_urdf"] = str(urdf_out)
        save_identification_result(args.output, result)
        print(f"[saved] {urdf_out}")


if __name__ == "__main__":
    main()
