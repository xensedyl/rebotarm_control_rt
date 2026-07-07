"""Dynamics parameter identification utilities.

The numerical backend is implemented in C++/Pinocchio and exposed through
``rebotarm_control_rt._math``.  This Python layer handles CSV datasets, YAML
serialization, and URDF inertial write-back.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math
import re
import tempfile
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from rebotarm_control_rt import _math
from rebotarm_control_rt.kinematics import load_robot_model
from rebotarm_control_rt.paths import resolve_urdf_path
from rebotarm_control_rt.identification.trajectory import (
    RecordedTrajectory,
    load_recorded_trajectory_csv,
    save_recorded_trajectory_csv,
    trajectory_summary,
)


JOINT_PREFIXES = ("q", "dq", "ddq", "tau")


@dataclass
class IdentificationDataset:
    q: np.ndarray
    dq: np.ndarray
    ddq: np.ndarray
    tau: np.ndarray
    time: np.ndarray | None = None

    @property
    def samples(self) -> int:
        return int(self.q.shape[0])

    @property
    def dof(self) -> int:
        return int(self.q.shape[1])


def _column_indices(header: list[str], prefix: str, dof: int | None) -> list[int]:
    if dof is None:
        indices = [
            idx for idx, name in enumerate(header)
            if name == prefix or name.startswith(prefix)
        ]
        # Avoid q matching qd-style names; accepted names are q1 or q_1.
        if prefix == "q":
            indices = [
                idx for idx in indices
                if header[idx] == "q" or header[idx].startswith("q_") or header[idx][1:].isdigit()
            ]
        return indices

    names = []
    for i in range(1, dof + 1):
        names.extend([f"{prefix}{i}", f"{prefix}_{i}", f"{prefix}.joint_{i}"])
    indices = []
    for name in names:
        if name in header:
            indices.append(header.index(name))
    return indices


def load_identification_csv(path: str | Path, dof: int | None = None) -> IdentificationDataset:
    path = Path(path)
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [[float(cell) for cell in row] for row in reader if row]

    if not rows:
        raise ValueError(f"{path} contains no samples")
    data = np.asarray(rows, dtype=float)
    if dof is None:
        q_cols = _column_indices(header, "q", None)
        if not q_cols:
            raise ValueError("could not infer dof from q columns")
        dof = len(q_cols)

    cols = {name: _column_indices(header, name, dof) for name in JOINT_PREFIXES}
    for name, indices in cols.items():
        if len(indices) != dof:
            raise ValueError(f"expected {dof} {name} columns, got {len(indices)}")

    time = data[:, header.index("time")] if "time" in header else None
    return IdentificationDataset(
        q=data[:, cols["q"]],
        dq=data[:, cols["dq"]],
        ddq=data[:, cols["ddq"]],
        tau=data[:, cols["tau"]],
        time=time,
    )


def save_identification_csv(path: str | Path, dataset: IdentificationDataset) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ["time"]
    for prefix in JOINT_PREFIXES:
        header.extend(f"{prefix}{i}" for i in range(1, dataset.dof + 1))

    time = dataset.time if dataset.time is not None else np.arange(dataset.samples, dtype=float)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(dataset.samples):
            row = [time[i]]
            row.extend(dataset.q[i].tolist())
            row.extend(dataset.dq[i].tolist())
            row.extend(dataset.ddq[i].tolist())
            row.extend(dataset.tau[i].tolist())
            writer.writerow(row)
    return path


def build_regression_matrix(model, dataset: IdentificationDataset, include_friction: bool = True,
                            coulomb_eps: float = 1e-3) -> np.ndarray:
    return np.asarray(
        _math.build_regression_matrix(
            model,
            np.asarray(dataset.q, dtype=float),
            np.asarray(dataset.dq, dtype=float),
            np.asarray(dataset.ddq, dtype=float),
            bool(include_friction),
            float(coulomb_eps),
        )
    )


def stack_tau_samples(dataset: IdentificationDataset) -> np.ndarray:
    return np.asarray(_math.stack_tau_samples(np.asarray(dataset.tau, dtype=float)))


def inverse_dynamics_samples(model, dataset: IdentificationDataset) -> np.ndarray:
    tau = np.zeros((dataset.samples, dataset.dof), dtype=float)
    for i in range(dataset.samples):
        tau[i] = np.asarray(
            _math.inverse_dynamics(
                model,
                np.asarray(dataset.q[i], dtype=float),
                np.asarray(dataset.dq[i], dtype=float),
                np.asarray(dataset.ddq[i], dtype=float),
            ),
            dtype=float,
        )
    return tau


def _metrics_to_dict(metrics) -> dict:
    return {
        "rmse": float(metrics.rmse),
        "mae": float(metrics.mae),
        "max_abs": float(metrics.max_abs),
        "r2": float(metrics.r2),
        "per_joint_rmse": [float(v) for v in np.asarray(metrics.per_joint_rmse)],
        "per_joint_mae": [float(v) for v in np.asarray(metrics.per_joint_mae)],
    }


def fit_dynamics(
    model,
    dataset: IdentificationDataset,
    mode: str = "full",
    include_friction: bool = True,
    coulomb_eps: float = 1e-3,
    rcond: float = 1e-12,
    use_model_prior: bool = True,
) -> dict:
    mode = mode.lower()
    if mode not in {"full", "base"}:
        raise ValueError("mode must be 'full' or 'base'")

    Y = build_regression_matrix(model, dataset, include_friction, coulomb_eps)
    tau = stack_tau_samples(dataset)
    names = _math.total_parameter_names(model, include_friction)
    dyn_count = int(_math.num_dynamic_parameters(model))

    if mode == "full":
        result = _math.fit_least_squares(Y, tau, float(rcond))
        beta = np.asarray(result.beta, dtype=float)
        if use_model_prior:
            prior = np.asarray(_math.model_dynamic_parameters(model), dtype=float)
            if include_friction:
                prior = np.concatenate([prior, np.zeros(dataset.dof * 2, dtype=float)])
            beta = prior + np.linalg.pinv(Y, rcond=rcond) @ (tau - Y @ prior)
        tau_pred = np.asarray(result.tau_pred, dtype=float)
        if use_model_prior:
            tau_pred = Y @ beta
        selected_columns: list[int] | None = None
        beta_base = None
    else:
        result = _math.fit_base_parameters_qr(Y, tau, float(rcond))
        beta_base = np.asarray(result.beta_base, dtype=float)
        selected_columns = [int(v) for v in np.asarray(result.selected_columns)]
        tau_pred = np.asarray(result.tau_pred, dtype=float)
        beta = None

    metrics = _math.regression_metrics(tau, tau_pred, dataset.dof)
    payload = {
        "mode": mode,
        "samples": dataset.samples,
        "dof": dataset.dof,
        "include_friction": bool(include_friction),
        "use_model_prior": bool(use_model_prior) if mode == "full" else False,
        "coulomb_eps": float(coulomb_eps),
        "rcond": float(rcond),
        "rank": int(result.rank),
        "condition": float(result.condition),
        "residual_norm": float(np.linalg.norm(tau - tau_pred)),
        "dynamic_parameter_count": dyn_count,
        "parameter_names": list(names),
        "metrics": _metrics_to_dict(metrics),
        "tau_pred": tau_pred.reshape(dataset.samples, dataset.dof).tolist(),
    }
    if beta is not None:
        payload["beta"] = beta.tolist()
        payload["dynamic_parameters"] = beta[:dyn_count].tolist()
        if include_friction:
            payload["friction_parameters"] = beta[dyn_count:].tolist()
    if beta_base is not None:
        payload["beta_base"] = beta_base.tolist()
        payload["selected_columns"] = selected_columns
        payload["selected_parameter_names"] = [names[i] for i in selected_columns]
    return payload


def save_identification_result(path: str | Path, result: dict) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True), encoding="utf-8")
    return path


def load_identification_result(path: str | Path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _format_float(value: float) -> str:
    return f"{float(value):.12g}"


def _write_urdf(tree: ET.ElementTree, path: str | Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return path


def _format_inertial_block(params: np.ndarray, *, indent: str = "    ", rpy: str = "0 0 0") -> str:
    mass, com, ic = _inertia_from_dynamic_params(params)
    child = f"{indent}  "
    attr = f"{indent}    "
    return "\n".join(
        [
            f"{indent}<inertial>",
            f"{child}<origin",
            f'{attr}xyz="{" ".join(_format_float(v) for v in com)}"',
            f'{attr}rpy="{rpy}" />',
            f"{child}<mass",
            f'{attr}value="{_format_float(mass)}" />',
            f"{child}<inertia",
            f'{attr}ixx="{_format_float(ic[0, 0])}"',
            f'{attr}ixy="{_format_float(ic[0, 1])}"',
            f'{attr}ixz="{_format_float(ic[0, 2])}"',
            f'{attr}iyy="{_format_float(ic[1, 1])}"',
            f'{attr}iyz="{_format_float(ic[1, 2])}"',
            f'{attr}izz="{_format_float(ic[2, 2])}" />',
            f"{indent}</inertial>",
        ]
    )


def _replace_link_inertial_text(
    urdf_in: Path,
    urdf_out: Path,
    *,
    link_name: str,
    params: np.ndarray,
) -> bool:
    """Replace one link inertial block while preserving the source URDF style."""
    text = urdf_in.read_text(encoding="utf-8")
    link_pattern = re.compile(
        r"<link\b(?=[^>]*\bname\s*=\s*([\"'])" + re.escape(link_name) + r"\1)[\s\S]*?</link>",
        re.MULTILINE,
    )
    link_match = link_pattern.search(text)
    if link_match is None:
        return False

    link_text = link_match.group(0)
    inertial_pattern = re.compile(r"(?P<indent>[ \t]*)<inertial>[\s\S]*?(?P=indent)</inertial>", re.MULTILINE)
    inertial_match = inertial_pattern.search(link_text)
    if inertial_match is None:
        return False

    origin_match = re.search(r"<origin\b[^>]*\brpy\s*=\s*([\"'])(?P<rpy>.*?)\1", inertial_match.group(0), re.DOTALL)
    rpy = origin_match.group("rpy") if origin_match is not None else "0 0 0"
    new_inertial = _format_inertial_block(params, indent=inertial_match.group("indent"), rpy=rpy)
    new_link_text = (
        link_text[: inertial_match.start()]
        + new_inertial
        + link_text[inertial_match.end() :]
    )
    new_text = text[: link_match.start()] + new_link_text + text[link_match.end() :]
    urdf_out.parent.mkdir(parents=True, exist_ok=True)
    urdf_out.write_text(new_text, encoding="utf-8")
    return True


def _symmetric_from_params(values: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [values[0], values[1], values[3]],
            [values[1], values[2], values[4]],
            [values[3], values[4], values[5]],
        ],
        dtype=float,
    )


def _symmetric_to_params(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float)
    return np.array(
        [matrix[0, 0], matrix[0, 1], matrix[1, 1], matrix[0, 2], matrix[1, 2], matrix[2, 2]],
        dtype=float,
    )


def _parallel_axis(mass: float, com: np.ndarray) -> np.ndarray:
    com = np.asarray(com, dtype=float)
    return float(mass) * ((com @ com) * np.eye(3) - np.outer(com, com))


def _dynamic_params_from_inertia(mass: float, com: np.ndarray, inertia_at_com: np.ndarray) -> np.ndarray:
    mass = float(mass)
    com = np.asarray(com, dtype=float)
    inertia_at_com = np.asarray(inertia_at_com, dtype=float)
    params = np.zeros(10, dtype=float)
    params[0] = mass
    params[1:4] = mass * com
    params[4:10] = _symmetric_to_params(inertia_at_com + _parallel_axis(mass, com))
    return params


def _inertia_from_dynamic_params(params: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    params = np.asarray(params, dtype=float)
    if params.shape != (10,):
        raise ValueError("one inertial block must contain 10 parameters")
    mass = float(params[0])
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError(f"identified mass must be positive, got {mass}")
    com = params[1:4] / mass
    inertia_at_link_origin = _symmetric_from_params(params[4:10])
    ic = inertia_at_link_origin - _parallel_axis(mass, com)
    return mass, com, ic


def _payload_params_with_preserved_com_inertia(base_params: np.ndarray, payload_parameters: np.ndarray) -> np.ndarray:
    """Update mass/COM while preserving the input URDF inertia about its COM.

    Four-parameter payload identification estimates only mass and first
    moments. Keeping the remaining dynamic parameters unchanged would keep the
    inertia about the link origin, which can become non-physical after the COM
    moves. URDF stores inertia about the inertial origin, so preserve that
    original COM inertia instead.
    """
    base_mass, _base_com, base_inertia_at_com = _inertia_from_dynamic_params(base_params)
    mass = float(payload_parameters[0])
    if not math.isfinite(mass) or mass <= 0.0:
        raise ValueError(f"identified payload mass must be positive, got {mass}")
    com = np.asarray(payload_parameters[1:4], dtype=float) / mass
    inertia_at_com = base_inertia_at_com * (mass / base_mass)
    return _dynamic_params_from_inertia(mass, com, inertia_at_com)


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=float)
    return rz @ ry @ rx


def _joint_origin_transform(joint: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    origin = joint.find("origin")
    xyz = np.zeros(3, dtype=float)
    rpy = np.zeros(3, dtype=float)
    if origin is not None:
        xyz_text = origin.get("xyz")
        rpy_text = origin.get("rpy")
        if xyz_text:
            xyz = np.array([float(v) for v in xyz_text.split()], dtype=float)
        if rpy_text:
            rpy = np.array([float(v) for v in rpy_text.split()], dtype=float)
    return xyz, _rpy_to_matrix(rpy)


def _transform_dynamic_params(params: np.ndarray, xyz: np.ndarray, rot: np.ndarray) -> np.ndarray:
    mass, com, inertia_at_com = _inertia_from_dynamic_params(params)
    com_parent = np.asarray(xyz, dtype=float) + np.asarray(rot, dtype=float) @ com
    inertia_parent_at_com = rot @ inertia_at_com @ rot.T
    return _dynamic_params_from_inertia(mass, com_parent, inertia_parent_at_com)


def _default_payload_params(mass: float = 0.5) -> np.ndarray:
    mass = float(mass)
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError("default payload mass must be positive")
    return np.array([mass, 0.0, 0.0, 0.0, 1e-5, 0.0, 1e-5, 0.0, 0.0, 1e-5], dtype=float)


def _dynamic_params_from_link(root: ET.Element, link_name: str, *, default_mass: float | None = None) -> np.ndarray:
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"URDF link not found: {link_name}")
    inertial = link.find("inertial")
    if inertial is None:
        if default_mass is None:
            raise ValueError(f"URDF link has no inertial: {link_name}")
        return _default_payload_params(default_mass)

    origin = inertial.find("origin")
    mass_el = inertial.find("mass")
    inertia_el = inertial.find("inertia")
    if mass_el is None or inertia_el is None:
        if default_mass is None:
            raise ValueError(f"URDF link inertial is incomplete: {link_name}")
        return _default_payload_params(default_mass)

    mass = float(mass_el.get("value", "0"))
    xyz = origin.get("xyz", "0 0 0").split() if origin is not None else ["0", "0", "0"]
    if len(xyz) != 3:
        raise ValueError(f"invalid inertial origin xyz for link {link_name}")
    com = np.array([float(v) for v in xyz], dtype=float)
    params = np.zeros(10, dtype=float)
    params[0] = mass
    params[1:4] = mass * com
    inertia_at_com = np.array(
        [
            [float(inertia_el.get("ixx", "0")), float(inertia_el.get("ixy", "0")), float(inertia_el.get("ixz", "0"))],
            [float(inertia_el.get("ixy", "0")), float(inertia_el.get("iyy", "0")), float(inertia_el.get("iyz", "0"))],
            [float(inertia_el.get("ixz", "0")), float(inertia_el.get("iyz", "0")), float(inertia_el.get("izz", "0"))],
        ],
        dtype=float,
    )
    params[4:10] = _symmetric_to_params(inertia_at_com + _parallel_axis(mass, com))
    return params


def _write_link_dynamic_parameters(root: ET.Element, link_name: str, params: np.ndarray) -> None:
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"URDF link not found: {link_name}")
    inertial = link.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(link, "inertial")
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.SubElement(inertial, "origin")
    mass_el = inertial.find("mass")
    if mass_el is None:
        mass_el = ET.SubElement(inertial, "mass")
    inertia_el = inertial.find("inertia")
    if inertia_el is None:
        inertia_el = ET.SubElement(inertial, "inertia")

    mass, com, ic = _inertia_from_dynamic_params(params)
    origin.set("xyz", " ".join(_format_float(v) for v in com))
    origin.set("rpy", origin.get("rpy", "0 0 0"))
    mass_el.set("value", _format_float(mass))
    inertia_el.set("ixx", _format_float(ic[0, 0]))
    inertia_el.set("ixy", _format_float(ic[0, 1]))
    inertia_el.set("ixz", _format_float(ic[0, 2]))
    inertia_el.set("iyy", _format_float(ic[1, 1]))
    inertia_el.set("iyz", _format_float(ic[1, 2]))
    inertia_el.set("izz", _format_float(ic[2, 2]))


def _link_name_from_joint_child(joint: ET.Element) -> str | None:
    child = joint.find("child")
    if child is None:
        return None
    return child.get("link")


def _movable_joint_child_links(root: ET.Element) -> list[str]:
    links: list[str] = []
    for joint in root.findall("joint"):
        if joint.get("type") == "fixed":
            continue
        child_link = _link_name_from_joint_child(joint)
        if child_link:
            links.append(child_link)
    return links


def _fixed_descendant_links(root: ET.Element, parents: list[str]) -> list[str]:
    fixed_children: dict[str, list[str]] = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = parent.get("link")
        child_name = child.get("link")
        if parent_name and child_name:
            fixed_children.setdefault(parent_name, []).append(child_name)

    out: list[str] = []
    stack = list(parents)
    seen = set(stack)
    while stack:
        parent = stack.pop()
        for child in fixed_children.get(parent, []):
            if child in seen:
                continue
            seen.add(child)
            out.append(child)
            stack.append(child)
    return out


def _fixed_child_inertial_sum(root: ET.Element, parent_link: str) -> np.ndarray:
    fixed_edges: dict[str, list[tuple[str, np.ndarray, np.ndarray]]] = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            continue
        parent_name = parent.get("link")
        child_name = child.get("link")
        if not parent_name or not child_name:
            continue
        xyz, rot = _joint_origin_transform(joint)
        fixed_edges.setdefault(parent_name, []).append((child_name, xyz, rot))

    total = np.zeros(10, dtype=float)

    def walk(link_name: str, xyz_acc: np.ndarray, rot_acc: np.ndarray) -> None:
        nonlocal total
        for child_name, xyz_rel, rot_rel in fixed_edges.get(link_name, []):
            xyz_child = xyz_acc + rot_acc @ xyz_rel
            rot_child = rot_acc @ rot_rel
            child_link = root.find(f"./link[@name='{child_name}']")
            if child_link is not None and child_link.find("inertial") is not None:
                child_params = _dynamic_params_from_link(root, child_name)
                total += _transform_dynamic_params(child_params, xyz_child, rot_child)
            walk(child_name, xyz_child, rot_child)

    walk(parent_link, np.zeros(3, dtype=float), np.eye(3, dtype=float))
    return total


def apply_dynamic_parameters_to_urdf(
    urdf_in: str | Path,
    dynamic_parameters: np.ndarray,
    urdf_out: str | Path,
    *,
    link_names: list[str] | None = None,
    remove_fixed_child_inertials: bool = True,
    preserve_fixed_child_inertials: bool = False,
) -> Path:
    urdf_in = resolve_urdf_path(urdf_in)
    urdf_out = Path(urdf_out)
    dynamic_parameters = np.asarray(dynamic_parameters, dtype=float)
    if dynamic_parameters.size % 10 != 0:
        raise ValueError("dynamic parameter vector length must be a multiple of 10")

    tree = ET.parse(urdf_in)
    root = tree.getroot()
    if link_names is None:
        link_names = _movable_joint_child_links(root)

    blocks = dynamic_parameters.reshape(-1, 10)
    if len(link_names) != blocks.shape[0]:
        raise ValueError(f"link count {len(link_names)} does not match parameter blocks {blocks.shape[0]}")

    if remove_fixed_child_inertials and preserve_fixed_child_inertials:
        raise ValueError("remove_fixed_child_inertials and preserve_fixed_child_inertials cannot both be true")

    for link_name, params in zip(link_names, blocks):
        params_to_write = params
        if preserve_fixed_child_inertials:
            params_to_write = params - _fixed_child_inertial_sum(root, link_name)
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise ValueError(f"URDF link not found: {link_name}")
        inertial = link.find("inertial")
        if inertial is None:
            inertial = ET.SubElement(link, "inertial")
        origin = inertial.find("origin")
        if origin is None:
            origin = ET.SubElement(inertial, "origin")
        mass_el = inertial.find("mass")
        if mass_el is None:
            mass_el = ET.SubElement(inertial, "mass")
        inertia_el = inertial.find("inertia")
        if inertia_el is None:
            inertia_el = ET.SubElement(inertial, "inertia")

        _write_link_dynamic_parameters(root, link_name, params_to_write)

    if remove_fixed_child_inertials:
        for link_name in _fixed_descendant_links(root, link_names):
            link = root.find(f"./link[@name='{link_name}']")
            if link is None:
                continue
            inertial = link.find("inertial")
            if inertial is not None:
                link.remove(inertial)

    return _write_urdf(tree, urdf_out)


def write_urdf_without_link_inertial(
    urdf_in: str | Path,
    urdf_out: str | Path,
    *,
    link_name: str = "end_link",
) -> Path:
    urdf_in = resolve_urdf_path(urdf_in)
    urdf_out = Path(urdf_out)
    tree = ET.parse(urdf_in)
    root = tree.getroot()
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"URDF link not found: {link_name}")
    inertial = link.find("inertial")
    if inertial is not None:
        link.remove(inertial)
    return _write_urdf(tree, urdf_out)


def apply_payload_parameters_to_urdf(
    urdf_in: str | Path,
    payload_parameters: np.ndarray,
    urdf_out: str | Path,
    *,
    link_name: str = "end_link",
    default_mass: float = 0.5,
) -> Path:
    urdf_in = resolve_urdf_path(urdf_in)
    urdf_out = Path(urdf_out)
    tree = ET.parse(urdf_in)
    root = tree.getroot()
    payload_parameters = np.asarray(payload_parameters, dtype=float)
    if payload_parameters.shape == (4,):
        base_params = _dynamic_params_from_link(root, link_name, default_mass=default_mass)
        params = _payload_params_with_preserved_com_inertia(base_params, payload_parameters)
    elif payload_parameters.shape == (10,):
        params = payload_parameters
    else:
        raise ValueError("payload_parameters must contain 4 or 10 values")
    if _replace_link_inertial_text(urdf_in, urdf_out, link_name=link_name, params=params):
        return urdf_out

    _write_link_dynamic_parameters(root, link_name, params)
    return _write_urdf(tree, urdf_out)


def _payload_parameter_indices(parameter_count: int) -> list[int]:
    if parameter_count == 4:
        return [0, 1, 2, 3]
    if parameter_count == 10:
        return list(range(10))
    raise ValueError("payload parameter count must be 4 or 10")


def _payload_parameter_names(link_name: str, parameter_count: int) -> list[str]:
    fields = ["m", "mcx", "mcy", "mcz", "ixx", "ixy", "iyy", "ixz", "iyz", "izz"]
    return [f"{link_name}.{fields[i]}" for i in _payload_parameter_indices(parameter_count)]


def _write_temp_payload_urdf(
    urdf_in: Path,
    out_path: Path,
    *,
    link_name: str,
    params: np.ndarray | None,
) -> Path:
    tree = ET.parse(urdf_in)
    root = tree.getroot()
    if params is None:
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise ValueError(f"URDF link not found: {link_name}")
        inertial = link.find("inertial")
        if inertial is not None:
            link.remove(inertial)
    else:
        _write_link_dynamic_parameters(root, link_name, params)
    return _write_urdf(tree, out_path)


def fit_payload_dynamics(
    urdf_path: str | Path | None,
    dataset: IdentificationDataset,
    *,
    link_name: str = "end_link",
    parameter_count: int = 4,
    default_mass: float = 0.5,
    finite_difference_eps: float = 1e-5,
    rcond: float = 1e-12,
) -> dict:
    if finite_difference_eps <= 0:
        raise ValueError("finite_difference_eps must be positive")
    parameter_indices = _payload_parameter_indices(parameter_count)
    urdf_resolved = resolve_urdf_path(urdf_path)
    root = ET.parse(urdf_resolved).getroot()
    nominal_params = _dynamic_params_from_link(root, link_name, default_mass=default_mass)

    with tempfile.TemporaryDirectory(prefix="rebotarm_payload_id_") as tmp:
        tmpdir = Path(tmp)
        arm_only_urdf = _write_temp_payload_urdf(
            urdf_resolved,
            tmpdir / "arm_only.urdf",
            link_name=link_name,
            params=None,
        )
        nominal_urdf = _write_temp_payload_urdf(
            urdf_resolved,
            tmpdir / "payload_nominal.urdf",
            link_name=link_name,
            params=nominal_params,
        )
        arm_model = load_robot_model(str(arm_only_urdf))
        nominal_model = load_robot_model(str(nominal_urdf))
        tau_arm = inverse_dynamics_samples(arm_model, dataset)
        tau_nominal = inverse_dynamics_samples(nominal_model, dataset)

        cols = []
        for col, param_index in enumerate(parameter_indices):
            perturbed = nominal_params.copy()
            scale = max(abs(float(nominal_params[param_index])), 1.0)
            step = finite_difference_eps * scale
            if param_index == 0:
                step = max(step, finite_difference_eps)
            perturbed[param_index] += step
            if perturbed[0] <= 0:
                raise ValueError("payload mass perturbation became non-positive")
            perturbed_urdf = _write_temp_payload_urdf(
                urdf_resolved,
                tmpdir / f"payload_perturbed_{col}.urdf",
                link_name=link_name,
                params=perturbed,
            )
            perturbed_model = load_robot_model(str(perturbed_urdf))
            tau_perturbed = inverse_dynamics_samples(perturbed_model, dataset)
            cols.append(((tau_perturbed - tau_nominal) / step).reshape(-1))

    Y = np.column_stack(cols)
    tau = stack_tau_samples(dataset)
    tau_nominal_flat = tau_nominal.reshape(-1)
    nominal_selected = nominal_params[parameter_indices]
    tau_fixed = tau_nominal_flat - Y @ nominal_selected
    residual_tau = tau - tau_fixed
    fit = _math.fit_least_squares(Y, residual_tau, float(rcond))
    beta = np.asarray(fit.beta, dtype=float)
    payload_params = nominal_params.copy()
    payload_params[parameter_indices] = beta
    tau_payload_pred = np.asarray(fit.tau_pred, dtype=float)
    tau_pred = tau_fixed + tau_payload_pred
    metrics = _math.regression_metrics(tau, tau_pred, dataset.dof)
    return {
        "mode": "payload",
        "samples": dataset.samples,
        "dof": dataset.dof,
        "payload_link": link_name,
        "payload_parameter_count": int(parameter_count),
        "payload_parameter_names": _payload_parameter_names(link_name, parameter_count),
        "payload_beta": beta.tolist(),
        "payload_dynamic_parameters": payload_params.tolist(),
        "nominal_payload_dynamic_parameters": nominal_params.tolist(),
        "default_mass": float(default_mass),
        "finite_difference_eps": float(finite_difference_eps),
        "rcond": float(rcond),
        "rank": int(fit.rank),
        "condition": float(fit.condition),
        "residual_norm": float(np.linalg.norm(tau - tau_pred)),
        "metrics": _metrics_to_dict(metrics),
        "tau_pred": tau_pred.reshape(dataset.samples, dataset.dof).tolist(),
        "tau_arm_only": tau_arm.tolist(),
        "tau_fixed_non_payload": tau_fixed.reshape(dataset.samples, dataset.dof).tolist(),
    }


def load_model_for_identification(urdf_path: str | Path | None = None):
    return load_robot_model(None if urdf_path is None else str(urdf_path))


__all__ = [
    "IdentificationDataset",
    "load_identification_csv",
    "save_identification_csv",
    "load_model_for_identification",
    "build_regression_matrix",
    "stack_tau_samples",
    "fit_dynamics",
    "fit_payload_dynamics",
    "save_identification_result",
    "load_identification_result",
    "apply_dynamic_parameters_to_urdf",
    "apply_payload_parameters_to_urdf",
    "write_urdf_without_link_inertial",
    "inverse_dynamics_samples",
    "RecordedTrajectory",
    "trajectory_summary",
    "save_recorded_trajectory_csv",
    "load_recorded_trajectory_csv",
]
