from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from rebotarm_control_rt.identification import (
    IdentificationDataset,
    apply_dynamic_parameters_to_urdf,
    apply_payload_parameters_to_urdf,
    build_regression_matrix,
    fit_payload_dynamics,
    fit_dynamics,
    save_identification_csv,
    load_identification_csv,
    stack_tau_samples,
    write_urdf_without_link_inertial,
)
from rebotarm_control_rt.kinematics import load_robot_model
from rebotarm_control_rt import _math


def _synthetic_dataset(model, samples=40, include_friction=True):
    rng = np.random.default_rng(123)
    q = rng.uniform(-0.8, 0.4, size=(samples, model.nq))
    dq = rng.uniform(-0.8, 0.8, size=(samples, model.nv))
    ddq = rng.uniform(-1.0, 1.0, size=(samples, model.nv))
    tmp = IdentificationDataset(q=q, dq=dq, ddq=ddq, tau=np.zeros((samples, model.nv)))
    Y = build_regression_matrix(model, tmp, include_friction=include_friction)
    beta = np.linspace(0.1, 1.0, Y.shape[1])
    tau = (Y @ beta).reshape(samples, model.nv)
    return IdentificationDataset(q=q, dq=dq, ddq=ddq, tau=tau), beta


def test_full_identification_predicts_synthetic_torque():
    model = load_robot_model()
    dataset, _ = _synthetic_dataset(model)
    result = fit_dynamics(model, dataset, mode="full", include_friction=True)
    assert result["metrics"]["rmse"] < 1e-8
    assert result["metrics"]["r2"] > 0.999999


def test_base_identification_predicts_synthetic_torque():
    model = load_robot_model()
    dataset, _ = _synthetic_dataset(model)
    result = fit_dynamics(model, dataset, mode="base", include_friction=True)
    assert result["metrics"]["rmse"] < 1e-8
    assert len(result["selected_columns"]) == result["rank"]


def test_csv_roundtrip(tmp_path: Path):
    model = load_robot_model()
    dataset, _ = _synthetic_dataset(model, samples=5)
    path = save_identification_csv(tmp_path / "id.csv", dataset)
    loaded = load_identification_csv(path, dof=model.nq)
    assert np.allclose(loaded.q, dataset.q)
    assert np.allclose(loaded.dq, dataset.dq)
    assert np.allclose(loaded.ddq, dataset.ddq)
    assert np.allclose(loaded.tau, dataset.tau)
    assert stack_tau_samples(loaded).shape == (loaded.samples * loaded.dof,)


def test_apply_dynamic_parameters_to_urdf(tmp_path: Path):
    urdf_in = tmp_path / "robot.urdf"
    urdf_out = tmp_path / "identified.urdf"
    urdf_in.write_text(
        """<robot name="test">
  <link name="base_link" />
  <link name="link1">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1" />
    </inertial>
  </link>
</robot>
""",
        encoding="utf-8",
    )
    params = np.array([2.0, 0.2, -0.4, 0.6, 0.01, 0.002, 0.03, 0.004, 0.005, 0.06])
    apply_dynamic_parameters_to_urdf(urdf_in, params, urdf_out, link_names=["link1"])
    inertial = ET.parse(urdf_out).getroot().find("./link[@name='link1']/inertial")
    assert inertial.find("mass").attrib["value"] == "2"
    assert np.allclose(
        [float(v) for v in inertial.find("origin").attrib["xyz"].split()],
        [0.1, -0.2, 0.3],
    )
    inertia = inertial.find("inertia").attrib
    assert float(inertia["ixx"]) == pytest.approx(0.01)
    assert float(inertia["iyy"]) == pytest.approx(0.03)
    assert float(inertia["izz"]) == pytest.approx(0.06)


def test_payload_identification_recovers_end_link_mass_and_com(tmp_path: Path):
    urdf_in = Path("calibration/tool_calibration.urdf")
    if not urdf_in.exists():
        from rebotarm_control_rt.paths import default_urdf_path

        urdf_in = default_urdf_path()
    arm_only = write_urdf_without_link_inertial(urdf_in, tmp_path / "arm_only.urdf", link_name="end_link")
    true_params = np.array([0.82, 0.82 * 0.03, 0.82 * -0.02, 0.82 * 0.11])
    full_urdf = apply_payload_parameters_to_urdf(arm_only, true_params, tmp_path / "payload.urdf", link_name="end_link")

    model = load_robot_model(str(full_urdf))
    rng = np.random.default_rng(7)
    samples = 24
    q = rng.uniform(-0.6, 0.3, size=(samples, model.nq))
    dq = rng.uniform(-0.2, 0.2, size=(samples, model.nv))
    ddq = rng.uniform(-0.3, 0.3, size=(samples, model.nv))
    tau = np.vstack([
        np.asarray(_math.inverse_dynamics(model, q[i], dq[i], ddq[i]), dtype=float)
        for i in range(samples)
    ])
    dataset = IdentificationDataset(q=q, dq=dq, ddq=ddq, tau=tau)

    result = fit_payload_dynamics(arm_only, dataset, link_name="end_link", parameter_count=4)
    assert np.allclose(result["payload_beta"], true_params, atol=1e-6)
