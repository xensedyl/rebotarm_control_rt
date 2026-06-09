from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from rebotarm_control_rt.calibration.tcp import (
    apply_tool_to_urdf,
    matrix_to_xyz_rpy_deg,
    solve_tcp_full,
    solve_tcp_position,
    tool_axis_from_pose,
    urdf_joint_origin_matrix,
    xyz_rpy_deg_to_matrix,
)


def _rot_from_rpy(rpy: np.ndarray) -> np.ndarray:
    return xyz_rpy_deg_to_matrix([0.0, 0.0, 0.0], np.degrees(rpy))[:3, :3]


def _pose(rot: np.ndarray, trans: np.ndarray) -> np.ndarray:
    out = np.eye(4)
    out[:3, :3] = rot
    out[:3, 3] = trans
    return out


def test_solve_tcp_position_recovers_translation_and_world_point():
    rng = np.random.default_rng(42)
    u_true = np.array([0.032, -0.014, 0.118])
    world_point_true = np.array([0.35, -0.12, 0.22])
    poses = []

    for _ in range(8):
        rot = _rot_from_rpy(rng.uniform(-1.2, 1.2, size=3))
        trans = world_point_true - rot @ u_true
        poses.append(_pose(rot, trans))

    u, world_point, res_max_mm, res_mean_mm = solve_tcp_position(poses)

    assert np.allclose(u, u_true, atol=1e-9)
    assert np.allclose(world_point, world_point_true, atol=1e-9)
    assert res_max_mm < 1e-6
    assert res_mean_mm < 1e-6


def test_solve_tcp_full_recovers_tool_orientation():
    rng = np.random.default_rng(7)
    u_true = np.array([0.025, 0.011, 0.105])
    world_point = np.array([0.4, 0.08, 0.18])
    tool_rot_true = _rot_from_rpy(np.array([0.3, -0.45, 0.8]))
    x_true = tool_rot_true[:, 0]
    z_true = tool_rot_true[:, 2]

    touch_poses = []
    for _ in range(8):
        rot = _rot_from_rpy(rng.uniform(-1.0, 1.0, size=3))
        touch_poses.append(_pose(rot, world_point - rot @ u_true))

    z_flange_rot = _rot_from_rpy(np.array([0.2, 0.4, -0.5]))
    z_pose = _pose(z_flange_rot, world_point + z_flange_rot @ z_true * 0.12 - z_flange_rot @ u_true)

    x_flange_rot = _rot_from_rpy(np.array([-0.4, 0.1, 0.6]))
    x_pose = _pose(x_flange_rot, world_point + x_flange_rot @ x_true * 0.10 - x_flange_rot @ u_true)

    transform, residuals = solve_tcp_full(touch_poses, z_pose, x_pose)

    assert np.allclose(transform[:3, 3], u_true, atol=1e-9)
    assert np.allclose(transform[:3, :3], tool_rot_true, atol=1e-9)
    assert residuals["max"] < 1e-6
    assert residuals["mean"] < 1e-6
    assert abs(residuals["x_dot_z"]) < 1e-9
    assert np.allclose(tool_axis_from_pose(z_pose, u_true, world_point), z_true, atol=1e-9)


def test_xyz_rpy_roundtrip():
    transform = xyz_rpy_deg_to_matrix([0.1, -0.2, 0.3], [20.0, -30.0, 40.0])
    xyz, rpy = matrix_to_xyz_rpy_deg(transform)
    assert np.allclose(xyz, [0.1, -0.2, 0.3])
    assert np.allclose(rpy, [20.0, -30.0, 40.0])


def test_apply_tool_to_urdf_writes_copy(tmp_path: Path):
    urdf_in = tmp_path / "input.urdf"
    urdf_out = tmp_path / "output.urdf"
    urdf_in.write_text(
        """<?xml version="1.0"?>
<robot name="test">
  <link name="link6" />
  <link name="end_link" />
  <joint name="end_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="link6" />
    <child link="end_link" />
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    transform = xyz_rpy_deg_to_matrix([0.1, -0.2, 0.3], [10.0, 20.0, -30.0])

    result_path = apply_tool_to_urdf(urdf_in, transform, urdf_out)

    assert result_path == urdf_out
    in_origin = ET.parse(urdf_in).getroot().find("./joint[@name='end_joint']/origin")
    out_origin = ET.parse(urdf_out).getroot().find("./joint[@name='end_joint']/origin")
    assert in_origin.attrib["xyz"] == "0 0 0"
    assert np.allclose([float(v) for v in out_origin.attrib["xyz"].split()], [0.1, -0.2, 0.3])
    assert np.allclose(
        [float(v) for v in out_origin.attrib["rpy"].split()],
        np.radians([10.0, 20.0, -30.0]),
    )


def test_apply_tool_to_urdf_can_preserve_original_rpy_text(tmp_path: Path):
    urdf_in = tmp_path / "input.urdf"
    urdf_out = tmp_path / "output.urdf"
    original_rpy = "0 -1.5708 3.1415"
    urdf_in.write_text(
        f"""<?xml version="1.0"?>
<robot name="test">
  <link name="link6" />
  <link name="end_link" />
  <joint
    name="end_joint"
    type="fixed">
    <origin
      xyz="0 0 0.15539"
      rpy="{original_rpy}" />
    <parent link="link6" />
    <child link="end_link" />
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    transform = urdf_joint_origin_matrix(urdf_in)
    transform[:3, 3] = [0.01, -0.02, 0.19]

    apply_tool_to_urdf(urdf_in, transform, urdf_out, preserve_rpy=True)

    output_text = urdf_out.read_text(encoding="utf-8")
    out_origin = ET.parse(urdf_out).getroot().find("./joint[@name='end_joint']/origin")
    assert original_rpy in output_text
    assert out_origin.attrib["rpy"] == original_rpy
    assert np.allclose([float(v) for v in out_origin.attrib["xyz"].split()], [0.01, -0.02, 0.19])


def test_urdf_joint_origin_matrix_reads_origin(tmp_path: Path):
    urdf = tmp_path / "input.urdf"
    urdf.write_text(
        """<robot name="test">
  <joint name="end_joint" type="fixed">
    <origin xyz="0.1 -0.2 0.3" rpy="0.174532925 0.34906585 -0.523598776" />
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    transform = urdf_joint_origin_matrix(urdf)
    xyz, rpy = matrix_to_xyz_rpy_deg(transform)
    assert np.allclose(xyz, [0.1, -0.2, 0.3])
    assert np.allclose(rpy, [10.0, 20.0, -30.0], atol=1e-6)


if __name__ == "__main__":
    raise SystemExit(pytest.main([str(Path(__file__)), "-v"]))
