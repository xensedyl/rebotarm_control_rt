"""Offline regression tests for packaged URDF visualization resources."""
from __future__ import annotations

import sys
from pathlib import Path
from unittest.mock import Mock

import numpy as np
import pytest

pin = pytest.importorskip("pinocchio")
pytest.importorskip("meshcat")

EXAMPLE_PYTHON_ROOT = Path(__file__).resolve().parents[2] / "example" / "python"
if str(EXAMPLE_PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(EXAMPLE_PYTHON_ROOT))

from rebotarm_control_rt.kinematics import load_robot_model
from rebotarm_control_rt.paths import default_urdf_path, robstride_urdf_path
from sim.visualizer import (
    Visualizer,
    _build_visual_geometry,
    _joint_configuration_slices,
    _mesh_resolved_urdf,
)


@pytest.mark.parametrize(
    ("source_urdf", "expected_geometry_count"),
    [(default_urdf_path(), 8), (robstride_urdf_path(), 23)],
)
def test_packaged_visual_geometry_loads(source_urdf: Path, expected_geometry_count: int):
    mesh_urdf = Path(_mesh_resolved_urdf(source_urdf))
    try:
        model = pin.buildModelFromUrdf(str(mesh_urdf))
        geometry = _build_visual_geometry(model, str(mesh_urdf), source_urdf)
    finally:
        mesh_urdf.unlink(missing_ok=True)

    assert len(geometry.geometryObjects) == expected_geometry_count
    assert all(Path(obj.meshPath).is_file() for obj in geometry.geometryObjects)


def _update_only_visualizer(source_urdf: Path) -> Visualizer:
    viz = Visualizer.__new__(Visualizer)
    viz._rt_model = load_robot_model(str(source_urdf))
    viz._model = pin.buildModelFromUrdf(str(source_urdf))
    viz._visual_neutral = np.asarray(pin.neutral(viz._model), dtype=float)
    viz._joint_q_slices = _joint_configuration_slices(
        viz._model, viz._rt_model.joint_names(), viz._rt_model.nq
    )
    viz._viz = Mock()
    return viz


def test_robstride_update_maps_six_arm_joints_into_full_model():
    viz = _update_only_visualizer(robstride_urdf_path())
    q = np.arange(1, viz.nq + 1, dtype=float) / 10.0

    viz.update(q)

    visual_q = viz._viz.display.call_args.args[0]
    assert visual_q.shape == (8,)
    assert np.array_equal(visual_q[:6], q)
    assert np.array_equal(visual_q[6:], viz._visual_neutral[6:])


def test_default_update_keeps_six_joint_configuration():
    viz = _update_only_visualizer(default_urdf_path())
    q = np.arange(1, viz.nq + 1, dtype=float) / 10.0

    viz.update(q)

    assert np.array_equal(viz._viz.display.call_args.args[0], q)
    with pytest.raises(ValueError, match=r"q must have shape \(6,\)"):
        viz.update(np.zeros(8))
