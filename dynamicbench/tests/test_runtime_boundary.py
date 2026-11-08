"""Protect the on-disk pose contract across the Lab quaternion migration."""

from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from dynamicbench.envs import runtime
from dynamicbench.envs.poses import quat_wxyz_matrix


def test_lab61_pose_boundary_preserves_rotation_and_source(monkeypatch):
    monkeypatch.setattr(runtime, "ISAAC61", True)
    xyzw = Rotation.from_euler("xyz", [0.3, -0.7, 1.1]).as_quat()
    wxyz = xyzw[[3, 0, 1, 2]]
    np.testing.assert_allclose(runtime.lab_quaternion(wxyz), xyzw)
    original = np.r_[0.1, -0.2, 0.4, xyzw][None, :]
    proxy = SimpleNamespace(torch=SimpleNamespace(numpy=lambda: original))
    public = runtime.public_pose(proxy)
    np.testing.assert_allclose(quat_wxyz_matrix(public[0, 3:]), Rotation.from_quat(xyzw).as_matrix())
    np.testing.assert_array_equal(original[0, 3:], xyzw)
    np.testing.assert_array_equal(public[0, :3], original[0, :3])


def test_legacy_boundary_does_not_reorder(monkeypatch):
    monkeypatch.setattr(runtime, "ISAAC61", False)
    original = np.array([[1.0, 2.0, 3.0, 0.5, 0.5, -0.5, 0.5]])
    value = SimpleNamespace(numpy=lambda: original)
    np.testing.assert_array_equal(runtime.public_pose(value), original)
    np.testing.assert_array_equal(runtime.lab_quaternion(original[0, 3:]), original[0, 3:])
    assert runtime.tensor(value) is value
