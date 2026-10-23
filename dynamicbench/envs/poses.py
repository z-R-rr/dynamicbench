"""Right-handed world EEF10: xyz, rotation columns 0 and 1, gripper [-1,+1]."""

import numpy as np
from scipy.spatial.transform import Rotation


def validate_eef10(value):
    value = np.asarray(value, dtype=np.float64)
    if value.shape != (10,) or not np.isfinite(value).all():
        raise ValueError("EEF10 must be finite shape (10,)")
    if not -1 <= value[9] <= 1:
        raise ValueError("gripper outside [-1,1]")
    a, b = value[3:6], value[6:9]
    if not (
        np.isclose(np.linalg.norm(a), 1, atol=1e-4)
        and np.isclose(np.linalg.norm(b), 1, atol=1e-4)
        and abs(np.dot(a, b)) < 1e-4
    ):
        raise ValueError("rotation columns must be orthonormal")
    return value.copy()


def encode_eef10(position, rotation_matrix, gripper):
    r = np.asarray(rotation_matrix)
    return validate_eef10(np.r_[position, r[:, 0], r[:, 1], gripper])


def decode_eef10(value):
    v = validate_eef10(value)
    r = np.column_stack([v[3:6], v[6:9], np.cross(v[3:6], v[6:9])])
    return v[:3], r, v[9]


def quat_wxyz_matrix(q):
    q = np.asarray(q)
    return Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
