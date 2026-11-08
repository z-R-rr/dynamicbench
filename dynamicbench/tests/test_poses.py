import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dynamicbench.envs.poses import decode_eef10, encode_eef10, quat_wxyz_matrix


def test_rotation_columns_world_quaternion_and_gripper_are_explicit():
    r = Rotation.from_euler("z", 90, degrees=True).as_matrix()
    value = encode_eef10([1, 2, 3], r, -1)
    np.testing.assert_allclose(value[3:9], [0, 1, 0, -1, 0, 0], atol=1e-15)
    p, decoded, g = decode_eef10(value)
    np.testing.assert_allclose(decoded, r, atol=1e-15)
    np.testing.assert_equal(p, [1, 2, 3])
    np.testing.assert_allclose(quat_wxyz_matrix([2**-0.5, 0, 0, 2**-0.5]), r, atol=1e-15)
    assert g == -1
    value[6:9] = value[3:6]
    with pytest.raises(ValueError, match="orthonormal"):
        decode_eef10(value)
