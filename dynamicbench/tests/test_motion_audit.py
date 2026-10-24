import numpy as np

from dynamicbench.data.motion_audit import lift_motion
from dynamicbench.envs.poses import encode_eef10


def test_carrying_cube_with_gripper_is_not_reported_as_slip():
    eef = np.array([encode_eef10([x, 0, 1], np.eye(3), -1) for x in [0., .5, 1.]])
    cube = np.zeros((3, 7))
    cube[:, :3] = eef[:, :3] + [.145, 0, 0]
    arrays = {
        "observation.state.eef10": eef, "oracle_debug.cube_pose": cube,
        "oracle_debug.finger_contact_forces": np.ones((3, 2)) * 15,
    }
    result = lift_motion(arrays, [dict(phase="lift")] * 2)
    assert result["cube_world_delta_m"][0] == 1.
    assert result["cube_in_eef_peak_change_m"] < 1e-12
    cube[-1, 1] += .01
    assert np.isclose(lift_motion(arrays, [dict(phase="lift")] * 2)["cube_in_eef_peak_change_m"], .01)
