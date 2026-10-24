import numpy as np

from dynamicbench.data.motion_audit import contact_motion, lift_motion
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


def test_contact_audit_catches_runaway_before_lift_and_includes_tail():
    eef = np.array([encode_eef10([x, 0, 1], np.eye(3), -1) for x in [0., .01, .13, .135]])
    cube = np.zeros((4, 7))
    cube[:, :3] = eef[:, :3] + [.145, 0, 0]
    arrays = {
        "observation.state.eef10": eef, "action.eef10": eef[1:],
        "oracle_debug.cube_pose": cube,
        "oracle_debug.finger_contact_forces": np.array([[0, 0], [15, 15], [15, 15], [15, 15]]),
    }
    trace = [dict(phase=p) for p in ["close_gripper", "close_gripper", "lift"]]
    result = contact_motion(arrays, trace)
    assert result["first_contact_tick"] == 1
    assert np.isclose(result["cube_pre_lift_delta_m"][0], .12)
    assert np.isclose(result["eef_contact_xy_peak_m"], .125)
    assert np.isclose(result["command_contact_xy_peak_m"], .005)
    assert np.isclose(result["contact_to_lift_seconds"], 1 / 30)
