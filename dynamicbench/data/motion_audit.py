"""Separate world transport from actual cube motion in the gripper frame."""

import numpy as np

from dynamicbench.envs.poses import decode_eef10


def lift_motion(arrays, trace):
    starts = [i for i, row in enumerate(trace) if row.get("phase") == "lift"]
    if not starts:
        return None
    start = starts[0]
    cube = arrays["oracle_debug.cube_pose"][:, :3]
    relative = np.asarray(
        [decode_eef10(e)[1].T @ (c - e[:3]) for e, c in zip(arrays["observation.state.eef10"], cube, strict=True)]
    )
    delta = relative[start:] - relative[start]
    result = dict(
        start_tick=start,
        end_observation_tick=len(cube) - 1,
        duration_seconds=(len(cube) - 1 - start) / 30,
        cube_world_delta_m=(cube[-1] - cube[start]).tolist(),
        cube_in_eef_final_delta_m=delta[-1].tolist(),
        cube_in_eef_peak_change_m=float(np.linalg.norm(delta, axis=1).max()),
        min_finger_contact_N=arrays["oracle_debug.finger_contact_forces"][start:].min(axis=0).tolist(),
    )
    if "oracle_debug.belt_surface_velocity" in arrays:
        velocity = arrays["oracle_debug.belt_surface_velocity"]
        result["belt_surface_velocity_min_m_s"] = velocity.min(axis=0).tolist()
        result["belt_surface_velocity_max_m_s"] = velocity.max(axis=0).tolist()
    if "oracle_debug.cube_in_pinch_frame" in arrays:
        pinch = arrays["oracle_debug.cube_in_pinch_frame"]
        result["cube_in_pinch_peak_change_m"] = float(np.linalg.norm(pinch[start:] - pinch[start], axis=1).max())
    return result
