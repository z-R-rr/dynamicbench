from copy import deepcopy

import numpy as np

from dynamicbench.envs.poses import decode_eef10, encode_eef10
from dynamicbench.experts.conveyor_cube_expert import ConveyorCubeExpert


def test_live_lift_reacts_to_current_cube_and_preserves_observation():
    config = dict(robot=dict(tcp_offset_link6=[0.145, 0, 0]), expert=dict(
        lift_xy_mode="track_cube", approach_height=.12, lift_height=.13, max_tcp_speed=.25,
        descend_speed=.08, intercept_seconds=.1,
    ))
    expert = ConveyorCubeExpert(config, .765)
    expert.phase = "lift"
    expert.grasp_xy = np.array([-.08, 0.])
    obs = dict(
        eef10=encode_eef10([0., 0., 1.], expert.rotation, -1),
        oracle_debug=dict(cube_pose=np.array([.08, 0., .8]), cube_velocity=np.zeros(3)),
    )
    before = deepcopy(obs)
    live = decode_eef10(expert.action(obs))[0]
    expert.lift_xy_mode = "fixed"
    fixed = decode_eef10(expert.action(obs))[0]
    assert live[0] > 0 and fixed[0] < 0
    assert np.linalg.norm(live - obs["eef10"][:3]) <= .25 / 30 + 1e-12
    np.testing.assert_array_equal(obs["oracle_debug"]["cube_pose"], before["oracle_debug"]["cube_pose"])


def test_actual_pinch_transform_does_not_chase_jaw_center_motion():
    config = dict(robot=dict(tcp_offset_link6=[.145, 0, 0]), cube=dict(size=.035), expert=dict(
        lift_xy_mode="track_cube", lift_tcp_frame="actual_pinch", close_ticks=18,
        approach_height=.12, lift_height=.13, max_tcp_speed=.25, descend_speed=.08, intercept_seconds=.1,
    ))
    expert = ConveyorCubeExpert(config, .765)
    expert.phase, expert.phase_tick = "close_gripper", 17
    pos = np.array([0., 0., 1.])
    midpoint = np.array([.08657, .003, -.00024])
    grasp = np.array([.0594, 0., .00082])
    obs = dict(
        eef10=encode_eef10(pos, expert.rotation, -1),
        oracle_debug=dict(
            cube_pose=pos + expert.rotation @ (midpoint + grasp), cube_velocity=np.zeros(3),
            finger_midpoint_in_eef=midpoint.copy(), finger_contact_forces=np.ones(2) * 15,
        ),
    )
    expert.action(obs)
    assert expert.phase == "lift"
    # The same held cube follows a 1-cm change of the actual finger midpoint.
    obs["oracle_debug"]["finger_midpoint_in_eef"][1] -= .01
    obs["oracle_debug"]["cube_pose"] = pos + expert.rotation @ (
        obs["oracle_debug"]["finger_midpoint_in_eef"] + grasp
    )
    command = decode_eef10(expert.action(obs))[0]
    np.testing.assert_allclose(command[:2], pos[:2], atol=1e-12)
    assert command[2] > pos[2]

