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

