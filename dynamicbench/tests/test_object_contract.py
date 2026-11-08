"""Object configuration preserves contact control and physical success evidence."""

from copy import deepcopy

import numpy as np
import pytest

from dynamicbench.data.dataset_reader import read_episode
from dynamicbench.data.recorder import EpisodeRecorder
from dynamicbench.envs.poses import decode_eef10, encode_eef10
from dynamicbench.experts.conveyor_cube_expert import ConveyorCubeExpert
from dynamicbench.tests.test_expert import anchored_expert_and_contact


def test_object_contact_locks_world_pose_when_object_and_grasp_point_move(tmp_path):
    from scipy.spatial.transform import Rotation

    from dynamicbench.objects import grasp_position, resolve_object_settings, vertical_bounds

    legacy, obs = anchored_expert_and_contact()
    settings = deepcopy(legacy.settings)
    settings.pop("cube")
    settings["expert"]["success_height"] = 0.05
    settings["object"] = dict(
        id="moving_box",
        shape="box",
        size=[0.035] * 3,
        grasp=dict(position=[0.01, 0, 0.00175], quaternion_wxyz=[1, 0, 0, 0]),
    )
    settings = resolve_object_settings(settings, tmp_path)
    obj = settings["object"]
    expert = ConveyorCubeExpert(settings, legacy.belt_top)
    expert.phase = "close_gripper"
    anchor, anchor_rotation, _ = decode_eef10(obs["eef10"])
    anchor = anchor.copy()
    dbg = obs["oracle_debug"]
    dbg["cube_pose"] = np.r_[dbg["cube_pose"], 1, 0, 0, 0]

    def update_object():
        dbg["object_pose"] = dbg["cube_pose"].copy()
        dbg["object_bottom_z"], dbg["object_top_z"] = vertical_bounds(obj["geometry"], dbg["object_pose"])
        dbg["object_grasp_position"] = grasp_position(obj, dbg["object_pose"])

    update_object()
    dbg["finger_contact_forces"] = np.array([15.0, 0.0])
    first = expert.action(obs)
    np.testing.assert_allclose(first[:3], anchor)
    assert expert.phase == "grasp_settle"
    previous_z = first[2]
    initial_grasp_point = dbg["object_grasp_position"].copy()
    for step in range(20):
        # Both the configured grasp frame and the measured EEF drift after contact.
        dbg["cube_pose"][:3] += [0.002, -0.001, 0.003]
        dbg["cube_pose"][3:7] = Rotation.from_euler("z", 5 * (step + 1), degrees=True).as_quat()[[3, 0, 1, 2]]
        dbg["cube_velocity"] = np.array([0.3, -0.2, 0.1])
        dbg["finger_contact_forces"][:] = 15.0
        obs["eef10"][:2] += [0.0005, -0.0003]
        update_object()
        action = expert.action(obs)
        position, rotation, opening = decode_eef10(action)
        np.testing.assert_allclose(position[:2], anchor[:2], atol=1e-12)
        np.testing.assert_allclose(rotation, anchor_rotation, atol=1e-12)
        assert opening == -1
        assert -1e-12 <= position[2] - previous_z <= settings["expert"]["max_tcp_speed"] / 30 + 1e-12
        previous_z = position[2]
    assert np.linalg.norm(dbg["object_grasp_position"][:2] - initial_grasp_point[:2]) > 0.02
    assert expert.phase == "lift" and expert.failure is None
    assert previous_z > anchor[2]


def test_tilted_box_readback_waits_for_full_bottom_clearance_hold(tmp_path):
    import json

    from dynamicbench.objects import resolve_object_settings

    obj = resolve_object_settings(dict(object=dict(id="tilted_box", shape="box", size=[0.035] * 3)), tmp_path)["object"]
    belt_top, success_height, hold = 0.7622119447849692, 0.05, 15
    threshold, half, angle = belt_top + success_height, 0.0175, np.deg2rad(1.0)
    # Independent analytic support of a cube tilted about Y, including its lower corner.
    extent = half * (np.cos(angle) + np.sin(angle))
    quaternion = [np.cos(angle / 2), 0, np.sin(angle / 2), 0]

    def obs(tick):
        center_z = threshold + half - 0.001 if tick == 0 else threshold + extent + 0.001
        if tick == 1:
            center_z = threshold + half + 0.000165
        pose = np.r_[0, 0, center_z, quaternion]
        return dict(
            eef10=encode_eef10([0, 0, 1], np.eye(3), -1),
            qpos=np.zeros(8),
            qvel=np.zeros(8),
            head=np.full((8, 12, 3), tick, dtype=np.uint8),
            sim_time=tick / 30,
            control_tick=tick,
            physics_step=tick * 10,
            oracle_debug=dict(
                cube_pose=pose.copy(),
                cube_velocity=np.zeros(3),
                finger_contact_forces=np.full(2, 15.0),
                object_pose=pose.copy(),
                object_bottom_z=center_z - extent,
            ),
        )

    recorder = EpisodeRecorder()

    def append(tick):
        recorder.append(
            dict(
                observation=obs(tick),
                next_observation=obs(tick + 1),
                action=obs(tick)["eef10"],
                joint_target=np.ones(8),
            ),
            dict(control_tick=tick),
        )

    for tick in range(hold):
        append(tick)
    meta = dict(
        success=False,
        failure_reason="synthetic_partial_hold",
        seed=1,
        belt_speed=0.03,
        source_commit="unit-test",
        object=obj,
        belt_top=belt_top,
        success_height=success_height,
        success_hold_ticks=hold,
        cube_size=0.035,
    )
    recorder.save(tmp_path, "early", meta, video=False)
    arrays, saved, _ = read_episode(tmp_path / "early")
    assert np.all(arrays["oracle_debug.cube_pose"][-hold:, 2] - half > threshold)
    assert np.count_nonzero(arrays["oracle_debug.object_bottom_z"][-hold:] > threshold) == hold - 1
    # A prematurely asserted success must fail strict disk readback despite the legacy cube_size.
    saved.update(success=True, failure_reason=None)
    (tmp_path / "early/metadata.json").write_text(json.dumps(saved))
    with pytest.raises(ValueError, match="success lift evidence missing"):
        read_episode(tmp_path / "early")
    append(hold)
    meta.update(success=True, failure_reason=None)
    recorder.save(tmp_path, "complete", meta, video=False)
    _, _, summary = read_episode(tmp_path / "complete")
    assert summary["success"] and summary["actions"] == hold + 1
