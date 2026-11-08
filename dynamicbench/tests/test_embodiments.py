import numpy as np
import pytest

from dynamicbench.envs.kinematics import URDFArmIK


@pytest.mark.parametrize("name", ["franka", "piper"])
def test_new_embodiments_inherit_installation_without_moving_scene(tmp_path, monkeypatch, name):
    import copy

    import yaml

    from dynamicbench.embodiments import resolve_embodiment
    from dynamicbench.envs.poses import quat_wxyz_matrix

    directory = tmp_path / ("Assets/Robots/franka" if name == "franka" else "piper")
    directory.mkdir(parents=True)
    source = dict(
        urdf_path="robot.urdf", gripper_bias=0.12, gripper_scale=[-0.01, 0.04], delta_matrix=np.eye(3).tolist()
    )
    if name == "franka":
        source.update(arm_joints_name=["a"], gripper_joints_name=["g1", "g2"], ee_link="tool")
    else:
        source.update(
            arm_joints_name=[["a"]], gripper_name=[dict(base="g1", mimic=[["g2", 1.0, 0.0]])], move_group=["tool"]
        )
    (directory / ("robot_config.yml" if name == "franka" else "config.yml")).write_text(yaml.safe_dump(source))
    (directory / "robot.urdf").write_text("""<robot name="fixture">
      <joint name="g1" type="prismatic"><child link="f1"/><limit lower="0" upper="0.04"/></joint>
      <joint name="g2" type="prismatic"><child link="f2"/><limit lower="0" upper="0.04"/></joint></robot>""")
    settings = dict(
        robot=dict(
            base_position=[-0.3, -0.45, 0.765], base_quaternion_wxyz=[2**-0.5, 0, 0, 2**-0.5], gripper_effort=15
        ),
        layout=dict(robot_config="official_front.yml", source_sha256={}),
        belt=dict(center=[-0.12, -0.02, 0.725], size=[0.7, 0.28, 0.08]),
        cube=dict(initial_xy=[-0.25, -0.02]),
        camera=dict(head_position=[1, 2, 3], head_lens={"f": 13}),
    )
    before = copy.deepcopy(settings)
    seen = {}

    class IK:
        lower, upper = np.array([-1.0]), np.array([1.0])

        def __init__(self, path, position, rotation, joints, end_link):
            seen.update(position=position, rotation=rotation)

        def solve(self, position, rotation, seed):
            return np.zeros(1)

    monkeypatch.setattr("dynamicbench.embodiments.URDFArmIK", IK)
    result = resolve_embodiment(settings, tmp_path, name, directory)
    assert settings == before
    assert result["robot"]["base_position"] == before["robot"]["base_position"] == seen["position"]
    assert result["robot"]["base_quaternion_wxyz"] == before["robot"]["base_quaternion_wxyz"]
    np.testing.assert_allclose(seen["rotation"], quat_wxyz_matrix(before["robot"]["base_quaternion_wxyz"]))
    assert result["belt"] == before["belt"] and result["cube"] == before["cube"]
    for key, value in before["camera"].items():
        assert result["camera"][key] == value
    assert result["camera"]["wrist"]["position"] == [0.05, 0, 0.025]
    assert result["camera"]["wrist"]["quaternion_wxyz"] == [0, 1, 0, 0]


def test_fixed_tool_chain_is_included_in_fk_and_ik(tmp_path):
    path = tmp_path / "arm.urdf"
    path.write_text("""<robot name="test"><link name="base"/><link name="arm"/><link name="tool"/>
      <joint name="j" type="revolute"><parent link="base"/><child link="arm"/>
      <origin xyz="0 0 0.3"/><axis xyz="0 0 1"/><limit lower="-2" upper="2"/></joint>
      <joint name="tool_fixed" type="fixed"><parent link="arm"/><child link="tool"/>
      <origin xyz="0.1 0 0.107" rpy="0 0 -0.7853981633974483"/></joint></robot>""")
    ik = URDFArmIK(path, [0, 0, 0], np.eye(3), ["j"], end_link="tool")
    transform = ik.fk([np.pi / 2])
    np.testing.assert_allclose(transform[:3, 3], [0, 0.1, 0.407], atol=1e-12)
    q = ik.solve(transform[:3, 3], transform[:3, :3], [0.5])
    np.testing.assert_allclose(q, [np.pi / 2], atol=1e-5)
    with pytest.raises(ValueError, match="chain"):
        URDFArmIK(path, [0, 0, 0], np.eye(3), ["wrong_joint"], end_link="tool")


def test_partial_interval_cannot_silently_change_expert_clock():
    from dynamicbench.envs.fixed_rate_controller import FixedRateController
    from dynamicbench.tests.test_zoh import Backend

    backend = Backend()
    controller = FixedRateController(backend)
    with pytest.raises(ValueError, match="evaluation timing"):
        controller.step(controller.previous_action, hold=True, physics_steps=5)
    assert backend.physics_step == 0


def test_v2_timestamps_are_explicit_and_v1_stays_strict(tmp_path):
    from dynamicbench.data.dataset_reader import read_episode, validate
    from dynamicbench.tests.test_dataset import fixture

    arrays, metadata, _ = read_episode(fixture(tmp_path))
    arrays["physics_step"] = np.array([0, 10, 15, 25])
    arrays["timestamp.sim"] = arrays["physics_step"] / 300
    with pytest.raises(ValueError, match="physical step"):
        validate(arrays, metadata)
    metadata["schema_version"] = "dynamicbench.eef10.eval.v2"
    assert validate(arrays, metadata)["duration"] == 25 / 300
    arrays["timestamp.sim"][2] += 0.01
    with pytest.raises(ValueError, match="timestamp"):
        validate(arrays, metadata)
