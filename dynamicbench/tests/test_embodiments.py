import numpy as np
import pytest

from dynamicbench.envs.kinematics import URDFArmIK


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
