import pytest

from dynamicbench.runtime.policy_contract import CheckedPolicy


@pytest.mark.parametrize(
    "embodiment,robot,frame",
    [
        ("x5", "arx_x5", "world_link6"),
        ("piper", "piper", "world_link6"),
        ("franka", "franka_panda", "world_panda_hand"),
    ],
)
def test_model_frame_and_robot_are_checked_before_loading(embodiment, robot, frame):
    config = dict(
        robot=robot,
        action_frame=frame,
        action_representation="absolute_eef10",
        rotation_6d="first_two_columns_concatenated",
        gripper="minus1_closed_plus1_open",
        control_hz=30,
        checkpoint="unit-test",
        checkpoint_sha256="0" * 64,
        normalization="unit-test",
        training_data="unit-test",
        model_class="unit-test",
    )
    CheckedPolicy.validate_config(config, embodiment=embodiment)
    with pytest.raises(ValueError, match="action_frame"):
        CheckedPolicy.validate_config({**config, "action_frame": "tcp"}, embodiment=embodiment)
    with pytest.raises(ValueError, match="robot"):
        CheckedPolicy.validate_config({**config, "robot": "unverified"}, embodiment=embodiment)
    with pytest.raises(ValueError, match="control_hz"):
        CheckedPolicy.validate_config({**config, "control_hz": 20}, embodiment=embodiment)
