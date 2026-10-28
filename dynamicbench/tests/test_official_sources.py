from pathlib import Path
import shutil

import numpy as np
import pytest
import yaml

from dynamicbench.rendering import resolve_render_settings
from dynamicbench.robot_sources import resolve_robot_asset_config
from dynamicbench.scene import resolve_scene

ROOT = Path(__file__).resolve().parents[2]


def test_robot_tcp_and_wrist_follow_asset_file_and_physical_joint_limits(tmp_path):
    asset = tmp_path / "Assets/Robots/x5"
    asset.mkdir(parents=True)
    (tmp_path / "robots.yml").write_text("robots: [{robot_name: x5}]")
    source = dict(
        urdf_path="robot.urdf",
        ee_link="link6",
        gripper_bias=0.177,
        arm_joints_name=["a", "b"],
        gripper_joints_name=["g1", "g2"],
        gripper_scale=[-0.02, 0.05],
        delta_matrix=np.eye(3).tolist(),
        camera=[
            dict(
                name="wrist",
                link="native_mount",
                mesh="pinhole",
                type="d435",
                pos=[0.001, 0.002, 0.003],
                ori=[0, 0, 90],
            )
        ],
    )
    (asset / "robot_config.yml").write_text(yaml.safe_dump(source))
    (asset / "robot.urdf").write_text("""<robot name="fake"><link name="native_mount"/>
      <joint name="g1"><child link="finger_a"/><limit lower="0" upper="0.044"/></joint>
      <joint name="g2"><child link="finger_b"/><limit lower="0.001" upper="0.04"/></joint></robot>""")
    original = dict(layout=dict(robot_config="robots.yml", robot_index=0), robot={}, camera={})
    result = resolve_robot_asset_config(original, tmp_path)
    assert original["robot"] == {} and original["camera"] == {}
    assert result["robot"]["tcp_offset_link6"] == [0.177, 0, 0]
    assert result["robot"]["gripper_range"] == [0.001, 0.04]
    assert result["robot"]["finger_links"] == ["finger_a", "finger_b"]
    assert result["camera"]["wrist"]["mount_link"] == "native_mount"
    assert result["camera"]["wrist"]["position"] == [0.001, 0.002, 0.003]
    assert np.allclose(result["camera"]["wrist"]["quaternion_wxyz"], [2**-0.5, 0, 0, 2**-0.5])
    assert result["camera"]["wrist"]["lens"]["focal_length"] == 13
    assert len(result["layout"]["source_sha256"]) == 2


def test_missing_robot_source_does_not_fall_back_to_duplicated_values(tmp_path):
    (tmp_path / "robots.yml").write_text("robots: [{robot_name: x5}]")
    settings = dict(layout=dict(robot_config="robots.yml", robot_index=0), robot={}, camera={})
    with pytest.raises(FileNotFoundError):
        resolve_robot_asset_config(settings, tmp_path)


def test_scene_preserves_room_collision_and_resolves_official_stand(tmp_path):
    source = yaml.safe_load((ROOT / "env_cfg/scene/conveyor.yml").read_text())
    # An explicitly authored source choice must also survive resolution.
    source["Room"]["collision"] = True
    (tmp_path / "scene.yml").write_text(yaml.safe_dump(source))
    room = tmp_path / "Assets/Room/Simple_Room/base.usd"
    hdr = tmp_path / "Assets/Background/brown_photostudio_02_4k.hdr"
    stand = tmp_path / "Assets/Object/RoboDojo/Geometry/camera_stand/00000/object.usd"
    for path in (room, hdr, stand):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    result = resolve_scene(dict(scene=dict(config="scene.yml")), tmp_path)["scene"]
    assert result["room"]["collision"] is True
    assert result["camera_stand"]["position"] == [0, -0.47, 0.765]
    assert result["camera_stand"]["quaternion_wxyz"] == [0.707, -0.707, 0, 0]
    assert result["camera_stand"]["usd_path"] == str(stand)


def test_quality_profile_is_not_just_an_antialiasing_flag(tmp_path):
    render = resolve_render_settings(ROOT)
    assert render["defaults"]["rendering_mode"] == "quality"
    assert render["defaults"]["antialiasing_mode"] == "DLAA"
    assert render["defaults"]["enable_reflections"] and render["defaults"]["enable_global_illumination"]
    for key in (
        "/rtx/reflections/enabled",
        "/rtx/indirectDiffuse/enabled",
        "/rtx-transient/dldenoiser/enabled",
        "/rtx/ambientOcclusion/enabled",
    ):
        assert render["quality_settings"][key] is True
    # Guard against silently keeping the wrong preset if upstream changes mode.
    target = tmp_path / "env/environment/base_env.py"
    target.parent.mkdir(parents=True)
    shutil.copyfile(ROOT / "env/environment/base_env.py", target)
    target.write_text(target.read_text().replace('"rendering_mode": "quality"', '"rendering_mode": "balanced"'))
    with pytest.raises(ValueError, match="quality"):
        resolve_render_settings(tmp_path)
