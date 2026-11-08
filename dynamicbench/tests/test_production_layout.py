import json

import numpy as np
import pytest

from dynamicbench.production_layout import resolve_production_layout


def fixture(root):
    (root / "layout.json").write_text(
        json.dumps(
            dict(
                Dynamic=dict(
                    conveyor=[
                        dict(
                            category_idx=1,
                            default_pos=[-2.0, 0.2, 0.05],
                            default_ori=[0.0, 0.0, 0.0, 1.0],
                            scale=[1.0, 1.0, 1.0],
                        )
                    ]
                ),
                Ground=dict(geometry="cube", thickness=0.1, axis="Z", default_pos=[0, 0, 0]),
            )
        )
    )
    asset = root / "Assets/Object/RoboDojo/Dynamic/conveyor/00001"
    asset.mkdir(parents=True)
    (asset / "metadata.json").write_text(
        json.dumps(
            dict(
                geometry=dict(aligned_bbox=dict(vertices=[[-4, -0.6, 0], [0, 0.6, 1]])),
                passive=dict(support={"conveyor/0": dict(center=[[-2, 0, 0.7, 1, 0, 0, 0]])}),
            )
        )
    )
    (root / "robots.yml").write_text(
        "robots: [{robot_name: x5}, {robot_name: x5, default_root_pos: [0.3, -0.45, 0.765]}]"
    )
    source = root / "env/robot_manager/robot_config/x5.py"
    source.parent.mkdir(parents=True)
    source.write_text("cfg = ArticulationCfg.InitialStateCfg(joint_pos={'joint1': 0.1, 'joint2': 0.2})")
    return dict(
        layout=dict(profile="official", reference_layout="layout.json", robot_config="robots.yml", robot_index=0),
        scene=dict(source="robodojo"),
        belt=dict(source="robodojo", size=[0.7, 0.28, 0.08]),
        robot=dict(embodiment="x5", arm_joint_names=["joint1", "joint2"], initial_q=[1, 2]),
    )


def test_official_transform_and_home_follow_sources_not_compact_constants(tmp_path):
    from dynamicbench.embodiments import resolve_initial_pose

    settings = fixture(tmp_path)
    result = resolve_production_layout(settings, tmp_path)
    assert result["belt"]["root_transform"] == dict(
        position=[-2.0, 0.2, 0.05], quaternion_wxyz=[0.0, 0.0, 0.0, 1.0], scale=[1.0, 1.0, 1.0]
    )
    np.testing.assert_allclose(result["belt"]["size"][:2], [4, 1.2])
    assert result["robot"]["initial_q"] == [0.1, 0.2]
    assert resolve_initial_pose(result) == result
    assert len(result["layout"]["passive_robots"]) == 1
    assert "official_ground" in result["scene"]
    assert len(result["layout"]["source_sha256"]) == 3
    assert settings["robot"]["initial_q"] == [1, 2]
    assert settings["belt"]["size"] == [0.7, 0.28, 0.08]


def test_compact_is_explicit_and_official_cannot_silently_use_primitives(tmp_path):
    settings = fixture(tmp_path)
    settings["belt"]["source"] = "primitive"
    with pytest.raises(ValueError, match="Official layout requires"):
        resolve_production_layout(settings, tmp_path)
    settings["layout"]["profile"] = "compact"
    result = resolve_production_layout(settings, tmp_path)
    assert result == settings
    assert "root_transform" not in result["belt"]


@pytest.mark.parametrize("name", ["franka", "piper"])
def test_cli_official_pregrasp_ignores_compact_height(tmp_path, monkeypatch, name):
    from types import SimpleNamespace

    import yaml

    from dynamicbench.cli import load_settings

    settings = fixture(tmp_path)
    settings["robot"]["gripper_effort"] = 15
    settings["cube"] = dict(initial_xy=[-0.25, -0.02])
    settings["layout"].update(camera_config="cameras.yml", head_camera="cam_head")
    (tmp_path / "robots.yml").write_text(
        "robots: [{default_root_pos: [-0.3, -0.45, 0.765], default_root_rot: [0.707, 0, 0, 0.707]}]"
    )
    (tmp_path / "cameras.yml").write_text("cam_head: {camera: {type: d435, pos: [0, -0.41, 1.308], ori: [30, 0, 0]}}")
    template = tmp_path / "env_cfg/camera/template.py"
    template.parent.mkdir(parents=True)
    template.write_text("# Source-hash fixture; lens values come from the imported camera template.\n")
    directory = tmp_path / ("Assets/Robots/franka" if name == "franka" else "piper")
    directory.mkdir(parents=True)
    source = dict(urdf_path="robot.urdf", gripper_bias=0.12, gripper_scale=[0.0, 0.04], delta_matrix=np.eye(3).tolist())
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
    targets = []

    class IK:
        lower, upper = np.array([-1.0]), np.array([1.0])

        def __init__(self, *args, **kwargs):
            pass

        def solve(self, position, rotation, seed):
            tcp = position + rotation @ np.array([0.0, 0.0, source["gripper_bias"]])
            # Saved support z=0.7, translated by 0.05; no compact height may reach IK.
            np.testing.assert_allclose(tcp, [-0.25, -0.12, 0.75 + 0.185])
            targets.append(tcp)
            return np.array([tcp[2]])

    monkeypatch.setattr("dynamicbench.cli.ROOT", tmp_path)
    monkeypatch.setattr("dynamicbench.embodiments.URDFArmIK", IK)
    monkeypatch.setattr("dynamicbench.rendering.resolve_render_settings", lambda root: {})
    monkeypatch.setattr("dynamicbench.scene.resolve_scene", lambda settings, root: settings)
    args = SimpleNamespace(
        config=tmp_path / "task.yml",
        embodiment=name,
        piper_asset_root=directory,
        belt_source=None,
        lift_xy_mode=None,
        lift_tcp_frame=None,
        scene_source=None,
        layout_profile=None,
    )
    poses = []
    for compact_height in (0.725, 20.0):
        settings["belt"]["center"] = [-0.12, -0.02, compact_height]
        args.config.write_text(yaml.safe_dump(settings))
        result = load_settings(args)
        poses.append(result["robot"]["initial_q"])
        assert result["belt"]["support_height_metadata"] == pytest.approx(0.75)
    assert len(targets) == 2
    assert poses[0] == poses[1]
