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
    settings = fixture(tmp_path)
    result = resolve_production_layout(settings, tmp_path)
    assert result["belt"]["root_transform"] == dict(
        position=[-2.0, 0.2, 0.05], quaternion_wxyz=[0.0, 0.0, 0.0, 1.0], scale=[1.0, 1.0, 1.0]
    )
    np.testing.assert_allclose(result["belt"]["size"][:2], [4, 1.2])
    assert result["robot"]["initial_q"] == [0.1, 0.2]
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
