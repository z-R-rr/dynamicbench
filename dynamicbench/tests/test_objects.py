from copy import deepcopy

import numpy as np
import pytest

from dynamicbench.objects import grasp_position, resolve_object_settings, spawn_position, vertical_bounds


def box(root):
    return resolve_object_settings(dict(object=dict(id="test", shape="box", size=[0.03, 0.04, 0.06])), root)["object"]


def test_legacy_cube_matches_default_geometry_and_landing(tmp_path):
    legacy = dict(cube=dict(size=0.035, mass=0.05, friction=1.5, initial_xy=[-0.25, -0.02], initial_jitter=0.005))
    before = deepcopy(legacy)
    cfg = resolve_object_settings(legacy, tmp_path)
    assert "cube" not in cfg and legacy == before
    p = spawn_position(cfg["object"], 0.762)
    np.testing.assert_allclose(p, [-0.25, -0.02, 0.762 + 0.0175 + 0.002])
    p[2] -= 0.002  # settled support
    np.testing.assert_allclose(grasp_position(cfg["object"], [*p, 1, 0, 0, 0]), [-0.25, -0.02, 0.762 + 0.035 * 0.55])


def test_rotated_box_and_offset_asset_land_on_bottom_not_origin(tmp_path):
    cfg = box(tmp_path)
    cfg["initial_quaternion_wxyz"] = [2**-0.5, 0, 2**-0.5, 0]
    pose = [*spawn_position(cfg, 0.7), *cfg["initial_quaternion_wxyz"]]
    assert np.isclose(vertical_bounds(cfg["geometry"], pose)[0], 0.702)
    assert np.isclose(pose[2], 0.717)  # half X after rotation, not half Z
    cfg["geometry"] = dict(kind="usd_local_bounds", center=[0, 0, 0.1], half_extents=[0.02, 0.02, 0.1])
    cfg["initial_quaternion_wxyz"] = [1, 0, 0, 0]
    assert np.isclose(spawn_position(cfg, 0.7)[2], 0.702)  # bottom-origin USD


def test_rolling_cylinder_uses_radial_support():
    q = [2**-0.5, 2**-0.5, 0, 0]
    low, high = vertical_bounds(dict(kind="cylinder", radius=0.02, height=0.15), [0, 0, 1, *q])
    np.testing.assert_allclose([low, high], [0.98, 1.02])


def test_grasp_offset_rotates_with_object(tmp_path):
    cfg = box(tmp_path)
    cfg["grasp"]["position"] = [0.02, 0, 0]
    p = grasp_position(cfg, [1, 2, 3, 2**-0.5, 0, 0, 2**-0.5])
    np.testing.assert_allclose(p, [1, 2.02, 3])


def test_override_replaces_legacy_and_records_asset_hash(tmp_path):
    asset = tmp_path / "asset.usd"
    asset.write_text("placeholder: simulator validates USD structure separately")
    override = tmp_path / "object.yaml"
    override.write_text("object:\n  id: custom\n  source: usd\n  usd_path: asset.usd\n")
    cfg = resolve_object_settings(dict(cube={}), tmp_path, override)["object"]
    assert cfg["usd_path"] == str(asset) and len(cfg["asset_sha256"]) == 64
    assert len(cfg["config_source_sha256"]) == 1


@pytest.mark.parametrize(
    "bad",
    [
        dict(size=[0, 0.04, 0.06]),
        dict(mass=-1),
        dict(color=[2, 0, 0]),
        dict(initial_quaternion_wxyz=[0, 0, 0, 0]),
        dict(grasp=dict(opening=2)),
        dict(typo=True),
    ],
)
def test_invalid_object_rejected_before_simulation(tmp_path, bad):
    spec = dict(id="bad", shape="box", size=[0.03, 0.04, 0.06])
    spec.update(bad)
    with pytest.raises((ValueError, KeyError)):
        resolve_object_settings(dict(object=spec), tmp_path)


def test_ambiguous_old_and_new_config_rejected(tmp_path):
    with pytest.raises(ValueError, match="not both"):
        resolve_object_settings(dict(cube={}, object={}), tmp_path)
