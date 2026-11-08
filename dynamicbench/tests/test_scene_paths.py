"""Regression for the pinned Isaac Lab AssetBase deletion matching contract."""

import re

from dynamicbench.envs.scene_paths import CUBE_PRIM_PATH, ENV_PRIM_PATH


def deletion_invalidates(asset_expression, deleted_path):
    # Exact predicate from the server's AssetBase._on_prim_deletion. It treats
    # env_.* as a Python regex, so .* is allowed to consume path separators.
    prefix = "/".join(asset_expression.split("/")[: deleted_path.count("/") + 1])
    return re.match("^" + prefix + "$", deleted_path) is not None


def test_embedded_demo_cube_deletion_preserves_task_cube_subscription():
    foreign_cube = f"{ENV_PRIM_PATH}/OriginalConveyor/Cube"
    assert deletion_invalidates("/World/envs/env_.*/Cube", foreign_cube)
    assert not deletion_invalidates(CUBE_PRIM_PATH, foreign_cube)
    assert deletion_invalidates(CUBE_PRIM_PATH, CUBE_PRIM_PATH)
    assert deletion_invalidates(CUBE_PRIM_PATH, ENV_PRIM_PATH)
