"""Exact paths for the deliberately single-environment scene.

Isaac Lab's deletion callback treats ``.*`` as a cross-directory regex. A
foreign asset's nested Cube must never invalidate our task cube's callbacks.
"""

ENV_PRIM_PATH = "/World/envs/env_0"
CUBE_PRIM_PATH = f"{ENV_PRIM_PATH}/Cube"
