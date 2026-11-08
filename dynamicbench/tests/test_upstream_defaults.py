"""Execute the unchanged upstream interpolation function without importing Isaac.

This is a source/behavior regression contract, not a full legacy GPU episode.
"""

import ast
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]


def test_original_default_frequencies_are_preserved():
    sim = yaml.safe_load((ROOT / "env_cfg/sim/sim_config.yml").read_text())
    robot = yaml.safe_load((ROOT / "env_cfg/arx_x5.yml").read_text())
    assert sim["dt"] == 0.004 and sim["decimation"] == 1
    assert robot["observation"]["collect_freq"] == 25


def test_original_actual_source_retains_eighty_twenty_interpolation():
    tree = ast.parse((ROOT / "src/eval_client/eval_env.py").read_text())
    matches = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "process_control_info"]
    assert len(matches) == 1
    namespace = dict(np=np, deepcopy=deepcopy)
    exec(
        compile(ast.Module(body=matches, type_ignores=[]), str(ROOT / "src/eval_client/eval_env.py"), "exec"), namespace
    )
    robot = SimpleNamespace(
        type="target",
        arm_name="arm",
        gripper_name="ee",
        ee_type="gripper",
        gripper_scale=[0, 0.044],
        gripper_move={"mimic": ["joint8", 1, 0]},
    )
    manager = SimpleNamespace(
        robot_list=[robot],
        process_name=lambda n: n + "_joint_state",
        get_joint=lambda *a, **k: {0: np.zeros(6)},
        get_end_effector_real_val=lambda *a, **k: {0: np.zeros(2)},
    )
    env = SimpleNamespace(robot_manager=manager, obs_manager=SimpleNamespace(collect_interval=10))
    target = dict(arm_joint_state={"position": np.ones(6)}, ee_joint_state={"position": [0.044, 0.044]})
    seq = namespace["process_control_info"](env, target, 0)
    assert len(seq) == 10
    np.testing.assert_allclose([x["arm_joint_state"]["position"][0] for x in seq], np.r_[np.arange(1, 9) / 9, 1, 1])
    np.testing.assert_allclose(
        [x["ee_joint_state"]["position"][0] for x in seq], np.r_[np.arange(1, 9) / 9, 1, 1] * 0.044
    )
