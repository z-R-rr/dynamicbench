"""Resolve the production conveyor layout from the original saved RoboDojo scene."""

import ast
import copy
import hashlib
import json

import numpy as np
import yaml

from dynamicbench.envs.poses import quat_wxyz_matrix


def official_joint_defaults(path):
    tree = ast.parse(path.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "InitialStateCfg":
            for keyword in node.keywords:
                if keyword.arg == "joint_pos":
                    return ast.literal_eval(keyword.value)
    raise ValueError(f"No literal official joint defaults in {path}")


def resolve_production_layout(settings, root):
    result = copy.deepcopy(settings)
    layout = result["layout"]
    profile = layout.get("profile", "compact")
    if profile == "compact":
        return result
    if profile != "official":
        raise ValueError("Unknown layout profile")
    belt = result["belt"]
    if belt.get("source") != "robodojo" or result["scene"].get("source") != "robodojo":
        raise ValueError(
            "Official layout requires RoboDojo belt and scene; use --layout-profile compact for primitives"
        )
    reference = root / layout["reference_layout"]
    original = json.loads(reference.read_text())
    entries = original["Dynamic"]["conveyor"]
    if len(entries) != 1:
        raise ValueError("Expected exactly one saved conveyor placement")
    source = entries[0]
    directory = root / "Assets/Object/RoboDojo/Dynamic/conveyor" / f"{source['category_idx']:05d}"
    metadata_path = directory / "metadata.json"
    metadata = json.loads(metadata_path.read_text())
    position = np.asarray(source["default_pos"], dtype=float)
    scale = np.asarray(source["scale"], dtype=float)
    quaternion = np.asarray(source["default_ori"], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    rotation = quat_wxyz_matrix(quaternion)
    if np.any(scale <= 0) or not np.isfinite(position).all():
        raise ValueError("Invalid saved conveyor transform")
    vertices = np.asarray(metadata["geometry"]["aligned_bbox"]["vertices"])
    world = (vertices * scale) @ rotation.T + position
    support = np.asarray(metadata["passive"]["support"]["conveyor/0"]["center"])[:, :3]
    support = (support * scale) @ rotation.T + position
    # Bootstrap height only: the executor measures the spawned Belt mesh before
    # reset and derives the actual cube initialization and success plane from it.
    height = float(np.median(support[:, 2]))
    belt.update(
        asset_index=f"{source['category_idx']:05d}",
        root_transform=dict(position=position.tolist(), quaternion_wxyz=quaternion.tolist(), scale=scale.tolist()),
        center=[*world.mean(0)[:2], height - 0.04],
        size=[*(world.max(0) - world.min(0))[:2], 0.08],
        source_layout=str(reference),
        support_height_metadata=height,
    )
    result["scene"]["official_ground"] = copy.deepcopy(original["Ground"])
    if result["robot"].get("embodiment", "x5") == "x5":
        config_path = root / "env/robot_manager/robot_config/x5.py"
        defaults = official_joint_defaults(config_path)
        result["robot"]["initial_q"] = [float(defaults[n]) for n in result["robot"]["arm_joint_names"]]
        result["robot"]["initial_pose_source"] = str(config_path)
        robots = yaml.safe_load((root / layout["robot_config"]).read_text())["robots"]
        result["layout"]["passive_robots"] = [r for i, r in enumerate(robots) if i != layout["robot_index"]]
        layout.setdefault("source_sha256", {})[str(config_path.relative_to(root))] = hashlib.sha256(
            config_path.read_bytes()
        ).hexdigest()
    for path in (reference, metadata_path):
        layout.setdefault("source_sha256", {})[str(path.relative_to(root))] = hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
    return result
