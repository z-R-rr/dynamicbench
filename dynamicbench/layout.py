"""Resolve original RoboDojo placement and lens configuration without copying it."""

import copy
import hashlib

import numpy as np
from scipy.spatial.transform import Rotation
import yaml


def official_camera_quaternion(orientation):
    # CameraManager calls euler_angles_to_quat(..., extrinsic=False), whose
    # actual default is intrinsic XYZ. Lower-case xyz only agrees for one axis.
    if len(orientation) == 3:
        return Rotation.from_euler("XYZ", orientation, degrees=True).as_quat()[[3, 0, 1, 2]].tolist()
    if len(orientation) != 4:
        raise ValueError("Camera orientation must be XYZ degrees or WXYZ quaternion")
    return list(orientation)


def resolve_robodojo_layout(settings, root):
    from env_cfg.camera import template

    result = copy.deepcopy(settings)
    layout = result["layout"]
    robot_path = root / layout["robot_config"]
    camera_path = root / layout["camera_config"]
    robot = yaml.safe_load(robot_path.read_text())["robots"][layout["robot_index"]]
    camera = yaml.safe_load(camera_path.read_text())[layout["head_camera"]]["camera"]
    lens = getattr(template, camera["type"].upper())
    quaternion = np.asarray(robot["default_root_rot"], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    result["robot"].update(base_position=list(robot["default_root_pos"]), base_quaternion_wxyz=quaternion.tolist())
    orientation = official_camera_quaternion(camera["ori"])
    result.setdefault("camera", {}).update(
        head_position=list(camera["pos"]),
        head_quaternion_wxyz=list(orientation),
        width=lens["resolution"][0],
        height=lens["resolution"][1],
        head_lens=copy.deepcopy(lens),
    )
    layout["source_sha256"] = {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in (robot_path, camera_path, root / "env_cfg/camera/template.py")
    }
    return result
