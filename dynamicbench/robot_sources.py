"""Resolve X5 end-effector and camera values from the shipped asset config."""

import copy
import hashlib
import xml.etree.ElementTree as ET

from scipy.spatial.transform import Rotation
import yaml


def resolve_robot_asset_config(settings, root):
    from env_cfg.camera import template

    result = copy.deepcopy(settings)
    layout = result["layout"]
    robots = yaml.safe_load((root / layout["robot_config"]).read_text())["robots"]
    name = robots[layout["robot_index"]]["robot_name"]
    if name != "x5":
        raise ValueError("The conveyor embodiment adapter currently supports the official X5 only")
    directory = root / "Assets/Robots" / name
    path = directory / "robot_config.yml"
    source = yaml.safe_load(path.read_text())
    urdf = directory / source["urdf_path"]
    tree = ET.parse(urdf).getroot()
    if source["ee_link"] != "link6":
        raise ValueError("The existing EEF10 contract requires the official X5 link6 frame")
    gripper = [tree.find(f"joint[@name='{joint}']") for joint in source["gripper_joints_name"]]
    if any(joint is None for joint in gripper):
        raise ValueError("Official gripper joints missing in URDF")
    lower = max(float(joint.find("limit").attrib["lower"]) for joint in gripper)
    upper = min(float(joint.find("limit").attrib["upper"]) for joint in gripper)
    # RoboDojo's command scale can extend beyond the physical URDF limits.
    # Resolve their intersection instead of duplicating either range in YAML.
    limits = [max(lower, source["gripper_scale"][0]), min(upper, source["gripper_scale"][1])]
    if limits[0] >= limits[1]:
        raise ValueError("Official gripper command scale and URDF limits do not overlap")
    result["robot"].update(
        asset_config_path=str(path),
        urdf_path=str(urdf),
        eef_link=source["ee_link"],
        arm_joint_names=source["arm_joints_name"],
        gripper_joint_names=source["gripper_joints_name"],
        finger_links=[joint.find("child").attrib["link"] for joint in gripper],
        tcp_offset_link6=[float(source["gripper_bias"]), 0.0, 0.0],
        gripper_range=limits,
        official_gripper_command_scale=source["gripper_scale"],
        official_delta_matrix=source["delta_matrix"],
    )
    cameras = source["camera"]
    if len(cameras) != 1 or cameras[0]["mesh"] != "pinhole":
        raise ValueError("Expected one official X5 pinhole wrist camera")
    camera = cameras[0]
    if tree.find(f"link[@name='{camera['link']}']") is None:
        raise ValueError("Official wrist mount is absent from the URDF")
    angles = camera["ori"]
    quaternion = (
        Rotation.from_euler("xyz", angles, degrees=True).as_quat()[[3, 0, 1, 2]].tolist()
        if len(angles) == 3
        else list(angles)
    )
    result["camera"]["wrist"] = dict(
        name=camera["name"],
        mount_link=camera["link"],
        position=list(camera["pos"]),
        quaternion_wxyz=quaternion,
        lens=copy.deepcopy(getattr(template, camera["type"].upper())),
        type=camera["type"],
        source=str(path),
    )
    layout.setdefault("source_sha256", {}).update(
        {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in (path, urdf)}
    )
    return result
