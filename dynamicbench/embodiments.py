"""Task-local robot adapters. Upstream RoboDojo robot configurations are read-only."""

import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from dynamicbench.envs.kinematics import URDFArmIK
from dynamicbench.robot_sources import resolve_robot_asset_config


def resolve_embodiment(settings, root, name="x5", piper_asset_root=None):
    if name == "x5":
        result = resolve_robot_asset_config(settings, root)
        result["robot"].update(
            embodiment="x5", usd_path=str(root / "Assets/Robots/x5/ARX.usd"), gripper_actuator="gripper"
        )
        return result
    if name not in {"franka", "piper"}:
        raise ValueError("unknown embodiment")
    from env_cfg.camera.template import D435

    result = copy.deepcopy(settings)
    directory = (
        root / "Assets/Robots/franka"
        if name == "franka"
        else Path(piper_asset_root)
        if piper_asset_root
        else root.parent / "RoboTwin/assets/embodiments/piper"
    )
    path = directory / ("robot_config.yml" if name == "franka" else "config.yml")
    source = yaml.safe_load(path.read_text())
    urdf = (directory / source["urdf_path"]).resolve()
    tree = ET.parse(urdf).getroot()
    arm = source["arm_joints_name"] if name == "franka" else source["arm_joints_name"][0]
    gripper = (
        source["gripper_joints_name"]
        if name == "franka"
        else [source["gripper_name"][0]["base"], source["gripper_name"][0]["mimic"][0][0]]
    )
    eef = source["ee_link"] if name == "franka" else source["move_group"][0]
    fingers, bounds = [], []
    for joint_name in gripper:
        j = tree.find(f"joint[@name='{joint_name}']")
        if j is None or j.attrib["type"] != "prismatic":
            raise ValueError("expected two independently driven prismatic finger joints")
        fingers.append(j.find("child").attrib["link"])
        bounds.append([float(j.find("limit").attrib[k]) for k in ("lower", "upper")])
    # Both selected source URDFs use positive joint travel to open each finger.
    lo = max(source["gripper_scale"][0], *(b[0] for b in bounds))
    hi = min(source["gripper_scale"][1], *(b[1] for b in bounds))
    if not lo < hi:
        raise ValueError("invalid gripper limits")
    rotation = np.diag([1.0, -1.0, -1.0])
    if name == "piper":
        # The source Piper wrist has a +/-1.22 rad joint5 limit. An angled
        # approach stays inside it; the symmetric jaw yaw avoids the joint6 seam.
        rotation = Rotation.from_euler("y", -30, degrees=True).as_matrix() @ rotation @ np.diag([-1.0, -1.0, 1.0])
    offset = [0.0, 0.0, float(source["gripper_bias"])]
    rc = result["robot"]
    rc.update(
        embodiment=name,
        asset_config_path=str(path),
        urdf_path=str(urdf),
        usd_path=str(directory / "Franka.usd") if name == "franka" else None,
        arm_joint_names=arm,
        gripper_joint_names=gripper,
        finger_links=fingers,
        eef_link=eef,
        tcp_offset_link6=offset,
        tcp_offset_eef=offset,
        gripper_range=[lo, hi],
        official_gripper_command_scale=source["gripper_scale"],
        official_delta_matrix=source["delta_matrix"],
        base_position=[-0.65, -0.02, 0.765],
        base_quaternion_wxyz=[1.0, 0.0, 0.0, 0.0],
        gripper_actuator="panda_hand" if name == "franka" else "gripper",
        grasp_rotation_world=rotation.tolist(),
        initial_pose_source="task pregrasp URDF IK",
        arm_drive_stiffness=2000.0,
        arm_drive_damping=40.0,
        asset_source="RoboDojo Franka" if name == "franka" else "RoboTwin Piper",
        gripper_effort=min(rc["gripper_effort"], 10.0) if name == "piper" else rc["gripper_effort"],
    )
    # Preserve the task's belt/cube and head camera. Each adapter gets a reachable
    # pregrasp pose, not the X5's joint values or base pose.
    ik = URDFArmIK(urdf, rc["base_position"], np.eye(3), arm, end_link=eef)
    tcp = np.array(
        [result["cube"]["initial_xy"][0], -0.12, result["belt"]["center"][2] + result["belt"]["size"][2] / 2 + 0.185]
    )
    position = tcp - rotation @ np.asarray(offset)
    rng = np.random.default_rng(42)
    seeds = [(ik.lower + ik.upper) / 2] + [rng.uniform(ik.lower + 0.01, ik.upper - 0.01) for _ in range(24)]
    for seed in seeds:
        q = ik.solve(position, rotation, seed)
        if q is not None:
            rc["initial_q"] = q.tolist()
            break
    else:
        raise ValueError(f"{name}: pregrasp pose unreachable")
    result["camera"]["wrist"] = dict(
        name="task_wrist",
        mount_link=eef,
        position=[0.05, 0.0, 0.025],
        quaternion_wxyz=[0.0, 1.0, 0.0, 0.0],
        lens=copy.deepcopy(D435),
        type="d435",
        source="DynamicBench task-mounted D435; not an official robot camera mount",
    )
    result["layout"]["source_sha256"].update({str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in (path, urdf)})
    return result


def articulation_config(rc, root):
    """Called only after AppLauncher. Conversion writes to a task-owned cache."""
    name = rc.get("embodiment", "x5")
    if name != "piper":
        import importlib

        return importlib.import_module(f"env.robot_manager.robot_config.{name}").get_robot_config()
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    import isaaclab.sim as sim_utils
    from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg

    digest = hashlib.sha256(Path(rc["urdf_path"]).read_bytes()).hexdigest()[:16]
    cache = root / "outputs/assets/piper" / digest
    converted = UrdfConverter(
        UrdfConverterCfg(
            asset_path=rc["urdf_path"],
            usd_dir=str(cache),
            usd_file_name="piper.usd",
            fix_base=True,
            merge_fixed_joints=False,
            make_instanceable=False,
            joint_drive=UrdfConverterCfg.JointDriveCfg(
                gains=UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=1000.0, damping=200.0)
            ),
        )
    )
    rc["usd_path"] = converted.usd_path
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(converted.usd_path)
    root_path = stage.GetDefaultPrim().GetPath()
    rc["body_prim_paths"] = {}
    for link in [rc["eef_link"], *rc["finger_links"]]:
        matches = [p for p in stage.Traverse() if p.GetName() == link and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(matches) != 1:
            raise ValueError(f"Piper USD must contain exactly one rigid body for {link}")
        rc["body_prim_paths"][link] = str(matches[0].GetPath().MakeRelativePath(root_path))
    return ArticulationCfg(
        spawn=sim_utils.UsdFileCfg(
            usd_path=converted.usd_path,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True, max_depenetration_velocity=5.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                # Native URDF importer already authored a world fixed joint.
                # Re-fixing relocates a referenced Newton root in Lab 3 and fails.
                fix_root_link=None,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                **dict.fromkeys(rc["arm_joint_names"], 0.0),
                **dict.fromkeys(rc["gripper_joint_names"], rc["gripper_range"][1]),
            }
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=rc["arm_joint_names"], effort_limit_sim=100.0, stiffness=1000.0, damping=200.0
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=rc["gripper_joint_names"], effort_limit_sim=10.0, stiffness=1000.0, damping=200.0
            ),
        },
    )
