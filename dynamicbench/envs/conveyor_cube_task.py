"""Isolated ARX X5 scene; import only after Isaac Sim AppLauncher starts."""

import json
from pathlib import Path

from isaaclab.assets import RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensorCfg
import isaaclab.sim as sim_utils
import numpy as np
from omegaconf import OmegaConf
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
import torch

from env.camera_manager.capture.render_sync import wait_for_latest_cameras
from env.robot_manager.robot_config.x5 import get_robot_config

from .kinematics import URDFArmIK
from .physics_clock import integral_ratio
from .poses import decode_eef10, encode_eef10, quat_wxyz_matrix
from .runtime import (
    ISAAC61,
    joint_state,
    lab_quaternion,
    position_target,
    public_pose,
    root_state,
    tensor,
    velocity_target,
)
from .scene_paths import CUBE_PRIM_PATH

if ISAAC61:
    from .base_env61 import BaseEnv61 as BaseEnv
else:
    from env.environment.base_env import BaseEnv

ROOT = Path(__file__).resolve().parents[2]


def audit_assets(asset_root):
    root = Path(asset_root)
    required = [root / "Robots/x5/ARX.usd", root / "Robots/x5/X5A.urdf"]
    for path in required:
        if not path.is_file():
            raise FileNotFoundError(f"asset_missing:{path}; run bash scripts/init_assets.sh")
    conveyors = []
    for path in sorted((root / "Object/RoboDojo/Dynamic/conveyor").glob("*/object.usdz")):
        stage = Usd.Stage.Open(str(path))
        motion = []
        for prim in stage.Traverse():
            values = {
                a.GetName(): str(a.Get())
                for a in prim.GetAttributes()
                if any(word in a.GetName().lower() for word in ("velocity", "speed"))
            }
            if values or prim.GetTypeName() in ("OmniGraph", "OmniGraphNode"):
                motion.append(dict(path=str(prim.GetPath()), type=prim.GetTypeName(), values=values))
        conveyors.append(dict(path=str(path), motion=motion))
    return dict(
        robot_assets=[str(p) for p in required],
        original_conveyors=conveyors,
        selected_belt="primitive collision belt + PhysxSurfaceVelocityAPI; original USD not assumed functional",
    )


class ConveyorCubeTask(BaseEnv):
    def __init__(self, settings, app, seed=42, belt_speed=None):
        self.settings = settings
        self.seed_value = int(seed)
        self.generation = 0
        self.belt_source = settings["belt"].get("source", "primitive")
        if self.belt_source not in ("primitive", "robodojo"):
            raise ValueError("unknown belt source")
        self.speed = settings["belt"]["speed"] if belt_speed is None else float(belt_speed)
        if self.speed not in (0.0, 0.03, 0.06):
            raise ValueError("MVP only accepts nonnegative speed 0, .03, .06; see report for validated values")
        integral_ratio(1 / settings["physics_hz"], settings["control_hz"])
        self.asset_audit = audit_assets(ROOT / "Assets")
        cfg = OmegaConf.create(
            dict(
                dt=1 / 300,
                decimation=1,
                render_interval=10,
                seed=seed,
                device="cpu",
                use_fabric=False,
                scene=dict(num_envs=1, env_spacing=7),
                physx=dict(enable_ccd=True, enable_stabilization=True),
                render=settings["render"]["defaults"],
            )
        )
        scene = InteractiveSceneCfg(num_envs=1, env_spacing=7, replicate_physics=False)
        rc = settings["robot"]
        robot_cfg = get_robot_config()
        robot_cfg.spawn.activate_contact_sensors = True
        robot_cfg.actuators["gripper"].effort_limit_sim = rc["gripper_effort"]
        robot_cfg.init_state.pos = tuple(rc["base_position"])
        robot_cfg.init_state.rot = lab_quaternion(rc["base_quaternion_wxyz"])
        for name, q in zip(rc["arm_joint_names"], rc["initial_q"], strict=True):
            robot_cfg.init_state.joint_pos[name] = q
        scene.robot0 = robot_cfg.replace(prim_path="{ENV_REGEX_NS}/robot0")
        bc, cc = settings["belt"], settings["cube"]
        self.belt_top = bc["center"][2] + bc["size"][2] / 2
        self.cube_start = np.r_[cc["initial_xy"], self.belt_top + cc["size"] / 2 + 0.002]
        material = sim_utils.RigidBodyMaterialCfg(
            static_friction=bc["friction"], dynamic_friction=bc["friction"], restitution=0, friction_combine_mode="max"
        )
        primitive_belt = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Belt",
            spawn=sim_utils.CuboidCfg(
                size=tuple(bc["size"]),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0),
                mass_props=sim_utils.MassPropertiesCfg(mass=10),
                physics_material=material,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.12, 0.15, 0.18)),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(bc["center"])),
        )
        if self.belt_source == "primitive":
            scene.belt = primitive_belt
        scene.cube = RigidObjectCfg(
            # This task has one environment. An env_.* regex also matches the
            # original USD's nested demo Cube during prim-deletion callbacks,
            # which would unsubscribe OUR cube before the first PLAY event.
            prim_path=CUBE_PRIM_PATH,
            spawn=sim_utils.CuboidCfg(
                size=(cc["size"],) * 3,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    # Surface-speed changes do not wake a box resting on an
                    # otherwise stationary kinematic actor. Keep this task's
                    # one dynamic box awake; do not drive its pose or velocity.
                    sleep_threshold=0.0,
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                    max_depenetration_velocity=1,
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=cc["mass"]),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0),
                physics_material=material.replace(static_friction=cc["friction"], dynamic_friction=cc["friction"]),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1, 0.55, 0.02)),
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(self.cube_start)),
        )
        cam = settings["camera"]
        lens = cam["head_lens"]
        scene.head = CameraCfg(
            prim_path="{ENV_REGEX_NS}/Head",
            update_period=0,
            height=cam["height"],
            width=cam["width"],
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=lens["focal_length"],
                horizontal_aperture=lens["horizontal_aperture"],
                vertical_aperture=lens["vertical_aperture"],
                clipping_range=tuple(lens["clipping_range"]),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=tuple(cam["head_position"]),
                rot=lab_quaternion(cam["head_quaternion_wxyz"]),
                convention="opengl",
            ),
        )
        scene.finger_left = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/robot0/link7",
            update_period=0,
            history_length=1,
            filter_prim_paths_expr=[CUBE_PRIM_PATH],
        )
        scene.finger_right = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/robot0/link8",
            update_period=0,
            history_length=1,
            filter_prim_paths_expr=[CUBE_PRIM_PATH],
        )
        wrist = cam["wrist"]
        wrist_lens = wrist["lens"]
        scene.wrist = CameraCfg(
            prim_path="{ENV_REGEX_NS}/robot0/" + wrist["mount_link"] + "/DynamicBenchWrist",
            update_period=0,
            height=wrist_lens["resolution"][1],
            width=wrist_lens["resolution"][0],
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=wrist_lens["focal_length"],
                horizontal_aperture=wrist_lens["horizontal_aperture"],
                vertical_aperture=wrist_lens["vertical_aperture"],
                clipping_range=tuple(wrist_lens["clipping_range"]),
            ),
            offset=CameraCfg.OffsetCfg(
                pos=tuple(wrist["position"]),
                rot=lab_quaternion(wrist["quaternion_wxyz"]),
                convention="opengl",
            ),
        )
        if ISAAC61:
            from isaaclab_physx.renderers import IsaacRtxRendererCfg

            from dynamicbench.rendering import isaac61_global_settings

            renderer = IsaacRtxRendererCfg(global_settings=isaac61_global_settings(settings["render"]))
            scene.head.renderer_cfg = renderer
            scene.wrist.renderer_cfg = renderer.copy()
        super().__init__(cfg, app, interactiveSceneCfg=scene)
        self.launch_sim(cfg)
        self.robot = self.sim.scene["robot0"]
        self.cube = self.sim.scene["cube"]
        self.head = self.sim.scene["head"]
        self.wrist = self.sim.scene["wrist"]
        self.arm_ids, self.arm_names = self.robot.find_joints(rc["arm_joint_names"], preserve_order=True)
        self.grip_ids, _ = self.robot.find_joints(rc["gripper_joint_names"], preserve_order=True)
        self.eef_id = self.robot.body_names.index(rc["eef_link"])
        self.finger_ids = [self.robot.body_names.index(x) for x in rc["finger_links"]]
        self.ik = URDFArmIK(
            Path(rc["urdf_path"]),
            rc["base_position"],
            quat_wxyz_matrix(rc["base_quaternion_wxyz"]),
            self.arm_names,
        )
        self.target = tensor(self.robot.data.default_joint_pos)[0].numpy().copy()
        self.contact_sensors = [self.sim.scene["finger_left"], self.sim.scene["finger_right"]]
        self.reset(seed)
        from dynamicbench.rendering import audit_render_settings

        self.asset_audit["render"] = audit_render_settings(settings["render"])
        if not self.asset_audit["render"]["passed"]:
            raise RuntimeError("Official_render_quality_settings_mismatch")

    @property
    def physics_step(self):
        return self.sim.unwrapped._sim_step_counter

    def _setup_scene(self, sim):
        super()._setup_scene(sim)
        ground = sim_utils.CuboidCfg(
            size=(4, 4, 0.05),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.25, 0.25, 0.28)),
        )
        ground.func("/World/Ground", ground, translation=(0, 0, -0.025))
        if self.settings.get("scene", {}).get("source") == "robodojo":
            from dynamicbench.scene import spawn_scene

            UsdGeom.Imageable(self.stage.GetPrimAtPath("/World/Ground")).MakeInvisible()
            self.visual_fixtures, self.asset_audit["scene"] = spawn_scene(self.settings["scene"], self.stage)
        else:
            sim_utils.DomeLightCfg(intensity=1800).func("/World/Light", sim_utils.DomeLightCfg(intensity=1800))
        if self.belt_source == "robodojo":
            prim = self._spawn_original_belt()
        else:
            prim = self.stage.GetPrimAtPath("/World/envs/env_0/Belt")
        self.surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
        self.surface.CreateSurfaceVelocityEnabledAttr(True)
        self.surface.CreateSurfaceVelocityLocalSpaceAttr(False)
        self.surface.CreateSurfaceVelocityAttr(Gf.Vec3f(0, 0, 0))

    def _spawn_original_belt(self):
        """Reuse the conveyor USD with physics-step-owned contact motion.

        Read geometry/support coordinates from metadata. Disable only the
        source instance's demo cube, render products, extra physics scene and
        render-tick graph. Never edit the downloaded source assets.
        """
        bc = self.settings["belt"]
        directory = ROOT / "Assets/Object/RoboDojo/Dynamic/conveyor" / bc.get("asset_index", "00001")
        info = json.loads((directory / "metadata.json").read_text())
        vertices = np.asarray(info["geometry"]["aligned_bbox"]["vertices"])
        bounds_min, bounds_max = vertices.min(0), vertices.max(0)
        scale = np.r_[np.asarray(bc["size"][:2]) / (bounds_max - bounds_min)[:2], 0.2]
        supports = np.asarray(info["passive"]["support"]["conveyor/0"]["center"])
        surface_z = float(np.median(supports[:, 2]))
        xy = np.asarray(bc["center"][:2]) - (bounds_min + bounds_max)[:2] / 2 * scale[:2]
        translation = np.r_[xy, self.belt_top - surface_z * scale[2]]
        path = "/World/envs/env_0/OriginalConveyor"
        spawn = sim_utils.UsdFileCfg(usd_path=str(directory / "object.usdz"), scale=tuple(scale))
        spawn.func(path, spawn, translation=tuple(translation))
        root = self.stage.GetPrimAtPath(path)
        belts = []
        for prim in list(Usd.PrimRange(root)):
            if not prim.IsValid() or not prim.IsActive():
                continue
            if (
                prim.GetTypeName() in ("OmniGraph", "RenderProduct", "RenderSettings", "Camera")
                or prim.IsA(UsdPhysics.Scene)
                or (prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() != "Belt")
            ):
                prim.SetActive(False)
            elif prim.GetName() == "Belt" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                belts.append(prim)
        if len(belts) != 1:
            raise RuntimeError(f"original_conveyor_body_unresolved:{len(belts)}")
        belt = belts[0]
        UsdPhysics.RigidBodyAPI(belt).CreateKinematicEnabledAttr(True)
        material_path = path + "/DynamicBenchContactMaterial"
        material = sim_utils.RigidBodyMaterialCfg(
            static_friction=bc["friction"], dynamic_friction=bc["friction"], restitution=0, friction_combine_mode="max"
        )
        material.func(material_path, material)
        sim_utils.bind_physics_material(belt.GetPath().pathString, material_path)
        self.asset_audit["selected_belt"] = dict(
            source="robodojo",
            usd=str(directory / "object.usdz"),
            rigid_body=str(belt.GetPath()),
            scale=scale.tolist(),
            translation=translation.tolist(),
            authored_surface_z=surface_z,
            motion="per-physics-step PhysxSurfaceVelocityAPI; source graph disabled",
        )
        return belt

    def _post_setup_scene(self, sim):
        print(
            "[DynamicBench] rigid objects: " + str({k: v.cfg.prim_path for k, v in sim.scene.rigid_objects.items()}),
            flush=True,
        )

    def setup_physics(self, sim):
        super().setup_physics(sim)
        # Upstream unconditionally forces GPU dynamics, even with device=cpu.
        # Override it only in this scene; SurfaceVelocity and tensor-device
        # diagnostics must describe the backend that actually runs.
        self.physics_interface.overwrite_gpu_setting(0)
        if self.physics_interface.get_overwrite_gpu_setting() != 0:
            raise RuntimeError("CPU_PhysX_override_failed")

    def reset(self, seed=None, options=None):
        self.seed_value = self.seed_value if seed is None else int(seed)
        self.generation += 1
        self.sim.reset()
        q = tensor(self.robot.data.default_joint_pos).clone()
        joint_state(self.robot, q, torch.zeros_like(q))  # reset only
        self.target = q[0].numpy().copy()
        position_target(self.robot, q)
        state = (
            torch.cat([tensor(self.cube.data.default_root_pose), tensor(self.cube.data.default_root_vel)], dim=-1)
            if ISAAC61
            else self.cube.data.default_root_state
        ).clone()
        rng = np.random.default_rng(self.seed_value)
        state[0, :3] = torch.tensor(
            self.cube_start
            + np.r_[
                rng.uniform(-self.settings["cube"]["initial_jitter"], self.settings["cube"]["initial_jitter"], 2), 0
            ]
        )
        state[0, 7:] = 0
        root_state(self.cube, state)  # reset only, never during grasp
        self.surface.GetSurfaceVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        for _ in range(150):
            self.sim_step(render=False)
        for sensor in self.contact_sensors:
            sensor.reset()
        actual = public_pose(self.robot.data.body_link_pose_w)[0, self.eef_id]
        predicted = self.ik.fk(tensor(self.robot.data.joint_pos)[0, self.arm_ids].numpy())
        if np.linalg.norm(actual[:3] - predicted[:3, 3]) > 0.005:
            raise RuntimeError(f"URDF_USD_FK_mismatch:{actual[:3]}:{predicted[:3, 3]}")
        angle = np.linalg.norm(
            __import__("scipy")
            .spatial.transform.Rotation.from_matrix(quat_wxyz_matrix(actual[3:]) @ predicted[:3, :3].T)
            .as_rotvec()
        )
        if angle > 0.03:
            raise RuntimeError(f"URDF_USD_rotation_mismatch:{angle}")
        if self.settings.get("scene", {}).get("source") == "robodojo":
            from dynamicbench.scene import room_physics_snapshot

            after = room_physics_snapshot(self.stage, "/World/envs/env_0/Room")
            scene_audit = self.asset_audit["scene"]
            scene_audit["room_physics_after_reset"] = after
            scene_audit["room_physics_preserved"] = after == scene_audit["room_physics_source"]
            if not scene_audit["room_physics_preserved"]:
                raise RuntimeError("Room_source_physics_changed")
        self.before_physics_step()
        self.success_count = 0
        self.max_cube_z = float(tensor(self.cube.data.root_pos_w)[0, 2])

    def before_physics_step(self):
        v = np.asarray(self.settings["belt"]["direction"]) * self.speed
        self.surface.GetSurfaceVelocityAttr().Set(Gf.Vec3f(*v.tolist()))

    def sim_step(self, render=False):
        super().sim_step(render=render)

    def actual_joint_target(self):
        return tensor(self.robot.data.joint_pos)[0].numpy().copy()

    def solve_target(self, action):
        pos, rot, grip = decode_eef10(action)
        q = tensor(self.robot.data.joint_pos)[0, self.arm_ids].numpy()
        limits = tensor(self.robot.data.soft_joint_pos_limits)[0, self.arm_ids].numpy()
        solution = self.ik.solve(pos, rot, q, (limits[:, 0], limits[:, 1]))
        if solution is None:
            return None, "ik_failure"
        target = self.target.copy()
        # Rate-limit commanded targets as well as actual-state tracking error.
        delta = self.settings["robot"]["max_joint_velocity"] / 30
        previous = target[self.arm_ids]
        solution = np.clip(solution, previous - delta, previous + delta)
        if np.any(solution < limits[:, 0]) or np.any(solution > limits[:, 1]):
            return None, "joint_limit"
        target[self.arm_ids] = solution
        lo, hi = self.settings["robot"]["gripper_range"]
        target[self.grip_ids] = lo + (grip + 1) / 2 * (hi - lo)
        return target, None

    def set_joint_target(self, q):
        self.target = np.asarray(q).copy()
        position_target(self.robot, torch.tensor(q, dtype=torch.float32).unsqueeze(0))
        velocity_target(self.robot, torch.zeros((1, len(q))))

    def observe(self):
        # Rendering/dispatch can take wall time but cannot advance physics.
        step = self.physics_step
        wait_for_latest_cameras(self)
        self.head.update(0, force_recompute=True)
        image = tensor(self.head.data.output["rgb"])[0, :, :, :3].cpu().numpy().copy()
        self.wrist.update(0, force_recompute=True)
        wrist_image = tensor(self.wrist.data.output["rgb"])[0, :, :, :3].cpu().numpy().copy()
        if image.dtype != np.uint8 or image.size == 0 or image.shape[-1] != 3:
            raise RuntimeError("render_resource_unavailable")
        if self.physics_step != step:
            raise RuntimeError("render_advanced_physics")
        qpos = tensor(self.robot.data.joint_pos)[0].numpy().copy()
        pose = public_pose(self.robot.data.body_link_pose_w)[0, self.eef_id]
        lo, hi = self.settings["robot"]["gripper_range"]
        grip = float(np.clip((qpos[self.grip_ids].mean() - lo) / (hi - lo) * 2 - 1, -1, 1))
        forces = [
            float(tensor(s.data.normal_force_matrix_w if ISAAC61 else s.data.force_matrix_w).norm().item())
            for s in self.contact_sensors
        ]
        cube = (
            np.r_[public_pose(self.cube.data.root_link_pose_w)[0], tensor(self.cube.data.root_com_vel_w)[0].numpy()]
            if ISAAC61
            else self.cube.data.root_state_w[0].numpy().copy()
        )
        rotation = quat_wxyz_matrix(pose[3:])
        finger_midpoint = tensor(self.robot.data.body_link_pos_w)[0, self.finger_ids].mean(dim=0).numpy().copy()
        return dict(
            eef10=encode_eef10(pose[:3], quat_wxyz_matrix(pose[3:]), grip),
            qpos=qpos,
            qvel=tensor(self.robot.data.joint_vel)[0].numpy().copy(),
            head=image,
            wrist=wrist_image,
            oracle_debug=dict(
                cube_pose=cube[:7],
                cube_velocity=cube[7:10],
                finger_contact_forces=np.asarray(forces),
                cube_in_eef=rotation.T @ (cube[:3] - pose[:3]),
                finger_midpoint_in_eef=rotation.T @ (finger_midpoint - pose[:3]),
                cube_in_pinch_frame=rotation.T @ (cube[:3] - finger_midpoint),
                belt_surface_velocity=np.asarray(self.surface.GetSurfaceVelocityAttr().Get()),
            ),
        )

    def outcome(self, observation, tick):
        dbg = observation["oracle_debug"]
        cube = dbg["cube_pose"][:3]
        if not np.isfinite(cube).all():
            return False, "numerical_instability"
        self.max_cube_z = max(self.max_cube_z, float(cube[2]))
        if cube[2] < self.belt_top - 0.08:
            return False, "dropped"
        bc = self.settings["belt"]
        if abs(cube[0] - bc["center"][0]) > bc["size"][0] / 2 or abs(cube[1] - bc["center"][1]) > bc["size"][1] / 2:
            return False, "out_of_reach"
        # Bottom of the same dynamic cube must clear the belt by the threshold,
        # with physical contact from BOTH fingers at each successful control tick.
        lifted = cube[2] - self.settings["cube"]["size"] / 2 > self.belt_top + self.settings["expert"]["success_height"]
        contacts = np.all(dbg["finger_contact_forces"] > 0.01)
        self.success_count = self.success_count + 1 if lifted and contacts else 0
        if self.success_count >= self.settings["expert"]["success_hold_ticks"]:
            return True, None
        if tick >= self.settings["expert"]["timeout_ticks"]:
            return False, "timeout"
        return False, None
