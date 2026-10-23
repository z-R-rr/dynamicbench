"""Isolated ARX X5 scene; import only after Isaac Sim AppLauncher starts."""
from pathlib import Path
import json
import math
import numpy as np
import torch
from omegaconf import OmegaConf
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import CameraCfg, ContactSensor, ContactSensorCfg
from env.environment.base_env import BaseEnv
from env.robot_manager.robot_config.x5 import get_robot_config
from env.camera_manager.capture.render_sync import wait_for_latest_cameras
from .kinematics import URDFArmIK
from .poses import decode_eef10, encode_eef10, quat_wxyz_matrix
from .physics_clock import integral_ratio

ROOT = Path(__file__).resolve().parents[2]


def audit_assets(asset_root):
    root = Path(asset_root)
    required = [root / 'Robots/x5/ARX.usd', root / 'Robots/x5/X5A.urdf']
    for path in required:
        if not path.is_file():
            raise FileNotFoundError(f'asset_missing:{path}; run bash scripts/init_assets.sh')
    conveyors = []
    for path in sorted((root / 'Object/RoboDojo/Dynamic/conveyor').glob('*/object.usdz')):
        stage = Usd.Stage.Open(str(path))
        motion = []
        for prim in stage.Traverse():
            values = {a.GetName(): str(a.Get()) for a in prim.GetAttributes()
                      if any(word in a.GetName().lower() for word in ('velocity', 'speed'))}
            if values or prim.GetTypeName() in ('OmniGraph', 'OmniGraphNode'):
                motion.append(dict(path=str(prim.GetPath()), type=prim.GetTypeName(), values=values))
        conveyors.append(dict(path=str(path), motion=motion))
    return dict(robot_assets=[str(p) for p in required], original_conveyors=conveyors,
                selected_belt='primitive collision belt + PhysxSurfaceVelocityAPI; original USD not assumed functional')


def camera_quaternion(position, target):
    z = np.asarray(position, float) - target
    z /= np.linalg.norm(z)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    from scipy.spatial.transform import Rotation
    q = Rotation.from_matrix(np.column_stack([x, y, z])).as_quat()
    return tuple(q[[3, 0, 1, 2]])


class ConveyorCubeTask(BaseEnv):
    def __init__(self, settings, app, seed=42, belt_speed=None):
        self.settings = settings
        self.seed_value = int(seed)
        self.generation = 0
        self.speed = settings['belt']['speed'] if belt_speed is None else float(belt_speed)
        if self.speed not in (0.0, 0.03, 0.06):
            raise ValueError('MVP only accepts nonnegative speed 0, .03, .06; see report for validated values')
        integral_ratio(1 / settings['physics_hz'], settings['control_hz'])
        self.asset_audit = audit_assets(ROOT / 'Assets')
        cfg = OmegaConf.create(dict(dt=1/300, decimation=1, render_interval=10,
                                    seed=seed, device='cpu', use_fabric=False,
                                    scene=dict(num_envs=1, env_spacing=7),
                                    physx=dict(enable_ccd=True, enable_stabilization=True),
                                    render=dict(antialiasing_mode='TAA')))
        scene = InteractiveSceneCfg(num_envs=1, env_spacing=7, replicate_physics=False)
        rc = settings['robot']
        robot_cfg = get_robot_config()
        robot_cfg.spawn.activate_contact_sensors = True
        robot_cfg.actuators['gripper'].effort_limit_sim = rc['gripper_effort']
        robot_cfg.init_state.pos = tuple(rc['base_position'])
        robot_cfg.init_state.rot = tuple(rc['base_quaternion_wxyz'])
        for i, q in enumerate(rc['initial_q']):
            robot_cfg.init_state.joint_pos[f'joint{i+1}'] = q
        scene.robot0 = robot_cfg.replace(prim_path='{ENV_REGEX_NS}/robot0')
        bc, cc = settings['belt'], settings['cube']
        self.belt_top = bc['center'][2] + bc['size'][2] / 2
        self.cube_start = np.r_[cc['initial_xy'], self.belt_top + cc['size']/2 + .002]
        material = sim_utils.RigidBodyMaterialCfg(static_friction=bc['friction'],
                    dynamic_friction=bc['friction'], restitution=0,
                    friction_combine_mode='max')
        scene.belt = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Belt',
            spawn=sim_utils.CuboidCfg(size=tuple(bc['size']),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True, disable_gravity=True),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=.001, rest_offset=0),
                mass_props=sim_utils.MassPropertiesCfg(mass=10), physics_material=material,
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.12,.15,.18))),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(bc['center'])))
        scene.cube = RigidObjectCfg(prim_path='{ENV_REGEX_NS}/Cube',
            spawn=sim_utils.CuboidCfg(size=(cc['size'],)*3,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False,
                    solver_position_iteration_count=16, solver_velocity_iteration_count=4,
                    max_depenetration_velocity=1),
                mass_props=sim_utils.MassPropertiesCfg(mass=cc['mass']),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=.001, rest_offset=0),
                physics_material=material.replace(static_friction=cc['friction'], dynamic_friction=cc['friction']),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1,.55,.02))),
            init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(self.cube_start)))
        cam = settings['camera']
        scene.head = CameraCfg(prim_path='{ENV_REGEX_NS}/Head', update_period=0,
            height=cam['height'], width=cam['width'], data_types=['rgb'],
            spawn=sim_utils.PinholeCameraCfg(focal_length=18, focus_distance=1,
                                           horizontal_aperture=20.955, clipping_range=(.05,10)),
            offset=CameraCfg.OffsetCfg(pos=tuple(cam['head_position']),
                rot=camera_quaternion(cam['head_position'], cam['head_target']), convention='opengl'))
        # Contact sensor paths are resolved from the actual articulation after launch.
        super().__init__(cfg, app, interactiveSceneCfg=scene)
        self.launch_sim(cfg)
        self.robot = self.sim.scene['robot0']
        self.cube = self.sim.scene['cube']
        self.head = self.sim.scene['head']
        self.arm_ids, self.arm_names = self.robot.find_joints([f'joint{i}' for i in range(1,7)], preserve_order=True)
        self.grip_ids, _ = self.robot.find_joints(['joint7','joint8'], preserve_order=True)
        self.eef_id = self.robot.body_names.index('link6')
        self.finger_ids = [self.robot.body_names.index(x) for x in ('link7', 'link8')]
        self.ik = URDFArmIK(ROOT/'Assets/Robots/x5/X5A.urdf', rc['base_position'],
                           quat_wxyz_matrix(rc['base_quaternion_wxyz']), self.arm_names)
        self.target = self.robot.data.default_joint_pos[0].numpy().copy()
        # USD link hierarchy is not assumed; use paths exposed by PhysX.
        self.contact_sensors = []
        for name in ('link7', 'link8'):
            matches = [p.GetPath().pathString for p in self.stage.Traverse()
                       if p.GetName() == name and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            if len(matches) != 1:
                raise RuntimeError(f'finger_contact_path_unresolved:{name}:{matches}')
            sensor = ContactSensor(ContactSensorCfg(prim_path=matches[0], update_period=0,
                history_length=1, filter_prim_paths_expr=['/World/envs/env_0/Cube']))
            self.contact_sensors.append(sensor)
        self.reset(seed)

    @property
    def physics_step(self):
        return self.sim.unwrapped._sim_step_counter

    def _setup_scene(self, sim):
        super()._setup_scene(sim)
        ground = sim_utils.CuboidCfg(size=(4,4,.05),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(.25,.25,.28)))
        ground.func('/World/Ground', ground, translation=(0,0,-.025))
        sim_utils.DomeLightCfg(intensity=1800).func('/World/Light', sim_utils.DomeLightCfg(intensity=1800))
        prim = self.stage.GetPrimAtPath('/World/envs/env_0/Belt')
        self.surface = PhysxSchema.PhysxSurfaceVelocityAPI.Apply(prim)
        self.surface.CreateSurfaceVelocityEnabledAttr(True)
        self.surface.CreateSurfaceVelocityLocalSpaceAttr(False)
        self.surface.CreateSurfaceVelocityAttr(Gf.Vec3f(0,0,0))

    def _post_setup_scene(self, sim):
        pass

    def reset(self, seed=None, options=None):
        self.seed_value = self.seed_value if seed is None else int(seed)
        self.generation += 1
        self.sim.reset()
        q = self.robot.data.default_joint_pos.clone()
        self.robot.write_joint_state_to_sim(q, torch.zeros_like(q))  # reset only
        self.target = q[0].numpy().copy()
        self.robot.set_joint_position_target(q)
        state = self.cube.data.default_root_state.clone()
        rng = np.random.default_rng(self.seed_value)
        state[0,:3] = torch.tensor(self.cube_start + np.r_[rng.uniform(
            -self.settings['cube']['initial_jitter'], self.settings['cube']['initial_jitter'],2),0])
        state[0,7:] = 0
        self.cube.write_root_state_to_sim(state)  # reset only, never during grasp
        self.surface.GetSurfaceVelocityAttr().Set(Gf.Vec3f(0,0,0))
        for _ in range(150):
            self.sim_step(render=False)
        for sensor in self.contact_sensors:
            sensor.reset()
        actual = self.robot.data.body_link_pose_w[0,self.eef_id].numpy()
        predicted = self.ik.fk(self.robot.data.joint_pos[0,self.arm_ids].numpy())
        if np.linalg.norm(actual[:3]-predicted[:3,3]) > .005:
            raise RuntimeError(f'URDF_USD_FK_mismatch:{actual[:3]}:{predicted[:3,3]}')
        angle = np.linalg.norm(__import__('scipy').spatial.transform.Rotation.from_matrix(
            quat_wxyz_matrix(actual[3:]) @ predicted[:3,:3].T).as_rotvec())
        if angle > .03:
            raise RuntimeError(f'URDF_USD_rotation_mismatch:{angle}')
        self.before_physics_step()
        self.success_count = 0
        self.max_cube_z = float(self.cube.data.root_pos_w[0,2])

    def before_physics_step(self):
        v = np.asarray(self.settings['belt']['direction']) * self.speed
        self.surface.GetSurfaceVelocityAttr().Set(Gf.Vec3f(*v.tolist()))

    def sim_step(self, render=False):
        super().sim_step(render=render)
        for sensor in getattr(self,'contact_sensors',[]):
            sensor.update(1/300, force_recompute=True)

    def actual_joint_target(self):
        return self.robot.data.joint_pos[0].numpy().copy()

    def solve_target(self, action):
        pos, rot, grip = decode_eef10(action)
        q = self.robot.data.joint_pos[0,self.arm_ids].numpy()
        limits = self.robot.data.soft_joint_pos_limits[0,self.arm_ids].numpy()
        solution = self.ik.solve(pos,rot,q,(limits[:,0],limits[:,1]))
        if solution is None:
            return None, 'ik_failure'
        target = self.target.copy()
        # Rate-limit commanded targets as well as actual-state tracking error.
        delta = self.settings['robot']['max_joint_velocity']/30
        previous = target[self.arm_ids]
        solution = np.clip(solution, previous-delta, previous+delta)
        if np.any(solution < limits[:,0]) or np.any(solution > limits[:,1]):
            return None, 'joint_limit'
        target[self.arm_ids] = solution
        lo, hi = self.settings['robot']['gripper_range']
        target[self.grip_ids] = lo + (grip+1)/2*(hi-lo)
        return target, None

    def set_joint_target(self, q):
        self.target = np.asarray(q).copy()
        self.robot.set_joint_position_target(torch.tensor(q,dtype=torch.float32).unsqueeze(0))
        self.robot.set_joint_velocity_target(torch.zeros((1,len(q))))

    def observe(self):
        # Rendering/dispatch can take wall time but cannot advance physics.
        step = self.physics_step
        wait_for_latest_cameras(self)
        self.head.update(0, force_recompute=True)
        image = self.head.data.output['rgb'][0,:,:,:3].cpu().numpy().copy()
        if image.dtype != np.uint8 or image.size == 0 or image.shape[-1] != 3:
            raise RuntimeError('render_resource_unavailable')
        if self.physics_step != step:
            raise RuntimeError('render_advanced_physics')
        qpos = self.robot.data.joint_pos[0].numpy().copy()
        pose = self.robot.data.body_link_pose_w[0,self.eef_id].numpy().copy()
        lo,hi = self.settings['robot']['gripper_range']
        grip = float(np.clip((qpos[self.grip_ids].mean()-lo)/(hi-lo)*2-1,-1,1))
        forces = [float(s.data.force_matrix_w.norm().item()) for s in self.contact_sensors]
        cube = self.cube.data.root_state_w[0].numpy().copy()
        return dict(eef10=encode_eef10(pose[:3],quat_wxyz_matrix(pose[3:]),grip),
            qpos=qpos, qvel=self.robot.data.joint_vel[0].numpy().copy(), head=image,
            oracle_debug=dict(cube_pose=cube[:7], cube_velocity=cube[7:10],
                              finger_contact_forces=np.asarray(forces)))

    def outcome(self, observation, tick):
        dbg = observation['oracle_debug']
        cube = dbg['cube_pose'][:3]
        if not np.isfinite(cube).all():
            return False, 'numerical_instability'
        self.max_cube_z = max(self.max_cube_z,float(cube[2]))
        if cube[2] < self.belt_top-.08:
            return False, 'dropped'
        bc = self.settings['belt']
        if abs(cube[0]-bc['center'][0]) > bc['size'][0]/2 or abs(cube[1]-bc['center'][1]) > bc['size'][1]/2:
            return False, 'out_of_reach'
        # Bottom of the same dynamic cube must clear the belt by the threshold,
        # with physical contact from BOTH fingers at each successful control tick.
        lifted = cube[2]-self.settings['cube']['size']/2 > self.belt_top+self.settings['expert']['success_height']
        contacts = np.all(dbg['finger_contact_forces'] > .01)
        self.success_count = self.success_count+1 if lifted and contacts else 0
        if self.success_count >= self.settings['expert']['success_hold_ticks']:
            return True, None
        if tick >= self.settings['expert']['timeout_ticks']:
            return False, 'timeout'
        return False, None
