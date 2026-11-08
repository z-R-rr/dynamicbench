"""Reuse RoboDojo fixture components while keeping task physics unchanged."""

import copy
import hashlib

import yaml


def resolve_camera_stand(original, root):
    """Resolve the fixed stand exactly as the official saved-layout generator."""
    matches = [
        entry
        for entry in original.get("Geometry", [])
        if "camera_stand" in entry.get("select_mode", {}).get("label", [])
    ]
    if len(matches) != 1:
        raise ValueError("Expected one official camera_stand entry")
    entry = matches[0]
    common = entry["common"]
    if common.get("rotate_rand", False) or common["relative_plane"] != "Ground":
        raise ValueError("Expected a fixed Ground-relative camera stand")
    categories = entry["category"]
    if len(categories) != 1 or len(categories[0]["index"]) != 1:
        raise ValueError("Expected one fixed camera stand asset")
    category = categories[0]
    directory = root / "Assets/Object/RoboDojo/Geometry" / category["name"] / f"{category['index'][0]:05d}"
    usd = directory / "object.usdz"
    if not usd.is_file():
        usd = directory / "object.usd"
    if not usd.is_file():
        raise FileNotFoundError(usd)
    position = []
    for axis in ("xlim", "ylim", "zlim"):
        if common[axis][0] != common[axis][1]:
            raise ValueError("Camera stand placement must be fixed")
        position.append(float(common[axis][0]))
    ground = original["Ground"]
    if ground["axis"] != "Z":
        raise ValueError("The official conveyor layout expects a Z ground support")
    position[2] += ground["default_pos"][2] + ground["thickness"] / 2
    return dict(
        usd_path=str(usd),
        position=position,
        quaternion_wxyz=common["qpos"],
        scale=[1, 1, 1],
        collision=False,
        source="conveyor.yml Geometry/camera_stand; official GeometryObject default",
        ground_reference_height=ground["default_pos"][2] + ground["thickness"] / 2,
    )


def room_physics_snapshot(stage, root):
    from pxr import Usd, UsdPhysics

    records = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        values = {}
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            values["collision_enabled"] = UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            values["rigid_body_enabled"] = UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        if values:
            records[str(prim.GetPath()).removeprefix(root)] = values
    return records


def resolve_scene(settings, root):
    result = copy.deepcopy(settings)
    cfg = result["scene"]
    path = root / cfg["config"]
    original = yaml.safe_load(path.read_text())
    room = original["Room"].copy()
    if room.get("random", False) or original["Background"].get("random", False):
        raise ValueError("fixture comparison requires a fixed room/background")
    files = sorted((root / "Assets/Room" / room["default"]).glob("*.usd"))
    if len(files) != 1:
        raise FileNotFoundError(f"expected one room USD: {room['default']}")
    room.update(usd_path=str(files[0]))
    background = original["Background"].copy()
    background["texture_path"] = str(root / "Assets/Background" / background["default"])
    if not (root / "Assets/Background" / background["default"]).is_file():
        raise FileNotFoundError(background["texture_path"])
    cfg.update(
        room=room,
        background=background,
        camera_stand=resolve_camera_stand(original, root),
        source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
    )
    if "official_ground" in cfg:
        ground = cfg["official_ground"]
        if ground["geometry"] != "cube" or ground["axis"] != "Z":
            raise ValueError("Official conveyor ground must be a Z cube mesh")
        files = sorted((root / "Assets/Material" / ground["materials"]["default"]).glob("**/*.mdl"))
        if len(files) != 1:
            raise FileNotFoundError("Expected one official ground MDL")
        ground["mdl_path"] = str(files[0])
        ground["mdl_sha256"] = hashlib.sha256(files[0].read_bytes()).hexdigest()
    return result


def spawn_official_ground(cfg, stage):
    """Mirror RoboDojo Ground's mesh, UVs, op order and material on Sim 6.1."""
    import isaaclab.sim as sim_utils
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    path = "/World/Ground"
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([Gf.Vec3f(x, y, z) for z in (-1, 1) for y in (-1, 1) for x in (-1, 1)])
    mesh.CreateFaceVertexCountsAttr([4] * 6)
    mesh.CreateFaceVertexIndicesAttr([4, 5, 7, 6, 1, 0, 2, 3, 2, 6, 7, 3, 0, 1, 5, 4, 5, 1, 3, 7, 0, 4, 6, 2])
    mesh.CreateNormalsAttr(
        [Gf.Vec3f(*n) for n in [(0, 0, 1), (0, 0, -1), (0, 1, 0), (0, -1, 0), (1, 0, 0), (-1, 0, 0)]]
    )
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.uniform)
    UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying).Set(
        [(0, 0), (1, 0), (1, 1), (0, 1)] * 6
    )
    mesh.AddScaleOp().Set(Gf.Vec3f(3.5, 3.5, cfg["thickness"] / 2))
    pos = list(cfg["default_pos"])
    pos[2] -= cfg["thickness"] / 2
    mesh.AddTranslateOp().Set(Gf.Vec3d(*pos))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim()).CreateCollisionEnabledAttr(True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("none")
    physics = sim_utils.RigidBodyMaterialCfg(**cfg["physics_material"])
    physics.func(path + "/PhysicsMaterial", physics)
    sim_utils.bind_physics_material(path, path + "/PhysicsMaterial")
    material = sim_utils.MdlFileCfg(mdl_path=cfg["mdl_path"])
    material.func(path + "/Looks/OfficialGround", material)
    sim_utils.bind_visual_material(path, path + "/Looks/OfficialGround")
    return dict(prim_path=path, visible=True, source="official Ground mesh/UV/op order and MDL", config=cfg)


def spawn_scene(cfg, stage):
    from omegaconf import OmegaConf
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    from scipy.spatial.transform import Rotation

    from dynamicbench.envs.runtime import ISAAC61

    if ISAAC61:
        from dynamicbench.envs.fixtures61 import Background, Light, Room
    else:
        from env.scene_manager.objects.background import Background
        from env.scene_manager.objects.light import Light
        from env.scene_manager.objects.room import Room

    root = "/World/envs/env_0/Room"
    room = Room(prim_path=root, usd_path=cfg["room"]["usd_path"], instance_config=cfg["room"])
    lights = []
    light_cfgs = []
    room_physics = room_physics_snapshot(stage, root)
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath(root))):
        if not prim.IsValid() or not prim.IsActive():
            continue
        if prim.GetTypeName() != "RectLight":
            continue
        # Recreate the room's authored light through RoboDojo's Light class.
        # Bake its composed transform into world dimensions; the room itself
        # retains its original .5 scale and referenced material/texture paths.
        transform = Gf.Transform(UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
        pos, scale = list(transform.GetTranslation()), list(transform.GetScale())
        quat = transform.GetRotation().GetQuat()
        rotation = Rotation.from_quat([*quat.GetImaginary(), quat.GetReal()]).as_euler("xyz", degrees=True).tolist()

        def attr(name, fallback):
            value = prim.GetAttribute("inputs:" + name).Get()
            return fallback if value is None else value

        intensity = float(attr("intensity", 0)) * 2 ** float(attr("exposure", 0))
        intensity *= cfg.get("light_intensity_scale", 1.0)
        color = list(attr("color", (1, 1, 1)))
        width, height = float(attr("width", 1)) * abs(scale[0]), float(attr("height", 1)) * abs(scale[1])
        spec = dict(
            common=dict(initial_pos_range=pos + pos, initial_ori_range=rotation + rotation, color_range=[color, color]),
            Rect=dict(
                intensity_range=[intensity, intensity], width_range=[width, width], height_range=[height, height]
            ),
        )
        if attr("enableColorTemperature", False):
            temp = float(attr("colorTemperature", 6500))
            spec["common"]["color_temperature_range"] = [temp, temp]
        original_path = str(prim.GetPath())
        prim.SetActive(False)
        light_path = f"/World/envs/env_0/Lighting/Rect{len(lights)}"
        lights.append(Light(light_path, "Rect", OmegaConf.create(spec)))
        light_cfgs.append(dict(source_prim=original_path, prim_path=light_path, config=spec))
    if not lights:
        raise RuntimeError("RoboDojo room has no authored RectLight to reuse")
    background = Background("/World/Background", OmegaConf.create(cfg["background"]))
    stand_cfg = cfg["camera_stand"]
    stand_path = "/World/envs/env_0/CameraStand"
    stand = stage.DefinePrim(stand_path, "Xform")
    stand.GetReferences().AddReference(stand_cfg["usd_path"])
    stand_xf = UsdGeom.Xformable(stand)
    stand_xf.ClearXformOpOrder()
    stand_xf.AddTranslateOp().Set(Gf.Vec3d(*stand_cfg["position"]))
    q = stand_cfg["quaternion_wxyz"]
    stand_xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(float(q[0]), Gf.Vec3d(*q[1:])).GetNormalized()
    )
    stand_xf.AddScaleOp().Set(Gf.Vec3f(*stand_cfg["scale"]))
    # Match GeometryObject's defaults on the stand root only. Room physics
    # and descendant asset settings are left authored by their source USDs.
    UsdPhysics.CollisionAPI.Apply(stand).CreateCollisionEnabledAttr(stand_cfg["collision"])
    if stand.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI(stand).CreateRigidBodyEnabledAttr(False)
    audit = dict(
        components=["Room", "Background", "Light", "camera_stand"],
        component_backend="USD adapters (Sim 6.1)" if ISAAC61 else "RoboDojo component classes",
        room=cfg["room"],
        background=cfg["background"],
        lights=light_cfgs,
        camera_stand=dict(prim_path=stand_path, **stand_cfg),
        room_colliders_disabled=0,
        room_collision_policy="preserve source-authored CollisionAPI and RigidBodyAPI values",
        room_physics_source=room_physics,
        physics_ground="unchanged collider; hidden from RGB",
        source_sha256=cfg["source_sha256"],
    )
    return (room, background, lights, stand), audit
