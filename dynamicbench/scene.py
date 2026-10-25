"""Reuse RoboDojo fixture components while keeping task physics unchanged."""

import copy
import hashlib

import yaml


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
    room.update(usd_path=str(files[0]), collision=False)
    background = original["Background"].copy()
    background["texture_path"] = str(root / "Assets/Background" / background["default"])
    if not (root / "Assets/Background" / background["default"]).is_file():
        raise FileNotFoundError(background["texture_path"])
    cfg.update(room=room, background=background, source_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    return result


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
    disabled_colliders = 0
    for prim in list(Usd.PrimRange(stage.GetPrimAtPath(root))):
        if not prim.IsValid() or not prim.IsActive():
            continue
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
            disabled_colliders += 1
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
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
    audit = dict(
        components=["Room", "Background", "Light"],
        component_backend="USD adapters (Sim 6.1)" if ISAAC61 else "RoboDojo component classes",
        room=cfg["room"],
        background=cfg["background"],
        lights=light_cfgs,
        room_colliders_disabled=disabled_colliders,
        physics_ground="unchanged collider; hidden from RGB",
        source_sha256=cfg["source_sha256"],
    )
    return (room, background, lights), audit
