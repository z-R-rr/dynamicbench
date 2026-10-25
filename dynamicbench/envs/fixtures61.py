"""Static RoboDojo fixture configurations authored through USD in Sim 6.1.

Sim 6 removed SingleGeometryPrim/SingleXFormPrim and core.utils. These adapters
reuse the same asset, transforms and light values without importing that API.
They intentionally accept only the fixed fixtures produced by scene.py.
"""

import carb.settings
from isaaclab.sim.utils import get_current_stage
from pxr import Gf, Sdf, UsdGeom, UsdLux
from scipy.spatial.transform import Rotation


def transform(prim, position, wxyz, scale=(1, 1, 1)):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp(opSuffix="dynamicbench61").Set(Gf.Vec3d(*position))
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble, opSuffix="dynamicbench61").Set(
        Gf.Quatd(float(wxyz[0]), Gf.Vec3d(*wxyz[1:]))
    )
    xform.AddScaleOp(opSuffix="dynamicbench61").Set(Gf.Vec3f(*scale))


class Room:
    def __init__(self, prim_path, usd_path, instance_config):
        self.prim = get_current_stage().DefinePrim(prim_path, "Xform")
        self.prim.GetReferences().AddReference(usd_path)
        transform(
            self.prim,
            instance_config.get("default_pos", [0, 0, 0]),
            instance_config.get("default_ori", [1, 0, 0, 0]),
            instance_config.get("scale", [1, 1, 1]),
        )


class Background:
    def __init__(self, prim_path, inst_config):
        settings = carb.settings.get_settings()
        if settings.get("/rtx/rendermode") == "RaytracedLighting":
            settings.set("/omni/replicator/RTSubframes", max(3, settings.get("/omni/replicator/RTSubframes") or 0))
        self.light_prim = UsdLux.DomeLight.Define(get_current_stage(), prim_path)
        self.light_prim.CreateIntensityAttr(float(inst_config["intensity"]))
        self.light_prim.CreateTextureFileAttr(Sdf.AssetPath(inst_config["texture_path"]))


def fixed_pair(values):
    if values[0] != values[1]:
        raise ValueError("6.1 fixture adapter requires fixed lighting values")
    return values[0]


class Light:
    def __init__(self, prim_path, light_type, config):
        if light_type != "Rect":
            raise ValueError("conveyor fixture adapter supports the room's Rect lights")
        common, rect = config["common"], config["Rect"]
        pos, angles = common["initial_pos_range"], common["initial_ori_range"]
        if list(pos[:3]) != list(pos[3:]) or list(angles[:3]) != list(angles[3:]):
            raise ValueError("6.1 fixture adapter requires fixed transforms")
        light = UsdLux.RectLight.Define(get_current_stage(), prim_path)
        q = Rotation.from_euler("xyz", angles[:3], degrees=True).as_quat()
        transform(light.GetPrim(), pos[:3], q[[3, 0, 1, 2]])
        light.CreateColorAttr(Gf.Vec3f(*fixed_pair(common["color_range"])))
        light.CreateIntensityAttr(float(fixed_pair(rect["intensity_range"])))
        light.CreateWidthAttr(float(fixed_pair(rect["width_range"])))
        light.CreateHeightAttr(float(fixed_pair(rect["height_range"])))
        enabled = "color_temperature_range" in common
        light.CreateEnableColorTemperatureAttr(enabled)
        if enabled:
            light.CreateColorTemperatureAttr(float(fixed_pair(common["color_temperature_range"])))
        self.light_prim = light
