"""Simulator-side single-body asset validation and spawning."""

from pathlib import Path

import isaaclab.sim as sim_utils
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics


def object_spawn_config(obj):
    if obj["source"] == "usd":
        stage = Usd.Stage.Open(obj["usd_path"])
        root = stage.GetDefaultPrim()
        bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if not root or len(bodies) != 1 or bodies[0] != root:
            raise ValueError("Object USD requires exactly one rigid body on its default prim")
        if any(
            p.IsA(UsdPhysics.Joint) or "Graph" in p.GetTypeName() or p.IsA(UsdPhysics.Scene) for p in stage.Traverse()
        ):
            raise ValueError("Object USD must not contain joints, physics scenes or runtime graphs")
        if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in stage.Traverse()):
            raise ValueError("Object USD requires authored colliders")
        # Audit every external USD dependency; USDZ archives are already hashed whole.
        import hashlib

        from pxr import UsdUtils

        layers, assets, unresolved = UsdUtils.ComputeAllDependencies(obj["usd_path"])
        if unresolved:
            raise ValueError(f"Unresolved object USD dependencies: {unresolved}")
        obj["dependency_sha256"] = {}
        for name in [*(layer.realPath for layer in layers), *assets]:
            path = Path(name)
            if path.is_file():
                obj["dependency_sha256"][str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        box = (
            UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
            .ComputeUntransformedBound(root)
            .ComputeAlignedRange()
        )
        lo, hi = np.asarray(box.GetMin()) * obj["scale"], np.asarray(box.GetMax()) * obj["scale"]
        if not np.isfinite([lo, hi]).all() or np.any(hi <= lo):
            raise ValueError("Object USD has empty or invalid geometry bounds")
        obj["geometry"] = dict(
            kind="usd_local_bounds", center=((lo + hi) / 2).tolist(), half_extents=((hi - lo) / 2).tolist()
        )
        factory, shape = sim_utils.UsdFileCfg, dict(usd_path=obj["usd_path"], scale=tuple(obj["scale"]))
    else:
        if obj["shape"] == "box":
            factory, shape = sim_utils.CuboidCfg, dict(size=tuple(obj["size"]))
        elif obj["shape"] == "cylinder":
            factory, shape = sim_utils.CylinderCfg, dict(radius=obj["radius"], height=obj["height"], axis="Z")
        else:
            factory, shape = sim_utils.SphereCfg, dict(radius=obj["radius"])
    return factory(
        **shape,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            kinematic_enabled=False,
            disable_gravity=False,
            sleep_threshold=0.0,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=4,
            max_depenetration_velocity=1,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=obj["mass"]),
        collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=obj["friction"],
            dynamic_friction=obj["friction"],
            restitution=0,
            friction_combine_mode="max",
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(obj["color"])) if "color" in obj else None,
    )
