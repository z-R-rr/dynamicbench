"""Single rigid-object configuration and geometry, independent of the simulator."""

import copy
import hashlib
from pathlib import Path

import numpy as np
import yaml

from dynamicbench.envs.poses import quat_wxyz_matrix


def _vector(value, size, name, *, positive=False):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all() or (positive and np.any(a <= 0)):
        raise ValueError(f"Invalid object.{name}")
    return a.tolist()


def _quaternion(value, name):
    q = np.asarray(_vector(value, 4, name))
    if np.linalg.norm(q) < 1e-8:
        raise ValueError(f"Zero object.{name}")
    return (q / np.linalg.norm(q)).tolist()


def resolve_object_settings(settings, root, override=None):
    result = copy.deepcopy(settings)
    source_hashes = {}
    if override is not None:
        path = Path(override).resolve()
        document = yaml.safe_load(path.read_text())
        if not isinstance(document, dict) or set(document) != {"object"}:
            raise ValueError("--object-config requires a YAML document with only an object mapping")
        result.pop("cube", None)
        result["object"] = document["object"]
        source_hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
    if "object" in result and "cube" in result:
        raise ValueError("Use object or legacy cube, not both")
    if "object" not in result:
        old = result.pop("cube")
        result["object"] = dict(
            id="legacy_cube",
            source="primitive",
            shape="box",
            size=[old["size"]] * 3,
            **{k: old[k] for k in ("mass", "friction", "initial_xy", "initial_jitter")},
        )
    obj = result["object"]
    if not isinstance(obj, dict):
        raise ValueError("object must be a mapping")
    allowed = {
        "id",
        "source",
        "shape",
        "size",
        "radius",
        "height",
        "usd_path",
        "scale",
        "mass",
        "friction",
        "color",
        "initial_xy",
        "initial_jitter",
        "initial_quaternion_wxyz",
        "spawn_clearance",
        "grasp",
    }
    if set(obj) - allowed:
        raise ValueError(f"Unknown object keys: {sorted(set(obj) - allowed)}")
    if not isinstance(obj.get("id"), str) or not obj["id"].strip():
        raise ValueError("object.id must be a nonempty string")
    obj.setdefault("source", "primitive")
    for name, default in (("mass", 0.05), ("friction", 1.5), ("initial_jitter", 0.005), ("spawn_clearance", 0.002)):
        value = float(obj.get(name, default))
        if not np.isfinite(value) or value < 0 or (name == "mass" and value == 0):
            raise ValueError(f"Invalid object.{name}")
        obj[name] = value
    obj["initial_xy"] = _vector(obj.get("initial_xy", [-0.25, -0.02]), 2, "initial_xy")
    obj["initial_quaternion_wxyz"] = _quaternion(
        obj.get("initial_quaternion_wxyz", [1, 0, 0, 0]), "initial_quaternion_wxyz"
    )
    if obj["source"] == "primitive":
        if any(k in obj for k in ("usd_path", "scale")):
            raise ValueError("Primitive dimensions use size/radius/height, not usd_path/scale")
        shape = obj.setdefault("shape", "box")
        dimensions = {"box": {"size"}, "cylinder": {"radius", "height"}, "sphere": {"radius"}}
        if ({"size", "radius", "height"} & set(obj)) - dimensions.get(shape, set()):
            raise ValueError("Object dimensions do not match its shape")
        if shape == "box":
            obj["size"] = _vector(obj["size"], 3, "size", positive=True)
            obj["geometry"] = dict(kind="box", center=[0, 0, 0], half_extents=(np.array(obj["size"]) / 2).tolist())
        elif shape in ("cylinder", "sphere"):
            for key in ["radius", "height"] if shape == "cylinder" else ["radius"]:
                obj[key] = float(obj[key])
                if not np.isfinite(obj[key]) or obj[key] <= 0:
                    raise ValueError(f"Invalid object.{key}")
            obj["geometry"] = dict(kind=shape, radius=obj["radius"])
            if shape == "cylinder":
                obj["geometry"]["height"] = obj["height"]
        else:
            raise ValueError("object.shape must be box, cylinder or sphere")
        obj["color"] = _vector(obj.get("color", [1, 0.55, 0.02]), 3, "color")
    elif obj["source"] == "usd":
        if any(k in obj for k in ("shape", "size", "radius", "height")):
            raise ValueError("USD geometry comes from usd_path; use scale for resizing")
        path = Path(obj["usd_path"])
        path = (Path(root) / path).resolve() if not path.is_absolute() else path.resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        obj["usd_path"] = str(path)
        obj["asset_sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
        obj["scale"] = _vector(obj.get("scale", [1, 1, 1]), 3, "scale", positive=True)
        if "color" in obj:
            obj["color"] = _vector(obj["color"], 3, "color")
    else:
        raise ValueError("object.source must be primitive or usd")
    if "color" in obj and np.any((np.array(obj["color"]) < 0) | (np.array(obj["color"]) > 1)):
        raise ValueError("object.color must be in [0,1]")
    grasp = obj.setdefault("grasp", {})
    if set(grasp) - {"position", "quaternion_wxyz", "opening"}:
        raise ValueError("Unknown object.grasp key")
    if "position" in grasp:
        grasp["position"] = _vector(grasp["position"], 3, "grasp.position")
    if "quaternion_wxyz" in grasp:
        grasp["quaternion_wxyz"] = _quaternion(grasp["quaternion_wxyz"], "grasp.quaternion_wxyz")
    grasp["opening"] = float(grasp.get("opening", 1.0))
    if not np.isfinite(grasp["opening"]) or not -1 <= grasp["opening"] <= 1:
        raise ValueError("object.grasp.opening must be in [-1,1]")
    obj["config_source_sha256"] = source_hashes
    obj["schema_version"] = "dynamicbench.object.v1"
    return result


def vertical_bounds(geometry, pose):
    """Exact primitive bounds; conservative oriented local bounds for USD."""
    pose = np.asarray(pose)
    r = quat_wxyz_matrix(pose[3:7])
    kind = geometry["kind"]
    center_z = float(pose[2])
    if kind == "sphere":
        extent = geometry["radius"]
    elif kind == "cylinder":
        extent = geometry["radius"] * np.linalg.norm(r[2, :2]) + geometry["height"] / 2 * abs(r[2, 2])
    else:
        center_z += float(r[2] @ np.asarray(geometry["center"]))
        extent = float(np.abs(r[2]) @ np.asarray(geometry["half_extents"]))
    return center_z - extent, center_z + extent


def spawn_position(obj, belt_top):
    low, _ = vertical_bounds(obj["geometry"], [0, 0, 0, *obj["initial_quaternion_wxyz"]])
    return np.array([*obj["initial_xy"], belt_top - low + obj["spawn_clearance"]])


def grasp_position(obj, pose):
    """Grasp offsets are meters in the scaled rigid body's local frame."""
    geometry = obj["geometry"]
    default = geometry.get("center", [0, 0, 0]).copy()
    if obj.get("shape") == "box":
        default[2] += obj["size"][2] * 0.05  # preserve the original cube grasp height
    offset = obj["grasp"].get("position", default)
    return np.asarray(pose[:3]) + quat_wxyz_matrix(pose[3:7]) @ np.asarray(offset)
