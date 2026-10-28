"""Strict no-pickle episode reader with observation-before-command alignment."""

import json
from pathlib import Path

import numpy as np

from dynamicbench.envs.poses import validate_eef10


def validate(arrays, metadata):
    n = len(arrays["action.eef10"])
    if n < 1:
        raise ValueError("empty episode")
    for name in (
        "observation.state.eef10",
        "observation.state.qpos",
        "observation.state.qvel",
        "timestamp.sim",
        "control_tick",
        "physics_step",
        "observation.images.head",
    ):
        if len(arrays[name]) != n + 1:
            raise ValueError(f"{name}: expected N+1 observations")
    for name in ("action.joint_target", "done"):
        if len(arrays[name]) != n:
            raise ValueError(f"{name}: expected N actions")
    qpos, qvel = arrays["observation.state.qpos"], arrays["observation.state.qvel"]
    if qpos.ndim != 2 or qpos.shape != qvel.shape or arrays["action.joint_target"].shape != (n, qpos.shape[1]):
        raise ValueError("joint state/command dimension mismatch")
    for name in ("observation.state.eef10", "action.eef10"):
        if arrays[name].shape[-1] != 10:
            raise ValueError("EEF10 shape mismatch")
        for row in arrays[name]:
            validate_eef10(row)
    for name, value in arrays.items():
        if value.dtype.kind not in "biuf":
            raise ValueError(f"unsafe data type:{name}")
        if not np.isfinite(value).all():
            raise ValueError(f"nonfinite:{name}")
    for name in ("observation.images.head", "observation.images.wrist"):
        if name not in arrays:
            if name.endswith("wrist") and metadata["wrist_status"] == "not_available":
                continue
            raise ValueError("missing required RGB")
        image = arrays[name]
        if image.dtype != np.uint8 or image.ndim != 4 or image.shape[0] != n + 1 or image.shape[-1] != 3:
            raise ValueError(f"invalid RGB:{name}")
    if not np.array_equal(arrays["control_tick"], np.arange(n + 1)):
        raise ValueError("nonsequential ticks")
    if not np.array_equal(arrays["physics_step"], np.arange(n + 1) * 10):
        raise ValueError("physical step alignment mismatch")
    if not np.allclose(arrays["timestamp.sim"], np.arange(n + 1) / 30, rtol=0, atol=1e-12):
        raise ValueError("simulation timestamp mismatch")
    done = arrays["done"]
    if not (done[-1] and not done[:-1].any()):
        raise ValueError("episode terminal alignment mismatch")
    if bool(metadata["success"]) == bool(metadata.get("failure_reason")):
        raise ValueError("success/failure labels inconsistent")
    if metadata["control_hz"] != 30 or metadata["physics_hz"] != 300:
        raise ValueError("invalid episode frequencies")
    if metadata["success"]:
        hold = metadata["success_hold_ticks"]
        cubes = arrays["oracle_debug.cube_pose"][-hold:]
        contacts = arrays["oracle_debug.finger_contact_forces"][-hold:]
        if len(cubes) < hold or not np.all(
            cubes[:, 2] - metadata["cube_size"] / 2 > metadata["belt_top"] + metadata["success_height"]
        ):
            raise ValueError("success lift evidence missing")
        if not np.all(contacts > 0.01):
            raise ValueError("success bilateral contact evidence missing")
    return dict(
        actions=n,
        observations=n + 1,
        head_shape=list(arrays["observation.images.head"].shape),
        duration=n / 30,
        success=metadata["success"],
    )


def read_episode(path):
    path = Path(path)
    metadata = json.loads((path / "metadata.json").read_text())
    with np.load(path / "trajectory.npz", allow_pickle=False) as f:
        arrays = {key: f[key] for key in f.files}
    return arrays, metadata, validate(arrays, metadata)


def export_video(arrays, path, camera="head"):
    import imageio.v2 as imageio

    with imageio.get_writer(str(path), fps=30, codec="libx264", macro_block_size=1) as writer:
        for image in arrays[f"observation.images.{camera}"]:
            writer.append_data(image)
