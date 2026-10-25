"""Detect a previous episode's RGB at the unoccluded conveyor reset pose.

This scene-specific diagnostic compares the orange cube's image centroid with
its geometric projection. It detects cross-reset stale frames; its pixel
tolerance cannot establish sub-tick freshness during steady motion.
"""

import numpy as np

from dynamicbench.envs.poses import quat_wxyz_matrix


def audit_reset_image(image, cube_position, camera, tolerance_px=10.0):
    rotation = quat_wxyz_matrix(camera["head_quaternion_wxyz"])
    point = rotation.T @ (np.asarray(cube_position) - camera["head_position"])
    height, width = image.shape[:2]
    # USD's horizontal aperture and image aspect determine the square-pixel K.
    focal_px = width * camera["head_lens"]["focal_length"] / camera["head_lens"]["horizontal_aperture"]
    if point[2] >= 0:
        raise ValueError("reset cube is behind the head camera")
    projected = np.array([width / 2 + focal_px * point[0] / -point[2], height / 2 - focal_px * point[1] / -point[2]])
    rgb = image.astype(float)
    mask = (
        (rgb[:, :, 0] > 120)
        & (rgb[:, :, 1] > 90)
        & (rgb[:, :, 2] < 100)
        & (rgb[:, :, 0] > 1.5 * rgb[:, :, 2])
        & (rgb[:, :, 1] > 1.5 * rgb[:, :, 2])
    )
    y, x = np.nonzero(mask)
    centroid = np.array([x.mean(), y.mean()]) if len(x) >= 20 else None
    error = float(np.linalg.norm(centroid - projected)) if centroid is not None else None
    return dict(
        passed=error is not None and error <= tolerance_px,
        projected_cube_center_px=projected.tolist(),
        orange_centroid_px=centroid.tolist() if centroid is not None else None,
        orange_pixels=len(x),
        center_error_px=error,
        tolerance_px=tolerance_px,
        scope="unoccluded reset pose; detects cross-reset stale RGB, not sub-tick lag",
    )
