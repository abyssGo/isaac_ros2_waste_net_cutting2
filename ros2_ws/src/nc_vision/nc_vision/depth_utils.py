"""Camera geometry helpers for NetClean vision nodes."""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np


def camera_intrinsics(k: Sequence[float]) -> Tuple[float, float, float, float]:
    """Return fx, fy, cx and cy from a ROS CameraInfo K matrix."""
    if len(k) != 9:
        raise ValueError("CameraInfo.k must contain 9 values")

    fx = float(k[0])
    fy = float(k[4])
    cx = float(k[2])
    cy = float(k[5])

    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("Camera focal lengths must be positive")

    return fx, fy, cx, cy


def pixel_ray_camera(
    u: float,
    v: float,
    k: Sequence[float],
) -> np.ndarray:
    """Create a non-normalized ray in a ROS optical camera frame."""
    fx, fy, cx, cy = camera_intrinsics(k)
    return np.array(
        [(float(u) - cx) / fx, (float(v) - cy) / fy, 1.0],
        dtype=np.float64,
    )


def point_from_pixel_depth(
    u: float,
    v: float,
    depth_m: float,
    k: Sequence[float],
) -> np.ndarray:
    """Back-project a pixel and metric depth into the optical frame."""
    if not np.isfinite(depth_m) or depth_m <= 0.0:
        raise ValueError("Depth must be a positive finite value")

    return pixel_ray_camera(u, v, k) * float(depth_m)


def median_depth_m(
    depth_image: np.ndarray,
    u: int,
    v: int,
    window_size: int,
    depth_scale: float,
    minimum_m: float,
    maximum_m: float,
) -> float:
    """Return the median valid depth around a pixel in metres."""
    if depth_image.ndim != 2:
        raise ValueError("Depth image must have one channel")
    if window_size < 1:
        raise ValueError("window_size must be at least 1")
    if depth_scale <= 0.0:
        raise ValueError("depth_scale must be positive")

    height, width = depth_image.shape
    u = int(np.clip(u, 0, width - 1))
    v = int(np.clip(v, 0, height - 1))
    half = window_size // 2

    x0 = max(0, u - half)
    x1 = min(width, u + half + 1)
    y0 = max(0, v - half)
    y1 = min(height, v + half + 1)

    values = np.asarray(depth_image[y0:y1, x0:x1], dtype=np.float64)
    values = values.reshape(-1) * float(depth_scale)
    valid = values[
        np.isfinite(values)
        & (values >= float(minimum_m))
        & (values <= float(maximum_m))
    ]

    if valid.size == 0:
        raise ValueError(f"No valid depth near pixel ({u}, {v})")

    return float(np.median(valid))


def bbox_center_depth_m(
    depth_image: np.ndarray,
    bbox_xyxy: Sequence[float],
    center_ratio: float,
    minimum_valid_pixels: int,
    depth_scale: float,
    minimum_m: float,
    maximum_m: float,
) -> Tuple[int, int, Tuple[int, int, int, int], float]:
    """Return bbox center, center sample bounds and median metric Depth."""
    if depth_image.ndim != 2:
        raise ValueError("Depth image must have one channel")
    if len(bbox_xyxy) != 4:
        raise ValueError("bbox_xyxy must contain four values")
    if not 0.0 < center_ratio <= 1.0:
        raise ValueError("center_ratio must be in (0, 1]")
    if minimum_valid_pixels < 1:
        raise ValueError("minimum_valid_pixels must be positive")
    if depth_scale <= 0.0:
        raise ValueError("depth_scale must be positive")

    height, width = depth_image.shape
    x1, y1, x2, y2 = bbox_xyxy
    x1 = int(np.clip(round(x1), 0, width - 1))
    y1 = int(np.clip(round(y1), 0, height - 1))
    x2 = int(np.clip(round(x2), x1 + 1, width))
    y2 = int(np.clip(round(y2), y1 + 1, height))

    center_u = int(round((x1 + x2 - 1) * 0.5))
    center_v = int(round((y1 + y2 - 1) * 0.5))
    sample_width = max(1, int(round((x2 - x1) * center_ratio)))
    sample_height = max(1, int(round((y2 - y1) * center_ratio)))

    sample_x0 = max(x1, center_u - sample_width // 2)
    sample_y0 = max(y1, center_v - sample_height // 2)
    sample_x1 = min(x2, sample_x0 + sample_width)
    sample_y1 = min(y2, sample_y0 + sample_height)

    values = np.asarray(
        depth_image[sample_y0:sample_y1, sample_x0:sample_x1],
        dtype=np.float64,
    )
    values = values.reshape(-1) * float(depth_scale)
    valid = values[
        np.isfinite(values)
        & (values >= float(minimum_m))
        & (values <= float(maximum_m))
    ]
    if valid.size < minimum_valid_pixels:
        raise ValueError(
            f"only {valid.size} valid Depth pixels; "
            f"need {minimum_valid_pixels}"
        )

    return (
        center_u,
        center_v,
        (sample_x0, sample_y0, sample_x1, sample_y1),
        float(np.median(valid)),
    )


def quaternion_xyzw_to_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """Convert an xyzw quaternion into a 3x3 rotation matrix."""
    if len(quaternion_xyzw) != 4:
        raise ValueError("Quaternion must contain x, y, z and w")

    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1.0e-9:
        raise ValueError("Camera world quaternion is not configured")

    x, y, z, w = quaternion / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def intersect_pixel_with_world_plane(
    u: float,
    v: float,
    k: Sequence[float],
    camera_position_world: Sequence[float],
    camera_quaternion_world_xyzw: Sequence[float],
    plane_point_world: Sequence[float],
    plane_normal_world: Sequence[float],
) -> np.ndarray:
    """Intersect a camera pixel ray with a plane and return camera XYZ."""
    camera_position = np.asarray(camera_position_world, dtype=np.float64)
    plane_point = np.asarray(plane_point_world, dtype=np.float64)
    plane_normal = np.asarray(plane_normal_world, dtype=np.float64)

    if camera_position.shape != (3,):
        raise ValueError("camera_position_world must contain 3 values")
    if plane_point.shape != (3,) or plane_normal.shape != (3,):
        raise ValueError("Plane point and normal must contain 3 values")

    normal_norm = float(np.linalg.norm(plane_normal))
    if normal_norm < 1.0e-9:
        raise ValueError("Plane normal must not be zero")
    plane_normal = plane_normal / normal_norm

    ray_camera = pixel_ray_camera(u, v, k)
    camera_to_world = quaternion_xyzw_to_matrix(
        camera_quaternion_world_xyzw
    )
    ray_world = camera_to_world @ ray_camera

    denominator = float(np.dot(plane_normal, ray_world))
    if abs(denominator) < 1.0e-9:
        raise ValueError("Pixel ray is parallel to the net plane")

    distance = float(
        np.dot(plane_normal, plane_point - camera_position) / denominator
    )
    if distance <= 0.0:
        raise ValueError("Net plane intersection is behind the camera")

    return ray_camera * distance
