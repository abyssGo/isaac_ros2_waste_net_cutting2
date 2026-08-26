"""Depth-guided action target selection used by Vision2 V3."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class DepthTargetSelection:
    """A robust target pixel and depth selected inside one detection box."""

    pixel: Tuple[int, int]
    sample_bounds: Tuple[int, int, int, int]
    depth_m: float
    median_depth_m: float
    valid_ratio: float


def select_depth_guided_target(
    depth_image: np.ndarray,
    bbox_xyxy: Sequence[float],
    *,
    margin_ratio: float,
    minimum_valid_pixels: int,
    minimum_valid_ratio: float,
    depth_scale: float,
    minimum_m: float,
    maximum_m: float,
    tolerance_ratio: float,
    minimum_tolerance_m: float,
    patch_radius: int,
) -> DepthTargetSelection:
    """Select the central pixel belonging to the median-depth surface.

    The box edge is removed first.  The remaining valid depths define a median
    surface; pixels close to that surface are candidates, and the spatially
    most central candidate becomes the robot target.  A local patch median is
    used for the final deprojection depth.
    """

    if depth_image.ndim != 2:
        raise ValueError("Depth image must be single-channel")
    if len(bbox_xyxy) != 4:
        raise ValueError("bbox_xyxy must contain four values")

    image_h, image_w = depth_image.shape
    x1, y1, x2, y2 = [int(round(float(value))) for value in bbox_xyxy]
    x1 = int(np.clip(x1, 0, image_w - 1))
    y1 = int(np.clip(y1, 0, image_h - 1))
    x2 = int(np.clip(x2, x1 + 1, image_w))
    y2 = int(np.clip(y2, y1 + 1, image_h))

    margin_x = int(round((x2 - x1) * margin_ratio))
    margin_y = int(round((y2 - y1) * margin_ratio))
    inner_x1 = x1 + margin_x
    inner_y1 = y1 + margin_y
    inner_x2 = x2 - margin_x
    inner_y2 = y2 - margin_y
    if inner_x2 <= inner_x1 or inner_y2 <= inner_y1:
        raise ValueError("Depth target ROI disappeared after margin")

    inner_depth = np.asarray(
        depth_image[inner_y1:inner_y2, inner_x1:inner_x2],
        dtype=np.float64,
    ) * depth_scale
    valid_mask = (
        np.isfinite(inner_depth)
        & (inner_depth >= minimum_m)
        & (inner_depth <= maximum_m)
    )
    valid_count = int(np.count_nonzero(valid_mask))
    valid_ratio = valid_count / float(inner_depth.size)
    if valid_count < minimum_valid_pixels:
        raise ValueError(
            f"only {valid_count} valid Depth pixels; need {minimum_valid_pixels}"
        )
    if valid_ratio < minimum_valid_ratio:
        raise ValueError(
            f"valid Depth ratio {valid_ratio:.1%} < {minimum_valid_ratio:.1%}"
        )

    median_depth_m = float(np.median(inner_depth[valid_mask]))
    tolerance_m = max(
        minimum_tolerance_m,
        median_depth_m * tolerance_ratio,
    )
    surface_mask = valid_mask & (
        np.abs(inner_depth - median_depth_m) <= tolerance_m
    )
    candidate_v, candidate_u = np.nonzero(surface_mask)
    if candidate_u.size == 0:
        raise ValueError("no median-surface Depth candidate")

    center_u = (inner_depth.shape[1] - 1) / 2.0
    center_v = (inner_depth.shape[0] - 1) / 2.0
    squared_distance = (
        (candidate_u.astype(np.float64) - center_u) ** 2
        + (candidate_v.astype(np.float64) - center_v) ** 2
    )
    best_index = int(np.argmin(squared_distance))
    local_u = int(candidate_u[best_index])
    local_v = int(candidate_v[best_index])
    target_u = inner_x1 + local_u
    target_v = inner_y1 + local_v

    patch_x1 = max(inner_x1, target_u - patch_radius)
    patch_y1 = max(inner_y1, target_v - patch_radius)
    patch_x2 = min(inner_x2, target_u + patch_radius + 1)
    patch_y2 = min(inner_y2, target_v + patch_radius + 1)
    patch = np.asarray(
        depth_image[patch_y1:patch_y2, patch_x1:patch_x2],
        dtype=np.float64,
    ) * depth_scale
    patch_valid = (
        np.isfinite(patch)
        & (patch >= minimum_m)
        & (patch <= maximum_m)
        & (np.abs(patch - median_depth_m) <= tolerance_m)
    )
    if not np.any(patch_valid):
        raise ValueError("selected target has no valid local Depth patch")
    final_depth_m = float(np.median(patch[patch_valid]))

    return DepthTargetSelection(
        pixel=(target_u, target_v),
        sample_bounds=(inner_x1, inner_y1, inner_x2, inner_y2),
        depth_m=final_depth_m,
        median_depth_m=median_depth_m,
        valid_ratio=valid_ratio,
    )
