"""YOLO result and cut-pixel helpers for NetClean Vision1."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, Mapping, Sequence, Tuple

import numpy as np


@dataclass(frozen=True)
class Detection:
    """One filtered detection in full-image pixel coordinates."""

    class_name: str
    confidence: float
    bbox_xyxy: Tuple[float, float, float, float]


def _class_name(names: Mapping[int, str] | Sequence[str], class_id: int) -> str:
    if isinstance(names, Mapping):
        return str(names[class_id])
    return str(names[class_id])


def _to_numpy(value: object) -> np.ndarray:
    """Convert an Ultralytics torch/numpy result field to numpy."""
    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        return value.numpy()
    return np.asarray(value)


def best_detection_per_class(
    result: object,
    names: Mapping[int, str] | Sequence[str],
    allowed_classes: Iterable[str],
    confidence_threshold: float,
    offset_xy: Tuple[int, int] = (0, 0),
) -> Dict[str, Detection]:
    """Keep only the highest-confidence detection for each allowed class."""
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return {}

    allowed = set(allowed_classes)
    xyxy = _to_numpy(boxes.xyxy)
    confidences = _to_numpy(boxes.conf)
    class_ids = _to_numpy(boxes.cls).astype(np.int64)
    offset_x, offset_y = offset_xy

    selected: Dict[str, Detection] = {}
    for coordinates, confidence, class_id in zip(
        xyxy,
        confidences,
        class_ids,
    ):
        name = _class_name(names, int(class_id))
        score = float(confidence)
        if name not in allowed or score < confidence_threshold:
            continue

        x1, y1, x2, y2 = [float(value) for value in coordinates]
        detection = Detection(
            class_name=name,
            confidence=score,
            bbox_xyxy=(
                x1 + offset_x,
                y1 + offset_y,
                x2 + offset_x,
                y2 + offset_y,
            ),
        )

        previous = selected.get(name)
        if previous is None or detection.confidence > previous.confidence:
            selected[name] = detection

    return selected


def select_cut_pixels(
    bbox_xyxy: Sequence[float],
    image_width: int,
    image_height: int,
    margin_ratio: float,
    minimum_margin_px: int,
    axis: str = "auto",
    bounds_xyxy: Sequence[int] | None = None,
) -> Tuple[Tuple[int, int], Tuple[int, int]]:
    """Choose two pixels just outside the object's bounding box."""
    if len(bbox_xyxy) != 4:
        raise ValueError("bbox_xyxy must contain four values")
    if image_width < 2 or image_height < 2:
        raise ValueError("Image is too small")
    if axis not in {"auto", "horizontal", "vertical"}:
        raise ValueError("cut_axis must be auto, horizontal or vertical")

    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    box_width = max(1.0, x2 - x1)
    box_height = max(1.0, y2 - y1)
    selected_axis = axis
    if selected_axis == "auto":
        selected_axis = "horizontal" if box_width >= box_height else "vertical"

    if bounds_xyxy is None:
        bound_x0, bound_y0 = 0, 0
        bound_x1, bound_y1 = image_width - 1, image_height - 1
    else:
        if len(bounds_xyxy) != 4:
            raise ValueError("bounds_xyxy must contain four values")
        bound_x0, bound_y0, bound_x1, bound_y1 = [
            int(value) for value in bounds_xyxy
        ]
        bound_x0 = int(np.clip(bound_x0, 0, image_width - 1))
        bound_y0 = int(np.clip(bound_y0, 0, image_height - 1))
        bound_x1 = int(np.clip(bound_x1, bound_x0, image_width - 1))
        bound_y1 = int(np.clip(bound_y1, bound_y0, image_height - 1))

    if selected_axis == "horizontal":
        margin = max(minimum_margin_px, int(round(box_width * margin_ratio)))
        center_y = int(round((y1 + y2) * 0.5))
        point1 = (int(round(x1)) - margin, center_y)
        point2 = (int(round(x2)) + margin, center_y)
    else:
        margin = max(minimum_margin_px, int(round(box_height * margin_ratio)))
        center_x = int(round((x1 + x2) * 0.5))
        point1 = (center_x, int(round(y1)) - margin)
        point2 = (center_x, int(round(y2)) + margin)

    def clamp(point: Tuple[int, int]) -> Tuple[int, int]:
        return (
            int(np.clip(point[0], bound_x0, bound_x1)),
            int(np.clip(point[1], bound_y0, bound_y1)),
        )

    point1 = clamp(point1)
    point2 = clamp(point2)
    if point1 == point2:
        raise ValueError("Cut pixels collapsed to the same point")

    return point1, point2
