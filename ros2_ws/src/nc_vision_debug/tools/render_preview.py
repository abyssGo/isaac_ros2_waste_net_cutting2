"""Render deterministic preview PNGs without ROS 2."""

from pathlib import Path
import sys

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from nc_vision_debug.renderer import PopupRenderer  # noqa: E402


def sample_frame() -> np.ndarray:
    frame = np.full((720, 1280, 3), (58, 66, 72), dtype=np.uint8)
    cv2.rectangle(frame, (180, 130), (1100, 620), (66, 78, 70), -1)
    for x in range(180, 1101, 35):
        cv2.line(frame, (x, 130), (x, 620), (82, 96, 84), 1)
    for y in range(130, 621, 35):
        cv2.line(frame, (180, y), (1100, y), (82, 96, 84), 1)
    return frame


def base_status() -> dict:
    return {
        "phase": "ROBOT WORKING",
        "scan": "INITIAL SCAN",
        "station": "ARRIVED",
        "action_server": "ACTIVE",
        "current_task": 2,
        "total_tasks": 3,
        "model_name": "best.pt",
        "device": "CUDA:0",
        "inference_ms": 17.9,
        "fps": 32.1,
        "validation": {
            "precision": 0.91,
            "recall": 0.88,
            "map50": 0.92,
            "map50_95": 0.71,
        },
        "detections": [
            {
                "object_id": "plastic_bottle_01",
                "class_name": "plastic_bottle",
                "confidence": 0.87,
                "bbox": [350, 201, 492, 371],
                "action_included": True,
                "action_state": "COMPLETE",
                "task_index": 1,
                "depth_m": 1.240,
                "camera_xyz": [-0.612, 2.104, 0.823],
            },
            {
                "object_id": "can_01",
                "class_name": "can",
                "confidence": 0.91,
                "bbox": [520, 230, 660, 410],
                "action_included": True,
                "action_state": "REMOVING",
                "task_index": 2,
                "depth_m": 1.180,
                "camera_xyz": [-0.530, 2.080, 0.790],
            },
            {
                "object_id": "buoy_01",
                "class_name": "buoy",
                "confidence": 0.42,
                "bbox": [720, 250, 880, 430],
                "action_included": False,
                "action_state": "EXCLUDED",
                "reject_reason": "LOW CONFIDENCE",
            },
        ],
    }


def main() -> None:
    output_dir = PROJECT_ROOT / "preview"
    output_dir.mkdir(exist_ok=True)
    frame = sample_frame()
    for mode in ("vision1", "vision2"):
        status = base_status()
        if mode == "vision1":
            status["message"] = (
                "CUTTING OBJECT 2/3 - can_01 - ACTION IN PROGRESS"
            )
        else:
            status["message"] = (
                "REMOVING OBJECT 2/3 - can_01 - SUCTION ACTION IN PROGRESS"
            )
        renderer = PopupRenderer(mode, roi=[180, 130, 1100, 620])
        image = renderer.render(frame, status)
        target = output_dir / f"{mode}_popup_preview.png"
        if not cv2.imwrite(str(target), image):
            raise RuntimeError(f"failed to write {target}")
        print(target)


if __name__ == "__main__":
    main()
