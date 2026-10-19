"""Compare what the vision pipeline gets from two sets of render_cameras.py outputs.

For each view: pixel difference, YOLO detections (team best.pt) and, for the
Station 2 view, the Vision 2 depth -> 3D target with the vision2_v2.yaml values.

    python3 compare_vision.py <reference_dir> <candidate_dir> [<candidate_dir> ...]

Needs ultralytics; run from the set root or anywhere (paths are resolved from this file).
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "ros2_ws/src/nc_vision"))
from nc_vision.depth_target_utils_v2 import select_depth_guided_target  # noqa: E402
from nc_vision.depth_utils import bbox_center_depth_m, point_from_pixel_depth  # noqa: E402
from ultralytics import YOLO  # noqa: E402

MODEL = ROOT / "models/netclean_yolo11n/weights/best.pt"
K = [295.69, 0, 320, 0, 295.69, 320, 0, 0, 1]   # CameraInfo of the 640x640 OV9782 color camera
VIEWS = ["station1", "station2", "empty1", "empty2"]


def detections(model, path):
    result = model.predict(str(path), conf=0.10, imgsz=640, verbose=False)[0]
    out = []
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        out.append((model.names[int(box.cls)], float(box.conf), (x1, y1, x2, y2)))
    return sorted(out)


def vision2_targets(model, folder):
    depth = np.load(folder / "station2_depth.npy")
    depth = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
    targets = {}
    for name, conf, box in detections(model, folder / "station2_rgb.png"):
        if conf < 0.40:
            continue
        if name == "plastic_bottle":
            s = select_depth_guided_target(
                depth_image=depth, bbox_xyxy=box, margin_ratio=0.20, minimum_valid_pixels=20,
                minimum_valid_ratio=0.25, depth_scale=1.0, minimum_m=0.05, maximum_m=5.0,
                tolerance_ratio=0.05, minimum_tolerance_m=0.03, patch_radius=3)
            u, v, d = s.pixel[0], s.pixel[1], s.depth_m
        else:
            u, v, _, d = bbox_center_depth_m(
                depth_image=depth, bbox_xyxy=box, center_ratio=0.25, minimum_valid_pixels=20,
                depth_scale=1.0, minimum_m=0.05, maximum_m=5.0)
        targets[name] = np.asarray(point_from_pixel_depth(u, v, d, K))
    return targets


def main():
    reference, candidates = Path(sys.argv[1]), [Path(p) for p in sys.argv[2:]]
    model = YOLO(str(MODEL))
    ref_targets = vision2_targets(model, reference)
    for folder in [reference] + candidates:
        print(f"== {folder.name}")
        for view in VIEWS:
            path = folder / f"{view}_rgb.png"
            if not path.exists():
                continue
            a = np.asarray(Image.open(reference / f"{view}_rgb.png"))[..., :3].astype(int)
            b = np.asarray(Image.open(path))[..., :3].astype(int)
            diff = np.abs(a - b).max(axis=2)
            dets = ", ".join(f"{n} {c:.3f} @({(x1 + x2) / 2:.0f},{(y1 + y2) / 2:.0f})" for n, c, (x1, y1, x2, y2) in detections(model, path))
            print(f"  {view:9s} pixel diff mean {diff.mean():5.2f} max {diff.max():3d} | YOLO: {dets or 'none'}")
        for name, p in sorted(vision2_targets(model, folder).items()):
            delta = np.linalg.norm(p - ref_targets[name]) * 1000 if name in ref_targets else float("nan")
            print(f"  vision2 target {name:15s} cam=({p[0]:+.4f},{p[1]:+.4f},{p[2]:.4f}) m  |Δ| vs reference {delta:.1f} mm")


if __name__ == "__main__":
    main()
