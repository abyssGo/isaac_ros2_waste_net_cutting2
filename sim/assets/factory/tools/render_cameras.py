"""Render the two vision cameras of a world without running the process.

The net (with bottle/can/buoy) is placed at Station 1 and Station 2 and each
station camera is rendered at 640 x 640, the resolution of the ROS camera graph.
Physics is not played, so two worlds can be compared pixel by pixel.

    $ISAAC_SIM_ROOT/python.sh render_cameras.py --usd <world.usd[a]> --out <dir> [--views ...]

Writes <out>/<view>_rgb.png and <out>/<view>_depth.npy (metres, distance to image plane).
Views: station1 (camera1, net at y=-2), station2 (camera2, net at y=+2),
empty1 / empty2 (same cameras, net moved away), plus any /World/FactoryCams/* name
rendered at 1600 x 900 for presentation.
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--usd", required=True)
parser.add_argument("--out", required=True)
parser.add_argument("--views", nargs="+", default=["station1", "station2", "empty1", "empty2"])
parser.add_argument("--settle-frames", type=int, default=90)
args = parser.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "width": 1600, "height": 900})

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, UsdGeom  # noqa: E402

CAM = {"camera1": "/World/AG_Camera1/RSD455/Camera_OmniVision_OV9782_Color",
       "camera2": "/World/AG_Camera2/RSD455/Camera_OmniVision_OV9782_Color"}
NET = "/World/collected_net"
VIEWS = {"station1": ("camera1", -2.0), "station2": ("camera2", 2.0),
         "empty1": ("camera1", -8.0), "empty2": ("camera2", -8.0)}

usd = os.path.abspath(args.usd)
if not open_stage(usd):
    sys.exit(f"cannot open {usd}")
while is_stage_loading():
    app.update()
stage = omni.usd.get_context().get_stage()
translate = UsdGeom.Xformable(stage.GetPrimAtPath(NET)).GetOrderedXformOps()[0]
spawn = translate.Get()
os.makedirs(args.out, exist_ok=True)
rep.orchestrator.set_capture_on_play(False)

products = {}


def annotators_for(camera_path, resolution):
    """One render product (+ rgb/depth annotators) per camera, reused across views."""
    if camera_path not in products:
        product = rep.create.render_product(camera_path, resolution)
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        depth = rep.AnnotatorRegistry.get_annotator("distance_to_image_plane")
        rgb.attach([product])
        depth.attach([product])
        products[camera_path] = (rgb, depth)
    return products[camera_path]


for view in args.views:
    if view in VIEWS:
        camera, net_y = VIEWS[view]
        camera_path, resolution = CAM[camera], (640, 640)
    else:
        camera_path, resolution, net_y = f"/World/FactoryCams/{view}", (1600, 900), -2.0
    translate.Set(type(spawn)(spawn[0], net_y, spawn[2]))
    rgb, depth = annotators_for(camera_path, resolution)
    for _ in range(args.settle_frames):
        app.update()
    rep.orchestrator.step(rt_subframes=8)
    image = np.asarray(rgb.get_data())[..., :3]
    Image.fromarray(image).save(os.path.join(args.out, f"{view}_rgb.png"))
    np.save(os.path.join(args.out, f"{view}_depth.npy"), np.asarray(depth.get_data(), dtype=np.float32))
    print(f"[render] {view}: {camera_path} -> {args.out}", flush=True)

translate.Set(spawn)
app.close()
