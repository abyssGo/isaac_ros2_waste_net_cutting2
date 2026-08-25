import numpy as np

from nc_vision.depth_utils import (
    bbox_center_depth_m,
    intersect_pixel_with_world_plane,
    point_from_pixel_depth,
)
from nc_vision.yolo_utils import select_cut_pixels


CAMERA_K = [100.0, 0.0, 50.0, 0.0, 100.0, 50.0, 0.0, 0.0, 1.0]


def test_center_pixel_depth_projects_on_optical_axis():
    point = point_from_pixel_depth(50, 50, 2.0, CAMERA_K)
    assert np.allclose(point, [0.0, 0.0, 2.0])


def test_camera_plane_points_keep_calibrated_net_depth():
    point1 = point_from_pixel_depth(25, 40, 0.59, CAMERA_K)
    point2 = point_from_pixel_depth(75, 60, 0.59, CAMERA_K)
    assert np.isclose(point1[2], 0.59)
    assert np.isclose(point2[2], 0.59)


def test_bbox_center_depth_ignores_box_edges():
    depth = np.full((10, 10), 3.0, dtype=np.float32)
    depth[3:6, 3:6] = 1.2
    center_u, center_v, bounds, depth_m = bbox_center_depth_m(
        depth_image=depth,
        bbox_xyxy=[2.0, 2.0, 8.0, 8.0],
        center_ratio=0.5,
        minimum_valid_pixels=5,
        depth_scale=1.0,
        minimum_m=0.05,
        maximum_m=5.0,
    )
    assert (center_u, center_v) == (4, 4)
    assert bounds == (3, 3, 6, 6)
    assert np.isclose(depth_m, 1.2)


def test_pixel_ray_intersects_world_plane():
    point = intersect_pixel_with_world_plane(
        u=100,
        v=50,
        k=CAMERA_K,
        camera_position_world=[0.0, 0.0, 0.0],
        camera_quaternion_world_xyzw=[0.0, 0.0, 0.0, 1.0],
        plane_point_world=[0.0, 0.0, 2.0],
        plane_normal_world=[0.0, 0.0, 1.0],
    )
    assert np.allclose(point, [1.0, 0.0, 2.0])


def test_auto_cut_pixels_follow_long_bbox_axis():
    point1, point2 = select_cut_pixels(
        bbox_xyxy=[40.0, 20.0, 60.0, 80.0],
        image_width=100,
        image_height=100,
        margin_ratio=0.1,
        minimum_margin_px=5,
        axis="auto",
    )
    assert point1 == (50, 14)
    assert point2 == (50, 86)
