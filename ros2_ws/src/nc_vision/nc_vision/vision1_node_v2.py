"""NetClean Station1 V2 entry point with the existing ROS API preserved."""

from __future__ import annotations

import json
import os
import time
from typing import Dict, List, Sequence, Tuple

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from ultralytics import YOLO

from nc_interfaces.action import ExecuteCut
from nc_interfaces.msg import CutTarget

from .depth_utils import (
    intersect_pixel_with_world_plane,
    median_depth_m,
    point_from_pixel_depth,
)
from .yolo_utils import Detection, best_detection_per_class, select_cut_pixels


class Vision1Node(Node):
    """Detect Station1 objects and send their P1/P2 cut targets."""

    def __init__(self) -> None:
        super().__init__("vision1_node")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.station_arrived = False
        self.cycle_finished = False
        self.processing = False
        self.goal_in_flight = False
        self.detection_attempt = 0
        self.frame_processed = False
        self.pending_targets: List[CutTarget] | None = None
        self.pending_stamp = None
        self.debug_detections: List[dict] = []
        self.debug_phase = "WAITING"
        self.debug_scan = "INITIAL SCAN"
        self.debug_station = "WAITING"
        self.debug_message = "Waiting for Station1 net"
        self.debug_current_task = 0
        self.debug_total_tasks = 0
        self.debug_inference_ms: float | None = None
        self.debug_fps: float | None = None
        self.last_popup_image_message: Image | None = None

        self.model = self._load_model()

        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            camera_qos,
        )
        self.station_subscription = self.create_subscription(
            Bool,
            self.station_arrived_topic,
            self._on_station_arrived,
            event_qos,
        )

        self.rgb_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=camera_qos,
        )
        self.depth_subscriber = message_filters.Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=camera_qos,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_subscriber, self.depth_subscriber],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop_sec,
        )
        self.synchronizer.registerCallback(self._on_synchronized_images)

        self.debug_publisher = self.create_publisher(
            Image,
            self.debug_image_topic,
            camera_qos,
        )
        self.debug_status_publisher = self.create_publisher(
            String,
            self.debug_status_topic,
            event_qos,
        )
        self.popup_image_publisher = self.create_publisher(
            Image,
            self.popup_image_topic,
            camera_qos,
        )
        self.cut_action_client = ActionClient(
            self,
            ExecuteCut,
            self.cut_action_name,
        )
        self.action_wait_timer = self.create_timer(
            0.5,
            self._try_send_pending_goal,
        )
        self.debug_status_timer = self.create_timer(
            0.5,
            self._publish_debug_status,
        )

        self.get_logger().info("NetClean Vision1 V2 started (ROS API unchanged)")
        self.get_logger().info(
            f"Camera topics: {self.rgb_topic}, {self.depth_topic}, "
            f"{self.camera_info_topic}"
        )
        self.get_logger().info(
            f"Coordinate mode: {self.coordinate_mode}; "
            f"expected classes: {self.expected_classes}"
        )
        if self.coordinate_mode == "camera_plane":
            self.get_logger().info(
                "Camera-plane mode uses calibrated net depth "
                f"{self.net_plane_depth_camera_m:.3f} m"
            )
        elif self.coordinate_mode == "depth":
            self.get_logger().warning(
                "Depth mode uses net-side P1/P2 pixels. Switch to plane mode "
                "after the Camera1 optical world pose is confirmed."
            )
        self._publish_debug_status()

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "model_path",
            "/home/rokey/netclean_project/models/netclean_yolo11n/weights/best.pt",
        )
        self.declare_parameter("device", "0")
        self.declare_parameter("model_imgsz", 640)
        self.declare_parameter("confidence", 0.40)
        self.declare_parameter("action_confidence", 0.50)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_detections", 3)
        self.declare_parameter(
            "expected_classes",
            ["plastic_bottle", "can", "buoy"],
        )
        self.declare_parameter("require_all_classes", True)

        self.declare_parameter("rgb_topic", "/camera1/rgb/image_raw")
        self.declare_parameter("depth_topic", "/camera1/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera1/camera_info")
        self.declare_parameter("camera_frame", "camera1_optical_frame")
        self.declare_parameter(
            "station_arrived_topic",
            "/station1/net_arrived",
        )
        self.declare_parameter("cut_action_name", "/cut/execute")
        self.declare_parameter(
            "debug_image_topic",
            "/vision1/debug_image",
        )
        self.declare_parameter(
            "debug_status_topic",
            "/vision1/debug/status",
        )
        self.declare_parameter(
            "popup_image_topic",
            "/vision1/debug/source_image",
        )
        self.declare_parameter("validation_precision", -1.0)
        self.declare_parameter("validation_recall", -1.0)
        self.declare_parameter("validation_map50", -1.0)
        self.declare_parameter("validation_map50_95", -1.0)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.10)

        self.declare_parameter("use_roi", False)
        self.declare_parameter("roi_x0", 0)
        self.declare_parameter("roi_y0", 0)
        self.declare_parameter("roi_width", 1280)
        self.declare_parameter("roi_height", 720)

        self.declare_parameter("cut_axis", "auto")
        self.declare_parameter("cut_margin_ratio", 0.10)
        self.declare_parameter("minimum_cut_margin_px", 8)
        self.declare_parameter("minimum_cut_distance_m", 0.05)

        self.declare_parameter("coordinate_mode", "camera_plane")
        self.declare_parameter("net_plane_depth_camera_m", 0.59)
        self.declare_parameter("depth_window", 9)
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("minimum_depth_m", 0.05)
        self.declare_parameter("maximum_depth_m", 5.0)

        self.declare_parameter(
            "net_plane_point_world",
            [-1.0, -2.0, 1.1892],
        )
        self.declare_parameter(
            "net_plane_normal_world",
            [0.0, 1.0, 0.0],
        )
        self.declare_parameter(
            "camera_position_world",
            [0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            "camera_quaternion_world_xyzw",
            [0.0, 0.0, 0.0, 0.0],
        )

    def _read_parameters(self) -> None:
        def value(name: str) -> object:
            return self.get_parameter(name).value

        self.model_path = str(value("model_path"))
        self.device = str(value("device"))
        self.model_imgsz = int(value("model_imgsz"))
        self.confidence = float(value("confidence"))
        self.action_confidence = float(value("action_confidence"))
        self.iou_threshold = float(value("iou_threshold"))
        self.max_detections = int(value("max_detections"))
        self.expected_classes = [str(item) for item in value("expected_classes")]
        self.require_all_classes = bool(value("require_all_classes"))

        self.rgb_topic = str(value("rgb_topic"))
        self.depth_topic = str(value("depth_topic"))
        self.camera_info_topic = str(value("camera_info_topic"))
        self.camera_frame = str(value("camera_frame"))
        self.station_arrived_topic = str(value("station_arrived_topic"))
        self.cut_action_name = str(value("cut_action_name"))
        self.debug_image_topic = str(value("debug_image_topic"))
        self.debug_status_topic = str(value("debug_status_topic"))
        self.popup_image_topic = str(value("popup_image_topic"))
        self.validation_precision = float(value("validation_precision"))
        self.validation_recall = float(value("validation_recall"))
        self.validation_map50 = float(value("validation_map50"))
        self.validation_map50_95 = float(value("validation_map50_95"))
        self.sync_queue_size = int(value("sync_queue_size"))
        self.sync_slop_sec = float(value("sync_slop_sec"))

        self.use_roi = bool(value("use_roi"))
        self.roi_x0 = int(value("roi_x0"))
        self.roi_y0 = int(value("roi_y0"))
        self.roi_width = int(value("roi_width"))
        self.roi_height = int(value("roi_height"))

        self.cut_axis = str(value("cut_axis"))
        self.cut_margin_ratio = float(value("cut_margin_ratio"))
        self.minimum_cut_margin_px = int(value("minimum_cut_margin_px"))
        self.minimum_cut_distance_m = float(value("minimum_cut_distance_m"))

        self.coordinate_mode = str(value("coordinate_mode"))
        self.net_plane_depth_camera_m = float(
            value("net_plane_depth_camera_m")
        )
        self.depth_window = int(value("depth_window"))
        self.depth_scale = float(value("depth_scale"))
        self.minimum_depth_m = float(value("minimum_depth_m"))
        self.maximum_depth_m = float(value("maximum_depth_m"))

        self.net_plane_point_world = list(value("net_plane_point_world"))
        self.net_plane_normal_world = list(value("net_plane_normal_world"))
        self.camera_position_world = list(value("camera_position_world"))
        self.camera_quaternion_world_xyzw = list(
            value("camera_quaternion_world_xyzw")
        )

    def _validate_parameters(self) -> None:
        if not self.model_path:
            raise ValueError("model_path is empty")
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")
        if self.coordinate_mode not in {"camera_plane", "depth", "plane"}:
            raise ValueError(
                "coordinate_mode must be camera_plane, depth or plane"
            )
        if self.net_plane_depth_camera_m <= 0.0:
            raise ValueError("net_plane_depth_camera_m must be positive")
        if self.cut_axis not in {"auto", "horizontal", "vertical"}:
            raise ValueError("cut_axis must be auto, horizontal or vertical")
        if len(set(self.expected_classes)) != len(self.expected_classes):
            raise ValueError("expected_classes contains duplicate names")
        if self.depth_window < 1:
            raise ValueError("depth_window must be positive")
        if self.coordinate_mode == "plane":
            quaternion_norm = float(
                np.linalg.norm(self.camera_quaternion_world_xyzw)
            )
            if quaternion_norm < 1.0e-9:
                raise ValueError(
                    "Plane mode requires camera_quaternion_world_xyzw"
                )

    def _load_model(self) -> YOLO:
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        model = YOLO(self.model_path)
        model_names = set(str(name) for name in model.names.values())
        missing = set(self.expected_classes) - model_names
        if missing:
            raise ValueError(
                "YOLO model is missing expected classes: "
                + ", ".join(sorted(missing))
            )
        return model

    def _on_camera_info(self, message: CameraInfo) -> None:
        if message.header.frame_id and message.header.frame_id != self.camera_frame:
            self.get_logger().warning(
                "CameraInfo frame_id is "
                f"'{message.header.frame_id}', expected '{self.camera_frame}'",
                throttle_duration_sec=5.0,
            )
        self.camera_info = message

    def _on_station_arrived(self, message: Bool) -> None:
        if message.data and not self.station_arrived:
            self.station_arrived = True
            self.cycle_finished = False
            self.processing = False
            self.goal_in_flight = False
            self.detection_attempt = 0
            self.frame_processed = False
            self.pending_targets = None
            self.pending_stamp = None
            self.debug_detections = []
            self._set_debug_state(
                phase="DETECTING",
                station="ARRIVED",
                current_task=0,
                total_tasks=0,
                message="Station1 arrived - waiting for synchronized RGB-D frame",
            )
            self.get_logger().info(
                "Station1 net arrived; Vision1 is armed for one Action goal"
            )
        elif not message.data:
            self.station_arrived = False
            self.cycle_finished = False
            self.frame_processed = False
            self.pending_targets = None
            self.pending_stamp = None
            self.debug_detections = []
            self._set_debug_state(
                phase="WAITING",
                station="WAITING",
                current_task=0,
                total_tasks=0,
                message="Waiting for Station1 net",
            )

    def _on_synchronized_images(
        self,
        rgb_message: Image,
        depth_message: Image,
    ) -> None:
        if not self.station_arrived or self.cycle_finished:
            return
        if self.processing or self.goal_in_flight or self.frame_processed:
            return
        if self.camera_info is None:
            self._set_debug_state(
                phase="DETECTING",
                message="WARNING - waiting for Camera1 CameraInfo",
            )
            self.get_logger().warning(
                "Waiting for Camera1 CameraInfo",
                throttle_duration_sec=2.0,
            )
            return
        self.processing = True
        self.detection_attempt += 1
        try:
            self._process_frame(rgb_message, depth_message)
        except (CvBridgeError, ValueError, RuntimeError) as error:
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - Vision1 frame rejected: {error}",
            )
            self.get_logger().error(f"Vision1 frame rejected: {error}")
        except Exception as error:  # noqa: BLE001
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - unexpected Vision1 failure: {error}",
            )
            self.get_logger().error(f"Unexpected Vision1 error: {error}")
        finally:
            self.processing = False

    def _process_frame(
        self,
        rgb_message: Image,
        depth_message: Image,
    ) -> None:
        rgb_image = self.bridge.imgmsg_to_cv2(
            rgb_message,
            desired_encoding="rgb8",
        )
        depth_image = self.bridge.imgmsg_to_cv2(
            depth_message,
            desired_encoding="passthrough",
        )
        depth_image = np.asarray(depth_image)

        if rgb_image.shape[:2] != depth_image.shape[:2]:
            raise ValueError(
                "RGB and Depth resolutions differ: "
                f"{rgb_image.shape[:2]} vs {depth_image.shape[:2]}"
            )

        height, width = rgb_image.shape[:2]
        roi_bounds = self._roi_bounds(width, height)
        x0, y0, x1, y1 = roi_bounds
        roi_rgb = rgb_image[y0:y1, x0:x1]
        inference_bgr = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2BGR)

        inference_started = time.perf_counter()
        results = self.model.predict(
            source=inference_bgr,
            imgsz=self.model_imgsz,
            conf=self.confidence,
            iou=self.iou_threshold,
            max_det=self.max_detections,
            device=self.device,
            verbose=False,
        )
        if not results:
            raise RuntimeError("YOLO returned no result object")
        self.debug_inference_ms = (
            time.perf_counter() - inference_started
        ) * 1000.0
        self.debug_fps = (
            1000.0 / self.debug_inference_ms
            if self.debug_inference_ms > 0.0
            else None
        )

        raw_debug_detections = self._collect_debug_detections(
            results[0],
            offset_xy=(x0, y0),
        )

        detections = best_detection_per_class(
            result=results[0],
            names=self.model.names,
            allowed_classes=self.expected_classes,
            confidence_threshold=self.action_confidence,
            offset_xy=(x0, y0),
        )

        popup_source_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        self._publish_popup_source(popup_source_image, rgb_message)
        debug_image = popup_source_image.copy()
        self._draw_roi(debug_image, roi_bounds)
        targets = self._build_targets(
            detections=detections,
            depth_image=depth_image,
            image_size=(width, height),
            roi_bounds=roi_bounds,
            debug_image=debug_image,
        )
        self._publish_debug(debug_image, rgb_message)

        missing_classes = [
            name for name in self.expected_classes if name not in detections
        ]
        if self.require_all_classes and missing_classes:
            self.debug_detections = self._prepare_debug_detections(
                raw_debug_detections,
                target_names=[],
                action_enabled=False,
                unavailable_reason="WAITING REQUIRED CLASS",
            )
            self._set_debug_state(
                phase="DETECTING",
                current_task=0,
                total_tasks=0,
                message=(
                    "WARNING - waiting for all classes; missing: "
                    + ", ".join(missing_classes)
                ),
            )
            self.get_logger().warning(
                "Waiting for all classes; missing: "
                + ", ".join(missing_classes),
                throttle_duration_sec=1.0,
            )
            return

        if self.require_all_classes and len(targets) != len(self.expected_classes):
            self.debug_detections = self._prepare_debug_detections(
                raw_debug_detections,
                target_names=[target.object_id for target in targets],
                action_enabled=False,
                unavailable_reason="INVALID CUT TARGET",
            )
            self._set_debug_state(
                phase="DETECTING",
                current_task=0,
                total_tasks=0,
                message="WARNING - at least one P1/P2 conversion failed",
            )
            self.get_logger().warning(
                "All detections exist, but at least one P1/P2 conversion failed",
                throttle_duration_sec=1.0,
            )
            return
        if not targets:
            self.debug_detections = self._prepare_debug_detections(
                raw_debug_detections,
                target_names=[],
                action_enabled=False,
                unavailable_reason="INVALID CUT TARGET",
            )
            self._set_debug_state(
                phase="DETECTING",
                current_task=0,
                total_tasks=0,
                message="WARNING - no valid Station1 cut target",
            )
            self.get_logger().warning(
                "No valid Station1 cut target",
                throttle_duration_sec=1.0,
            )
            return

        self.pending_targets = targets
        self.pending_stamp = rgb_message.header.stamp
        self.frame_processed = True
        target_names = [target.object_id for target in targets]
        self.debug_detections = self._prepare_debug_detections(
            raw_debug_detections,
            target_names=target_names,
            action_enabled=True,
        )
        self._set_debug_state(
            phase="TARGETS READY",
            current_task=0,
            total_tasks=len(targets),
            message=f"Detected {len(targets)} valid cut target(s) - Action queued",
        )
        self.get_logger().info(
            f"Cached {len(targets)} targets from the first valid frame"
        )
        self._try_send_pending_goal()

    def _roi_bounds(
        self,
        image_width: int,
        image_height: int,
    ) -> Tuple[int, int, int, int]:
        if not self.use_roi:
            return 0, 0, image_width, image_height

        x0 = int(np.clip(self.roi_x0, 0, image_width - 1))
        y0 = int(np.clip(self.roi_y0, 0, image_height - 1))
        x1 = int(np.clip(x0 + self.roi_width, x0 + 1, image_width))
        y1 = int(np.clip(y0 + self.roi_height, y0 + 1, image_height))
        return x0, y0, x1, y1

    def _build_targets(
        self,
        detections: Dict[str, Detection],
        depth_image: np.ndarray,
        image_size: Tuple[int, int],
        roi_bounds: Tuple[int, int, int, int],
        debug_image: np.ndarray,
    ) -> List[CutTarget]:
        width, height = image_size
        targets: List[CutTarget] = []

        for class_name in self.expected_classes:
            detection = detections.get(class_name)
            if detection is None:
                continue

            self._draw_detection(debug_image, detection)
            try:
                point1_pixel, point2_pixel = select_cut_pixels(
                    bbox_xyxy=detection.bbox_xyxy,
                    image_width=width,
                    image_height=height,
                    margin_ratio=self.cut_margin_ratio,
                    minimum_margin_px=self.minimum_cut_margin_px,
                    axis=self.cut_axis,
                    bounds_xyxy=(
                        roi_bounds[0],
                        roi_bounds[1],
                        roi_bounds[2] - 1,
                        roi_bounds[3] - 1,
                    ),
                )
                point1_xyz = self._pixel_to_camera_point(
                    point1_pixel,
                    depth_image,
                )
                point2_xyz = self._pixel_to_camera_point(
                    point2_pixel,
                    depth_image,
                )

                distance = float(np.linalg.norm(point2_xyz - point1_xyz))
                if distance < self.minimum_cut_distance_m:
                    raise ValueError(
                        f"P1/P2 distance {distance:.3f} m is too short"
                    )

                target = CutTarget()
                target.object_id = class_name
                target.point1 = self._to_point_message(point1_xyz)
                target.point2 = self._to_point_message(point2_xyz)
                targets.append(target)

                self._draw_cut_pixels(
                    debug_image,
                    point1_pixel,
                    point2_pixel,
                )
                self.get_logger().info(
                    f"{class_name}: P1={point1_xyz.round(4).tolist()}, "
                    f"P2={point2_xyz.round(4).tolist()}"
                )
            except ValueError as error:
                self.get_logger().warning(f"{class_name} target invalid: {error}")

        return targets

    def _pixel_to_camera_point(
        self,
        pixel: Tuple[int, int],
        depth_image: np.ndarray,
    ) -> np.ndarray:
        if self.camera_info is None:
            raise ValueError("CameraInfo is unavailable")

        u, v = pixel
        if self.coordinate_mode == "camera_plane":
            return point_from_pixel_depth(
                u,
                v,
                self.net_plane_depth_camera_m,
                self.camera_info.k,
            )

        if self.coordinate_mode == "depth":
            depth_m = median_depth_m(
                depth_image=depth_image,
                u=u,
                v=v,
                window_size=self.depth_window,
                depth_scale=self.depth_scale,
                minimum_m=self.minimum_depth_m,
                maximum_m=self.maximum_depth_m,
            )
            return point_from_pixel_depth(u, v, depth_m, self.camera_info.k)

        return intersect_pixel_with_world_plane(
            u=u,
            v=v,
            k=self.camera_info.k,
            camera_position_world=self.camera_position_world,
            camera_quaternion_world_xyzw=(
                self.camera_quaternion_world_xyzw
            ),
            plane_point_world=self.net_plane_point_world,
            plane_normal_world=self.net_plane_normal_world,
        )

    @staticmethod
    def _to_point_message(point_xyz: Sequence[float]) -> Point:
        point = Point()
        point.x = float(point_xyz[0])
        point.y = float(point_xyz[1])
        point.z = float(point_xyz[2])
        return point

    def _try_send_pending_goal(self) -> None:
        if not self.station_arrived or self.cycle_finished:
            return
        if self.goal_in_flight or not self.pending_targets:
            return
        if not self.cut_action_client.server_is_ready():
            self._set_debug_state(
                phase="TARGETS READY",
                message=f"Waiting for Action server {self.cut_action_name}",
            )
            self.get_logger().warning(
                f"Targets cached; waiting for Action server "
                f"{self.cut_action_name}",
                throttle_duration_sec=2.0,
            )
            return

        targets = self.pending_targets
        stamp = self.pending_stamp
        try:
            self._send_cut_goal(targets, stamp)
        except Exception as error:  # noqa: BLE001
            self.cycle_finished = True
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteCut goal send failed: {error}",
            )
            self.get_logger().error(f"ExecuteCut goal send failed: {error}")
            return

        self.pending_targets = None
        self.pending_stamp = None

    def _send_cut_goal(self, targets: List[CutTarget], stamp: object) -> None:
        goal = ExecuteCut.Goal()
        goal.header.stamp = stamp
        goal.header.frame_id = self.camera_frame
        goal.targets = targets

        self.goal_in_flight = True
        self._set_debug_state(
            phase="ACTION QUEUED",
            current_task=0,
            total_tasks=len(targets),
            message=f"Sending {len(targets)} target(s) to {self.cut_action_name}",
        )
        self.get_logger().info(
            f"Sending {len(targets)} targets to {self.cut_action_name}"
        )
        try:
            future = self.cut_action_client.send_goal_async(
                goal,
                feedback_callback=self._on_action_feedback,
            )
        except Exception:
            self.goal_in_flight = False
            raise
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: object) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:  # noqa: BLE001
            self.goal_in_flight = False
            self.cycle_finished = True
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteCut goal call failed: {error}",
            )
            self.get_logger().error(f"ExecuteCut goal call failed: {error}")
            return

        if not goal_handle.accepted:
            self.goal_in_flight = False
            self.cycle_finished = True
            self._mark_all_action_detections("REJECTED")
            self._set_debug_state(
                phase="ERROR",
                message="ERROR - Robot1 rejected the ExecuteCut goal",
            )
            self.get_logger().error("Robot1 rejected the ExecuteCut goal")
            return

        self._set_debug_state(
            phase="ROBOT WORKING",
            message="ExecuteCut goal accepted - waiting for feedback",
        )
        self.get_logger().info("Robot1 accepted the ExecuteCut goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def _on_action_feedback(self, feedback_message: object) -> None:
        feedback = feedback_message.feedback
        self._mark_action_progress(
            object_id=str(feedback.current_object_id),
            current=int(feedback.current_target),
            total=int(feedback.total_targets),
            active_state="CUTTING",
        )
        self._set_debug_state(
            phase="ROBOT WORKING",
            current_task=int(feedback.current_target),
            total_tasks=int(feedback.total_targets),
            message=(
                f"CUTTING OBJECT {feedback.current_target}/"
                f"{feedback.total_targets} - {feedback.current_object_id}"
            ),
        )
        self.get_logger().info(
            "Robot1 cutting "
            f"{feedback.current_object_id} "
            f"({feedback.current_target}/{feedback.total_targets})"
        )

    def _on_action_result(self, future: object) -> None:
        self.goal_in_flight = False
        self.cycle_finished = True
        try:
            result = future.result().result
        except Exception as error:  # noqa: BLE001
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteCut result failed: {error}",
            )
            self.get_logger().error(f"ExecuteCut result failed: {error}")
            return

        if result.success:
            self._mark_all_action_detections("COMPLETE")
            self._set_debug_state(
                phase="COMPLETE",
                current_task=self.debug_total_tasks,
                message=f"CUTTING COMPLETE - {result.message}",
            )
            self.get_logger().info(f"ExecuteCut succeeded: {result.message}")
        else:
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteCut failed: {result.message}",
            )
            self.get_logger().error(f"ExecuteCut failed: {result.message}")

    def _collect_debug_detections(
        self,
        result: object,
        offset_xy: Tuple[int, int],
    ) -> List[dict]:
        """Return the best visible detection per NON-NET class for the popup."""
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        xyxy_values = boxes.xyxy.detach().cpu().numpy()
        confidence_values = boxes.conf.detach().cpu().numpy()
        class_values = boxes.cls.detach().cpu().numpy()
        offset_x, offset_y = offset_xy
        best_by_class: Dict[str, dict] = {}

        for bbox, confidence, class_index in zip(
            xyxy_values,
            confidence_values,
            class_values,
        ):
            class_name = str(self.model.names[int(class_index)])
            if class_name not in self.expected_classes:
                continue
            confidence_value = float(confidence)
            previous = best_by_class.get(class_name)
            if previous is not None and previous["confidence"] >= confidence_value:
                continue
            x1, y1, x2, y2 = [float(value) for value in bbox]
            best_by_class[class_name] = {
                "object_id": class_name,
                "class_name": class_name,
                "confidence": confidence_value,
                "bbox": [
                    int(round(x1 + offset_x)),
                    int(round(y1 + offset_y)),
                    int(round(x2 + offset_x)),
                    int(round(y2 + offset_y)),
                ],
                "action_included": False,
                "action_state": "EXCLUDED",
                "task_index": 0,
                "reject_reason": "",
            }

        return [
            best_by_class[class_name]
            for class_name in self.expected_classes
            if class_name in best_by_class
        ]

    def _prepare_debug_detections(
        self,
        detections: List[dict],
        target_names: List[str],
        action_enabled: bool,
        unavailable_reason: str = "INVALID CUT TARGET",
    ) -> List[dict]:
        target_indices = {
            class_name: index
            for index, class_name in enumerate(target_names, start=1)
        }
        prepared: List[dict] = []
        for detection in detections:
            item = dict(detection)
            class_name = item["class_name"]
            if float(item["confidence"]) < self.action_confidence:
                item.update(
                    action_included=False,
                    action_state="EXCLUDED",
                    task_index=0,
                    reject_reason="LOW CONFIDENCE",
                )
            elif action_enabled and class_name in target_indices:
                item.update(
                    action_included=True,
                    action_state="QUEUED",
                    task_index=target_indices[class_name],
                    reject_reason="",
                )
            else:
                item.update(
                    action_included=False,
                    action_state="EXCLUDED",
                    task_index=0,
                    reject_reason=unavailable_reason,
                )
            prepared.append(item)
        return prepared

    def _mark_action_progress(
        self,
        object_id: str,
        current: int,
        total: int,
        active_state: str,
    ) -> None:
        for detection in self.debug_detections:
            if not detection.get("action_included", False):
                continue
            task_index = int(detection.get("task_index", 0))
            if detection.get("object_id") == object_id or task_index == current:
                detection["action_state"] = active_state
            elif task_index and task_index < current:
                detection["action_state"] = "COMPLETE"
            else:
                detection["action_state"] = "QUEUED"
        self.debug_current_task = current
        self.debug_total_tasks = total

    def _mark_all_action_detections(self, state: str) -> None:
        for detection in self.debug_detections:
            if detection.get("action_included", False):
                detection["action_state"] = state

    def _set_debug_state(
        self,
        *,
        phase: str | None = None,
        station: str | None = None,
        current_task: int | None = None,
        total_tasks: int | None = None,
        message: str | None = None,
    ) -> None:
        if phase is not None:
            self.debug_phase = phase
        if station is not None:
            self.debug_station = station
        if current_task is not None:
            self.debug_current_task = current_task
        if total_tasks is not None:
            self.debug_total_tasks = total_tasks
        if message is not None:
            self.debug_message = message
        if hasattr(self, "debug_status_publisher"):
            self._publish_debug_status()

    @staticmethod
    def _metric_or_none(value: float) -> float | None:
        return value if value >= 0.0 else None

    def _publish_debug_status(self) -> None:
        if not hasattr(self, "debug_status_publisher"):
            return
        payload = {
            "phase": self.debug_phase,
            "scan": self.debug_scan,
            "station": self.debug_station,
            "action_server": (
                "READY" if self.cut_action_client.server_is_ready() else "WAITING"
            ),
            "current_task": self.debug_current_task,
            "total_tasks": self.debug_total_tasks,
            "model_name": os.path.basename(self.model_path),
            "device": (
                "CPU" if self.device.lower() == "cpu" else f"CUDA:{self.device}"
            ),
            "inference_ms": self.debug_inference_ms,
            "fps": self.debug_fps,
            "validation": {
                "precision": self._metric_or_none(self.validation_precision),
                "recall": self._metric_or_none(self.validation_recall),
                "map50": self._metric_or_none(self.validation_map50),
                "map50_95": self._metric_or_none(self.validation_map50_95),
            },
            "detections": self.debug_detections,
            "message": self.debug_message,
        }
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.debug_status_publisher.publish(message)
        if self.last_popup_image_message is not None:
            self.popup_image_publisher.publish(self.last_popup_image_message)

    def _draw_roi(
        self,
        image: np.ndarray,
        bounds: Tuple[int, int, int, int],
    ) -> None:
        x0, y0, x1, y1 = bounds
        cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), (255, 180, 0), 2)

    @staticmethod
    def _draw_detection(image: np.ndarray, detection: Detection) -> None:
        x1, y1, x2, y2 = [int(round(value)) for value in detection.bbox_xyxy]
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 220, 0), 2)
        label = f"{detection.class_name} {detection.confidence:.2f}"
        cv2.putText(
            image,
            label,
            (x1, max(20, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 220, 0),
            2,
            cv2.LINE_AA,
        )

    @staticmethod
    def _draw_cut_pixels(
        image: np.ndarray,
        point1: Tuple[int, int],
        point2: Tuple[int, int],
    ) -> None:
        cv2.circle(image, point1, 7, (255, 255, 0), -1)
        cv2.circle(image, point2, 7, (255, 0, 255), -1)
        cv2.putText(
            image,
            "P1",
            (point1[0] + 8, point1[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            "P2",
            (point2[0] + 8, point2[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

    def _publish_debug(self, image: np.ndarray, source_message: Image) -> None:
        debug_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_message.header = source_message.header
        debug_message.header.frame_id = self.camera_frame
        self.debug_publisher.publish(debug_message)

    def _publish_popup_source(
        self,
        image: np.ndarray,
        source_message: Image,
    ) -> None:
        """Publish the exact clean frame used to compute popup detections."""
        popup_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        popup_message.header = source_message.header
        popup_message.header.frame_id = self.camera_frame
        self.last_popup_image_message = popup_message
        self.popup_image_publisher.publish(popup_message)


def main(args: Sequence[str] | None = None) -> None:
    """Run the NetClean Vision1 node."""
    rclpy.init(args=args)
    node: Vision1Node | None = None
    try:
        node = Vision1Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
