"""NetClean Station2 V2 with class-specific Depth-guided grasp targets.

v2 변경 사항 (긴급 수정 - 시간 압박으로 실측 튜닝 없이 가장 안전한 기본값으로 적용)
------------------------------------------------------------------
grasp_surface_offset_m 버그 수정: 카메라 optical +Z(깊이 증가) 방향이 이
카메라/로봇 배치에서 world -X(=net 쪽)와 일치함을 실제 좌표 계산으로 확인.
기존 기본값 0.08을 '+='로 적용하고 있어서, 측정된 물체 표면 지점에서 8cm를
net 쪽으로 밀어넣고 있었다 (물체를 지나쳐 net까지 이동 -> 흡착 불가 원인).
- 기본값 0.08 -> 0.0 (측정된 표면 지점을 그대로 신뢰)
- 부호도 '+=' -> '-='로 반전 (향후 미세 보정 시 양수값 = 카메라/로봇 쪽 이동)
launch/파라미터 YAML에서 이 값을 override하고 있다면 거기서도 반드시
0.0으로 맞추거나 제거해야 이 수정이 실제로 적용된다.
"""

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

from nc_interfaces.action import ExecuteRemove
from nc_interfaces.msg import RemoveTarget

from .depth_utils import bbox_center_depth_m, point_from_pixel_depth
from .depth_target_utils_v2 import select_depth_guided_target
from .yolo_utils import Detection, best_detection_per_class


class Vision2Node(Node):
    """Detect Station2 objects and send camera-frame grasp positions."""

    def __init__(self) -> None:
        super().__init__("vision2_node")

        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.station_arrived = False
        self.cycle_finished = False
        self.processing = False
        self.goal_in_flight = False
        self.frame_locked = False
        self.removal_action_succeeded = False
        self.empty_frame_count = 0
        self.pending_targets: List[RemoveTarget] | None = None
        self.pending_stamp = None
        self.last_goal_stamp = None
        self.minimum_frame_stamp_ns = -1
        self.next_detection_not_before_ns = 0
        self.debug_detections: List[dict] = []
        self.debug_phase = "WAITING"
        self.debug_station = "WAITING"
        self.debug_message = "Waiting for Station2 net"
        self.debug_current_task = 0
        self.debug_total_tasks = 0
        self.debug_inference_ms: float | None = None
        self.debug_fps: float | None = None
        self.debug_recheck_count = 0
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
        self.completion_publisher = self.create_publisher(
            Bool,
            self.completion_topic,
            event_qos,
        )
        self.remove_action_client = ActionClient(
            self,
            ExecuteRemove,
            self.remove_action_name,
        )
        self.action_wait_timer = self.create_timer(
            0.5,
            self._try_send_pending_goal,
        )
        self.debug_status_timer = self.create_timer(
            0.5,
            self._publish_debug_status,
        )

        self.get_logger().info("NetClean Vision2 V2 started (ROS API unchanged)")
        self.get_logger().info(
            f"Camera topics: {self.rgb_topic}, {self.depth_topic}, "
            f"{self.camera_info_topic}"
        )
        self.get_logger().info(
            f"Depth-guided target class='{self.depth_guided_class}'; "
            "other classes keep bbox-center targets"
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

        self.declare_parameter("rgb_topic", "/camera2/rgb/image_raw")
        self.declare_parameter("depth_topic", "/camera2/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera2/camera_info")
        self.declare_parameter("camera_frame", "camera2_optical_frame")
        self.declare_parameter(
            "station_arrived_topic",
            "/station2/net_arrived",
        )
        self.declare_parameter("remove_action_name", "/remove/execute")
        self.declare_parameter(
            "debug_image_topic",
            "/vision2/debug_image",
        )
        self.declare_parameter(
            "debug_status_topic",
            "/vision2/debug/status",
        )
        self.declare_parameter(
            "popup_image_topic",
            "/vision2/debug/source_image",
        )
        self.declare_parameter("validation_precision", -1.0)
        self.declare_parameter("validation_recall", -1.0)
        self.declare_parameter("validation_map50", -1.0)
        self.declare_parameter("validation_map50_95", -1.0)
        self.declare_parameter(
            "completion_topic",
            "/station2/complete",
        )
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.10)

        self.declare_parameter("use_roi", False)
        self.declare_parameter("roi_x0", 0)
        self.declare_parameter("roi_y0", 0)
        self.declare_parameter("roi_width", 1280)
        self.declare_parameter("roi_height", 720)

        self.declare_parameter("suction_center_ratio", 0.25)
        # (v2) 버그 수정: 기존 0.08은 카메라 optical +Z(깊이 증가) 방향으로
        # 더하고 있었는데, 이 방향은 world 좌표계에서 net 쪽(-X)이었다.
        # 즉 측정된 물체 표면 지점에서 8cm를 net 쪽으로 더 밀어넣고 있었고,
        # 이게 "흡착 안 되고 그물로 가버리는" 증상의 원인이었다.
        # 깊이로 측정된 지점이 이미 흡착 대상 표면이므로 추가 보정 없이
        # 0.0으로 시작한다. 실측 후 필요하면 아주 작은 값(수 mm)만 보정한다.
        # 기존 (주석 처리, 통합 테스트 후 문제 없으면 삭제):
        # self.declare_parameter("grasp_surface_offset_m", 0.08)
        self.declare_parameter("grasp_surface_offset_m", 0.0)
        self.declare_parameter("minimum_valid_depth_pixels", 20)
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("minimum_depth_m", 0.05)
        self.declare_parameter("maximum_depth_m", 5.0)
        self.declare_parameter("depth_guided_class", "plastic_bottle")
        self.declare_parameter("depth_guided_margin_ratio", 0.20)
        self.declare_parameter("depth_guided_minimum_valid_ratio", 0.25)
        self.declare_parameter("depth_guided_tolerance_ratio", 0.05)
        self.declare_parameter("depth_guided_minimum_tolerance_m", 0.03)
        self.declare_parameter("depth_guided_patch_radius", 3)
        self.declare_parameter("reinspect_delay_sec", 0.50)
        self.declare_parameter("detection_retry_period_sec", 0.20)
        self.declare_parameter("empty_confirmation_frames", 5)

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

        self.rgb_topic = str(value("rgb_topic"))
        self.depth_topic = str(value("depth_topic"))
        self.camera_info_topic = str(value("camera_info_topic"))
        self.camera_frame = str(value("camera_frame"))
        self.station_arrived_topic = str(value("station_arrived_topic"))
        self.remove_action_name = str(value("remove_action_name"))
        self.debug_image_topic = str(value("debug_image_topic"))
        self.debug_status_topic = str(value("debug_status_topic"))
        self.popup_image_topic = str(value("popup_image_topic"))
        self.validation_precision = float(value("validation_precision"))
        self.validation_recall = float(value("validation_recall"))
        self.validation_map50 = float(value("validation_map50"))
        self.validation_map50_95 = float(value("validation_map50_95"))
        self.completion_topic = str(value("completion_topic"))
        self.sync_queue_size = int(value("sync_queue_size"))
        self.sync_slop_sec = float(value("sync_slop_sec"))

        self.use_roi = bool(value("use_roi"))
        self.roi_x0 = int(value("roi_x0"))
        self.roi_y0 = int(value("roi_y0"))
        self.roi_width = int(value("roi_width"))
        self.roi_height = int(value("roi_height"))

        self.suction_center_ratio = float(value("suction_center_ratio"))
        self.grasp_surface_offset_m = float(
            value("grasp_surface_offset_m")
        )
        self.minimum_valid_depth_pixels = int(
            value("minimum_valid_depth_pixels")
        )
        self.depth_scale = float(value("depth_scale"))
        self.minimum_depth_m = float(value("minimum_depth_m"))
        self.maximum_depth_m = float(value("maximum_depth_m"))
        self.depth_guided_class = str(value("depth_guided_class"))
        self.depth_guided_margin_ratio = float(
            value("depth_guided_margin_ratio")
        )
        self.depth_guided_minimum_valid_ratio = float(
            value("depth_guided_minimum_valid_ratio")
        )
        self.depth_guided_tolerance_ratio = float(
            value("depth_guided_tolerance_ratio")
        )
        self.depth_guided_minimum_tolerance_m = float(
            value("depth_guided_minimum_tolerance_m")
        )
        self.depth_guided_patch_radius = int(
            value("depth_guided_patch_radius")
        )
        self.reinspect_delay_sec = float(value("reinspect_delay_sec"))
        self.detection_retry_period_sec = float(
            value("detection_retry_period_sec")
        )
        self.empty_confirmation_frames = int(
            value("empty_confirmation_frames")
        )

    def _validate_parameters(self) -> None:
        if not self.model_path:
            raise ValueError("model_path is empty")
        if not os.path.isfile(self.model_path):
            raise FileNotFoundError(f"YOLO model not found: {self.model_path}")
        if len(set(self.expected_classes)) != len(self.expected_classes):
            raise ValueError("expected_classes contains duplicate names")
        if not 0.0 < self.suction_center_ratio <= 1.0:
            raise ValueError("suction_center_ratio must be in (0, 1]")
        if self.minimum_valid_depth_pixels < 1:
            raise ValueError("minimum_valid_depth_pixels must be positive")
        if self.depth_scale <= 0.0:
            raise ValueError("depth_scale must be positive")
        if self.minimum_depth_m >= self.maximum_depth_m:
            raise ValueError("minimum_depth_m must be less than maximum_depth_m")
        if self.depth_guided_class not in self.expected_classes:
            raise ValueError("depth_guided_class must be in expected_classes")
        if not 0.0 <= self.depth_guided_margin_ratio < 0.5:
            raise ValueError("depth_guided_margin_ratio must be in [0, 0.5)")
        if not 0.0 < self.depth_guided_minimum_valid_ratio <= 1.0:
            raise ValueError(
                "depth_guided_minimum_valid_ratio must be in (0, 1]"
            )
        if self.depth_guided_tolerance_ratio <= 0.0:
            raise ValueError("depth_guided_tolerance_ratio must be positive")
        if self.depth_guided_minimum_tolerance_m <= 0.0:
            raise ValueError(
                "depth_guided_minimum_tolerance_m must be positive"
            )
        if self.depth_guided_patch_radius < 1:
            raise ValueError("depth_guided_patch_radius must be positive")
        if self.reinspect_delay_sec < 0.0:
            raise ValueError("reinspect_delay_sec must not be negative")
        if self.detection_retry_period_sec <= 0.0:
            raise ValueError("detection_retry_period_sec must be positive")
        if self.empty_confirmation_frames < 1:
            raise ValueError("empty_confirmation_frames must be positive")

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
            self.frame_locked = False
            self.removal_action_succeeded = False
            self.empty_frame_count = 0
            self.pending_targets = None
            self.pending_stamp = None
            self.last_goal_stamp = None
            self.minimum_frame_stamp_ns = -1
            self.next_detection_not_before_ns = 0
            self.debug_detections = []
            self.debug_recheck_count = 0
            self._set_debug_state(
                phase="DETECTING",
                station="ARRIVED",
                current_task=0,
                total_tasks=0,
                message="Station2 arrived - waiting for synchronized RGB-D frame",
            )
            self.get_logger().info(
                "Station2 net arrived; Vision2 is armed"
            )
        elif not message.data:
            self.station_arrived = False
            self.cycle_finished = False
            self.frame_locked = False
            self.empty_frame_count = 0
            self.pending_targets = None
            self.pending_stamp = None
            self.debug_detections = []
            self.debug_recheck_count = 0
            self._set_debug_state(
                phase="WAITING",
                station="WAITING",
                current_task=0,
                total_tasks=0,
                message="Waiting for Station2 net",
            )

    def _on_synchronized_images(
        self,
        rgb_message: Image,
        depth_message: Image,
    ) -> None:
        if not self.station_arrived or self.cycle_finished:
            return
        if self.processing or self.goal_in_flight or self.frame_locked:
            return
        if self.camera_info is None:
            self._set_debug_state(
                phase="DETECTING",
                message="WARNING - waiting for Camera2 CameraInfo",
            )
            self.get_logger().warning(
                "Waiting for Camera2 CameraInfo",
                throttle_duration_sec=2.0,
            )
            return

        frame_stamp_ns = self._stamp_to_ns(rgb_message.header.stamp)
        if frame_stamp_ns <= self.minimum_frame_stamp_ns:
            return
        if self.get_clock().now().nanoseconds < self.next_detection_not_before_ns:
            return

        self.processing = True
        try:
            self._process_frame(rgb_message, depth_message)
        except (CvBridgeError, ValueError, RuntimeError) as error:
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - Vision2 frame rejected: {error}",
            )
            self.get_logger().error(f"Vision2 frame rejected: {error}")
            self._schedule_detection_retry()
        except Exception as error:  # noqa: BLE001
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - unexpected Vision2 failure: {error}",
            )
            self.get_logger().error(f"Unexpected Vision2 error: {error}")
            self._schedule_detection_retry()
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
        targets, target_details = self._build_targets(
            detections=detections,
            depth_image=depth_image,
            debug_image=debug_image,
        )
        self._publish_debug(debug_image, rgb_message)

        if not detections:
            self.debug_detections = self._prepare_debug_detections(
                raw_debug_detections,
                target_details={},
            )
            self._handle_empty_detection()
            return

        self.empty_frame_count = 0
        if not targets:
            self.debug_detections = self._prepare_debug_detections(
                raw_debug_detections,
                target_details=target_details,
            )
            self._set_debug_state(
                phase="DETECTING" if not self.removal_action_succeeded else "RECHECK",
                current_task=0,
                total_tasks=0,
                message="WARNING - objects detected but center Depth is invalid",
            )
            self.get_logger().warning(
                "Objects detected, but no valid center Depth was available",
                throttle_duration_sec=1.0,
            )
            self._schedule_detection_retry()
            return

        self.pending_targets = targets
        self.pending_stamp = rgb_message.header.stamp
        self.frame_locked = True
        self.debug_detections = self._prepare_debug_detections(
            raw_debug_detections,
            target_details=target_details,
        )
        self._set_debug_state(
            phase="TARGETS READY" if not self.debug_recheck_count else "RECHECK",
            current_task=0,
            total_tasks=len(targets),
            message=f"Detected {len(targets)} valid removal target(s) - Action queued",
        )
        self.get_logger().info(
            f"Cached {len(targets)} Station2 remove targets"
        )
        self._try_send_pending_goal()

    def _handle_empty_detection(self) -> None:
        if not self.removal_action_succeeded:
            self._set_debug_state(
                phase="DETECTING",
                current_task=0,
                total_tasks=0,
                message="WARNING - no valid Station2 Action target detected yet",
            )
            self.get_logger().warning(
                "No Station2 objects detected yet",
                throttle_duration_sec=1.0,
            )
            self._schedule_detection_retry()
            return

        self.empty_frame_count += 1
        self._set_debug_state(
            phase="RECHECK",
            current_task=0,
            total_tasks=0,
            message=(
                "RECHECK - remaining valid NON-NET targets: 0 - confirmation "
                f"{self.empty_frame_count}/{self.empty_confirmation_frames}"
            ),
        )
        self.get_logger().info(
            "Station2 empty confirmation "
            f"{self.empty_frame_count}/{self.empty_confirmation_frames}"
        )
        if self.empty_frame_count < self.empty_confirmation_frames:
            self._schedule_detection_retry()
            return

        self.cycle_finished = True
        self._publish_completion(True)
        self._set_debug_state(
            phase="COMPLETE",
            current_task=0,
            total_tasks=0,
            message="RECHECK COMPLETE - valid NON-NET targets: 0 - Station2 complete",
        )
        self.get_logger().info(
            "Station2 removal complete; no objects remain"
        )

    def _build_targets(
        self,
        detections: Dict[str, Detection],
        depth_image: np.ndarray,
        debug_image: np.ndarray,
    ) -> Tuple[List[RemoveTarget], Dict[str, dict]]:
        targets: List[RemoveTarget] = []
        target_details: Dict[str, dict] = {}
        for class_name in self.expected_classes:
            detection = detections.get(class_name)
            if detection is None:
                continue

            self._draw_detection(debug_image, detection)
            try:
                if class_name == self.depth_guided_class:
                    (
                        position,
                        center,
                        sample_bounds,
                        depth_m,
                        valid_depth_ratio,
                    ) = self._depth_guided_position_from_depth(
                        detection,
                        depth_image,
                    )
                    target_mode = "DEPTH_MEDIAN_CENTER"
                else:
                    position, center, sample_bounds, depth_m = (
                        self._center_position_from_depth(detection, depth_image)
                    )
                    valid_depth_ratio = None
                    target_mode = "BBOX_CENTER"
                # (v2) 버그 수정: 부호 반전.
                # camera_frame(optical)의 +Z는 카메라에서 멀어지는 방향인데,
                # 이 카메라/로봇 배치에서는 그 방향이 net 쪽(-X, world)이었다.
                # 즉 기존 '+='는 항상 net 쪽으로 미는 방향이라 물체를 지나쳐
                # net까지 가버리는 원인이었다. 이제 '-='로 바꿔서, 양수값을 넣으면
                # 카메라(=로봇) 쪽으로 당겨오는 의미가 되도록 정정한다.
                # 기본값은 0.0이라 지금은 사실상 no-op이며, 실측 후 미세 보정이
                # 필요해지면 이 부호 기준으로 작은 양수값(수 mm)을 넣으면 된다.
                # 기존 (주석 처리, 통합 테스트 후 문제 없으면 삭제):
                # grasp_position[2] += self.grasp_surface_offset_m
                grasp_position = np.asarray(position, dtype=float).copy()
                grasp_position[2] -= self.grasp_surface_offset_m

                target = RemoveTarget()
                target.object_id = class_name
                target.class_name = class_name
                target.position = self._to_point_message(grasp_position)
                target.fixed_joint_path = ""
                targets.append(target)
                bbox_center = (
                    int(round((detection.bbox_xyxy[0] + detection.bbox_xyxy[2]) / 2.0)),
                    int(round((detection.bbox_xyxy[1] + detection.bbox_xyxy[3]) / 2.0)),
                )
                target_details[class_name] = {
                    "depth_m": float(depth_m),
                    "camera_xyz": [float(value) for value in position],
                    "bbox_center": [int(bbox_center[0]), int(bbox_center[1])],
                    "grasp_pixel": [int(center[0]), int(center[1])],
                    "target_mode": target_mode,
                }
                if valid_depth_ratio is not None:
                    target_details[class_name]["valid_depth_ratio"] = float(
                        valid_depth_ratio
                    )

                self._draw_grasp_sample(
                    debug_image,
                    bbox_center,
                    center,
                    sample_bounds,
                    depth_m,
                    target_mode,
                )
                self.get_logger().info(
                    f"{class_name}: {target_mode} pixel={center}, "
                    f"raw={position.round(4).tolist()}, "
                    f"grasp={grasp_position.round(4).tolist()}, "
                    "fixed_joint_path=''"
                )
            except ValueError as error:
                self.get_logger().warning(
                    f"{class_name} center target invalid: {error}"
                )
        return targets, target_details

    def _depth_guided_position_from_depth(
        self,
        detection: Detection,
        depth_image: np.ndarray,
    ) -> Tuple[
        np.ndarray,
        Tuple[int, int],
        Tuple[int, int, int, int],
        float,
        float,
    ]:
        """Select a body-centered median-Depth target for plastic_bottle."""

        if self.camera_info is None:
            raise ValueError("CameraInfo is unavailable")
        selection = select_depth_guided_target(
            depth_image=depth_image,
            bbox_xyxy=detection.bbox_xyxy,
            margin_ratio=self.depth_guided_margin_ratio,
            minimum_valid_pixels=self.minimum_valid_depth_pixels,
            minimum_valid_ratio=self.depth_guided_minimum_valid_ratio,
            depth_scale=self.depth_scale,
            minimum_m=self.minimum_depth_m,
            maximum_m=self.maximum_depth_m,
            tolerance_ratio=self.depth_guided_tolerance_ratio,
            minimum_tolerance_m=self.depth_guided_minimum_tolerance_m,
            patch_radius=self.depth_guided_patch_radius,
        )
        center_u, center_v = selection.pixel
        position = point_from_pixel_depth(
            center_u,
            center_v,
            selection.depth_m,
            self.camera_info.k,
        )
        return (
            position,
            selection.pixel,
            selection.sample_bounds,
            selection.depth_m,
            selection.valid_ratio,
        )

    def _center_position_from_depth(
        self,
        detection: Detection,
        depth_image: np.ndarray,
    ) -> Tuple[np.ndarray, Tuple[int, int], Tuple[int, int, int, int], float]:
        if self.camera_info is None:
            raise ValueError("CameraInfo is unavailable")
        center_u, center_v, sample_bounds, depth_m = bbox_center_depth_m(
            depth_image=depth_image,
            bbox_xyxy=detection.bbox_xyxy,
            center_ratio=self.suction_center_ratio,
            minimum_valid_pixels=self.minimum_valid_depth_pixels,
            depth_scale=self.depth_scale,
            minimum_m=self.minimum_depth_m,
            maximum_m=self.maximum_depth_m,
        )
        position = point_from_pixel_depth(
            center_u,
            center_v,
            depth_m,
            self.camera_info.k,
        )
        return (
            position,
            (center_u, center_v),
            sample_bounds,
            depth_m,
        )

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

    def _try_send_pending_goal(self) -> None:
        if not self.station_arrived or self.cycle_finished:
            return
        if self.goal_in_flight or not self.pending_targets:
            return
        if not self.remove_action_client.server_is_ready():
            self._set_debug_state(
                phase="TARGETS READY",
                message=f"Waiting for Action server {self.remove_action_name}",
            )
            self.get_logger().warning(
                f"Targets cached; waiting for Action server "
                f"{self.remove_action_name}",
                throttle_duration_sec=2.0,
            )
            return

        targets = self.pending_targets
        stamp = self.pending_stamp
        try:
            self._send_remove_goal(targets, stamp)
        except Exception as error:  # noqa: BLE001
            self.cycle_finished = True
            self._publish_completion(False)
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteRemove goal send failed: {error}",
            )
            self.get_logger().error(f"ExecuteRemove goal send failed: {error}")
            return

        self.pending_targets = None
        self.pending_stamp = None

    def _send_remove_goal(
        self,
        targets: List[RemoveTarget],
        stamp: object,
    ) -> None:
        goal = ExecuteRemove.Goal()
        goal.header.stamp = stamp
        goal.header.frame_id = self.camera_frame
        goal.targets = targets

        self.goal_in_flight = True
        self.last_goal_stamp = stamp
        self._set_debug_state(
            phase="ACTION QUEUED",
            current_task=0,
            total_tasks=len(targets),
            message=f"Sending {len(targets)} target(s) to {self.remove_action_name}",
        )
        self.get_logger().info(
            f"Sending {len(targets)} targets to {self.remove_action_name}"
        )
        try:
            future = self.remove_action_client.send_goal_async(
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
            self._publish_completion(False)
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteRemove goal call failed: {error}",
            )
            self.get_logger().error(f"ExecuteRemove goal call failed: {error}")
            return

        if not goal_handle.accepted:
            self.goal_in_flight = False
            self.cycle_finished = True
            self._publish_completion(False)
            self._mark_all_action_detections("REJECTED")
            self._set_debug_state(
                phase="ERROR",
                message="ERROR - Robot2 rejected the ExecuteRemove goal",
            )
            self.get_logger().error("Robot2 rejected the ExecuteRemove goal")
            return

        self._set_debug_state(
            phase="ROBOT WORKING",
            message="ExecuteRemove goal accepted - waiting for feedback",
        )
        self.get_logger().info("Robot2 accepted the ExecuteRemove goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def _on_action_feedback(self, feedback_message: object) -> None:
        feedback = feedback_message.feedback
        self._mark_action_progress(
            object_id=str(feedback.current_object_id),
            current=int(feedback.current_target),
            total=int(feedback.total_targets),
            active_state="REMOVING",
        )
        self._set_debug_state(
            phase="ROBOT WORKING",
            current_task=int(feedback.current_target),
            total_tasks=int(feedback.total_targets),
            message=(
                f"REMOVING OBJECT {feedback.current_target}/"
                f"{feedback.total_targets} - {feedback.current_object_id}"
            ),
        )
        self.get_logger().info(
            "Robot2 removing "
            f"{feedback.current_object_id} "
            f"({feedback.current_target}/{feedback.total_targets})"
        )

    def _on_action_result(self, future: object) -> None:
        self.goal_in_flight = False
        try:
            result = future.result().result
        except Exception as error:  # noqa: BLE001
            self.cycle_finished = True
            self._publish_completion(False)
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteRemove result failed: {error}",
            )
            self.get_logger().error(f"ExecuteRemove result failed: {error}")
            return

        if not result.success:
            self.cycle_finished = True
            self._publish_completion(False)
            self._mark_all_action_detections("FAILED")
            self._set_debug_state(
                phase="ERROR",
                message=f"ERROR - ExecuteRemove failed: {result.message}",
            )
            self.get_logger().error(f"ExecuteRemove failed: {result.message}")
            return

        self._mark_all_action_detections("COMPLETE")
        self.removal_action_succeeded = True
        self.frame_locked = False
        self.empty_frame_count = 0
        if self.last_goal_stamp is not None:
            self.minimum_frame_stamp_ns = self._stamp_to_ns(
                self.last_goal_stamp
            )
        delay_ns = int(self.reinspect_delay_sec * 1_000_000_000)
        self.next_detection_not_before_ns = (
            self.get_clock().now().nanoseconds + delay_ns
        )
        self.debug_recheck_count += 1
        self._set_debug_state(
            phase="RECHECK",
            current_task=self.debug_total_tasks,
            message=(
                f"REMOVAL ACTION COMPLETE - {result.message}; "
                "waiting for a newer Camera2 frame"
            ),
        )
        self.get_logger().info(
            f"ExecuteRemove succeeded: {result.message}; "
            "waiting for a newer Camera2 frame"
        )

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
        target_details: Dict[str, dict],
    ) -> List[dict]:
        target_indices = {
            class_name: index
            for index, class_name in enumerate(
                [
                    name
                    for name in self.expected_classes
                    if name in target_details
                ],
                start=1,
            )
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
            elif class_name in target_indices:
                item.update(
                    action_included=True,
                    action_state="QUEUED",
                    task_index=target_indices[class_name],
                    reject_reason="",
                )
                item.update(target_details[class_name])
            else:
                item.update(
                    action_included=False,
                    action_state="EXCLUDED",
                    task_index=0,
                    reject_reason="INVALID DEPTH",
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
            "scan": (
                "INITIAL SCAN"
                if self.debug_recheck_count == 0
                else f"RECHECK {self.debug_recheck_count}"
            ),
            "station": self.debug_station,
            "action_server": (
                "READY"
                if self.remove_action_client.server_is_ready()
                else "WAITING"
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

    def _schedule_detection_retry(self) -> None:
        delay_ns = int(self.detection_retry_period_sec * 1_000_000_000)
        self.next_detection_not_before_ns = (
            self.get_clock().now().nanoseconds + delay_ns
        )

    def _publish_completion(self, completed: bool) -> None:
        message = Bool()
        message.data = completed
        self.completion_publisher.publish(message)

    @staticmethod
    def _stamp_to_ns(stamp: object) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    @staticmethod
    def _to_point_message(point_xyz: Sequence[float]) -> Point:
        point = Point()
        point.x = float(point_xyz[0])
        point.y = float(point_xyz[1])
        point.z = float(point_xyz[2])
        return point

    @staticmethod
    def _draw_roi(
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
    def _draw_grasp_sample(
        image: np.ndarray,
        bbox_center: Tuple[int, int],
        center: Tuple[int, int],
        bounds: Tuple[int, int, int, int],
        depth_m: float,
        target_mode: str,
    ) -> None:
        x0, y0, x1, y1 = bounds
        cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 2)
        cv2.circle(image, bbox_center, 6, (255, 255, 255), 2)
        cv2.putText(
            image,
            "C",
            (bbox_center[0] + 7, bbox_center[1] - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.line(image, bbox_center, center, (40, 220, 80), 2, cv2.LINE_AA)
        cv2.circle(image, center, 7, (40, 220, 80), -1)
        cv2.putText(
            image,
            "G",
            (center[0] + 8, center[1] - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (40, 220, 80),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"{target_mode} {depth_m:.3f} m",
            (center[0] + 9, center[1] + 18),
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
    """Run the NetClean Vision2 node."""
    rclpy.init(args=args)
    node: Vision2Node | None = None
    try:
        node = Vision2Node()
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
