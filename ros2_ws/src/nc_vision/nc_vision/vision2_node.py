"""NetClean Station2 object detection and remove-target Action client."""

from __future__ import annotations

import os
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
from std_msgs.msg import Bool
from ultralytics import YOLO

from nc_interfaces.action import ExecuteRemove
from nc_interfaces.msg import RemoveTarget

from .depth_utils import bbox_center_depth_m, point_from_pixel_depth
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

        self.get_logger().info("NetClean Vision2 started")
        self.get_logger().info(
            f"Camera topics: {self.rgb_topic}, {self.depth_topic}, "
            f"{self.camera_info_topic}"
        )
        self.get_logger().info(
            "Vision2 sends bbox-center camera XYZ and class_name; "
            "Robot2 owns fixed-joint mapping"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("model_path", "")
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
            "completion_topic",
            "/vision2/removal_complete",
        )
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("sync_slop_sec", 0.10)

        self.declare_parameter("use_roi", False)
        self.declare_parameter("roi_x0", 0)
        self.declare_parameter("roi_y0", 0)
        self.declare_parameter("roi_width", 1280)
        self.declare_parameter("roi_height", 720)

        self.declare_parameter("suction_center_ratio", 0.25)
        self.declare_parameter("minimum_valid_depth_pixels", 20)
        self.declare_parameter("depth_scale", 1.0)
        self.declare_parameter("minimum_depth_m", 0.05)
        self.declare_parameter("maximum_depth_m", 5.0)
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
        self.completion_topic = str(value("completion_topic"))
        self.sync_queue_size = int(value("sync_queue_size"))
        self.sync_slop_sec = float(value("sync_slop_sec"))

        self.use_roi = bool(value("use_roi"))
        self.roi_x0 = int(value("roi_x0"))
        self.roi_y0 = int(value("roi_y0"))
        self.roi_width = int(value("roi_width"))
        self.roi_height = int(value("roi_height"))

        self.suction_center_ratio = float(value("suction_center_ratio"))
        self.minimum_valid_depth_pixels = int(
            value("minimum_valid_depth_pixels")
        )
        self.depth_scale = float(value("depth_scale"))
        self.minimum_depth_m = float(value("minimum_depth_m"))
        self.maximum_depth_m = float(value("maximum_depth_m"))
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
            self._publish_completion(False)
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
            self.get_logger().error(f"Vision2 frame rejected: {error}")
            self._schedule_detection_retry()
        except Exception as error:  # noqa: BLE001
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

        detections = best_detection_per_class(
            result=results[0],
            names=self.model.names,
            allowed_classes=self.expected_classes,
            confidence_threshold=self.action_confidence,
            offset_xy=(x0, y0),
        )

        debug_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        self._draw_roi(debug_image, roi_bounds)
        targets = self._build_targets(
            detections=detections,
            depth_image=depth_image,
            debug_image=debug_image,
        )
        self._publish_debug(debug_image, rgb_message)

        if not detections:
            self._handle_empty_detection()
            return

        self.empty_frame_count = 0
        if not targets:
            self.get_logger().warning(
                "Objects detected, but no valid center Depth was available",
                throttle_duration_sec=1.0,
            )
            self._schedule_detection_retry()
            return

        self.pending_targets = targets
        self.pending_stamp = rgb_message.header.stamp
        self.frame_locked = True
        self.get_logger().info(
            f"Cached {len(targets)} Station2 remove targets"
        )
        self._try_send_pending_goal()

    def _handle_empty_detection(self) -> None:
        if not self.removal_action_succeeded:
            self.get_logger().warning(
                "No Station2 objects detected yet",
                throttle_duration_sec=1.0,
            )
            self._schedule_detection_retry()
            return

        self.empty_frame_count += 1
        self.get_logger().info(
            "Station2 empty confirmation "
            f"{self.empty_frame_count}/{self.empty_confirmation_frames}"
        )
        if self.empty_frame_count < self.empty_confirmation_frames:
            self._schedule_detection_retry()
            return

        self.cycle_finished = True
        self._publish_completion(True)
        self.get_logger().info(
            "Station2 removal complete; no objects remain"
        )

    def _build_targets(
        self,
        detections: Dict[str, Detection],
        depth_image: np.ndarray,
        debug_image: np.ndarray,
    ) -> List[RemoveTarget]:
        targets: List[RemoveTarget] = []
        for class_name in self.expected_classes:
            detection = detections.get(class_name)
            if detection is None:
                continue

            self._draw_detection(debug_image, detection)
            try:
                position, center, sample_bounds, depth_m = (
                    self._center_position_from_depth(detection, depth_image)
                )
                target = RemoveTarget()
                target.object_id = class_name
                target.class_name = class_name
                target.position = self._to_point_message(position)
                target.fixed_joint_path = ""
                targets.append(target)

                self._draw_grasp_sample(
                    debug_image,
                    center,
                    sample_bounds,
                    depth_m,
                )
                self.get_logger().info(
                    f"{class_name}: center XYZ={position.round(4).tolist()}, "
                    "fixed_joint_path=''"
                )
            except ValueError as error:
                self.get_logger().warning(
                    f"{class_name} center target invalid: {error}"
                )
        return targets

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
            self.get_logger().error(f"ExecuteRemove goal call failed: {error}")
            return

        if not goal_handle.accepted:
            self.goal_in_flight = False
            self.cycle_finished = True
            self.get_logger().error("Robot2 rejected the ExecuteRemove goal")
            return

        self.get_logger().info("Robot2 accepted the ExecuteRemove goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_action_result)

    def _on_action_feedback(self, feedback_message: object) -> None:
        feedback = feedback_message.feedback
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
            self.get_logger().error(f"ExecuteRemove result failed: {error}")
            return

        if not result.success:
            self.cycle_finished = True
            self.get_logger().error(f"ExecuteRemove failed: {result.message}")
            return

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
        self.get_logger().info(
            f"ExecuteRemove succeeded: {result.message}; "
            "waiting for a newer Camera2 frame"
        )

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
        center: Tuple[int, int],
        bounds: Tuple[int, int, int, int],
        depth_m: float,
    ) -> None:
        x0, y0, x1, y1 = bounds
        cv2.rectangle(image, (x0, y0), (x1 - 1, y1 - 1), (0, 255, 255), 2)
        cv2.circle(image, center, 7, (255, 0, 255), -1)
        cv2.putText(
            image,
            f"{depth_m:.3f} m",
            (center[0] + 9, center[1]),
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
