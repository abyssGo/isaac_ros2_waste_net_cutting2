"""ROS 2 wrapper around the OpenCV popup renderer."""

from __future__ import annotations

import json
import os
import threading
import time
from typing import Any, Dict, Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ultralytics import YOLO

from .renderer import PopupRenderer


class DebugPopupNode(Node):
    """Subscribe to an image and JSON status, render, publish, and display it."""

    def __init__(self, mode: str) -> None:
        node_name = f"{mode}_debug_popup"
        super().__init__(node_name)
        defaults = self._defaults(mode)

        self.declare_parameter("image_topic", defaults["image_topic"])
        self.declare_parameter("status_topic", defaults["status_topic"])
        self.declare_parameter("output_topic", defaults["output_topic"])
        self.declare_parameter("window_name", defaults["window_name"])
        self.declare_parameter("enable_window", True)
        self.declare_parameter("window_width", 1280)
        self.declare_parameter("window_height", 800)
        self.declare_parameter("render_fps", 20.0)
        self.declare_parameter("live_detection_enabled", True)
        self.declare_parameter(
            "model_path",
            "models/netclean_yolo11n/weights/best.pt",
        )
        self.declare_parameter("device", "0")
        self.declare_parameter("model_imgsz", 640)
        self.declare_parameter("confidence", 0.40)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("max_detections", 3)
        self.declare_parameter("live_detection_interval_sec", 0.10)
        self.declare_parameter(
            "expected_classes",
            ["plastic_bottle", "can", "buoy"],
        )
        self.declare_parameter("roi", [0, 0, 0, 0])
        self.declare_parameter("validation_precision", -1.0)
        self.declare_parameter("validation_recall", -1.0)
        self.declare_parameter("validation_map50", -1.0)
        self.declare_parameter("validation_map50_95", -1.0)

        image_topic = str(self.get_parameter("image_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._window_name = str(self.get_parameter("window_name").value)
        self._window_enabled = bool(self.get_parameter("enable_window").value)
        self._window_created = False
        self._bridge = CvBridge()
        self._last_frame = None
        self._frame_generation = 0
        self._status: Dict[str, Any] = {}
        self._invalid_json_warned = False
        self._data_lock = threading.Lock()
        self._worker_event = threading.Event()
        self._worker_stop = threading.Event()
        self._worker_thread: Optional[threading.Thread] = None
        self._live_detections = []
        self._live_inference_ms: Optional[float] = None
        self._live_fps: Optional[float] = None
        self._live_detection_enabled = bool(
            self.get_parameter("live_detection_enabled").value
        )
        self._live_detection_interval_sec = max(
            0.01,
            float(self.get_parameter("live_detection_interval_sec").value),
        )
        self._expected_classes = [
            str(value)
            for value in self.get_parameter("expected_classes").value
        ]
        self._model = None

        roi = list(self.get_parameter("roi").value)
        if len(roi) != 4 or roi == [0, 0, 0, 0]:
            roi = None
        validation_defaults = {
            "precision": self.get_parameter("validation_precision").value,
            "recall": self.get_parameter("validation_recall").value,
            "map50": self.get_parameter("validation_map50").value,
            "map50_95": self.get_parameter("validation_map50_95").value,
        }
        self._renderer = PopupRenderer(
            mode=mode,
            width=int(self.get_parameter("window_width").value),
            height=int(self.get_parameter("window_height").value),
            roi=roi,
            validation_defaults=validation_defaults,
        )

        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._status_sub = self.create_subscription(
            String,
            status_topic,
            self._on_status,
            10,
        )
        self._output_pub = self.create_publisher(Image, output_topic, 10)

        render_fps = max(1.0, float(self.get_parameter("render_fps").value))
        self._timer = self.create_timer(1.0 / render_fps, self._render)

        if self._live_detection_enabled:
            model_path = str(self.get_parameter("model_path").value)
            self.get_logger().info(
                f"Loading UI-only live detector: {model_path}"
            )
            self._model = YOLO(model_path)
            self._worker_thread = threading.Thread(
                target=self._live_detection_loop,
                name=f"{mode}_live_detector",
                daemon=True,
            )
            self._worker_thread.start()

        if self._window_enabled and not self._has_display():
            self._window_enabled = False
            self.get_logger().warning(
                "DISPLAY/WAYLAND_DISPLAY is unavailable. The popup is disabled, "
                f"but annotated frames will still be published on {output_topic}."
            )

        self.get_logger().info(
            f"{mode} popup ready: image={image_topic}, status={status_topic}, "
            f"output={output_topic}, window={self._window_enabled}"
        )

    @staticmethod
    def _defaults(mode: str) -> Dict[str, str]:
        if mode == "vision1":
            return {
                "image_topic": "/camera1/rgb/image_raw",
                "status_topic": "/vision1/debug/status",
                "output_topic": "/vision1/debug_dashboard",
                "window_name": "NetClean Vision 1 - Cutting Debug",
            }
        return {
            "image_topic": "/camera2/rgb/image_raw",
            "status_topic": "/vision2/debug/status",
            "output_topic": "/vision2/debug_dashboard",
            "window_name": "NetClean Vision 2 - Removal Debug",
        }

    @staticmethod
    def _has_display() -> bool:
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

    def _on_image(self, message: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            with self._data_lock:
                self._last_frame = frame
                self._frame_generation += 1
            self._worker_event.set()
        except Exception as exc:  # cv_bridge raises backend-specific exceptions
            self.get_logger().error(f"Failed to convert debug image: {exc}")

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("top-level JSON value must be an object")
            with self._data_lock:
                self._status = payload
            self._invalid_json_warned = False
        except (json.JSONDecodeError, ValueError) as exc:
            if not self._invalid_json_warned:
                self.get_logger().warning(f"Ignored invalid debug status JSON: {exc}")
            self._invalid_json_warned = True

    def _live_detection_loop(self) -> None:
        """Run UI-only YOLO in a worker; never publish an Action target."""

        last_generation = -1
        next_allowed = 0.0
        while not self._worker_stop.is_set():
            self._worker_event.wait(timeout=0.20)
            self._worker_event.clear()
            if self._worker_stop.is_set():
                break
            now = time.monotonic()
            if now < next_allowed:
                self._worker_stop.wait(next_allowed - now)
                if self._worker_stop.is_set():
                    break
            with self._data_lock:
                if self._last_frame is None or self._frame_generation == last_generation:
                    continue
                frame = self._last_frame.copy()
                generation = self._frame_generation
            started = time.perf_counter()
            try:
                results = self._model.predict(
                    source=frame,
                    imgsz=int(self.get_parameter("model_imgsz").value),
                    conf=float(self.get_parameter("confidence").value),
                    iou=float(self.get_parameter("iou_threshold").value),
                    max_det=int(self.get_parameter("max_detections").value),
                    device=str(self.get_parameter("device").value),
                    verbose=False,
                )
                detections = self._result_detections(results[0]) if results else []
                inference_ms = (time.perf_counter() - started) * 1000.0
                with self._data_lock:
                    self._live_detections = detections
                    self._live_inference_ms = inference_ms
                    self._live_fps = 1000.0 / inference_ms if inference_ms > 0 else None
                last_generation = generation
                next_allowed = time.monotonic() + self._live_detection_interval_sec
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"UI-only live detection failed: {exc}")
                next_allowed = time.monotonic() + 1.0

    def _result_detections(self, result: Any) -> list:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []
        names = self._model.names
        best_by_class = {}
        for xyxy, confidence, class_index in zip(
            boxes.xyxy.detach().cpu().numpy(),
            boxes.conf.detach().cpu().numpy(),
            boxes.cls.detach().cpu().numpy(),
        ):
            class_name = str(names[int(class_index)])
            if class_name not in self._expected_classes:
                continue
            x1, y1, x2, y2 = [int(round(float(value))) for value in xyxy]
            confidence_value = float(confidence)
            previous = best_by_class.get(class_name)
            if previous is not None and previous["confidence"] >= confidence_value:
                continue
            best_by_class[class_name] = {
                "object_id": class_name,
                "class_name": class_name,
                "confidence": confidence_value,
                "bbox": [x1, y1, x2, y2],
            }
        return [
            best_by_class[class_name]
            for class_name in self._expected_classes
            if class_name in best_by_class
        ]

    @staticmethod
    def _merge_live_status(status: Dict[str, Any], live_detections: list) -> Dict[str, Any]:
        merged_status = dict(status)
        action_by_class = {
            str(item.get("class_name")): item
            for item in status.get("detections", [])
            if isinstance(item, dict)
        }
        merged = []
        for live in live_detections:
            item = dict(action_by_class.get(live["class_name"], {}))
            item.update(live)
            if "action_state" not in item:
                item.update(
                    action_included=False,
                    action_state="LIVE MONITOR",
                    task_index=0,
                    reject_reason="",
                )
            item["live_detection"] = True
            merged.append(item)
        merged_status["detections"] = merged
        merged_status["display_mode"] = "LIVE UI DETECTION"
        return merged_status

    def _render(self) -> None:
        with self._data_lock:
            if self._last_frame is None:
                return
            frame = self._last_frame.copy()
            live_detections = list(self._live_detections)
            live_inference_ms = self._live_inference_ms
            live_fps = self._live_fps
            status = dict(self._status)
        if self._live_detection_enabled:
            status = self._merge_live_status(status, live_detections)
            status["inference_ms"] = live_inference_ms
            status["fps"] = live_fps
        if frame is None:
            return
        canvas = self._renderer.render(frame, status)
        output_message = self._bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        output_message.header.stamp = self.get_clock().now().to_msg()
        output_message.header.frame_id = "vision_debug_dashboard"
        self._output_pub.publish(output_message)

        if not self._window_enabled:
            return
        try:
            if not self._window_created:
                cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(
                    self._window_name,
                    self._renderer.width,
                    self._renderer.height,
                )
                self._window_created = True
            cv2.imshow(self._window_name, canvas)
            cv2.waitKey(1)
        except cv2.error as exc:
            self._window_enabled = False
            self.get_logger().error(
                f"OpenCV popup was disabled after a GUI error: {exc}"
            )

    def destroy_node(self) -> bool:
        self._worker_stop.set()
        self._worker_event.set()
        if self._worker_thread is not None:
            self._worker_thread.join(timeout=2.0)
        if self._window_created:
            try:
                cv2.destroyWindow(self._window_name)
                cv2.waitKey(1)
            except cv2.error:
                pass
        return super().destroy_node()


def run_popup(mode: str, args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = DebugPopupNode(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
