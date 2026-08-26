"""ROS 2 wrapper around the OpenCV popup renderer."""

from __future__ import annotations

import json
import os
from typing import Any, Dict, Optional

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

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
        self.declare_parameter("window_height", 720)
        self.declare_parameter("render_fps", 20.0)
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
        self._status: Dict[str, Any] = {}
        self._invalid_json_warned = False

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
                "image_topic": "/vision1/debug/source_image",
                "status_topic": "/vision1/debug/status",
                "output_topic": "/vision1/debug_dashboard",
                "window_name": "NetClean Vision 1 - Cutting Debug",
            }
        return {
            "image_topic": "/vision2/debug/source_image",
            "status_topic": "/vision2/debug/status",
            "output_topic": "/vision2/debug_dashboard",
            "window_name": "NetClean Vision 2 - Removal Debug",
        }

    @staticmethod
    def _has_display() -> bool:
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))

    def _on_image(self, message: Image) -> None:
        try:
            self._last_frame = self._bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception as exc:  # cv_bridge raises backend-specific exceptions
            self.get_logger().error(f"Failed to convert debug image: {exc}")

    def _on_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError("top-level JSON value must be an object")
            self._status = payload
            self._invalid_json_warned = False
        except (json.JSONDecodeError, ValueError) as exc:
            if not self._invalid_json_warned:
                self.get_logger().warning(f"Ignored invalid debug status JSON: {exc}")
                self._invalid_json_warned = True

    def _render(self) -> None:
        if self._last_frame is None:
            return
        canvas = self._renderer.render(self._last_frame, self._status)
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
