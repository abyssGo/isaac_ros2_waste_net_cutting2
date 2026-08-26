#!/usr/bin/env python3
"""NetClean Standalone의 표준 ROS 2 저수준 인터페이스."""

from __future__ import annotations

from collections import deque
import math
from typing import Deque, Optional, Sequence, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String, UInt32

from m0609_controller import TcpMotionRequest, normalize_quaternion_wxyz


WORLD_FRAME = "world"


class SimRosInterface(Node):
    """Callback에서는 값 검증과 queue 적재만 수행한다."""

    def __init__(self, topics: dict) -> None:
        super().__init__("netclean_standalone")
        self.topics = dict(topics)

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sim_ready_pub = self.create_publisher(
            Bool, self._topic("sim_ready"), latched
        )
        self.reset_done_pub = self.create_publisher(
            Bool, self._topic("reset_done"), reliable
        )
        self.net_pose_pub = self.create_publisher(
            Pose, self._topic("net_current_pose"), reliable
        )
        self.robot1_done_pub = self.create_publisher(
            Bool, self._topic("robot1_motion_done"), reliable
        )
        self.robot2_done_pub = self.create_publisher(
            Bool, self._topic("robot2_motion_done"), reliable
        )
        self.suction_state_pub = self.create_publisher(
            Bool, self._topic("suction_state"), reliable
        )
        self.joint_release_done_pub = self.create_publisher(
            Bool, self._topic("joint_release_done"), reliable
        )
        self.state_pub = self.create_publisher(
            String, self._topic("sim_state"), latched
        )
        self.fault_pub = self.create_publisher(
            String, self._topic("sim_fault"), latched
        )
        self.cycle_pub = self.create_publisher(
            UInt32, self._topic("sim_cycle"), latched
        )

        self.net_targets: Deque[np.ndarray] = deque(maxlen=1)
        self.robot1_requests: Deque[TcpMotionRequest] = deque(maxlen=1)
        self.robot2_requests: Deque[TcpMotionRequest] = deque(maxlen=1)
        self.suction_requests: Deque[bool] = deque(maxlen=1)
        self.release_requests: Deque[str] = deque(maxlen=1)
        self.reset_requests: Deque[bool] = deque(maxlen=1)

        self.create_subscription(
            Bool,
            self._topic("reset_request"),
            self._reset_callback,
            reliable,
        )
        self.create_subscription(
            Pose,
            self._topic("net_target_pose"),
            self._net_target_callback,
            reliable,
        )
        self.create_subscription(
            PoseStamped,
            self._topic("robot1_motion_command"),
            self._robot1_callback,
            reliable,
        )
        self.create_subscription(
            PoseStamped,
            self._topic("robot2_motion_command"),
            self._robot2_callback,
            reliable,
        )
        self.create_subscription(
            Bool,
            self._topic("suction_command"),
            self._suction_callback,
            reliable,
        )
        self.create_subscription(
            String,
            self._topic("release_object"),
            self._release_callback,
            reliable,
        )

    def _topic(self, key: str) -> str:
        value = str(self.topics.get(key, "")).strip()
        if not value.startswith("/"):
            raise ValueError(f"topics.{key}가 절대 Topic 이름이 아닙니다: {value!r}")
        return value

    def _net_target_callback(self, msg: Pose) -> None:
        values = np.array(
            [msg.position.x, msg.position.y, msg.position.z], dtype=float
        )
        if np.any(~np.isfinite(values)):
            self.get_logger().error("/net/target_pose에 NaN/Inf가 있어 거부했습니다.")
            return
        if len(self.net_targets) >= self.net_targets.maxlen:
            self.get_logger().error("대기 중인 Net 명령이 있어 새 명령을 거부했습니다.")
            return
        self.net_targets.append(values)

    def _robot1_callback(self, msg: PoseStamped) -> None:
        request = self._motion_request(msg, "robot1")
        if request is None:
            self.publish_robot1_done(False)
            return
        if len(self.robot1_requests) >= self.robot1_requests.maxlen:
            self.get_logger().error("Robot1 명령 queue가 차 있어 거부했습니다.")
            self.publish_robot1_done(False)
            return
        self.robot1_requests.append(request)

    def _robot2_callback(self, msg: PoseStamped) -> None:
        request = self._motion_request(msg, "robot2")
        if request is None:
            self.publish_robot2_done(False)
            return
        if len(self.robot2_requests) >= self.robot2_requests.maxlen:
            self.get_logger().error("Robot2 명령 queue가 차 있어 거부했습니다.")
            self.publish_robot2_done(False)
            return
        self.robot2_requests.append(request)

    def _motion_request(
        self, msg: PoseStamped, label: str
    ) -> Optional[TcpMotionRequest]:
        if msg.header.frame_id.strip() != WORLD_FRAME:
            self.get_logger().error(
                f"{label}: frame_id는 {WORLD_FRAME!r}여야 합니다: "
                f"{msg.header.frame_id!r}"
            )
            return None
        position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        if np.any(~np.isfinite(position)):
            self.get_logger().error(f"{label}: target position에 NaN/Inf가 있습니다.")
            return None
        try:
            orientation = normalize_quaternion_wxyz(
                (
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                ),
                f"{label} target orientation",
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return None
        return TcpMotionRequest(position, orientation)

    def _suction_callback(self, msg: Bool) -> None:
        if len(self.suction_requests) >= self.suction_requests.maxlen:
            self.get_logger().error("Suction 명령 queue가 차 있어 거부했습니다.")
            self.publish_suction_state(False)
            return
        self.suction_requests.append(bool(msg.data))

    def _release_callback(self, msg: String) -> None:
        class_name = msg.data.strip().lower()
        if not class_name:
            self.get_logger().error("빈 class_name Joint 해제 요청을 거부했습니다.")
            self.publish_joint_release_done(False)
            return
        if len(self.release_requests) >= self.release_requests.maxlen:
            self.get_logger().error("Joint 해제 queue가 차 있어 거부했습니다.")
            self.publish_joint_release_done(False)
            return
        self.release_requests.append(class_name)

    def _reset_callback(self, msg: Bool) -> None:
        if msg.data:
            self.reset_requests.clear()
            self.reset_requests.append(True)

    def clear_work_queues(self, preserve_reset: bool = False) -> None:
        self.net_targets.clear()
        self.robot1_requests.clear()
        self.robot2_requests.clear()
        self.suction_requests.clear()
        self.release_requests.clear()
        if not preserve_reset:
            self.reset_requests.clear()

    @staticmethod
    def _publish_bool(publisher, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def publish_ready(self, ready: bool) -> None:
        self._publish_bool(self.sim_ready_pub, ready)

    def publish_reset_done(self, success: bool) -> None:
        self._publish_bool(self.reset_done_pub, success)

    def publish_robot1_done(self, success: bool) -> None:
        self._publish_bool(self.robot1_done_pub, success)

    def publish_robot2_done(self, success: bool) -> None:
        self._publish_bool(self.robot2_done_pub, success)

    def publish_suction_state(self, enabled: bool) -> None:
        self._publish_bool(self.suction_state_pub, enabled)

    def publish_joint_release_done(self, success: bool) -> None:
        self._publish_bool(self.joint_release_done_pub, success)

    def publish_net_pose(
        self,
        position_xyz: Sequence[float],
        orientation_wxyz: Sequence[float],
    ) -> None:
        position = np.asarray(position_xyz, dtype=float)
        orientation = np.asarray(orientation_wxyz, dtype=float)
        msg = Pose()
        msg.position.x = float(position[0])
        msg.position.y = float(position[1])
        msg.position.z = float(position[2])
        msg.orientation.x = float(orientation[1])
        msg.orientation.y = float(orientation[2])
        msg.orientation.z = float(orientation[3])
        msg.orientation.w = float(orientation[0])
        self.net_pose_pub.publish(msg)

    def publish_state(self, value: str) -> None:
        msg = String()
        msg.data = str(value)
        self.state_pub.publish(msg)

    def publish_fault(self, value: str) -> None:
        msg = String()
        msg.data = str(value)
        self.fault_pub.publish(msg)

    def publish_cycle(self, cycle: int) -> None:
        msg = UInt32()
        msg.data = max(0, int(cycle))
        self.cycle_pub.publish(msg)
