#!/usr/bin/env python3
"""NetClean 반복 공정과 어망 이송을 관리하는 ROS 2 상위 FSM."""

from __future__ import annotations

from enum import Enum, auto
import math
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class ProcessState(Enum):
    WAIT_SIM = auto()
    MOVING_STATION1 = auto()
    WAIT_STATION1_COMPLETE = auto()
    MOVING_STATION2 = auto()
    WAIT_STATION2_COMPLETE = auto()
    MOVING_EXIT = auto()
    RESETTING = auto()
    COMPLETED = auto()
    ERROR = auto()


class ControlNode(Node):
    """START -> S1 -> S2 -> EXIT -> RESET 반복 상태 머신."""

    def __init__(self) -> None:
        super().__init__("control_node")

        # NetRoot 원점의 World 좌표
        self._declare_xyz("station1", (-0.7, -2.0, 0.71))
        self._declare_xyz("station2", (-0.7, 2.0, 0.71))
        self._declare_xyz("exit", (-0.7, 4.0, 0.71))
        self.declare_parameter("position_tolerance", 0.02)
        self.declare_parameter("expected_net_speed_mps", 0.30)

        # 반복 정책: max_cycles=0이면 Ctrl+C까지 무한 반복
        self.declare_parameter("repeat_enabled", True)
        self.declare_parameter("max_cycles", 0)

        # Watchdog는 wall clock을 사용해 /clock 정지까지 감지한다.
        self.declare_parameter("minimum_net_timeout_sec", 20.0)
        self.declare_parameter("net_timeout_scale", 2.5)
        self.declare_parameter("net_timeout_margin_sec", 8.0)
        self.declare_parameter("station1_work_timeout_sec", 180.0)
        self.declare_parameter("station2_work_timeout_sec", 240.0)
        self.declare_parameter("reset_timeout_sec", 45.0)

        self.station1_pose = self._pose_from_params("station1")
        self.station2_pose = self._pose_from_params("station2")
        self.exit_pose = self._pose_from_params("exit")
        self.position_tolerance = float(
            self.get_parameter("position_tolerance").value
        )
        self.expected_net_speed = float(
            self.get_parameter("expected_net_speed_mps").value
        )
        self.repeat_enabled = bool(
            self.get_parameter("repeat_enabled").value
        )
        self.max_cycles = int(self.get_parameter("max_cycles").value)

        if self.position_tolerance <= 0.0:
            raise ValueError("position_tolerance은 0보다 커야 합니다.")
        if self.expected_net_speed <= 0.0:
            raise ValueError("expected_net_speed_mps는 0보다 커야 합니다.")
        if self.max_cycles < 0:
            raise ValueError("max_cycles는 0 이상이어야 합니다.")

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # control -> standalone
        self.target_pose_pub = self.create_publisher(
            Pose, "/net/target_pose", reliable
        )
        self.reset_request_pub = self.create_publisher(
            Bool, "/sim/reset_request", reliable
        )

        # control -> vision
        self.station1_arrived_pub = self.create_publisher(
            Bool, "/station1/net_arrived", reliable
        )
        self.station2_arrived_pub = self.create_publisher(
            Bool, "/station2/net_arrived", reliable
        )
        self.process_state_pub = self.create_publisher(
            String, "/process/state", latched
        )
        self.process_fault_pub = self.create_publisher(
            String, "/process/fault", latched
        )

        # standalone -> control
        self.create_subscription(
            Bool, "/sim/ready", self._sim_ready_callback, latched
        )
        self.create_subscription(
            Pose, "/net/current_pose", self._current_pose_callback, reliable
        )
        self.create_subscription(
            Bool, "/sim/reset_done", self._reset_done_callback, reliable
        )
        self.create_subscription(
            String, "/sim/fault", self._sim_fault_callback, latched
        )

        # robot/vision -> control
        self.create_subscription(
            Bool,
            "/station1/cut_complete",
            self._cut_complete_callback,
            reliable,
        )
        self.create_subscription(
            Bool,
            "/station2/complete",
            self._station2_complete_callback,
            reliable,
        )

        self.state = ProcessState.WAIT_SIM
        self.cycle_count = 1
        self._current_pose: Optional[Pose] = None
        self._state_started_wall = time.monotonic()
        self._state_timeout_sec: Optional[float] = None
        self._sim_ready = False

        # use_sim_time=True여도 /clock 정지를 감지하도록 steady wall clock timer 사용.
        self._watchdog_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.25, self._watchdog, clock=self._watchdog_clock)
        self._publish_process_state()
        self.get_logger().info("어망 분리 공정을 시작합니다!")
        self.get_logger().info("Isaac Sim 준비를 기다리는 중...")

    def _declare_xyz(self, prefix: str, values) -> None:
        self.declare_parameter(f"{prefix}_x", float(values[0]))
        self.declare_parameter(f"{prefix}_y", float(values[1]))
        self.declare_parameter(f"{prefix}_z", float(values[2]))

    def _pose_from_params(self, prefix: str) -> Pose:
        pose = Pose()
        pose.position.x = float(self.get_parameter(f"{prefix}_x").value)
        pose.position.y = float(self.get_parameter(f"{prefix}_y").value)
        pose.position.z = float(self.get_parameter(f"{prefix}_z").value)
        # Standalone 레일은 회전하지 않고 현재 NetRoot orientation을 보존한다.
        pose.orientation.w = 1.0
        return pose

    def _sim_ready_callback(self, msg: Bool) -> None:
        self._sim_ready = bool(msg.data)
        if not msg.data:
            if self.state not in {
                ProcessState.WAIT_SIM,
                ProcessState.RESETTING,
                ProcessState.COMPLETED,
                ProcessState.ERROR,
            }:
                self.get_logger().warning(
                    f"작업 중 /sim/ready=False 수신: state={self.state.name}"
                )
            return
        if self.state is ProcessState.WAIT_SIM:
            self.get_logger().info("Isaac Sim 준비 완료!")
            self._command_station1()

    def _current_pose_callback(self, msg: Pose) -> None:
        self._current_pose = msg
        if self.state is ProcessState.MOVING_STATION1:
            if self._is_arrived(msg, self.station1_pose):
                self._transition(
                    ProcessState.WAIT_STATION1_COMPLETE,
                    float(self.get_parameter("station1_work_timeout_sec").value),
                )
                self.get_logger().info("어망이 1번 공정 구역에 도착했습니다!")
                self._publish_bool(self.station1_arrived_pub, True)
            return

        if self.state is ProcessState.MOVING_STATION2:
            if self._is_arrived(msg, self.station2_pose):
                self._transition(
                    ProcessState.WAIT_STATION2_COMPLETE,
                    float(self.get_parameter("station2_work_timeout_sec").value),
                )
                self.get_logger().info("어망이 2번 공정 구역에 도착했습니다!")
                self._publish_bool(self.station2_arrived_pub, True)
            return

        if self.state is ProcessState.MOVING_EXIT:
            if self._is_arrived(msg, self.exit_pose):
                self._finish_cycle()

    def _cut_complete_callback(self, msg: Bool) -> None:
        if self.state is not ProcessState.WAIT_STATION1_COMPLETE:
            return
        if not msg.data:
            self._enter_error("Robot1이 절단 실패를 보고했습니다.")
            return
        self.get_logger().info("어망이 2번 공정 구역으로 진입하는 중...")
        self._command_net(self.station2_pose, ProcessState.MOVING_STATION2)

    def _station2_complete_callback(self, msg: Bool) -> None:
        if self.state is not ProcessState.WAIT_STATION2_COMPLETE:
            return
        if not msg.data:
            self._enter_error("Vision2가 2번 공정 실패를 보고했습니다.")
            return
        self.get_logger().info(
            "어망 분리 공정이 종료되었습니다! EXIT로 이동합니다."
        )
        self._command_net(self.exit_pose, ProcessState.MOVING_EXIT)

    def _finish_cycle(self) -> None:
        self.get_logger().info(f"{self.cycle_count}번째 공정이 완료되었습니다!")
        reached_limit = self.max_cycles > 0 and self.cycle_count >= self.max_cycles
        if not self.repeat_enabled or reached_limit:
            self._transition(ProcessState.COMPLETED, None)
            self.get_logger().info("설정된 전체 공정이 완료되었습니다.")
            return

        self._transition(
            ProcessState.RESETTING,
            float(self.get_parameter("reset_timeout_sec").value),
        )
        self.get_logger().info("다음 Cycle을 위해 시뮬레이션을 초기화합니다...")
        self._publish_bool(self.reset_request_pub, True)

    def _reset_done_callback(self, msg: Bool) -> None:
        if self.state is not ProcessState.RESETTING:
            return
        if not msg.data:
            self._enter_error("Standalone cycle reset 실패")
            return
        self.cycle_count += 1
        self.get_logger().info("시뮬레이션 초기화가 완료되었습니다!")
        self._command_station1()

    def _sim_fault_callback(self, msg: String) -> None:
        message = msg.data.strip()
        if message:
            self._enter_error(f"Standalone FAULT: {message}")

    def _command_station1(self) -> None:
        self.get_logger().info(
            f"{self.cycle_count}번째 어망이 1번 공정 구역으로 진입하는 중..."
        )
        self._command_net(self.station1_pose, ProcessState.MOVING_STATION1)

    def _command_net(self, pose: Pose, state: ProcessState) -> None:
        timeout = self._net_timeout_for(pose)
        self._transition(state, timeout)
        self.target_pose_pub.publish(pose)

    def _net_timeout_for(self, target: Pose) -> float:
        minimum = float(self.get_parameter("minimum_net_timeout_sec").value)
        scale = float(self.get_parameter("net_timeout_scale").value)
        margin = float(self.get_parameter("net_timeout_margin_sec").value)
        if self._current_pose is None:
            return minimum
        distance = self._distance(self._current_pose, target)
        expected = distance / self.expected_net_speed
        return max(minimum, expected * scale + margin)

    def _watchdog(self) -> None:
        if self._state_timeout_sec is None:
            return
        if self.state in {ProcessState.ERROR, ProcessState.COMPLETED}:
            return
        elapsed = time.monotonic() - self._state_started_wall
        if elapsed <= self._state_timeout_sec:
            return
        self._enter_error(
            f"State timeout: {self.state.name}, "
            f"elapsed={elapsed:.1f}s, limit={self._state_timeout_sec:.1f}s"
        )

    def _transition(
        self,
        state: ProcessState,
        timeout_sec: Optional[float],
    ) -> None:
        self.state = state
        self._state_started_wall = time.monotonic()
        self._state_timeout_sec = timeout_sec
        self._publish_process_state()

    def _enter_error(self, message: str) -> None:
        if self.state is ProcessState.ERROR:
            return
        self._transition(ProcessState.ERROR, None)
        self.get_logger().error(message)
        fault = String()
        fault.data = message
        self.process_fault_pub.publish(fault)

    def _publish_process_state(self) -> None:
        msg = String()
        msg.data = f"{self.state.name}:cycle={self.cycle_count}"
        self.process_state_pub.publish(msg)

    @staticmethod
    def _publish_bool(publisher, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)

    def _is_arrived(self, current: Pose, target: Pose) -> bool:
        return self._distance(current, target) <= self.position_tolerance

    @staticmethod
    def _distance(current: Pose, target: Pose) -> float:
        dx = current.position.x - target.position.x
        dy = current.position.y - target.position.y
        dz = current.position.z - target.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
