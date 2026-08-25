#!/usr/bin/env python3
"""NetClean 전체 공정과 어망 이송을 관리하는 ROS 2 노드."""

from enum import Enum, auto
import math

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from std_msgs.msg import Bool


class ProcessState(Enum):
    """control_node가 관리하는 상위 공정 상태."""

    WAIT_SIM = auto()
    MOVING_STATION1 = auto()
    WAIT_STATION1_COMPLETE = auto()
    MOVING_STATION2 = auto()
    WAIT_STATION2_COMPLETE = auto()
    MOVING_EXIT = auto()
    COMPLETED = auto()


class ControlNode(Node):
    """어망을 Station 1, Station 2, EXIT 순서로 보내는 상태 머신."""

    def __init__(self) -> None:
        super().__init__('control_node')

        # 실제 월드 좌표가 확정되면 이 기본값을 수정하거나 launch/YAML에서 덮어씁니다.
        self.declare_parameter('station1_x', 1.0)
        self.declare_parameter('station1_y', 0.0)
        self.declare_parameter('station1_z', 1.0)

        self.declare_parameter('station2_x', 2.0)
        self.declare_parameter('station2_y', 0.0)
        self.declare_parameter('station2_z', 1.0)

        self.declare_parameter('exit_x', 3.0)
        self.declare_parameter('exit_y', 0.0)
        self.declare_parameter('exit_z', 1.0)

        self.declare_parameter('position_tolerance', 0.02)

        self.station1_pose = self._make_pose_from_parameters('station1')
        self.station2_pose = self._make_pose_from_parameters('station2')
        self.exit_pose = self._make_pose_from_parameters('exit')
        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )

        if self.position_tolerance <= 0.0:
            raise ValueError('position_tolerance은 0보다 커야 합니다.')

        # control_node -> standalone.py
        self.target_pose_pub = self.create_publisher(
            Pose,
            '/net/target_pose',
            10,
        )

        # control_node -> vision1_node / vision2_node
        self.station1_arrived_pub = self.create_publisher(
            Bool,
            '/station1/net_arrived',
            10,
        )
        self.station2_arrived_pub = self.create_publisher(
            Bool,
            '/station2/net_arrived',
            10,
        )

        # standalone.py -> control_node
        self.sim_ready_sub = self.create_subscription(
            Bool,
            '/sim/ready',
            self._sim_ready_callback,
            10,
        )
        self.current_pose_sub = self.create_subscription(
            Pose,
            '/net/current_pose',
            self._current_pose_callback,
            10,
        )

        # robot1_node -> control_node
        self.cut_complete_sub = self.create_subscription(
            Bool,
            '/station1/cut_complete',
            self._cut_complete_callback,
            10,
        )

        # vision2_node -> control_node
        self.station2_complete_sub = self.create_subscription(
            Bool,
            '/station2/complete',
            self._station2_complete_callback,
            10,
        )

        self.state = ProcessState.WAIT_SIM

        self.get_logger().info('어망 분리 공정을 시작합니다!')
        self.get_logger().info('어망 기다리는 중...')

    def _make_pose_from_parameters(self, prefix: str) -> Pose:
        """station1_x 같은 ROS 파라미터로 목표 Pose를 만듭니다."""
        pose = Pose()
        pose.position.x = float(self.get_parameter(f'{prefix}_x').value)
        pose.position.y = float(self.get_parameter(f'{prefix}_y').value)
        pose.position.z = float(self.get_parameter(f'{prefix}_z').value)

        # 어망 자세는 바꾸지 않고 기본 단위 쿼터니언을 사용합니다.
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def _sim_ready_callback(self, msg: Bool) -> None:
        """Isaac Sim 준비가 끝나면 첫 번째 공정 구역으로 보냅니다."""
        if not msg.data or self.state is not ProcessState.WAIT_SIM:
            return

        self.get_logger().info('어망이 1번 공정 구역으로 진입하는 중...')
        self._publish_target_pose(self.station1_pose)
        self.state = ProcessState.MOVING_STATION1

    def _current_pose_callback(self, msg: Pose) -> None:
        """어망의 현재 위치를 확인하여 각 구역 도착을 판정합니다."""
        if self.state is ProcessState.MOVING_STATION1:
            if not self._is_arrived(msg, self.station1_pose):
                return

            self.state = ProcessState.WAIT_STATION1_COMPLETE
            self.get_logger().info('어망이 1번 공정 구역에 도착했습니다!')
            self._publish_bool(self.station1_arrived_pub, True)
            return

        if self.state is ProcessState.MOVING_STATION2:
            if not self._is_arrived(msg, self.station2_pose):
                return

            self.state = ProcessState.WAIT_STATION2_COMPLETE
            self.get_logger().info('어망이 2번 공정 구역에 도착했습니다!')
            self._publish_bool(self.station2_arrived_pub, True)
            return

        if self.state is ProcessState.MOVING_EXIT:
            if self._is_arrived(msg, self.exit_pose):
                self.state = ProcessState.COMPLETED

    def _cut_complete_callback(self, msg: Bool) -> None:
        """Robot 1의 절단 완료 신호를 받으면 Station 2로 이동합니다."""
        if (
            not msg.data
            or self.state is not ProcessState.WAIT_STATION1_COMPLETE
        ):
            return

        self.get_logger().info('어망이 2번 공정 구역으로 진입하는 중...')
        self._publish_target_pose(self.station2_pose)
        self.state = ProcessState.MOVING_STATION2

    def _station2_complete_callback(self, msg: Bool) -> None:
        """Vision 2의 최종 재검사 완료 후 어망을 EXIT로 보냅니다."""
        if (
            not msg.data
            or self.state is not ProcessState.WAIT_STATION2_COMPLETE
        ):
            return

        self.get_logger().info(
            '어망 분리 공정이 종료되었습니다! 다음 공정으로 이동합니다.'
        )
        self._publish_target_pose(self.exit_pose)
        self.state = ProcessState.MOVING_EXIT

    def _publish_target_pose(self, target_pose: Pose) -> None:
        """Standalone이 실제 어망을 움직일 목표 위치를 발행합니다."""
        self.target_pose_pub.publish(target_pose)

    @staticmethod
    def _publish_bool(publisher, value: bool) -> None:
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def _is_arrived(self, current: Pose, target: Pose) -> bool:
        """현재 위치와 목표 위치의 3차원 거리를 비교합니다."""
        dx = current.position.x - target.position.x
        dy = current.position.y - target.position.y
        dz = current.position.z - target.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance <= self.position_tolerance


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


if __name__ == '__main__':
    main()