#!/usr/bin/env python3
"""NetClean Robot 1 절단 상태 머신.

Vision 1 -> /cut/execute (ExecuteCut Action)
Robot 1 -> /robot1/motion_command (PoseStamped) -> Standalone
Standalone -> /robot1/motion_done (Bool) -> Robot 1
Robot 1 -> /station1/cut_complete (Bool) -> Control

각 객체의 P1, P2를 Approach -> Contact -> Hold -> Retreat 순으로 처리한다.
Robot 1은 Fixed Joint 해제, 흡착, 잔여 객체 검사를 수행하지 않는다.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
import threading
import time
from typing import Optional, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nc_interfaces.action import ExecuteCut
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from nc_robot.transform_utils import (
    add_tool_offset,
    make_pose_stamped,
    make_transform_matrix,
    normalize_quaternion_xyzw,
    offset_point,
    transform_point,
)


# =============================================================================
# 실제 Isaac Sim Stage에서 찾아서 입력할 값
# =============================================================================
# 모든 TODO 값을 확인하기 전에는 False로 둡니다. 확인 후 True로 바꾸세요.
# False 상태에서는 예시 좌표로 로봇이 움직이지 않도록 Action Goal을 거절합니다.

CALIBRATION_READY = False


# 1) Camera 1 World Pose -------------------------------------------------------
# Isaac Sim: Stage에서 Camera 1 Prim 선택 -> Window > Script Editor
# 아래 스크립트에서 PRIM_PATH만 실제 Camera 1 경로로 바꿔 실행하세요.
#
# from omni.usd import get_context
# from pxr import UsdGeom
# PRIM_PATH = "/World/실제/Camera1/Prim/경로"
# stage = get_context().get_stage()
# prim = stage.GetPrimAtPath(PRIM_PATH)
# matrix = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
# t = matrix.ExtractTranslation()
# q = matrix.ExtractRotationQuat()
# qi = q.GetImaginary()
# print("position_xyz =", tuple(t))
# print("quaternion_xyzw =", (qi[0], qi[1], qi[2], q.GetReal()))
#
# Property 창 값은 부모가 있으면 Local Pose일 수 있으므로 위 World Pose를 씁니다.

CAMERA1_WORLD_POSITION = (0.0, 0.0, 0.0)  # TODO: Camera 1 World XYZ [m]
CAMERA1_PRIM_WORLD_QUATERNION_XYZW = (
    0.0, 0.0, 0.0, 1.0,
)  # TODO: Camera 1 World Quaternion [x, y, z, w]

# Vision의 점이 ROS optical frame(+X 오른쪽, +Y 아래, +Z 전방)이면 True.
VISION_USES_ROS_OPTICAL_FRAME = True


# 2) 어망 방향 ---------------------------------------------------------------
# 첫 번째 벡터: 어망의 왼쪽 -> 오른쪽 World 방향(객체 처리 순서에 사용)
# 두 번째 벡터: 어망 표면 -> Robot 1 World 방향(Approach/Retreat에 사용)
# 정확히 단위 벡터가 아니어도 코드가 자동 정규화합니다.

NET_LEFT_TO_RIGHT_AXIS_WORLD = (
    1.0, 0.0, 0.0,
)  # TODO: 실제 어망 왼쪽->오른쪽 World 방향

NET_NORMAL_TOWARD_ROBOT1_WORLD = (
    0.0, 1.0, 0.0,
)  # TODO: 실제 어망 표면->Robot 1 World 방향


# 3) 칼날 TCP Contact 자세 및 보정 -------------------------------------------
# Robot 1을 수동으로 정상 Contact 자세에 놓고, Standalone/Lula가 사용하는
# end_effector_frame(TCP) Prim의 World Quaternion을 위 스크립트로 출력하세요.

CUTTER_CONTACT_QUATERNION_XYZW = (
    0.0, 1.0, 0.0, 0.0,
)  # TODO: 칼날 Contact TCP World Quaternion [x, y, z, w]

# 비전 절단점과 실제 칼날 TCP가 일치하면 0을 유지합니다.
CUT_POINT_OFFSET_WORLD = (
    0.0, 0.0, 0.0,
)  # TODO: 필요할 때만 절단점->TCP World XYZ 보정 [m]


# 4) 전체 작업 후 Robot 1 Home/Safe Pose ------------------------------------
# 충돌 없는 대기 자세로 수동 이동한 뒤 같은 TCP Prim의 World Pose를 출력합니다.

ROBOT1_HOME_POSITION = (
    0.40, -0.60, 0.80,
)  # TODO: Robot 1 Home TCP World XYZ [m]

ROBOT1_HOME_QUATERNION_XYZW = (
    0.0, 1.0, 0.0, 0.0,
)  # TODO: Robot 1 Home TCP World Quaternion [x, y, z, w]


# =============================================================================
# 일반 동작 설정값
# =============================================================================

ACTION_NAME = '/cut/execute'
MOTION_COMMAND_TOPIC = '/robot1/motion_command'
MOTION_DONE_TOPIC = '/robot1/motion_done'
STATION1_COMPLETE_TOPIC = '/station1/cut_complete'

CAMERA_FRAME = 'camera1_optical_frame'
WORLD_FRAME = 'world'

# Vision 1과 동일한 클래스 문자열을 사용합니다.
VALID_CLASSES = {'plastic_bottle', 'can', 'buoy'}

APPROACH_DISTANCE_M = 0.08
RETREAT_DISTANCE_M = 0.10
CONTACT_HOLD_SEC = 0.50
MOTION_TIMEOUT_SEC = 15.0


class Robot1State(Enum):
    IDLE = auto()
    APPROACH = auto()
    CONTACT = auto()
    HOLD = auto()
    RETREAT = auto()
    HOME = auto()


@dataclass(frozen=True)
class PreparedTarget:
    name: str
    point1_world: Point
    point2_world: Point
    left_to_right_score: float


def _normalized_vector(values: Sequence[float], name: str) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f'{name}은 유효한 XYZ 세 값이어야 합니다.')
    length = float(np.linalg.norm(vector))
    if length < 1.0e-9:
        raise ValueError(f'{name}은 0 벡터일 수 없습니다.')
    return vector / length


def _point_is_finite(point: Point) -> bool:
    return all(math.isfinite(v) for v in (point.x, point.y, point.z))


def _point_from_xyz(values: Sequence[float]) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(value) for value in values)
    return point


def _target_name(target) -> str:
    """CutTarget.object_id에 담긴 YOLO 클래스명을 정규화한다."""
    return str(target.object_id).strip().lower()


class Robot1Node(Node):
    def __init__(self) -> None:
        super().__init__('robot1_node')

        self.state = Robot1State.IDLE
        self._callback_group = ReentrantCallbackGroup()
        self._busy_lock = threading.Lock()
        self._busy = False

        self._motion_lock = threading.Lock()
        self._motion_event = threading.Event()
        self._waiting_for_motion = False
        self._motion_result: Optional[bool] = None

        self._left_to_right_axis = _normalized_vector(
            NET_LEFT_TO_RIGHT_AXIS_WORLD,
            'NET_LEFT_TO_RIGHT_AXIS_WORLD',
        )
        self._net_normal = _normalized_vector(
            NET_NORMAL_TOWARD_ROBOT1_WORLD,
            'NET_NORMAL_TOWARD_ROBOT1_WORLD',
        )
        self._cutter_quaternion = normalize_quaternion_xyzw(
            CUTTER_CONTACT_QUATERNION_XYZW
        )
        self._home_quaternion = normalize_quaternion_xyzw(
            ROBOT1_HOME_QUATERNION_XYZW
        )

        # USD Camera local(+X 오른쪽,+Y 위,-Z 전방)과 ROS optical
        # (+X 오른쪽,+Y 아래,+Z 전방)의 축 차이를 자동 보정합니다.
        t_world_camera_prim = make_transform_matrix(
            CAMERA1_WORLD_POSITION,
            CAMERA1_PRIM_WORLD_QUATERNION_XYZW,
        )
        if VISION_USES_ROS_OPTICAL_FRAME:
            optical_to_usd = np.diag([1.0, -1.0, -1.0, 1.0])
            self._t_world_camera = t_world_camera_prim @ optical_to_usd
        else:
            self._t_world_camera = t_world_camera_prim

        self._motion_command_pub = self.create_publisher(
            PoseStamped, MOTION_COMMAND_TOPIC, 10
        )
        self._station1_complete_pub = self.create_publisher(
            Bool, STATION1_COMPLETE_TOPIC, 10
        )
        self._motion_done_sub = self.create_subscription(
            Bool,
            MOTION_DONE_TOPIC,
            self._motion_done_callback,
            10,
            callback_group=self._callback_group,
        )
        self._action_server = ActionServer(
            self,
            ExecuteCut,
            ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info('NetClean Robot 1 node started')
        if not CALIBRATION_READY:
            self.get_logger().warn(
                'CALIBRATION_READY=False: TODO 값을 입력하기 전에는 Goal을 거절합니다.'
            )

    def _goal_callback(self, goal: ExecuteCut.Goal) -> GoalResponse:
        if not CALIBRATION_READY:
            self.get_logger().error(
                'Goal rejected: TODO 보정값 입력 후 CALIBRATION_READY=True로 바꾸세요.'
            )
            return GoalResponse.REJECT
        if not goal.targets:
            self.get_logger().warn('Goal rejected: targets가 비어 있습니다.')
            return GoalResponse.REJECT

        source_frame = goal.header.frame_id or CAMERA_FRAME
        if source_frame not in (CAMERA_FRAME, WORLD_FRAME):
            self.get_logger().error(
                f'Goal rejected: 지원하지 않는 frame_id={source_frame!r}'
            )
            return GoalResponse.REJECT

        names = set()
        for target in goal.targets:
            name = _target_name(target)
            if not name or name in names:
                self.get_logger().warn(
                    f'Goal rejected: 비어 있거나 중복된 객체 이름={name!r}'
                )
                return GoalResponse.REJECT
            if name not in VALID_CLASSES:
                self.get_logger().warn(
                    f'Goal rejected: 지원하지 않는 클래스={name!r}'
                )
                return GoalResponse.REJECT
            names.add(name)
            if not _point_is_finite(target.point1):
                self.get_logger().warn(f'Goal rejected: {name} P1 오류')
                return GoalResponse.REJECT
            if not _point_is_finite(target.point2):
                self.get_logger().warn(f'Goal rejected: {name} P2 오류')
                return GoalResponse.REJECT

        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: Robot 1 작업 중')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        # Bool 완료 인터페이스에는 Standalone 정지 승인이 없으므로 안전상 거절.
        self.get_logger().warn('Cancel rejected: 긴급 정지는 Standalone에서 처리하세요.')
        return CancelResponse.REJECT

    def _execute_callback(self, goal_handle) -> ExecuteCut.Result:
        result = ExecuteCut.Result()
        try:
            targets = self._prepare_targets(goal_handle.request)
            total_objects = len(targets)
            total_points = total_objects * 2
            self.get_logger().info(
                '왼쪽->오른쪽 순서: ' + ' -> '.join(t.name for t in targets)
            )

            for object_index, target in enumerate(targets, start=1):
                for index, point in enumerate(
                    (target.point1_world, target.point2_world), start=1
                ):
                    label = f'{target.name}/P{index}'
                    if not self._execute_one_point(
                        goal_handle,
                        label,
                        point,
                        object_index,
                        total_objects,
                        target.name,
                    ):
                        return self._abort(
                            goal_handle, result, f'{label} 절단 동작 실패'
                        )

            self.state = Robot1State.HOME
            self._feedback(
                goal_handle,
                total_objects,
                total_objects,
                'home',
                'HOME',
            )
            if not self._send_motion_and_wait(self._home_pose(), 'HOME'):
                return self._abort(goal_handle, result, 'Robot 1 Home 복귀 실패')

            done = Bool()
            done.data = True
            self._station1_complete_pub.publish(done)

            goal_handle.succeed()
            result.success = True
            result.message = (
                f'{total_objects}개 객체, {total_points}개 절단점 처리 완료'
            )
            self.get_logger().info(result.message)
            self.get_logger().info('/station1/cut_complete=True published')
            return result
        except Exception as error:
            self.get_logger().error(f'Robot 1 실행 오류: {error}')
            goal_handle.abort()
            result.success = False
            result.message = str(error)
            return result
        finally:
            self.state = Robot1State.IDLE
            with self._busy_lock:
                self._busy = False

    def _prepare_targets(self, goal: ExecuteCut.Goal) -> list[PreparedTarget]:
        source_frame = goal.header.frame_id or CAMERA_FRAME
        prepared = []

        for target in goal.targets:
            if source_frame == CAMERA_FRAME:
                p1 = transform_point(target.point1, self._t_world_camera)
                p2 = transform_point(target.point2, self._t_world_camera)
            else:
                p1 = _point_from_xyz((
                    target.point1.x,
                    target.point1.y,
                    target.point1.z,
                ))
                p2 = _point_from_xyz((
                    target.point2.x,
                    target.point2.y,
                    target.point2.z,
                ))

            p1 = add_tool_offset(p1, CUT_POINT_OFFSET_WORLD)
            p2 = add_tool_offset(p2, CUT_POINT_OFFSET_WORLD)
            center = np.array([
                (p1.x + p2.x) * 0.5,
                (p1.y + p2.y) * 0.5,
                (p1.z + p2.z) * 0.5,
            ])
            prepared.append(PreparedTarget(
                name=_target_name(target),
                point1_world=p1,
                point2_world=p2,
                left_to_right_score=float(np.dot(center, self._left_to_right_axis)),
            ))

        prepared.sort(key=lambda item: (item.left_to_right_score, item.name))
        return prepared

    def _execute_one_point(
        self,
        goal_handle,
        label: str,
        contact: Point,
        current_target: int,
        total_targets: int,
        current_object_id: str,
    ) -> bool:
        approach = offset_point(contact, self._net_normal, APPROACH_DISTANCE_M)
        retreat = offset_point(contact, self._net_normal, RETREAT_DISTANCE_M)

        self.state = Robot1State.APPROACH
        self._feedback(
            goal_handle,
            current_target,
            total_targets,
            current_object_id,
            f'{label}:APPROACH',
        )
        if not self._send_motion_and_wait(self._cut_pose(approach), f'{label}:APPROACH'):
            return False

        self.state = Robot1State.CONTACT
        self._feedback(
            goal_handle,
            current_target,
            total_targets,
            current_object_id,
            f'{label}:CONTACT',
        )
        if not self._send_motion_and_wait(self._cut_pose(contact), f'{label}:CONTACT'):
            return False

        self.state = Robot1State.HOLD
        self._feedback(
            goal_handle,
            current_target,
            total_targets,
            current_object_id,
            f'{label}:HOLD',
        )
        time.sleep(CONTACT_HOLD_SEC)

        self.state = Robot1State.RETREAT
        self._feedback(
            goal_handle,
            current_target,
            total_targets,
            current_object_id,
            f'{label}:RETREAT',
        )
        return self._send_motion_and_wait(self._cut_pose(retreat), f'{label}:RETREAT')

    def _cut_pose(self, point: Point) -> PoseStamped:
        return make_pose_stamped(
            self, point, self._cutter_quaternion, WORLD_FRAME
        )

    def _home_pose(self) -> PoseStamped:
        return make_pose_stamped(
            self,
            _point_from_xyz(ROBOT1_HOME_POSITION),
            self._home_quaternion,
            WORLD_FRAME,
        )

    def _send_motion_and_wait(self, pose: PoseStamped, label: str) -> bool:
        with self._motion_lock:
            self._motion_event.clear()
            self._motion_result = None
            self._waiting_for_motion = True

        self._motion_command_pub.publish(pose)
        self.get_logger().info(
            f'Motion [{label}]: '
            f'({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f}, '
            f'{pose.pose.position.z:.3f})'
        )

        if not self._motion_event.wait(timeout=MOTION_TIMEOUT_SEC):
            with self._motion_lock:
                self._waiting_for_motion = False
            self.get_logger().error(f'Motion timeout: {label}')
            return False

        with self._motion_lock:
            success = bool(self._motion_result)
            self._waiting_for_motion = False
        return success

    def _motion_done_callback(self, msg: Bool) -> None:
        with self._motion_lock:
            if not self._waiting_for_motion or self._motion_result is not None:
                return
            self._motion_result = bool(msg.data)
            self._motion_event.set()

    @staticmethod
    def _feedback(
        goal_handle,
        current_target: int,
        total_targets: int,
        current_object_id: str,
        _state: str,
    ) -> None:
        feedback = ExecuteCut.Feedback()
        feedback.current_target = current_target
        feedback.total_targets = total_targets
        feedback.current_object_id = current_object_id
        goal_handle.publish_feedback(feedback)

    def _abort(
        self,
        goal_handle,
        result: ExecuteCut.Result,
        message: str,
    ) -> ExecuteCut.Result:
        goal_handle.abort()
        result.success = False
        result.message = message
        self.get_logger().error(message)
        return result

    def destroy_node(self) -> bool:
        self._action_server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Robot1Node()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()