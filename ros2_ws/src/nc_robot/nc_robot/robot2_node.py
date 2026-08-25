#!/usr/bin/env python3
"""NetClean Robot 2 객체 제거 상태 머신.

Vision 2 -> /remove/execute (ExecuteRemove Action)
Robot 2 -> /robot2/motion_command (PoseStamped) -> Standalone
Standalone -> /robot2/motion_done (Bool) -> Robot 2
Robot 2 -> /robot2/suction_command (Bool) -> Standalone
Standalone -> /robot2/suction_state (Bool) -> Robot 2
Robot 2 -> /robot2/release_attached_object (String) -> Standalone
Standalone -> /robot2/joint_release_done (Bool) -> Robot 2

한 번 받은 전체 객체를 어망 왼쪽에서 오른쪽 순서로 처리한다.
객체 하나의 순서는 다음과 같다.

Pre-grasp -> 잠시 정지 -> Contact -> 흡착 ON -> Fixed Joint 해제
-> Retreat -> Drop -> 흡착 OFF

전체 객체가 끝난 뒤 Home으로 복귀하고 Action Result를 반환한다.
잔여 객체 재촬영과 /station2/complete 발행은 Vision 2가 담당한다.
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
from nc_interfaces.action import ExecuteRemove
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

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


# 1) Camera 2 World Pose -------------------------------------------------------
# Isaac Sim: Stage에서 Camera 2 Prim 선택 -> Window > Script Editor
# 아래 스크립트에서 PRIM_PATH만 실제 Camera 2 경로로 바꿔 실행하세요.
#
# from omni.usd import get_context
# from pxr import UsdGeom
# PRIM_PATH = "/World/실제/Camera2/Prim/경로"
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

CAMERA2_WORLD_POSITION = (0.0, 0.0, 0.0)  # TODO: Camera 2 World XYZ [m]
CAMERA2_PRIM_WORLD_QUATERNION_XYZW = (
    0.0, 0.0, 0.0, 1.0,
)  # TODO: Camera 2 World Quaternion [x, y, z, w]

# Vision 2 position이 ROS optical frame(+X 오른쪽,+Y 아래,+Z 전방)이면 True.
VISION_USES_ROS_OPTICAL_FRAME = True


# 2) 어망 방향 ---------------------------------------------------------------
# 첫 번째 벡터: 어망 왼쪽 -> 오른쪽 World 방향(객체 처리 순서에 사용)
# 두 번째 벡터: 어망 표면 -> Robot 2 World 방향(Pre-grasp/Retreat에 사용)
# 정확히 단위 벡터가 아니어도 코드가 자동 정규화합니다.

NET_LEFT_TO_RIGHT_AXIS_WORLD = (
    1.0, 0.0, 0.0,
)  # TODO: 실제 어망 왼쪽->오른쪽 World 방향

NET_NORMAL_TOWARD_ROBOT2_WORLD = (
    0.0, 1.0, 0.0,
)  # TODO: 실제 어망 표면->Robot 2 World 방향


# 3) Surface Gripper TCP Contact 자세 및 보정 --------------------------------
# Robot 2를 수동으로 정상 Contact 자세에 놓고, Standalone/Lula가 사용하는
# end_effector_frame(TCP) Prim의 World Quaternion을 위 스크립트로 출력하세요.

GRIPPER_CONTACT_QUATERNION_XYZW = (
    0.0, 1.0, 0.0, 0.0,
)  # TODO: Gripper Contact TCP World Quaternion [x, y, z, w]

# Vision 2가 보내는 물체 중심과 실제 Surface Gripper TCP가 일치하면 0 유지.
GRASP_POINT_OFFSET_WORLD = (
    0.0, 0.0, 0.0,
)  # TODO: 필요할 때만 물체 중심->TCP World XYZ 보정 [m]


# 4) 컨베이어 Drop Pose -------------------------------------------------------
# Robot 2가 쓰레기를 놓을 컨베이어 위의 TCP World Pose입니다.
# 그리퍼를 OFF했을 때 객체가 컨베이어 위로 안전하게 떨어지는 위치를 입력하세요.

DROP_POSITION = (
    0.80, 0.00, 0.50,
)  # TODO: Drop TCP World XYZ [m]

DROP_QUATERNION_XYZW = (
    0.0, 1.0, 0.0, 0.0,
)  # TODO: Drop TCP World Quaternion [x, y, z, w]


# 5) 전체 작업 후 Robot 2 Home/Safe Pose ------------------------------------
# 충돌 없는 대기 자세로 수동 이동한 뒤 같은 TCP Prim의 World Pose를 출력합니다.

ROBOT2_HOME_POSITION = (
    0.40, 0.60, 0.80,
)  # TODO: Robot 2 Home TCP World XYZ [m]

ROBOT2_HOME_QUATERNION_XYZW = (
    0.0, 1.0, 0.0, 0.0,
)  # TODO: Robot 2 Home TCP World Quaternion [x, y, z, w]


# =============================================================================
# 일반 동작 설정값
# =============================================================================

ACTION_NAME = '/remove/execute'

MOTION_COMMAND_TOPIC = '/robot2/motion_command'
MOTION_DONE_TOPIC = '/robot2/motion_done'

SUCTION_COMMAND_TOPIC = '/robot2/suction_command'
SUCTION_STATE_TOPIC = '/robot2/suction_state'

RELEASE_OBJECT_TOPIC = '/robot2/release_attached_object'
JOINT_RELEASE_DONE_TOPIC = '/robot2/joint_release_done'

CAMERA_FRAME = 'camera2_optical_frame'
WORLD_FRAME = 'world'

VALID_CLASSES = {'plastic_bottle', 'can', 'buoy'}

PREGRASP_DISTANCE_M = 0.12
RETREAT_DISTANCE_M = 0.15
PREGRASP_HOLD_SEC = 0.50
CONTACT_HOLD_SEC = 0.25

MOTION_TIMEOUT_SEC = 15.0
SUCTION_TIMEOUT_SEC = 5.0
JOINT_RELEASE_TIMEOUT_SEC = 5.0


class Robot2State(Enum):
    IDLE = auto()
    PREGRASP = auto()
    PREGRASP_HOLD = auto()
    CONTACT = auto()
    CONTACT_HOLD = auto()
    SUCTION_ON = auto()
    JOINT_RELEASE = auto()
    RETREAT = auto()
    DROP = auto()
    SUCTION_OFF = auto()
    HOME = auto()


@dataclass(frozen=True)
class PreparedTarget:
    object_id: str
    class_name: str
    contact_world: Point
    fixed_joint_path: str
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
    return all(math.isfinite(value) for value in (point.x, point.y, point.z))


def _point_from_xyz(values: Sequence[float]) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(value) for value in values)
    return point


class Robot2Node(Node):
    """한 어망의 전체 객체를 순차 제거하는 Robot 2 Action Server."""

    def __init__(self) -> None:
        super().__init__('robot2_node')

        self.state = Robot2State.IDLE
        self._callback_group = ReentrantCallbackGroup()
        self._busy_lock = threading.Lock()
        self._busy = False

        self._motion_lock = threading.Lock()
        self._motion_event = threading.Event()
        self._motion_waiting = False
        self._motion_result: Optional[bool] = None

        self._suction_lock = threading.Lock()
        self._suction_event = threading.Event()
        self._suction_waiting = False
        self._expected_suction_state: Optional[bool] = None

        self._joint_lock = threading.Lock()
        self._joint_event = threading.Event()
        self._joint_waiting = False
        self._joint_result: Optional[bool] = None

        self._left_to_right_axis = _normalized_vector(
            NET_LEFT_TO_RIGHT_AXIS_WORLD,
            'NET_LEFT_TO_RIGHT_AXIS_WORLD',
        )
        self._net_normal = _normalized_vector(
            NET_NORMAL_TOWARD_ROBOT2_WORLD,
            'NET_NORMAL_TOWARD_ROBOT2_WORLD',
        )
        self._gripper_quaternion = normalize_quaternion_xyzw(
            GRIPPER_CONTACT_QUATERNION_XYZW
        )
        self._drop_quaternion = normalize_quaternion_xyzw(
            DROP_QUATERNION_XYZW
        )
        self._home_quaternion = normalize_quaternion_xyzw(
            ROBOT2_HOME_QUATERNION_XYZW
        )

        # USD Camera local(+X 오른쪽,+Y 위,-Z 전방)과 ROS optical
        # (+X 오른쪽,+Y 아래,+Z 전방)의 축 차이를 자동 보정합니다.
        t_world_camera_prim = make_transform_matrix(
            CAMERA2_WORLD_POSITION,
            CAMERA2_PRIM_WORLD_QUATERNION_XYZW,
        )
        if VISION_USES_ROS_OPTICAL_FRAME:
            optical_to_usd = np.diag([1.0, -1.0, -1.0, 1.0])
            self._t_world_camera = t_world_camera_prim @ optical_to_usd
        else:
            self._t_world_camera = t_world_camera_prim

        self._motion_command_pub = self.create_publisher(
            PoseStamped, MOTION_COMMAND_TOPIC, 10
        )
        self._suction_command_pub = self.create_publisher(
            Bool, SUCTION_COMMAND_TOPIC, 10
        )
        self._release_object_pub = self.create_publisher(
            String, RELEASE_OBJECT_TOPIC, 10
        )

        self._motion_done_sub = self.create_subscription(
            Bool,
            MOTION_DONE_TOPIC,
            self._motion_done_callback,
            10,
            callback_group=self._callback_group,
        )
        self._suction_state_sub = self.create_subscription(
            Bool,
            SUCTION_STATE_TOPIC,
            self._suction_state_callback,
            10,
            callback_group=self._callback_group,
        )
        self._joint_done_sub = self.create_subscription(
            Bool,
            JOINT_RELEASE_DONE_TOPIC,
            self._joint_release_done_callback,
            10,
            callback_group=self._callback_group,
        )

        self._action_server = ActionServer(
            self,
            ExecuteRemove,
            ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info('NetClean Robot 2 node started')
        if not CALIBRATION_READY:
            self.get_logger().warn(
                'CALIBRATION_READY=False: TODO 값을 입력하기 전에는 Goal을 거절합니다.'
            )

    # -------------------------------------------------------------------------
    # Action Goal 검사
    # -------------------------------------------------------------------------

    def _goal_callback(self, goal: ExecuteRemove.Goal) -> GoalResponse:
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

        object_ids = set()
        for target in goal.targets:
            object_id = target.object_id.strip()
            class_name = target.class_name.strip().lower()

            if not object_id or object_id in object_ids:
                self.get_logger().warn(
                    f'Goal rejected: 비어 있거나 중복된 object_id={object_id!r}'
                )
                return GoalResponse.REJECT
            object_ids.add(object_id)

            if class_name not in VALID_CLASSES:
                self.get_logger().warn(
                    f'Goal rejected: 지원하지 않는 class_name={class_name!r}'
                )
                return GoalResponse.REJECT
            if not _point_is_finite(target.position):
                self.get_logger().warn(
                    f'Goal rejected: {object_id} position이 잘못되었습니다.'
                )
                return GoalResponse.REJECT

        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('Goal rejected: Robot 2 작업 중')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        # 현재 통신에는 Standalone 동작/흡착을 원자적으로 취소하는 명령이 없습니다.
        self.get_logger().warn('Cancel rejected: 긴급 정지는 Standalone에서 처리하세요.')
        return CancelResponse.REJECT

    # -------------------------------------------------------------------------
    # 전체 어망 제거 Action 실행
    # -------------------------------------------------------------------------

    def _execute_callback(self, goal_handle) -> ExecuteRemove.Result:
        result = ExecuteRemove.Result()
        try:
            targets = self._prepare_targets(goal_handle.request)
            total_targets = len(targets)
            self.get_logger().info(
                '왼쪽->오른쪽 제거 순서: '
                + ' -> '.join(target.object_id for target in targets)
            )

            for current_target, target in enumerate(targets, start=1):
                if not self._execute_one_target(
                    goal_handle,
                    target,
                    current_target,
                    total_targets,
                ):
                    return self._abort(
                        goal_handle,
                        result,
                        f'{target.object_id} 제거 실패',
                    )

            self.state = Robot2State.HOME
            self._feedback(
                goal_handle,
                total_targets,
                total_targets,
                'home',
                'HOME',
            )
            if not self._send_motion_and_wait(self._home_pose(), 'HOME'):
                return self._abort(goal_handle, result, 'Robot 2 Home 복귀 실패')

            goal_handle.succeed()
            result.success = True
            result.message = f'{total_targets}개 객체 제거 완료'
            self.get_logger().info(result.message)
            return result
        except Exception as error:
            self.get_logger().error(f'Robot 2 실행 오류: {error}')
            goal_handle.abort()
            result.success = False
            result.message = str(error)
            return result
        finally:
            self.state = Robot2State.IDLE
            with self._busy_lock:
                self._busy = False

    def _prepare_targets(
        self,
        goal: ExecuteRemove.Goal,
    ) -> list[PreparedTarget]:
        source_frame = goal.header.frame_id or CAMERA_FRAME
        prepared = []

        for target in goal.targets:
            if source_frame == CAMERA_FRAME:
                position_world = transform_point(
                    target.position,
                    self._t_world_camera,
                )
            else:
                position_world = _point_from_xyz((
                    target.position.x,
                    target.position.y,
                    target.position.z,
                ))

            contact = add_tool_offset(
                position_world,
                GRASP_POINT_OFFSET_WORLD,
            )
            contact_array = np.array(
                [contact.x, contact.y, contact.z],
                dtype=float,
            )

            fixed_joint_path = target.fixed_joint_path.strip()
            if fixed_joint_path:
                self.get_logger().info(
                    f'{target.object_id}: fixed_joint_path 필드는 보존하지만 '
                    '현재 구조에서는 Standalone이 class_name으로 경로를 조회합니다.'
                )

            prepared.append(PreparedTarget(
                object_id=target.object_id.strip(),
                class_name=target.class_name.strip().lower(),
                contact_world=contact,
                fixed_joint_path=fixed_joint_path,
                left_to_right_score=float(
                    np.dot(contact_array, self._left_to_right_axis)
                ),
            ))

        prepared.sort(
            key=lambda target: (
                target.left_to_right_score,
                target.object_id,
            )
        )
        return prepared

    # -------------------------------------------------------------------------
    # 객체 하나 제거
    # -------------------------------------------------------------------------

    def _execute_one_target(
        self,
        goal_handle,
        target: PreparedTarget,
        current_target: int,
        total_targets: int,
    ) -> bool:
        pregrasp = offset_point(
            target.contact_world,
            self._net_normal,
            PREGRASP_DISTANCE_M,
        )
        retreat = offset_point(
            target.contact_world,
            self._net_normal,
            RETREAT_DISTANCE_M,
        )

        self.state = Robot2State.PREGRASP
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'PREGRASP'
        )
        if not self._send_motion_and_wait(
            self._gripper_pose(pregrasp),
            f'{target.object_id}:PREGRASP',
        ):
            return False

        self.state = Robot2State.PREGRASP_HOLD
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'PREGRASP_HOLD'
        )
        time.sleep(PREGRASP_HOLD_SEC)

        self.state = Robot2State.CONTACT
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'CONTACT'
        )
        if not self._send_motion_and_wait(
            self._gripper_pose(target.contact_world),
            f'{target.object_id}:CONTACT',
        ):
            return False

        self.state = Robot2State.CONTACT_HOLD
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'CONTACT_HOLD'
        )
        time.sleep(CONTACT_HOLD_SEC)

        self.state = Robot2State.SUCTION_ON
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'SUCTION_ON'
        )
        if not self._send_suction_and_wait(True):
            return False

        # Surface Gripper 흡착이 확인된 뒤에만 어망-쓰레기 Fixed Joint를 해제합니다.
        self.state = Robot2State.JOINT_RELEASE
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'JOINT_RELEASE'
        )
        if not self._release_joint_and_wait(target.class_name):
            return False

        self.state = Robot2State.RETREAT
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'RETREAT'
        )
        if not self._send_motion_and_wait(
            self._gripper_pose(retreat),
            f'{target.object_id}:RETREAT',
        ):
            return False

        self.state = Robot2State.DROP
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'DROP'
        )
        if not self._send_motion_and_wait(
            self._drop_pose(),
            f'{target.object_id}:DROP',
        ):
            return False

        self.state = Robot2State.SUCTION_OFF
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'SUCTION_OFF'
        )
        return self._send_suction_and_wait(False)

    # -------------------------------------------------------------------------
    # Standalone 명령 및 응답
    # -------------------------------------------------------------------------

    def _gripper_pose(self, point: Point) -> PoseStamped:
        return make_pose_stamped(
            self,
            point,
            self._gripper_quaternion,
            WORLD_FRAME,
        )

    def _drop_pose(self) -> PoseStamped:
        return make_pose_stamped(
            self,
            _point_from_xyz(DROP_POSITION),
            self._drop_quaternion,
            WORLD_FRAME,
        )

    def _home_pose(self) -> PoseStamped:
        return make_pose_stamped(
            self,
            _point_from_xyz(ROBOT2_HOME_POSITION),
            self._home_quaternion,
            WORLD_FRAME,
        )

    def _send_motion_and_wait(self, pose: PoseStamped, label: str) -> bool:
        with self._motion_lock:
            self._motion_event.clear()
            self._motion_result = None
            self._motion_waiting = True

        self._motion_command_pub.publish(pose)
        self.get_logger().info(
            f'Motion [{label}]: '
            f'({pose.pose.position.x:.3f}, '
            f'{pose.pose.position.y:.3f}, '
            f'{pose.pose.position.z:.3f})'
        )

        if not self._motion_event.wait(timeout=MOTION_TIMEOUT_SEC):
            with self._motion_lock:
                self._motion_waiting = False
            self.get_logger().error(f'Motion timeout: {label}')
            return False

        with self._motion_lock:
            success = bool(self._motion_result)
            self._motion_waiting = False
        return success

    def _motion_done_callback(self, msg: Bool) -> None:
        with self._motion_lock:
            if not self._motion_waiting or self._motion_result is not None:
                return
            self._motion_result = bool(msg.data)
            self._motion_event.set()

    def _send_suction_and_wait(self, enabled: bool) -> bool:
        with self._suction_lock:
            self._suction_event.clear()
            self._expected_suction_state = enabled
            self._suction_waiting = True

        command = Bool()
        command.data = enabled
        self._suction_command_pub.publish(command)
        self.get_logger().info(f'Suction command: {enabled}')

        if not self._suction_event.wait(timeout=SUCTION_TIMEOUT_SEC):
            with self._suction_lock:
                self._suction_waiting = False
                self._expected_suction_state = None
            self.get_logger().error(
                f'Suction state timeout: expected={enabled}'
            )
            return False

        with self._suction_lock:
            self._suction_waiting = False
            self._expected_suction_state = None
        return True

    def _suction_state_callback(self, msg: Bool) -> None:
        with self._suction_lock:
            if not self._suction_waiting:
                return
            if msg.data != self._expected_suction_state:
                return
            self._suction_event.set()

    def _release_joint_and_wait(self, class_name: str) -> bool:
        with self._joint_lock:
            self._joint_event.clear()
            self._joint_result = None
            self._joint_waiting = True

        request = String()
        request.data = class_name
        self._release_object_pub.publish(request)
        self.get_logger().info(
            f'Fixed Joint release request: class_name={class_name}'
        )

        if not self._joint_event.wait(timeout=JOINT_RELEASE_TIMEOUT_SEC):
            with self._joint_lock:
                self._joint_waiting = False
            self.get_logger().error(
                f'Fixed Joint release timeout: class_name={class_name}'
            )
            return False

        with self._joint_lock:
            success = bool(self._joint_result)
            self._joint_waiting = False
        return success

    def _joint_release_done_callback(self, msg: Bool) -> None:
        with self._joint_lock:
            if not self._joint_waiting or self._joint_result is not None:
                return
            self._joint_result = bool(msg.data)
            self._joint_event.set()

    # -------------------------------------------------------------------------
    # Action Feedback/Result
    # -------------------------------------------------------------------------

    def _feedback_for_target(
        self,
        goal_handle,
        target: PreparedTarget,
        current_target: int,
        total_targets: int,
        state: str,
    ) -> None:
        self._feedback(
            goal_handle,
            current_target,
            total_targets,
            target.object_id,
            state,
        )

    def _feedback(
        self,
        goal_handle,
        current_target: int,
        total_targets: int,
        current_object_id: str,
        state: str,
    ) -> None:
        feedback = ExecuteRemove.Feedback()
        feedback.current_target = current_target
        feedback.total_targets = total_targets
        feedback.current_object_id = current_object_id
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(
            f'[{current_target}/{total_targets}] '
            f'{current_object_id}: {state}'
        )

    def _abort(
        self,
        goal_handle,
        result: ExecuteRemove.Result,
        message: str,
    ) -> ExecuteRemove.Result:
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
    node = Robot2Node()
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