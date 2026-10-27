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
-> Retreat -> 흡착 OFF

전체 객체가 끝난 뒤 Home으로 복귀하고 Action Result를 반환한다.
잔여 객체 재촬영과 /station2/complete 발행은 Vision 2가 담당한다.

v2 변경 사항 (로봇1과 동일한 패턴 적용, 토픽/메시지 타입은 변경 없음)
------------------------------------------------------------------
CONTACT 실패 시 재시도 없이 곧바로 전체 Action을 abort하던 것을,
로봇1처럼 "RETREAT(안전 후퇴) -> RECOVERY_HOME -> PREGRASP(재진입) -> CONTACT 재시도"
순서로 한 번 더 기회를 주도록 변경. RECOVERY_HOME까지 실패하면 안전 재후퇴를
한 번 더 시도하고, 그래도 실패하면 Robot2HomeRecoveryFailure를 발생시켜 명확한
사유와 함께 전체 Action을 abort한다. (로봇2는 원래도 실패 시 전체 abort 정책이라
최종 결과는 동일하지만, 포기 전 안전 경로 재시도 기회와 더 명확한 실패 사유를 남긴다.)
모션 실패 시 목표 pose 좌표를 로그에 남기도록 _send_motion_and_wait()도 함께 수정.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
import math
import threading
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

from nc_robot.motion_utils import sleep_sim_time
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

CALIBRATION_READY = True


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

CAMERA2_WORLD_POSITION = (-0.1, 2.0, 1.2)  # Camera 2 World XYZ [m]
CAMERA2_PRIM_WORLD_QUATERNION_XYZW = (
    0.5, 0.5, 0.4999994933605194, 0.5,
)  # Camera 2 World Quaternion [x, y, z, w]

# Vision 2 position이 ROS optical frame(+X 오른쪽,+Y 아래,+Z 전방)이면 True.
VISION_USES_ROS_OPTICAL_FRAME = True


# 2) 어망 방향 ---------------------------------------------------------------
# 첫 번째 벡터: 어망 왼쪽 -> 오른쪽 World 방향(객체 처리 순서에 사용)
# 두 번째 벡터: 어망 표면 -> Robot 2 World 방향(Pre-grasp/Retreat에 사용)
# 정확히 단위 벡터가 아니어도 코드가 자동 정규화합니다.

NET_LEFT_TO_RIGHT_AXIS_WORLD = (
    0.0, 1.0, 0.0,
)  # 어망 왼쪽->오른쪽 World 방향 (+Y)

NET_NORMAL_TOWARD_ROBOT2_WORLD = (
    1.0, 0.0, 0.0,
)  # 어망 표면->Robot 2 World 방향 (+X)


# 3) Surface Gripper TCP Contact 자세 및 보정 --------------------------------
# Robot 2를 수동으로 정상 Contact 자세에 놓고, Standalone/Lula가 사용하는
# end_effector_frame(TCP) Prim의 World Quaternion을 위 스크립트로 출력하세요.

GRIPPER_CONTACT_QUATERNION_XYZW = (
    -0.006937357587962067,
    -0.6960032573162326,
    -0.021696120472133148,
     0.7176772375039838,
)  # Gripper Contact TCP World Quaternion [x, y, z, w]

# Vision 2는 Depth 표면점을 camera optical frame에서 구한 뒤 아래의 정확한
# Camera->World 변환을 거친다. 따라서 별도의 실측 잔차가 확인되지 않은 상태에서는
# 반드시 0을 유지한다. 기존 임시값 (0.0, -0.23, 0.26)은 목표를 약 34.7cm 옮겨
# PREGRASP를 M0609 작업공간 밖으로 밀어내며 robot2 Lula IK 실패를 일으켰다.
# 향후 실제 TCP-표면 잔차를 측정했을 때만 수 mm 단위로 보정한다.
GRASP_POINT_OFFSET_WORLD = (
    0.0,
    0.0,
    0.0,
)  # 필요할 때만 물체 중심->TCP World XYZ 보정 [m]




# 5) 전체 작업 후 Robot 2 Home/Safe Pose ------------------------------------
# 충돌 없는 대기 자세로 수동 이동한 뒤 같은 TCP Prim의 World Pose를 출력합니다.

ROBOT2_HOME_POSITION = (
    -0.007563448984626552,
    2.4009529933486675,
    1.2495370123800529,
)  # Robot 2 Home TCP World XYZ [m]

ROBOT2_HOME_QUATERNION_XYZW = (
    -0.006190069020578589,
    -0.006658029743382334,
    0.7083097101155121,
    -0.7058432603925151,
)  # Robot 2 Home TCP World Quaternion [x, y, z, w]


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
# 대기(HOLD)는 시뮬 시간 기준이다(sleep_sim_time). 벽시계 time.sleep 이면 시뮬이 느린
# PC·무거운 장면에서 같은 대기 동안 흐르는 시뮬 시간이 달라져 결과가 바뀌었다.

# CONTACT 는 비전 표면점에서 어망 법선(로봇 쪽)으로 이만큼 앞에 멈춘다.
# 표면점은 물체 중앙의 깊이라, 페트병은 가장 튀어나온 면(AABB 앞면)이 표면점보다
# 8.5 mm 앞에 있다. 표면점까지 밀고 들어가면 흡착컵이 병에 먼저 닿고, 강성이 낮은
# 1축(160)이 밀려 TCP 가 옆으로 4-7 cm 미끄러져 흡착 판정(3 cm)에서 거부됐다
# (docs/TROUBLESHOOTING.md #7). 흡착 판정은 AABB 거리라 1.2 cm 앞에서도 통과한다.
CONTACT_STANDOFF_M = 0.012

MOTION_TIMEOUT_SEC = 70.0
SUCTION_TIMEOUT_SEC = 5.0
JOINT_RELEASE_TIMEOUT_SEC = 5.0

# 로봇 이동 없이 물체를 TCP로 순간이동시키는 진단 모드는 사용하지 않는다.
# 최종 공정은 반드시 PREGRASP -> CONTACT -> 흡착 순서로 로봇이 직접 이동한다.
DIRECT_SUCTION_ONLY_MODE = False
DIRECT_SUCTION_HOLD_SEC = 1.0

# 현재 USD는 랜덤 스폰을 사용하지 않는다. 아래 좌표는 실제 실행 로그의
# DIRECT SNAP에서 역산한 "이동 전 물체 AABB의 가장 가까운 표면점"이다.
# Vision2 Camera->World 결과는 plastic_bottle에서 실제 표면보다 World Y가
# 약 -0.285m 벗어나 로봇 작업공간 밖 목표를 만들었으므로, 로봇 동작에는
# 클래스별 실제 USD 표면점을 사용한다. 비전은 클래스 식별/처리 순서만 담당한다.
USE_MEASURED_USD_CONTACT_POINTS = False
MEASURED_USD_CONTACT_POINT_WORLD = {
    'plastic_bottle': (-0.6008, 1.6931, 1.0968),
    'buoy': (-0.5761, 2.1358, 1.2432),
    'can': (-0.5295, 2.3743, 1.1810),
}


class Robot2HomeRecoveryFailure(Exception):
    """(v2) RECOVERY_HOME과 안전 재후퇴까지 모두 실패해 로봇 위치를 신뢰할 수
    없을 때 발생시킨다. 로봇2는 원래도 실패 시 전체 Action을 abort하지만,
    이 예외는 그 사유가 "일반 실패"가 아니라 "안전 확보 실패"임을 명확히
    구분해서 로그/결과 메시지에 남기기 위한 용도다.
    """

    def __init__(self, object_id: str, message: str) -> None:
        super().__init__(message)
        self.object_id = object_id


class Robot2State(Enum):
    IDLE = auto()
    PREGRASP = auto()
    PREGRASP_HOLD = auto()
    CONTACT = auto()
    CONTACT_HOLD = auto()
    SUCTION_ON = auto()
    JOINT_RELEASE = auto()
    RETREAT = auto()
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
        self._suction_active = False

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
        grasp_offset_norm = float(np.linalg.norm(GRASP_POINT_OFFSET_WORLD))
        if grasp_offset_norm > 0.05:
            self.get_logger().warning(
                'GRASP_POINT_OFFSET_WORLD 크기가 '
                f'{grasp_offset_norm:.3f} m입니다. 현재 값 '
                f'{GRASP_POINT_OFFSET_WORLD}은 실측 보정값이어야 하며, '
                'TCP가 물체에서 크게 벗어나면 (0, 0, 0)부터 다시 보정하세요.'
            )
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
            if DIRECT_SUCTION_ONLY_MODE:
                # 이 모드는 처음부터 로봇을 움직이지 않으므로 Home 명령도 필요 없다.
                # Station2 전체에서 /robot2/motion_command를 한 번도 보내지 않는다.
                self.get_logger().info(
                    'DIRECT_SUCTION_ONLY: Robot2 motion/Home 명령을 생략합니다.'
                )
            elif not self._send_motion_and_wait(self._home_pose(), 'HOME'):
                return self._abort(goal_handle, result, 'Robot 2 Home 복귀 실패')

            goal_handle.succeed()
            result.success = True
            result.message = f'{total_targets}개 객체 제거 완료'
            self.get_logger().info(result.message)
            return result
        except Robot2HomeRecoveryFailure as error:
            # (v2) RECOVERY_HOME + 안전 재후퇴까지 모두 실패한 경우:
            # 일반 실패와 구분되는 명확한 사유를 남기고 abort한다.
            self.get_logger().error(
                f'Robot 2 안전 복구 실패로 전체 작업을 중단합니다: {error}'
            )
            goal_handle.abort()
            result.success = False
            result.message = str(error)
            return result
        except Exception as error:
            self.get_logger().error(f'Robot 2 실행 오류: {error}')
            goal_handle.abort()
            result.success = False
            result.message = str(error)
            return result
        finally:
            # 예외/모션 실패가 어느 단계에서 나더라도 다음 Cycle에 armed 상태나
            # Runtime FixedJoint가 남지 않도록 마지막으로 OFF를 보장한다.
            if self._suction_active:
                command = Bool()
                command.data = False
                self._suction_command_pub.publish(command)
                self._suction_active = False
                self.get_logger().warning(
                    'Action 종료 정리: 확인되지 않은 suction OFF를 재전송했습니다.'
                )
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

            surface = add_tool_offset(
                position_world,
                GRASP_POINT_OFFSET_WORLD,
            )
            contact = offset_point(surface, self._net_normal, CONTACT_STANDOFF_M)
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

            self.get_logger().info(
                f'{target.object_id}: source_frame={source_frame}, '
                f'vision=({target.position.x:.4f}, {target.position.y:.4f}, '
                f'{target.position.z:.4f}), world=({position_world.x:.4f}, '
                f'{position_world.y:.4f}, {position_world.z:.4f}), '
                f'contact=({contact.x:.4f}, {contact.y:.4f}, {contact.z:.4f}) '
                f'(standoff {CONTACT_STANDOFF_M * 1000:.0f} mm)'
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
        if DIRECT_SUCTION_ONLY_MODE:
            return self._execute_one_target_direct_suction(
                goal_handle,
                target,
                current_target,
                total_targets,
            )

        if USE_MEASURED_USD_CONTACT_POINTS:
            measured_xyz = MEASURED_USD_CONTACT_POINT_WORLD.get(
                target.class_name
            )
            if measured_xyz is None:
                self.get_logger().error(
                    f'{target.object_id}: 실제 USD 접촉 좌표가 없습니다.'
                )
                return False
            measured_contact = _point_from_xyz(measured_xyz)
            self.get_logger().warning(
                f'{target.object_id}: 실제 USD 표면 좌표로 이동합니다. '
                f'vision_contact=({target.contact_world.x:.4f}, '
                f'{target.contact_world.y:.4f}, {target.contact_world.z:.4f}), '
                f'USD_contact=({measured_contact.x:.4f}, '
                f'{measured_contact.y:.4f}, {measured_contact.z:.4f})'
            )
            target = PreparedTarget(
                object_id=target.object_id,
                class_name=target.class_name,
                contact_world=measured_contact,
                fixed_joint_path=target.fixed_joint_path,
                left_to_right_score=float(
                    np.dot(
                        np.asarray(measured_xyz, dtype=float),
                        self._left_to_right_axis,
                    )
                ),
            )

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
        self._hold(PREGRASP_HOLD_SEC, 'PREGRASP_HOLD')

        self.state = Robot2State.CONTACT
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'CONTACT'
        )
        # ============================================================
        # v2 이전 로직 (주석 처리, 통합 테스트 후 문제 없으면 삭제):
        #
        # if not self._send_motion_and_wait(
        #     self._gripper_pose(target.contact_world),
        #     f'{target.object_id}:CONTACT',
        # ):
        #     return False
        #
        # 문제점: CONTACT 실패 시 재시도 없이 곧바로 전체 Action abort.
        # 로봇1과 달리 net 밖 완충 지점을 거친 재시도 기회가 전혀 없었음.
        # ============================================================
        if not self._contact_with_recovery(target, pregrasp, retreat):
            return False

        self.state = Robot2State.CONTACT_HOLD
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'CONTACT_HOLD'
        )
        self._hold(CONTACT_HOLD_SEC, 'CONTACT_HOLD')

        self.state = Robot2State.SUCTION_ON
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'SUCTION_ON'
        )
        if not self._send_suction_and_wait(True):
            self._recover_after_attachment_failure(target, retreat, 'SUCTION_ON')
            return False

        # Surface Gripper 흡착이 확인된 뒤에만 어망-쓰레기 Fixed Joint를 해제합니다.
        self.state = Robot2State.JOINT_RELEASE
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'JOINT_RELEASE'
        )
        if not self._release_joint_and_wait(target.class_name):
            self._recover_after_attachment_failure(target, retreat, 'JOINT_RELEASE')
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

        self.state = Robot2State.SUCTION_OFF
        self._feedback_for_target(
            goal_handle, target, current_target, total_targets, 'SUCTION_OFF'
        )
        return self._send_suction_and_wait(False)

    def _execute_one_target_direct_suction(
        self,
        goal_handle,
        target: PreparedTarget,
        current_target: int,
        total_targets: int,
    ) -> bool:
        """IK 없이 쓰레기를 SuctionTCP에 붙이고 어망에서 즉시 분리한다."""
        self.get_logger().warning(
            f'{target.object_id}: DIRECT_SUCTION_ONLY - '
            'Cartesian IK를 건너뛰고 직접 흡착/분리합니다.'
        )

        self.state = Robot2State.SUCTION_ON
        self._feedback_for_target(
            goal_handle,
            target,
            current_target,
            total_targets,
            'DIRECT_SUCTION_ON',
        )
        if not self._send_suction_and_wait(True):
            return False

        self.state = Robot2State.JOINT_RELEASE
        self._feedback_for_target(
            goal_handle,
            target,
            current_target,
            total_targets,
            'DIRECT_ATTACH_AND_RELEASE',
        )
        if not self._release_joint_and_wait(target.class_name):
            self.get_logger().error(
                f'{target.object_id}: 직접 흡착/어망 Joint 해제 실패'
            )
            self._send_suction_and_wait(False)
            return False

        # 화면에서 쓰레기가 흡착기에 붙은 상태를 확인할 수 있도록 잠시 유지한다.
        self._hold(DIRECT_SUCTION_HOLD_SEC, 'DIRECT_SUCTION_HOLD')

        self.state = Robot2State.SUCTION_OFF
        self._feedback_for_target(
            goal_handle,
            target,
            current_target,
            total_targets,
            'DIRECT_DROP',
        )
        return self._send_suction_and_wait(False)

    def _hold(self, seconds: float, label: str) -> None:
        """Wait ``seconds`` of simulation time (/clock)."""
        if not sleep_sim_time(self.get_clock(), seconds):
            self.get_logger().warning(
                f'{label}: 시뮬 시간이 흐르지 않아 벽시계 제한 시간 후 진행합니다.'
            )

    def _recover_after_attachment_failure(
        self,
        target: PreparedTarget,
        retreat: Point,
        failed_state: str,
    ) -> None:
        """흡착 계통 실패 시 접촉 자세에 멈추지 않고 안전하게 복귀한다."""
        self.get_logger().error(
            f'{target.object_id}: {failed_state} 실패 - '
            '흡착 해제 후 RETREAT/HOME 안전 복귀를 시작합니다.'
        )

        # ON 응답 자체가 유실된 경우에도 Standalone 쪽은 이미 armed일 수 있으므로
        # 로컬 상태와 무관하게 OFF를 반드시 한 번 보낸다. OFF는 idempotent하다.
        if not self._send_suction_and_wait(False):
            self.get_logger().error(
                f'{target.object_id}: 실패 복구 중 suction OFF 확인 실패'
            )

        retreat_ok = self._send_motion_and_wait(
            self._gripper_pose(retreat),
            f'{target.object_id}:ABORT_RETREAT',
        )
        if not retreat_ok:
            self.get_logger().error(
                f'{target.object_id}: 실패 복구 RETREAT 실패 - HOME 직접 시도'
            )

        if not self._send_motion_and_wait(
            self._home_pose(),
            f'{target.object_id}:ABORT_HOME',
        ):
            self.get_logger().error(
                f'{target.object_id}: 실패 복구 HOME 복귀 실패'
            )

    def _contact_with_recovery(
        self,
        target: PreparedTarget,
        pregrasp: Point,
        retreat: Point,
    ) -> bool:
        """(v2) CONTACT 실패 시 로봇1과 동일한 패턴으로 복구를 시도한다.

        실패 -> RECOVERY_RETREAT(안전 후퇴) -> RECOVERY_HOME
             -> RECOVERY_PREGRASP(재진입) -> CONTACT 재시도.

        RECOVERY_HOME까지 실패하면 안전 재후퇴를 한 번 더 시도하고, 그래도
        실패하면 Robot2HomeRecoveryFailure를 발생시켜 전체 Action을 abort한다.
        (로봇2는 원래도 실패 시 전체 abort 정책이므로 최종 동작은 동일하지만,
        포기 전 안전 경로 재시도 기회와 더 명확한 실패 사유를 남긴다.)
        """
        for retry in range(2):
            contact_label = f'{target.object_id}:CONTACT_ATTEMPT_{retry + 1}'
            if self._send_motion_and_wait(
                self._gripper_pose(target.contact_world), contact_label
            ):
                return True

            self.get_logger().warning(
                f'{target.object_id} CONTACT 시도 실패 ({retry + 1}/2)'
            )

            if retry == 0:
                # 흡착이 아직 이루어지지 않은 상태이므로 net 밖으로 안전 후퇴 가능
                retreat_label = f'{target.object_id}:RECOVERY_RETREAT'
                if not self._send_motion_and_wait(
                    self._gripper_pose(retreat), retreat_label
                ):
                    self.get_logger().warning(
                        f'{target.object_id} RECOVERY_RETREAT 실패 - '
                        f'현재 위치에서 Home 시도'
                    )

                home_label = f'{target.object_id}:RECOVERY_HOME'
                recovery_success = self._send_motion_and_wait(
                    self._home_pose(), home_label
                )

                if not recovery_success:
                    self.get_logger().error(
                        f'{target.object_id} RECOVERY_HOME 실패 - 안전 확보를 '
                        f'위해 재후퇴를 한 번 더 시도합니다.'
                    )
                    failsafe_label = f'{target.object_id}:RECOVERY_FAILSAFE_RETREAT'
                    failsafe_ok = self._send_motion_and_wait(
                        self._gripper_pose(retreat), failsafe_label
                    )

                    if not failsafe_ok:
                        raise Robot2HomeRecoveryFailure(
                            target.object_id,
                            f'{target.object_id}: RECOVERY_HOME 실패 + 안전 재후퇴도 '
                            f'실패 - 로봇 위치를 신뢰할 수 없어 전체 작업을 '
                            f'중단합니다.',
                        )

                    self.get_logger().error(
                        f'{target.object_id} RECOVERY_HOME은 실패했지만 안전 재후퇴는 '
                        f'성공 - 이 객체 제거를 중단합니다.'
                    )
                    return False

                pregrasp_label = f'{target.object_id}:RECOVERY_PREGRASP'
                if not self._send_motion_and_wait(
                    self._gripper_pose(pregrasp), pregrasp_label
                ):
                    self.get_logger().warning(
                        f'{target.object_id} RECOVERY_PREGRASP 실패 - '
                        f'현재 위치에서 CONTACT 재시도'
                    )

        return False

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
            # (v2) 기존: self.get_logger().error(f'Motion timeout: {label}')
            # 실패 시 목표 pose 좌표를 함께 남겨 사후 원인 분석을 돕는다.
            self.get_logger().error(
                f'Motion timeout: {label} target=('
                f'{pose.pose.position.x:.4f}, '
                f'{pose.pose.position.y:.4f}, '
                f'{pose.pose.position.z:.4f})'
            )
            return False

        with self._motion_lock:
            success = bool(self._motion_result)
            self._motion_waiting = False

        if not success:
            # (v2) Standalone이 명시적으로 실패(Bool=False)를 보고한 경우에도
            # 동일하게 목표 pose 좌표를 남긴다.
            self.get_logger().error(
                f'Motion failed (standalone reported False): {label} target=('
                f'{pose.pose.position.x:.4f}, '
                f'{pose.pose.position.y:.4f}, '
                f'{pose.pose.position.z:.4f})'
            )
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
            self._suction_active = enabled
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
