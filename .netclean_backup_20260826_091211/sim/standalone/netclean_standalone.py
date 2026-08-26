#!/usr/bin/env python3
"""NetClean Isaac Sim 5.1 standalone execution layer.

역할
----
1. 저장된 netclean_world.usd를 GUI로 연다.
2. World/Physics를 계속 step한다.
3. /net/target_pose를 받아 NetRoot를 보간 이동한다.
4. Robot1/2의 world TCP Pose 명령을 Lula IK로 joint 목표로 바꿔 보간한다.
5. Robot2 Surface Gripper를 실제로 열고 닫는다.
6. Robot2가 보낸 class_name을 고정 매핑하여 해당 Fixed Joint를 해제한다.
7. 외부 ROS 2 노드에 준비/현재 pose/완료 상태를 발행한다.
8. 한 사이클 종료 후 reset 요청을 받으면 월드를 초기 상태로 복원한다.

실행 예시
---------
Isaac Sim 설치 폴더에서:

    ./python.sh /절대/경로/netclean_standalone.py

또는 USD 경로만 명령행에서 덮어쓰기:

    ./python.sh /절대/경로/netclean_standalone.py --usd /절대/경로/netclean_world.usd

중요
----
- 이 파일은 Isaac Sim 전용 Python으로 실행해야 한다.
- 아래 "사용자가 채울 값"에 있는 TODO를 실제 월드 정보로 바꾸기 전에는
  안전을 위해 초기화 단계에서 중단한다.
- 랜덤 스폰, 동적 registry, YOLO, 공정 FSM은 포함하지 않는다.
- 현재 robot2_node.py에 맞춰 class_name 기반 Joint 해제 방식을 사용한다.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
import os
import signal
import sys
import time
from typing import Deque, Dict, Optional, Sequence, Tuple

import numpy as np


# =============================================================================
# 사용자 입력: GUI 월드를 만드는 동안 최종적으로 확보해야 하는 정보
# =============================================================================

# 모든 TODO를 채운 뒤 True로 변경한다.
# False이면 잘못된 예시 경로로 로봇/물리를 움직이지 않고 즉시 종료한다.
CONFIG_READY = False


# 1) 저장한 최종 USD의 절대 경로
# 예: "/home/yong/netclean/assets/netclean_world.usd"
USD_PATH = "/TODO/absolute/path/to/netclean_world.usd"


# 2) 월드 안의 필수 Prim 경로
# Stage 창에서 Prim을 우클릭하여 Copy Prim Path로 복사한다.
#
# ROBOT*_PRIM_PATH는 M0609의 Articulation Root가 붙은 Prim이어야 한다.
# NET_ROOT_PRIM_PATH는 어망과 아직 붙어 있는 쓰레기가 함께 이동하는 상위 Prim이다.
# Physics 중 set_world_pose로 움직여야 하므로 Static Collider로 만들면 안 된다.
# Rigid Body를 적용했다면 Kinematic Enabled를 켜고, 자식 Fixed Joint의 Body0/Body1가
# 실제 Net/쓰레기 Rigid Body를 정확히 가리키는지 확인한다.
# ROBOT2_SURFACE_GRIPPER_PRIM_PATH는 Create > Robots > Surface Gripper로 만든
# Surface Gripper Schema Prim이다. 흡착 컵의 시각 Mesh 경로가 아니다.
# 이 Prim의 Attachment Points에는 USD에서 미리 만든 D6 Joint가 연결되어 있어야 한다.
ROBOT1_PRIM_PATH = "/World/TODO/Robot1_M0609"
ROBOT2_PRIM_PATH = "/World/TODO/Robot2_M0609"
NET_ROOT_PRIM_PATH = "/World/TODO/NetRoot"
ROBOT2_SURFACE_GRIPPER_PRIM_PATH = "/World/TODO/Robot2/SurfaceGripper"


# 3) USD에 저장해 둔 Camera와 ROS Action Graph Prim 경로
# Standalone은 카메라 토픽이나 /clock을 직접 publish하지 않는다.
# 아래 Prim들이 USD에 존재하는지만 검사하고, 실제 publish는 저장된 Action Graph가 한다.
CAMERA1_PRIM_PATH = "/World/TODO/Camera1"
CAMERA2_PRIM_PATH = "/World/TODO/Camera2"
ROS_CLOCK_GRAPH_PRIM_PATH = "/World/TODO/AG_ROS_Clock"
CAMERA1_GRAPH_PRIM_PATH = "/World/TODO/AG_Camera1"
CAMERA2_GRAPH_PRIM_PATH = "/World/TODO/AG_Camera2"


# 4) M0609 Lula IK 파일
# Robot1과 Robot2가 같은 M0609이므로 URDF/descriptor는 한 세트만 사용한다.
#
# URDF 필수 내용:
# - USD Articulation과 동일한 6개 관절 이름 및 순서
# - 각 관절 position limit
# - Lula에서 목표로 사용할 flange/link_6 frame 이름
#
# robot_description.yaml 필수 내용:
# - 6개 actuated joint 목록
# - default c-space configuration
# 순수 Lula IK만 사용하므로 collision sphere는 없어도 된다.
M0609_URDF_PATH = "/TODO/absolute/path/to/m0609.urdf"
M0609_LULA_DESCRIPTOR_PATH = "/TODO/absolute/path/to/robot_description.yaml"
M0609_END_EFFECTOR_FRAME = "TODO_link_6"


# 5) Lula end-effector frame(flange/link_6) -> 실제 TCP 변환
# Robot 노드가 보내는 PoseStamped는 칼날/흡착 TCP의 world pose다.
# Lula의 end-effector frame이 TCP 자체라면 position=(0,0,0), quaternion=(1,0,0,0).
#
# position 단위: m, flange 좌표계에서 본 TCP 위치
# quaternion 순서: Isaac Sim/Lula 형식 [w, x, y, z]
#
# 주의: robot1_node의 CUT_POINT_OFFSET_WORLD와 robot2_node의
# GRASP_POINT_OFFSET_WORLD는 비전점 미세보정이다. 아래 기계적 TCP offset과 다르며,
# 같은 값을 양쪽에 중복 적용하면 안 된다.
ROBOT1_FLANGE_TO_TCP_POSITION = (0.0, 0.0, 0.0)  # TODO: Cutter 장착 치수
ROBOT1_FLANGE_TO_TCP_QUATERNION_WXYZ = (1.0, 0.0, 0.0, 0.0)  # TODO

ROBOT2_FLANGE_TO_TCP_POSITION = (0.0, 0.0, 0.0)  # TODO: Suction 장착 치수
ROBOT2_FLANGE_TO_TCP_QUATERNION_WXYZ = (1.0, 0.0, 0.0, 0.0)  # TODO


# 6) class_name -> Fixed Joint Prim 경로
# 현재 확정 조건:
# - 작업 중인 어망 한 장에는 클래스별 객체가 최대 한 개다.
# - 랜덤 스폰하지 않는다.
# - robot2_node가 plastic_bottle/can/buoy 문자열을 보낸다.
#
# 반드시 실제 "Physics Fixed Joint" Prim 경로를 넣는다.
# PET/CAN/BUOY Mesh Prim 경로가 아니다.
JOINT_PATH_BY_CLASS: Dict[str, str] = {
    "plastic_bottle": "/World/TODO/NetRoot/Joints/PET_01_joint",
    "can": "/World/TODO/NetRoot/Joints/CAN_01_joint",
    "buoy": "/World/TODO/NetRoot/Joints/BUOY_01_joint",
}


# 7) 시간과 허용오차
# 아래 값은 일단 시작 가능한 보수적 기본값이다. 월드 완성 후 실제 시연으로 조정한다.
PHYSICS_DT = 1.0 / 60.0
RENDERING_DT = 1.0 / 60.0
WARMUP_STEPS = 30

NET_SPEED_MPS = 0.20
NET_CURRENT_POSE_HZ = 20.0
READY_PUBLISH_HZ = 1.0

ROBOT_JOINT_SPEED_RAD_S = 0.50
ROBOT_MIN_MOTION_DURATION_SEC = 0.30
ROBOT_JOINT_TOLERANCE_RAD = 0.02
ROBOT_MOTION_TIMEOUT_SEC = 15.0

SUCTION_TIMEOUT_SEC = 5.0


# =============================================================================
# 이 Standalone에 입력하지 않는 값
# =============================================================================
# - START/S1/S2/EXIT 좌표: control_node 파라미터이며 /net/target_pose로 수신한다.
# - Camera1/2 world pose: robot1_node/robot2_node의 Camera->World 변환 보정값이다.
# - Robot1/2 Home TCP pose: 각 robot node가 motion_command로 보낸다.
# - Robot2 Drop pose: robot2_node가 motion_command로 보낸다.
# - 절단점/파지점: vision node와 robot node가 계산한다.
# - 공정 순서: control_node와 robot node의 FSM이 담당한다.


# =============================================================================
# ROS Topic: 현재 업로드된 control/robot 노드 코드와 정확히 일치
# =============================================================================

SIM_READY_TOPIC = "/sim/ready"
SIM_RESET_REQUEST_TOPIC = "/sim/reset_request"
SIM_RESET_DONE_TOPIC = "/sim/reset_done"
NET_TARGET_POSE_TOPIC = "/net/target_pose"
NET_CURRENT_POSE_TOPIC = "/net/current_pose"

ROBOT1_MOTION_COMMAND_TOPIC = "/robot1/motion_command"
ROBOT1_MOTION_DONE_TOPIC = "/robot1/motion_done"

ROBOT2_MOTION_COMMAND_TOPIC = "/robot2/motion_command"
ROBOT2_MOTION_DONE_TOPIC = "/robot2/motion_done"
ROBOT2_SUCTION_COMMAND_TOPIC = "/robot2/suction_command"
ROBOT2_SUCTION_STATE_TOPIC = "/robot2/suction_state"

# 현재 robot2_node.py가 class_name을 보내는 토픽이다.
ROBOT2_RELEASE_OBJECT_TOPIC = "/robot2/release_attached_object"
ROBOT2_JOINT_RELEASE_DONE_TOPIC = "/robot2/joint_release_done"

WORLD_FRAME = "world"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="NetClean Isaac Sim standalone")
    parser.add_argument(
        "--usd",
        default=USD_PATH,
        help="netclean_world.usd absolute path (overrides USD_PATH)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without GUI; default is GUI mode",
    )
    return parser.parse_args()


ARGS = _parse_args()


# SimulationApp은 다른 Isaac Sim/Omniverse 모듈보다 반드시 먼저 생성한다.
from isaacsim import SimulationApp


simulation_app = SimulationApp(
    {
        "headless": bool(ARGS.headless),
        "width": 1280,
        "height": 720,
    }
)


# SimulationApp 생성 이후에만 import 가능한 런타임 모듈들이다.
import carb
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from pxr import UsdPhysics


# ROS 2 Bridge와 Surface Gripper extension을 코드에서 명시적으로 활성화한다.
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.robot.surface_gripper")
simulation_app.update()


import isaacsim.robot.surface_gripper._surface_gripper as surface_gripper
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _is_todo(value: str) -> bool:
    lowered = value.strip().lower()
    return not lowered or "todo" in lowered


def _normalize_quaternion_wxyz(values: Sequence[float], name: str) -> np.ndarray:
    quat = np.asarray(values, dtype=float)
    if quat.shape != (4,) or not np.all(np.isfinite(quat)):
        raise ValueError(f"{name}: quaternion must contain four finite values")
    norm = float(np.linalg.norm(quat))
    if norm < 1.0e-9:
        raise ValueError(f"{name}: zero quaternion is invalid")
    return quat / norm


def _quaternion_multiply_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def _quaternion_inverse_wxyz(quat: np.ndarray) -> np.ndarray:
    # 입력은 위에서 단위 quaternion으로 정규화되어 있다.
    return np.array([quat[0], -quat[1], -quat[2], -quat[3]], dtype=float)


def _quaternion_to_rotation_matrix_wxyz(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = quat
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _ros_xyzw_to_isaac_wxyz(msg_quaternion) -> np.ndarray:
    return _normalize_quaternion_wxyz(
        (
            msg_quaternion.w,
            msg_quaternion.x,
            msg_quaternion.y,
            msg_quaternion.z,
        ),
        "ROS target orientation",
    )


def _tcp_target_to_flange_target(
    tcp_position_world: np.ndarray,
    tcp_orientation_world_wxyz: np.ndarray,
    flange_to_tcp_position: np.ndarray,
    flange_to_tcp_orientation_wxyz: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """T_world_flange = T_world_tcp * inverse(T_flange_tcp)."""
    flange_orientation = _quaternion_multiply_wxyz(
        tcp_orientation_world_wxyz,
        _quaternion_inverse_wxyz(flange_to_tcp_orientation_wxyz),
    )
    flange_orientation = _normalize_quaternion_wxyz(
        flange_orientation,
        "computed flange orientation",
    )
    rotation_world_flange = _quaternion_to_rotation_matrix_wxyz(flange_orientation)
    flange_position = (
        tcp_position_world
        - rotation_world_flange @ flange_to_tcp_position
    )
    return flange_position, flange_orientation


def _smoothstep(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * (3.0 - 2.0 * value)


@dataclass(frozen=True)
class RobotConfig:
    name: str
    prim_path: str
    urdf_path: str
    descriptor_path: str
    end_effector_frame: str
    flange_to_tcp_position: Tuple[float, float, float]
    flange_to_tcp_orientation_wxyz: Tuple[float, float, float, float]
    motion_done_topic: str


@dataclass
class MotionRequest:
    position_world: np.ndarray
    orientation_world_wxyz: np.ndarray


class NetMotionController:
    """NetRoot position만 속도 제한 보간하고 시작 orientation은 보존한다."""

    def __init__(self, net_root: SingleXFormPrim, speed_mps: float) -> None:
        self._net_root = net_root
        self._speed_mps = float(speed_mps)
        initial_position, initial_orientation = self._net_root.get_world_pose()
        self._initial_orientation = np.asarray(initial_orientation, dtype=float)
        self._target_position = np.asarray(initial_position, dtype=float)

    def set_target(self, position_world: Sequence[float]) -> None:
        target = np.asarray(position_world, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Net target position must be three finite values")
        self._target_position = target

    def update(self, dt: float) -> None:
        current_position, _ = self._net_root.get_world_pose()
        current = np.asarray(current_position, dtype=float)
        delta = self._target_position - current
        distance = float(np.linalg.norm(delta))
        if distance < 1.0e-6:
            return
        step_distance = min(distance, self._speed_mps * dt)
        next_position = current + delta * (step_distance / distance)
        self._net_root.set_world_pose(
            position=next_position,
            orientation=self._initial_orientation,
        )

    def get_world_pose(self) -> Tuple[np.ndarray, np.ndarray]:
        return self._net_root.get_world_pose()

    def reset(self) -> None:
        """World reset 후 현재 초기 pose를 새 목표로 사용한다."""
        position, orientation = self._net_root.get_world_pose()
        self._initial_orientation = np.asarray(orientation, dtype=float)
        self._target_position = np.asarray(position, dtype=float)


class RobotMotionController:
    """한 로봇의 Pose 명령 queue, Lula IK, joint 보간, 완료 이벤트를 관리한다."""

    def __init__(
        self,
        robot: SingleArticulation,
        config: RobotConfig,
        result_publisher,
    ) -> None:
        self._robot = robot
        self._config = config
        self._result_publisher = result_publisher
        self._requests: Deque[MotionRequest] = deque()

        self._flange_to_tcp_position = np.asarray(
            config.flange_to_tcp_position,
            dtype=float,
        )
        self._flange_to_tcp_orientation = _normalize_quaternion_wxyz(
            config.flange_to_tcp_orientation_wxyz,
            f"{config.name} flange_to_tcp_orientation",
        )

        self._lula = LulaKinematicsSolver(
            robot_description_path=config.descriptor_path,
            urdf_path=config.urdf_path,
        )

        # Articulation Root Prim이 URDF root/base frame과 같은 pose라는 전제다.
        # 로봇 USD에서 별도의 상위 Xform에 배치했다면 이 값이 맞는지 확인해야 한다.
        base_position, base_orientation = self._robot.get_world_pose()
        self._lula.set_robot_base_pose(base_position, base_orientation)

        self._ik = ArticulationKinematicsSolver(
            self._robot,
            self._lula,
            config.end_effector_frame,
        )

        self._active = False
        self._start_time = 0.0
        self._elapsed = 0.0
        self._duration = 0.0
        self._indices = np.array([], dtype=np.int64)
        self._start_joint_positions = np.array([], dtype=float)
        self._target_joint_positions = np.array([], dtype=float)

    def enqueue(self, request: MotionRequest) -> None:
        self._requests.append(request)

    def reset(self) -> None:
        """진행 중 동작과 대기 명령을 폐기하고 Lula base pose를 다시 맞춘다."""
        self._requests.clear()
        self._active = False
        self._start_time = 0.0
        self._elapsed = 0.0
        self._duration = 0.0
        self._indices = np.array([], dtype=np.int64)
        self._start_joint_positions = np.array([], dtype=float)
        self._target_joint_positions = np.array([], dtype=float)

        base_position, base_orientation = self._robot.get_world_pose()
        self._lula.set_robot_base_pose(base_position, base_orientation)

    def update(self, dt: float) -> None:
        if not self._active:
            if not self._requests:
                return
            request = self._requests.popleft()
            if not self._begin_request(request):
                self._publish_result(False)
                return

        self._elapsed += dt
        ratio = self._elapsed / max(self._duration, 1.0e-6)
        blend = _smoothstep(ratio)
        command = (
            self._start_joint_positions
            + blend * (self._target_joint_positions - self._start_joint_positions)
        )
        self._robot.apply_action(
            ArticulationAction(
                joint_positions=command,
                joint_indices=self._indices,
            )
        )

        current = np.asarray(
            self._robot.get_joint_positions(joint_indices=self._indices),
            dtype=float,
        )
        error = float(np.max(np.abs(current - self._target_joint_positions)))

        if ratio >= 1.0 and error <= ROBOT_JOINT_TOLERANCE_RAD:
            self._active = False
            self._publish_result(True)
            return

        if time.monotonic() - self._start_time > ROBOT_MOTION_TIMEOUT_SEC:
            carb.log_error(
                f"[{self._config.name}] motion timeout; max joint error={error:.4f} rad"
            )
            self._active = False
            self._publish_result(False)

    def _begin_request(self, request: MotionRequest) -> bool:
        try:
            flange_position, flange_orientation = _tcp_target_to_flange_target(
                request.position_world,
                request.orientation_world_wxyz,
                self._flange_to_tcp_position,
                self._flange_to_tcp_orientation,
            )
            target_action, success = self._ik.compute_inverse_kinematics(
                target_position=flange_position,
                target_orientation=flange_orientation,
            )
        except Exception as exc:
            carb.log_error(f"[{self._config.name}] IK exception: {exc}")
            return False

        if not success or target_action.joint_positions is None:
            carb.log_error(f"[{self._config.name}] IK failed")
            return False

        target_positions = np.asarray(target_action.joint_positions, dtype=float)
        if target_action.joint_indices is None:
            indices = np.arange(target_positions.size, dtype=np.int64)
        else:
            indices = np.asarray(target_action.joint_indices, dtype=np.int64)

        if target_positions.ndim != 1 or target_positions.size != indices.size:
            carb.log_error(
                f"[{self._config.name}] unexpected IK action shape: "
                f"positions={target_positions.shape}, indices={indices.shape}"
            )
            return False
        if not np.all(np.isfinite(target_positions)):
            carb.log_error(f"[{self._config.name}] IK returned NaN/Inf")
            return False

        start_positions = np.asarray(
            self._robot.get_joint_positions(joint_indices=indices),
            dtype=float,
        )
        max_delta = float(np.max(np.abs(target_positions - start_positions)))
        duration = max(
            ROBOT_MIN_MOTION_DURATION_SEC,
            max_delta / ROBOT_JOINT_SPEED_RAD_S,
        )

        self._indices = indices
        self._start_joint_positions = start_positions
        self._target_joint_positions = target_positions
        self._duration = duration
        self._elapsed = 0.0
        self._start_time = time.monotonic()
        self._active = True
        carb.log_info(
            f"[{self._config.name}] motion started; duration={duration:.2f}s"
        )
        return True

    def _publish_result(self, success: bool) -> None:
        msg = Bool()
        msg.data = bool(success)
        # 명령 하나당 정확히 한 번만 발행한다.
        self._result_publisher.publish(msg)


class SuctionController:
    """Surface Gripper 명령을 실행하고 USD runtime status를 확인한다."""

    def __init__(self, gripper_prim_path: str, state_publisher) -> None:
        self._path = gripper_prim_path
        self._publisher = state_publisher
        self._interface = surface_gripper.acquire_surface_gripper_interface()
        self._requests: Deque[bool] = deque()
        self._active = False
        self._expected = False
        self._start_time = 0.0

    def enqueue(self, enabled: bool) -> None:
        self._requests.append(bool(enabled))

    def is_closed(self) -> bool:
        status = str(self._interface.get_gripper_status(self._path)).lower()
        return status == "closed"

    def is_open(self) -> bool:
        status = str(self._interface.get_gripper_status(self._path)).lower()
        return status == "open"

    def reset(self) -> None:
        """대기 명령을 폐기하고 Surface Gripper를 강제로 Open 상태로 만든다."""
        self._requests.clear()
        self._active = False
        self._expected = False
        self._start_time = 0.0
        self._interface.open_gripper(self._path)

    def update(self) -> None:
        if not self._active:
            if not self._requests:
                return
            self._expected = self._requests.popleft()
            self._start_time = time.monotonic()
            self._active = True
            if self._expected:
                self._interface.close_gripper(self._path)
            else:
                self._interface.open_gripper(self._path)

        status = str(self._interface.get_gripper_status(self._path)).lower()
        reached = status == ("closed" if self._expected else "open")
        if reached:
            self._publish_actual_state()
            self._active = False
            return

        if time.monotonic() - self._start_time > SUCTION_TIMEOUT_SEC:
            carb.log_error(
                f"[robot2 suction] timeout; expected={self._expected}, status={status}"
            )
            # Bool 프로토콜에는 별도 success 필드가 없으므로 확인된 실제 상태를 보낸다.
            # robot2_node는 기대값과 다르면 계속 기다리다가 자체 timeout으로 실패 처리한다.
            self._publish_actual_state()
            self._active = False

    def _publish_actual_state(self) -> None:
        msg = Bool()
        msg.data = self.is_closed()
        self._publisher.publish(msg)


class FixedJointReleaseController:
    """class_name을 allowlist Joint 경로로 바꾸고 Fixed Joint를 해제한다."""

    def __init__(
        self,
        stage,
        joint_path_by_class: Dict[str, str],
        suction_controller: SuctionController,
        result_publisher,
    ) -> None:
        self._stage = stage
        self._paths = dict(joint_path_by_class)
        self._suction = suction_controller
        self._publisher = result_publisher
        self._requests: Deque[str] = deque()

    def enqueue(self, class_name: str) -> None:
        self._requests.append(class_name.strip().lower())

    def reset(self) -> None:
        """대기 중 해제 요청을 폐기하고 모든 쓰레기 Fixed Joint를 재활성화한다."""
        self._requests.clear()
        failures = []
        for class_name, joint_path in self._paths.items():
            prim = self._stage.GetPrimAtPath(joint_path)
            if not prim.IsValid() or not prim.IsA(UsdPhysics.FixedJoint):
                failures.append(f"{class_name}: {joint_path}")
                continue
            joint_schema = UsdPhysics.Joint(prim)
            enabled_attr = joint_schema.GetJointEnabledAttr()
            if not enabled_attr.IsValid():
                enabled_attr = joint_schema.CreateJointEnabledAttr(True)
            enabled_attr.Set(True)
            if enabled_attr.Get() is not True:
                failures.append(f"{class_name}: {joint_path}")

        if failures:
            raise RuntimeError(
                "Failed to restore Fixed Joints: " + ", ".join(failures)
            )

    def all_enabled(self) -> bool:
        for joint_path in self._paths.values():
            prim = self._stage.GetPrimAtPath(joint_path)
            if not prim.IsValid() or not prim.IsA(UsdPhysics.FixedJoint):
                return False
            enabled_attr = UsdPhysics.Joint(prim).GetJointEnabledAttr()
            if not enabled_attr.IsValid() or enabled_attr.Get() is not True:
                return False
        return True

    def update(self) -> None:
        if not self._requests:
            return
        class_name = self._requests.popleft()
        success = self._release(class_name)
        msg = Bool()
        msg.data = success
        # 요청 하나당 정확히 한 번만 발행한다.
        self._publisher.publish(msg)

    def _release(self, class_name: str) -> bool:
        joint_path = self._paths.get(class_name)
        if joint_path is None:
            carb.log_error(f"[joint release] unknown class_name={class_name!r}")
            return False
        if not self._suction.is_closed():
            carb.log_error("[joint release] rejected because suction is not Closed")
            return False

        prim = self._stage.GetPrimAtPath(joint_path)
        if not prim.IsValid():
            carb.log_error(f"[joint release] missing Prim: {joint_path}")
            return False
        if not prim.IsA(UsdPhysics.FixedJoint):
            carb.log_error(f"[joint release] Prim is not Physics FixedJoint: {joint_path}")
            return False

        joint_schema = UsdPhysics.Joint(prim)
        enabled_attr = joint_schema.GetJointEnabledAttr()
        if not enabled_attr.IsValid():
            enabled_attr = joint_schema.CreateJointEnabledAttr(True)

        if enabled_attr.Get() is False:
            carb.log_warn(f"[joint release] already disabled: {joint_path}")
            return True

        enabled_attr.Set(False)
        success = enabled_attr.Get() is False
        if success:
            carb.log_info(f"[joint release] disabled: {joint_path}")
        else:
            carb.log_error(f"[joint release] verification failed: {joint_path}")
        return success


class SimRosInterface(Node):
    """ROS callback에서는 명령을 queue에만 저장하고 USD/PhysX는 건드리지 않는다."""

    def __init__(self) -> None:
        super().__init__("netclean_standalone")

        self.sim_ready_pub = self.create_publisher(Bool, SIM_READY_TOPIC, 10)
        self.sim_reset_done_pub = self.create_publisher(
            Bool,
            SIM_RESET_DONE_TOPIC,
            10,
        )
        self.net_current_pose_pub = self.create_publisher(
            Pose,
            NET_CURRENT_POSE_TOPIC,
            10,
        )
        self.robot1_motion_done_pub = self.create_publisher(
            Bool,
            ROBOT1_MOTION_DONE_TOPIC,
            10,
        )
        self.robot2_motion_done_pub = self.create_publisher(
            Bool,
            ROBOT2_MOTION_DONE_TOPIC,
            10,
        )
        self.robot2_suction_state_pub = self.create_publisher(
            Bool,
            ROBOT2_SUCTION_STATE_TOPIC,
            10,
        )
        self.robot2_joint_release_done_pub = self.create_publisher(
            Bool,
            ROBOT2_JOINT_RELEASE_DONE_TOPIC,
            10,
        )

        self.net_targets: Deque[np.ndarray] = deque()
        self.robot1_requests: Deque[MotionRequest] = deque()
        self.robot2_requests: Deque[MotionRequest] = deque()
        self.suction_requests: Deque[bool] = deque()
        self.release_requests: Deque[str] = deque()
        self.reset_requests: Deque[bool] = deque()

        self.create_subscription(
            Bool,
            SIM_RESET_REQUEST_TOPIC,
            self._reset_request_callback,
            10,
        )

        self.create_subscription(
            Pose,
            NET_TARGET_POSE_TOPIC,
            self._net_target_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            ROBOT1_MOTION_COMMAND_TOPIC,
            self._robot1_motion_callback,
            10,
        )
        self.create_subscription(
            PoseStamped,
            ROBOT2_MOTION_COMMAND_TOPIC,
            self._robot2_motion_callback,
            10,
        )
        self.create_subscription(
            Bool,
            ROBOT2_SUCTION_COMMAND_TOPIC,
            self._suction_callback,
            10,
        )
        self.create_subscription(
            String,
            ROBOT2_RELEASE_OBJECT_TOPIC,
            self._release_callback,
            10,
        )

    def _net_target_callback(self, msg: Pose) -> None:
        position = np.array(
            [msg.position.x, msg.position.y, msg.position.z],
            dtype=float,
        )
        if np.all(np.isfinite(position)):
            self.net_targets.append(position)
        else:
            self.get_logger().error("Rejected non-finite /net/target_pose")

    def _robot1_motion_callback(self, msg: PoseStamped) -> None:
        request = self._make_motion_request(msg, "robot1")
        if request is None:
            self._publish_bool_once(self.robot1_motion_done_pub, False)
            return
        self.robot1_requests.append(request)

    def _robot2_motion_callback(self, msg: PoseStamped) -> None:
        request = self._make_motion_request(msg, "robot2")
        if request is None:
            self._publish_bool_once(self.robot2_motion_done_pub, False)
            return
        self.robot2_requests.append(request)

    def _make_motion_request(
        self,
        msg: PoseStamped,
        robot_name: str,
    ) -> Optional[MotionRequest]:
        frame_id = msg.header.frame_id.strip()
        if frame_id != WORLD_FRAME:
            self.get_logger().error(
                f"{robot_name}: frame_id must be 'world', got {frame_id!r}"
            )
            return None
        position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        if not np.all(np.isfinite(position)):
            self.get_logger().error(f"{robot_name}: target position contains NaN/Inf")
            return None
        try:
            orientation = _ros_xyzw_to_isaac_wxyz(msg.pose.orientation)
        except ValueError as exc:
            self.get_logger().error(f"{robot_name}: {exc}")
            return None
        return MotionRequest(position, orientation)

    def _suction_callback(self, msg: Bool) -> None:
        self.suction_requests.append(bool(msg.data))

    def _release_callback(self, msg: String) -> None:
        class_name = msg.data.strip().lower()
        if not class_name:
            self.get_logger().error("Rejected empty release class_name")
            self._publish_bool_once(self.robot2_joint_release_done_pub, False)
            return
        self.release_requests.append(class_name)

    def _reset_request_callback(self, msg: Bool) -> None:
        if msg.data:
            self.reset_requests.append(True)

    def clear_command_queues(self) -> None:
        """새 사이클에 이전 사이클 명령이 섞이지 않도록 모든 queue를 비운다."""
        self.net_targets.clear()
        self.robot1_requests.clear()
        self.robot2_requests.clear()
        self.suction_requests.clear()
        self.release_requests.clear()
        self.reset_requests.clear()

    @staticmethod
    def _publish_bool_once(publisher, value: bool) -> None:
        msg = Bool()
        msg.data = bool(value)
        publisher.publish(msg)


def _validate_static_config(usd_path: str) -> None:
    errors = []
    if not CONFIG_READY:
        errors.append("CONFIG_READY is False")

    required_files = {
        "USD_PATH/--usd": usd_path,
        "M0609_URDF_PATH": M0609_URDF_PATH,
        "M0609_LULA_DESCRIPTOR_PATH": M0609_LULA_DESCRIPTOR_PATH,
    }
    for name, path in required_files.items():
        if _is_todo(path):
            errors.append(f"{name} still contains TODO")
        elif not os.path.isfile(path):
            errors.append(f"{name} does not exist: {path}")

    required_strings = {
        "M0609_END_EFFECTOR_FRAME": M0609_END_EFFECTOR_FRAME,
    }
    for name, value in required_strings.items():
        if _is_todo(value):
            errors.append(f"{name} still contains TODO")

    required_prim_paths = {
        "ROBOT1_PRIM_PATH": ROBOT1_PRIM_PATH,
        "ROBOT2_PRIM_PATH": ROBOT2_PRIM_PATH,
        "NET_ROOT_PRIM_PATH": NET_ROOT_PRIM_PATH,
        "ROBOT2_SURFACE_GRIPPER_PRIM_PATH": ROBOT2_SURFACE_GRIPPER_PRIM_PATH,
        "CAMERA1_PRIM_PATH": CAMERA1_PRIM_PATH,
        "CAMERA2_PRIM_PATH": CAMERA2_PRIM_PATH,
        "ROS_CLOCK_GRAPH_PRIM_PATH": ROS_CLOCK_GRAPH_PRIM_PATH,
        "CAMERA1_GRAPH_PRIM_PATH": CAMERA1_GRAPH_PRIM_PATH,
        "CAMERA2_GRAPH_PRIM_PATH": CAMERA2_GRAPH_PRIM_PATH,
        **{f"JOINT_PATH_BY_CLASS[{key!r}]": value for key, value in JOINT_PATH_BY_CLASS.items()},
    }
    for name, path in required_prim_paths.items():
        if _is_todo(path) or not path.startswith("/"):
            errors.append(f"{name} is not a completed absolute Prim path: {path}")

    if PHYSICS_DT <= 0.0 or RENDERING_DT <= 0.0:
        errors.append("PHYSICS_DT and RENDERING_DT must be positive")
    if NET_SPEED_MPS <= 0.0 or ROBOT_JOINT_SPEED_RAD_S <= 0.0:
        errors.append("Net/robot speeds must be positive")

    if errors:
        joined = "\n  - ".join(errors)
        raise RuntimeError(f"Standalone configuration is incomplete:\n  - {joined}")


def _validate_stage(stage) -> None:
    required = {
        "Robot1 articulation": ROBOT1_PRIM_PATH,
        "Robot2 articulation": ROBOT2_PRIM_PATH,
        "NetRoot": NET_ROOT_PRIM_PATH,
        "Robot2 Surface Gripper": ROBOT2_SURFACE_GRIPPER_PRIM_PATH,
        "Camera1": CAMERA1_PRIM_PATH,
        "Camera2": CAMERA2_PRIM_PATH,
        "ROS clock graph": ROS_CLOCK_GRAPH_PRIM_PATH,
        "Camera1 graph": CAMERA1_GRAPH_PRIM_PATH,
        "Camera2 graph": CAMERA2_GRAPH_PRIM_PATH,
        **{f"{key} Fixed Joint": path for key, path in JOINT_PATH_BY_CLASS.items()},
    }
    missing = [f"{name}: {path}" for name, path in required.items() if not stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError("Missing required Stage Prims:\n  - " + "\n  - ".join(missing))

    physics_scenes = [prim for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    if not physics_scenes:
        raise RuntimeError("No Physics Scene exists in the loaded USD")

    for class_name, path in JOINT_PATH_BY_CLASS.items():
        if not stage.GetPrimAtPath(path).IsA(UsdPhysics.FixedJoint):
            raise RuntimeError(
                f"JOINT_PATH_BY_CLASS[{class_name!r}] is not a Physics FixedJoint: {path}"
            )


def _publish_net_pose(ros: SimRosInterface, controller: NetMotionController) -> None:
    position, orientation_wxyz = controller.get_world_pose()
    msg = Pose()
    msg.position.x = float(position[0])
    msg.position.y = float(position[1])
    msg.position.z = float(position[2])
    msg.orientation.x = float(orientation_wxyz[1])
    msg.orientation.y = float(orientation_wxyz[2])
    msg.orientation.z = float(orientation_wxyz[3])
    msg.orientation.w = float(orientation_wxyz[0])
    ros.net_current_pose_pub.publish(msg)


def _perform_cycle_reset(
    world: World,
    ros: SimRosInterface,
    net_controller: NetMotionController,
    robot1_controller: RobotMotionController,
    robot2_controller: RobotMotionController,
    suction_controller: SuctionController,
    joint_controller: FixedJointReleaseController,
) -> None:
    """한 공정의 런타임 상태를 USD에 저장된 최초 상태로 되돌린다."""
    carb.log_info("[NetClean] cycle reset started")

    # 이전 사이클 명령이 reset 직후 실행되지 않도록 가장 먼저 비운다.
    ros.clear_command_queues()
    robot1_controller.reset()
    robot2_controller.reset()
    suction_controller.reset()
    joint_controller.reset()

    # World.reset은 로봇 관절, NetRoot와 물리 객체를 최초/default 상태로 복귀시킨다.
    world.reset()

    # World reset 뒤 runtime handle과 목표값을 다시 초기 상태에 맞춘다.
    net_controller.reset()
    robot1_controller.reset()
    robot2_controller.reset()
    suction_controller.reset()
    joint_controller.reset()

    # Surface Gripper Manager와 PhysX constraint가 안정화될 때까지 step한다.
    max_reset_steps = max(
        WARMUP_STEPS,
        int(math.ceil(SUCTION_TIMEOUT_SEC / PHYSICS_DT)),
    )
    for step_index in range(max_reset_steps):
        world.step(render=not ARGS.headless)
        if step_index + 1 >= WARMUP_STEPS and suction_controller.is_open():
            break

    if not suction_controller.is_open():
        raise RuntimeError("Surface Gripper did not return to Open during reset")
    if not joint_controller.all_enabled():
        raise RuntimeError("One or more object Fixed Joints are disabled after reset")

    carb.log_info("[NetClean] cycle reset completed")


def main() -> int:
    world: Optional[World] = None
    ros: Optional[SimRosInterface] = None
    shutting_down = False

    def _request_shutdown(_signum=None, _frame=None) -> None:
        nonlocal shutting_down
        shutting_down = True

    signal.signal(signal.SIGINT, _request_shutdown)
    signal.signal(signal.SIGTERM, _request_shutdown)

    try:
        _validate_static_config(ARGS.usd)

        carb.log_info(f"[NetClean] opening USD: {ARGS.usd}")
        if not open_stage(ARGS.usd):
            raise RuntimeError(f"Failed to open USD: {ARGS.usd}")
        simulation_app.update()

        stage = omni.usd.get_context().get_stage()
        _validate_stage(stage)

        world = World(
            physics_dt=PHYSICS_DT,
            rendering_dt=RENDERING_DT,
            stage_units_in_meters=1.0,
        )

        robot1 = world.scene.add(
            SingleArticulation(prim_path=ROBOT1_PRIM_PATH, name="robot1_m0609")
        )
        robot2 = world.scene.add(
            SingleArticulation(prim_path=ROBOT2_PRIM_PATH, name="robot2_m0609")
        )
        net_root = world.scene.add(
            SingleXFormPrim(prim_path=NET_ROOT_PRIM_PATH, name="net_root")
        )

        world.reset()
        for _ in range(WARMUP_STEPS):
            world.step(render=not ARGS.headless)

        rclpy.init(args=None)
        ros = SimRosInterface()

        robot1_config = RobotConfig(
            name="robot1",
            prim_path=ROBOT1_PRIM_PATH,
            urdf_path=M0609_URDF_PATH,
            descriptor_path=M0609_LULA_DESCRIPTOR_PATH,
            end_effector_frame=M0609_END_EFFECTOR_FRAME,
            flange_to_tcp_position=ROBOT1_FLANGE_TO_TCP_POSITION,
            flange_to_tcp_orientation_wxyz=ROBOT1_FLANGE_TO_TCP_QUATERNION_WXYZ,
            motion_done_topic=ROBOT1_MOTION_DONE_TOPIC,
        )
        robot2_config = RobotConfig(
            name="robot2",
            prim_path=ROBOT2_PRIM_PATH,
            urdf_path=M0609_URDF_PATH,
            descriptor_path=M0609_LULA_DESCRIPTOR_PATH,
            end_effector_frame=M0609_END_EFFECTOR_FRAME,
            flange_to_tcp_position=ROBOT2_FLANGE_TO_TCP_POSITION,
            flange_to_tcp_orientation_wxyz=ROBOT2_FLANGE_TO_TCP_QUATERNION_WXYZ,
            motion_done_topic=ROBOT2_MOTION_DONE_TOPIC,
        )

        net_controller = NetMotionController(net_root, NET_SPEED_MPS)
        robot1_controller = RobotMotionController(
            robot1,
            robot1_config,
            ros.robot1_motion_done_pub,
        )
        robot2_controller = RobotMotionController(
            robot2,
            robot2_config,
            ros.robot2_motion_done_pub,
        )
        suction_controller = SuctionController(
            ROBOT2_SURFACE_GRIPPER_PRIM_PATH,
            ros.robot2_suction_state_pub,
        )
        joint_controller = FixedJointReleaseController(
            stage,
            JOINT_PATH_BY_CLASS,
            suction_controller,
            ros.robot2_joint_release_done_pub,
        )

        ready_period = 1.0 / READY_PUBLISH_HZ
        pose_period = 1.0 / NET_CURRENT_POSE_HZ
        last_ready_publish = -math.inf
        last_pose_publish = -math.inf
        simulated_time = 0.0
        sim_ready = True

        carb.log_info("[NetClean] standalone READY; waiting for ROS commands")

        while simulation_app.is_running() and not shutting_down:
            world.step(render=not ARGS.headless)
            simulated_time += PHYSICS_DT

            # callback 안에서는 queue에만 적재한다.
            rclpy.spin_once(ros, timeout_sec=0.0)

            # EXIT 도착 후 control_node가 보낸 reset 요청을 최우선으로 처리한다.
            if ros.reset_requests:
                sim_ready = False
                ros._publish_bool_once(ros.sim_ready_pub, False)
                try:
                    _perform_cycle_reset(
                        world,
                        ros,
                        net_controller,
                        robot1_controller,
                        robot2_controller,
                        suction_controller,
                        joint_controller,
                    )
                except Exception as exc:
                    carb.log_error(f"[NetClean] cycle reset failed: {exc}")
                    ros._publish_bool_once(ros.sim_reset_done_pub, False)
                else:
                    sim_ready = True
                    ros._publish_bool_once(ros.sim_reset_done_pub, True)
                    _publish_net_pose(ros, net_controller)
                    last_ready_publish = -math.inf
                    last_pose_publish = simulated_time
                continue

            # queue를 실제 simulation controller로 넘긴다.
            while ros.net_targets:
                net_controller.set_target(ros.net_targets.popleft())
            while ros.robot1_requests:
                robot1_controller.enqueue(ros.robot1_requests.popleft())
            while ros.robot2_requests:
                robot2_controller.enqueue(ros.robot2_requests.popleft())
            while ros.suction_requests:
                suction_controller.enqueue(ros.suction_requests.popleft())
            while ros.release_requests:
                joint_controller.enqueue(ros.release_requests.popleft())

            net_controller.update(PHYSICS_DT)
            robot1_controller.update(PHYSICS_DT)
            robot2_controller.update(PHYSICS_DT)
            suction_controller.update()
            joint_controller.update()

            if simulated_time - last_pose_publish >= pose_period:
                _publish_net_pose(ros, net_controller)
                last_pose_publish = simulated_time

            # 현재 control_node는 volatile QoS이므로 late join도 받을 수 있게 반복 발행한다.
            if simulated_time - last_ready_publish >= ready_period:
                ros._publish_bool_once(ros.sim_ready_pub, sim_ready)
                last_ready_publish = simulated_time

        return 0

    except Exception as exc:
        carb.log_error(f"[NetClean] fatal error: {exc}")
        return 1

    finally:
        if ros is not None:
            try:
                ros._publish_bool_once(ros.sim_ready_pub, False)
                ros.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()
        if world is not None:
            try:
                world.stop()
                world.clear_instance()
            except Exception:
                pass
        simulation_app.close()


if __name__ == "__main__":
    sys.exit(main())
