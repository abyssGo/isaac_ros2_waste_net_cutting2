#!/usr/bin/env python3
"""NetClean Isaac Sim 5.1 standalone execution layer.

역할
----
1. 저장된 netclean_world.usd를 GUI로 연다.
2. World/Physics를 계속 step한다.
3. /net/target_pose를 받아 실제 net_green + carriage 4개를 함께 보간 이동한다.
4. Robot1/2의 world TCP Pose 명령을 Lula IK로 joint 목표로 바꿔 보간한다.
5. Robot2 suction 명령을 Runtime FixedJoint 방식으로 처리한다.
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
- 2026-08-26 오전 확인한 실제 USD/Robot/Camera/TCP/Joint 경로를 반영했다.
- 랜덤 스폰, 동적 registry, YOLO, 공정 FSM은 포함하지 않는다.
- 현재 robot2_node.py에 맞춰 class_name 기반 Joint 해제 방식을 사용한다.

v2 변경 사항 (로봇1/2 -v2 노드와 맞물려 동작하도록 수정, 토픽/메시지 변경 없음)
------------------------------------------------------------------
RuntimeSuctionController.attach_class()의 접촉 거리 판정 버그 수정.
기존에는 link_6 원점(ROBOT2_SUCTION_BODY_PRIM_PATH) 기준으로 물체와의 거리를
쟀는데, 실제 접촉면은 거기서 ROBOT2_FLANGE_TO_TCP_POSITION(~12.2cm)만큼 떨어진
SuctionTCP다. 그래서 TCP가 물체에 완벽히 닿아도 거리 계산은 항상 ~12cm로 나와
접촉 허용값을 항상 초과해 attach가 100% 실패하고 있었다.
(이 때문에 SUCTION_ON은 성공하지만 JOINT_RELEASE가 매번 실패해 로봇2 Action이
abort되는 증상이 재현됨.) 접촉 판정 기준점을 SuctionTCP로 바꿨다. Runtime
FixedJoint의 body0은 물리적으로 올바른 link_6를 유지하되, body0/body1의 local
joint frame은 생성 순간의 SuctionTCP world pose에서 서로 일치하도록 계산한다.
로봇1/2 노드에 추가된 재시도·복구 로직은 기존 PoseStamped를 그대로 재사용하는
방식이라 이 standalone과의 토픽 인터페이스에는 영향이 없다.
"""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
import os
from pathlib import Path
import signal
import sys
import time
from typing import Deque, Dict, Optional, Sequence, Tuple

import numpy as np


# =============================================================================
# 사용자 입력: GUI 월드를 만드는 동안 최종적으로 확보해야 하는 정보
# =============================================================================

# 실제 월드 확인값 반영 완료. Stage validation을 통과해야만 실행된다.
CONFIG_READY = True


# 1) 저장소에 포함된 최종 USD와 관련 에셋 경로
REPO_ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = REPO_ROOT / "sim" / "assets" / "project1"
USD_PATH = str(ASSET_ROOT / "simulation_integration_v3.usd")


# 2) 월드 안의 필수 Prim 경로
# Stage 창에서 Prim을 우클릭하여 Copy Prim Path로 복사한다.
#
# ROBOT*_PRIM_PATH는 M0609의 Articulation Root가 붙은 Prim이어야 한다.
# NET_ROOT_PRIM_PATH는 ROS에 publish할 logical anchor다.
# 실제 이동은 NET_PHYSICAL_RIGID_PRIM_PATH(net_green) + carriage follower가 담당한다.
ROBOT1_PRIM_PATH = "/World/robot1"
ROBOT2_PRIM_PATH = "/World/robot2/m0609"
NET_ROOT_PRIM_PATH = "/World/collected_net"


# 3) USD에 저장해 둔 Camera와 ROS Action Graph Prim 경로
# Standalone은 카메라 토픽이나 /clock을 직접 publish하지 않는다.
# 아래 Prim들이 USD에 존재하는지만 검사하고, 실제 publish는 저장된 Action Graph가 한다.
CAMERA1_PRIM_PATH = "/World/AG_Camera1/RSD455/Camera_OmniVision_OV9782_Color"
CAMERA2_PRIM_PATH = "/World/AG_Camera2/RSD455/Camera_OmniVision_OV9782_Color"
ROS_CLOCK_GRAPH_PRIM_PATH = "/ROSGraphs/Clock"
CAMERA1_GRAPH_PRIM_PATH = "/ROSGraphs/AG_Camera1"
CAMERA2_GRAPH_PRIM_PATH = "/ROSGraphs/AG_Camera2"


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
M0609_URDF_PATH = str(ASSET_ROOT / "robot2_sample" / "m0609_isaac_sim.urdf")
M0609_LULA_DESCRIPTOR_PATH = str(ASSET_ROOT / "robot2_sample" / "m0609_description.yaml")
M0609_END_EFFECTOR_FRAME = "link_6"


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
ROBOT1_FLANGE_TO_TCP_POSITION = (0.0, 0.0, 0.13)  # /World/robot1/link_6/CutterTCP
ROBOT1_FLANGE_TO_TCP_QUATERNION_WXYZ = (1.0, 0.0, 0.0, 0.0)

ROBOT2_FLANGE_TO_TCP_POSITION = (-0.00113215, -0.00004321, 0.12225925)  # SuctionTCP
ROBOT2_FLANGE_TO_TCP_QUATERNION_WXYZ = (1.0, 0.0, 0.0, 0.0)


# 6) class_name -> Fixed Joint Prim 경로
# 현재 확정 조건:
# - 작업 중인 어망 한 장에는 클래스별 객체가 최대 한 개다.
# - 랜덤 스폰하지 않는다.
# - robot2_node가 plastic_bottle/can/buoy 문자열을 보낸다.
#
# 반드시 실제 "Physics Fixed Joint" Prim 경로를 넣는다.
# PET/CAN/BUOY Mesh Prim 경로가 아니다.
JOINT_PATH_BY_CLASS: Dict[str, str] = {
    "plastic_bottle": "/World/collected_net/pet_bottle/FixedJoint",
    "can": "/World/collected_net/can/FixedJoint",
    "buoy": "/World/collected_net/buoy/FixedJoint",
}


# 7) 시간과 허용오차
# 아래 값은 일단 시작 가능한 보수적 기본값이다. 월드 완성 후 실제 시연으로 조정한다.
PHYSICS_DT = 1.0 / 60.0
RENDERING_DT = 1.0 / 60.0
WARMUP_STEPS = 30

NET_SPEED_MPS = 0.5
NET_CURRENT_POSE_HZ = 20.0
READY_PUBLISH_HZ = 1.0

ROBOT_JOINT_SPEED_RAD_S = 0.35
ROBOT_MIN_MOTION_DURATION_SEC = 0.30
ROBOT_JOINT_TOLERANCE_RAD = math.radians(1.0)
ROBOT_MOTION_TIMEOUT_SEC = 40.0

SUCTION_TIMEOUT_SEC = 5.0
# Runtime FixedJoint 생성 전 suction TCP-object *표면* 접촉 허용 거리.
# RGB-D/외부 파라미터/물리 step 오차를 포함하므로 1 cm는 지나치게 엄격하다.
SUCTION_CONTACT_TOLERANCE_M = 0.03

# 물체를 SuctionTCP로 순간이동시키는 진단 모드는 최종 공정에서 금지한다.
# Robot2가 실제 물체 표면까지 이동한 뒤에만 접촉 거리 검사를 통과하고 Joint가
# 생성되어야 한다.
DIRECT_SUCTION_SNAP_TO_TCP = False


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

# =============================================================================
# 2026-08-26 오전 실제 월드에서 확인한 참고값
# =============================================================================
# 이 값들은 현재 협업자 원본 코드가 직접 사용하지 않는 항목도 있지만,
# 단일파일 v2에서 Transport/Reset/Home 검증을 붙일 때 기준값으로 사용한다.
NET_PHYSICAL_RIGID_PRIM_PATH = "/World/collected_net/World4/net_collected/net_green"
CARRIAGE_PRIM_PATHS = (
    "/World/collected_net/World4/net_collected/carriage_TL",
    "/World/collected_net/World4/net_collected/carriage_TR",
    "/World/collected_net/World4/net_collected/carriage_BL",
    "/World/collected_net/World4/net_collected/carriage_BR",
)

NET_SPAWN_XYZ = (-0.7, -5.0, 0.71)
NET_WAIT_XYZ = (-0.7, -4.0, 0.71)
NET_STATION1_XYZ = (-0.7, -2.0, 0.71)
NET_STATION2_XYZ = (-0.7,  2.0, 0.71)
NET_EXIT_XYZ = (-0.7,  4.0, 0.71)

SAFE_HOME_JOINTS_RAD = (
    -math.pi / 2.0,
    -math.pi / 2.0,
     math.pi / 2.0,
     0.0,
     0.0,
     0.0,
)

ROBOT1_HOME_POSITION_WORLD = np.asarray(
    [0.018859214318574534, -1.601662366502892, 1.2578182164041043],
    dtype=float,
)
ROBOT2_HOME_POSITION_WORLD = np.asarray(
    [-0.007563448984626552, 2.4009529933486675, 1.2495370123800529],
    dtype=float,
)
HOME_POSITION_TOLERANCE_M = 0.01

ROBOT1_TCP_PRIM_PATH = "/World/robot1/link_6/CutterTCP"
ROBOT2_TCP_PRIM_PATH = "/World/robot2/m0609/link_6/SuctionTCP"

# Runtime suction은 Surface Gripper Schema를 사용하지 않는다.
ROBOT2_SUCTION_BODY_PRIM_PATH = "/World/robot2/m0609/link_6"
RUNTIME_ROOT_PRIM_PATH = "/World/NetCleanRuntime"
RUNTIME_SUCTION_JOINT_PATH = "/World/NetCleanRuntime/Robot2SuctionJoint"

NET_TARGET_TOLERANCE_M = 0.005

ROBOT_STIFFNESS = (160.0, 4000.0, 2500.0, 682.2491, 100.0, 11.111)
ROBOT_DAMPING = (4.5, 42.0, 90.0, 38.206, 3.0, 0.622)
ROBOT_MAX_EFFORT = (163.0, 163.0, 96.0, 50.0, 50.0, 50.0)
APPLY_RUNTIME_GAINS = True
APPLY_RUNTIME_EFFORT_LIMITS = False


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
from isaacsim.core.prims import SingleArticulation, SingleRigidPrim, SingleXFormPrim
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import is_stage_loading, open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


# ROS 2 Bridge만 명시적으로 활성화한다.
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
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

class NetTransportController:
    """logical Net pose와 실제 net_green/carriage 이동을 분리한다."""

    def __init__(self, logical_root, physical_net, carriages, speed_mps):
        self._logical_root = logical_root
        self._physical_net = physical_net
        self._carriages = list(carriages)
        self._speed_mps = float(speed_mps)
        logical_pos, logical_ori = self._logical_root.get_world_pose()
        logical_pos = np.asarray(logical_pos, dtype=float)
        self._orientation = np.asarray(logical_ori, dtype=float)
        physical_pos, physical_ori = self._physical_net.get_world_pose()
        self._physical_offset = np.asarray(physical_pos, dtype=float) - logical_pos
        self._physical_orientation = np.asarray(physical_ori, dtype=float)
        self._carriage_offsets = []
        self._carriage_orientations = []
        for carriage in self._carriages:
            pos, ori = carriage.get_world_pose()
            self._carriage_offsets.append(np.asarray(pos, dtype=float) - logical_pos)
            self._carriage_orientations.append(np.asarray(ori, dtype=float))
        self._current_position = logical_pos.copy()
        self._target_position = logical_pos.copy()
        print(f"[NetClean] transport initialized: physical_net_offset={self._physical_offset.round(6).tolist()}, carriages={len(self._carriages)}", flush=True)

    def set_target(self, position_world):
        target = np.asarray(position_world, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError("Net target position must be three finite values")
        self._target_position = target.copy()

    def update(self, dt):
        delta = self._target_position - self._current_position
        distance = float(np.linalg.norm(delta))
        if distance > 1.0e-9:
            step_distance = min(distance, self._speed_mps * float(dt))
            self._current_position += delta * (step_distance / distance)
        self._apply_pose()

    def _apply_pose(self):
        self._physical_net.set_world_pose(position=self._current_position + self._physical_offset, orientation=self._physical_orientation)
        for carriage, offset, orientation in zip(self._carriages, self._carriage_offsets, self._carriage_orientations):
            carriage.set_world_pose(position=self._current_position + offset, orientation=orientation)

    def get_world_pose(self):
        return self._current_position.copy(), self._orientation.copy()

    def at_target(self, tolerance_m=NET_TARGET_TOLERANCE_M):
        return float(np.linalg.norm(self._target_position - self._current_position)) <= float(tolerance_m)

    def reset_to_spawn(self):
        self._current_position = np.asarray(NET_SPAWN_XYZ, dtype=float)
        self._target_position = self._current_position.copy()
        self._apply_pose()

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


        # Robot2 보간 종료 후 IK 최종 관절값을 정확히 적용한다.
        # SuctionTCP와 물체 중심 사이에 남는 관절 추종 오차를 제거한다.
        if ratio >= 1.0:
            self._robot.set_joint_positions(
                self._target_joint_positions,
                joint_indices=self._indices,
            )

        current = np.asarray(
            self._robot.get_joint_positions(joint_indices=self._indices),
            dtype=float,
        )
        error = float(np.max(np.abs(current - self._target_joint_positions)))

        if ratio >= 1.0 and error <= ROBOT_JOINT_TOLERANCE_RAD:
            self._active = False

            try:
                stage = omni.usd.get_context().get_stage()
                tcp_path = (
                    ROBOT1_TCP_PRIM_PATH
                    if self._config.name == "robot1"
                    else ROBOT2_TCP_PRIM_PATH
                )
                tcp_pos, _ = _get_stage_world_pose(stage, tcp_path)

                print(
                    f"[TCP ACTUAL] {self._config.name} "
                    f"xyz={tcp_pos.round(4).tolist()}",
                    flush=True,
                )
            except Exception as exc:
                print(
                    f"[TCP ACTUAL] read failed: {exc}",
                    flush=True,
                )

            self._publish_result(True)
            return

        if time.monotonic() - self._start_time > ROBOT_MOTION_TIMEOUT_SEC:
            joint_error = current - self._target_joint_positions

            print(
                f"[JOINT TIMEOUT] {self._config.name}\n"
                f"  current = {current.round(4).tolist()}\n"
                f"  target  = {self._target_joint_positions.round(4).tolist()}\n"
                f"  error   = {joint_error.round(4).tolist()}",
                flush=True,
            )

            carb.log_error(
                f"[{self._config.name}] motion timeout; "
                f"max joint error={error:.4f} rad"
            )
            self._active = False
            self._publish_result(False)

    def _begin_request(self, request: MotionRequest) -> bool:
        # Home 복귀는 Cartesian IK를 사용하지 않고 Joint 목표로 이동한다.
        # Home은 작업 TCP 정밀 위치가 아니라 반복 시작/종료 안전 자세이므로
        # IK branch 선택 문제를 피한다.
        home_position = (
            ROBOT1_HOME_POSITION_WORLD
            if self._config.name == "robot1"
            else ROBOT2_HOME_POSITION_WORLD
        )
        is_home_request = (
            float(np.linalg.norm(request.position_world - home_position))
            <= HOME_POSITION_TOLERANCE_M
        )

        if is_home_request:
            target_positions = np.asarray(
                SAFE_HOME_JOINTS_RAD,
                dtype=float,
            )
            indices = np.arange(len(SAFE_HOME_JOINTS_RAD), dtype=np.int64)
            success = True
        else:
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
                self._log_ik_failure(
                    request,
                    reason=f"IK exception: {exc}",
                )
                carb.log_error(f"[{self._config.name}] IK exception: {exc}")
                return False

            if target_action is None:
                self._log_ik_failure(
                    request,
                    reason="IK returned target_action=None",
                    flange_position=flange_position,
                    flange_orientation=flange_orientation,
                )
                carb.log_error(
                    f"[{self._config.name}] IK returned target_action=None"
                )
                return False

            if not success or target_action.joint_positions is None:
                self._log_ik_failure(
                    request,
                    reason=(
                        f"solver_success={bool(success)}, "
                        "joint_positions="
                        f"{'set' if target_action.joint_positions is not None else 'None'}"
                    ),
                    flange_position=flange_position,
                    flange_orientation=flange_orientation,
                )
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

        raw_start_positions = self._robot.get_joint_positions(
            joint_indices=indices
        )

        if raw_start_positions is None:
            carb.log_error(
                f"[{self._config.name}] Articulation joint positions are None "
                f"(robot is not initialized)"
            )
            return False

        start_positions = np.asarray(
            raw_start_positions,
            dtype=float,
        )

        max_delta = float(np.max(np.abs(target_positions - start_positions)))

        print(
            f"[NetClean IK] {self._config.name} SUCCESS\n"
            f"  start  = {start_positions.round(4).tolist()}\n"
            f"  target = {target_positions.round(4).tolist()}\n"
            f"  max_delta = {max_delta:.4f} rad",
            flush=True,
        )
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

    def _log_ik_failure(
        self,
        request: MotionRequest,
        *,
        reason: str,
        flange_position=None,
        flange_orientation=None,
    ) -> None:
        """IK 실패 시 좌표계/도달성 원인을 한 로그에서 판별 가능하게 한다."""
        lines = [
            f"[NetClean IK] {self._config.name} FAILED",
            f"  reason = {reason}",
            f"  requested_tcp_xyz = "
            f"{request.position_world.round(4).tolist()}",
            f"  requested_tcp_quat_wxyz = "
            f"{request.orientation_world_wxyz.round(5).tolist()}",
        ]

        if flange_position is not None:
            lines.append(
                f"  converted_flange_xyz = "
                f"{np.asarray(flange_position).round(4).tolist()}"
            )
        if flange_orientation is not None:
            lines.append(
                f"  converted_flange_quat_wxyz = "
                f"{np.asarray(flange_orientation).round(5).tolist()}"
            )

        try:
            base_position, _ = self._robot.get_world_pose()
            base_position = np.asarray(base_position, dtype=float)
            distance = float(
                np.linalg.norm(request.position_world - base_position)
            )
            lines.append(f"  robot_base_xyz = {base_position.round(4).tolist()}")
            lines.append(f"  tcp_target_distance_from_base = {distance:.4f} m")
        except Exception as exc:
            lines.append(f"  robot_base_read_error = {exc}")

        try:
            stage = omni.usd.get_context().get_stage()
            tcp_path = (
                ROBOT1_TCP_PRIM_PATH
                if self._config.name == "robot1"
                else ROBOT2_TCP_PRIM_PATH
            )
            current_tcp, _ = _get_stage_world_pose(stage, tcp_path)
            lines.append(
                f"  current_tcp_xyz = {current_tcp.round(4).tolist()}"
            )
        except Exception as exc:
            lines.append(f"  current_tcp_read_error = {exc}")

        try:
            current_joints = np.asarray(
                self._robot.get_joint_positions(),
                dtype=float,
            )
            lines.append(
                f"  current_joints = {current_joints.round(4).tolist()}"
            )
        except Exception as exc:
            lines.append(f"  current_joints_read_error = {exc}")

        print("\n".join(lines), flush=True)

    def _publish_result(self, success: bool) -> None:
        msg = Bool()
        msg.data = bool(success)
        # 명령 하나당 정확히 한 번만 발행한다.
        self._result_publisher.publish(msg)

class RuntimeSuctionController:
    """VG10 suction: TCP 기준 virtual attach 방식."""

    def __init__(
        self,
        stage,
        object_body_path_by_class,
        object_wrapper_by_class,
        state_publisher,
    ):
        self._stage = stage
        self._object_body_paths = dict(object_body_path_by_class)
        self._object_wrappers = dict(object_wrapper_by_class)
        self._publisher = state_publisher
        self._requests = deque()

        self._armed = False
        self._attached_class = None
        self._attached_offset = None
        self._attached_rotation_offset = None
        self._attached_object_wrapper = None

    def enqueue(self, enabled):
        self._requests.append(bool(enabled))

    def is_closed(self):
        return bool(self._armed)

    def is_open(self):
        return not self._armed

    def reset(self):
        self._requests.clear()
        self._armed = False
        self._attached_class = None
        self._attached_offset = None
        self._attached_rotation_offset = None
        self._attached_object_wrapper = None

    def update(self):
        if self._requests:
            enabled = self._requests.popleft()

            if enabled:
                self._armed = True
                self._publish_state(True)
                carb.log_info("[robot2 suction] armed")
            else:
                self._armed = False
                self._attached_class = None
                self._attached_offset = None
                self._attached_rotation_offset = None
                self._attached_object_wrapper = None
                self._publish_state(False)
                carb.log_info("[robot2 suction] released")

        if (
            self._attached_class is not None
            and self._attached_offset is not None
            and self._attached_object_wrapper is not None
        ):
            tcp_pos, tcp_q = _get_stage_world_pose(
                self._stage,
                ROBOT2_TCP_PRIM_PATH,
            )

            object_pos = tcp_pos + self._attached_offset

            object_q = _quaternion_multiply_wxyz(
                tcp_q,
                self._attached_rotation_offset,
            )

            self._attached_object_wrapper.set_world_pose(
                position=object_pos,
                orientation=object_q,
            )

            try:
                self._attached_object_wrapper.set_linear_velocity(np.zeros(3))
                self._attached_object_wrapper.set_angular_velocity(np.zeros(3))
            except Exception:
                pass

    def attach_class(self, class_name):
        class_name = class_name.strip().lower()

        if not self._armed:
            carb.log_error("[robot2 suction] attach rejected: suction is not armed")
            return False

        object_body_path = self._object_body_paths.get(class_name)

        if object_body_path is None:
            carb.log_error(f"[robot2 suction] no body mapping for {class_name!r}")
            return False

        try:
            tcp_pos, tcp_q = _get_stage_world_pose(
                self._stage,
                ROBOT2_TCP_PRIM_PATH,
            )

            (
                surface_distance,
                closest_surface_point,
                _,
                _,
            ) = _distance_to_prim_world_bounds(
                self._stage,
                object_body_path,
                tcp_pos,
            )

            if surface_distance > SUCTION_CONTACT_TOLERANCE_M:
                carb.log_error(
                    f"[robot2 suction] attach rejected: surface_distance={surface_distance:.4f}m"
                )
                return False

            # TCP-접촉면 offset이 아니라 실제 RigidBody 원점과 TCP 사이의
            # 상대 위치를 저장한다.
            # update()에서 set_world_pose()는 RigidBody 원점 위치를 넣기 때문에
            # 물체 중심(origin) 기준 offset이어야 한다.
            object_pos, _ = _get_stage_world_pose(
                self._stage,
                object_body_path,
            )

            self._attached_offset = object_pos - tcp_pos

            # TCP 기준 물체 회전 상대값 저장
            object_q = _get_stage_world_pose(
                self._stage,
                object_body_path,
            )[1]

            self._attached_rotation_offset = _quaternion_multiply_wxyz(
                _quaternion_inverse_wxyz(tcp_q),
                object_q,
            )

            self._attached_object_wrapper = self._object_wrappers[class_name]
            self._attached_class = class_name

            carb.log_info(
                f"[robot2 suction] virtual attach success class={class_name}, "
                f"surface_distance={surface_distance:.4f}m"
            )

            return True

        except Exception as exc:
            carb.log_error(f"[robot2 suction] virtual attach failed: {exc}")
            return False

    def detach_runtime_joint(self):
        self._attached_class = None
        self._attached_offset = None
        self._attached_rotation_offset = None
        self._attached_object_wrapper = None

    def _remove_runtime_joint(self):
        if self._stage.GetPrimAtPath(RUNTIME_SUCTION_JOINT_PATH).IsValid():
            self._stage.RemovePrim(RUNTIME_SUCTION_JOINT_PATH)

    def _publish_state(self, value):
        msg = Bool()
        msg.data = bool(value)
        self._publisher.publish(msg)

class FixedJointReleaseController:
    """class_name의 Net FixedJoint를 끊고 Runtime suction joint로 ownership을 넘긴다."""

    def __init__(self, stage, joint_path_by_class, suction_controller, result_publisher):
        self._stage = stage
        self._paths = dict(joint_path_by_class)
        self._suction = suction_controller
        self._publisher = result_publisher
        self._requests = deque()

    def enqueue(self, class_name):
        self._requests.append(class_name.strip().lower())

    def reset(self):
        self._requests.clear()
        failures = []
        for class_name, joint_path in self._paths.items():
            prim = self._stage.GetPrimAtPath(joint_path)
            if not prim.IsValid() or not prim.IsA(UsdPhysics.FixedJoint):
                failures.append(f"{class_name}: {joint_path}"); continue
            joint_schema = UsdPhysics.Joint(prim)
            enabled_attr = joint_schema.GetJointEnabledAttr()
            if not enabled_attr.IsValid(): enabled_attr = joint_schema.CreateJointEnabledAttr(True)
            enabled_attr.Set(True)
            if enabled_attr.Get() is not True: failures.append(f"{class_name}: {joint_path}")
        if failures: raise RuntimeError("Failed to restore Fixed Joints: " + ", ".join(failures))

    def all_enabled(self):
        for joint_path in self._paths.values():
            prim = self._stage.GetPrimAtPath(joint_path)
            if not prim.IsValid() or not prim.IsA(UsdPhysics.FixedJoint): return False
            enabled_attr = UsdPhysics.Joint(prim).GetJointEnabledAttr()
            if not enabled_attr.IsValid() or enabled_attr.Get() is not True: return False
        return True

    def update(self):
        if not self._requests: return
        class_name = self._requests.popleft()
        try:
            success = self._release(class_name)
        except Exception as exc:
            self._suction.detach_runtime_joint()
            carb.log_error(
                f"[joint release] unexpected failure for {class_name!r}: {exc}"
            )
            success = False
        msg = Bool(); msg.data = bool(success); self._publisher.publish(msg)

    def _release(self, class_name):
        joint_path = self._paths.get(class_name)
        if joint_path is None:
            carb.log_error(f"[joint release] unknown class_name={class_name!r}"); return False
        if not self._suction.is_closed():
            carb.log_error("[joint release] rejected because suction is not armed"); return False
        prim = self._stage.GetPrimAtPath(joint_path)
        if not prim.IsValid() or not prim.IsA(UsdPhysics.FixedJoint):
            carb.log_error(f"[joint release] invalid FixedJoint: {joint_path}"); return False
        if not self._suction.attach_class(class_name): return False
        joint_schema = UsdPhysics.Joint(prim)
        enabled_attr = joint_schema.GetJointEnabledAttr()
        if not enabled_attr.IsValid(): enabled_attr = joint_schema.CreateJointEnabledAttr(True)
        if enabled_attr.Get() is False: return True
        enabled_attr.Set(False)
        success = enabled_attr.Get() is False
        if not success:
            self._suction.detach_runtime_joint()
            carb.log_error(f"[joint release] verification failed: {joint_path}")
            return False
        carb.log_info(f"[joint release] disabled after suction attach: {joint_path}")
        return True

class SimRosInterface(Node):
    """ROS callback에서는 명령을 queue에만 저장하고 USD/PhysX는 건드리지 않는다."""

    def __init__(self) -> None:
        super().__init__("netclean_standalone")

        ready_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.sim_ready_pub = self.create_publisher(
            Bool,
            SIM_READY_TOPIC,
            ready_qos,
        )
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

def _get_stage_world_pose(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid(): raise RuntimeError(f"Missing Prim for world pose: {prim_path}")
    matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(prim)
    translation = matrix.ExtractTranslation()
    quat = matrix.ExtractRotationQuat(); imag = quat.GetImaginary()
    position = np.array([float(translation[0]), float(translation[1]), float(translation[2])], dtype=float)
    orientation = _normalize_quaternion_wxyz([float(quat.GetReal()), float(imag[0]), float(imag[1]), float(imag[2])], f"{prim_path} world orientation")
    return position, orientation


def _distance_to_prim_world_bounds(stage, prim_path, point_world):
    """Return point-to-object world AABB distance and diagnostic geometry.

    ``body1``의 원점이 아니라 현재 렌더/물리 객체의 월드 바운딩 표면을
    기준으로 흡착 접촉을 판정한다. BBoxCache는 호출 시마다 새로 만들어
    PhysX가 갱신한 현재 transform을 사용한다.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        raise RuntimeError(f"Missing Prim for world bounds: {prim_path}")

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy,
        ],
        useExtentsHint=True,
    )
    aligned_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    bounds_min = np.asarray(aligned_range.GetMin(), dtype=float)
    bounds_max = np.asarray(aligned_range.GetMax(), dtype=float)
    point = np.asarray(point_world, dtype=float)

    if (
        point.shape != (3,)
        or not np.all(np.isfinite(point))
        or not np.all(np.isfinite(bounds_min))
        or not np.all(np.isfinite(bounds_max))
        or np.any(bounds_max < bounds_min)
    ):
        raise RuntimeError(f"Invalid world bounds for suction object: {prim_path}")

    closest = np.clip(point, bounds_min, bounds_max)
    distance = float(np.linalg.norm(point - closest))
    return distance, closest, bounds_min, bounds_max


def _prepare_transport_stage(stage):
    net_prim = stage.GetPrimAtPath(NET_PHYSICAL_RIGID_PRIM_PATH)
    if not net_prim.IsValid(): raise RuntimeError(f"Missing physical net Prim: {NET_PHYSICAL_RIGID_PRIM_PATH}")
    rigid_api = UsdPhysics.RigidBodyAPI(net_prim) if net_prim.HasAPI(UsdPhysics.RigidBodyAPI) else UsdPhysics.RigidBodyAPI.Apply(net_prim)
    attr = rigid_api.GetKinematicEnabledAttr()
    if not attr.IsValid(): attr = rigid_api.CreateKinematicEnabledAttr(True)
    attr.Set(True)
    print(f"[NetClean] physical net KINEMATIC: {NET_PHYSICAL_RIGID_PRIM_PATH}", flush=True)
    for carriage_path in CARRIAGE_PRIM_PATHS:
        root = stage.GetPrimAtPath(carriage_path)
        if not root.IsValid(): raise RuntimeError(f"Missing carriage Prim: {carriage_path}")
        collision_off = joint_off = 0
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                api = UsdPhysics.CollisionAPI(prim); a = api.GetCollisionEnabledAttr()
                if not a.IsValid(): a = api.CreateCollisionEnabledAttr(True)
                a.Set(False); collision_off += 1
            if prim.IsA(UsdPhysics.FixedJoint):
                joint = UsdPhysics.Joint(prim); a = joint.GetJointEnabledAttr()
                if not a.IsValid(): a = joint.CreateJointEnabledAttr(True)
                a.Set(False); joint_off += 1
        print(f"[NetClean] carriage visual follower: {carriage_path} (collision_off={collision_off}, joint_off={joint_off})", flush=True)


def _nearest_rigid_body_path(stage, relationship_target):
    """Resolve a Joint target to its nearest RigidBodyAPI ancestor.

    일부 저장된 Joint는 RigidBodyAPI Prim 자체가 아니라 그 아래 mesh/collider
    Prim을 relationship target으로 갖는다. Runtime suction용 body 경로를 만들 때
    해당 자식 경로에서 부모 방향으로 올라가 가장 가까운 강체를 사용한다.
    """
    current = Sdf.Path(relationship_target)
    # Isaac Sim 5.1에 포함된 일부 pxr Python 바인딩에는 Sdf.Path.IsEmpty()가
    # 노출되지 않는다. 빈 경로와 pseudo-root는 문자열 값으로 판정해 버전별
    # API 차이 없이 부모 방향으로 순회한다.
    while str(current) not in ("", "/"):
        prim = stage.GetPrimAtPath(current)
        if prim.IsValid() and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return current
        parent = current.GetParentPath()
        if parent == current:
            break
        current = parent
    return None


def _object_body_paths_from_joint_map(stage, mapping):
    result = {}
    for class_name, joint_path in mapping.items():
        prim = stage.GetPrimAtPath(joint_path)
        if not prim.IsValid() or not prim.IsA(UsdPhysics.FixedJoint): raise RuntimeError(f"{class_name}: invalid FixedJoint {joint_path}")
        joint_schema = UsdPhysics.Joint(prim)
        side_targets = {
            "body0": list(joint_schema.GetBody0Rel().GetTargets()),
            "body1": list(joint_schema.GetBody1Rel().GetTargets()),
        }
        if any(len(targets) > 1 for targets in side_targets.values()):
            raise RuntimeError(
                f"{class_name}: FixedJoint body relationship has multiple targets: "
                f"{joint_path}"
            )

        joint_parent = Sdf.Path(joint_path).GetParentPath()
        net_body = Sdf.Path(NET_PHYSICAL_RIGID_PRIM_PATH)
        candidates = []
        for side, targets in side_targets.items():
            if not targets:
                # 빈 relationship는 static world를 뜻할 수 있다.
                continue
            relationship_target = targets[0]
            target_prim = stage.GetPrimAtPath(relationship_target)
            if not target_prim.IsValid():
                raise RuntimeError(
                    f"{class_name}: missing {side} target "
                    f"{relationship_target} at {joint_path}"
                )
            rigid_body_path = _nearest_rigid_body_path(
                stage,
                relationship_target,
            )
            if rigid_body_path is None:
                raise RuntimeError(
                    f"{class_name}: {side} target has no RigidBody ancestor: "
                    f"{relationship_target}"
                )

            # Joint가 객체 Prim 아래에 있으므로 그 부모와 가장 가까운 Body가
            # 쓰레기다. 어망 physical body는 명시적으로 감점한다. 따라서 USD에서
            # Body0/Body1을 어느 순서로 연결했는지에 의존하지 않는다.
            score = 0
            if rigid_body_path == joint_parent:
                score += 100
            elif rigid_body_path.HasPrefix(joint_parent):
                score += 80
            elif joint_parent.HasPrefix(rigid_body_path):
                score += 40
            if (
                rigid_body_path == net_body
                or rigid_body_path.HasPrefix(net_body)
                or net_body.HasPrefix(rigid_body_path)
            ):
                score -= 100
            candidates.append(
                (score, side, rigid_body_path, relationship_target)
            )

        if not candidates:
            raise RuntimeError(
                f"{class_name}: FixedJoint has no RigidBody target: {joint_path}"
            )
        candidates.sort(key=lambda item: item[0], reverse=True)
        if len(candidates) > 1 and candidates[0][0] == candidates[1][0]:
            raise RuntimeError(
                f"{class_name}: cannot distinguish object body from FixedJoint "
                f"targets at {joint_path}: {candidates}"
            )

        score, side, selected_path, relationship_target = candidates[0]
        if score < 0:
            raise RuntimeError(
                f"{class_name}: selected FixedJoint body is the net, not the "
                f"trash object: {selected_path}"
            )
        body_path = str(selected_path)
        print(
            f"[NetClean] suction body map: class={class_name}, "
            f"side={side}, relation={relationship_target}, "
            f"body={body_path}, score={score}",
            flush=True,
        )
        result[class_name] = body_path
    if len(set(result.values())) != len(result):
        raise RuntimeError(
            "Each Station2 class must map to a different FixedJoint body1"
        )
    return result


class TrackedRigidObjects:
    def __init__(self, wrappers):
        self._wrappers = dict(wrappers); self._initial = {}
        for name, wrapper in self._wrappers.items():
            p, q = wrapper.get_world_pose(); self._initial[name] = (np.asarray(p, dtype=float).copy(), np.asarray(q, dtype=float).copy())
    def restore(self):
        for name, wrapper in self._wrappers.items():
            p, q = self._initial[name]; wrapper.set_world_pose(position=p, orientation=q)
            try:
                wrapper.set_linear_velocity(np.zeros(3)); wrapper.set_angular_velocity(np.zeros(3))
            except Exception: pass


def _apply_robot_drive_settings(robot, name):
    if not APPLY_RUNTIME_GAINS: return
    controller = robot.get_articulation_controller()
    controller.set_gains(kps=np.asarray(ROBOT_STIFFNESS, dtype=float), kds=np.asarray(ROBOT_DAMPING, dtype=float), save_to_usd=False)
    if APPLY_RUNTIME_EFFORT_LIMITS and hasattr(controller, "set_max_efforts"):
        controller.set_max_efforts(np.asarray(ROBOT_MAX_EFFORT, dtype=float))
    actual_kps, actual_kds = controller.get_gains()
    print(f"[NetClean DRIVE] {name} KP={np.asarray(actual_kps).round(4).tolist()} KD={np.asarray(actual_kds).round(4).tolist()}", flush=True)


def _validate_static_config(usd_path):
    errors = []
    if not CONFIG_READY: errors.append("CONFIG_READY is False")
    for name, path in {"USD_PATH/--usd": usd_path, "M0609_URDF_PATH": M0609_URDF_PATH, "M0609_LULA_DESCRIPTOR_PATH": M0609_LULA_DESCRIPTOR_PATH}.items():
        if _is_todo(path): errors.append(f"{name} still contains TODO")
        elif not os.path.isfile(path): errors.append(f"{name} does not exist: {path}")
    required = [ROBOT1_PRIM_PATH, ROBOT2_PRIM_PATH, NET_ROOT_PRIM_PATH, NET_PHYSICAL_RIGID_PRIM_PATH, ROBOT1_TCP_PRIM_PATH, ROBOT2_TCP_PRIM_PATH, ROBOT2_SUCTION_BODY_PRIM_PATH, CAMERA1_PRIM_PATH, CAMERA2_PRIM_PATH, ROS_CLOCK_GRAPH_PRIM_PATH, CAMERA1_GRAPH_PRIM_PATH, CAMERA2_GRAPH_PRIM_PATH, *CARRIAGE_PRIM_PATHS, *JOINT_PATH_BY_CLASS.values()]
    for path in required:
        if _is_todo(path) or not path.startswith('/'): errors.append(f"Incomplete Prim path: {path}")
    if errors: raise RuntimeError("Standalone configuration is incomplete:\n  - " + "\n  - ".join(errors))


def _validate_stage(stage):
    required = [ROBOT1_PRIM_PATH, ROBOT2_PRIM_PATH, NET_ROOT_PRIM_PATH, NET_PHYSICAL_RIGID_PRIM_PATH, ROBOT1_TCP_PRIM_PATH, ROBOT2_TCP_PRIM_PATH, ROBOT2_SUCTION_BODY_PRIM_PATH, CAMERA1_PRIM_PATH, CAMERA2_PRIM_PATH, ROS_CLOCK_GRAPH_PRIM_PATH, CAMERA1_GRAPH_PRIM_PATH, CAMERA2_GRAPH_PRIM_PATH, *CARRIAGE_PRIM_PATHS, *JOINT_PATH_BY_CLASS.values()]
    missing = [p for p in required if not stage.GetPrimAtPath(p).IsValid()]
    if missing: raise RuntimeError("Missing required Stage Prims:\n  - " + "\n  - ".join(missing))
    if not any(prim.IsA(UsdPhysics.Scene) for prim in stage.Traverse()): raise RuntimeError("No Physics Scene exists")
    for class_name, path in JOINT_PATH_BY_CLASS.items():
        if not stage.GetPrimAtPath(path).IsA(UsdPhysics.FixedJoint): raise RuntimeError(f"{class_name}: not FixedJoint: {path}")


def _publish_net_pose(ros, controller):
    position, orientation_wxyz = controller.get_world_pose(); msg = Pose()
    msg.position.x, msg.position.y, msg.position.z = map(float, position)
    msg.orientation.x = float(orientation_wxyz[1]); msg.orientation.y = float(orientation_wxyz[2]); msg.orientation.z = float(orientation_wxyz[3]); msg.orientation.w = float(orientation_wxyz[0])
    ros.net_current_pose_pub.publish(msg)


def _move_net_blocking(world, controller, target_xyz, timeout_sec=30.0):
    controller.set_target(target_xyz)
    for _ in range(max(1, int(math.ceil(timeout_sec / PHYSICS_DT)))):
        controller.update(PHYSICS_DT); world.step(render=not ARGS.headless)
        if controller.at_target(): controller.update(0.0); return
    raise RuntimeError(f"Net transport timeout to {list(target_xyz)}")


def _perform_cycle_reset(world, ros, net_controller, robot1_controller, robot2_controller, suction_controller, joint_controller, tracked_objects, robot1, robot2):
    carb.log_info("[NetClean] cycle reset started")
    ros.clear_command_queues(); robot1_controller.reset(); robot2_controller.reset(); suction_controller.reset(); joint_controller.reset()
    world.reset(); _apply_robot_drive_settings(robot1, "robot1"); _apply_robot_drive_settings(robot2, "robot2")
    net_controller.reset_to_spawn(); tracked_objects.restore(); robot1_controller.reset(); robot2_controller.reset(); suction_controller.reset(); joint_controller.reset()
    for _ in range(WARMUP_STEPS): net_controller.update(0.0); world.step(render=not ARGS.headless)
    _move_net_blocking(world, net_controller, NET_WAIT_XYZ)
    if not suction_controller.is_open(): raise RuntimeError("Runtime suction did not open during reset")
    if not joint_controller.all_enabled(): raise RuntimeError("Object FixedJoint reset failed")
    carb.log_info("[NetClean] cycle reset completed at WAIT")


def main():
    world = None; ros = None; shutting_down = False
    def _request_shutdown(_signum=None, _frame=None):
        nonlocal shutting_down; shutting_down = True
    signal.signal(signal.SIGINT, _request_shutdown); signal.signal(signal.SIGTERM, _request_shutdown)
    try:
        _validate_static_config(ARGS.usd)
        carb.log_info(f"[NetClean] opening USD: {ARGS.usd}")
        if not open_stage(ARGS.usd): raise RuntimeError(f"Failed to open USD: {ARGS.usd}")
        loading_updates = 0
        while is_stage_loading():
            simulation_app.update(); loading_updates += 1
            if loading_updates > 1800: raise RuntimeError("USD stage loading timeout")
        simulation_app.update(); print(f"[NetClean] stage loading completed (updates={loading_updates})", flush=True)
        stage = omni.usd.get_context().get_stage(); _validate_stage(stage); _prepare_transport_stage(stage)
        object_body_paths = _object_body_paths_from_joint_map(stage, JOINT_PATH_BY_CLASS)
        world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT, stage_units_in_meters=1.0)
        robot1 = world.scene.add(SingleArticulation(prim_path=ROBOT1_PRIM_PATH, name="robot1_m0609"))
        robot2 = world.scene.add(SingleArticulation(prim_path=ROBOT2_PRIM_PATH, name="robot2_m0609"))
        logical_net_root = world.scene.add(SingleXFormPrim(prim_path=NET_ROOT_PRIM_PATH, name="net_logical_root"))
        physical_net = world.scene.add(SingleXFormPrim(prim_path=NET_PHYSICAL_RIGID_PRIM_PATH, name="net_physical_green"))
        carriage_wrappers = [world.scene.add(SingleXFormPrim(prim_path=p, name=f"net_carriage_{i}")) for i,p in enumerate(CARRIAGE_PRIM_PATHS)]
        object_wrappers = {name: world.scene.add(SingleRigidPrim(prim_path=path, name=f"trash_{i}_{name}")) for i,(name,path) in enumerate(object_body_paths.items())}
        world.reset(); _apply_robot_drive_settings(robot1, "robot1"); _apply_robot_drive_settings(robot2, "robot2")
        for _ in range(WARMUP_STEPS): world.step(render=not ARGS.headless)
        tracked_objects = TrackedRigidObjects(object_wrappers)
        net_controller = NetTransportController(logical_net_root, physical_net, carriage_wrappers, NET_SPEED_MPS); net_controller.reset_to_spawn()
        rclpy.init(args=None); ros = SimRosInterface()
        robot1_config = RobotConfig(name="robot1", prim_path=ROBOT1_PRIM_PATH, urdf_path=M0609_URDF_PATH, descriptor_path=M0609_LULA_DESCRIPTOR_PATH, end_effector_frame=M0609_END_EFFECTOR_FRAME, flange_to_tcp_position=ROBOT1_FLANGE_TO_TCP_POSITION, flange_to_tcp_orientation_wxyz=ROBOT1_FLANGE_TO_TCP_QUATERNION_WXYZ, motion_done_topic=ROBOT1_MOTION_DONE_TOPIC)
        robot2_config = RobotConfig(name="robot2", prim_path=ROBOT2_PRIM_PATH, urdf_path=M0609_URDF_PATH, descriptor_path=M0609_LULA_DESCRIPTOR_PATH, end_effector_frame=M0609_END_EFFECTOR_FRAME, flange_to_tcp_position=ROBOT2_FLANGE_TO_TCP_POSITION, flange_to_tcp_orientation_wxyz=ROBOT2_FLANGE_TO_TCP_QUATERNION_WXYZ, motion_done_topic=ROBOT2_MOTION_DONE_TOPIC)
        robot1_controller = RobotMotionController(robot1, robot1_config, ros.robot1_motion_done_pub); robot2_controller = RobotMotionController(robot2, robot2_config, ros.robot2_motion_done_pub)
        suction_controller = RuntimeSuctionController(
            stage,
            object_body_paths,
            object_wrappers,
            ros.robot2_suction_state_pub,
        )
        joint_controller = FixedJointReleaseController(stage, JOINT_PATH_BY_CLASS, suction_controller, ros.robot2_joint_release_done_pub)
        suction_controller.reset(); joint_controller.reset(); tracked_objects.restore()
        for _ in range(WARMUP_STEPS): net_controller.update(0.0); world.step(render=not ARGS.headless)
        _move_net_blocking(world, net_controller, NET_WAIT_XYZ)
        ready_period = 1.0 / READY_PUBLISH_HZ; pose_period = 1.0 / NET_CURRENT_POSE_HZ; last_ready_publish = -math.inf; last_pose_publish = -math.inf; simulated_time = 0.0; sim_ready = True
        print("[NetClean] standalone READY at WAIT; waiting for ROS commands", flush=True)
        while simulation_app.is_running() and not shutting_down:
            world.step(render=not ARGS.headless); simulated_time += PHYSICS_DT; rclpy.spin_once(ros, timeout_sec=0.0)
            if ros.reset_requests:
                sim_ready = False; ros._publish_bool_once(ros.sim_ready_pub, False)
                try:
                    _perform_cycle_reset(world, ros, net_controller, robot1_controller, robot2_controller, suction_controller, joint_controller, tracked_objects, robot1, robot2)
                except Exception as exc:
                    carb.log_error(f"[NetClean] cycle reset failed: {exc}"); ros._publish_bool_once(ros.sim_reset_done_pub, False)
                else:
                    sim_ready = True; ros._publish_bool_once(ros.sim_reset_done_pub, True); _publish_net_pose(ros, net_controller); last_ready_publish = -math.inf; last_pose_publish = simulated_time
                continue
            while ros.net_targets: net_controller.set_target(ros.net_targets.popleft())
            while ros.robot1_requests: robot1_controller.enqueue(ros.robot1_requests.popleft())
            while ros.robot2_requests: robot2_controller.enqueue(ros.robot2_requests.popleft())
            while ros.suction_requests: suction_controller.enqueue(ros.suction_requests.popleft())
            while ros.release_requests: joint_controller.enqueue(ros.release_requests.popleft())
            net_controller.update(PHYSICS_DT); robot1_controller.update(PHYSICS_DT); robot2_controller.update(PHYSICS_DT); suction_controller.update(); joint_controller.update()
            if simulated_time - last_pose_publish >= pose_period: _publish_net_pose(ros, net_controller); last_pose_publish = simulated_time
            if simulated_time - last_ready_publish >= ready_period: ros._publish_bool_once(ros.sim_ready_pub, sim_ready); last_ready_publish = simulated_time
        return 0
    except Exception as exc:
        carb.log_error(f"[NetClean] fatal error: {exc}"); return 1
    finally:
        if ros is not None:
            try: ros._publish_bool_once(ros.sim_ready_pub, False); ros.destroy_node()
            except Exception: pass
        if rclpy.ok(): rclpy.shutdown()
        if world is not None:
            try: world.stop(); world.clear_instance()
            except Exception: pass
        simulation_app.close()


if __name__ == "__main__":
    sys.exit(main())
