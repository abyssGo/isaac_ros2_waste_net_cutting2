"""NetClean 로봇 노드 공용 좌표 변환 함수.

이 파일은 ROS 2 노드가 아니므로 단독 실행하지 않는다.
``robot1_node.py``와 ``robot2_node.py``에서 import해서 사용한다.

좌표/Quaternion 규칙
--------------------
* 길이 단위: metre
* Quaternion 순서: ROS 표준 ``[x, y, z, w]``
* 변환행렬 이름 ``T_TARGET_SOURCE``는 source 좌표를 target 좌표로
  바꾸는 4x4 행렬을 뜻한다.
  예: ``T_WORLD_CAMERA1``은 Camera 1 좌표를 World 좌표로 변환한다.
"""

from __future__ import annotations

from math import sqrt
from typing import Sequence

import numpy as np
from geometry_msgs.msg import Point, PoseStamped


_EPSILON = 1.0e-9


def _xyz(values: Sequence[float], name: str) -> np.ndarray:
    """길이 3의 값을 float NumPy 배열로 검사·변환한다."""
    array = np.asarray(values, dtype=float)

    if array.shape != (3,):
        raise ValueError(f"{name}은(는) 길이 3이어야 합니다: {array.shape}")

    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name}에 NaN 또는 inf가 포함되어 있습니다.")

    return array


def validate_transform_matrix(transform_matrix: Sequence[Sequence[float]]) -> np.ndarray:
    """4x4 동차변환행렬을 검사하고 NumPy 배열로 반환한다."""
    matrix = np.asarray(transform_matrix, dtype=float)

    if matrix.shape != (4, 4):
        raise ValueError(
            "transform_matrix는 4x4 행렬이어야 합니다: "
            f"{matrix.shape}"
        )

    if not np.all(np.isfinite(matrix)):
        raise ValueError("transform_matrix에 NaN 또는 inf가 포함되어 있습니다.")

    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-6):
        raise ValueError(
            "transform_matrix의 마지막 행은 [0, 0, 0, 1]이어야 합니다."
        )

    return matrix


def normalize_quaternion_xyzw(
    quaternion_xyzw: Sequence[float],
) -> tuple[float, float, float, float]:
    """ROS 순서 [x, y, z, w] Quaternion을 단위 Quaternion으로 만든다."""
    if len(quaternion_xyzw) != 4:
        raise ValueError("quaternion_xyzw는 [x, y, z, w] 네 값이어야 합니다.")

    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = sqrt(x * x + y * y + z * z + w * w)

    if norm < _EPSILON:
        raise ValueError("크기가 0인 Quaternion은 사용할 수 없습니다.")

    return x / norm, y / norm, z / norm, w / norm


def quaternion_xyzw_to_rotation_matrix(
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """ROS 순서 Quaternion [x, y, z, w]을 3x3 회전행렬로 변환한다."""
    x, y, z, w = normalize_quaternion_xyzw(quaternion_xyzw)

    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def make_transform_matrix(
    position_xyz: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """위치와 ROS Quaternion으로 4x4 동차변환행렬을 만든다.

    카메라의 월드 위치와 월드 회전을 입력하면 ``T_WORLD_CAMERA``를
    만들 수 있다.
    """
    position = _xyz(position_xyz, "position_xyz")
    rotation = quaternion_xyzw_to_rotation_matrix(quaternion_xyzw)

    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = position

    return transform


def transform_xyz(
    source_xyz: Sequence[float],
    transform_matrix: Sequence[Sequence[float]],
) -> tuple[float, float, float]:
    """3D 좌표에 4x4 변환행렬을 적용한다."""
    source = _xyz(source_xyz, "source_xyz")
    transform = validate_transform_matrix(transform_matrix)
    source_homogeneous = np.append(source, 1.0)
    target = transform @ source_homogeneous

    return float(target[0]), float(target[1]), float(target[2])


def transform_point(
    source_point: Point,
    transform_matrix: Sequence[Sequence[float]],
) -> Point:
    """geometry_msgs/Point에 4x4 변환행렬을 적용한다."""
    x, y, z = transform_xyz(
        (source_point.x, source_point.y, source_point.z),
        transform_matrix,
    )

    target_point = Point()
    target_point.x = x
    target_point.y = y
    target_point.z = z

    return target_point


def offset_point(
    source_point: Point,
    direction_xyz: Sequence[float],
    distance: float,
) -> Point:
    """Point를 지정한 방향으로 distance만큼 이동한다.

    direction_xyz는 함수 내부에서 자동 정규화된다. 따라서 어망의
    법선 벡터와 Approach/Retreat 거리를 전달하면 목표점을 만들 수 있다.
    """
    direction = _xyz(direction_xyz, "direction_xyz")
    direction_norm = float(np.linalg.norm(direction))

    if direction_norm < _EPSILON:
        raise ValueError("direction_xyz는 0 벡터일 수 없습니다.")

    unit_direction = direction / direction_norm
    offset = unit_direction * float(distance)

    result = Point()
    result.x = float(source_point.x + offset[0])
    result.y = float(source_point.y + offset[1])
    result.z = float(source_point.z + offset[2])

    return result


def add_tool_offset(
    target_point: Point,
    tool_offset_xyz: Sequence[float],
) -> Point:
    """비전 목표점에 엔드이펙터 TCP 보정값을 더한다."""
    offset = _xyz(tool_offset_xyz, "tool_offset_xyz")

    result = Point()
    result.x = float(target_point.x + offset[0])
    result.y = float(target_point.y + offset[1])
    result.z = float(target_point.z + offset[2])

    return result


def make_pose_stamped(
    node,
    position: Point,
    orientation_xyzw: Sequence[float],
    frame_id: str = "world",
) -> PoseStamped:
    """월드 Point와 도구 자세로 Standalone 전송용 PoseStamped를 만든다."""
    if not frame_id:
        raise ValueError("frame_id는 빈 문자열일 수 없습니다.")

    qx, qy, qz, qw = normalize_quaternion_xyzw(orientation_xyzw)

    pose = PoseStamped()
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.header.frame_id = frame_id

    pose.pose.position.x = float(position.x)
    pose.pose.position.y = float(position.y)
    pose.pose.position.z = float(position.z)

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose