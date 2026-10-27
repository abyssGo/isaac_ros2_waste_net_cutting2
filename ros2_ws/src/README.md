# ROS 2 패키지 지도

NetClean의 ROS 2 패키지 6개입니다. 전체 구조와 통신은 [루트 README의 🛠️ 시스템 구조](../../README.md#️-시스템-구조)에 있습니다.

## 패키지 한눈에

| 패키지 | 실행 위치 | 역할 | 실행 파일(노드) |
|---|---|---|---|
| [`nc_interfaces`](nc_interfaces/) | 공통 | `ExecuteCut` · `ExecuteRemove` Action, `CutTarget` · `RemoveTarget` 메시지 | — |
| [`nc_control`](nc_control/) | PC A | 어망 이송과 전체 공정 상태 머신, Watchdog | `control_node` |
| [`nc_robot`](nc_robot/) | PC A | Robot 1 절단 · Robot 2 흡착 Action Server, 좌표 변환 · 모션 유틸 | `robot1_node` · `robot2_node` |
| [`nc_vision`](nc_vision/) | PC B | YOLO11 추론, RGB-D 동기화 · 3D 목표 계산, Action Client | `vision1_node_v2` · `vision2_node_v2` |
| [`nc_vision_debug`](nc_vision_debug/) | PC B (선택) | Vision 1 · 2 디버그 팝업과 대시보드 영상 | `vision1_debug_popup_v3` · `vision2_debug_popup_v3` 외 |
| [`nc_bringup`](nc_bringup/) | PC A · PC B | 실행 파일(launch)과 공통 파라미터(config) | — |

## 실행 파일 · 설정

| launch | 실행하는 노드 | 설정 파일 |
|---|---|---|
| `nc_bringup/launch/pc_a.launch.py` | `control_node` · `robot1_node` · `robot2_node` | `config/netclean.yaml` |
| `nc_bringup/launch/pc_b_v2.launch.py` | `vision1_node_v2` · `vision2_node_v2` | `config/vision1_v2.yaml` · `config/vision2_v2.yaml` |
| `nc_vision_debug/launch/debug_popups_v3.launch.py` | `vision1_debug_popup_v3` · `vision2_debug_popup_v3` | — |

주요 파라미터:

| 파라미터 | 값 | 파일 |
|---|---|---|
| `position_tolerance` | 0.02 m — 어망 도착 판정 허용 오차 | `netclean.yaml` |
| `station1_work_timeout_sec` · `station2_work_timeout_sec` · `reset_timeout_sec` | 180 s · 240 s · 45 s | `netclean.yaml` |
| `model_path` | `models/netclean_yolo11n/weights/best.pt` | `vision1_v2.yaml` · `vision2_v2.yaml` |
| `device` | `"0"` — 추론에 쓸 GPU 번호 | `vision1_v2.yaml` · `vision2_v2.yaml` |
| `arrival_settle_sec` | 1.0 s — 도착 후 어망이 멈춘 프레임만 사용 | `vision1_v2.yaml` · `vision2_v2.yaml` |
| `empty_confirmation_frames` | 5 — 재검사에서 빈 프레임 연속 확인 수 | `vision2_v2.yaml` |

## 인터페이스

```text
ExecuteCut.action        goal: header · CutTarget[] targets
ExecuteRemove.action     goal: header · RemoveTarget[] targets
                         result: success · message
                         feedback: current_target · total_targets · current_object_id

CutTarget.msg            object_id · point1 · point2            (절단점 P1 · P2)
RemoveTarget.msg         object_id · class_name · position · fixed_joint_path
```

## 단위 시험

ROS 2 Jazzy 환경에서 빌드 · `source` 뒤 저장소 루트에서 실행합니다.

```bash
python3 -m pytest \
  ros2_ws/src/nc_vision/test/test_vision_geometry.py \
  ros2_ws/src/nc_vision/test/test_sync_utils.py \
  ros2_ws/src/nc_vision/test/test_image_msg_utils.py \
  ros2_ws/src/nc_robot/test/test_motion_utils.py -q     # 18 passed
```

| 시험 파일 | 개수 | 확인하는 것 |
|---|---|---|
| `nc_vision/test/test_vision_geometry.py` | 5 | 픽셀 + Depth → 3D 변환, 절단점 생성 |
| `nc_vision/test/test_sync_utils.py` | 3 | 시뮬 리셋으로 시간이 되감길 때 RGB-D 동기화 큐 처리 |
| `nc_vision/test/test_image_msg_utils.py` | 4 | OpenCV 5 · `cv_bridge` 충돌을 피한 이미지 메시지 변환 |
| `nc_robot/test/test_motion_utils.py` | 6 | M0609 도달 범위 검사 · 절단점 보정, 시뮬 시간 대기 |

각 패키지의 `test_copyright.py` · `test_flake8.py` · `test_pep257.py`는 ROS 2 패키지 템플릿 기본 검사입니다.
