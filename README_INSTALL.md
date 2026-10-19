# 설치 · 실행 안내

Isaac Sim 5.1 + ROS 2 Jazzy 기반 NetClean 공정(어망 이송 → 절단 → 흡착 제거 → 재검사)을
**GPU PC 1대**에서 실행하는 방법입니다. GPU가 없는 PC에서는 3번(빌드 · 단위 시험)까지만 할 수 있습니다.

## 1. 준비

| 항목 | 값 |
|---|---|
| OS · ROS | Ubuntu 24.04 · ROS 2 Jazzy · `rmw_fastrtps_cpp` |
| 시뮬레이터 | NVIDIA Isaac Sim 5.1.0 (RTX GPU 필요) |
| Python 패키지 | `ultralytics`(YOLO11), `numpy<2`, `opencv-python`, `message_filters`, `cv_bridge` |
| 모델 | `models/netclean_yolo11n/weights/best.pt` (저장소에 포함) |

```bash
pip install ultralytics      # numpy 2.x 는 ROS cv_bridge 와 충돌하므로 numpy<2 로 맞춥니다
git clone https://github.com/suuuhululu/isaac_ros2_waste_net_cutting2.git
cd isaac_ros2_waste_net_cutting2
```

## 2. 빌드

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
cd ..
```

`ros2_ws/build`, `install`, `log`는 빌드한 PC의 절대경로를 기록하므로, 폴더를 다른 PC로 복사할 때는 빼고 새 PC에서 다시 빌드합니다.

## 3. 단위 시험 (GPU 불필요)

```bash
python3 -m pytest ros2_ws/src/nc_vision/test/test_vision_geometry.py ros2_ws/src/nc_vision/test/test_sync_utils.py ros2_ws/src/nc_vision/test/test_image_msg_utils.py ros2_ws/src/nc_robot/test/test_motion_utils.py -q     # 18 passed
python3 scripts/verify_submission.py                                           # 제출 구성 · 상대경로 점검
```

## 4. 전체 실행

터미널 3개에서 같은 환경 설정을 먼저 합니다.

```bash
export ROS_DOMAIN_ID=144 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
```

| 순서 | 대상 | 명령 |
|---|---|---|
| ① | PC A — control · robot1 · robot2 | `ros2 launch nc_bringup pc_a.launch.py` |
| ② | PC B — vision1 · vision2 | `ros2 launch nc_bringup pc_b_v2.launch.py` |
| ③ | Isaac Sim | `export ISAAC_SIM_ROOT=<Isaac Sim 설치 경로>` → `./scripts/run_standalone.sh` |

- 시뮬레이터가 `/sim/ready`를 보내면 Control이 공정을 시작하고, 어망 이송 → 절단 → 제거 → 재검사 → Exit → 리셋을 반복합니다. 처음 실행은 월드 로딩에 2분 남짓 걸립니다.
- 화면 없이 돌리려면 `./scripts/run_standalone.sh --headless`입니다. 헤드리스에서도 카메라는 발행되고, 물리만 돌릴 때는 `--no-render`(카메라 토픽 없음)를 붙입니다.
- `pc_b.launch.py`와 `pc_b_v2.launch.py`는 내용이 같습니다. 둘 중 하나만 실행합니다.
- 비전 설정의 `device: "0"`(`ros2_ws/src/nc_bringup/config/vision*_v2.yaml`)은 GPU 번호입니다. GPU가 없으면 `"cpu"`로 바꿉니다.

### 월드 선택

| 월드 | 파일 | 설명 |
|---|---|---|
| 공장 배경(기본) | `sim/assets/project1/simulation_integration_v4_factory.usda` | 원본 v4 위에 공장 장식 레이어 3개 |
| 원본 | `sim/assets/project1/simulation_integration_v4.usd` | 장식 없는 공정 월드 |

```bash
NETCLEAN_USD=$PWD/sim/assets/project1/simulation_integration_v4.usd ./scripts/run_standalone.sh
```

두 월드 모두 6 Cycle 연속 완주를 확인했습니다([`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md)).

## 5. 연동 확인

```bash
ros2 topic echo /sim/ready --once              # 시뮬레이터 준비
ros2 topic echo /process/state                 # 공정 상태(WAIT_SIM → MOVING_STATION1 → … → COMPLETED)
ros2 topic list | grep -E "camera|station|robot"
ros2 action list                               # /cut/execute · /remove/execute
```

카메라 `/camera1·2/rgb/image_raw`, `depth/image_raw`, `camera_info`가 나오는지 확인합니다(CameraInfo `fx = fy = 295.69`, 640×640).

## 6. 비전 디버그 팝업 (선택)

```bash
ros2 launch nc_vision_debug debug_popups.launch.py
```

| 용도 | Vision 1 | Vision 2 |
|---|---|---|
| 검출 영상 | `/vision1/debug_image` | `/vision2/debug_image` |
| 팝업 동기화 영상 | `/vision1/debug/source_image` | `/vision2/debug/source_image` |
| 상태 JSON | `/vision1/debug/status` | `/vision2/debug/status` |
| 대시보드 영상 | `/vision1/debug_dashboard` | `/vision2/debug_dashboard` |

팝업은 YOLO 박스 · 신뢰도 · 절단점(Vision 1) · 흡착 목표(Vision 2)와 Action 진행 상태를 보여 줍니다. `ExecuteRemove` Goal은 `camera2_optical_frame` 기준 좌표를 보내므로 팝업도 `CAMERA (X, Y, Z)`로 표시합니다(월드 변환은 `robot2_node`가 합니다).

## 7. 문제가 생기면

| 증상 | 확인 |
|---|---|
| Vision 1이 `KeyError: 16`을 반복 | OpenCV 5와 `cv_bridge` 충돌 — 이 저장소의 `image_msg_utils.py`로 해결됨. 빌드가 최신인지 확인 |
| `--headless`에서 카메라 토픽이 없음 | `--no-render`를 붙였는지 확인 |
| Station 1에서 멈춤(`Waiting for all classes`) | Vision 1은 `plastic_bottle` · `can` · `buoy` 3종이 모두 보여야 절단을 시작합니다 |
| 월드가 안 열림 | 로그에서 못 찾는 파일 확인. NVIDIA 원격 에셋(머티리얼)은 첫 실행 때 인터넷으로 받습니다 |
| 7가지 트러블슈팅 전체 | [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) |
