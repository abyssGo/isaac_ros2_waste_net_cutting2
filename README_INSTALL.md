# NetClean Vision + Debug Popup Fix

이 수정본은 `nc_vision`과 `nc_vision_debug`만 변경합니다. 로봇 노드,
`nc_control`, Standalone은 포함하거나 수정하지 않습니다.

## 변경 내용

- `vision1_node`가 `/vision1/debug/status` JSON을 2 Hz로 발행
- `vision2_node`가 `/vision2/debug/status` JSON을 2 Hz로 발행
- 실제 YOLO Bounding Box, 클래스, Confidence와 Action 포함 여부 표시
- Vision 1: Bounding Box 네 꼭짓점 P1~P4 표시
- Vision 2: 중심점, Depth, `camera2_optical_frame` XYZ 표시
- Action Feedback 기반 업무 번호와 `QUEUED`, `CUTTING`, `REMOVING`,
  `COMPLETE`, `FAILED` 상태 갱신
- 재검사 횟수와 Station 2 완료 상태 표시
- 기존 YOLO 검출 영상과 팝업 대시보드 출력 토픽 분리
- 추론에 실제 사용한 동일 RGB 프레임으로 팝업을 구성하여 프레임 시차 제거
- 터미널과 나란히 표시하기 위한 1280x720 레이아웃과 확대된 글자/박스 적용
- 영상 위 중복 상태 배지를 줄이고 상세 상태는 오른쪽 패널에 크게 표시
- 별도 오른쪽 칸을 제거하고 전체 영상 위에 검은색 반투명 정보 오버레이 적용
- 중앙 Crop 좌표를 Bounding Box 변환에도 반영하여 전체 폭 영상과 좌표 일치 유지
- 검증값이 모두 `N/A`이면 MODEL VALIDATION 영역 자동 숨김
- 기존 비전 노드를 보존한 선택 실행용 V2 노드와 `pc_b_v2.launch.py` 추가
- V2 Vision 2에서 `plastic_bottle`만 Margin + median-Depth 파지점 적용
- 팝업에 Bounding Box 중심 `C`와 실제 Action 전달점 `G` 동시 표시

## 토픽

| 용도 | Vision 1 | Vision 2 |
|---|---|---|
| 기존 YOLO 검출 영상 | `/vision1/debug_image` | `/vision2/debug_image` |
| 팝업 동기화 영상 | `/vision1/debug/source_image` | `/vision2/debug/source_image` |
| 팝업 상태 JSON | `/vision1/debug/status` | `/vision2/debug/status` |
| 완성 대시보드 영상 | `/vision1/debug_dashboard` | `/vision2/debug_dashboard` |

## 1. 기존 파일 백업

실행 중인 ROS 2 launch를 먼저 종료한 뒤 실행합니다.

```bash
cp ~/netclean_project/ros2_ws/src/nc_vision/nc_vision/vision1_node.py \
  ~/netclean_project/ros2_ws/src/nc_vision/nc_vision/vision1_node.py.before_popup_fix

cp ~/netclean_project/ros2_ws/src/nc_vision/nc_vision/vision2_node.py \
  ~/netclean_project/ros2_ws/src/nc_vision/nc_vision/vision2_node.py.before_popup_fix
```

## 2. 수정본 적용

다운로드한 ZIP 파일이 `~/Downloads`에 있다고 가정합니다.

```bash
unzip -o ~/Downloads/NetClean_Vision_Popup_Fix.zip \
  -d ~/netclean_project
```

압축 내부가 `ros2_ws/src/...` 구조이므로 위 경로로 풀어야 합니다.

## 3. 빌드

```bash
cd ~/netclean_project/ros2_ws

source /opt/ros/jazzy/setup.bash

colcon build \
  --symlink-install \
  --packages-select nc_vision nc_vision_debug nc_bringup

source ~/netclean_project/ros2_ws/install/setup.bash
```

## 4. 실행

모든 ROS 2 터미널에서 Domain ID를 144로 통일합니다.

### 비전 노드 터미널

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=144

source /opt/ros/jazzy/setup.bash
source ~/netclean_project/ros2_ws/install/setup.bash

ros2 launch nc_bringup pc_b.launch.py
```

### Depth 보정 V2 비전 노드

기존 방식 대신 V2를 시험할 때만 다음 Launch를 사용합니다.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=144

source /opt/ros/jazzy/setup.bash
source ~/netclean_project/ros2_ws/install/setup.bash

ros2 launch nc_bringup pc_b_v2.launch.py
```

`pc_b.launch.py`와 `pc_b_v2.launch.py`를 동시에 실행하면 안 됩니다. 두 Launch는
외부 호환성을 위해 같은 ROS 노드명, 토픽과 Action 이름을 사용합니다.

| 구분 | 기존 방식 | V2 방식 |
|---|---|---|
| Vision 1 실행 파일 | `vision1_node.py` | `vision1_node_v2.py` |
| Vision 2 실행 파일 | `vision2_node.py` | `vision2_node_v2.py` |
| 설정 | `vision1.yaml`, `vision2.yaml` | `vision1_v2.yaml`, `vision2_v2.yaml` |
| Launch | `pc_b.launch.py` | `pc_b_v2.launch.py` |
| 외부 ROS 노드명 | `/vision1_node`, `/vision2_node` | 동일 |
| Robot Action | `/cut/execute`, `/remove/execute` | 동일 |

V2에서 `plastic_bottle`은 다음 순서로 Action 좌표를 계산합니다.

1. Bounding Box 각 변에서 20% Margin 제거
2. 유효 Depth 비율 25% 이상인지 확인
3. 유효 Depth의 중앙값 계산
4. 중앙값에서 `max(3 cm, 5%)` 이내인 표면 픽셀 선택
5. 내부 ROI 중심과 가장 가까운 표면 픽셀을 `G`로 선택
6. `G` 주변 7x7 Depth 중앙값으로 3D 좌표 변환

`can`, `buoy`는 기존 Bounding Box 중심 방식을 유지합니다. 유효 Depth가 부족한
PET는 잘못된 좌표를 보내지 않고 `INVALID DEPTH`로 제외합니다.

### 팝업 터미널

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=144

source /opt/ros/jazzy/setup.bash
source ~/netclean_project/ros2_ws/install/setup.bash

ros2 launch nc_vision_debug debug_popups.launch.py
```

## 5. 연동 확인

Station 1 또는 Station 2 검사 신호가 들어온 상태에서 확인합니다.

```bash
ros2 topic info /vision1/debug/status -v
ros2 topic info /vision2/debug/status -v
```

각 토픽에 비전 노드 Publisher 1개와 팝업 Subscriber 1개가 있어야 합니다.

Bounding Box는 카메라의 현재 프레임이 아니라 다음 동기화 영상에 그려집니다.

```bash
ros2 topic info /vision1/debug/source_image -v
ros2 topic info /vision2/debug/source_image -v
```

각 토픽에도 비전 노드 Publisher 1개와 팝업 Subscriber 1개가 있어야 합니다.

V2가 실행 중인지 로그에서도 확인할 수 있습니다.

```text
NetClean Vision1 V2 started (ROS API unchanged)
NetClean Vision2 V2 started (ROS API unchanged)
Depth-guided target class='plastic_bottle'
```

```bash
timeout 5 ros2 topic echo /vision1/debug/status --once
timeout 5 ros2 topic echo /vision2/debug/status --once
```

팝업의 완성 영상도 확인할 수 있습니다.

```bash
timeout 5 ros2 topic hz /vision1/debug_dashboard
timeout 5 ros2 topic hz /vision2/debug_dashboard
```

## 좌표 표시 주의사항

현재 `vision2_node`의 `ExecuteRemove` Goal은 `camera2_optical_frame` 기준 좌표를
보냅니다. 따라서 팝업도 이를 `CAMERA (X, Y, Z)`로 정확히 표시합니다. World
Transform을 수행하지 않은 좌표를 `WORLD`라고 잘못 표기하지 않습니다.

Precision, Recall, mAP는 검증 데이터셋의 실제 측정값을 YAML에 입력하기 전까지
`N/A`로 표시됩니다. 실시간 Confidence를 모델 정확도로 잘못 표시하지 않습니다.

## 팝업 크기

기본 크기는 1280x720입니다. 영상은 상단·하단 상태 바 사이의 전체 폭을 사용하고,
정보는 영상 오른쪽에 검은색 반투명 오버레이로 표시됩니다. 터미널과 함께 보일
때는 팝업을 임의로 크게 축소하지 말고 화면 한쪽에 배치하십시오. 더 큰 모니터에서는
`nc_vision_debug/config/debug_popups.yaml`의 `window_width`, `window_height`를
같은 비율로 늘릴 수 있습니다.
