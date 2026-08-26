# NetClean Vision Debug Popups

ROS 2 Jazzy용 Vision 1·Vision 2 디버깅 팝업 패키지입니다.

- 세 클래스를 `NON-NET`으로 묶어 오른쪽 패널에 표시
- 객체별 클래스, Confidence, 바운딩박스 표시
- Vision 1: 바운딩박스 네 꼭짓점 `P1~P4` 표시
- Vision 2: 중심점, 현재 처리 대상의 Depth·World XYZ 표시
- Action 포함/제외/진행/완료 상태 및 업무 번호 표시
- 실시간 추론 시간·FPS·평균 Confidence 표시
- 검증 데이터셋의 Precision·Recall·mAP 표시
- GUI가 없는 환경에서도 팝업 대신 완성 영상을 ROS 토픽으로 계속 발행
- 카메라 영상을 전체 폭으로 표시하고 정보를 오른쪽 반투명 오버레이로 표시
- 실제 값이 없는 MODEL VALIDATION 영역은 숨겨 영상 공간 확보
- V2 Vision 2 상태정보가 있으면 흰색 `C`(박스 중심)와 초록색 `G`(Action 전달점) 표시

## 1. 설치

이 폴더를 ROS 2 워크스페이스의 `src` 아래에 `nc_vision_debug`이라는 이름으로 복사합니다.

```bash
cp -r netclean_vision_debug_popups \
  ~/netclean_project/ros2_ws/src/nc_vision_debug

cd ~/netclean_project/ros2_ws
colcon build --symlink-install --packages-select nc_vision_debug
source install/setup.bash
```

## 2. 실행

두 팝업을 함께 실행합니다.

```bash
ros2 launch nc_vision_debug debug_popups.launch.py
```

각 팝업을 따로 실행할 수도 있습니다.

```bash
ros2 run nc_vision_debug vision1_debug_popup \
  --ros-args --params-file \
  ~/netclean_project/ros2_ws/src/nc_vision_debug/config/debug_popups.yaml

ros2 run nc_vision_debug vision2_debug_popup \
  --ros-args --params-file \
  ~/netclean_project/ros2_ws/src/nc_vision_debug/config/debug_popups.yaml
```

## 3. 입력과 출력 토픽

| 팝업 | 영상 입력 | 상태 입력 | 완성 영상 출력 |
|---|---|---|---|
| Vision 1 | `/vision1/debug/source_image` | `/vision1/debug/status` | `/vision1/debug_dashboard` |
| Vision 2 | `/vision2/debug/source_image` | `/vision2/debug/status` | `/vision2/debug_dashboard` |

`/vision*/debug/status`는 `std_msgs/msg/String` JSON입니다. 기존 비전 노드에
`examples/vision_node_integration_snippet.py`의 publisher와 payload 생성 부분을
넣어야 객체와 Action 정보가 팝업에 표시됩니다.

기존 `vision1_node`와 `vision2_node`의 `/vision*/debug_image`는 YOLO가 직접 그린
검출 이미지를 유지합니다. 팝업이 완성한 대시보드는 `/vision*/debug_dashboard`로
분리되어 있으므로 Publisher가 충돌하지 않습니다.

## 4. JSON 상태 형식

```json
{
  "phase": "ROBOT WORKING",
  "scan": "INITIAL SCAN",
  "station": "ARRIVED",
  "action_server": "ACTIVE",
  "current_task": 2,
  "total_tasks": 3,
  "model_name": "best.pt",
  "device": "CUDA:0",
  "inference_ms": 17.9,
  "fps": 32.1,
  "validation": {
    "precision": 0.91,
    "recall": 0.88,
    "map50": 0.92,
    "map50_95": 0.71
  },
  "detections": [
    {
      "object_id": "can_01",
      "class_name": "can",
      "confidence": 0.91,
      "bbox": [520, 230, 660, 410],
      "action_included": true,
      "action_state": "REMOVING",
      "task_index": 2,
      "depth_m": 1.18,
      "camera_xyz": [-0.53, 2.08, 0.79],
      "reject_reason": ""
    }
  ],
  "message": "REMOVING OBJECT 2/3 - can_01 - SUCTION ACTION IN PROGRESS"
}
```

`action_state` 권장값은 다음과 같습니다.

- `QUEUED`
- `IN_PROGRESS`, `CUTTING`, `REMOVING`
- `COMPLETE`
- `EXCLUDED`

Action에서 제외했다면 `action_included=false`와 함께 `reject_reason`에
`LOW CONFIDENCE`, `INVALID DEPTH`, `OUTSIDE ROI`, `OUT OF WORKSPACE` 등을
넣습니다.

## 5. 성능지표 주의사항

객체 옆의 퍼센트는 해당 탐지의 실시간 `Confidence`입니다. Precision, Recall,
mAP는 라벨이 있는 검증 데이터셋에서 얻은 오프라인 성능입니다. 실제 값이 아직
없으면 YAML의 기본값 `-1.0`을 유지하십시오. 팝업에는 `N/A`로 표시됩니다.

## 6. 영상과 Bounding Box 동기화

팝업은 카메라의 계속 변하는 현재 영상이 아니라 비전 노드가 추론에 실제 사용한
깨끗한 프레임을 `/vision*/debug/source_image`로 받아 표시합니다. 따라서 JSON의
Bounding Box 좌표와 배경 영상이 같은 검사 시점에 고정됩니다. 이 토픽을 다시
`/camera*/rgb/image_raw`로 바꾸면 움직이는 장면에서 박스가 밀릴 수 있습니다.

## 7. 팝업 미리보기

ROS 2 없이 화면 레이아웃만 확인할 수 있습니다.

```bash
cd nc_vision_debug_popups
python3 tools/render_preview.py
```

생성 파일은 `preview/vision1_popup_preview.png`,
`preview/vision2_popup_preview.png`입니다.
