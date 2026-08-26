# NetClean YOLO11n Final Model

NetClean 프로젝트에서 폐어망에 얽힌 이물질을 탐지하기 위해 학습한 YOLO11 Detect 모델입니다.

## Model Information

* Model type: YOLO11 Detect
* Base model: YOLO11n
* Input size: 640×640
* Dataset size: 1,500 images

  * Train: 1,200
  * Validation: 150
  * Test: 150

### Classes

| Class ID | Class name       |
| -------: | ---------------- |
|        0 | `plastic_bottle` |
|        1 | `can`            |
|        2 | `buoy`           |

클래스 이름은 ROS 2 메시지, 설정 파일 및 USD 객체 매핑에서 동일하게 사용해야 합니다.

## Runtime Model

`vision1_node`와 `vision2_node`에서 사용하는 최종 가중치:

```text
weights/best.pt
```

`last.pt`는 학습 재개용이며 실제 추론에는 `best.pt`를 사용합니다.

## Validation Results

합성 Validation 데이터 기준 최종 성능:

| Metric    |  Result |
| --------- | ------: |
| Precision | 약 0.999 |
| Recall    |   1.000 |
| mAP50     |   0.995 |
| mAP50-95  | 약 0.994 |

위 결과는 동일한 Isaac Sim 합성 환경에서 생성된 Validation 데이터 기준입니다. 최종 통합 전 Camera1과 Camera2의 실제 ROS 영상에서도 별도로 탐지 결과를 확인해야 합니다.

## Recommended Inference Configuration

```yaml
model:
  path: weights/best.pt
  type: detect
  imgsz: 640
  confidence: 0.40
  iou_threshold: 0.45
  max_detections: 3
  device: "0"

classes:
  0: plastic_bottle
  1: can
  2: buoy
```

* `confidence`: 해당 값보다 낮은 신뢰도의 검출을 제거합니다.
* `iou_threshold`: 동일 객체에 생성된 중복 박스를 제거하는 NMS 기준입니다.
* `max_detections`: 현재 시연 환경에서는 클래스당 객체가 하나이므로 최대 3개로 제한합니다.

객체 누락이 많으면 `confidence`를 `0.25`까지 낮추고, 오탐이 발생하면 `0.50~0.60`으로 높입니다.

## Runtime Environment

학습 및 검증에 사용한 환경:

```text
Python: 3.12.3
ROS 2: Jazzy
OpenCV: 4.6.0
PyTorch: 2.13.0+cu130
Ultralytics: 8.4.123
CUDA: Enabled
RMW: rmw_fastrtps_cpp
ROS_DOMAIN_ID: 143
```

다른 컴퓨터에서 사용할 때 PyTorch, Ultralytics와 CUDA 호환 환경이 설치되어 있어야 합니다.

## Model Verification

모델 로딩과 클래스 정보를 확인합니다.

```bash
python3 -c \
"from ultralytics import YOLO; \
model=YOLO('weights/best.pt'); \
print(model.names)"
```

정상 출력:

```text
{0: 'plastic_bottle', 1: 'can', 2: 'buoy'}
```

이미지 추론 시험:

```bash
yolo detect predict \
  model=weights/best.pt \
  source=/path/to/test_image.png \
  imgsz=640 \
  conf=0.40 \
  iou=0.45 \
  device=0 \
  save=True
```

## ROS 2 Camera Topics

### Camera1

```text
/camera1/rgb/image_raw
/camera1/depth/image_raw
/camera1/camera_info
```

### Camera2

```text
/camera2/rgb/image_raw
/camera2/depth/image_raw
/camera2/camera_info
```

현재 카메라 원본 해상도는 1280×720입니다. Vision 노드는 어망 영역을 ROI로 자른 뒤 모델 추론에 사용하고, 검출 좌표를 원본 이미지 좌표로 복원하여 Depth와 대응시킵니다.

## Vision Node Usage

### vision1_node

* Camera1 RGB·Depth·CameraInfo 동기화
* ROI 내 `plastic_bottle`, `can`, `buoy` 탐지
* Bounding Box 기준 P1/P2 후보 픽셀 선정
* Camera ray와 Net 평면의 교차점 계산
* `/cut/execute` Action Goal 전송

Robot1 절단점은 객체 표면 Depth가 아니라 Station1의 Net 평면을 기준으로 계산합니다.

### vision2_node

* Camera2 RGB·Depth·CameraInfo 동기화
* ROI 내 객체 탐지
* Bounding Box 중앙 영역의 유효 Depth 중앙값 계산
* 객체 표면의 3D 흡착점 생성
* `/remove/execute` Action Goal 전송
* Action Result 이후 새로운 프레임으로 재검사

## ROI Policy

학습 영상에서는 어망이 화면 대부분을 차지했으므로 실제 추론에서도 어망 주변 ROI를 사용하는 것을 권장합니다.

```text
1280×720 원본 영상
→ 어망 주변 ROI crop
→ YOLO 추론
→ 검출 좌표를 원본 영상 좌표로 복원
→ 원본 Depth에서 3D 좌표 계산
```

ROI는 어망의 객체 배치 가능 영역보다 약 5~10% 넓게 설정합니다.

## Deployment Structure

권장 배포 구조:

```text
netclean_yolo11n/
├── weights/
│   └── best.pt
├── model_config.yaml
├── README.md
└── test_images/
    └── sample.png
```

추론만 실행하는 컴퓨터에는 전체 학습 데이터셋이나 `last.pt`가 필요하지 않습니다. `best.pt`, 설정 파일, Vision 노드 및 Python 의존성만 있으면 됩니다.

## Important Notes

* 모델은 Detect 모델이며 segmentation 모델이 아닙니다.
* 클래스 이름을 `pet`, `plastic bottle`, `CAN` 등으로 변경하지 않습니다.
* 실제 로봇 Action Goal은 권장 신뢰도 이상인 검출에 대해서만 생성합니다.
* ROI 밖의 검출은 로봇 작업 대상으로 사용하지 않습니다.
* Camera1과 Camera2의 `frame_id`는 각각 고유하게 설정해야 합니다.
* RGB·Depth·CameraInfo는 동기화된 프레임만 사용합니다.
* Vision2 재검사는 이전 Action Result보다 새로운 timestamp의 프레임을 사용합니다.

