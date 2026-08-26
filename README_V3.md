# NetClean Vision V3 Final

이 배포본은 업로드된 `nc_vision`을 기준으로 V1/V2를 보존하고 V3만 추가한다.
`nc_control`은 수정하지 않는다.

## V3 핵심 수정

- `vision2_node_v3.py`가 재검사에서 빈 화면을 연속 5회 확인하면
  `/station2/complete`에 `std_msgs/Bool(data=True)`를 한 번 발행한다.
- `/station2/complete`에는 공정 도착 시 `False`를 보내지 않는다.
  Control은 대기 상태에서 `False`를 받으면 즉시 실패로 처리하기 때문이다.
- 기존 모니터링 호환용 `/vision2/removal_complete`는 유지한다.
- 완료 또는 terminal Action 실패 후 내부 station latch를 해제하여 다음 Cycle의
  `/station2/net_arrived=True`로 다시 무장된다.
- Vision1도 Action 종료 후 내부 station latch를 해제하여 반복 Cycle에 대비한다.
- `plastic_bottle`은 V2의 Depth 기반 중앙 표면 파지점 방식을 그대로 유지한다.
- V3 팝업은 원본 화면 비율을 유지하고, 원본 카메라는 20 FPS로 갱신하며,
  UI 전용 Bounding Box는 GPU 부하를 줄이기 위해 최대 5 Hz로 갱신한다.

## 설치

```bash
cd ~/netclean_project
unzip -o ~/Downloads/NetClean_Vision_V3_Final.zip -d ~/netclean_project

cd ~/netclean_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select nc_vision nc_vision_debug nc_bringup
source install/setup.bash
```

## 권장 실행 순서

모든 터미널에서 다음 환경을 사용한다.

```bash
# 현재 임시 도메인 값. 팀의 모든 PC와 Standalone에서 같은 값이어야 한다.
export NETCLEAN_DOMAIN_ID=144
export ROS_DOMAIN_ID=$NETCLEAN_DOMAIN_ID
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash
source ~/netclean_project/ros2_ws/install/setup.bash
```

1. Standalone 실행
2. Vision V3 실행

```bash
ros2 launch nc_bringup pc_b_v3.launch.py
```

3. 최종 시연용 Station2 팝업만 실행

```bash
ros2 launch nc_vision_debug debug_popup_station2_v3.launch.py
```

두 팝업이 모두 필요하면 다음을 사용한다.

```bash
ros2 launch nc_vision_debug debug_popups_v3.launch.py
```

4. Control/Robot launch 실행

Vision이 `/station2/net_arrived`의 일회성 신호를 놓치지 않도록 Control은 Vision보다
나중에 실행한다.

## 완료 신호 확인

공정 시작 전에 구독을 걸어 둔다.

```bash
ros2 topic echo /station2/complete
```

재검사 완료 시 다음 값이 한 번 출력되어야 한다.

```text
data: true
```

연결 상태 확인:

```bash
ros2 topic info /station2/complete -v
```

정상 기준은 Publisher가 `/vision2_node`, Subscriber가 `/control_node`로 각각 1개다.

## 포함 파일

- `nc_vision/nc_vision/vision1_node_v3.py`
- `nc_vision/nc_vision/vision2_node_v3.py`
- `nc_vision/nc_vision/depth_target_utils_v3.py`
- `nc_bringup/config/vision1_v3.yaml`
- `nc_bringup/config/vision2_v3.yaml`
- `nc_bringup/launch/pc_b_v3.launch.py`
- `nc_vision_debug/config/debug_popups_v3.yaml`
- `nc_vision_debug/launch/debug_popups_v3.launch.py`
- `nc_vision_debug/launch/debug_popup_station2_v3.launch.py`
- `nc_vision_debug/nc_vision_debug/vision1_debug_popup_v3_node.py`
- `nc_vision_debug/nc_vision_debug/vision2_debug_popup_v3_node.py`

## 기존 버전 보존

`vision1_node.py`, `vision2_node.py`, `vision1_node_v2.py`,
`vision2_node_v2.py`는 삭제하지 않는다. V3는 `pc_b_v3.launch.py`를 실행할 때만
선택된다. 외부 ROS node name과 Action/카메라 토픽은 기존과 동일하다.
