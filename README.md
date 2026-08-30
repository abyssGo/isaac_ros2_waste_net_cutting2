# NetClean Isaac Sim + ROS 2

폐어망 절단·수거 시뮬레이션을 위한 Isaac Sim 실행 코드와 ROS 2 패키지입니다.
`feature/su`와 `feature/su2`의 기능을 통합했으며, PC A와 PC B 실행 파일을 모두
포함합니다.

## 포함 내용

- 메인 시뮬레이션 월드: `sim/assets/project1/simulation_integration_v3.usd`
- USD가 참조하는 로봇, 컨베이어, 카메라, 폐기물, 재질 및 텍스처 에셋
- M0609 URDF와 Lula descriptor
- Isaac Sim standalone Python 코드: `sim/standalone/`
- ROS 2 워크스페이스: `ros2_ws/src/`
- PC A bringup: `ros2_ws/src/nc_bringup/launch/pc_a.launch.py`
- PC B bringup: `ros2_ws/src/nc_bringup/launch/pc_b.launch.py`
- PC B depth 보정판: `ros2_ws/src/nc_bringup/launch/pc_b_v2.launch.py`

생성물인 `build/`, `install/`, `log/` 디렉터리와 ZIP 백업 파일은 Git에서
제외합니다.

## 디렉터리

| 경로 | 내용 |
|---|---|
| `sim/assets/project1/` | 메인 USD와 상대 참조 에셋 |
| `sim/cobot3_ws/` | 메인 USD의 Robot 1 상대 참조 에셋 |
| `sim/standalone/` | Isaac Sim 실행 계층과 개발 이력 파일 |
| `sim/config/` | 시뮬레이션 파라미터 참고 설정 |
| `ros2_ws/src/` | `nc_interfaces`, `nc_control`, `nc_robot`, `nc_vision`, `nc_vision_debug`, `nc_bringup` |
| `scripts/` | 실행 보조 스크립트 |

## 요구 환경

- Ubuntu와 ROS 2 Jazzy
- Isaac Sim 5.1
- `colcon`, `numpy`, ROS 2 Python 의존성
- 비전 실행 시 OpenCV와 Ultralytics

월드의 `OmniPBR.mdl`, `OmniGlass.mdl` 및 일부 NVIDIA Base Material URL은
Isaac Sim 기본 재질/온라인 에셋을 사용합니다. Isaac Sim 설치와 해당 에셋 접근이
가능해야 재질까지 완전하게 표시됩니다. 프로젝트 고유 USD, URDF, 메시, 텍스처는
저장소 안에 포함되어 있습니다.

## ROS 2 빌드

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

모든 실행 PC에서 같은 Domain ID를 사용합니다.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=144
```

PC A는 제어 노드와 Robot 1·2 노드를 실행합니다.

```bash
ros2 launch nc_bringup pc_a.launch.py
```

PC B는 Vision 1·2 노드를 실행합니다.

```bash
ros2 launch nc_bringup pc_b.launch.py
```

PET depth 보정판을 사용할 때는 기본 PC B launch 대신 다음 하나만 실행합니다.

```bash
ros2 launch nc_bringup pc_b_v2.launch.py
```

디버그 팝업은 별도 터미널에서 실행할 수 있습니다.

```bash
ros2 launch nc_vision_debug debug_popups.launch.py
```

비전 V2와 팝업의 자세한 내용은 [README_INSTALL.md](README_INSTALL.md)를
참고하십시오.

## Isaac Sim 실행

Isaac Sim 설치 경로를 지정한 후 저장소 루트에서 실행합니다.

```bash
export ISAAC_SIM_ROOT=/absolute/path/to/isaacsim
./scripts/run_standalone.sh
```

GUI 없이 실행하려면 `--headless`를 추가하고, 다른 월드를 시험하려면 `--usd`로
경로를 덮어쓸 수 있습니다.

```bash
./scripts/run_standalone.sh --headless
./scripts/run_standalone.sh --usd /absolute/path/to/other_world.usd
```

기본 실행 파일은 저장소 위치를 자동으로 계산하므로 `/home/rokey/...` 같은 사용자별
절대경로가 필요하지 않습니다. `sim/standalone/`의 버전명 파일들은 개발 이력을
보존한 실험판이며, 정상 실행 진입점은 `netclean_standalone.py`입니다.
