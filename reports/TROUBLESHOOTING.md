# 트러블슈팅 — GPU(RTX 3080) 실검증 (2026-10-09)

환경: RTX 3080 10 GB · 드라이버 580.173.02 · Ubuntu 24.04.4 · ROS 2 Jazzy · Isaac Sim 5.1.0 · ultralytics 8.4.126 · opencv-python 5.0.0.93
차수별 실행 로그: `reports/gpu_logs/run*/` (검증 복사본 `~/netclean_gpu_verify` 는 정리하며 삭제), 수정 반영본: 이 폴더.

| # | 증상 | 원인 | 조치 | 상태 |
|---|---|---|---|---|
| 2 | `--headless` 실행 시 `/camera1·2/*` 토픽 없음 | standalone 이 `world.step(render=not ARGS.headless)` 로 렌더링을 건너뜀 | 헤드리스에서도 렌더링(`RENDER_FRAMES`), 물리만 돌릴 때는 `--no-render` | 해결(2026-10-10), 헤드리스 6 Cycle 완주(16차) |
| 3 | Vision 1 이 매 프레임 `KeyError: 16` → Station 1 timeout | pip OpenCV 5 와 Jazzy `cv_bridge`(OpenCV 4.6 빌드)의 `cv2_to_imgmsg` 충돌 | `nc_vision/image_msg_utils.py` 로 디버그 이미지 직접 변환 | 해결 |
| 4 | 2번째 Cycle 에서 Vision 1 이 도착 신호를 무시 | control 이 `/stationN/net_arrived` 를 False 로 되돌리지 않음 | 작업 완료·오류 시 False 발행, 대기 중 1 s 마다 True 재발행 (`control_node.py`) | 해결 |
| 5 | 리셋 뒤 Vision 1 이 로그 없이 116–137 s 무응답 → Station 1 timeout | 아래 상세 | `nc_vision/sync_utils.py` | 해결, 6 Cycle 연속 완주 |
| 7 | 최종 설정(비전 좌표)에서 Robot 2 흡착이 가끔 거부 (표면 거리 3.3–4.7 cm > 허용 3 cm) | Vision 2 가 어망이 멈추기 전 프레임을 사용 → 목표가 이송 반대 방향으로 0–5 cm 밀림. 아래 상세 | `vision2_node_v2.py` `arrival_settle_sec`(1.0 s): 도착 후 첫 프레임 시각 + 1 s 이후 프레임만 사용 | 해결, 최종 설정 6 Cycle 연속 완주 |
| 6 | Robot 1 이 일부 절단점에서 IK 실패 → 2회 시도 후 skip (5차 4 Cycle 중 2회, 7차 0회) | 어망 좌우 가장자리의 위쪽 절단점이 M0609 도달 범위를 6.5–11.4 mm 넘음. 아래 상세 | `robot1_node` 사전 도달 검사(`motion_utils.pull_into_reach`): 범위 밖 절단점을 절단선 따라 안쪽으로 당기고, 그래도 안 되면 시도 없이 skip | 해결(2026-10-10), 16·17차 IK 실패 0 |
| 8 | Robot 2 페트병 흡착 여유가 3 mm(표면 거리 2.70 cm / 허용 3 cm). 시뮬이 느려지면 거부 → 공정 정지(공장 배경 12·13차) | CONTACT 가 물체 중앙 표면점까지 들어가 흡착컵이 더 튀어나온 부분에 먼저 닿고, 강성 낮은 1축이 밀려 TCP 가 4–7 cm 옆으로 미끄러짐. 아래 상세 | `robot2_node` `CONTACT_STANDOFF_M` 12 mm(표면 앞에서 정지), 대기를 시뮬 시간 기준(`sleep_sim_time`) | 해결(2026-10-10), 페트병 0.37–0.61 cm |

## #5 상세 — 리셋 후 Vision 무응답

**원인.** `message_filters.ApproximateTimeSynchronizer` 는 토픽마다 짝 없는 메시지를 최대 `queue_size`(10)개 보관하고,
넘치면 *타임스탬프가 가장 작은* 메시지를 버린다. standalone 의 Cycle 리셋(`world.reset()`)은 시뮬레이션 시간을 0 으로 되돌린다.
리셋 전 프레임이 큐에 10개 남아 있으면, 리셋 뒤 들어오는 새 프레임(작은 타임스탬프)은 들어오자마자 버려져
시뮬레이션 시간이 남은 프레임 시각을 넘을 때까지(약 1 Cycle) RGB-D 쌍이 만들어지지 않는다.

**증거 (6차 실행, `[DIAG]` 큐 상태 로그).**

```text
last_rgb=118.53 armed=False | rgb:n=10 min=115.17 max=118.53 | depth:n=6  min=116.38 max=118.30   <- 리셋 직전
last_rgb=0.67   armed=False | rgb:n=10 min=0.63   max=118.48 | depth:n=10 min=117.98 max=118.78   <- 리셋 직후
last_rgb=7.30   armed=True  | rgb:n=10 min=7.27   max=118.48 | depth:n=10 min=117.98 max=118.78   <- 도착, 쌍 없음
last_rgb=118.12 armed=True  processed=True | depth:n=8 min=118.28                                <- 117.98 s 를 넘자 재개
```
도착 → 절단 Goal 136.8 s (벽시계).

**조치.**
- 도착 신호로 재무장할 때 동기화 큐를 비운다 (Vision 1 · 2).
- RGB 타임스탬프가 1 s 넘게 뒤로 가면 큐를 비우고 경고를 남긴다. 이 콜백은 동기화기보다 먼저 등록한다.
- `test/test_sync_utils.py`: 실제 `ApproximateTimeSynchronizer` 로 버그 재현 + 수정 확인 (3 tests).

**검증 (7차 실행, Robot 2 는 USD 실측 접촉점 진단 모드).**

| 항목 | 수정 전 | 수정 후 |
|---|---|---|
| 도착 → 절단 Goal | 최대 136.8 s | 0.0–2.0 s |
| 연속 완주 Cycle | 3 | 6 (수동 종료) |
| Cycle 시간 | — | 120.3–125.6 s (평균 124.3 s) |
| 절단점 / 제거 | — | 36/36 · 18/18 |

## #6 상세 — Robot 1 IK 실패 (절단점 skip)

**증상.** 페트병 위쪽 절단점(P1)은 3·4·6차에서 매번, 5차에서는 페트병 P1·캔 P1 이 한 번씩 IK 실패(`solver_success=False`)로
2회 시도 후 skip. 7차 6 Cycle 에서는 0회.

**분석.** Isaac 의 IK 는 Lula CCD. 시드는 현재 관절 자세 1개 + 고정 난수(`sampling_seed` 123456) 100회라서 결과가 결정적이다
→ 같은 자세에서의 재시도는 항상 같은 결과.

**재현 (`reports/ik_analysis/ik_repro.py`, Isaac Sim 없이 Lula 만 사용, FK 가 로그와 0.3 mm 이내 일치).**
- 실패 목표에서 위치 오차 8.88 mm (허용 1 mm) 로 실패 재현.
- 목표 Y 를 5 mm 씩 옮기면 y = −2.425 m 까지 성공, −2.430 m 부터 실패하며 오차가 거리에 비례해 증가.
- 난수 시드 40가지 / 시도 500회로도 해 없음 → 솔버가 해를 놓친 것이 아니라 **기구학적으로 도달 불가**.
- 경계의 해는 관절 3 = 0 (팔꿈치 완전히 펴짐).

**도달 여유.** 수평 절단 자세(공구 −X)에서 손목 중심은 TCP 뒤 0.251 m. 손목 중심은 어깨(베이스 위 0.6345 m)에서
0.779 m(상완 0.411 + 전완 0.368) 안에 있어야 한다. 로그의 절단 CONTACT 명령 86개에 대해
`도달 여유 = 0.779 m − 어깨~손목 거리` 를 계산 (`reach_table.py`, 결과 `gpu_logs/robot1_contact_reach.csv`):

| | 개수 | 도달 여유 |
|---|---|---|
| IK 실패 | 10 | −6.5 ~ −11.4 mm (전부 음수) |
| IK 성공 | 76 | −1.0 ~ +168 mm (−1.0 mm 는 IK 허용 1 mm 이내) |

7차에서 실패가 없었던 것은 페트병 P1 이 +8.7 mm, 캔 P1 이 +2.2 mm 로 경계 바로 안쪽이었기 때문이다.
리셋마다 물체 위치가 1–2 cm 달라져 어망 가장자리의 위쪽 절단점이 경계를 넘나든다.

**대책 (미적용, 팀 결정 필요).**
- `robot1_node` 에서 접촉 전 도달 여유를 검사해 범위 밖이면 경계 안쪽으로 당기거나 즉시 skip (재시도·복구 약 20 s 절약).
- Station 1 어망 정지 위치 또는 Robot 1 베이스(현재 y = −2.0 m) 조정.
- 먼 절단점에서 공구를 기울인 자세 허용.
- `cut_margin_ratio`(0.10) 축소로 위쪽 절단점을 물체 중심 쪽으로 당김.

## #7 상세 — Vision 2 좌표 오차 (흡착 거부)

**잘못된 첫 진단(철회).** 처음에는 Vision 2 좌표를 robot2_node 의 `MEASURED_USD_CONTACT_POINT_WORLD` 와 비교해 "6–16 cm 오차" 로 보고했다.
이 점들은 물체 AABB 의 모서리(최대 x·y·z)라 Vision 이 겨냥하는 물체 중심과 다른 점이다.

**카메라 모델 검증.** 렌더 카메라 `/World/AG_Camera2/RSD455/Camera_OmniVision_OV9782_Color` 의 위치 (−0.1, 2.0, 1.2)·자세·렌즈
(fx = 1.8 / 3.896 × 640 = 295.69) 가 코드 상수·CameraInfo 와 일치. 실행 중 측정한 물체 위치를 영상에 투영하면 YOLO 박스 중심과
3.5–5 px 이내 (`reports/vision2_analysis/truth_overlay_cycle1.png`). Station 1 절단은 물체를 밀지 않음(실측 변화 0.1 mm 미만).

**원인.** Control 은 어망이 목표 ±2 cm 에 들어오면 도착을 알리고(위치 보고 20 Hz), Vision 2 는 도착 직후 첫 프레임을
시각 제한 없이 사용했다(`minimum_frame_stamp_ns = -1`). 어망은 0.5 m/s 로 움직이므로 그 프레임은 멈추기 직전 장면이다.
8차 실행 도착 직후 프레임의 페트병 위치는 정착 위치보다 0 ~ −29 px (최대 약 5 cm) 밀려 있었고 사이클마다 달랐다
(`reports/vision2_analysis/baseline_arrival_frame_offsets.txt`).

**조치.** `arrival_settle_sec`(기본 1.0 s, `vision2_v2.yaml`): 도착 후 첫 프레임 시각 + 1 s 이후 렌더된 프레임만 검출에 사용.
처음 구현은 노드 시계(`/clock`) 기준이었는데, `/clock` 은 리셋돼도 계속 증가하고 카메라 시각은 0 으로 돌아가
(USD 카메라 그래프 `resetSimulationTimeOnStop=True`) 9차 3번째 Cycle 에서 멈췄다 → 카메라 프레임 시각 기준으로 수정.

**검증 (10차, 최종 설정 `USE_MEASURED_USD_CONTACT_POINTS=False`).**

| 항목 | 수정 전(8차) | 수정 후(10차) |
|---|---|---|
| 페트병 검출 u (정착 위치 130 px) | 96–120 px | 129 px (6 Cycle 동일) |
| 연속 완주 | 4 Cycle, 5번째 흡착 거부 | 6/6 |
| 흡착 거부 · 제거 | 1회 · 12/15 | 0회 · 18/18 |
| Cycle 시간 | — | 112.8–134.8 s |
| 물체 중심 대비 오차(가로·세로) | — | 1.4 cm 이내 |

Vision 1 에도 같은 정착 대기(`vision1_v2.yaml` `arrival_settle_sec: 1.0`)를 적용했다(2026-10-10). 적용 후 캔 P1 절단점이 y −1.701 → −1.645 로 5.6 cm 바뀌었다 — 이전 절단점은 움직이던 어망에서 읽은 값이었다.

## #8 상세 — Robot 2 흡착 여유 (2026-10-10)

**측정(15차, `gpu_logs/run15_suction_diag`).** 흡착 판정 순간의 TCP 와 물체 AABB 를 로그에 남겼다.

| 물체 | 명령한 CONTACT 목표 | 판정 순간 TCP | 차이 | 물체 AABB |
|---|---|---|---|---|
| 페트병 | (−0.609, 1.671, 0.997) | (−0.580, 1.613, 0.985) | 6.6 cm | 정지 위치 그대로 |
| 부표 | (−0.577, 2.019, 1.219) | (−0.575, 2.019, 1.219) | 0.2 cm | 정지 위치 그대로 |
| 캔 | (−0.610, 2.307, 1.083) | (−0.576, 2.375, 1.070) | 7.8 cm | 정지 위치 그대로 |

물체는 움직이지 않았고 **로봇 TCP 가 옆으로 미끄러졌다.** 목표는 물체 중앙의 깊이(표면)인데, 페트병은 가장 튀어나온 면(AABB 앞면 x = −0.6008)이
목표보다 8.5 mm 앞에 있다. 흡착컵이 거기에 먼저 닿고, 강성이 낮은 1축(160)이 밀려 곡면을 따라 미끄러진다. 미끄러진 양이 시간에 따라 변해,
벽시계 대기·시뮬 속도에 따라 2.3–3.9 cm 로 흔들렸다(공장 배경 복사본 12–14차).

**조치.** CONTACT 를 표면점에서 어망 법선 방향으로 12 mm 앞에 멈춘다(`CONTACT_STANDOFF_M`). 대기 3곳(`PREGRASP_HOLD`, `CONTACT_HOLD`,
`DIRECT_SUCTION_HOLD`)과 Robot 1 `CONTACT_HOLD` 는 시뮬 시간 기준으로 바꿨다(`nc_robot/motion_utils.py`, 시험 6개).

**검증 (Robot 1 도달 검사 · Vision 1 정착 대기 · 헤드리스 수정 포함, 흡착 허용 3 cm 그대로).**

| 항목 | 수정 전(10차, 원본) | 16차 원본 월드·헤드리스 | 17차 공장 배경·GUI (복사본) |
|---|---|---|---|
| 연속 완주 | 6/6 | 6/6 | 6/6 (수정 전 12차는 1 Cycle 에서 정지) |
| 페트병 흡착 표면 거리 | 2.70 cm | 0.37–0.61 cm | 0.44–0.61 cm |
| 부표 · 캔 | 0.09 · 0–0.23 cm | 0.59–0.60 · 0 cm | 0.60 · 0 cm |
| 흡착 거부 · 제거 | 0 · 18/18 | 0 · 18/18 | 0 · 18/18 |
| Robot 1 IK 실패 / 절단점 | 1회 skip · 35/36 | 0 · 36/36 (안쪽 당김 7회) | 0 · 36/36 (안쪽 당김 7회) |
| Cycle 시간(벽시계) | 112.8–134.8 s | 121.1–148.4 s | 134.6–142.9 s |

로그: `gpu_logs/run16_fixes_headless/`, 공장 배경은 `netclean_run_set_factory/reports/gpu_logs/run17_factory_fixes/`.

## 남은 이슈
- 카메라 실제 발행 주기는 약 46 Hz (녹화 프레임 수 기준). `ros2 topic hz` 는 12–30 Hz 로 낮게 측정됨.
- 캔 P1 절단점은 정착 후 위치(y −1.645)에서 매 Cycle 도달 여유 −10 mm 라 19 mm 안쪽으로 당겨 자른다. 배치 조정(어망 정지 위치·로봇 베이스)이 근본 대책.

## 사진 · 영상 (`reports/media/`)
- `process_sheet.png` — 탐지 → 절단 → 흡착 제거 → 재검사 4단계
- `vision1_detection_cycle3.png`, `vision2_recheck_empty_cycle5.png`, `isaac_view.png`
- `station1_cut_cycle3.mp4` (2x), `station2_remove_cycle3.mp4` (2x), `isaac_full_cycle_4x.mp4` (4x)
- `run6_repro_isaac.mp4` — 수정 전 재현 실행 화면 녹화
