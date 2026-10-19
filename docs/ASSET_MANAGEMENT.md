# 시뮬레이션 에셋 관리

## 월드 구성

실행 기본 월드는 `sim/assets/project1/simulation_integration_v4_factory.usda`이고, 장식 없는 공정 월드는 `simulation_integration_v4.usd`입니다. 두 월드 모두 6 Cycle 연속 완주를 확인했습니다([`TROUBLESHOOTING.md`](TROUBLESHOOTING.md)).

| 파일 | 설명 |
|---|---|
| `simulation_integration_v4.usd` | 공정 월드. 같은 폴더의 `simulation_integration_v3.usd`를 sublayer로 얹은 구조 |
| `simulation_integration_v3.usd` | v4의 바탕이 되는 월드(로봇 2대 · 어망 이송부 · 카메라 2대 · ROS 그래프) |
| `simulation_integration_v4_factory.usda` | v4 위에 공장 배경 레이어 3개(`sim/assets/factory/*.usda`)를 얹은 월드(실행 기본) |
| `sim/assets/factory/` | 공장 장식 레이어 · 소품 · 표지판 텍스처 · 재생성 도구. 모두 시각 전용(충돌 · 강체 · 조인트 끔). 구성은 [`sim/assets/factory/README.md`](../sim/assets/factory/README.md) |
| `sim/assets/project1/robot2_sample/` | M0609 URDF(저장소 상대경로) · Lula descriptor · collision/white 메시 |
| `sim/cobot3_ws/` | Robot 1이 상대경로로 참조하는 M0609 USD |

- 공장 소품(Isaac Sim 5.1 공개 창고 소품 약 410 MB)은 `sim/assets/factory/isaac/`에 포함되어 있어 인터넷 없이 열립니다.
- 월드를 바꿔 실행하려면 `NETCLEAN_USD=<경로> ./scripts/run_standalone.sh`를 씁니다.
- 코드가 요구하는 prim(로봇 · 어망 · 카메라 · ROS 그래프 · FixedJoint · TCP 19개)과 로컬 참조 에셋이 모두 있는지 USD 의존성 분석으로 확인했습니다.
- `Collected_collected_net/net_collected.obj`(약 18 MB)는 v3가 참조하는 어망 payload입니다. USD payload로 열 수 없어 로드 경고가 날 수 있지만, 6 Cycle 완주 구성과 같은 파일이므로 지우지 않습니다.
- 월드가 참조하지 않는 고해상도 재질 텍스처(단일 파일 최대 약 87 MB)는 넣지 않았습니다. 원격 머티리얼은 Isaac Sim이 첫 실행 때 인터넷으로 받습니다.

## Git 관리 정책

- `build/`, `install/`, `log/`, Python 캐시와 압축 파일은 `.gitignore`로 제외합니다.
- 모든 개별 파일이 50MiB 미만이므로 Git LFS를 사용하지 않습니다.
- 에셋을 바꿀 때는 메인 USD를 다른 위치의 깨끗한 클론에서 다시 열어 참조가 깨지지 않았는지 확인하고, `python3 scripts/verify_submission.py`를 실행합니다.

GitHub 일반 Git은 100MiB보다 큰 단일 파일을 허용하지 않습니다. 반드시 필요한 단일 USD/USDC/모델 파일이 100MiB를 넘는 경우에만 해당 경로를 Git LFS로 관리합니다.

```bash
git lfs install
git lfs track "sim/assets/path/to/large_asset.usd"
git add .gitattributes sim/assets/path/to/large_asset.usd
git commit -m "Track large simulation asset with Git LFS"
```

LFS를 적용한 저장소를 받는 PC에는 Git LFS가 설치되어 있어야 하며, 클론 후 `git lfs pull`로 실제 파일이 내려왔는지 확인해야 합니다.

## 제출 방법

GitHub 링크 제출이 가능하면 `main` 브랜치 URL과 최종 커밋 해시를 제출합니다. 단일 ZIP 제출이 필요하면 최종 변경을 커밋한 뒤 다음 명령을 실행합니다.

```bash
./scripts/make_submission_archive.sh
```

이 명령은 Git에 추적된 최종 커밋만 묶으므로 `build`, `install`, `log`와 로컬 캐시가 자동으로 빠집니다. 생성된 ZIP은 저장소의 한 단계 위에 만들어지며 그 ZIP을 Git에 다시 추가하지 않습니다.

## 외부 런타임 에셋

`OmniPBR.mdl`, `OmniGlass.mdl`과 NVIDIA Base Material 일부는 Isaac Sim의 기본 또는 온라인 재질 resolver가 제공합니다. 이 항목들은 기하 · 물리 · ROS prim이 아니라 표시용 재질이며, 오프라인 PC에서는 일부 색상이나 표면 표현이 달라질 수 있습니다.
