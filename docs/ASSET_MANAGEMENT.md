# 시뮬레이션 에셋 및 제출 관리

## 최종 월드 구성 (2026-10 GPU 검증 반영)

실행 기본 월드는 `sim/assets/project1/simulation_integration_v4_factory.usda`이고, 장식 없는 원본 공정 월드는 `simulation_integration_v4.usd`입니다. 두 월드 모두 RTX 3080 + Isaac Sim 5.1에서 6 Cycle 연속 완주를 확인했습니다([`reports/TROUBLESHOOTING.md`](../reports/TROUBLESHOOTING.md)).

| 파일 | 설명 |
|---|---|
| `simulation_integration_v4.usd` | 공정 월드. 같은 폴더의 `simulation_integration_v3.usd`를 sublayer로 얹은 구조 |
| `simulation_integration_v3.usd` | v4의 바탕이 되는 월드. 아래 "v3 · 어망 OBJ" 참고 |
| `simulation_integration_v4_factory.usda` | v4 위에 공장 배경 레이어 3개(`sim/assets/factory/*.usda`)를 얹은 월드(실행 기본) |
| `sim/assets/factory/` | 공장 장식 레이어 · 소품 · 표지판 텍스처 · 재생성 도구. 모두 시각 전용(충돌 · 강체 · 조인트 끔). 구성은 [`sim/assets/factory/README.md`](../sim/assets/factory/README.md) |

- 공장 소품(Isaac Sim 5.1 공개 창고 소품 약 410 MB)은 `sim/assets/factory/isaac/`에 포함되어 있어 인터넷 없이 열립니다.
- 월드를 바꿔 실행하려면 `NETCLEAN_USD=<경로> ./scripts/run_standalone.sh`를 씁니다.
- 코드가 요구하는 prim(로봇 · 어망 · 카메라 · ROS 그래프 · FixedJoint · TCP 19개)과 로컬 참조 에셋이 모두 있음을 USD 의존성 분석으로 확인했습니다.

### v3 · 어망 OBJ

v4가 얹히는 `simulation_integration_v3.usd`는 원본(Notion `project1.zip`) 파일입니다. 아래 "초기 정리 기록"에서 정리했던 v3(OBJ payload 제거본)와 다르며, 원본 v3는 어망의 중복 payload로 `Collected_collected_net/net_collected.obj`(약 18 MB)를 참조합니다. v4 검증은 이 구성으로 했기 때문에 파일을 함께 두었습니다. 이 OBJ는 USD payload로 열 수 없어 로드 경고를 낼 수 있습니다. 정리했던 v3는 Git 이력에 남아 있습니다.

### 포함하지 않은 파일

원본 압축본에는 구버전 월드(`v1` · `v2`), 백업 USD, 중복 로봇 디렉터리와, 월드가 참조하지 않거나 NVIDIA 원격 에셋으로 대체되는 고해상도 재질 텍스처(단일 파일 최대 약 87 MB)가 있습니다. 이들은 저장소에 넣지 않았습니다. 원격 머티리얼은 Isaac Sim이 첫 실행 때 인터넷으로 받습니다.

---

## 초기 정리 기록 (v3 기준)


## 기준 압축본 점검 결과

원본 `isaac_simulation_intergration (2).zip`은 약 666MB이며, 압축 해제 시
151개 파일과 약 724.8MiB의 데이터를 포함합니다. 이 파일은 원본 보관용 자료이며
Git 저장소에는 넣지 않습니다.

메인 실행 파일 `sim/assets/project1/simulation_integration_v3.usd`의 USD 구성과
의존성을 검사해 실제 실행에 필요한 파일만 `sim/assets/project1/` 아래에
보존했습니다. 압축본에만 있는 구버전 월드, 백업 USD, 중복된 로봇 디렉터리와
메인 월드에서 참조하지 않는 고해상도 재질 텍스처는 제외했습니다.

M0609 URDF가 참조하던 DAE 파일은 원본 압축본에 없었습니다. 로컬에 설치된
`dsr_description2` 패키지의 M0609 collision/white 메시 20개를 프로젝트 에셋으로
포함하고, URDF를 저장소 상대경로로 변경했습니다.

`Collected_collected_net/net_collected.obj`는 실제 공정에 사용하는 물리 어망
`Collected_net/World4.usd`와 별개의 중복 payload였습니다. OBJ는 USD payload로
열 수 없어 경고를 발생시켰으므로 중복 prim과 OBJ 파일을 제거했습니다. 실제
`net_green`과 carriage prim은 그대로 유지됩니다.

## 현재 Git 관리 정책

- `build/`, `install/`, `log/`, Python 캐시와 압축 백업은 `.gitignore`로 제외합니다.
- 원본 666MB ZIP을 다시 Git에 추가하지 않습니다.
- 현재 필요한 개별 파일은 모두 50MiB 미만이므로 Git LFS를 사용하지 않습니다.
- 제출 전에 `python3 scripts/verify_submission.py`를 실행합니다.
- 에셋을 변경할 때는 메인 USD를 다른 위치의 깨끗한 클론에서 다시 엽니다.

GitHub 일반 Git은 100MiB보다 큰 단일 파일을 허용하지 않습니다. 향후 반드시
필요한 단일 USD/USDC/모델 파일이 100MiB를 넘는 경우에만 해당 경로를 Git LFS로
관리합니다.

```bash
git lfs install
git lfs track "sim/assets/path/to/large_asset.usd"
git add .gitattributes sim/assets/path/to/large_asset.usd
git commit -m "Track large simulation asset with Git LFS"
```

LFS를 적용한 저장소를 받는 PC에는 Git LFS가 설치되어 있어야 하며, 클론 후
`git lfs pull`로 실제 파일이 내려왔는지 확인해야 합니다. 현재 저장소에는 이
추가 절차가 필요하지 않습니다.

## 제출 방법

GitHub 링크 제출이 가능하면 `main` 브랜치 URL과 최종 커밋 해시를 제출합니다.
단일 ZIP 제출이 필요하면 최종 변경을 커밋한 뒤 다음 명령을 실행합니다.

```bash
./scripts/make_submission_archive.sh
```

이 명령은 Git에 추적된 최종 커밋만 묶으므로 `build`, `install`, `log`, 로컬 캐시와
원본 압축 백업이 자동으로 빠집니다. 생성된 ZIP은 저장소의 한 단계 위에 만들어지며
그 ZIP을 Git에 다시 추가하지 않습니다.

## 외부 런타임 에셋

`OmniPBR.mdl`, `OmniGlass.mdl`과 NVIDIA Base Material 일부는 Isaac Sim의 기본
또는 온라인 재질 resolver가 제공합니다. 이 항목들은 기하·물리·ROS prim이 아니라
표시용 재질이며, 오프라인 PC에서는 일부 색상이나 표면 표현이 달라질 수 있습니다.
