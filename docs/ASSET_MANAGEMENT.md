# 시뮬레이션 에셋 및 제출 관리

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
