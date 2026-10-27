# 문서 지도

NetClean 저장소의 문서를 한곳에 모았습니다. 프로젝트 소개 · 결과 · 팀 정보는 [루트 README](../README.md)에 있습니다.

## 처음 읽는 순서

1. [루트 README](../README.md) — 무엇을 왜 만들었고, 어떻게 동작하며, 결과가 어땠는지
2. [README_INSTALL.md](../README_INSTALL.md) — 설치 · 빌드 · 단위 시험 · 전체 실행
3. [ROS 2 패키지 지도](../ros2_ws/src/README.md) — 패키지 · 노드 · launch · 파라미터 · 시험
4. [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — 실행하며 막혔던 7가지와 해결

## 문서별 안내

| 문서 | 내용 |
|---|---|
| [`../README.md`](../README.md) | 프로젝트 소개 · 데모 · 핵심 기능 · 시스템 구조 · 결과 · 팀 |
| [`../README_INSTALL.md`](../README_INSTALL.md) | 설치 · 빌드 · 시험 · 전체 실행 순서 · 월드 선택 · 연동 확인 · 문제 해결 |
| [`../ros2_ws/src/README.md`](../ros2_ws/src/README.md) | ROS 2 패키지 6개 — 역할 · 실행 파일 · 주요 파라미터 · 인터페이스 · 단위 시험 |
| [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) | 트러블슈팅 7가지 — 증상 · 원인 · 해결 · 결과와 상세 설명 |
| [`ASSET_MANAGEMENT.md`](ASSET_MANAGEMENT.md) | 최종 월드(v4 · 공장 배경) 구성 · 에셋 포함 범위 · Git 관리 정책 |
| [`../sim/assets/factory/README.md`](../sim/assets/factory/README.md) | 공장 배경 에셋 레이어 구성 |
| [`../models/netclean_yolo11n/README.md`](../models/netclean_yolo11n/README.md) | YOLO11n 탐지 모델 — 학습 설정 · 성능 · 사용 방법 |
| [`../ros2_ws/src/nc_vision_debug/README.md`](../ros2_ws/src/nc_vision_debug/README.md) | Vision 1 · 2 디버그 팝업 설치 · 실행 |
| [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md) | 외부 에셋 출처 · 라이선스 |

## 이미지

[`images/`](images/)에는 루트 README에 쓰는 그림과 GIF가 있습니다.

| 파일 | 내용 |
|---|---|
| `factory_process.gif` | 공장 배경 월드에서의 전체 공정(8배속) |
| `process_sheet.png` | 탐지 → 절단 → 흡착 제거 → 재검사 4단계 한 장 요약 |
| `station1_cut.gif` | Station 1 — Robot 1 절단 |
| `station2_remove.gif` | Station 2 — Robot 2 흡착 제거 · 재검사 |
| `vision1_detection.png` | Vision 1 탐지 결과와 절단점 P1 · P2 |
| `vision2_recheck.png` | Vision 2 재검사 — 제거 후 빈 어망 |
| `factory_isaac_view.png` | 공장 배경 월드 Isaac Sim 화면 |
