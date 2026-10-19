# Robot 1 IK 실패 분석 (2026-10-09)

- `ik_repro.py` — Isaac Sim 없이 Lula 만으로 실패 목표를 재현 (`~/isaacsim/python.sh ik_repro.py`). 결과: `ik_repro_output.txt`
- `reach_table.py` — `../gpu_logs/run*/` 의 robot1 모션 명령과 IK 결과를 짝지어 CONTACT 명령마다 도달 여유를 계산 (`python3 reach_table.py`). 결과: `reach_table_output.txt`, `../gpu_logs/robot1_contact_reach.csv`

결론: IK 실패 10건은 모두 도달 여유(팔 길이 0.779 m − 어깨~손목 중심 거리)가 −6.5 ~ −11.4 mm 인 목표였다.
성공 76건은 모두 −1.0 mm 이상. 솔버 문제가 아니라 M0609 도달 범위 경계 문제다. 상세는 `../TROUBLESHOOTING.md` #6.
