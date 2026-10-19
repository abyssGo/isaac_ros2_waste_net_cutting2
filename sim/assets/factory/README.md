# 폐어망 공장 배경 (factory background)

원본 월드 `../project1/simulation_integration_v4.usd` 는 **수정하지 않는다**. 장식은 별도 레이어로 만들어 sublayer 로 얹는다.

| 파일 | 내용 | 카메라에 보이나 |
|---|---|---|
| `factory_outside_fov.usda` | 1단계: 로봇 뒤 벽·기둥·랙, 측벽(x ≥ 0.5), 선별품 팔레트(PET/캔/부표), 드럼통, 지게차, 카트, 통로 표시, 안전 표지 | 안 보임 (그림자도 시야 밖) |
| `factory_station_backdrop.usda` | 2단계: 어망 뒤 단색 차폐판 1장 (x = −2.6, 폭 10 m, 높이 4.3 m, 무광 회색 albedo 0.35) | **보임 (의도된 유일한 변화)** |
| `factory_enclosure.usda` | 2단계: 원래 흰 벽(`/World/wall_plane`) 비활성화, 원격 벽·측벽, 지붕, 펜던트 조명, 랙, 수거 어망 적치장 | 안 보임 (차폐판이 가림) |
| `../project1/simulation_integration_v4_factory_step1.usda` | 1단계 + v4 | |
| `../project1/simulation_integration_v4_factory.usda` | 1·2단계 + v4 (**이 복사본의 기본 월드**) | |

모든 장식은 시각 전용이다: 충돌·강체·조인트 비활성화. 지붕·조명·로봇 뒤 벽은 그림자를 만들지 않는다(`primvars:doNotCastShadows`).
새 광원은 추가하지 않았다(기존 DistantLight 하나 그대로).

## 다시 만들기 / 검증
```bash
export ISAAC_SIM_ROOT=~/isaacsim
EXT=$(ls -d $ISAAC_SIM_ROOT/extscache/omni.usd.libs-*)
PY="env PYTHONPATH=$EXT LD_LIBRARY_PATH=$EXT/bin $ISAAC_SIM_ROOT/kit/python/bin/python3 -P"
$PY tools/fetch_isaac_assets.py      # (최초 1회, 인터넷) Isaac 5.1 창고 소품 → isaac/ (약 430 MB)
python3 tools/make_signs.py          # 한글 표지판 텍스처 → textures/
$PY tools/build_factory_world.py     # 레이어 4개 생성 + 카메라 시야/그림자 검사 → visibility_check.txt
$ISAAC_SIM_ROOT/python.sh tools/render_cameras.py --usd ../project1/simulation_integration_v4_factory.usda --out /tmp/r_final
$ISAAC_SIM_ROOT/python.sh tools/render_cameras.py --usd ../project1/simulation_integration_v4.usd --out /tmp/r_orig
python3 tools/compare_vision.py /tmp/r_orig /tmp/r_final   # YOLO·Vision2 3D 목표 비교
```
`build_factory_world.py` 는 장식 prim 마다 바운딩 박스를 표본화해 두 카메라 시야(94.5°, 5 % 여유)와
광원 방향(−X, 45° 하향) 그림자 경로를 검사하고, 차폐판 외 무엇이든 영상에 들어가면 실패(exit 1)한다.

## 장식을 추가할 때 규칙
- 로봇 쪽(x > 0)에 높이 h 인 물체를 x 에 두면 그림자가 x − h 까지 뻗는다. 그림자가 x < 0.3 에 닿으면 검사가 잡는다.
- 어망과 차폐판 사이(−2.6 < x < −0.1)에는 아무것도 두지 않는다. 카메라에 그대로 보인다.
- 지붕·천장 소품은 반드시 그림자 끄기. 광원·발광 재질 추가 금지(카메라 밝기 변화).
- 추가 후 `build_factory_world.py` → `render_cameras.py` → `compare_vision.py` 를 다시 돌린다.

에셋 출처: NVIDIA Isaac Sim 5.1 공개 에셋(`Environments/Simple_Warehouse/Props`, `Props/Forklift`),
수거 어망 적치장은 팀 어망 메시(`project1/Collected_collected_net/.../net_collected.usd`)를 물리 끄고 재사용.
