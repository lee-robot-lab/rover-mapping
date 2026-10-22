# KSRC 최종 맵 임무 이관 (2026-10-22)

Codex가 만든 새 경기장(`rover/docs/handoff/2026-10-22-ksrc-final-map.md`)에서 1회차·2회차 임무를 시뮬로 돌리기 위한 파일 모음이다. 이 폴더는 **확정한 설정(f판)** 을 재현하는 데 필요한 것만 담는다.

## 임무 정의 (사용자 확정)
- **1회차**: 구역 1에서 30° 시도(전진 5 s·전체 12 s) → 후퇴 → 옆으로 이동 → 15° 등반 → 정상에서 30° 램프 쪽으로 돌아 4 s 정지(관찰) → 180° 회전 → 복도 직진 → 바깥 7° 하행(구역 4→10→9→8→7→6→1).
- **2회차**: 15° 등반 → 7° 하행(30° 시도·관찰 없음, 내리막 0.7배).
- 시작 pose (3.6, 2.82) m, 방향 −y. 뎁스 관측은 끈다(`--region-observation none`).

## 파일
| 파일 | 내용 |
|---|---|
| `anchors_ksrc_final_v2.json` | 임무 anchor(시작·접합점 (3.2,2.75)·정상 기준점 (2.8,0.4)·세 경로 직각 꼭짓점·설정) |
| `scenario.json` | 구역별 단계 좌표, 모서리 gate 좌표, 정상 관찰(look) 설정, 경사 회전 금지(`steep_pitch_deg` 22°) |
| `yw17.patch` | yw16 시뮬 코드(`experiments/`)에 scenario.json 지원·정상 관찰 단계·경사 회전 금지·단계별 도착 반경을 더하는 패치 |
| `harness_kinematic.py` | Isaac 없이 임무 코드+MPPI를 점질량 로버와 폐루프로 돌려 단계 순서를 확인 |
| `../2026-10-22-hardcoded-map-assumptions.md` | 맵 의존 하드코딩 감사 |

코드: `experiments/site_sim/rebuild_mission.py`(STL→임무 번들), `experiments/handheld_mapping/control_gateway.py`·`teleop/ksrc_snapshot/pi/teleop/gateway_client.py`(텔레옵↔자동 중계와 조이스틱 화면).

## 재현
```bash
# 1) 임무 번들 (STL은 모래 포함 m 단위, 크레이터는 --crop으로 제외해야 삼각형 200만 한도 안)
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.site_sim.rebuild_mission \
  --stl /home/su/rover/outputs/ksrc_final_map/20261022_board_level_sand/KSRC_board_level_sand_m.stl --units m \
  --crop -0.1 -0.1 4.1 3.3 --anchors docs/experiments/2026-10-22-ksrc-final-mission/anchors_ksrc_final_v2.json \
  --fillet 0.25 --output outputs/site_sim/ksrc_final/mission_fillet25      # 로버 반지름 기본 0.21, 이음매 허용 0.02 m
cp <KSRC regions.json> outputs/site_sim/ksrc_final/mission_fillet25/regions.json   # mission_regions/regions.json
cp docs/experiments/2026-10-22-ksrc-final-mission/scenario.json outputs/site_sim/ksrc_final/mission_fillet25/
# 2) 시뮬 코드: yw16 snapshot의 experiments/ 안에서
patch -p2 < yw17.patch
# 3) Isaac run: 장면 USD는 fence_1m_offset_1m/KSRC_final_black_fence_1m.usd
```
Isaac run 인자는 yw15 명령에서 `--wall-yaw --wall-yaw-sigma-deg 4 --filter-gyro-bias-init-sigma-dps 0.003 --wall-plan --corner-gate --mppi-vy 0 --mppi-clearance 1.5 --mppi-clearance-margin 0.15 --parity-physics`를 쓰고 `--region-observation none`으로 바꾼다. 임무 설정(`mission.json` settings): 1회차 `summit_look=1 trial30_forward_limit_s=5 trial30_elapsed_limit_s=12`, 2회차 `descent_speed_scale=0.7 trial30_forward_limit_s=0.5 trial30_elapsed_limit_s=0.5`.

## 시뮬 결과 (Isaac, μ 0.4, 시드별)
- **2회차 8/8 완주**(시드 2·4·7·9 × 두 판, 219~269 s).
- **1회차 f판(확정) 4/4 완주**(시드 2·4·7·9: 270·274·267·271 s). 관찰·회전 중 벽 최소 거리 0.30 m 이상.
- 1회차 이전 d판(정상 기준점 (3.2,0.4)·얕은 관찰)은 시드 8개 중 3개만 완주: 실패는 모두 15° 램프에서 ORB가 약 3 s(46프레임) 끊기며 추정이 0.6 m 이상 어긋난 경우. f판에서는 15° 램프 구간 상실이 없었다. 차이의 원인은 미확정(정상 기준점 이동이 램프 마지막 경로를 바꿨을 가능성, 또는 우연; 시드 4개뿐).
- 가파른 30° 입구에서 전진+회전 명령이 겹치면 전복(처음 판 시드 4·9) → 경사 22° 초과에서는 회전 명령 0(직진·직후진만).

## 한계·미검증
- 실기(Pi) 시험, 시작 오차(±5 cm, ±5°) 시뮬, 모래 색(실제는 더 흰색)·실제 마찰은 하지 않았다.
- 15° 램프 ORB 상실의 원인 확정과 f판 시드 확대(1·3·5·6)는 남았다.
- Pi용 MPPI 실행기(중계에 START/STOP·상태를 연결)는 아직 없다.
