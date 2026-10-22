# 맵·시나리오 하드코딩 감사 (2026-10-22)

대상: `outputs/pi_upper_control/bundle_yw15/experiments/` (region-mppi 경로, yw15). 읽기 전용 감사이며 핵심 줄은 직접 다시 확인했다(`region_mission.py:27-28,152-170`, `run.py:416`, `wall_yaw.py:89`, `wall_track.py:63`, `site_mission.py:71`).
표기: [R]=시작할 때 예외, [L]=움직인 뒤 늦게 실패, [S]=오류 없이 잘못 동작. 분류 A=옛 좌표, B=옛 구역/단계/경로 이름, C=옛 기하 가정, D=시나리오 튜닝.

## 위험도 상위 10개
1. `region_mission.py:27-28,30` — `gyusa_regions.json`/`gyusa_region_portals.json`을 고정 이름으로 로드, 지도 해시 검사 없음, 유일한 검사는 "start가 옛 구역 1 안". [S]
2. `region_mission.py:152-170,298` — 단계 사슬과 경유 좌표(entry30 [3.8,.4], retreat30 [3.7,.4], entry15/retreat15 [3.85,1.2], return10~6 [6.1,-.4] [6.1,-1.2] [5.25,-1.2] [2.9,-1.2] [2.9,-.45])가 코드에 박혀 있고 mission의 junction/trial_entry를 읽지 않는다. [S]/[L]
3. `wall_yaw.py:89-91`, `wall_track.py:113` — 벽 방향을 90° 배수 축으로 스냅. 경기장이 site 축에서 12° 미만 기울면 ESKF yaw에 편향이 조용히 들어간다. [S]
4. `region_mission.py:54` + `run.py:488-494` — `CORNER_GATE` 좌표와 단계 이름('return9','return7')이 두 파일에 중복 결합. 어긋나면 6 s 뒤 `timeout_accept`로 그냥 통과. [S]
5. 구역 id 의미 고정(1 시작, 2 30°, 3 15°, 4 정상, 4→10→9→8→7→6→1 하산): `region_mission.py:29,129,144-146,164,236,273`. [S]/[L]
6. `region_mission.py:312,317` + `site_mission.py:18` — route '30'은 35°, 그 외는 20° 경사 한도. 램프 각도가 바뀌면 계획 불가 또는 허용 과다. [L]/[S]
7. `run.py:416` — 전복 판정이 "시작점 높이 −0.1 m" 기준. 시작점이 가장 낮지 않으면 거짓 `fallen_or_tipped`. [S]
8. `wall_track.py:63` 복도 폭 0.6–1.0 m, 앞벽 0.35–0.8 m, `region_mission.py:91` 기대거리 실패 시 0.4 m. 범위 밖이면 보정이 조용히 꺼진다. [S]
9. `site_mission.py:127,131,71` — region-mppi가 안 쓰는데도 유효한 '10/15/30' 경로 3개와 `site_name ∈ {yeouido,sacheon,demo}`가 필수. [R]
10. `region_navigation.py:132`(구역 경계 0.125 m 하드코딩, JSON의 `boundary_overlap_m` 무시), `region_observation.py:84,103`(바닥 잔차 0.22, 장애물 높이 0.16). [S]

## 그 밖에 기억할 것
- 설정으로 못 바꾸는 값: `ascent15_no_progress_s`(30 s), `blocked_timeout_s`(20 s), `position_uncertainty_m`(0.15)은 `DEFAULT_SETTINGS`에 없어서 mission에서 바꾸면 ValueError.
- `sand_lane.py:38` `base_height_m=.165`(로버 형상), `run.py:171` 스폰 높이 ground+0.23, `run.py:205` 모래 텍스처 5.6×4.2 m 고정(시각만).
- 확인하지 못한 것: 아레나 USD에 `/World/SandScene/Sand`가 없을 때의 정확한 예외, `drive.py`·`isaac_scene.py`·`pixel_region_observation.py`는 전체를 읽지 않음.
