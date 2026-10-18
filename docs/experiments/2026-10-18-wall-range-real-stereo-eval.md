# 벽 거리(R)·직선(L)·모서리(C)·윗선(T) 보정: 시뮬 결과와 실기 스테레오 평가 절차

2026-10-18. 시뮬 결과는 **실기 예측이 아니다.** 다음 세션은 이 문서의 "실기 평가" 절부터 이어간다.

## 1. 시뮬에서 확인한 것 (기록 `claude1018e`, 영상+IMU 기준선 평균 6.4 / 최대 12.1 cm)

| 입력 | 조합 | 평균 오차 | 비고 |
|---|---|---|---|
| 정답 위치에서 만든 합성 깊이 | R, R+L, R+L+C | **1.0 cm** (최대 6.7) | 잡음 ×2에서도 1.4 cm. 코드·필터 연결이 맞다는 검증용이며 센서 성능이 아니다 |
| 시뮬 수동 스테레오(SGBM) | R, R+L, R+L+C | 55~120 cm | 편향 보정(×1.06, ×1.08)으로도 해결 안 됨 |
| 영상의 벽 윗선(허공-벽 경계) | T, 외벽 STL 지도 | 수천 cm 발산 | 경기장 안쪽 경사로 벽이 지도에 없어서 오대응 |
| 같은 윗선, 경기장 전체 메쉬 + 일치 검사 | T | 29.8 cm | 일치 검사 통과 프레임 9%. 벽 윗면 앞·뒤 가장자리 5~10 px 오프셋 |

### 스테레오 깊이가 틀리는 이유 (시뮬)
`python -m experiments.site_sim.stereo_depth_check RUN_DIR` 결과(정답 벽 깊이 대비): 검출률 60~80%, 측정/정답 비율 0.94~0.95(거리와 무관한 약 −5.5% 편향), 비율의 강건 σ 7~12%(합성은 1%+5 mm), 10% 넘게 틀린 픽셀 33~45%. 시뮬 벽이 단색이라 SGBM이 대응점을 못 찾는다. 렌더 지연(0 대 0.1 s)은 영향이 없었다.

### 윗선 검출은 잘 된다
정답 위치 기준으로 윗선 검출률은 100%였고, 0.3~0.8 m 벽에서 중앙값 0 px / σ 3 px였다. 멀수록 +10~15 px 어긋나는 것은 벽 윗면 두께(앞쪽 가장자리를 검출하고 예측은 먼 가장자리)와 지도에 없는 구조물 때문이다. `wall_top_overlay.png` 참고.

## 2. 코드 위치

| 파일 | 역할 |
|---|---|
| `experiments/localization/wall_range.py` | `Scene`(외벽 STL 또는 `mesh=`로 경기장 전체 메쉬, Open3D 가속), `column_scan`, `range_update`(R), `line_update`(L), `corner_update`(C), `detect_wall_top`·`gate_wall_top`·`wall_top_agrees`·`wall_top_update`(T), `add_depth_noise` |
| `experiments/site_sim/wall_replay.py` | 기록 재생. `--variants V,R,R+L,R+L+C,T`, `--noise`, `--synthetic`(정답 깊이 진단), `--mesh`, `--bias` |
| `experiments/site_sim/stereo_depth_check.py` | 스테레오 깊이 대 정답 벽 깊이 통계 |
| `experiments/localization/assets/gyusa_fence40.stl` | 외벽 STL(원본 단위 0.1 m, `STL_UNIT_M`로 변환) |

필터에 넣는 방식은 모두 x, y, yaw만 고치는 평면 갱신(`planar_update`)이며 바퀴 속도는 쓰지 않는다. 이전 실험에서 바퀴 속도와 같이 쓰면 yaw가 19°까지 틀어졌다.

## 3. 실기 평가 (다음 세션)

목표: 시뮬에서 실패한 이유가 "시뮬 스테레오의 한계"인지 확인한다. 실기 D435 IR 스테레오는 벽에 질감이 있으면 다를 수 있다.

필요한 입력
1. **좌우 스테레오 영상과 보정**: `stereo/calibration.json`(`fx, fy, cx, cy, T_left_right, T_base_left_optical`)과 `stereo/mav0/cam{0,1}/data/<stamp_ns>.png`. 형식은 `experiments/site_sim/stereo_capture.py`가 쓰는 EuRoC 형식이다.
2. **추정기 이벤트**: `estimator_calibration.json`, `estimator_events.jsonl`(`imu`, `frame`, `pose` 이벤트). 실기 기록을 이 형식으로 바꾸는 변환이 필요하다. `pose` 이벤트의 `position_m`, `quaternion_wxyz`는 평가용 정답이다.
3. **벽 지도**: 실제 경기장 STL을 **미터 단위로** 맞춘다(원본 단위 확인). 시작 위치·방향으로 좌표계를 정렬한다.
4. **정답 위치**: 실기에는 없으므로 모션캡처나 표시한 경로가 필요하다. 없으면 필터 NIS와 방문 지점 오차 같은 대체 지표로 본다(그 경우 오차 수치는 절대값이 아니다).

순서
1. `stereo_depth_check`로 실기 스테레오 깊이의 검출률·편향·σ를 시뮬 값과 비교한다.
2. 편향이 거리와 무관한 일정한 값이면 `--bias`(비율의 역수)로 보정한다.
3. `wall_replay --variants V,R,R+L,R+L+C,T`로 기준선 대비 오차를 비교한다.
4. 기준선보다 확실히 좋을 때만 주행 경로에 연결을 검토한다. 방침: 실기 검증 전에는 주행 경로에 넣지 않는다.

주의
- 윗선(T)은 벽 정면 거리 한 방향만 준다. 벽과 나란한 방향은 코너·측면 벽이 보일 때 보강된다.
- 지도에 없는 구조물이 보이면 윗선이 오대응한다. `wall_top_agrees`의 일치 검사와 거리 제한(`gate_wall_top`)이 이를 줄이지만 시뮬에서는 부족했다.
- 시뮬 IMU는 정답에서 만든 이상적인 값이라 IMU를 더 믿는 설정이 유리하게 나온다.
