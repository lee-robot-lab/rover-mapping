# 전체 임무 완주 설정 탐색 기록 (2026-10-19) — 미완주

목표: 시뮬(RL + IMU 현실화 + 명령 지연 + 시작 노이즈)에서 전체 임무를 접촉 0, 최종 오차 5 cm 이내로 완주하는 설정을 찾고, 완주 실행 1개와 설정을 문서화한다.
**현재 상태: 완주 실행 없음.** 서버(hpi-lab) VPN이 03:53에 끊겨 폐루프 실험이 멈췄다(재연결에 사용자 OTP 필요).

## 폐루프 실행 결과 (공통: RL res8_mlp_v1_rc4000, nominal IMU seed 1, drop 0.008, start noise 2 cm/2°, 지연 17.6~46.5 ms, wheel-effort 1.96)

| 런 | 변수 | 종료 s | 사유 | XY 오차 평균/최대/마지막 cm |
|---|---|---:|---|---|
| rt1018a | 기준(수정 전, 회전 영상 σ ×4) | 20.1 | unsafe_observed_motion | – / 12.5 / – |
| gx_as | IMU stress, ×4 | 9.8 | unsafe_local_segment | – / 14.8 / – |
| gx_a | nominal, ×4 | 11.0 | unsafe_local_segment | – / 24.0 / – |
| cx1019_t4a | ×4 (Codex) | 23.4 | unsafe_observed_motion | 7.24 / 46.86 / 17.36 |
| cx1019_t1a | ×1 (Codex) | 9.2 | unsafe_local_segment | 2.27 / 6.25 / 4.61 |
| cx1019_corner370 | ×1 + 직각 connector | 25.9 | unsafe_local_segment | 5.26 / 34.72 / 34.72 |
| cx1019_corner375 | ×1 + 직각 connector | 38.8 | unsafe_observed_motion | 8.34 / 22.84 / 2.85 |

## 확인된 것

1. 첫 코너 추정 오차의 직접 원인은 회전 중 영상 위치 σ ×4(`turn_position_scale`). 오프라인 재생에서 ×1이면 6~11 s 최대 오차 21.5→6.6 cm (gx_a). 폐루프 t1a도 최대 6.3 cm. 단 ×4는 실기 ORB 끊김 관측에서 온 시뮬 휴리스틱이라 실기 σ를 시뮬 결과만으로 낮추지 않는다.
2. ×1에서도 t1a는 조향 목표선이 코너를 가로질러 실패 → 직각 connector가 필요했다. connector 경로의 최대 여유는 시작 셀의 마스크 거리(0.375 m)가 상한이라 경로 offset만으로는 약 4 cm밖에 못 번다.
3. connector 이후 실패(corner370 25.9 s, corner375 38.8 s)는 코너·제자리 회전(실제 30~45 dps) 중 ORB RECENTLY_LOST → 시뮬 relocalization 대기 3 s(실기 설정에는 15 s) → 필터 속도 과신(σ 2.5 cm/s, 실제 10~17 cm/s) → yaw/위치 누적. GT를 ORB로 줘도 yaw 드리프트가 남아 필터 구조·IMU 모델 문제.
4. 실제 접촉 위험은 낮았다(footprint 간격 약 9 cm, ±3 cm). 실패는 안전 마스크 문턱(0.2854 m)에 0.4 mm 수준으로 걸린 것이 트리거인 경우가 있다.
5. IMU 가속도 잡음 모델 불일치: 필터 NoiseAcc 0.002 대 로그 오차 이동 중 15~30 cm/s². 개루프 `--filter-acc-noise` 0.03/0.1은 효과가 크지만(최대 22.8→11.2 cm) Codex는 고정 covariance 상향이 일관되지 않다고 보고. 폐루프 미검증.
6. 방향 관측 σ(0.01/0.02/0.04): 폐루프 직접 비교는 이른 실패로 불가, 개루프에서 일관된 개선 없음 → production 미채택.
7. RLC(맵 단서) 판단: **보류**. 846 holdout 회전 7쌍 p90 21.6°, 램프 경계선은 수평선 전제에서 체계 오차(경사 0/10/15°에서 추정 heading 0/12.4/18.5°), 실제 차선폭 미측정. 추정기 미연결.

## 구현 (로컬 커밋, push 없음)

- `0f98252` run.py `imu_realism` 대입 누락(NameError) 수정 + `build_estimator` 분리 회귀 테스트.
- `109ab29..e5ddfa4` blend 추종기: yaw+vy 혼합, 램프 게이트, 방향 오차 90° 초과 시 정지·재정렬(resume 45°), 옵션 `--follower blend --blend-max-yaw-rate`, `follower_metrics`. 기본 모드는 골든 해시로 비트 동일. 비물리 replay 8건 완주(실제 시뮬 검증은 서버 필요).
- 이 문서 작성 시점 테스트: handheld 7개 파일 135 passed, tests/site_sim+localization 202 passed, pyflakes 깨끗.

## 서버 복구 후 큐 (한 번에 한 변수, 감시 스크립트가 자동 시작)

1. `cy_y45a`/`cy_y45b`: 번들 `max_yaw_rate_rad_s` 0.6→0.45 (각각 corner370/corner375 번들). 기준 25.9 s / 38.8 s와 비교.
2. `cz_b370`/`cz_b375`: corner370/375 번들 그대로 `--follower blend`만 추가, 최대 200 s.
3. 이후 후보(각각 별도 실험): `--filter-acc-noise` 0.03/0.1, 시뮬 `StereoRelocalizationWaitSeconds` 15(이 키는 복구 대기 패치가 들어간 ORB 코어에서만 읽힌다. 시뮬 이미지 `rover-orbslam3-jazzy:4452a3c4`에 패치가 있는지 서버에서 먼저 확인해야 하며, 없으면 무시되어 실험이 무의미하다), `unsafe_observed_motion` 2연속 위반 요구(안전 의미 변경이므로 GT 마진 동시 평가).

완주(접촉 0, 최종 오차 ≤5 cm) 실행이 나오면 이 문서에 런 라벨, `git rev-parse HEAD`, 전체 옵션, GT 최소 마스크 거리, 최종 XY 오차를 추가한다.

## 서버 복구 후 결과 (11:07~11:25)

| 런 | 변수(기준 대비 한 개) | 종료 s | 사유 | XY 오차 평균/최대/마지막 cm | ORB 비정상 프레임 |
|---|---|---:|---|---|---:|
| cy_y45a | 회전 상한 0.45 (corner370 번들) | 15.8 | unsafe_local_segment | 4.0 / 38.7 / 38.7 | 45 (12.82 s, 회전 9~14 dps) |
| cy_y45b | 회전 상한 0.45 (corner375 번들) | 45.0 | unsafe_observed_motion | 11.0 / 40.5 / 40.5 | 0 |
| cz_b370 | `--follower blend` (corner370 번들) | 63.3 | unsafe_observed_motion (retreating) | 22.2 / 57.7 / 48.5 | 0 |
| cz_b375 | `--follower blend` (corner375 번들) | 50.1 | unsafe_observed_motion (retreating) | 8.2 / 21.6 / 19.5 | 17 |

판정:
- 회전 상한 0.45: 일관된 개선 없음 → 되돌림(0.6 유지). 명령 상한을 낮춰도 실제 GT 회전은 41~55 dps까지 올라간다.
  - 정정(병렬 분석 결과): cy_y45a의 12.82 s ORB 손실은 이 변수의 효과가 아니다. 손실 전 최대 요청 yaw 는 0.128 rad/s(<0.45)로 상한이 걸린 적이 없고, 같은 구간(x≈3.58, y≈1.21)을 지난 4개 런 중 이 런만 끊겼다(확률적 사건, 내부 inlier 수는 기록이 없어 미확인). 같은 시드·같은 명령에서도 ORB pose가 4 s부터 mm~cm 단위로 벌어진다. 따라서 **지금까지의 모든 비교가 단일 표본이며 ORB 비결정성이 섞여 있다**. 같은 설정 N≥3회 반복으로 손실 확률을 먼저 구해야 변수 효과를 판정할 수 있다. 손실 후에는 필터가 순수 IMU 적분으로 흘러(속도 오차 −0.15~−0.2 m/s, tilt 이중적분이 지배, 3 s에 38 cm) 좁은 안전 통로(x=3.3~3.4에서 폭 0.22 m)의 마스크 밖 셀을 carrot 선분이 지나 정지했다.
- blend: 기준(corner370 25.9 s, corner375 38.8 s)보다 오래 갔고 두 번호 모두 trial_entry·ascending(30° 램프 등판)을 거쳐 등판 정체로 retreating 까지 갔다. corner370의 ORB 손실은 29→0 프레임. 남은 실패는 램프 구간(등판 정체와 후퇴 중 위치 오차). 채택, 다음 변수는 램프 구간 분석 결과에 따른다.
- 이 표의 모든 실행은 `--follower` 외 옵션 동일(×1, nominal IMU 시드 1, 지연 17.6~46.5 ms, start 2 cm/2°). 단일 표본이라 ORB 비동기 편차가 섞여 있다.

## 다음 설계 원칙 (병렬 분석 반영)

- 변수 판정은 같은 설정 N≥3회 반복(같은 시드여도 ORB 비결정)의 손실 확률·종료 시각 분포로 한다. 단일 표본으로 채택/되돌림하지 않는다.
- 후보: (a) 무변경 반복으로 baseline 분포 측정, (b) ORB 추적 map point·inlier 수를 프레임마다 기록(동작 불변), (c) 시각 보정 미수용이 0.3 s를 넘으면 yaw_rate 0 또는 `good_s` 단축, (d) `minThFAST` 3→7(반복 필요), (e) `carrot_min_speed_factor` 0.35→0.6.
