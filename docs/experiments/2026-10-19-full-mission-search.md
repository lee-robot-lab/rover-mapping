# 전체 임무 완주 설정 탐색 기록 (2026-10-19) — 미완주

> **⚠ 정정 안내 (2026-10-19 독립 감사 후):** 이 문서의 해석·원인 귀속·"채택/되돌림" 판정 중 상당수가 감사에서 과장 또는 오류로 확인되었다. 반드시 `2026-10-19-audit-corrections.md`를 먼저 읽고, 충돌하면 그 문서를 따른다. 정량 수치 자체는 대부분 원자료와 일치했다.


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

## 반복 실험: unicycle 기준 vs blend (corner370 번들, 동일 옵션, N=3씩)

| 런 | 종료 s | 사유 | XY 오차 평균/최대/마지막 cm | ORB 손실 시작 s | 상태 이력 |
|---|---:|---|---|---|---|
| cx1019_corner370 (unicycle) | 25.9 | unsafe_local_segment | 5.3 / 34.7 / 34.7 | 24.0 | to_junction→trial_entry→ascending |
| cu_r2 (unicycle) | 26.4 | unsafe_observed_motion | 3.5 / 28.7 / 28.7 | 24.3 | to_junction→trial_entry |
| cu_r3 (unicycle) | 26.4 | unsafe_observed_motion | 6.0 / 37.2 / 37.2 | 24.2 | to_junction→trial_entry |
| cz_b370 (blend) | 63.3 | unsafe_observed_motion | 22.2 / 57.7 / 48.5 | 없음 | to_junction→trial_entry→ascending→retreating |
| cz_r2 (blend) | 34.5 | unsafe_observed_motion | 12.3 / 28.8 / 5.9 | 32.6 | to_junction |
| cz_r3 (blend) | 36.2 | no_progress_to_junction | 10.5 / 27.0 / 25.8 | 없음 | to_junction→failed |

- unicycle 기준은 3회 모두 25.9~26.4 s에 `trial_entry` 제자리 회전 중 ORB 손실(24.0~24.3 s)로 같은 방식으로 실패한다. 기준 실패는 결정적이다.
- blend 3회는 모두 기준보다 오래 갔다(34.5/36.2/63.3 s). 24 s대의 결정적 ORB 손실은 사라졌지만 새 실패가 생겼다(connector 구간 no_progress/ORB 손실, 램프 정체 후 후퇴 중 위치 오차). blend 변동이 크므로 이후 판정은 blend 쪽도 N≥3으로 한다.
- 추정기 구조 분석(cy_y45b 반사실): ORB 관측을 GT로 바꿔도 오차가 남고, 병진 residual 이 yaw·gyro bias z 로 새며(gb_z 참값 고정 시 최대 40.5→11.4 cm), 필터 가정 잡음이 시뮬 IMU 잡음보다 gyro 18~36배·accel 8~23배 작다. 변형 후보는 오프라인 다중 런 평가 중.

## 인계 (2026-10-19 12:00, 사용자 외출)

- 서버: 내 소유 실행 없음(GPU 반납). 동료(Codex)의 `cx1019_origin_r1/r2`가 돌고 있을 수 있다. VPN 키프알라이브 루프(PID 158258)는 계속 동작 중.
- 백그라운드 분석 에이전트 4개가 재개되어 진행 중: 오프라인 필터 변형 평가(V1~V5), blend connector 실패(r2·r3), cz_b370/cz_b375 램프 정체·후퇴 오차. 결과는 세션이 열리면 도착한다.
- 다음 한 변수 후보(오프라인 평가에서 일관된 개선이 나온 것 1개): gyro bias z 사전 σ 축소 / 병진 residual의 yaw·gb 경로 차단 / gyro 잡음 상향 / 상대 위치 σ 0.5 cm. 구현은 기본 동작을 바꾸지 않는 옵션으로, 서버에서 N≥3 반복으로 unicycle·blend 각각 확인.
- 완주 실행(접촉 0, 최종 오차 ≤5 cm)은 아직 없음. 완주가 나오면 이 문서에 라벨·`git rev-parse HEAD`·전체 옵션·GT 최소 마스크 거리·최종 오차를 추가한다.

## cz_b370(blend) 분석 결과 (읽기 전용 재생, 단일 런)

- 로버는 30° 램프에 올라간 적이 없다(GT x 최대 4.087, 램프 발치 x≈4.10 미달). 36~48 s에 GT가 북쪽으로 새서 30° 차선 밖(15° 입구)으로 갔고, 추정은 30° 차선 발치에 있다고 믿었다. 등판 정체는 슬립이 아니라 칸막이 끝에 걸린 것.
- 원인은 ESKF yaw 오염: corner B(20~30 s, 저속 0.03 m/s)에서 GT는 +39.6° 돌았는데 추정 yaw는 +2.3°만 변했고, 영상 위치 잔차가 yaw로 새어 누적 −38° 보정. 필터 σ_yaw 4.5° 대비 실제 40°↑(약 9σ), NIS 게이트 거부 0건. ORB 자체는 정상(회전 오차 평균 3.3°). 오염을 시작시킨 계기는 미확인.
- blend의 개선: ORB 손실 0, GT 최대 yaw rate 51→21 dps. 새 문제: 저속 구간 yaw 오염, connector 27.5 s(기준 20.6 s), 추정 기반 램프 게이트의 허위 발동(GT 경사 0~1.5°), stuck 판정이 추정 전진으로 리셋되어 약 12 s 지연, 후퇴 경로가 지그재그라 1.8 cm 이동에도 unsafe_observed_motion, 실제 접촉 위험(FR 바퀴–칸막이 끝 간격 약 1 cm).
- 개루프 재생 후보(pos 평균/최대 cm): 기준 22.2/57.7, C2(ORB 절대 yaw를 σ 5~10° 약관측으로 추가) 14.5/30.2, V3(영상 위치 잔차→yaw·gb_z 이득 차단) 17.7/34.9, C6(저속 영상 위치 σ ×3) 20.9/50.6. C2는 "절대 SLAM pose 반복 사용 금지" 원칙과의 충돌 여부 확인 필요. 회전 이득 전체 차단·accel bias 이득 차단은 발산하므로 제외.
- 추가 한 변수 후보: `blend_heading_hold=1`(저속 연속 회전이 오염을 촉발하는지), 후퇴 breadcrumb 간격 증가.

## 정정 (번들 확인, 12:10)

- **`cz_b375`는 corner375 번들이 아니라 corner370 번들로 돌았다**(mission.json sha256이 cz_b370·cx1019_corner370과 동일, connector [2.9,1.2]→[3.7,1.2]→[3.7,0.35]). 원인은 내 서버 스크립트: 번들 인자를 받는 `up` 수정이 `cy_waves.sh`에만 들어가고 `cz_waves*.sh`에는 반영되지 않았다. `cy_y45a/b`는 각자 번들(corner370/375)이 맞다(해시 확인).
- 따라서 위 문서의 "cz_b375 (corner375 번들)"와 "corner375 기준 대비 blend 개선" 서술은 무효다. `cz_b375`는 corner370 blend의 반복 표본이고, corner370 blend는 N=4(63.3 / 50.1 / 34.5 / 36.2 s)다. corner375 번들의 blend 실험은 아직 없다.
- cz_b375 분석 요지(유효, 단 corner370 번들 기준으로 읽을 것): 등판 정체 구간 슬립 약 93%(조인트 환산 vs GT, 램프 발치에서 앞바퀴만 살짝 올랐다 밀림), ESKF 전파 yaw는 GT와 2° 내인데 상대 영상 갱신이 yaw를 −14°·−8.5° 밀어 틀림(위치 잔차와 dyaw 상관 −0.78~−0.86), 램프 게이트는 추정 몸 중심 경사만 읽어 0회 발동, 후퇴 중 ORB 손실 1.07 s 연속 1건, 속도 상태가 정지 중에도 0.1~0.17 m/s(ZUPT 없음), 후퇴 종료는 추정 6 mm 이동이 트리거. 재생 후보: ZUPT, 위치 잔차→yaw gain 차단, 엔코더 속도 크기 관측, 램프 게이트를 앞바퀴 위치에서 평가, 진입 전 정렬 조건, `--mu` 보정.
- 두 blend 분석(cz_b370, cz_b375)이 같은 결론에 수렴한다: 남은 지배 원인은 ESKF의 yaw·속도 상태 오염이다.

## 오프라인 필터 변형 평가 (16개 런 개루프 재생, baseline 재생은 전 런 6e-8 m 일치)

| 변형 | XY 최대 중앙/최악 cm | 개선/악화 런 | yaw 최대 중앙/최악 ° |
|---|---|---|---|
| V0 baseline | 27.9 / 94.7 | – | 4.5 / 47.8 |
| V1 정지 평균으로 gyro bias 초기화 + gb_z σ 0.03 dps | 22.4 / 90.9 | 7/3 | 3.2 / 7.3 |
| V2 병진 residual의 yaw·gyro bias 이득 차단(단독) | 31.2 / 256 | 4/12 | 7.3 / 16.5 |
| V3 NoiseGyro ×20 | 15.6 / 37.4 | 13/0 | 6.0 / 13.8 |
| V4 상대 위치 σ 0.5 cm | 28.9 / 42025 | 7/8 | 3.3 / 43.4 (발산) |
| V5 V1+V2 | 14.7 / 40.9 | 12/3 | 2.8 / 4.6 |
| V6 V1+V3 | 14.9 / 46.3 | 12/2 | 2.9 / 4.6 |

- 가장 일관된 XY 개선은 V3(악화 0개, 최악 94.7→37.4 cm)이고, yaw는 V1이 15/16 개선(최악 47.8→7.3°). 평균 0인 채 σ만 줄이는 V1a와 V4는 과신으로 악화·발산하므로 금지. V2는 V1과 함께일 때만 유용.
- 한계: 개루프, 안전 정지 판정 미재현, 16런은 같은 시드·realism이라 상관 있음. cy_y45a·cx1019_corner370·rt1018b처럼 yaw 오차가 작은 런의 XY 오차(35~95 cm)는 어느 변형으로도 해결되지 않는다(원인 미확인).
- 추천 순서: V3 단독 폐루프 확인 → V6(V1+V3). 구현안: `--filter-gyro-noise-scale`(run.py, server.py의 `filter_acc_noise` 패턴, 기본 None=동작 불변, estimator_calibration.json에 사용값 기록, 전달 테스트). 서버 확인은 같은 시드로 baseline과 쌍으로, N≥3.

## blend connector 실패 분석 (cz_b370 / cz_r2 / cz_r3, corner370 번들, 읽기 전용 재생)

- 세 런은 코너(약 12~14 s)까지 사실상 같았고 하향 직선에서 갈렸다. connector 구간에서 재정렬 정지·램프 게이트는 한 번도 발동하지 않았다 → blend의 정지·재정렬 로직은 이 실패와 무관.
- cz_r2·cz_r3: GT yaw가 명령(−5.2 dps)을 따라가지 않았고 GT 속도도 명령의 절반 수준. 코너 직후 몸 기준 병진 방향 θ가 −75°~−115°로 벌어져 vx가 0 또는 음수가 됐고(순수 옆걸음), 26~28 s부터 벽 쪽(3.52~3.55, 0.62~0.65)에서 사실상 정지. cz_b370은 같은 상한에서 yaw가 명령을 따라 통과(θ −69°→−52°). |θ|≳70~75°에서 swerve 속도·yaw 실현이 모두 떨어진다는 가설은 3표본 상관뿐이며 미확인(조향 조인트 규약 미확인, 실기 vy 옆걸음은 정상이라 시뮬 한정일 수 있음).
- 추정 yaw가 −5~0°로 평평한데 GT yaw는 −35°까지 흘러(추정–GT yaw 오차 14 s +3.8° → 34 s +28°) 컨트롤러가 벽 쪽으로 틀어진 방향으로 명령했고, GT는 경로에서 0.17~0.18 m 이탈(추정은 경로 근처로 인식). 이 yaw 오차의 원인은 미확인.
- cz_r3 `no_progress_to_junction`: 재생으로 36.155 s에 effort 8.0 도달을 재현(실제 36.175 s). 진행은 `arcs[index]`(0.05 m 점 단위)로 재서 `min_progress_m` 0.025가 사실상 의미 없음. GT도 실제로 정지(30~36 s에 3.4 cm)라 판정은 오탐이 아님. cz_r2는 추정이 느리게 떠돌아 effort가 리셋되어 stuck을 피했을 뿐 → r2/r3 종료 사유 차이는 추정 드리프트 몇 cm에서 갈림.
- cz_r2: 32.62 s에 정지 상태(GT 0.003~0.01 m/s)에서 ORB 손실(특징점 1,776~2,007개로 충분, 원인 미확인, 회전 때문이 아님). 34.49 s unsafe_observed_motion은 추정이 벽 inflation 안(벽까지 0.27~0.286 m < 0.30)으로 4 mm 이동해 발동. GT–벽 최소 거리 r2 0.212 m, r3 0.203 m로 외접원 0.195 m 대비 여유 약 1~2 cm(접촉 센서 기록 없음, 실제 접촉 미확인). `run.py`는 `unsafe_observed_motion`이 sim_s≥5 이후 한 번이라도 나오면 즉시 종료하는 하네스 정책.
- 후보(한 번에 한 변수, 미실행): stuck 진행 측정을 경로 투영 거리로 연속화, 코너 반경 완화 또는 `carrot_brake_m`·`carrot_min_speed_factor` 조정, 코너 구간 θ 상한 게이트, 경로 x를 벽에서 0.08 m 이격, 같은 설정 3회 반복 시 14 s 시점 yaw·θ와 조향 조인트·RL action 로그를 저장해 θ 가설 검증.

## 폐루프: 필터 gyro 잡음 ×20 (`--filter-gyro-noise-scale 20`, blend, corner370 번들, 소스 83ca595)

| 런 | 종료 s | 사유 | XY 오차 평균/최대/마지막 cm | ORB 손실 시작 s | 상태 이력 |
|---|---:|---|---|---|---|
| cg_r1 | 52.2 | unsafe_observed_motion | 14.5 / 24.2 / 15.8 | 51.0 | to_junction→trial_entry→ascending |
| cg_r2 | 40.8 | unsafe_observed_motion | 9.3 / 25.8 / 25.8 | 37.4 | to_junction→trial_entry→ascending |
| (기준) blend 4회 | 63.3 / 50.1 / 34.5 / 36.2 | – | 최대 오차 57.7 / 21.6 / 28.8 / 27.0 | 없음/49.0/32.6/없음 | 2/4회만 ascending 도달 |

- 판정: 개선도 악화도 단정할 수 없다(종료 시각 40.8~52.2 s는 기준 분포 34.5~63.3 s 안, 최대 XY 오차 24~26 cm는 기준 중앙 범위). 두 번 모두 ascending까지 갔고(기준은 4회 중 2회) connector 구간 실패는 없었지만 N=2라 판단 보류. 되돌리지 않고 후보로 유지.
- 두 런 모두 끝 무렵(37~51 s)에 ORB 손실이 있었고 그 직후 종료. 오프라인 평가에서 기대한 XY 최대 오차 감소(최악 94.7→37.4 cm)가 폐루프에서 뚜렷하게 재현되지는 않았다.

## 폐루프: gyro bias 정지 평균 초기화 (`--filter-gyro-bias-init-sigma-dps 0.03`, blend, corner370, 소스 b119765)

| 런 | 종료 s | 사유 | XY 오차 평균/최대/마지막 cm | ORB 손실 | 상태 이력 |
|---|---:|---|---|---|---|
| cb_r1 | 78.0 | unsafe_observed_motion | 17.6 / 48.0 / 35.2 | 없음 | to_junction→trial_entry→ascending(30)→retreating→ascending(15) |
| cb_r2 | 24.2 | no_progress_to_junction | 4.7 / 9.0 / 5.8 | 없음 | to_junction→failed |

- cb_r1은 지금까지 폐루프 중 가장 오래 갔고(78.0 s) 처음으로 30° 후퇴 뒤 15° 차선 등판까지 진행했다. cb_r2는 추정 오차가 작은데(최대 9 cm) connector에서 no_progress로 24.2 s에 끝났다(GT 정체 여부 미확인 — cz_r3와 같은 종류의 정체일 가능성).
- N=2, 결과 분산이 매우 크다(24.2 vs 78.0 s). 기준 blend 분포(34.5~63.3 s)와 비교해 효과 판단 불가. 판정 보류, 반복 필요. 로그의 gyro bias z 추정은 두 런 모두 종료 시 0.82~1.0 dps(참값 0.486)로 초기화 이후에도 다시 흘렀다(필터가 계속 갱신).
