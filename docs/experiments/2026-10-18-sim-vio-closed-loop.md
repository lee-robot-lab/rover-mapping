# 시뮬레이션 VO+IMU(+엔코더) 추정과 닫힌 루프 주행

2026-10-17~18. 서버 Isaac 임무 시뮬레이션에서 로버가 정답 pose(GT) 대신 **ORB-SLAM3 stereo + IMU(+엔코더) ESKF 추정 위치**로 주행하는 경로를 만들고 평가했다. 이 문서의 결과는 모두 **시뮬레이션**이며 실기 예측이 아니다.

## 먼저 알아둘 한계

- **모래는 고정된 단단한 메시**다(`fixed baked sand mesh; no deformable soil`). 바퀴가 가라앉지 않으며 슬립은 강체 접촉 마찰에서 나온다. 슬립 비율과 감속 임계값은 실기 데이터로 다시 맞춰야 한다.
- 모래 **텍스처는 외관만** 바꾼다(ambientCG Ground080 2K, CC0로 알려져 있으나 에셋 페이지에서 직접 확인하지 못했다).
- 시뮬 **IMU는 GT pose에서 만든 이상적인 값**이다. 실기의 바이어스·진동·SPI 지연 구조는 `imu_degrade`로 일부만 흉내 낸다. 평상시 엔코더의 이득이 안 보이는 이유 중 하나다.
- 필터는 **알려진 시작 위치와 시작 3초 정지(영점)** 를 요구한다.
- 벽 방향·거리·STL 보정은 이 패키지에 없다(`gyusa_static_probe` 의존). 실제 스테레오 깊이 기준 벽 거리 보정은 평상시 이득이 없었고 바퀴와 같이 쓰면 실패한 이력이 있어, 실기 검증 전에는 주행 경로에 넣지 않는다.

## 구성

```text
시뮬(Isaac) ── stereo PNG(15Hz, D435 실측 내부 파라미터) ──► stream_orb(서버 ORB 이미지, 보조 컨테이너)
   │                                                         │ camera pose (파일 프로토콜, lock-step)
   │ 물리 step마다 GT pose ─► 인과적 IMU                      ▼
   └──────────────────────► OnlineEstimator(ESKF, 필터 시각 = 영상 시각 − 0.1 s)
                                      │ 추정 base pose
                                      ▼
                          MissionRuntime(+SensorSafety) ─► 조향·바퀴 목표
GT는 기록과 비교용으로만 남는다(pose_source = orb_eskf_estimate).
```

| 파일 | 역할 |
|---|---|
| `experiments/localization/inertial_filter.py` | ESKF: IMU 공백 처리, 공백 후 중력 정렬, 엔코더 속도·추측 항법, 영상 잡음 배율 |
| `experiments/localization/fusion.py` | 저장 기록을 읽는 오프라인 융합(ORB map 재닻, 기준선 갱신, 슬립 계수) |
| `experiments/localization/online.py` | 시뮬 안에서 한 스텝씩 도는 추정기, 인과적 IMU, 렌더 지연 보상 |
| `experiments/site_sim/stream_orb.cpp` | 서버 ORB-SLAM3 stereo 스트리밍 러너(파일 요청·응답) |
| `experiments/site_sim/orb_client.py` | 시뮬 쪽 요청·응답 클라이언트 |
| `experiments/site_sim/vio_offline.py` | 저장 기록의 ORB 재생·IMU 열화·융합·GT 비교 |
| `experiments/site_sim/rover_assets.py` | 최신 URDF → Isaac용 portable 자산과 FK 기반 swerve geometry |
| `experiments/site_sim/texture_quilt.py` | 사진 모래를 비반복 큰 텍스처로 합성하고 대비 단계를 만든다 |

## 측정으로 알게 된 것

1. **시뮬 영상은 timestamp보다 약 0.10 s 늦은 장면을 보여 준다.** GT 회전속도와 VO 회전속도의 시차 스캔에서 최적이 −0.10 s였다(RMS 4.45 → 2.8 deg/s). 렌더 파이프라인 지연이며 실기의 카메라–IMU 시각차와 같은 종류의 문제다. 오프라인은 camera time에서 0.1 s를 빼고(`--render-lag-s`), 온라인은 필터를 영상 시각에 맞춰 늦춘다(`RENDER_LAG_S`).
2. **정식 ORB-SLAM3는 리셋 뒤 map id를 0부터 다시 시작한다.** id 변화만으로 새 map을 판단하면 리셋된 map의 좌표를 옛 기준과 비교해 보정이 전부 거부된다. 추적 상실 뒤 복귀(상실 → OK)도 새 map으로 본다(`needs_reanchor`).
3. **거부된 상대 보정이 기준선을 그대로 두면 악순환이 생긴다.** 다음 비교가 누적 예측 오차를 안은 더 긴 기준선이 되어 계속 거부된다. 거부돼도 기준을 현재로 옮긴다(`move_baseline`).
4. **IMU 공백**: 공백 구간을 마지막 값으로 보간하면 위치가 폭주했다(0.5초 공백에 수 km). 공백(0.1초 초과)은 적분하지 않고 속도 0·불확실성 확대, 재개 첫 표본에서 중력으로 roll·pitch를 다시 맞춘다.
5. **엔코더는 슬립이 작을 때 IMU 공백의 추측 항법에만 쓴다.** 매 프레임 속도 관측으로 넣으면 30° 경사에서 바퀴가 헛도는 동안(엔코더 0.10 m/s, 실제 거의 정지) 필터가 끌려가 4 cm → 36 cm로 나빠졌다. 슬립 계수(융합 속력/엔코더 속력)가 0.6 아래면 엔코더를 쓰지 않는다.

## 결과 (모두 시뮬, 오프라인 재생)

| 기록 | 조건 | 위치 오차 RMSE / 최대 |
|---|---|---|
| 123 s, 6.6 m | 영상+IMU(50 Hz 평균·실측 잡음) | 4.0 cm / 11.3 cm |
| 123 s | IMU 공백 2회(2 s, 4.8 s) | 11.9 cm / 32.8 cm |
| 123 s | 공백 3회(3 s)가 ORB 상실과 겹침, 엔코더 없음 → 있음 | 65.7 cm → 20.8 cm (마지막 216 → 44 cm) |
| 239 s, 임무 완주 | 위 수정 전 | 149 cm / 498 cm |
| 239 s, 임무 완주 | 재닻·기준선 갱신·0.1 s 지연 보정 후 | 12.4 cm / 29.7 cm |

닫힌 루프(로버가 추정 위치로 주행) 첫 60초 시험: 오차 평균 18.2 cm, 최대 35.5 cm, 30° 등판 실패 후 후퇴 시작까지 GT 주행과 같은 흐름이었다. 전체 임무 결과는 아래 “닫힌 루프 전체 임무”에 기록한다.

## 실행

```bash
# 서버 한 번만: stream_orb 빌드(서버 ORB 이미지 안), 결과 /home/sumee/gyusa/runs/stream_orb
# 자산: 최신 URDF → portable, 질감 합성
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.site_sim.rover_assets --output outputs/site_sim/rover_assets_new
cp <orb.yaml> outputs/site_sim/rover_assets_new/orb_stereo.yaml     # vio_offline orb가 만드는 stereo ORB 설정

# GT 주행 + 영상 저장(후처리·평가용)
bash tools/run_site_sim_server.sh upload L outputs/site_sim/20261017c/mission --texture outputs/site_sim/sand_texture/sand_high.jpg
bash tools/run_site_sim_server.sh start L --gpu 0 --port 8773 --stereo-hz 15 --start-hold-s 3

# 추정 위치로 주행(닫힌 루프): ORB 보조 컨테이너가 같이 뜬다
bash tools/run_site_sim_server.sh start L --gpu 0 --port 8775 --stereo-hz 15 --start-hold-s 3 --estimator
```

저장 기록의 오프라인 평가: `python -m experiments.site_sim.vio_offline fuse STEREO ORB OUT --imu avg50 [--encoder] [--gaps t0 t1]`.
화면에는 GT가 분홍색, 추정이 주황색으로 겹쳐 그려진다.

## 아직 안 된 것

- **상황별 신뢰도 스케줄**의 기본 채택·실기 검증. 아래 후속 진단의 선택적 실험 옵션과 구분한다.
- **당근 추종기·슬립 기반 감속·필터 상태 → 안전 정책**은 Claude Code의 `353ee2a`, `0852c04`로 구현됐다.
  온라인 시뮬 추정기에는 실제 바퀴 슬립 계수가 연결되지 않았고 IMU 공백의 실기 동작도 별도 검증이 필요하다.
- 경사 직후 **재포착**(ORB 재지역화 또는 조건부 벽 보정).
- 실기 IMU 경로(SPI 50 Hz 평균)에 대한 −45 ms 상수는 `inertial_timing.MEASURED_LEGACY_SPI_CORRECTION_S`로 반영했고 raw 전송 경로에는 다시 측정해야 한다.

## Codex 후속 진단 — 2026-10-18

`closed1017c`의 정지 이유 `unsafe_observed_motion`은 `MissionRuntime.update`의
**이전·현재 추정 위치 사이 지도 통행 가능성 검사**에서 나온다. 명령 속도와 관측 속도의
불일치를 검사하는 코드가 아니며, 이 이유를 `SensorSafety`가 직접 계산하는 것도 아니다.
19.8 s의 실제 XY는 (3.670, 0.774), 추정 XY는 (3.826, 0.793) m다.
수평 오차 약 15.7 cm에 더해 z도 약 14.0 cm 낮게 추정했다.

### 확인한 IMU 생성 오류와 수정

1. `CausalImu`가 pose 세 개를 기다린 뒤 가장 최근 구간만 반환해 시작 구간의 회전을 잃었다.
   `OnlineEstimator`는 t=0의 자세로 시작하므로 그 누락이 지속적인 기울기 차이로 남았다.
   해당 기록에서는 약 0.0346°의 roll 차이가 중력을 약 5.9 mm/s²의 수평 가속도로 투영했다.
   시작 pose를 IMU 이력에 넣고 첫 구간의 gyro·비력도 생성하도록 수정했다.
   첫 구간 직전 속도는 명시된 정지 시작 조건에 따라 0으로 둔다.
2. 입력 GT는 base 중심인데 생성한 IMU를 장착된 IMU 중심의 필터에 그대로 넣었다.
   이제 GT base pose에 `T_base_imu`를 적용한 뒤 자이로·비력을 만든다.
   회전 중 장착 위치의 구심·접선 가속도와 센서 축 회전을 함께 반영한다.

기존 기록의 telemetry와 stereo GT를 합친 시각에서 재생했다. 원래 physics step 전체를
저장한 데이터는 아니므로 온라인 실행의 완전한 재현이나 실기 정확도 시험으로 간주하지 않는다.

| 동일 입력 재생 | 수정 전 | 수정 후 |
|---|---:|---:|
| `closed1017c`, 영상 없이 IMU만, 19.8 s 최종 3D 오차 | 1.190 m | 0.000423 m |
| 같은 시험의 최대 3D 오차 | 1.190 m | 0.00270 m |
| `full1017a` 첫 40 s, 저장 ORB도 융합, 3D RMSE | 0.1249 m | 0.1284 m |

IMU 생성 오류는 독립적으로 재현·수정했지만, **영상 융합 후 전체 오차 개선과 닫힌 루프 완주는
이 결과로 확인되지 않는다.** 비교 수치는
`outputs/site_sim/closed_1017c/inspection/startup_replay.json`에 남겼다.
시작 roll 변화와 서로 다른 IMU 장착 위치·방향의 회귀 검사는 수정 전 실패를 확인했다.

### 추가 진단과 신뢰도 스케줄

- 새 telemetry에 `estimated_yaw_rad`, 프레임별 camera pose·filter time·속도·bias를 기록한다.
  `estimator_calibration.json`과 `estimator_frames.jsonl`에도 보정값·시작 pose·모든 ORB 응답을 보존한다.
- `--adaptive-visual-noise`는 **기본 OFF인 시뮬 실험 옵션**이다.
  IMU가 예측한 base 기울기 >8°에서는 영상 위치·회전 표준편차를 각 4배,
  그 외 |base yaw rate| >0.3 rad/s에서는 회전 표준편차만 4배로 둔다.
  이는 보정된 실기 covariance나 모래 임계값이 아니다. 슬립은 이 스케줄에 연결하지 않았다.
- `full1017a` 첫 40 s에서 회전 시 영상 회전 표준편차만 4배로 한 탐색은
  방향 RMSE 1.50° → 0.84°였지만 위치 RMSE 12.84 → 13.31 cm였다.
  따라서 기본 설정으로 채택하지 않았다. 기울기까지 적용한 탐색도 위치 RMSE 13.57 cm였다.
  탐색 결과: `outputs/site_sim/closed_1017c/inspection/noise_schedule_probe.json`.
- `SimulationDrive`는 필터 상태를 `SensorSafety`의 `filter_status` 입력으로 전달한다.
  `recovering` 감속과 `imu_gap` 정지는 가짜 기체 경계 시험으로 검사한다.
- 추정 pose로 구동한 run을 summary/manifest에서 `sim_ground_truth`로 표시하던 메타데이터도 수정했다.
  위치 오차 평가와 지형 방문 기록에는 계속 GT를 사용하며, 제어 pose의 출처와 구분한다.
- Qt 검사는 반드시 `QT_QPA_PLATFORM=offscreen`을 설정한다. 설정 없이 실행하면
  `test_app.py`의 모의 촬영 창이 실제 데스크톱에서 열렸다 닫힌다.

### 서버 60초 진단 `codex_vio1018a`

- IMU 시작/장착 수정과 `353ee2a`의 당근 추종기를 함께 사용했다. 둘의 주행 효과를
  분리하는 A/B 실험은 아니다. 신뢰도 스케줄은 사용하지 않았다.
- 23.25 s 분기점 도착, 28.50 s 30° 등판 시작, 43.76 s 정체 판단 후 후퇴,
  51.06 s 15° 우회 선택까지 진행했다. 59.96 s에 **설정한 60 s 상한**으로 종료했다.
  안전 실패나 전체 임무 완주로 표시하지 않는다.
- 평면 위치 RMSE **2.20 cm**, 최대 **8.52 cm**, 마지막 **6.97 cm**.
  3D RMSE 2.36 cm, yaw RMSE 0.194°/최대 0.708°. ORB 900프레임 중 state 2는 791개,
  state 3은 107개, state 0은 2개였다. 이상적인 시뮬 IMU 조건의 수치다.
- 근거: `outputs/site_sim/codex_vio1018a/{evaluation.json,telemetry.jsonl,estimator_frames.jsonl,events.jsonl,diagnostic.png}`.
  코드별 hash와 실행 명령도 `source_manifest.json`, `launch.json`으로 보존했다.
  이 진단의 summary에는 수정 전 `pose_source=sim_ground_truth` 표기가 남아 있으나
  telemetry는 `orb_eskf_estimate`를 사용했다. 메타데이터 수정은 다음 실행부터 적용한다.
- 첫 서버 시작은 NVIDIA `libnvidia-rtcore` 그래픽 초기화 중 exit 139로 종료됐다.
  실패 자료를 서버 `startup_failure/`에 보존하고, 코드·실행 옵션을 바꾸지 않은
  한 차례 재시작에서 정상 주행했다. 다른 GPU/컨테이너는 중단하지 않았다.
- 전체 자동 검사: **711 passed, 16 skipped**. 변경분의 읽기 전용 검토에서도
  Important 이상 결함은 보고되지 않았다. 전체 임무 시험은 이 진단과 별도로 진행한다.

### 전체 시험 `codex_vio1018b`와 불규칙 IMU 간격

- `c9f5138`과 Claude Code의 두 변경을 합쳐 360 s 상한으로 실행했으나,
  **58.82 s, 15° 우회로로 방향을 바꾸는 중 `unsafe_observed_motion`으로 정지**했다.
  평면 RMSE 1.62 cm/최대·최종 5.68 cm, yaw RMSE 0.166°였다.
  기존 19.8 s 문제는 통과했지만 전체 완주를 확인한 결과는 아니다.
- 이 실행의 실제 IMU 입력은 저장되지 않았다. telemetry와 stereo GT를 합쳐 다시
  미분한 재생은 원래 온라인 위치와 최대 **15.9 cm** 달랐다.
  이 재생에서 noise 배율을 바꾼 결과를 온라인 개선 근거로 사용하지 않는다.
  이전 `full1017a` 탐색 수치도 해당 재구성 입력에서의 결과이며 원래 온라인 실행과의
  동등성을 확인한 것이 아니다.
- 시뮬 `world.step`은 렌더링 여부에 따라 5/50 ms의 입력 간격을 만들 수 있다.
  중앙 가속도를 최신 구간 길이로 적분하던 생성기는 비균일 간격에서 일관되지 않았다.
  독립 회귀 `x=0.0002*(1-cos(40t))`, 5/50 ms 교대 400구간(11 s)에서
  실제 진동 폭은 0.4 mm인데 위치 오차가 **31.23 mm**로 누적됐다.
- 비균일 3점 이차식으로 **구간 끝 속도**를 구하고, 직전 끝 속도와의 차이로
  구간 평균 가속도를 생성하도록 바꿨다. 같은 시험의 최대 오차는 **0.01965 mm**다.
  근거: `outputs/site_sim/codex_vio1018b/irregular_step_probe.json`.
  합성 회귀에서 수정 효과를 확인했으며, 58.82 s 실제 정지까지 해결했다는 뜻은 아니다.
- 다음 실행부터 `estimator_events.jsonl`에 **IMU/ORB/pose 조회의 실제 순서**를 저장한다.
  `OnlineEstimator.on_imu`로 원래 구간 표본을 넣어 재생하므로 GT를 다시 미분하지 않는다.
  GT는 재생 오차 평가에만 쓰며, 평가 GT를 10 m 바꿔도 추정이 바뀌지 않는 회귀를 추가했다.

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.site_sim.vio_offline online RUN NEW_OUT
# 같은 센서 입력으로 선택적 noise 스케줄만 비교
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.site_sim.vio_offline online RUN NEW_OUT --adaptive-visual-noise
```

재생 `summary.json`의 `recorded_estimate_max_diff_m`로 원래 온라인 출력과 일치하는지
먼저 검사한다. 새로운 전체 검사 **714 passed, 16 skipped**. 추가 읽기 전용 검토의
복합 회전·비대칭 장착·불규칙 간격·JSON 재생 시험에서도 원본/재생 차이 0을 확인했다.
