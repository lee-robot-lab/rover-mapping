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

- 경사에서 영상·엔코더 신뢰도를 낮추고, 회전 중 IMU를 더 믿는 **상황별 신뢰도 스케줄**: `relative_update`의 `position_scale`/`rotation_scale` 인자까지만 있고 주행 경로에는 연결하지 않았다.
- **당근 추종기와 슬립 기반 감속**, 필터 상태 → 안전 정책 연결(`SensorSafety`는 추정 품질을 받지만 `imu_gap` 상태 연결은 시뮬 IMU가 끊기지 않아 시험하지 못했다).
- 경사 직후 **재포착**(ORB 재지역화 또는 조건부 벽 보정).
- 실기 IMU 경로(SPI 50 Hz 평균)에 대한 −45 ms 상수는 `inertial_timing.MEASURED_LEGACY_SPI_CORRECTION_S`로 반영했고 raw 전송 경로에는 다시 측정해야 한다.
