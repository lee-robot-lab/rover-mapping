# 서버 시뮬에 실기 차이와 RL 밑단 넣기: 옵션과 실행 순서

2026-10-18. 서버 `site_sim`(Isaac) 시뮬을 실기에 더 가깝게 만드는 옵션과 KSRC Res8 정책(yunbeen)을 swerve 목표 아래에 얹는 옵션을 설명한다. **모든 옵션은 기본 꺼짐**이라 켜지 않으면 기존 실행과 같다.
**상태:** 로컬 테스트(site_sim·localization 152개)만 통과했다. Isaac 안에서의 실행(RL 관측용 속도 API, 시작 자세 틀기, 바퀴 토크, 마찰 바인딩)은 아직 한 번도 돌리지 못했다. 첫 실행은 반드시 스모크(아래 2단계)로 한다.

## 옵션과 실기 근거

| 옵션 | 내용 | 값의 근거 |
|---|---|---|
| `--control-dt 0.02` | 제어·IMU 구간 주기 [s]. 기본 0.05. RL은 0.02 필요(정책이 50 Hz로 학습됨) | 실기 `run_policy` 루프 20 ms (로그 실측) |
| `--imu-realism` | 추정기 IMU에 자이로 바이어스(−0.63, 0.485, 0.488 dps), 정지 백색 잡음, 이동 중 진동을 더함 | 10-18 12:04~12:08 student v3 로그(모래 15° 램프) |
| `--imu-drop-prob 0.008` | IMU 패킷 한 개 누락 확률(누락은 다음 패킷에 합쳐져 40 ms 구간) | 실기 handheld 로그: 40 ms 이상 간격 0.8% |
| `--imu-seed N` `--imu-vibration-scale X` | 난수 시드, 이동 진동 배율 | |
| `--command-delay-ms 17.6 46.5` `--delay-seed N` | 명령이 조인트에 닿기까지 균등 난수 지연 | Pi bringup 실측 17.6~46.5 ms |
| `--start-noise-xy-m 0.02 --start-noise-yaw-deg 2 --start-seed N` | 로버만 틀어 놓고 추정기는 이상적 시작 자세를 믿음 | **임의값**(줄자로 놓은 오차를 흉내) |
| `--wheel-effort 1.96` | 바퀴 최대 토크 [N m]. 기본 1.0 | 문서의 모터 정지 토크 약 1.96 N m (미검증) |
| `--mu M` | 지면 mesh와 바퀴 충돌체의 정지·동마찰 | 모래 마찰은 측정한 적 없음 → 임무 결과로 보정 |
| `--rl-policy res8_mlp_v1_rc4000` | swerve 목표 아래에 Res8 잔차를 얹음. `--control-dt 0.02`, `--start-hold-s 2` 이상, 업로드 때 `--ksrc` 필요 | Pi `run_policy` 루프를 같은 `student_runtime`으로 재현 |

RL 정책은 torch 없이 numpy로 추론한다(`<policy>.npz`, torch 출력과 최대 차이 3.8e-6). RL 관측용 gyro·비력에도 `--imu-realism`이 켜져 있으면 같은 현실 IMU 특성을 더하고, 시작 정지 2초 동안 자이로 평균을 구해 뺀다(Pi `--bias-s`와 같은 방식).

## 준비 (로컬, 한 번)

```bash
cd rover-mapping   # 이 저장소 체크아웃
# ksrc 체크아웃(YunBeeeen/ksrc main@0716134)에서 정책·펌웨어 C 소스·실행기만 모은다. outputs/는 git에 없다.
cp -r <ksrc 체크아웃> outputs/site_sim/ksrc_20261016
# 정책을 numpy로 꺼낸다(torch 필요: isaacsim conda 환경)
PYTHONNOUSERSITE=1 PYTHONPATH=. <torch가 있는 python> -m experiments.site_sim.export_policy_npz \
    outputs/site_sim/ksrc_20261016/pi/policy/res8_mlp_v1_rc4000
```
VPN을 연결해 서버(`ssh hpi-lab`)에 접속되는지 먼저 확인한다.

## 실행 순서

1. **업로드** (임무 `20261017c`는 repeat1018a와 site_identity가 같다)
```bash
PYTHONNOUSERSITE=1 bash tools/run_site_sim_server.sh upload LABEL outputs/site_sim/20261017c/mission \
  --texture outputs/site_sim/sand_texture/sand_high.jpg --ksrc outputs/site_sim/ksrc_20261016
```
업로드가 서버 호스트의 gcc로 `ksrc/sim/libksrc_common.so`를 빌드한다(Isaac 컨테이너에는 gcc가 없다).

2. **스모크**: sim 40초만. 예외 없이 도는지, GPU 사용률을 본다.
```bash
PYTHONNOUSERSITE=1 bash tools/run_site_sim_server.sh start LABEL --gpu 1 --port 8801 --max-sim-s 40 --max-wall-s 1500 \
  --stereo-hz 15 --start-hold-s 3 --estimator --adaptive-visual-noise --control-dt 0.02 \
  --rl-policy res8_mlp_v1_rc4000 --imu-realism --imu-drop-prob 0.008 --imu-seed 1 \
  --command-delay-ms 17.6 46.5 --delay-seed 1 --start-noise-xy-m 0.02 --start-noise-yaw-deg 2 --start-seed 1 --wheel-effort 1.96
```
`run.log`에서 `SITE_SIM_RL residual policy loaded`와 (`--mu` 사용 시) `SITE_SIM_FRICTION ... ground meshes N wheel colliders M`을 확인한다. `output/start_perturbation.json`, `output/rl_residual.json`이 생기면 옵션이 적용된 것이다.

3. **μ 보정**: 같은 구성에서 `--mu`만 바꿔 몇 번 돌려 **30° 실패, 15°·10° 성공**이 되는 값을 찾는다. 실기 15°에서 실제 속도는 바퀴 속도의 약 40~60%였다(10-18 로그). 모래 램프 한 점(A 램프 약 80 cm)에서는 강체 여의도 STL 기준 μ≈0.2~0.25였다.

4. **본 비교**: 이상적 기준선(옵션 없음, `--control-dt 0.02`), 실제 같은 IK(위 옵션에서 `--rl-policy` 제외), 실제 같은 IK+RL을 시드 1~3으로 돌린다.

5. **평가**: 위치 오차가 아니라 완주와 벽 거리를 본다.
```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.site_sim.clearance RUN/output outputs/site_sim/20261017c/mission
```
`completed`, `min_clearance_m`, `contact_samples`(GT 위치가 외접원 0.195 m 안으로 벽·단차에 접근한 표본 수), `stuck_events`, `reason`이 나온다. 결과는 `tools/run_site_sim_server.sh fetch LABEL` 로 받는다.

## 녹화 재생 (실험이 끝난 뒤 웹에서 다시 보기)

```bash
# 서버 기록을 받는다: 임무 폴더(mission_bundle, 벽 거리용)와 설정(run_options.json)도 같이 온다. stereo는 3장마다 1쌍.
PYTHONNOUSERSITE=1 bash tools/run_site_sim_server.sh fetch LABEL outputs/site_sim/LABEL
# 재생 서버를 연다(노트북, .venv-mapping). mission_bundle이 있으면 벽 거리를 자동으로 쓴다.
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.site_sim.viewer --output outputs/site_sim/LABEL --port 8790
```
브라우저에서 `http://127.0.0.1:8790`을 열고 **▶ 재생** 막대로 보면 된다(속도 조절, 슬라이더). 화면 위쪽 줄에 이 실행의 설정(RL, 실제 IMU, 지연, μ, 제어 주기 등 켜 둔 것만)이 나오고, 재생 막대에는 시각, 추정 오차, **벽까지 거리**(정답 위치 기준)가 나온다. 추적 시점(관찰 화면)과 로버 전방 영상(15 Hz)도 같이 재생된다. 서버에서 직접 보려면 `tunnel LABEL 8790`을 쓴다(서버 뷰어는 벽 거리를 표시하지 않는다).
옛 기록은 `run_options.json`이 없어 설정 줄이 비고, `--mission outputs/site_sim/20261017c/mission`을 직접 주면 벽 거리를 볼 수 있다. repeat1018a는 이렇게 재생해서 벽 최소 거리 30.9 cm, 접촉 표본 0을 확인했다.

## RL 경로의 규약 변환 (2026-10-18 디버깅으로 알게 된 것)

KSRC 정책과 펌웨어 IK는 **펌웨어 규약**(조향각 θ는 바퀴가 향하는 방향, 속도 부호는 그 방향 기준)으로 말하고, 시뮬 조인트는 URDF 규약(`steering_sign` −1, 왼쪽 바퀴는 `rolling_heading` π, 조향 조인트 이름이 앞·뒤 오른쪽에서 서로 바뀜)으로 움직인다. 처음에는 펌웨어 목표를 조인트에 그대로 줘서 회전 명령 −0.6 rad/s가 후진(vx −0.079, ω 0)으로, 직진 0.08 m/s가 회전(ω 0.378)으로 나왔고, RL 잔차가 한계에 포화했다. `experiments/site_sim/joint_convention.py`의 `to_joint`(목표)와 `from_joint`(관측)가 같은 물리 속도 벡터로 바꿔 준다. 회귀 테스트는 `tests/site_sim/test_joint_convention.py`.
RL을 켠 실행은 텔레메트리에 `rl_obs`(정책에 준 관측 29개), `rl_action`(정책 출력 8개), `rl_d_steer_rad`, `rl_wheel_gain`을 남긴다. 수정 후 14초 스모크에서 RL만 켠 경우와 RL+현실 옵션 전부 켠 경우 모두 회전 방향이 맞고 경로를 따라갔다(이동 64 cm, 52 cm). RL `|action|` 평균은 0.34~0.45, 최대 1.7~2.0으로 실기 로그(평균 0.2, 최대 1.3)보다 아직 조금 크다.

### 부호만 KSRC 규약으로 바꾼 자산 (`--ksrc-signs`)

ksrc URDF(`rover_rev5_isaac.urdf`, `urdf/rover.urdf`)는 조향축 +z, 바퀴축 +y, 이름과 위치가 일치해서 펌웨어·RL 규약과 변환 없이 맞는다. 서버 시뮬이 쓰던 URDF는 조향축 −z, 왼쪽 바퀴축 −y, 오른쪽 앞뒤 조향 조인트 이름이 뒤바뀌어 있었다. 위치·이름·mesh·카메라(20°)는 그대로 두고 **joint axis의 부호만** ksrc 규약으로 뒤집은 자산을 만든다.
```bash
PYTHONNOUSERSITE=1 PYTHONPATH=. .venv-mapping/bin/python -m experiments.site_sim.rover_assets --output outputs/site_sim/rover_assets_ksrcsign --ksrc-signs
cp outputs/site_sim/rover_assets_new/orb_stereo.yaml outputs/site_sim/rover_assets_ksrcsign/
# 업로드할 때: ... upload LABEL <임무> --assets outputs/site_sim/rover_assets_ksrcsign ...
```
뒤집힌 joint는 조향 4개와 왼쪽 바퀴 2개이고, 생성된 기하는 `steering_sign` +1, `rolling_heading` 0이다. 서버 14초 실행에서 IK는 그대로 정상이고, RL+현실 옵션은 변환 레이어(원본 URDF)와 같은 거동(첫 회전 −92°~−95°)을 보였다. 규약 변환(`joint_convention.py`)은 이 자산에서도 항등에 가깝게 동작하므로 그대로 두어도 된다.
**남은 관찰:** RL+현실 옵션을 같이 켜면 첫 제자리 회전에서 yaw가 −92°~−95°까지 돌았다가 돌아온다(IK+같은 옵션은 −38°, RL만은 −36°). 추정이 늦거나 RL의 바퀴 배율(최대 1.6)로 더 빨리 도는 것이 원인 후보이며 아직 가리지 못했다. 같은 구성의 두 실행(변환 레이어 쪽 정상 종료, KSRC 부호 쪽 6.5초에 `unsafe_observed_motion`)이 갈려서 시드를 바꾼 반복 실행이 필요하다.

## 주의

- GPU 한 장당 실행 하나(VRAM 약 13 GB, 사용률 약 98%: 10-17 실행 기록). 서버 래퍼는 여유 12 GB 미만이거나 사용률 20% 초과면 시작을 거부한다. 20 ms 제어에서의 값은 아직 재지 못했다.
- 20 ms 제어는 실행 시간이 더 길다(기존 repeat1018a는 실제 약 23분).
- 시뮬은 강체 마찰이라 모래 침하가 없다. ORB 설정(서버 특징점 2000, FAST 12/3 대 실기 1200, 20/7, raw)은 이번에 맞추지 않았다. Pi 기록으로 비교한 뒤 정한다.
- RL 정책은 v1(VO 입력 없음)을 썼다. v3로 바꾸려면 `<policy>.npz`를 같은 방법으로 만들고 `--rl-policy res8_mlp_v3vo_rc4000`을 준다(VO 항은 "없음" 값).
- 테스트: `KSRC_ROOT=outputs/site_sim/ksrc_20261016 QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/site_sim tests/localization -q`

## 코드 위치

`experiments/localization/imu_realism.py`, `online.py`(IMU 누락·현실화 연결), `experiments/site_sim/{rl_residual,export_policy_npz,start_noise,friction,clearance}.py`, `drive.py`(지연·RL 연결), `run.py`(옵션, `run_options.json` 기록), `server.py`(업로드·전달·fetch), `viewer.py`·`viewer.html`(재생에 설정·벽 거리 표시), 테스트 `tests/site_sim/test_{rl_residual,numpy_policy,realism_options,viewer_replay_extras}.py`, `tests/localization/test_imu_realism.py`.
