# 시뮬 VIO 닫힌 루프 · 실기 IMU 시각차 인수인계 (Codex ↔ Claude Code)

갱신: **2026-10-18** · 작업 위치: `/home/su/rover-mapping` · 이어받는 쪽은 이 문서와 `git log`, 아래 "미해결"만 읽으면 된다.

**Codex 후속 진행:** 현재 코드 기준은 `codex/site-mission`의 `54e7c50`이다.
시뮬 IMU의 시작 구간 누락·장착 변환·불규칙 5/50 ms 적분 일관성을 수정했고,
실제 IMU/ORB/pose 조회 순서를 저장해 온라인 추정과 같은 입력으로 재생할 수 있다.
전체 검사 **714 passed, 16 skipped**. `codex_vio1018a`는 60초 진단에서 30° 실패→후퇴→15° 우회를 진행했고,
`codex_vio1018b`는 58.82초 우회 회전 중 안전 정지했다.
현재 서버 **`codex_vio1018c` / GPU0 / 관찰 포트8782**로 전체 시험 중이다.
아래 과거 진단과 구분해 [후속 실험 기록](../experiments/2026-10-18-sim-vio-closed-loop.md)의 Codex 절을 먼저 읽는다.
Claude Code의 당근 추종기·슬립/필터 안전 정책은 `353ee2a`, `0852c04`로 반영됐다.

이 문서는 Codex 스레드 `01a14555…`(서버 임무 시뮬레이션)와 `01a14932…`(실기 카메라–IMU 시각차)를 Claude Code가 이어서 진행한 약 5시간 분량의 기록이다. 날짜별 상세 근거는 [시뮬 VIO 닫힌 루프](../experiments/2026-10-18-sim-vio-closed-loop.md)에 있다. 이 문서는 **상태·재개 방법·주의점**만 적는다.

## 1. 한눈에 보는 상태

| 항목 | 상태 | 근거 |
|---|---|---|
| 새 URDF로 서버 임무 시뮬 | 완료 | `rover_assets.py`, 커밋 `12b0b5a` |
| 비반복 사진 모래 텍스처(대비 3단계) | 완료 | `texture_quilt.py`, `f6edd59` |
| 카메라–IMU 시각차 | **−45 ms 상수 반영**(legacy SPI 50 Hz 경로 한정) | `c8d980c`, `inertial_timing.py` |
| ESKF를 이 저장소로 통합, IMU 공백 방어, 엔코더 추측 항법 | 완료·테스트 있음 | `c53f8d7` |
| ORB 상실 후 재닻·기준선 갱신·0.1 s 렌더 지연 보정 | 완료, 239 s 기록 오프라인 12.4 cm RMSE | `1583af3` |
| 닫힌 루프(추정 pose로 주행) 배선 | 완료, **전체 임무는 아직 못 끝냄** | `090cd64` |
| 당근 추종기, 슬립 감속, 상황별 신뢰도 스케줄, 경사 후 재포착 | **미구현**(설계만) | 4절 |

## 2. 반드시 알아둘 사실

- 시뮬 모래는 **단단한 고정 메시**다. 슬립·감속 임계값은 실기 모래 데이터로 다시 맞춘다.
- 시뮬 IMU는 GT pose에서 만든 **이상적인 값**이다(`imu_degrade`로 일부만 열화).
- 시뮬 영상은 timestamp보다 **0.10 s 늦은 장면**이다(`RENDER_LAG_S`). 오프라인은 `--render-lag-s`, 온라인은 필터 시각 지연으로 보정한다.
- 정식 ORB-SLAM3는 리셋 뒤 map id를 0부터 다시 쓴다. map id 변화만 보지 말고 상실→복귀도 새 map으로 본다.
- 실기 −45 ms는 **legacy SPI(50 Hz 평균) 경로에서만** 측정했다. raw 전송 펌웨어(220 Hz, 모터 데이터 없음, bench 전용)는 값이 다르므로 쓰기 전에 다시 잰다.
- 필터는 알려진 시작 위치와 시작 3초 정지(영점)를 요구한다(`--start-hold-s 3`).
- 벽·STL 점프 보정은 실기 검증 전까지 주행 경로에 넣지 않는다(평상시 이득 없음, 바퀴와 결합 시 실패 이력).

## 3. 재개 방법

```bash
cd /home/su/rover-mapping
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/localization tests/site_sim tests/handheld_mapping -q

# 서버(Isaac) 실행: tools/run_site_sim_server.sh
bash tools/run_site_sim_server.sh upload L outputs/site_sim/20261017c/mission --texture outputs/site_sim/sand_texture/sand_high.jpg
bash tools/run_site_sim_server.sh start L --gpu 0 --port 8775 --stereo-hz 15 --start-hold-s 3 --estimator   # 닫힌 루프
bash tools/run_site_sim_server.sh start L --gpu 0 --port 8773 --stereo-hz 15 --start-hold-s 3              # GT 주행 + 영상 저장
```

- 서버 SSH 터널은 유휴 30분이면 끊긴다(`ServerAliveCountMax=40` 적용). 끊기면 포트 포워딩만 다시 연다.
- `pkill -f`/`pgrep -f`는 자기 셸을 죽인다(exit 144). `pgrep -x ssh -a | grep …`를 쓴다.
- 이전 라벨·포트(8765/8770/8773/8775/8776)가 남아 있으면 새 라벨·포트를 쓴다. `visualfix1017a`는 다른 스레드 것이므로 건드리지 않는다.
- 서버 경로: `/home/sumee/gyusa/runs/{stream_orb, stream_orb.cpp, orb_test.yaml, chain_orb.sh}`, 실행 기록 `site_mission/{vio1017b, full1017a, closed1017a/b/c}`. ORB 이미지는 `rover-orbslam3-jazzy:4452a3c4`.
- 로컬 결과: `outputs/site_sim/{full_1017a, vio_1017a/b, closed_1017b/c}`(Git 제외).
- 기록 오프라인 평가: `python -m experiments.site_sim.vio_offline fuse STEREO ORB OUT --imu avg50 [--encoder] [--gaps t0 t1] --render-lag-s 0.1`.

## 4. 미해결과 다음 작업

### 4.1 닫힌 루프 전체 임무 (최우선)

`closed1017a/b/c`는 모두 junction까지 못 갔다.

- `closed1017b`: 36.9 s에 `no_progress_to_junction`. 추정이 GT보다 최대 33 cm 뒤처짐.
- `closed1017c`(45 s 진단 런, `estimator_frame` 로그 포함): 19.8 s에 `unsafe_observed_motion`으로 정지.
  - 0~14 s는 추정 오차 2~5 cm로 양호. GT 주행(`full_1017a`)과 같은 경로를 같은 속도로 따라간다.
  - 14 s 부근부터 ORB state가 2↔3(상실)로 번갈아 바뀌고, 15.2~16.8 s는 품질 `degraded`로 `SensorSafety`가 속도를 ×0.3로 줄인다. 이어서 −0.6 rad/s 제자리 회전 구간(17.4~19.6 s)에서 오차가 4 → 15 cm로 커지며 추정이 +x로 앞서간다. 실제 로버는 거의 제자리다. 관측 이동(추정) 대 명령 불일치로 안전 정지.
  - `full_1017a`의 ORB는 같은 구간에서 상실이 없었고(19.95 s에 1회뿐), 닫힌 루프는 14 s부터 상실이 잦다. **원인은 아직 확정하지 못했다.** 후보: (a) 저속+회전 구간의 ORB 상실과 상실 중 IMU만으로 위치가 밀림, (b) 상실→복귀 시 재닻 직후 보정, (c) `SensorSafety`의 `unsafe_observed_motion` 기준이 제자리 회전에서 추정 병진 잡음에 너무 민감.
  - 다음: 상실 직전·직후 `estimator_frame`의 `reason/nis/residual`과 추정 yaw를 같이 기록(현재 로그에는 추정 yaw가 없다), 같은 구간을 오프라인 `OnlineEstimator`에 GT-driven 기록으로 재생해 비교. 원인이 확인되기 전에는 임계값을 완화하지 않는다.
- `run.py`의 `estimator_frame` 진단은 커밋 전이다.

### 4.2 설계는 끝났고 구현 전인 것

1. **당근 추종기(구현됨, 2026-10-18)**: `carrot_follower.py`(`CarrotPath`)와 `mission_runtime.py`에 연결. lookahead 0.2 m, 35° 이상 꼭짓점은 넘지 않고 꼭짓점·끝 0.3 m 안에서 최소 0.35배까지 감속, 진행 점은 앞쪽 가장 가까운 점으로 갱신. 임무 connector가 2.5 cm 격자 지그재그라 기존 "다음 점 추종"이 좌우로 흔들렸고(지연 0.4 s에서 junction 17.6→32 s), 꼭짓점은 현 길이 0.15 m 창으로 찾아야 지그재그가 꼭짓점으로 잡히지 않는다. lookahead 0.25 m 이상은 좁은 구간에서 안쪽을 잘라 `unsafe_observed_motion`이 났다. 처음 설계: 경로 위 lookahead 점 추종, 급한 꼭짓점에서 corner gating, 경로 끝·꼭짓점 근처 속도 조절. Nav2 없이 고정 경로 임무에 충분하다. KSRC 학생 정책은 `cmd_vel` 위의 잔차(조향 ±30°, 바퀴 이득 ±60%) 구조 B로 붙인다.
2. **슬립 기반 감속(구현됨, 연결 전)**: `SensorSafety.observe_slip`(해제는 +0.05 여유), 샘플 키 `slip_ratio`로 `MissionRuntime`이 전달, 감속 중 stuck 시간은 배율만큼 늘려 센다. 실제 값을 `drive.py`/`run.py`에서 넣는 일은 남음. 처음 설계: 슬립 계수 <0.8이면 ×0.5, <0.6이면 ×0.3. 느려진 목표에서 로버가 더 힘을 쓰도록 하는 방식은 실기에서 확인한다.
3. **필터 상태 → 안전 정책(구현됨, 연결 전)**: `observe_filter`, 샘플 키 `filter_status`. `imu_gap`→`filter_imu_gap` 정지, `recovering`→느림. 처음 설계: `imu_gap` → 정지, `recovering` → degraded를 `SensorSafety`/`MissionRuntime`에 연결.
4. **상황별 신뢰도 스케줄**: 평지는 VO 우세, 회전은 IMU 우세, 경사(기울기 > 약 8°, |gz| > 약 0.3, 슬립)는 영상·엔코더 신뢰 낮춤. `relative_update`의 `position_scale`/`rotation_scale` 인자까지만 있다.
5. **경사 후 재포착**: ORB 재지역화 우선, 벽·STL 점프는 실데이터 검증과 STL 스케일 확인 뒤에만.

### 4.3 기타

- `~/rover/experiments/imu_raw_transport/README.md`(저장소 밖)가 오래됐다. raw 펌웨어는 220.7 Hz, gap 0, legacy 복귀 확인됨.
- raw IMU를 매핑 기록기에 넣으려면 SPI 소유자를 하나로 한다. raw 경로를 쓰면 시각차를 다시 측정한다.
- 18일(일) 모래 실주행 데이터로 슬립·기울기 임계값을 보정한다.
- 짧아진 하강 경로에 맞춰 `entry10` 대표점 정의를 다시 확인한다.
- main 푸시는 사용자가 승인했다. 문서 정리 후 푸시한다. 외부 경로 하드코딩(`/home/su/rocker_bogie_4wd_rover`, `/home/su/rover/...`의 기준 ORB 런타임, 서버 경로)은 README에 명시한다.

## 4.4 작업 분담 (Claude Code ↔ Codex)

두 쪽이 서로 다른 파일만 건드린다. 겹치는 파일을 고쳐야 하면 먼저 상대에게 알린다.

| 담당 | 일 | 건드리는 파일 |
|---|---|---|
| **A. Codex** | 4.1 닫힌 루프 원인 규명·수정, 전체 임무 재실행(서버), 4.2-4 신뢰도 스케줄 | `experiments/localization/*`, `experiments/site_sim/{run,drive,server,vio_offline,orb_client}.py`, `tests/localization`, 닫힌 루프 문서 |
| **B. Claude Code** | 4.2-1 당근 추종기, 4.2-2 슬립 감속, 4.2-3 필터 상태 → 안전 정책 | 새 `experiments/handheld_mapping/carrot_follower.py`, `mission_runtime.py`, `SensorSafety` 관련 파일, 해당 테스트 |

- B(Claude Code)는 서버를 쓰지 않고 `tests/handheld_mapping`의 단위 시험과 GT 주행 재생으로 검증한다. 서버 실행은 A(Codex)가 원인을 고친 뒤 합쳐서 한 번에 한다.
- B의 인터페이스: 추종기는 현재 추정 pose와 경로를 받아 `(vx, yaw_rate)`를 돌려준다. 필터 상태(`imu_gap`/`recovering`)와 슬립 계수는 `SensorSafety` 입력으로만 받는다. 추정기 내부는 읽지 않는다.
- 제자리 회전 구간의 `unsafe_observed_motion` 기준(4.1 후보 c)은 A(Codex)가 결론을 낼 때까지 B(Claude Code)가 바꾸지 않는다.
- 커밋은 각자 파일을 지정해서 한다(`git add -A` 금지). 푸시는 둘의 결과를 합치고 시험이 통과한 뒤 한 번에 한다.

> 2026-10-18 갱신: 사용자가 Codex에 디버깅(A)을 먼저 맡겼으므로 담당을 맞바꿨다. Claude Code가 남긴 단서: `closed1017c` GT 궤적에 IMU만 적용해 재생하면 yaw는 정확하고 위치는 정지 구간부터 약 6 mm/s²로 어긋나 19 s에 107 cm가 된다(중력 상수는 양쪽 9.81로 같다). ORB 상실 중에는 위치가 사실상 IMU만으로 유지되므로, 4.1의 후보 (a)를 먼저 확인할 것. 재생 스크립트는 저장소에 없다(`OnlineEstimator`에 GT 궤적을 `on_step`으로만 먹이고 `on_frame`은 호출하지 않으면 된다).

## 4.5 로컬라이제이션 오차 5 cm 목표: 2026-10-18 분석 결과

목표(사용자 지정): 전체 임무를 추정 오차 **5 cm 이내**로 완주. `codex_vio1018c`(Codex가 돌린 마지막 전체 시험, `outputs/site_sim/codex_1018c`)는 132.6 s에 15° 등판 중 `unsafe_observed_motion`으로 멈췄고, 오프라인 재생 기준 xy RMSE 5.9 cm, 최대 24.4 cm다.

1. **IMU는 병목이 아니다.** 시뮬 IMU가 GT에서 나온 이상적인 값이라 IMU만 적분해도 132 s에 0.1 cm다. 실측 잡음 밀도를 넣어 재생해도 융합 결과는 RMSE 6.1 cm, 최대 23.9 cm로 같다(`eval_noise.py`와 같은 방식: 기록 IMU 이벤트에 `GYRO_DENSITY/√dt`, `ACC_DENSITY/√dt` 잡음을 더해 재생). IMU만(잡음 포함)은 자세 random walk로 22 m까지 발산하므로 영상 융합은 필요하다.
2. **오차의 원인은 ORB 추적 상실과 새 지도 분리다.** 132 s 동안 지도 id 0~5가 생겼고(상실 8회, 각 약 3 s), 한 지도 안의 ORB 절대 오차는 20 s 동안 1~2 cm였다. 상실 때마다 재닻을 내려 오프셋이 남고, 5~10 cm가 쌓인다(구간별 평균 오차 1.8 → 2.2 → 2.9 → 6.9 → 6.2 → 7.0 → 10.7 cm).
3. **상실은 15° 경사와 제자리 회전에 몰린다.** 15° 경사 구간은 keypoint 평균 477, 하위 10% 140, 프레임의 56.5%가 상실이다. 평지 시작은 평균 847이다. 제자리 회전(0.6 rad/s)에서도 상실 4회. 경사 위 영상(117.02 s, 상실 직전)을 보면 로버가 코를 든 만큼 시야 위쪽 약 60%가 어두운 허공이고 모래는 먼 띠 하나뿐이라, 특징점이 저해상도 원거리에만 남는다. 임계값만으로는 한계가 있고 경사 구간은 시야·카메라 자세 영향이 크다.
4. **ORB 설정만으로 상실이 줄어든다(오프라인 재생, 같은 stereo 프레임 1988장).** 기본(`iniThFAST 20 / minThFAST 7`) 추적 성공 81.4%(1618/1988), `12 / 3` 90.5%(1800), `12 / 3 + nFeatures 2000` 92.8%(1845). 상실 프레임은 각각 324 / 184 / 138. 재생은 서버에서 한 번에 약 45 s다(`replay_orb.sh`: `stream_orb`를 stereo 폴더 전체에 돌려 `orb_poses_<tag>.csv`를 만든다. 서버 `/home/sumee/gyusa/runs/`). `8/2`, `6/1`, 12레벨·1.15, `8/2 + 1500` 변형은 서버 연결이 끊겨 돌리지 못했다.
5. **132.46 s 점프(7→22 cm)의 직접 원인**: 상실 직후 재닻 뒤 첫 상대 보정에서 위치 gain이 3~4였다. 필터가 위치 절대 σ를 46 cm로 믿고 위치·속도 오차 상관이 커서 3 cm 잔차가 10 cm 보정이 된다. 위치 보정을 잔차 크기로 제한하면(최대 22.6 → 16.6 cm) 점프만 줄고 RMSE는 그대로(5.5 → 6.0 cm)다. 즉 필터 gain 조정만으로는 5 cm에 못 간다.
6. **다음 순서**: (a) 서버가 돌아오면 ORB 설정 변형 4개를 재생해 상실률이 가장 낮은 설정을 고르고 `assets/orb_stereo.yaml`에 반영, (b) 그 설정으로 전체 임무 닫힌 루프 재실행, (c) 그래도 상실 구간 오프셋이 남으면 상실 중 속도 0 가정·재닻 시 오프셋 억제, (d) 실기 ORB 설정은 별도로 검증한다(시뮬 텍스처 대비에 맞춘 값이다).

`test_orb_client.py::test_track_round_trip_parses_state_map_and_pose`가 응답 스레드와 파일 삭제 경합으로 드물게 실패한다(4회 중 1회).

## 4.6 벽 거리(RLC)·윗선 보정: 시뮬 결과와 다음 세션

시뮬에서 합성 깊이로는 평균 1.0 cm였지만 실제 시뮬 스테레오·윗선으로는 모두 기준선(평균 6.4 cm)보다 나빴다. 실기 스테레오 평가 절차와 코드 위치는 [벽 보정 실기 평가 절차](../experiments/2026-10-18-wall-range-real-stereo-eval.md)에 있다. 이번 세션 기준 주행 최고 조합은 영상+IMU+회전·경사 신뢰도 조정(`claude_1018e`, 평균 6.4 / 최대 12.1 cm)이고 Stanley 횡오차 항(`stanley_gain`)과 슬립 주입(`--wheel-gain-left/right`)은 서버 시험 중이다.

## 5. 이번에 얻은 교훈

- 오프라인 재생이 정확해도 닫힌 루프는 다르다. 제어가 만드는 저속·제자리 회전 구간이 추정기에 불리하다. 닫힌 루프 진단은 `estimator_frame`처럼 프레임별로 남긴다.
- 엔코더는 매 프레임 속도 관측으로 넣으면 경사에서 36 cm까지 나빠진다. IMU 공백과 낮은 슬립에서만 쓴다.
- 오류 정정 기록: "+65 ms 부호 반전"은 −65 ms였고, "로버가 바닥과 같은 회색"이라던 것은 메시가 렌더되지 않은 것이었다. Pi의 ORB 부하는 sensors-only라 없었다.
- `git add -A`가 다른 에이전트의 미추적 파일(CLAHE VO, high-contrast VO, stereo capture, sensor safety, VO shadow prep 문서)을 내 커밋에 포함시켰다. 커밋은 파일을 지정해서 한다.
- Pi `rclpy` 실패는 python3.12의 `cap_sys_nice` 파일 capability가 `LD_LIBRARY_PATH`를 지운 탓이었다. 사용자가 `setcap -r`로 해제했다.

## 6. Codex에게 넘길 때

Codex는 이 문서와 [닫힌 루프 문서](../experiments/2026-10-18-sim-vio-closed-loop.md), `git log 12b0b5a..HEAD`를 읽고 4.1부터 이어간다. 다른 스레드의 미커밋 파일이 작업 트리에 섞이므로 커밋 전에 `git status`로 내 파일만 골라 담는다.
