# 시뮬 VIO 닫힌 루프 · 실기 IMU 시각차 인수인계 (Codex ↔ Claude Code)

갱신: **2026-10-18** · 작업 위치: `/home/su/rover-mapping` · 이어받는 쪽은 이 문서와 `git log`, 아래 "미해결"만 읽으면 된다.

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

1. **당근 추종기**: 경로 위 lookahead 점 추종, 급한 꼭짓점에서 corner gating, 경로 끝·꼭짓점 근처 속도 조절. Nav2 없이 고정 경로 임무에 충분하다. KSRC 학생 정책은 `cmd_vel` 위의 잔차(조향 ±30°, 바퀴 이득 ±60%) 구조 B로 붙인다.
2. **슬립 기반 감속**: 슬립 계수 <0.8이면 ×0.5, <0.6이면 ×0.3. 느려진 목표에서 로버가 더 힘을 쓰도록 하는 방식은 실기에서 확인한다.
3. **필터 상태 → 안전 정책**: `imu_gap` → 정지, `recovering` → degraded를 `SensorSafety`/`MissionRuntime`에 연결.
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

## 5. 이번에 얻은 교훈

- 오프라인 재생이 정확해도 닫힌 루프는 다르다. 제어가 만드는 저속·제자리 회전 구간이 추정기에 불리하다. 닫힌 루프 진단은 `estimator_frame`처럼 프레임별로 남긴다.
- 엔코더는 매 프레임 속도 관측으로 넣으면 경사에서 36 cm까지 나빠진다. IMU 공백과 낮은 슬립에서만 쓴다.
- 오류 정정 기록: "+65 ms 부호 반전"은 −65 ms였고, "로버가 바닥과 같은 회색"이라던 것은 메시가 렌더되지 않은 것이었다. Pi의 ORB 부하는 sensors-only라 없었다.
- `git add -A`가 다른 에이전트의 미추적 파일(CLAHE VO, high-contrast VO, stereo capture, sensor safety, VO shadow prep 문서)을 내 커밋에 포함시켰다. 커밋은 파일을 지정해서 한다.
- Pi `rclpy` 실패는 python3.12의 `cap_sys_nice` 파일 capability가 `LD_LIBRARY_PATH`를 지운 탓이었다. 사용자가 `setcap -r`로 해제했다.

## 6. Codex에게 넘길 때

Codex는 이 문서와 [닫힌 루프 문서](../experiments/2026-10-18-sim-vio-closed-loop.md), `git log 12b0b5a..HEAD`를 읽고 4.1부터 이어간다. 다른 스레드의 미커밋 파일이 작업 트리에 섞이므로 커밋 전에 `git status`로 내 파일만 골라 담는다.
