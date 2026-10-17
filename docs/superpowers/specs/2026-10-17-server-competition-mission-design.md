# 서버 Isaac 대회 임무·costmap 기록·실시간 관찰 설계

## 목적

서버에 이미 있는 경기장과 로버로 대표점 방문 임무를 실제 물리 시뮬레이션한다. 경로 설정, pose feedback에 따른 joint 구동, costmap과 주행 기록, 노트북 브라우저 실시간 관찰을 한 실행으로 연결한다. 사용자 요청은 서버 시뮬레이션에서 경로를 설정하고 따라가며 costmap을 기록하고 실시간으로 보는 것이다.

## 확인한 기존 자산

- 코드 기준: `/home/su/rover-mapping`, `codex/site-mission`. 기존 `SiteMap`, `SiteMission`, `MissionRuntime`을 재사용한다.
- 서버: SSH `hpi-lab`, `/home/sumee/gyusa`, RTX 5090 두 장. 2026-10-17 조회에서 GPU 작업은 없었고 별도 `ksrc_tb` TensorBoard만 실행 중이었다. 새 실행 직전에 사용량을 다시 확인한다. GPU 1만 사용한다.
- 이미지: 기존 `nvcr.io/nvidia/isaac-sim:6.1.0`. 새로운 이미지 build나 기존 환경 패키지 변경을 전제로 하지 않는다.
- 경기장: `/home/sumee/gyusa/arena_sand_v15_m5.usd`. 기존 MPM에서 얻은 고정 모래 표면과 칸막이 형상이다. 실행 중 변형 가능한 모래 물리 모델이 아니다.
- 기존 물리 주행: `/home/sumee/gyusa/runs/loop_route/20261016-2141_gyusa_ring`, `route_complete=true`, waypoint 10개, sim 340.34초. 외곽 폐루프의 증거이며 새 대회 임무 통과 증거는 아니다.
- 기존 scene importer·swerve joint 구동 소스는 `/home/su/rover/experiments/orbslam3_ros2`에 있다. 필요한 모듈만 Git 저장소에 가져와 출처를 기록한다. 원본 실행 폴더나 서버 기존 코드를 덮어쓰지 않는다.
- 기존 경기장 STL reader는 원본 좌표에 `0.1`을 곱한다. 이 STL을 m/mm로 임의 해석하지 않는다. meter 형상을 별도 생성하고 원본 hash·scale·축 변환을 provenance에 기록한다.
- 기존 sand 높이 NPZ는 `(x,y)` 축 순서다. 새 격자 `(row=y,col=x)`와 변환을 명시하고 좌표/축 검사를 수행한다.

## 선택한 접근

기존 Isaac scene·joint drive와 새 mission runtime을 연결하고, 별도 loopback HTTP 화면으로 관찰한다. WebRTC 전용 앱보다 설치·UDP 중계가 적고 costmap을 함께 보여주기 쉽다. 기존 합성 replay만 실행하는 방식은 서버 물리 주행 요구를 만족하지 못하므로 비교용 회귀 검사에만 사용한다. ROS/Nav2/ORB 전체 연결은 이번 첫 임무 시험의 전제가 아니다.

## 경기장·경로 설정

1. 기존 경기장과 일치하는 meter 형상, 모래 표면 및 칸막이를 바탕으로 임무 지도를 준비한다. 형상 좌표와 simulator world 좌표의 변환을 저장한다. 시뮬레이션 pose 입력은 명시적 `sim_ground_truth` 출처로 기록한다.
2. 출발·갈림길·공통 정상과 10°/15°/30° 경사 구간의 경유점을 설정한다. 지도상 안전 통로 연결은 기존 A*와 선분 검사로 확인하고, 경로 검사 실패 시 실행을 시작하지 않는다. 로버 여유 반경과 경사 제한을 저장한다.
3. 경로와 설정은 `SiteMission` bundle로 저장해 기존 임무 편집기에서 다시 열 수 있게 한다. 데모 평면 경로를 실제 경기장 경로인 것처럼 사용하지 않는다.
4. 초기화 후 실제 로버 pose가 출발 tolerance 안에 있는지 확인하고, 안착 과정은 임무 진행/정체 시간에 포함하지 않는다.

## 임무와 물리 구동

- 출발 → 갈림길 → 30° 진입 → 등판 → 정상 → 10° 하산 → 출발지 복귀. 진행 부족 시 실제 주행 breadcrumbs로 후퇴 → 15° 우회, 재실패 시 10° 상행.
- `MissionRuntime`의 bounded body velocity 요청을 기존 swerve 역운동학으로 네 steering position·wheel velocity joint에 적용한다. 시작 안착을 제외한 경로 추종 중 pose teleport나 정답 경로 pose 강제 주입은 하지 않는다.
- 물리 dt 0.005초, 임무 제어/telemetry 목표 0.05초, 화면 갱신 목표 wall 0.2~0.5초. 실제 갱신률·sim time·wall time을 기록한다. 실시간 배속 1을 보장한다고 표시하지 않는다.
- timeout, 비정상 pose, 경로 이탈/지도 검사 거부, 종료/예외 시 zero wheel target을 적용하고 이유를 저장한다. 기존 runtime의 추적/시각/좌표 유효성 검사를 보존한다. 실패를 정상 완료로 기록하지 않는다.
- 자연 주행에서는 30° 성공을 그대로 기록한다. 자연 주행이 후퇴 정책을 거치지 않으면 별도 `fault_injection` 실행으로 trial 구간 정체를 만들어 후퇴·우회 상태 전이를 검증한다. 주입 조건·시각·적용 명령을 별도 기록하고 화면에도 표시한다. 이를 모래에서의 자연 등판 실패로 주장하지 않는다.
- 실기 모터 송신·MCU/firmware·CAN·Pi 센서 및 다른 서버 작업은 변경하지 않는다.

## costmap과 기록

- 일반 경로와 30° 시험 경로의 경사 기준을 분리한 geometry costmap을 저장한다. unknown=255, blocked=254, risk=0..252를 보존한다. 시험 경사 예외로 벽/단차/unknown을 통과 가능하게 바꾸지 않는다.
- 주행 중 방문 횟수, 시각, 진행 부족 위치와 임무 이벤트를 별도 경험 층으로 누적한다. 이 층은 주행 관측이며 미방문 칸을 안전하다고 채우지 않는다. 첫 시험에서는 경험 층으로 자동 경로 재계획을 수행하지 않는다.
- 입력 geometry·임무 설정·원본 hash·코드 Git SHA·이미지 식별자·physics/drive 설정을 manifest에 저장한다.
- telemetry JSONL/CSV에 sim time, wall time, 실제 position/quaternion, 요청/적용 명령, 실제 steering/wheel joint 값, state/route/direction/대표점 상태를 기록한다.
- 시작·일정 sim 간격·상태 전환·종료 시 costmap/경험 층 snapshot을 NPZ로 저장한다. 최대 실행 시간과 snapshot 수를 제한하고 화면 thread가 물리 thread를 기다리게 하지 않는다.
- `events.jsonl`, 최종 `summary.json`, `completion.json`, 최종 지도·계획/실제 궤적 그림을 남긴다. completion은 임무 성공/실패/timeout을 구분한다. 중단 시 가능한 부분 로그와 종료 원인을 보존한다.
- 바퀴별 slip 또는 침하를 계산하지 않은 경우 0으로 꾸미지 않고 unavailable로 둔다. ground-truth 위치로 수행한 시험은 ORB localization 정확도 검증이 아니다.

## 실시간 화면

- 기존 stereo camera 중 전방 영상과 전체 경기장 관찰 camera를 낮은 주기로 렌더링한다.
- 한 화면에 경기장 영상, metric XY costmap, 세 계획 경로·실제 궤적·현재 위치/방향, 대표점 상태, 현재 단계/우회 이유, sim time·wall time·입력/영상 age, 기록 폴더를 보여준다.
- 지도 색상 범례와 simulation-only / pose source / fault-injection 유무를 표시한다. 영상 미준비·연결 실패·오래된 마지막 상태를 현재 진행으로 오인하지 않게 표시한다.
- HTTP는 서버 loopback에만 bind하고 SSH local tunnel로 `http://127.0.0.1:<port>`에 연결한다. 방화벽 변경·외부 공개 서비스·별도 계정 설정은 없다. 사용자 Codex 브라우저 패널에 화면을 연다.
- 브라우저 종료/시각화 통신 단절이 서버 기록을 중단시키지 않는다. 실행은 bounded timeout으로 종료하며 결과 화면과 저장 기록은 이후 확인할 수 있다.

## 실행 격리

전용 `/home/sumee/gyusa/runs/site_mission/<unique_run>/`와 독립 container를 사용한다. 기존 arena/rover 자산은 read-only로 mount하거나 run 안에 사본을 만든다. 기존 container·학습 로그·`latest` 링크는 건드리지 않는다. notebook에는 명시한 tunnel과 viewer만 추가한다. 새 실행만 종료할 수 있도록 container/PID·명령을 기록한다.

## 검증·인수 기준

1. 좌표 축·scale·origin과 경로 단위, geometry costmap의 unknown/blocked 구분, trial 제한, snapshot 왕복/기록 상한을 CPU 검사로 확인한다.
2. fake articulation에서 runtime 명령→swerve joint 적용, 정지/예외 시 zero target, natural/fault-injection 구분과 기록을 검사한다.
3. HTTP 화면의 지도/경로/상태/영상 endpoint와 stale 표시를 확인한다. snapshot과 telemetry의 시각·frame 일치를 확인한다.
4. GPU 1에서 기존 로버 asset의 안착·joint 구동·영상·costmap snapshot을 짧게 smoke 실행한다. 실패하면 원인을 기록하고 해당 연결을 수정한 뒤 재시험한다.
5. 전체 자연 임무를 실행해 대표점·정상·복귀와 계획 대비 궤적, costmap snapshot·이벤트를 확인한다. 물리 주행에서 실패하면 실제 실패 원인과 부분 결과를 보고한다.
6. 별도 명시적 정체 주입으로 30° 실패 → breadcrumb 후퇴 → 15°/10° 대체 → 정상·복귀를 검사한다. simulator의 실제 pose feedback으로 진행하며 실제 등판 성능으로 해석하지 않는다.
7. 사용자 브라우저에서 실제 갱신되는 서버 화면을 확인하고 local URL·결과 폴더·성공/실패 근거를 전달한다. 서버 실행이 불가능하면 로컬 합성 replay로 완료를 대체하지 않는다.

## 후속 범위

live ORB localization·STL 정합, 실기 모터 adapter, 동적 장애물 회피/온라인 재계획, 변형 모래·정량 견인력 보정, RL 정책 비교, 사천 prior와 현장 관측 병합은 별도 작업이다.
