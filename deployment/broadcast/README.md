# 중계 화면 배포 — 2026-10-23

최종 KSRC 맵의 바구니 출발점에서 RUN1 진입점까지의 시연과, 실제 로버 pose/회차 상태 중계를 제공한다. 노트북에서 실행한다. 네 방향 ANCHOR와 RUN3/4를 사용하기 전에 [CLI.md](../../CLI.md)의 절차로 Pi 주행 배포본을 업데이트한다. launcher가 Pi 스택이나 모터를 대신 시작하지는 않는다.

## 팀원 빠른 시작

기존 배포 checkout에서 이 Git commit/branch를 받는다. 노트북의 기존 numpy/pygame 환경을 쓴다. 새 환경이면 인터넷이 되는 곳에서 미리 `pip install -r deployment/broadcast/requirements.txt`로 준비한다.

### 로버 없이 시연

```bash
cd ~/rover-mapping
python deployment/broadcast/launch.py --demo
```

브라우저에서 **▶ 시뮬 재생**을 누르면 바구니→RUN1 진입 구간이 약36초간 한 번 재생된다. 다시 누르면 처음부터, STOP은 재생도 중단한다. Python은 실제 설치된 환경의 실행 파일을 쓰면 된다. 예: `/home/su/rover/.venv-mapping/bin/python`.

### 실제 로버와 연결

1. Pi는 기존 [CLI.md](../../CLI.md) 절차대로 `./ksrc ... start --real`을 실행한다. 최초에는 바퀴를 띄운 시험과 시뮬 검증을 권장한다.
2. 노트북에서 SSH 공개키 접속을 먼저 확인한다. Pi와 같은 AP LAN이면 되고, 외부 GPU 서버/VPN은 필요 없다.
3. 노트북에서 아래 하나를 실행한다. **실제 조이스틱도 함께 실행되므로 기존 조이스틱 창은 종료한다.**

```bash
cd ~/rover-mapping
python deployment/broadcast/launch.py --pi pi@192.168.0.101
```

이 명령은 중계 서버·읽기 전용 SSH 상태 relay·기존 조이스틱 GUI를 켜고 브라우저를 연다. Pi의 모터 스택을 시작하거나 START/ANCHOR를 대신 누르지는 않는다. 실제 출발·4방향 ANCHOR·E-STOP은 조이스틱에서 조작한다. 실기 모드에는 사이트 시뮬 재생 버튼이 없으며 API도 재생을 거부한다.

Pi 경로가 다르면 `--remote-root /actual/deployment/ksrc`, 상태 폴더를 직접 고르면 `--state-dir state/field_...`를 추가한다. 기본은 `state/latest_test.txt`가 가리키는 상태 폴더, 없으면 `state/`다. SSH가 끊기면 재접속하며 오래된 위치를 새 값처럼 전송하지 않는다. 종료는 조이스틱 창 닫기 또는 실행 터미널 Ctrl-C.

## 키보드 속도

`--grid-keys --max-lin .15` 기준:

- 평소 **0.10m/s**, 왼쪽/오른쪽 **Shift를 누르는 동안0.15m/s**, 떼면0.10.
- 대각선도 총 속력 상한은 동일. Space/S는 Shift와 무관하게 정지.
- 마우스 패드와 회전 배율은 기존 그대로.

## 실제 데이터와 상태 전환

- Pi의 status.json은 현재5Hz로 갱신된다. relay는 이 파일에서 pose를 읽고 연속 pose 차분으로 v/w·지면 속력을 추정한다. 고정 명령속도를 실제 속도로 표시하지 않는다.
- 속도 표시만 완만하게 평활화하며 임의 노이즈를 더하지 않는다. 기록과 제어값은 바꾸지 않는다.
- 바구니 출발 위치/방향: `(2.659343,5.572979)m`, yaw `-76.209°`. 최초 TELEOP pose를 이 기준에 정렬해 진입 기록을 시작한다. 따라서 연결은 바구니 출발 때 켜는 기준이다.
- 성공한 ANCHOR의 anchor_count 증가를 받으면 Pi가 재설정한 site pose를 그대로 사용한다.
- 실행기 active/run 상태로 RUN1~4 기록을 자동 전환한다. 반복 상태 패킷은 기록을 초기화하지 않는다. RUN 시작 시 pose를 출발점에 강제로 붙이지 않는다.
- RUN2는 RUN1 궤적을 보존한다. 사이트를 RUN2 도중 처음 켜면 이전 RUN1 기록을 복원할 수는 없다.
- 이 배포의 사전 MPPI/슬립 캐시는 **바구니→RUN1 진입과 RUN1~4 본 주행**을 지원한다. 본 주행은 배포 MPPI로 계산한 `mission_cache.npz`를 사용하며 회차·단계·pose/v/w가 모두 맞을 때만 표시한다. 회전·관찰·계획 불가·캐시 밖에서는 예측을 숨긴다.
- 캐시 범위: 경로 좌우15cm·방향±40°,44,449개 상태. 범위 밖은 실제 pose/궤적을 계속 표시하고 예측만 숨긴다.
- 슬립은 미교정 견인 모델 추정값이며 실측 바퀴 슬립이 아니다. 정지 중에는 수치를 비운다.
- 바구니 첫30cm는 중심선 통과만 확인했으며 차체 전체의 충돌·접촉·모래 물리를 검증한 것은 아니다.

## 검증 범위

배포 좌표는 float16으로 저장해 최대 약2mm 오차, 표시 후보16개. 제어에는 사용하지 않는다. 키보드는 dummy pygame→실제 localhost UDP 시험으로 기본/Shift/해제/대각/정지를 확인했다. 실제 로버 실주행 또는 원격 Pi 연결을 이번 변경에서 실행하지 않았다. 실제 연결 전에 모터 없는 상태와 시뮬레이션으로 확인한다.

개발 로그와 실기 원자료는 `outputs/broadcast_sessions/`에 저장하며 Git에 넣지 않는다. 기존 Pi `local/`, `state/`, 보정·모델은 덮어쓰지 않는다.

### RUN 3·4 수신

Pi/조이스틱에 추가된 START RUN3(30도 중단 후 외곽 우회), RUN4(외곽 직행)의 실제 수락 상태를 자동 수신한다. 기록은4회차 슬롯으로 분리하고 이전 회차를 보존한다. 중계에서 경로를 선택하거나 실기 명령을 보내지는 않는다. 회차3/4도 본 주행 MPPI 후보·슬립 모델 추정을 표시한다. RUN3/4 하행은 구역4→3→1의15도 경사이며, 캐시도 `descent15_entry→descent15→return15`에 맞춘다.

## 조이스틱 기구학 시뮬레이션

```bash
python deployment/broadcast/launch.py --sim
```

Pi·SSH·SPI·ROS 없이 localhost의 가상 로버, 실제 조이스틱, 사이트를 함께 실행한다. 기본은 RUN 출발점에서 시작하며 정지3초 후 START RUN1~4를 선택한다. 출발좌표는 같고 ANCHOR1=−Y,2=−X,3=+Y,4=+X를 선택할 수 있다. GUI는 맵축 기준 ↑4/+X,←3/+Y,↓2/−X,→1/−Y 배치다. 실제 가상 자세를 선택 방향으로 맞추고 ANCHOR를 누른다. 수동 키·Shift·E-STOP을 사용할 수 있다. 이후 출발점·방향으로 TELEOP 복귀하거나 launcher를 재시작해 다음 회차를 시험한다. 사이트 하단에 `KINEMATIC SIMULATION`이 표시된다. ORB 정상·이상적 SE2 운동을 가정하며 수동 충돌·바퀴/모래 접촉은 계산하지 않는다. `scipy`가 필요하므로 최신 requirements로 준비한다.

## 본 주행 캐시와 재현

실제 배포 주행 상태기·MPPI로 −Y·−X 두 출발 방향의 RUN1~4를 완주하고, 이동 4cm/회전 8°마다 횡방향 ±8cm·방향 ±20°의 주변 pose를 같은 planner로 계산했다. 16개 안전 후보와 선택 궤적을 float16으로 저장한다. 후보 색은 궤적의 평균 지도 경사이며 제어 목적함수 수치가 아니다. 조회 허용 범위는 위치 4cm, 방향 5°, v 차이 0.08m/s, w 차이 0.3rad/s다. 단계는 relay가 이미 받는 Pi status의 `stage`를 보존해 내부 조회에만 사용한다. Pi 제어·통신 payload 원본은 바꾸지 않는다.

실측 슬립이 아닌 기존 미교정 견인 모델을 사용하고, 후진은 이동 방향을 반대로 적용하며 정지 슬립은 표시하지 않는다. 캐시 생성 결과와 출처 hash는 [mission_coverage.json](data/mission_coverage.json)에 기록한다. 생성에는 numpy/scipy 환경을 사용한다.

```bash
OPENBLAS_NUM_THREADS=1 python deployment/broadcast/build_mission_cache.py
OPENBLAS_NUM_THREADS=1 python deployment/broadcast/verify_missions.py
```

두 번째 명령은 GUI 없이 4회차를 가상 주행하고 실제 UDP→HTTP 경로의 완주·표시 일치율·로컬 조회 시간을 확인한다. Pi 네트워크 지연 검증과는 구분한다.

이번 로컬 연결 시험은 네 출발방향×RUN1~4의16조합 모두 complete였다. AUTO 캐시 일치율90.2~100%, HTTP 조회 p95 약73ms(1,481회)이며 캐시 밖은 숨긴다. TELEOP 전후/좌우 이동의 UDP→HTTP30회 시험은 p95 약17ms, 모델 계산 p95 약11ms였다. [검증 기록](data/mission_verification.json)에 저장했다. 로컬 기구학·표시 시험이며 Pi 네트워크 지연을 뜻하지 않는다. 캐시는55,075개 상태, 약41.2MiB다.

## TELEOP 보조 예측

수동 주행 중에는 노트북에서 현재 pose·v·w·횡속도로 국소 MPPI를 계산한다. 횡속도도 이미 수신한 위치 차분으로 구하며 Pi에 추가 센서 스트림이나 전송 주기를 요구하지 않는다. 최대30cm/3초의 현재 이동 원호를 국소 목표로 사용하며 미션 waypoint는 참조하지 않는다. 전후진·좌우 병진을 지원하고 선택된 궤적은 지도 안전 검사를 거친 표시 예측이다. 조이스틱/모터 제어에는 반영하지 않는다.

사이트의 `TELEOP MODEL`은 이 보조 예측, `CACHE MATCH`는 사전 결과 조회를 뜻한다. 정지·순수 회전·stale·지도 밖·계획 불가 시 예측을 숨긴다. 속도는 계속 실제 수신 pose/시뮬 운동에서 표시한다. GUI 상단 `ANCHOR1~4`는 선택한 기준 방향이며 실제 로버를 회전시키는 버튼이 아니다. AUTO는 같은 회차·단계·pose/v/w 캐시만 사용한다. 캐시 밖 AUTO에 수동 모델을 섞지 않는다.

AUTO 캐시는 −Y·−X 두 기준 주행으로 생성한 world pose 결과다. 네 방향 모두 같은 조건으로 조회하며 +Y·+X 출발의 초기 회전에서는 예측이 비어 있을 수 있다. 실제 pose·속도·기록은 유지한다.
