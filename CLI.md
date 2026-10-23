# 현장 CLI — 배포부터 RUN1~4까지

기준 브랜치: **`claude/sim-realism-rl-replay`**. 2026-10-23 Pi 백업 `cmwcmw57/pi-backup@7e2f955`의 경로·설정·정책 인터페이스와 대조했다. 아래의 **노트북 로컬**과 **Pi SSH**를 구분해서 실행한다.

켜는 프로그램은 Pi의 주행 스택 하나와 노트북의 중계+조이스틱 launcher 하나다. 별도 `udp_spi_bridge.py`·`run_policy.py`는 중복 실행하지 않는다. 현장에는 같은 AP 연결이 필요하며 외부 서버·VPN·인터넷은 필요 없다. Git 업데이트와 의존성 설치는 현장에 가기 전에 끝낸다.

> `start --real`은 실제 SPI·모터 제어를 연다. 시뮬레이션 검증 후 바퀴를 띄운 시험부터 진행한다. 실제 출발 위치·방향을 확인한 뒤 START를 누른다. 현장에 정지 담당자를 둔다.

## 1. 노트북 코드 준비 — 인터넷이 되는 곳

**[노트북 로컬]** 처음 받는 팀원은 한 번만 clone한다.

```bash
git clone --branch claude/sim-realism-rl-replay https://github.com/lee-robot-lab/rover-mapping.git ~/rover-mapping
```

이미 checkout이 있으면 업데이트한다. 작업 중인 변경이 있으면 먼저 보존한다.

```bash
cd ~/rover-mapping
git fetch origin
git switch claude/sim-realism-rl-replay
git pull --ff-only origin claude/sim-realism-rl-replay
git log -1 --oneline
```

**Python 환경:** numpy와 pygame가 있는 기존 환경을 사용한다. 현재 개발 노트북은 `/home/su/rover/.venv-mapping/bin/python`이다. 다른 노트북에서 새로 준비할 때만 아래를 실행한다.

```bash
cd ~/rover-mapping
python3 -m venv .venv-broadcast
.venv-broadcast/bin/python -m pip install -r deployment/broadcast/requirements.txt
```

이 문서의 노트북 명령은 `.venv-broadcast/bin/python` 기준이다. 기존 환경을 쓴다면 해당 Python 실행 파일로 바꾼다. Pi의 Python·ROS·보정 설정은 기존 것을 유지한다.

## 2. Pi에 최신 주행 코드 배포 — 세션 종료 후 한 번

Pi 주소 예시는 `192.168.0.101`이다. 실제 AP에서 할당된 주소로 변경한다. 노트북과 Pi가 같은 AP에서 통신해야 한다.

**[노트북 로컬 → Pi SSH]**

```bash
ssh -o HostKeyAlias=raspberrypi.local pi@192.168.0.101
```

기존 스택이 실행 중이면 GUI E-STOP으로 정지를 확인하고, **[Pi SSH]**에서 기존 세션을 종료한다. 기존 세션이 없으면 이 종료 블록은 생략한다.

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="$(cat state/latest_test.txt)"
./ksrc --state-dir "$TEST_DIR" estop
./ksrc --state-dir "$TEST_DIR" stop
cat "$TEST_DIR/stopped.json"
```

**[노트북 로컬, 별도 터미널]** 배포 폴더를 복사한다. `--delete`는 사용하지 않는다. `local/` 보정·의존성과 `state/` 기록을 보존하고, `/home/pi/ksrc`의 펌웨어·정책·모델은 교체하지 않는다.

```bash
cd ~/rover-mapping
rsync -a --exclude local/ --exclude state/ --exclude __pycache__/ \
  -e 'ssh -o HostKeyAlias=raspberrypi.local' \
  deployment/ksrc/ pi@192.168.0.101:/home/pi/ksrc_final_20261022/deployment/ksrc/
```

Pi 백업 저장소는 스냅샷이다. 이번 변경은 `rover-mapping`에서 받아 위처럼 배포한다. Pi의 백업 폴더에서 `git pull`하는 방식이 아니다.

**[노트북 로컬]** 중계 relay는 비밀번호 입력 없이 SSH로 상태를 읽는다. 이 명령이 성공해야 한다. 실패하면 노트북 공개키 접속을 먼저 설정한다.

```bash
ssh -o BatchMode=yes -o HostKeyAlias=raspberrypi.local pi@192.168.0.101 'echo SSH_OK'
```

## 3. Pi 스택 시작

**[Pi SSH]**

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
./ksrc check
```

`check`가 성공한 뒤 새 세션을 만든다. 기본은 S1 plan 분리이며, 실기는 `--real`을 명시한다. 옵션 없는 `start`는 모터 없는 FAKE다.

```bash
TEST_DIR="state/field_S1_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
printf '%s\n' "$TEST_DIR" > state/latest_test.txt
./ksrc --state-dir "$TEST_DIR" start --real
./ksrc --state-dir "$TEST_DIR" view
```

확인: `started REAL supervisor ...`, `대기 / TELEOP`, 센서 정상. 시작 직후 `초기화 중`이면 잠시 뒤 `view`를 다시 실행한다. `start`는 프로그램을 켜는 명령이고 **RUN을 시작하지 않는다**. 기본 세션 최대 시간은 1시간이다.

같은 `state/` 아래 대기 중인 이전 세션은 자동 종료될 수 있다. AUTO 중이거나 상태를 알 수 없는 다른 세션은 자동 종료하지 않는다. 오류가 나면 아래 [문제 해결](#10-문제-해결)을 따른다. 스택이 실행 중인 상태에서 배포 파일을 덮어쓰지 않는다.

## 4. 중계 화면과 조이스틱 켜기

**[노트북 로컬, SSH하지 않은 터미널]** 바구니에서 출발하기 전에 실행한다.

```bash
cd ~/rover-mapping
.venv-broadcast/bin/python deployment/broadcast/launch.py --pi pi@192.168.0.101
```

브라우저 `http://127.0.0.1:8765`와 조이스틱 GUI가 열린다. GUI에서 `MODE TELEOP`, `laptop link OK`를 확인한다. 기존 조이스틱 창은 종료하고 하나만 사용한다. Pi 경로가 다르면 `--remote-root /actual/deployment/ksrc`, 세션을 직접 지정하려면 `--state-dir state/field_...`를 추가한다.

조이스틱은 UDP **5006**, 중계는 읽기 전용 SSH relay를 사용한다. 중계 서버는 노트북 HTTP 8765 / UDP 9870을 사용한다. Pi의 수락된 START·ANCHOR 상태를 받아 기록을 전환하며, 사이트에서 실기 RUN을 시작하지 않는다.

### 조작 키와 버튼

GUI를 클릭해 포커스를 둔다. launcher는 `--grid-keys --max-lin .15 --max-ang .8`을 적용한다.

| 조작 | 동작 |
|---|---|
| `w` / `x` | 전진 / 후진 |
| `a` / `d` | 좌 / 우 병진 |
| `q e z c` | 전방좌 / 전방우 / 후방좌 / 후방우 대각선 |
| 왼쪽·오른쪽 Shift | 누르는 동안 병진 속력 0.15 m/s, 떼면 0.10 m/s. 대각선도 같은 속력 |
| `←` / `→` | 반시계 / 시계 회전, 최대 0.4 rad/s |
| `s` | 누르는 동안 병진·회전 0 |
| 왼쪽 패드 | 마우스 병진, 최대 0.15 m/s |
| 오른쪽 패드 | 마우스 회전, 최대 0.4 rad/s |
| Space / `E-STOP` | AUTO 해제·정지 요청. `MODE TELEOP`과 실제 정지 확인 |
| `ORB RESET` | TELEOP 대기 중 ORB만 재시작. 복구 동안 로버 정지 |
| 네 방향 ANCHOR | 같은 출발 좌표에서 선택 방향으로 pose 기준 지정. 정지 3초·ORB 정상일 때만 활성 |
| `START RUN 1` ~ `START RUN 4` | 해당 회차 AUTO 요청 |
| Esc / 창 닫기 | GUI 종료. 먼저 E-STOP으로 정지 확인 |

AUTO에서는 이동 조작이 무시된다. GUI 포커스를 잃거나 창을 닫는 것만으로 AUTO가 취소되지는 않는다. Space는 GUI가 키 입력을 받을 때 동작한다. 노트북 연결이 끊겨도 Pi AUTO는 계속된다.

## 5. 바구니 출발 → RUN 진입 정렬

**바구니 출발**은 사람이 TELEOP으로 이동하는 구간이다. 중계는 처음 받은 pose를 바구니 기준 `(2.659343, 5.572979) m`, yaw `−76.209°`에 정렬해 표시한다. 이 중계 정렬은 Pi 주행 위치 추정의 ANCHOR와 별개다.

**RUN 출발점**은 로버 `base_link` 중심 **site `(3.60, 2.82) m`**, 기본 정면은 **site −y / yaw −90°(ANCHOR1)**다. body +x가 정면이다. 다른 출발 방향은 아래 ANCHOR를 선택한다. TELEOP으로 실제 위치·방향을 먼저 맞춘다. 그림은 [배포 설정 안내](deployment/ksrc/README.md#1-초기-위치와-방향)를 참고한다.

버튼은 현재 맵 축 `+X ↑ / +Y ←`에 맞춘 십자 배치다. 좌표는 모두 `(3.60, 2.82)m`로 같다. 버튼은 회전 명령이 아니며, 실제 로버를 그 방향으로 맞춘 다음 누른다.

| 화면 위치 | 버튼 | 실제 정면 | Pi 명령 |
|---|---|---|---|
| 위 | ANCHOR4 | +X / 0° | `./ksrc --state-dir "$TEST_DIR" anchor4` |
| 왼쪽 | ANCHOR3 | +Y / +90° | `./ksrc --state-dir "$TEST_DIR" anchor3` |
| 아래 | ANCHOR2 | −X / −180° | `./ksrc --state-dir "$TEST_DIR" anchor2` |
| 오른쪽 | ANCHOR1 | −Y / −90° | `./ksrc --state-dir "$TEST_DIR" anchor1` |

기존 `anchor` 명령은 ANCHOR1 별칭이다. GUI와 Pi `view/status`에 `anchor_id`가 표시된다. AUTO 중 ANCHOR는 거절된다.

1. 실제 RUN 출발점·원하는 방향에 맞추고 손을 뗀다.
2. **3초 이상 정지**하고 ORB 정상 상태를 확인한다.
3. 최초 출발 보정 또는 방향 변경이 필요하면 해당 방향의 `ANCHOR1~4`를 한 번 누른다. `ANCHOR applied`를 확인한다. 바구니에서 누르면 RUN 출발 좌표로 잘못 지정되므로 누르지 않는다.
4. `READY: START allowed`를 확인한 뒤 원하는 START 버튼을 누른다.

`READY`는 추정 위치가 출발점 8 cm·선택 방향 8° 이내이고 센서·정지 조건을 만족한다는 뜻이다. 실제 맵 배치를 자동 확인한 결과는 아니므로 실제 위치도 확인한다. `ANCHOR`는 로버를 이동시키지 않는다.

`ORB not tracking`이 지속되면 TELEOP에서 `ORB RESET`을 누르고 가만히 둔다. 재기동·초기화에 약 10~15초가 걸릴 수 있다. 복구 중 반복해서 누르지 않는다.

## 6. RUN1~4와 다음 회차

| 버튼 | 상승 동작 | 정상 이후 |
|---|---|---|
| START RUN 1 | 30° 시도 → 중단·후퇴 시 15° 우회 | 기존 정상 관찰·외곽 하행·복귀 |
| START RUN 2 | 15° 직행 | 정상 관찰 생략, 하행 속도 0.7배 |
| START RUN 3 | 30° 시도 → 중단·후퇴 시 외곽 경로 상승 | 정상 관찰 → 15° 하행 → 출발점 복귀 |
| START RUN 4 | 처음부터 외곽 경로 상승 | 정상 관찰 → 15° 하행 → 출발점 복귀 |

외곽은 통칭 “10도” 경로이며 실제 맵 경사(5°·7°·7.27° 등)를 사용한다. RUN3/4는 정상 관찰 후 `descent15_entry→descent15→return15` 단계로 구역 `4→3→1`을 전진 하행해 출발점으로 복귀한다. 상승 구역은 `1→6→7→8→9→10→4`, 단계는 `outer6→outer7→outer8→outer9→outer10→outer4→summit`이다. 30° 시도에는 기존 5초 전진/10초 경과 중단 정책이 있다. 시간 제한에 따른 중단은 실제 슬립 측정으로 판정한 실패와 구분한다.

START 후 GUI의 `MODE AUTO  RUN n  <단계>`를 확인한다. CLI의 `sent startN`은 송신 확인이므로 실제 수락은 GUI 또는 Pi `view`로 확인한다. RUN2도 `entry15`를 거쳐 `ascent15`로 진행한다.

완료·실패 시 자동으로 TELEOP으로 돌아와 정지하고 결과가 표시된다. **다음 회차는 자동 시작되지 않는다.** ORB·정책·스택은 계속 켜 둔다.

다음 회차는 실제 시작 구역에서 선택한 ANCHOR 방향에 맞추고 3초 정지한다. 기존 ANCHOR1(−Y)을 유지한다면 RUN1/2의 +X 복귀 후 시계 방향90°, RUN3/4의 +Y 복귀 후 대략180° 돌아 맞춘다. 현재 실제 방향으로 출발하려면 출발 좌표에서 해당 ANCHOR를 새로 적용한다. **위치 추정이 맞으면 ANCHOR·스택 재시작 없이 다음 START를 누른다.** 추정이 어긋났다면 정확한 출발 위치·방향에서 ANCHOR 후 시작한다. 중계는 회차별 기록을 보존하며, 새 RUN1 시작은 기존 회차 기록을 초기화한다.

### GUI 대신 Pi에서 요청할 때

**[Pi SSH]** GUI 버튼과 CLI 중 하나만 사용한다. 아래 START 명령은 원하는 회차 **한 줄만** 실행한다.

```bash
./ksrc --state-dir "$TEST_DIR" view
# 필요한 경우에만 출발 위치에서 보정
./ksrc --state-dir "$TEST_DIR" anchor
```

```bash
./ksrc --state-dir "$TEST_DIR" start1
```

```bash
./ksrc --state-dir "$TEST_DIR" start2
```

```bash
./ksrc --state-dir "$TEST_DIR" start3
```

```bash
./ksrc --state-dir "$TEST_DIR" start4
```

## 7. 정지·종료·로그 회수

먼저 GUI Space/E-STOP으로 TELEOP과 실제 정지를 확인한다. GUI 대신 **[Pi SSH]**에서 정지 요청할 수도 있다.

```bash
./ksrc --state-dir "$TEST_DIR" estop
```

전체 종료는 **[Pi SSH]**에서 실행한다.

```bash
./ksrc --state-dir "$TEST_DIR" stop
cat "$TEST_DIR/stopped.json"
ls -lh "$TEST_DIR/policy.npz"
```

`stopped owned processes`, `stopped=true`, 정책 기록 저장을 확인한다. 이후 GUI Esc 또는 launcher 터미널 Ctrl+C로 중계·조이스틱을 종료한다. `estop`은 UDP 소프트웨어 정지이며 `stop`은 세션 프로세스 종료다.

**[노트북 로컬]** Pi 로그를 가져온다.

```bash
mkdir -p ~/ksrc-field-logs
rsync -a -e 'ssh -o HostKeyAlias=raspberrypi.local' \
  pi@192.168.0.101:/home/pi/ksrc_final_20261022/deployment/ksrc/state/ ~/ksrc-field-logs/
```

노트북 중계 원자료는 `~/rover-mapping/outputs/broadcast_sessions/`에 저장된다. 이 폴더도 필요하면 별도 보관한다.

## 8. 상태 관찰·재접속·부하 기록 — 선택

별도 Pi SSH 터미널 또는 재접속한 터미널에서 기존 세션을 읽는다. 새 `start`를 호출하지 않는다. 터미널마다 변수는 다시 설정한다.

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="$(cat state/latest_test.txt)"
./ksrc --state-dir "$TEST_DIR" view --watch
```

Ctrl+C는 보기만 끝낸다. 원시 JSON은 `status`, 정책 로그는 `tail -f "$TEST_DIR/policy.log"`로 확인한다. 종료된 세션의 과거 AUTO 기록을 현재 상태로 읽지 않는다.

부하·온도 기록은 **[Pi SSH]**에서 스택 시작 성공 후 선택적으로 실행한다. 같은 세션에 수집기를 중복 실행하지 않는다.

```bash
nohup bash record_load.sh "$TEST_DIR" 600 > "$TEST_DIR/load_collector.log" 2>&1 < /dev/null &
```

수집 시간 `600`은 초 단위다. 종료 후 요약과 수집 완료를 확인한다.

```bash
./ksrc --state-dir "$TEST_DIR" report | tee "$TEST_DIR/summary.txt"
cat "$TEST_DIR/load/finished.txt"
```

스택 종료와 부하 수집 종료 시점은 다를 수 있다. 판정 기준·S0/S1 비교는 [TESTS_BEFORE_FIELD.md](TESTS_BEFORE_FIELD.md), [FIELD_TESTS.md](FIELD_TESTS.md)를 참고한다.

## 9. 로버 없이 조이스틱 시뮬레이션·화면 시연

### 조이스틱으로 RUN1~4 시험

**[노트북 로컬]** 최신 의존성을 준비한 뒤 실행한다. Pi·VPN·ROS·SSH 없이 로컬에서만 동작한다. 실기 launcher 또는 다른 simulator와 동시에 실행하지 않는다.

```bash
cd ~/rover-mapping
.venv-broadcast/bin/python -m pip install -r deployment/broadcast/requirements.txt
.venv-broadcast/bin/python deployment/broadcast/launch.py --sim
```

브라우저 `http://127.0.0.1:8765`와 실제 조이스틱 GUI가 열린다. **가상 로버는 RUN 출발점 `(3.60, 2.82), −y`에서 시작**한다. 정지 3초 후 START RUN1~4 중 하나를 누른다. 수동 이동·Shift·Space/E-STOP도 동일하게 시험한다. 회차가 끝나면 TELEOP으로 시작 위치·선택 ANCHOR 방향을 맞추고 3초 정지한 뒤 다음 회차를 시작한다. 처음부터 다시 배치하려면 E-STOP 후 launcher를 종료하고 `--sim`으로 다시 켠다.

이 모드는 배포 MPPI·상태기·gateway를 사용하고 사이트에 `KINEMATIC SIMULATION`을 표시한다. 위치는 이상적 SE2 운동이며, 수동 이동의 충돌·바퀴 접촉·모래·센서 오차·실기 통신을 재현하지 않는다. ORB는 정상으로 가정하며 ORB RESET 시험은 포함하지 않는다. 시뮬 ANCHOR는 출발점15cm 이내·선택 방향8° 이내에서3초 정지했을 때만 적용한다. 가상 로버를 강제로 회전시키지 않으므로 화살표 회전 키로 먼저 맞춘다.

4회차 자동 연결 검증을 다시 실행하려면 아래를 쓴다. GUI 없이 로컬 UDP→HTTP로 시험하고 `outputs/run_variants/broadcast_missions.json`에 결과를 저장한다.

```bash
OPENBLAS_NUM_THREADS=1 .venv-broadcast/bin/python deployment/broadcast/verify_missions.py
```

### 바구니 연결 구간 재생

**[노트북 로컬]** 실기·조이스틱 simulator와 동시에 실행하지 않는다.

```bash
cd ~/rover-mapping
.venv-broadcast/bin/python deployment/broadcast/launch.py --demo
```

사이트의 **▶ 시뮬 재생**을 누르면 바구니→RUN 진입 구간 약 36초를 재생한다. 다시 누르면 처음부터, STOP은 재생을 중단한다. RUN1~4 전체 주행 영상 재생은 아니다.

실기 중계는 pose·속도·궤적을 표시한다. 속도는 pose 차분 기반 추정에 표시 평활화만 적용하며 임의 노이즈는 넣지 않는다. AUTO의 사전 MPPI/슬립 캐시는 **바구니→RUN 진입과 RUN1~4 본 주행**을 지원한다. 본 주행은 회차·주행 단계와 위치·방향·속도 범위가 일치할 때만 표시한다. 회전·관찰 등 계획이 없는 단계, stale, 캐시 범위 밖에서는 예측을 숨기고 pose 기록은 계속한다. 슬립은 미교정 모델 추정값이며 실측 바퀴 슬립이 아니다. TELEOP에서도 이동 중에는 노트북의 **읽기 전용 MPPI 보조 예측**과 슬립 모델을 표시한다. 현재 pose·전후/좌우 속도·요속도에서 최대30cm/3초 원호를 국소 목표로 계산하며, 미션 목표나 제어 명령으로 사용하지 않는다. 사이트에는 `TELEOP MODEL`로 구분한다. 정지·순수 제자리 회전·stale·지도 밖·안전한 계획 불가 시 예측을 숨긴다. 시뮬 GROUND SPEED가 일정한 것은 물리 저항 없이 명령대로 이동하기 때문이며 표시 노이즈는 없다. 상세 범위는 [중계 배포 안내](deployment/broadcast/README.md).

## 10. 문제 해결

| 화면/증상 | 확인·조치 |
|---|---|
| `GATEWAY: no status` | Pi 스택·IP·UDP **5006** 확인. 조이스틱 단독 실행 시 `--gateway`와 `:5006` 모두 필요 |
| GUI는 연결됐지만 중계 pose가 안 옴 | 2단계의 BatchMode SSH 확인, launcher의 relay 로그·`latest_test.txt`·상태 폴더 확인 |
| `run=0`, 대기 | 아직 회차 미시작. 센서·출발 정렬·3초 정지 조건 확인 후 START |
| `NOT READY` / `start_rejected` | 표시된 정지 시간·ORB 추적·출발 위치/방향 조건 해결. 실제 배치가 틀린 상태를 ANCHOR로 덮지 않기 |
| `ORB not tracking` 지속 | 정지 후 특징점 있는 곳을 보게 하기. 10초 넘는 프레임 공백은 자동 ORB 재시작 대상이며 이후 재시작 대기 시간이 있음. 계속 안 풀리면 TELEOP에서 ORB RESET, `orb.log`·`supervisor.log` 확인 |
| `복구 중` | 센서 공백/ORB 원점 재연결. 명령 0 유지 확인하고 로버를 옮기지 않기 |
| AUTO 중 출발좌표 미정렬 | 출발점에서 이동한 결과일 수 있음. 현재 단계·복구·명령으로 판단 |
| 다른 세션 실행 중 / lock 오류 | `ps -eo pid,args`에서 `_supervise`의 `--state-dir` 확인 → 해당 세션 `view` → `estop` → `stop`. 실패한 시작 시도의 `latest_test.txt`가 현재 실행 세션을 가리키지 않을 수 있음 |
| `TEST_DIR`를 잊음 | Pi 배포 폴더에서 `TEST_DIR="$(cat state/latest_test.txt)"` 복원. 다른 세션이 실행 중이면 위 방법으로 확인 |
| 화면이 멈춘 듯함 | `view --watch`·`tail -f`는 Ctrl+C로 보기만 종료. E-STOP과 별개 |
| HTTP/UDP 포트 사용 중 | 이전 launcher를 종료. 실기·demo·조이스틱을 중복 실행하지 않기 |

## 검증 범위

기존 Pi의 FAKE 연결·plan 분리·정상 종료·정책 기록 확인과 이번 노트북 검증을 구분한다. 이번 RUN3/4 관련 테스트는 **168개 통과**했고, 15° 하행 변경 후 배포 MPPI+점기구학 시험은 RUN3 189.8초/RUN4 164.5초(시뮬 시간)에 복귀 완료했다. 바퀴 접촉·모래·센서 오차를 포함한 물리 시험이나 실제 모터 주행·현장 완주 검증은 아니다. 이번 변경을 실제 Pi에 업데이트해 실행한 검증도 아직 하지 않았다.

네 출발 방향×RUN1~4의16조합은 기구학 UDP→HTTP 시험에서 모두 완료됐다. AUTO 캐시는 −Y·−X 두 기준 주행의 world pose를 사용하므로 +Y·+X 초기 회전은 일치하는 예측이 없을 수 있다. 이때 추정값을 채워 넣지 않고 숨긴다. TELEOP은 별도 읽기 전용 모델을 사용한다.
