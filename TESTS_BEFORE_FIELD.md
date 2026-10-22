# 가기 전 테스트와 부하·로그 기록

실행 명령은 [CLI.md](CLI.md), 실제 주행 합격 기준은 [FIELD_TESTS.md](FIELD_TESTS.md)에 있다. **선택한 운용은 S1 plan 분리**다. 시험마다 다른 폴더에 기록하고 한 번에 세션 하나만 실행한다. 아래 명령은 별도 표시가 없으면 Pi SSH 터미널에서 실행한다.

## 무엇부터 해볼지

| 순서 | 시험·시간 | 지금 확인할 것 | 남길 기록 |
|---|---|---|---|
| A | AP 연결·점검, 2분 | 인터넷 없이 SSH·`check` 성공, 저장 공간 확보 | `check.log`, `load/meta.txt` |
| B | FAKE S1, 3분 | 시작 전 0 → `start1` AUTO → E-stop 0 → 재정렬 조건 후 `start2` | `actions.log`, `navigation.jsonl`, `policy.npz` |
| C | FAKE SSH 단절, 1분 | SSH 종료 후 재접속해 같은 세션의 로그·AUTO 유지 확인 | 재접속 시각, status 전후, 부하 기록 연속성 |
| D | 실제 스택 정지 관찰, 10분 | 실제 camera·ORB·IMU와 정책을 함께 켰을 때 코어 포화·온도·센서 공백 | `load/` 전체, `policy.npz`, `orb_*`, `navigation.jsonl` |
| E | 실제 ORB 손실·재시작 | 바퀴를 띄운 상태에서 복구 중 0, 마지막 pose·단계 유지, E-stop 후 자동 재개 없음 | 복구 전후 status, 이벤트·PID, 영상 |
| F | 바퀴를 띄운 전환·정지 | TELEOP→AUTO, AUTO→E-stop→TELEOP, 실제 바퀴 정지 | 명령·바퀴 정지 시각, 영상 |

A~C는 모터 없는 시험이다. D~F는 `--real`로 SPI·모터 제어가 열리므로 **로버를 안정적으로 고정하고 바퀴를 띄운 상태**에서 진행한다. 실제 초기 배치 `(3.60, 2.82) m / yaw −90°(site −y)`와 다른 위치에서는 AUTO를 시작하지 않는다. 정지 상태 D는 START 없이 센서와 정책 부하만 측정한다. 시뮬 검증 후 실기 검증 순서로 진행하고, 전체 코스 주행은 [현장 절차](FIELD_TESTS.md)를 따른다.

FAKE는 실제 camera·ORB가 없으므로 부하 여유나 실제 완주를 판단하는 데 쓰지 않는다. D도 정지 관찰이라 이동 중 영상·MPPI 계산 부하까지 대표하지 않는다. 기존 S0/S1 비교는 +x 출발이었으며 −y 출발 검증과 구분한다.

## 시험 폴더와 작업 기록

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="state/precheck_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
printf '%s\n' "$TEST_DIR" > state/latest_test.txt
date -Is | tee "$TEST_DIR/actions.log"
./ksrc check 2>&1 | tee "$TEST_DIR/check.log"
```

다른 SSH 터미널이나 재접속 후에는 다음으로 동일 폴더를 복원한다. 이후 모든 `./ksrc` 명령에 `--state-dir "$TEST_DIR"`를 붙인다.

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="$(cat state/latest_test.txt)"
```

## FAKE 전환·정지 시험

```bash
./ksrc --state-dir "$TEST_DIR" start --async-plan --upper-cpu 0 --lower-cpu 3 --rt 50 --seconds 600
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"

# orb_ok=true, stationary_s>=3, in_region1=true를 확인한 뒤
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" start1 | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"

./ksrc --state-dir "$TEST_DIR" estop | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
# active=false, command=[0,0,0] 확인. 최소 1초 후 TELEOP 확인
./ksrc --state-dir "$TEST_DIR" teleop 0.03 0 0 --seconds 1

# 정지 3초·센서·출발 조건 확인 후 2회차
./ksrc --state-dir "$TEST_DIR" anchor
./ksrc --state-dir "$TEST_DIR" start2
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
```

합성 입력은 실제 바퀴 위치와 무관하다. `anchor` 후 status에서 `active`·`reason`·`stage`를 확인하고 START 거부를 성공으로 기록하지 않는다. SSH 단절 시험은 AUTO 상태에서 SSH만 `exit` 후 재접속해 동일 폴더의 status와 로그 증가를 확인한다. 이때 `stop`은 호출하지 않는다. 실제 주행 중 AP 단절 시험은 물리 E-stop 담당자와 바퀴를 띄운 시험부터 진행한다.

## CPU·메모리·온도 기록 — SSH가 끊겨도 계속

시험 세션 시작 후 아래 명령을 실행한다. 설치 없이 Pi의 `top`, `vmstat`, `vcgencmd`를 사용한다. 기본 10분이고 마지막 숫자로 초를 지정한다(최대 3600초). 같은 폴더에서 수집기를 중복 실행하면 기존 부하 파일을 덮어쓰므로 시험마다 새 폴더를 사용한다.

```bash
nohup bash record_load.sh "$TEST_DIR" 600 > "$TEST_DIR/load_collector.log" 2>&1 < /dev/null &
echo "collector_pid=$!" | tee -a "$TEST_DIR/actions.log"
```

| 파일 | 의미·해석 |
|---|---|
| `load/top_threads.log` | 1초 간격 전체 코어·스레드 부하와 메모리. runner·MPPI worker·policy·ORB·camera를 PID/PPID로 매칭 |
| `load/processes.log` | 5초 간격 PID·PPID·스레드 ID·현재 CPU·스케줄 정책·RT 우선순위·RSS. `ps`의 CPU%는 생존 기간 평균 |
| `load/vmstat.log` | 시스템 메모리·swap·실행 대기·CPU idle·I/O wait. 첫 데이터 행은 부팅 이후 평균이므로 제외 |
| `load/thermal.log` | 5초 간격 온도(m°C), throttle 비트, ARM clock. `vcgencmd`가 없는 PC는 해당 값 생략 |
| `load/meta.txt` | 시각·OS·코어 수·가용 디스크·시작 시 소유 프로세스 명령 |
| `load/finished.txt` | 수집기가 지정 시간까지 정상 완료했을 때 생성 |

`top` 첫 프레임도 분석에서 제외한다. CPU%는 스레드별 **100%=코어 하나 사용** 기준으로 보고, 코어별 idle도 함께 본다. 전체 CPU 평균만 낮다고 여유가 있는 것은 아니다. S1에서 runner와 MPPI worker는 둘 다 CPU 0에 배치된다. PID·PPID로 구분한다. ORB 1·2 / 하위 정책 3, 정책 RT 50 적용 여부도 확인한다. 수집기 자체의 부하도 포함되므로 비교 때 동일하게 켜 둔다.

`throttled=0x0`인지 확인한다. 현재/과거 저전압·열 제한 플래그가 나타나면 전원·냉각 조건을 함께 기록한다. 과거 비트는 시험 전에 이미 있었는지 전후를 비교한다. 기준 온도·부하율 하나만으로 합격 처리하지 말고 센서 공백·정지·주기 지연과 연결해서 판단한다.

## 실제 스택 부하 시험

FAKE 세션을 `estop`·`stop`으로 종료한 뒤 **새 시험 폴더**를 만든다. 시작 명령만 다음으로 바꾸고 부하 수집기를 똑같이 실행한다. START 없이 10분 관찰한다. 기존 `local/real.json`·`local/orb.yaml` 보정이 필요하다([설정 안내](deployment/ksrc/README.md)).

```bash
./ksrc --state-dir "$TEST_DIR" start --real --async-plan --orb-cpus 1,2 --upper-cpu 0 --lower-cpu 3 --rt 50 --seconds 660
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
nohup bash record_load.sh "$TEST_DIR" 600 > "$TEST_DIR/load_collector.log" 2>&1 < /dev/null &
```

`active=false`와 명령 0을 유지하는지, ORB가 정상 추적하는지 확인한다. ORB 크래시 주입 명령은 [FIELD_TESTS.md](FIELD_TESTS.md#orb-자동-재시작-시험의-명령)에 있다. 그 예시의 `state/T09`를 이번 실제 `TEST_DIR`로 바꿔서 실행한다. 공유 프로세스 전체에 `pkill`을 사용하지 않는다.

## 종료·정책 주기 기록·로그 회수

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" estop
./ksrc --state-dir "$TEST_DIR" stop
cat "$TEST_DIR/stopped.json" | tee -a "$TEST_DIR/actions.log"
tail -n 20 "$TEST_DIR/policy.log"
ls -lh "$TEST_DIR/policy.npz" "$TEST_DIR/load/finished.txt"
```

최신 실행기는 정책에 `--log <시험폴더>/policy.npz`를 전달한다. 정상 종료 시 `loop_t`, **`period_ms`(하위 제어 주기)**, **`compute_ms`(한 주기 계산 시간)**와 관측·행동·telemetry를 저장한다. `policy.log` 끝에도 주기의 중앙값·p99·최대·25 ms 초과 횟수가 나온다. 강제 종료·전원 차단이면 NPZ가 남지 않을 수 있으므로 정상 `stop` 후 생성 여부를 확인한다. FAKE에서 policy 파일이 없는 PC는 이 파일이 생성되지 않는다.

수집기가 아직 지정 시간만큼 돌지 않았다면 `finished.txt`는 없다. 남은 시간까지 기다린 뒤 회수한다. `navigation.jsonl`은 5 Hz 상태와 복구 이벤트 기록이다. `pose_age_s`, `imu_age_s`, `recovering`, `stage`, `reason`으로 센서 지연·공백을 확인하지만, **50 Hz 상위 tick p99나 MPPI 계산 시간 전체를 측정하는 로그는 아니다.** 그 지표의 정밀 계측은 별도 실험이 필요하다.

```bash
# 노트북: 실제 AP의 Pi IP로 변경
PI_AP_IP='192.168.50.2'
rsync -a pi@"$PI_AP_IP":/home/pi/ksrc_final_20261022/deployment/ksrc/state/ ./ksrc-precheck-logs/
```

## 결과를 적어 둘 표

| 항목 | 결과 |
|---|---|
| 날짜·배포 commit·FAKE/REAL·시험 폴더 | |
| 시험 조작 시각·전환·E-stop 결과 | |
| CPU 0/1/2/3 평균·최대 부하, runner/worker/ORB/policy 부하 | |
| 메모리 RSS·swap·I/O wait 변화 | |
| 시작/최대/종료 온도·throttle 전후 | |
| 하위 period 중앙값/p99/최대·25 ms 초과 횟수 | |
| pose/IMU 공백·복구 횟수·복구 시간 | |
| SSH 재접속 후 같은 세션 유지 여부 | |
| 실제 바퀴 정지 확인·영상 경로 | |
| 실패 원인·남은 확인·회수한 로그 경로 | |

부하 표에는 측정 구간·샘플 수를 함께 적는다. 이전 Pi 실험의 하위 주기 p99 약 20.0~20.1 ms는 참고값이며 이번 장비 상태에서 다시 확인한다. 자동 복구 중 명령이 0이 아닌 경우나 E-stop 이후 AUTO가 되살아나는 경우는 다음 단계로 진행하지 않는다.

## 이번에 직접 확인한 기록 — 2026-10-22

- Pi FAKE S1·CPU 0/3·RT 50에서 `stop` 후 `policy.npz` 생성(44 KB)을 확인했다. 하위 주기 934개에서 p99 **20.01 ms**, 최대 **20.02 ms**, 25 ms 초과 **0회**였다. 짧은 FAKE 표본이며 실제 ORB·카메라·모터 부하 결과가 아니다.
- Pi에서 코어별·스레드별 `top`, `vmstat`, 온도·throttle·clock·PID 목록과 수집 완료 파일 생성을 확인했다. 12초 수집 구간의 온도 표본은 61.15→58.40→57.85°C, throttle은 모두 `0x0`이었다. 이 구간만으로 지속 부하·냉각 성능을 판단하지 않는다.
- FAKE 정책이 SIGTERM으로 즉시 끝나 로그를 빠뜨리는 경로를 수정했다. FAKE는 SIGINT로 정상 저장 후 종료하고, 기존 real wrapper는 SIGTERM 저장 경로를 유지한다. 종료·로그 전달 회귀를 포함한 navigation 테스트 **16개 통과**.
- Pi 원시 기록: `state/precheck_logverify_20261022/`(부하), `state/precheck_logverify2_20261022/`(정책 NPZ·주기·최종 수집기 확인). 세션은 모두 종료했다.
