# 가기 전 테스트 설명서 — 터미널별 순서

위에서부터 순서대로 진행한다. 실행 명령은 [CLI.md](CLI.md), 현장 합격 기준은 [FIELD_TESTS.md](FIELD_TESTS.md)에 있다. S1 plan 분리를 사용한다.

## 0. 노트북에서 터미널 3개 열기

| 터미널 | 명령이 실행되는 곳 | 역할 |
|---|---|---|
| **A — 제어** | 노트북에서 SSH 접속한 **Pi** | 시작·전환·E-stop·종료 |
| **B — 관찰** | 노트북에서 별도로 SSH 접속한 **Pi** | 상태·부하·로그 관찰 |
| **C — 회수** | **노트북 로컬**, SSH 접속하지 않음 | 마지막에 로그 다운로드 |

노트북과 Pi를 같은 AP에 연결한다. 아래 `192.168.50.2`는 예시이므로 **공유기에서 확인한 실제 Pi IP**로 바꾼다. 이 절차에 서버·VPN·인터넷은 필요 없다.

**[터미널 A · 노트북 로컬]**

```bash
PI_AP_IP='192.168.50.2'
ssh pi@"$PI_AP_IP"
```

**[터미널 B · 노트북 로컬]** — 별도 터미널이므로 IP 변수도 다시 지정한다.

```bash
PI_AP_IP='192.168.50.2'
ssh pi@"$PI_AP_IP"
```

A와 B는 이제 Pi에서 명령을 실행한다. `hostname`으로 Pi인지 확인할 수 있다. C는 SSH하지 않고 노트북에 둔다. 변수는 터미널 간 공유되지 않으므로 아래의 폴더 복원 명령도 각 터미널에서 실행한다.

## 1. 점검하고 FAKE 시험 폴더 만들기

**[터미널 A · Pi SSH]**

```bash
hostname
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="state/fake_precheck_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
printf '%s\n' "$TEST_DIR" > state/latest_test.txt
date -Is | tee "$TEST_DIR/actions.log"
./ksrc check 2>&1 | tee "$TEST_DIR/check.log"
df -h .
```

**확인:** `check`의 import·hash가 통과해야 한다. RT limit은 50 이상인지 확인한다. 오류가 있으면 여기서 멈추고 `check.log`를 확인한다. 이후 세션은 한 번에 하나만 실행한다.

## 2. 모터 없는 S1 실행과 부하 기록 시작

**[터미널 A · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" start --async-plan --upper-cpu 0 --lower-cpu 3 --rt 50 --seconds 600
nohup bash record_load.sh "$TEST_DIR" 180 > "$TEST_DIR/load_collector.log" 2>&1 < /dev/null &
echo "collector_pid=$!" | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
```

`--real`이 없으므로 모터·SPI를 열지 않는다. 부하 기록은 3분, 실행 세션은 최대 10분이다. 수집기를 같은 폴더에서 다시 실행하면 부하 파일을 덮어쓰므로 중복 실행하지 않는다.

**[터미널 B · Pi SSH]** — A에서 만든 시험 폴더를 불러온다.

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="$(cat state/latest_test.txt)"
printf '관찰할 폴더: %s\n' "$TEST_DIR"
tail -f "$TEST_DIR/navigation.jsonl"
```

**확인:** 시작 전 `active=false`, `command=[0,0,0]`. `orb_ok=true`, `stationary_s>=3`, `in_region1=true`, `start_aligned=true`가 될 때까지 기다린다. B의 `tail -f`는 계속 켜 둔다.

## 3. TELEOP → MPPI → E-stop 확인

**[터미널 A · Pi SSH]** — 아래 블록은 하나씩 실행하고 B에서 결과를 본다.

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" start1 | tee -a "$TEST_DIR/actions.log"
```

**확인:** B에서 `active=true`, `run=1`, `stage=entry30`와 MPPI 명령이 나와야 한다. `start_rejected`이면 START 성공으로 기록하지 말고 A에서 `status`로 준비 미달 이유를 확인한다.

**[터미널 A · Pi SSH]**

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" estop | tee -a "$TEST_DIR/actions.log"
```

**확인:** B의 다음 상태 갱신에서 `active=false`, 명령 0. 센서가 정상이어도 AUTO가 저절로 다시 켜지면 안 된다. 상태 로그는 5 Hz이므로 실제 정지 지연의 정밀 측정값은 아니다.

**[터미널 A · Pi SSH]** — E-stop 후 최소 1초 기다린 뒤 실행한다.

```bash
./ksrc --state-dir "$TEST_DIR" teleop 0.03 0 0 --seconds 1
```

FAKE라 실제 바퀴는 움직이지 않는다. 이 명령은 gateway에 1초간 전진 입력을 보내고 끝에 0을 보낸다. 실제 GUI 버튼과 바퀴 정지는 별도 실기 시험이다.

## 4. 정지 후 2회차 시작 확인

**[터미널 A · Pi SSH]** — 3초 이상 정지·정상 센서 조건을 확인한 뒤 진행한다.

```bash
./ksrc --state-dir "$TEST_DIR" anchor
```

**확인:** B에서 `operator_anchor` 기록이 나타나야 한다. `anchor`는 로버를 움직이는 명령이 아니다.

**[터미널 A · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" start2
```

**확인:** B에서 `active=true`, `run=2`, `stage=ascent15`. 합성 입력이므로 실제 등판·완주 성공을 의미하지 않는다.

## 5. FAKE에서 SSH 연결 끊었다가 재접속

**[터미널 A · Pi SSH]** — AUTO 상태에서 실행한다. 이때 `stop`은 실행하지 않는다.

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
exit
```

A만 노트북 로컬로 돌아온다. B는 그대로 Pi의 로그가 계속 증가하는지 관찰한다. 이것은 **SSH 한 세션 종료 시험**이며 AP 무선 단절 시험과 다르다.

**[터미널 A · 노트북 로컬]**

```bash
PI_AP_IP='192.168.50.2'
ssh pi@"$PI_AP_IP"
```

**[터미널 A · Pi SSH, 재접속 후]**

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="$(cat state/latest_test.txt)"
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
cat "$TEST_DIR/supervisor.json"
```

**확인:** 같은 시험 폴더·supervisor PID, 로그 시각 증가, AUTO 상태 유지. 최대 실행 시간 600초가 지난 뒤 확인하면 이미 자동 종료했을 수 있다.

## 6. 부하 로그 확인 후 FAKE 종료

**[터미널 B · Pi SSH]** — Ctrl+C로 `tail -f` 표시만 끝낸 뒤 실행한다.

```bash
tail -n 12 "$TEST_DIR/load/thermal.log"
tail -n 5 "$TEST_DIR/load/vmstat.log"
less "$TEST_DIR/load/top_threads.log"
```

`less`는 `q`로 나온다. 부하 수집 시작 후 3분이 지나면 아래 파일이 생긴다.

**[터미널 B · Pi SSH]**

```bash
cat "$TEST_DIR/load/finished.txt"
```

파일이 없으면 아직 수집 중일 수 있다. 3분이 지나도 없으면 `cat "$TEST_DIR/load_collector.log"`로 오류를 확인한다.

**[터미널 A · Pi SSH]**

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" estop
./ksrc --state-dir "$TEST_DIR" stop
cat "$TEST_DIR/stopped.json" | tee -a "$TEST_DIR/actions.log"
tail -n 20 "$TEST_DIR/policy.log"
ls -lh "$TEST_DIR/policy.npz"
```

**확인:** `stopped=true`, 정책 로그 끝의 주기 요약과 `policy.npz` 생성. 강제 종료·전원 차단이면 NPZ 저장이 빠질 수 있으므로 정상 `stop`을 사용한다. 정책 파일이 없는 노트북 단독 FAKE에서는 NPZ가 생성되지 않는다.

## 7. 선택: 실제 카메라·ORB·IMU를 켜고 10분 부하 확인

이 단계는 `--real`로 **실제 SPI·모터 제어가 열린다.** 시뮬 검증 후 로버를 안정적으로 고정하고 바퀴를 띄운 상태에서 진행한다. 기존 직접 SPI 제어 프로세스를 종료한다. FAKE 종료를 확인한 뒤 새 폴더를 만든다. 이 단계는 **START 없이 정지 관찰만** 한다.

**[터미널 A · Pi SSH]**

```bash
TEST_DIR="state/real_idle_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
printf '%s\n' "$TEST_DIR" > state/latest_test.txt
date -Is | tee "$TEST_DIR/actions.log"
./ksrc check 2>&1 | tee "$TEST_DIR/check.log"
./ksrc --state-dir "$TEST_DIR" start --real --async-plan --orb-cpus 1,2 --upper-cpu 0 --lower-cpu 3 --rt 50 --seconds 660
nohup bash record_load.sh "$TEST_DIR" 600 > "$TEST_DIR/load_collector.log" 2>&1 < /dev/null &
echo "collector_pid=$!" | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
```

실측 보정 `local/real.json`·`local/orb.yaml`이 필요하다([설정 안내](deployment/ksrc/README.md)).

**[터미널 B · Pi SSH]** — 새 폴더를 다시 불러온다. 이전 FAKE 폴더를 그대로 보지 않는다.

```bash
TEST_DIR="$(cat state/latest_test.txt)"
tail -f "$TEST_DIR/navigation.jsonl"
```

**확인:** `active=false`, 명령 0 유지, ORB 정상 추적·센서 공백·복구 횟수. 10분 뒤 B에서 Ctrl+C 후 `load/finished.txt`를 확인하고, A에서 **6단계의 종료 명령을 다시 실행**한다. 이 정지 시험은 실제 영상 입력 부하는 포함하지만 주행 중 영상·MPPI 계산 부하를 대표하지 않는다.

실제 ORB 크래시·추적 손실·바퀴 전환 시험은 [FIELD_TESTS.md](FIELD_TESTS.md)의 별도 절차를 따른다. 집/실험실에서 임의 위치를 `anchor`해 AUTO를 시작하지 않는다. 의도한 출발은 **base_link 중심 (3.60, 2.82) m / site −y, yaw −90°**다.

## 8. 노트북으로 로그 회수

**[터미널 C · 노트북 로컬, SSH 접속하지 않은 창]**

```bash
hostname
PI_AP_IP='192.168.50.2'
mkdir -p ~/ksrc-precheck-logs
rsync -a pi@"$PI_AP_IP":/home/pi/ksrc_final_20261022/deployment/ksrc/state/ ~/ksrc-precheck-logs/
ls -lt ~/ksrc-precheck-logs/
```

**확인:** 노트북의 `~/ksrc-precheck-logs/` 아래에 시험 폴더가 있어야 한다. Pi SSH 안에서 실행하면 다운로드 대상도 Pi가 되므로 C에서만 실행한다.

## 저장되는 값과 읽는 법

| 파일 | 기록·해석 |
|---|---|
| `check.log`, `actions.log` | 환경 점검, 조작 시각·status. 실제 바퀴 정지 시각은 영상으로 별도 기록 |
| `navigation.jsonl` | 5 Hz 상태·pose/IMU age·단계·복구 이벤트. 50 Hz 상위 tick p99·MPPI 계산 시간 전체를 측정하는 로그는 아님 |
| `policy.log`, `policy.npz` | 정상 종료 시 하위 `period_ms`, `compute_ms`, 관측·행동·telemetry. 중앙값·p99·최대·25 ms 초과 횟수 확인 |
| `load/top_threads.log` | 1초 간격 코어별·스레드별 CPU·메모리. 첫 프레임 제외 |
| `load/processes.log` | 5초 간격 PID·PPID·스레드·현재 CPU·RT 우선순위·RSS. `ps`의 CPU%는 생존 기간 평균 |
| `load/vmstat.log` | 메모리·swap·실행 대기·CPU idle·I/O wait. 첫 데이터 행은 부팅 이후 평균이므로 제외 |
| `load/thermal.log` | 온도(m°C)·throttle 비트·ARM clock. `throttled=0x0`인지 전후 비교 |
| `load/meta.txt`, `load/finished.txt` | 환경·실행 파일 hash·소유 PID, 수집 정상 완료 시각 |

CPU는 스레드 100%=코어 하나 기준으로 보고 코어별 idle도 확인한다. S1은 runner·MPPI worker가 CPU 0, ORB가 1·2, 정책이 3이다. runner·worker는 PID/PPID로 구분한다. 전체 평균 CPU가 낮아도 특정 코어는 포화될 수 있다. 수집기 부하도 포함되므로 비교 시 동일하게 켠다. FAKE 부하를 실제 ORB 여유로 해석하지 않는다.

## 시험 후 적을 결과

| 항목 | 결과 |
|---|---|
| 날짜·배포 commit·FAKE/REAL·시험 폴더 | |
| 시작·E-stop·재시작·SSH 재접속 결과 | |
| CPU 0/1/2/3·프로세스별 평균/최대·측정 구간 | |
| 메모리·swap·I/O wait 변화 | |
| 시작/최대/종료 온도·throttle 전후 | |
| 하위 주기 중앙/p99/최대·25 ms 초과·샘플 수 | |
| 센서 공백·복구 횟수·실패 stage/reason | |
| 노트북 로그 경로·실제 바퀴 정지 영상 | |

E-stop 후 AUTO가 되살아나거나 복구 중 자동 명령이 0이 아니면 다음 단계로 진행하지 않는다. GUI 조작·실제 바퀴 정지는 CLI FAKE 성공과 구분한다.

## 직접 확인한 기록 — 2026-10-22

Pi FAKE S1·CPU 0/3·RT 50에서 정상 `stop` 후 `policy.npz` 44 KB 생성. 하위 주기 934개에서 p99 **20.01 ms**, 최대 **20.02 ms**, 25 ms 초과 **0회**였다. 짧은 FAKE 표본으로 실기 부하 결과가 아니다. 부하 로그·완료 파일도 Pi에서 확인했다. 12초 수집 온도는 61.15→58.40→57.85°C, throttle은 `0x0`. FAKE 저장 종료 경로를 포함한 navigation 테스트 **16개 통과**. 원시 기록은 Pi `state/precheck_logverify_20261022/`, `state/precheck_logverify2_20261022/`이며 세션은 종료했다.

기존 S0/S1 시뮬 비교는 실제 +x 출발이었다. 의도한 −y 출발 검증과 구분한다.
