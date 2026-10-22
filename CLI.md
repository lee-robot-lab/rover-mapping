# 현장 CLI — 노트북과 Pi 구분

**선택: S1 plan 분리**(`--async-plan`). 출발 전 모터 없는 시험부터 진행하려면 [TESTS_BEFORE_FIELD.md](TESTS_BEFORE_FIELD.md)를 먼저 따른다. 초기 배치·보정은 [배포 설정 안내](deployment/ksrc/README.md), 실제 시험 기준은 [FIELD_TESTS.md](FIELD_TESTS.md)에 있다.

## 0. 터미널 역할

| 터미널 | 실행 위치 | 용도 |
|---|---|---|
| **A** | 노트북에서 SSH 접속한 **Pi** | 실행·텔레옵·MPPI·E-stop·종료 |
| **B** | 노트북에서 별도로 SSH 접속한 **Pi** | 로그·상태 관찰 |
| **C** | **노트북 로컬**, SSH하지 않음 | 로그 다운로드 |

명령 블록 위의 위치를 확인하고 실행한다. A/B/C 변수는 공유되지 않는다. Pi에서 사용하는 `./ksrc`와 로그 수집기를 노트북 로컬에서 실행하지 않는다. 서버·VPN·인터넷은 현장 실행에 필요 없다.

## 1. 같은 AP에 연결하고 Pi에 SSH 접속

**[터미널 A · 노트북 로컬]** — IP를 공유기에서 확인한 실제 Pi 주소로 변경한다.

```bash
PI_AP_IP='192.168.50.2'
ping -c 3 "$PI_AP_IP"
ssh pi@"$PI_AP_IP"
```

**[터미널 A · Pi SSH, 접속 후]**

```bash
hostname
cd /home/pi/ksrc_final_20261022/deployment/ksrc
./ksrc check
```

`check` 실패 시 먼저 원인을 해결한다. 현장에서 pip/apt 설치·git pull은 필요 없다. `192.168.50.2`는 예시다.

## 2. 실기 세션과 로그 폴더 만들기

이후는 **실제 SPI·모터 제어**를 여는 명령이다. 시뮬 검증 후 바퀴를 띄운 시험부터 진행하고, 기존 직접 SPI 제어는 종료한다. 한 번에 세션 하나만 실행한다. 모터 없는 확인은 [사전 테스트 설명서](TESTS_BEFORE_FIELD.md)를 따른다.

**[터미널 A · Pi SSH]**

```bash
TEST_DIR="state/field_S1_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
printf '%s\n' "$TEST_DIR" > state/latest_test.txt
date -Is | tee "$TEST_DIR/actions.log"
./ksrc check 2>&1 | tee "$TEST_DIR/check.log"
./ksrc --state-dir "$TEST_DIR" start --real --async-plan --orb-cpus 1,2 --upper-cpu 0 --lower-cpu 3 --rt 50
nohup bash record_load.sh "$TEST_DIR" 600 > "$TEST_DIR/load_collector.log" 2>&1 < /dev/null &
echo "collector_pid=$!" | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
```

세션은 기본 최대 1시간, 부하 수집은 10분이다. 수집 시간을 늘리려면 실행 전에 `600`을 초 단위로 바꾼다(최대 3600초). 같은 폴더에서 수집기를 중복 실행하지 않는다. 명령 직후 상태 반영에는 한 차례 갱신 시간이 필요하므로 `status`와 B의 로그에서 수락 여부를 확인한다.

## 3. 관찰용 터미널 연결

**[터미널 B · 노트북 로컬]**

```bash
PI_AP_IP='192.168.50.2'
ssh pi@"$PI_AP_IP"
```

**[터미널 B · Pi SSH, 접속 후]**

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="$(cat state/latest_test.txt)"
printf '관찰할 폴더: %s\n' "$TEST_DIR"
tail -f "$TEST_DIR/navigation.jsonl"
```

B는 로그 관찰용으로 유지한다. Ctrl+C는 로그 표시만 끝낸다. 다른 파일을 보려면 Ctrl+C 후 다음 중 하나를 실행한다.

**[터미널 B · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" status
tail -f "$TEST_DIR/policy.log"
```

`tail -f` 뒤의 명령은 Ctrl+C로 빠져나와야 입력할 수 있다.

## 4. 출발 위치 정렬·anchor

로버 **base_link 중심을 site (3.60, 2.82) m**, 정면을 **site −y, yaw −90°**로 맞춘다. 정면은 body +x이며 site 좌표의 −y다. 15° 통로 진입 방향이다. 배치 그림은 [초기 위치 안내](deployment/ksrc/README.md#1-초기-위치와-방향)를 참고한다.

**[터미널 A · Pi SSH]** — 필요한 경우 TELEOP 상태에서만 짧게 조작한다.

```bash
# body 기준 전진 0.03 m/s, 1초 후 0 명령. 실제 로버가 움직인다.
./ksrc --state-dir "$TEST_DIR" teleop 0.03 0 0 --seconds 1
```

위치·방향을 맞추고 **3초 이상 정지**한다. 센서가 정상일 때 아래 명령을 실행한다.

**[터미널 A · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" anchor
```

B에서 `operator_anchor` 이벤트를 확인한다. `anchor`는 로버를 움직이지 않는다. 적용 확인 후 준비 상태를 조회한다.

**[터미널 A · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
```

`orb_ok=true`, `stationary_s>=3`, `in_region1=true`, `start_aligned=true`, 시작 전 `active=false`·명령 0인지 확인한다. 실제 위치가 틀린 상태를 anchor로 덮어놓고 출발하지 않는다.

## 5. 1회차 MPPI 시작

**[터미널 A · Pi SSH]**

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" start1 | tee -a "$TEST_DIR/actions.log"
```

B에서 `active=true`, `run=1`, 단계·명령을 확인한다. 30° 시도·후퇴 → 15° 등판 → 정상 관찰 → 7° 하행·복귀 순서다. AUTO 중 텔레옵 입력은 무시된다. START 거부 시 원인을 확인하고, 반복 START를 성공으로 간주하지 않는다.

## 6. E-stop·TELEOP 복귀

**[터미널 A · Pi SSH]**

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" estop | tee -a "$TEST_DIR/actions.log"
```

B에서 `active=false`, 명령 0을 확인한다. 최소 1초 기다린 후 필요한 경우 수동 조작한다.

**[터미널 A · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" teleop 0.03 0 0 --seconds 1
```

E-stop 후 센서가 복구돼도 AUTO는 자동 재개하지 않는다. `estop`은 UDP 소프트웨어 정지다. AUTO는 SSH가 끊겨도 계속되므로 현장에는 물리 E-stop 담당자를 둔다.

## 7. 2회차 시작

실제 출발 위치·−y 방향으로 다시 정렬하고 3초 이상 정지한 뒤 진행한다.

**[터미널 A · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" anchor
```

B에서 적용 이벤트를 확인하고 A에서 준비 상태를 확인한다.

**[터미널 A · Pi SSH]**

```bash
./ksrc --state-dir "$TEST_DIR" status | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" start2
```

B에서 `active=true`, `run=2`, `stage=ascent15`를 확인한다. 30°·정상 관찰은 생략하고 하행은 속도 0.7배다.

## 8. 전체 종료·저장 확인

**[터미널 A · Pi SSH]**

```bash
date -Is | tee -a "$TEST_DIR/actions.log"
./ksrc --state-dir "$TEST_DIR" estop
./ksrc --state-dir "$TEST_DIR" stop
cat "$TEST_DIR/stopped.json" | tee -a "$TEST_DIR/actions.log"
tail -n 20 "$TEST_DIR/policy.log"
ls -lh "$TEST_DIR/policy.npz"
```

`stopped=true`와 정책 NPZ 저장을 확인한다. 정책 주기 중앙값·p99·최대는 `policy.log` 끝에도 나온다. 수집기는 지정 시간까지 별도로 동작한다. 수집 완료 확인은 B에서 Ctrl+C 후 실행한다.

**[터미널 B · Pi SSH]**

```bash
cat "$TEST_DIR/load/finished.txt"
```

아직 없으면 지정한 수집 시간이 지났는지 확인하고 `load_collector.log`를 본다. 부하 값의 해석은 [사전 테스트 설명서](TESTS_BEFORE_FIELD.md#저장되는-값과-읽는-법)를 참고한다.

## 9. 로그 다운로드

**[터미널 C · 노트북 로컬, SSH하지 않은 창]**

```bash
PI_AP_IP='192.168.50.2'
mkdir -p ~/ksrc-field-logs
rsync -a pi@"$PI_AP_IP":/home/pi/ksrc_final_20261022/deployment/ksrc/state/ ~/ksrc-field-logs/
ls -lt ~/ksrc-field-logs/
```

Pi SSH 안에서 실행하지 않는다. 결과는 노트북의 `~/ksrc-field-logs/`다.

## SSH 재접속·다음 시험

재접속은 A/B의 **노트북 로컬**에서 `ssh pi@<실제 Pi IP>`로 진행한다. 접속 후 아래는 **Pi SSH**에서 실행한다. 새 `start`를 호출하지 않고 기존 세션에 연결한다.

**[터미널 A 또는 B · Pi SSH, 재접속 후]**

```bash
cd /home/pi/ksrc_final_20261022/deployment/ksrc
TEST_DIR="$(cat state/latest_test.txt)"
./ksrc --state-dir "$TEST_DIR" status
```

새 시험은 기존 세션 종료 후 2단계부터 시작한다. B도 새 `latest_test.txt`를 읽어야 하며, 이전 `TEST_DIR` 변수가 자동 갱신되지는 않는다.

## 검증 범위

Pi에서 FAKE 연결·plan 분리·정상 종료·정책 주기 NPZ·부하 기록을 확인했다. 실제 모터 주행은 미검증이다. 기존 S0/S1 비교는 실제 yaw 0°(+x) 출발이었으며 의도한 −y 출발 검증과 구분한다.
