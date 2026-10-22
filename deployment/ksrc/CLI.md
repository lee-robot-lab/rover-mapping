# 현장 CLI 명령어

인터넷 없이 같은 AP에 연결한 노트북에서 SSH로 실행한다. 최종 운용 선택은 **S1: MPPI plan 분리**(`--async-plan`)다. 초기 배치 그림·설정 설명은 [README](README.md), 시험 순서·합격 기준은 [현장 시험 절차서](../../docs/experiments/2026-10-22-pi-navigation-field-tests.md)를 참고한다.

## 1. 접속과 점검

```bash
# 노트북: 공유기에서 확인한 실제 Pi IP로 변경
PI_AP_IP='192.168.50.2'
ssh pi@"$PI_AP_IP"

# 이하 Pi SSH 터미널
cd /home/pi/ksrc_final_20261022/deployment/ksrc
./ksrc check
```

예시 IP를 그대로 사용하지 않는다. 현장에서 pip/apt 설치나 git pull은 필요 없다.

## 2. 모터 없는 명령 연결 확인

```bash
./ksrc start --async-plan --upper-cpu 0 --lower-cpu 3 --seconds 120
./ksrc status
# orb_ok=true, stationary_s>=3, in_region1=true 확인 후
./ksrc start1
./ksrc status
./ksrc estop
./ksrc status
./ksrc stop
cat state/stopped.json
```

`--real`이 없으면 FAKE이며 SPI를 열지 않는다. 120초가 지나면 세션은 자동 종료한다. 이 명령 연결 시험은 Pi에서 직접 확인했으며 실제 주행 검증과 구분한다.

## 3. 실기 실행 — S1

실기 명령은 모터를 제어한다. 시뮬 검증 후 바퀴를 띄운 시험부터 진행한다. 기존 SPI 제어 프로세스는 먼저 종료하고 세션은 하나만 실행한다.

```bash
./ksrc start --real --async-plan --orb-cpus 1,2 --upper-cpu 0 --lower-cpu 3 --rt 50
./ksrc status
```

로버 **base_link 중심을 site (3.60, 2.82) m**, 정면을 **site −y, yaw −90°**로 맞춘다. 정면은 body +x이지만 site 좌표에서는 −y다. 실제 센서가 정상이고 3초 이상 정지한 뒤 기준을 지정한다.

```bash
# TELEOP 모드에서만: body 기준 전진 0.03 m/s, 1초 후 0 명령
./ksrc teleop 0.03 0 0 --seconds 1

# 실제 출발 위치와 방향으로 정렬하고 3초 이상 정지한 뒤
./ksrc anchor
./ksrc status

# 1회차 MPPI 시작: 30° 시도·후퇴 → 15° 등판 → 정상 관찰 → 7° 하행·복귀
./ksrc start1
./ksrc status
```

`anchor`는 로버를 움직이지 않는다. `status`에서 기준 적용과 준비 상태를 확인한다. AUTO 중 `teleop` 명령은 무시된다.

## 4. AUTO 중단·TELEOP 전환·2회차

```bash
# AUTO 취소 + 0 명령 + TELEOP 복귀
./ksrc estop
./ksrc status

# E-stop 후 최소 1초 기다린 뒤 필요한 경우에만 수동 이동
./ksrc teleop 0.03 0 0 --seconds 1

# 출발 위치·방향으로 다시 정렬하고 3초 이상 정지
./ksrc anchor
./ksrc status

# 2회차 MPPI: 30°·정상 관찰 생략, 하행 속도 0.7배
./ksrc start2
./ksrc status
```

E-stop 이후 센서가 복구돼도 AUTO는 자동 재개하지 않는다. `estop`은 UDP 소프트웨어 정지다. AUTO는 SSH가 끊겨도 계속되므로 현장에는 물리 E-stop 담당자를 둔다.

## 5. 종료와 로그

```bash
./ksrc estop
./ksrc stop
cat state/stopped.json
./ksrc logs

# 실행 중 로그 관찰은 별도 SSH 터미널에서
tail -f state/navigation.jsonl
tail -f state/policy.log
```

`tail -f`에서 Ctrl+C는 로그 표시만 끝낸다. 실행 종료는 `./ksrc stop`으로 한다.

```bash
# 노트북에서 로그 회수
PI_AP_IP='192.168.50.2'  # 실제 Pi IP
rsync -a pi@"$PI_AP_IP":/home/pi/ksrc_final_20261022/deployment/ksrc/state/ ./ksrc-field-logs/
```

시험별 로그를 분리하려면 모든 명령에 같은 `--state-dir`를 하위 명령 앞에 붙인다.

```bash
./ksrc --state-dir state/S1_01 start --real --async-plan --orb-cpus 1,2 --upper-cpu 0 --lower-cpu 3 --rt 50
./ksrc --state-dir state/S1_01 status
./ksrc --state-dir state/S1_01 anchor
./ksrc --state-dir state/S1_01 start1
./ksrc --state-dir state/S1_01 estop
./ksrc --state-dir state/S1_01 stop
```

## 검증 범위

Pi에서 `check/configure/start/status/start1/start2/estop/stop`의 FAKE 연결과 plan 분리 실행을 확인했다. 위 `--real` 명령으로 실제 모터 주행을 실행한 기록은 없다. 기존 S0/S1 비교 실험은 실제 초기 yaw가 0°(+x)였으므로 의도한 −y 출발 검증과 구분한다. −y 초기화 수정의 실제 시뮬 확인은 별도로 진행한다.
