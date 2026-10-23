# 실기 리허설과 현재 시뮬레이션의 차이 — 2026-10-24

운용 확정: RUN1~4는 AUTO. 응급상황은 E-STOP으로 AUTO를 해제한 뒤 TELEOP으로 인계한다. 이 점은 현재 gateway 동작과 일치한다. 이 문서는 코드·Pi 백업 정적 대조 결과이며, 이번에 Pi·모터를 실행한 기록이 아니다.

## 이미 확인한 것과 확인하지 않은 것

- 네 ANCHOR 방향×RUN1~4의16개 기구학 주행 완료, 상태전이·UDP→HTTP 중계·캐시 조건 확인.
- 배포 코드·지도·캐시의 저장 hash 일치, 기존 Pi 정책의 실행 인자·KSRC UDP·KVO1·SPI 프레임 형식에 명백한 정적 불일치는 발견하지 못함.
- 실제 설치 interpreter/ROS/ORB/model/SPI import 및 장치 구동은 이번에 확인하지 않음. 실제 위치 정확도·등판·하행·모래·센서 복구·네트워크 지연의 실기 검증도 아님.

## 1. 실행 환경과 check의 범위

`deployment/ksrc/cli.py:319`의 check는 Navigation import·runtime/grid hash 검사다. rclpy·Torch 모델·spidev·공통 C library·카메라/ORB 실행 파일을 전부 로드하는 실기 preflight가 아니다. wrapper `deployment/ksrc/ksrc`는 `/home/pi/venv/bin/python`이 없으면 python3로 fallback한다.

Pi 실행 스크립트 `deployment/ksrc/ros_env.sh`는 `/opt/ros/jazzy`와 `/home/pi/handheld_mapping`의 설치 ORB·vocabulary·library·FastDDS 설정에 의존한다. 개발 PC의 Python3.10/ROS Humble과 분리해서 확인해야 한다. 백업에는 venv·해당 외부 설치가 모두 들어 있지 않으므로, 백업 clone만으로 복구됐다고 판정할 수 없다. 실제 Pi에 없다고 단정하는 것은 아니다.

## 2. 시뮬 성공은 실제 하위 명령 전달 증거가 아님

`deployment/broadcast/simulate.py`는 실제 Gateway·상태기·MPPI를 쓰지만 출력 명령을 이상적 SE2 운동으로 적분한다. SPI·펌웨어·서보·정책 추론을 통과한 실제 바퀴 응답은 아니다.

`deployment/ksrc/fake_inputs.py:28`도 status.command를 직접 적분한다. 하위 정책 스크립트가 없어도 FAKE 실행은 가능하다(cli.py). 따라서 FAKE 완주만으로 gateway→policy→Nucleo 연결이 검증되지 않는다.

실기에서는 명령 송신, policy 수신 cmd, Nucleo cmd/seq_echo, 실제 조향·구동 응답을 따로 대조해야 한다. 기본속도0.10/Shift0.15m/s가 실제 지면속도가 되는지도 별도다.

## 3. 절대 위치·ANCHOR는 사람의 실제 배치 확인에 의존

실행기는 기본 출발 preset을 prior로 잡는다. ORB 정상·READY는 실제로 그 좌표에 로버가 있다는 독립 증거가 아니다. 바구니 기준 중계 정렬은 화면 변환이며 Pi localization 보정과 별개다.

실기 Navigation.anchor는 TELEOP·정상센서·3초정지만 검사하고 preset으로 estimator/checkpoint를 재설정한다. 독립 절대좌표가 없으므로 실제 시작점인지 판별하지 못한다. 반면 시뮬은 truth로 출발점15cm·방향8도 이내를 검사한다. 시뮬이 잘못된 위치의 ANCHOR를 거절한다고 실기도 같은 보호가 있다고 생각하면 안 된다.

실제 base_link 중심을 site(3.60,2.82)m에 놓고, 선택한 실제 방향을 맞춘 뒤 ANCHOR한다. 중간 위치에서 ANCHOR로 START 조건만 맞추면 좌표가 잘못된다. site metadata의 accuracy_verified는 false다.

## 4. E-STOP과 복구 동작

START RUN은 실제 AUTO 요청이다. AUTO 중 이동 조이스틱 명령은 무시한다. E-STOP은 gateway를 TELEOP으로 전환하고 상위 임무를 정지시킨다. GUI/Pi 수락과 실제 바퀴 정지를 모두 확인한 후 수동 조작한다.

현재 START는 중간 단계 resume이 아니다. 새 runtime을 만들며 출발 구역·선택 방향·3초정지를 요구한다. 중간 수동 회피 후 그 위치에서 바로 AUTO를 재개하는 기능은 없다.

브라우저 닫기·GUI Esc·launcher 종료·노트북 SSH 끊김은 Pi AUTO 정지가 아니다. 코드의 E-STOP은 UDP 소프트웨어 요청이다. 물리 정지장치의 유무·작동은 이 코드 audit만으로 확인하지 않았다.

## 5. READY에 하위 구동 건강 정보가 부족

`policy_telemetry.py:34`는 IMU_VALID를 검사하고 runner에 IMU·최대 wheel speed·MCU time을 전달한다. decoder가 가진 servo_valid, SERVO_READ_ERR, GATE_TIMEOUT, WHEEL_STALL, ENC_DIR, seq_echo, rx_err와 실제 적용 cmd가 upper READY/안전판정에 모두 연결되지는 않는다.

따라서 READY만으로 네 서보·구동 정상이나 명령 적용을 판정하지 않는다. 원시 로그에는 해당 정보가 남지만 실행 중의 건강표시와 별개다. 바퀴를 띄운 확인에서 fault flags·servo_valid4개·seq echo·cmd 반영·wheel PI를 별도로 본다.

## 6. 센서와 시간동기는 시뮬과 다름

시뮬은 ORB 정상·정확한 pose·완전한 roll/pitch를 가정한다. 실기는 IR stereo/ORB·보정 IMU와 벽 관측을 사용하고 특징점 부족·tracking loss·map reset·지연·자세 진동을 겪는다. 추적 공백 중 로버가 미끄러지거나 밀리면 마지막 checkpoint만으로 실제 이동을 복원할 수 없다.

policy_telemetry는 MCU tick 간격을 보존하지만 첫 sample을 receipt−20ms로 정렬하며 decoder의 imu_mean_read_offset_us/imu_n은 활용하지 않는다. 카메라와 정확한 표본 시각 정렬은 실측 확인 대상이다. 실제 Pi CPU 부하에서 pose350ms/IMU100ms freshness 조건을 유지하는지도 측정한다.

runner의 하위 VO는 연속 pose 두 개 차분을 최신 frame 시각으로 보낸다. 기존 vo_link의 0.2초 regression·창 중앙 시각과 노이즈/age 의미가 다르다. 15Hz에서 구간 중앙과 최신 시각 차이는 약33ms다. 프로토콜이 같아도 정책 입력 특성이 같다는 증거는 아니다.

저장 URDF/YAML/IMU hash는 백업과 맞지만 실기 start가 항상 원본 hash와 real.json/orb.yaml 상호 일관성을 재검사하는 것은 아니다.

## 7. 현재 장애물·코스트·슬립의 의미

실기 카메라는 IR 좌우640×480@15로 실행하며 depth는 OFF다. ros_inputs는 ORB pose/state·IR1을 전달한다. 벽 관측 보정은 있지만, 매번 새 장애물을 depth로 측정해 지도를 갱신하는 구조로 보면 안 된다. 기본 MPPI 제약과 화면 코스트는 저장 지형을 사용한다.

AUTO 화면 예측은 같은 RUN/stage/pose/v/w의 사전 MPPI 조회다. 실제 Pi가 그 순간 계산한 candidate 배열을 전송받는 것은 아니다. TELEOP은 노트북의 별도 국소 MPPI 모델이며 제어에 반영하지 않는다. 후보 색은 평균 지도 경사이지 실제 MPPI 목적함수 수치가 아니다.

GROUND SPEED는 실기에서는 pose 차분이며 독립 지면 속도 센서가 아니다. slip은 지형 경사·이동 방향의 미교정 견인 모델이다. 엔코더와 지면속도로 측정한 실제 휠 슬립률이 아니다. 수학값이 나오더라도 모래·하중·바퀴별 접촉·고착을 실측했다는 의미는 없다.

## 8. 30도 중단과 RUN2 정책

30도는 기존5초 전진명령/10초 경과 예산 중단 정책도 사용한다. 실기에서는 서보 gate·구동 보류로 명령 시간과 실제 바퀴 전진 시간이 다를 수 있다. 경과 예산에 의한 aborted와 실제 no_progress/tilt 실패를 구분한다.

RUN2의30도 생략·15도 직행·정상관찰생략·하행0.7배는 회차 설정이다. RUN1 측정 결과로 일반적인 학습·자동 경로 재학습을 수행하는 구조는 아니다. RUN3/4는 외곽 상승 후15도 하행하도록 구현되어 있지만 물리 등하행 검증은 별도다.

## 9. 중계에서 남은 구체적 공백

- SSH file_age는 Pi가 전송할 때 계산한다. 노트북에서 오래 지연돼 도착한 packet은 Session의 수신 시각 기준으로 fresh가 될 수 있다. 0.5초 stale는 '노트북에 새 packet이 안 온 시간'을 검사하며 실기 capture→화면 지연의 보장은 아니다.
- relay 재접속과 실제 Pi 세션 교체를 구분하는 상태가 부족하다. 송신ID 변경은 Session을 멈추지만 operator/phase/좌표 변환 전체를 새 Pi session 기준으로 재초기화하지 않는다. 같은 RUN 상태로 새 stack이 들어오는 경우 이전 정렬·기록을 이어갈 가능성을 확인해야 한다.
- stale flag가 떠도 AUTO active·elapsed·마지막 속도가 표시상 남을 수 있다. stale 화면을 실제 정지로 해석하면 안 된다.
- 현재 파일의 cache/map hash는 일치하지만 server 시작 시 전체 identity를 fail-closed 검증하지 않고, provenance는 transfer manifest 중심이다. 실기 기록에 mission/cache hash·site identity를 명시하는 개선이 남아 있다.

이번 추가로 TELEOP 궤적은 AUTO와 분리하고 ANCHOR·수신공백0.5초·송신ID 변경 때 선을 끊는다. 위의 Pi session 전체 상태 초기화·SSH end-to-end age 문제를 해결한 것은 아니다.

## 실제 리허설 실행 순서와 통과 기준

1. 노트북/Pi 버전과 해시·Python/ROS·정책/model/.so/ORB 경로를 확인한다. check 통과와 실제 의존성 로드를 구분한다.
2. 바퀴를 띄우고 센서 정상·하위 오류 flags·servo_valid·seq_echo를 확인한다. 저속/Shift/회전/정지의 명령과 실제 응답을 비교한다.
3. E-STOP 수락·실제 정지·TELEOP 인계를 확인한다. GUI focus/창종료/통신끊김과 E-STOP의 차이를 확인한다.
4. 실제 평지에서 출발점과 네 방향을 눈으로 확인하고 ANCHOR한다. 짧은 이동·왕복 후 실제 위치와 site 추정 오차를 비교한다.
5. ORB loss/reset 복구를 정지 상태에서 시험하고 capture/IMU age·CPU/온도·policy period/VO 유효율을 확인한다.
6. RUN1~4를 하나씩 실제로 주행한다. 30도 중단 이유·후퇴·15/외곽 상승·RUN3/4 15도 하행·종료후 TELEOP과 다음 회차 출발 정렬을 확인한다.
7. Pi status와 화면 위치를 함께 보며 실제 AP의 지연·끊김을 측정한다. SSH backlog와 Pi session 교체 표시 문제가 해소되기 전에는 화면 fresh만 믿지 않는다.
8. E-STOP 확인 후 Pi stop으로 정책 NPZ를 저장하고 중계를 정상 종료한다. Pi log·노트북 udp.jsonl/runs.json·수동/AUTO 궤적·OBS 화면을 회수한다.
