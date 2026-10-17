# 추적/IMU 손실 시 감속·정지 정책

## 구현 범위

`SensorSafety`를 `MissionRuntime`에 선택적으로 연결했다. 시뮬레이션·JSONL 재생으로 속도 요청만 검사하며 서버나 실기 모터 제어를 변경하지 않았다. 기존 GT/visual-only runtime 기본 동작은 유지한다.

| 상태 | 동작 |
|---|---|
| tracking_valid=True, tracking_quality=good, 센서 정상 | 정상 속도 |
| tracking_valid=True, tracking_quality=degraded | 선속도·yaw rate 모두 30% |
| tracking_valid=False | 즉시 두 속도 0 요청 |
| pose stale/missing, 필수 IMU stale/missing/invalid | 두 속도 0 요청 |
| 정상 복구 | 최소 3개의 새 정상 pose + 0.5초 유지 전까지 정지 |
| pose_epoch 변경/좌표 정합 오류/안전 경로 위반 | 기존 정지 조건 유지 |

임시 기본값은 pose 0.5초, 필수 IMU 0.1초다. 실기 검증된 안전 기준이 아니다. `tracking_quality`는 upstream이 제공할 명시적 good/degraded 값이며 특징점 수만으로 자동 판단하도록 구현하지 않았다.

## 센서 수신율 해석

기존 실기 기록(`2026-10-16-inertial-fusion.md`)의 Pi 수신 유효 표본은 평균 약39~50Hz였다. 대략45Hz이면 표본 간격 약22ms다. 이는 센서 자체의 내부 sampling 설정 또는 MCU 내부 처리 rate와 구분한다. 지금 실시간 Pi rate를 측정한 결과가 아니다. 예전 시뮬 IMU 200Hz와도 다르다.

IMU 100ms는 약4~5표본의 간격에 해당하지만 실제 수신 jitter, timestamp clock, transport buffering, 최대 허용 정지 지연을 확인하기 전 실기 기본값으로 확정하지 않는다.

## 입력과 watchdog

- 모든 stamp와 now_s는 같은 simulation/monotonic clock이어야 한다.
- pose: 기존 필드 + tracking_valid, 선택 tracking_quality. IMU stamp를 같은 packet에 넣거나 별도 `observe_imu(stamp_s,now_s)`로 전달한다.
- IMU는 `require_imu=True`일 때 필수다. visual-only 모드의 선택 IMU 누락/오류는 주행을 막지 않는다.
- **주기 제어 루프에서 `runtime.watchdog(now_s)`를 별도로 호출해야 한다.** 센서 callback만 감시하면 callback 자체가 끊겼을 때 정지를 내보낼 수 없다. 이번 작업은 실제 ROS timer/MCU motor 송신과 연결한 것이 아니다.
- command의 valid_until_s는 pose·필수 IMU freshness 중 짧은 것으로 제한한다. downstream도 expiry와 정지 전달을 검사해야 한다.
- stop은 zero velocity 요청이며 경사면 제동·밀림 방지의 물리적 보장은 아니다. 실기 적용 전 경사면 정지/hold와 모터 watchdog을 시뮬레이션에서 검증해야 한다.

## 파일 재생

`.venv-mapping` Python 3.10 CPU 환경:

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.handheld_mapping.mission_runtime \
  MISSION INPUT.jsonl NEW_OUTPUT --sensor-safety --require-imu
```

독립 IMU 이벤트: `{"sensor_event":"imu","imu_stamp_s":0.025,"now_s":0.025}`.
입력 침묵 검사 tick: `{"watchdog":true,"now_s":0.65}`.
JSONL은 정적 입력 재생이며 timer를 자동 생성하지 않는다. 출력은 파일이고 모터 송신은 없다.

## 확인 증거

`outputs/site_sim/sensor_safety_demo_20261017`에 실제 mission runtime + 합성 센서 시각의 430 command 기록을 생성했다. tracking loss, 필수 IMU 끊김, pose callback 침묵 모두 zero command, degraded 감속, 안정 시간 후 재개를 검사했다. 물리 주행/실기 IMU 실측 결과는 아니다.
