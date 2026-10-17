# Sensor motion safety implementation plan

**Goal:** 시뮬·파일 재생에서 추적 신뢰 저하 감속, 추적/필수 IMU 끊김 정지, 안정 복구 후 재개를 검증한다.
**Architecture:** 독립 sensor guard가 동일 monotonic/simulation clock의 pose·IMU freshness와 tracking 상태를 감시한다. MissionRuntime에 opt-in 연결하며 별도 watchdog 호출은 sensor callback이 없어도 정지 명령을 만든다. GT 기본 모드는 유지한다. 실기 모터 송신은 구현하지 않는다.
**Tech Stack:** Python 3.10 `.venv-mapping`, existing mission runtime/JSONL replay.

- [x] guard: pose 0.5s / IMU 0.1s, degraded scale .3, recovery .5s + 최소 3개의 정상 pose, future/nonmonotonic/invalid 입력 거부.
- [x] runtime opt-in 및 watchdog/IMU 독립 입력, 실제 command 양쪽 속도 제한과 valid_until freshness 반영.
- [x] JSONL dry-run CLI와 센서 loss/recovery/silence 시나리오 증거·회귀 검사.

추적 confidence 자동 추정은 이번 범위가 아니다. upstream은 tracking_valid와 tracking_quality='good'/'degraded'를 명시한다. 임시 threshold는 실기 검증값이 아니다. 오차/pose_epoch 변경은 기존 정지 조건을 유지한다. 실기 구동 전 주기 watchdog의 실제 scheduler·모터 정지 전달과 경사면 정지/제동을 시뮬레이션에서 먼저 검증해야 한다.
