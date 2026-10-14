# Handheld Mapping App Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. 사용자 요청에 따라 이 세션에서 직접 실행하며 추가 승인 질문 없이 진행한다.

**Goal:** 시작·종료 버튼으로 노트북 원본 저장과 TSDF 생성을 수행하고 ORB-LINE 비교 도구를 제공한다.

**Architecture:** Pi에서 카메라와 기존 ORB adapter를 실행하고 제한된 SSH stream으로 노트북에 원본을 기록한다. 노트북 controller가 종료·artifact 수신·keyframe TSDF를 담당하며 Qt 창은 상태와 버튼을 제공한다. depth와 최종 keyframe은 실제 timestamp/TF 보정 계약으로 연결한다.

**Tech Stack:** Python3.10/3.12, PyQt5, native ROS2 Humble/Jazzy, OpenCV, Open3D0.19, C++/ORB-LINE-SLAM.

**Spec:** `docs/superpowers/specs/2026-10-13-handheld-mapping-app-design.md`

## Global Constraints
- Pi 접속은 실기 시작 버튼에서만, 지금은 카메라 없이 검증한다.
- 원본은 노트북, Pi 새 실행 자료는 `/dev/shm`.
- source stamp 보존, calibrated depth scale/TF, unrelated process 종료 금지.
- map 자동 병합·pose 손실 보간 금지. 실제 하드웨어 미검증을 명시.
- Git 저장소가 없으므로 커밋 대신 progress 문서와 실행 로그로 증거 보존.

## Review Focus
1. SSH stdout 잡음/잘린 payload: 정상 종료와 혼동하지 않고 원본을 보존한다.
2. depth와 IR의 좌표·단위 차이: TF/scale 누락이면 융합을 거부한다.
3. 최종 keyframe 없는 기록: 빈 mesh를 성공 표시하지 않는다.
4. 중복 버튼/창 닫기: 소유 process만 정상 종료하고 저장 완료까지 기다린다.
5. Wi-Fi 대역폭/디스크 부족: queue 제한, drop과 불완전 상태를 표시한다.

### Task 1: 전송·원본 저장·TSDF
**Files:** `experiments/handheld_mapping/{protocol,storage,reconstruct}.py`, `tests/handheld_mapping/test_core.py`.
**Interfaces:** `write_packet(stream, header, payload=b'')`, `read_packet(stream)`, `RunWriter.accept(header,payload)`, `reconstruct(run_dir,voxel_size=.01) -> dict`.
- [x] 회귀 test 작성: 잘린 packet EOFError, 잘못된 크기 ValueError, depth단위/TF 필수, timestamp mismatch 제외, map별 별도 mesh.
- [x] `PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_core.py -q`로 RED 확인.
- [x] protocol은 big-endian header길이+payload길이, storage는 PNG와 JSONL index, reconstruct는 최종 keyframe CSV와 2ms 이내 depth를 일대일 연결하고 `inv(T_world_left @ T_left_depth)`로 TSDF 융합.
- [x] 같은 명령으로 GREEN, 합성 평면2m와 단차의 좌표 오차 확인.

### Task 2: Pi worker·노트북 session controller
**Files:** `experiments/handheld_mapping/{pi_worker,controller,demo}.py`, `tests/handheld_mapping/test_session.py`.
**Interfaces:** `Session.start(config)`, `Session.stop()`, `Session.snapshot()`; subprocess stdin `stop\n`, stdout packet stream. controller는 normal terminal packet과 exit0 둘 다 확인.
- [x] RED: 실제 local worker subprocess로 중복 시작, 정상 마감, 갑작스러운 EOF, 저장 오류, 소유 process만 종료 검사.
- [x] Pi worker에서 native ROS camera profile·CameraInfo·TF·depth scale 확보, prepare_run/ORB 실행, frame/state stream과 shutdown artifact 전달. stderr 로그는 RAM.
- [x] controller가 새 폴더와 stream 기록을 생성하고 종료 후 reconstruct를 호출; demo는 합성 raw+독립 GT pose로 동일 protocol을 통과.
- [x] session test GREEN과 전체 demo 실행 로그/mesh 저장.

### Task 3: Qt 제어창과 실행기
**Files:** `experiments/handheld_mapping/app.py`, `tools/handheld_mapping.sh`, `tests/handheld_mapping/test_app.py`.
**Interfaces:** `MappingWindow(session)`; controller 상태를 QTimer로 읽고 blocking I/O는 worker thread에서 실행.
- [x] RED: offscreen Qt에서 시작 중복 비활성, 종료→저장 중 버튼 차단, 실패 내용 표시, 창 닫기 처리 검사.
- [x] 연결 대상·모드·저장폴더와 핵심 버튼/preview 구현. `--demo`는 명확하게 합성 입력이라고 표시.
- [x] `QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_app.py -q` GREEN.
- [x] 실제 app demo를 offscreen으로 실행해 UI 이미지와 결과 저장.

### Task 4: ORB-LINE offline runner와 비교
**Files:** `experiments/handheld_mapping/orbline/*`, `tools/build_orbline.sh`, `experiments/handheld_mapping/compare.py`, `tests/handheld_mapping/test_compare.py`.
**Interfaces:** runner는 저장된 stereo PNG와 calibration으로 keyframe CSV/state timing 생성. compare는 서로 다른 backend 결과를 별도 폴더에 보존.
- [x] RED: 추적 실패의 성공률 산입 방지, 다른 map 연결 금지, 실제 처리시간/GT 미사용 표시 검사.
- [x] upstream clone을 C++14/OpenCV4 환경에 독립 build. upstream source 수정은 별도 patch로 보존. ROS2 runtime에 기존 core를 덮어쓰지 않는다.
- [x] 공용 stereo 입력으로 runner 실행 및 결과 검사, app에서 설치/검증 상태에 맞게 모드 활성화.

### Task 5: 전체 검증·문서
**Files:** `docs/experiments/handheld-mapping-app.md`, 이 plan과 progress, `outputs/handheld_mapping/20261013_app_validation/`.
- [x] 신규 suite·기존 headless suite, compileall, launcher syntax, 전체 합성 촬영→종료→TSDF 실행.
- [x] 별도 자체 review에서 좌표/종료/프로세스 소유권/성공 조건 점검. 중요 결함은 재현 test→수정→회귀 검사.
- [x] 사용법·실기 준비·검증된 범위·미검증 항목과 산출물 링크 기록.
