# Reference map workflow implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** RGB 기록부터 전역 재구성·중력 정렬·출발점/crop UI·검증된 반복 지도 병합까지 노트북에서 사용할 수 있게 한다.

**Architecture:** 센서 원본과 기존 결과를 보존하고 공통 TSDF 융합에 명시한 poses/output을 연결한다. 독립 core 모듈을 병렬 구현한 뒤 앱에서 비동기 작업으로 연결한다. 모든 불확실한 정합과 중력 추정은 거부/미검증 상태를 남긴다.

**Tech Stack:** Python 3.10, NumPy 1.26.4, SciPy 1.13.1, Open3D 0.19.0, OpenCV, PyQt5, Matplotlib.

**Spec:** `docs/superpowers/specs/2026-10-16-reference-map-workflow-design.md`, inherited `2026-10-16-depth-fragment-reconstruction-design.md`.

## Global constraints

- 실행 환경: `/home/su/rover/.venv-mapping/bin/python`; `PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen`.
- 기본 depth는 0.25~1 m, raw·기존 결과 보존, 새 output 경로만 사용.
- Open3D 0.19.0 유지, RGB 없는 기존 기록도 처리, Pi 저장은 RAM/노트북 전송.
- geometry와 pose 동일 SE3 적용, 실제 경사 보존, uncertainty를 정답 확률로 표시하지 않음.
- 하드웨어·모터·firmware 변경 없음. 코드 상단/클래스 역할 주석, 타입 힌트, 관련 코드만 수정.

## Review focus

- RGB 없는 구형 기록은 geometry를 유지하고 color coverage를 거짓으로 채우지 않는다.
- 평면·0 대응점·반복 구조에서 낮은 RMSE만 보고 정합을 승인하지 않는다.
- crop와 gravity를 저장/재로드할 때 source geometry·trajectory에 변환이 두 번 적용되지 않는다.
- 진행 중 촬영과 재처리를 중복 시작하거나 GUI thread에서 오래 계산하지 않는다.
- explicit output 경로와 source 선택이 원본 덮어쓰기·path escape·map ID 혼합으로 이어지지 않는다.

## Task 1: RGB와 명시한 pose/output TSDF

Files: `pi_worker.py`, `storage.py`, `reconstruct.py`, new `color.py`, `tests/handheld_mapping/test_color.py`.

Interfaces: `--rgb`; color calibration `T_color_depth`; `reconstruct(..., *, keyframe_dir=None, output_dir=None, color=False)`; optional-color capture metrics default remains three streams.

- [ ] RGB stride/encoding/raw 저장·정렬·occlusion·누락 tests를 먼저 작성하고 실패 확인.
  ```python
  assert np.array_equal(decode_image(rgb_header, payload)[0, 0], [30, 20, 10])
  assert summary['paths_relative_to'] == 'summary'
  assert source_depth.read_bytes() == before
  ```
- [ ] worker의 optional stream/CameraInfo/TF/profile 검증과 bounded RGB matching·depth-to-color projection 구현.
- [ ] 기존 signature/output은 유지하고 explicit pose/output 및 RGB fusion 추가. matched color coverage와 거부 사유 저장.
- [ ] `python -m pytest tests/handheld_mapping/test_color.py tests/handheld_mapping/test_core.py tests/handheld_mapping/test_capture_metrics.py -q` 실행.

## Task 2: Depth graph와 재융합

Files: new `registration.py`, `global_reconstruct.py`, `tests/handheld_mapping/test_global_reconstruct.py`.

Interfaces: spec의 `register_clouds(...)` 반환 계약과 `run_global(...)`; Task 1의 explicit pose/output API 사용.

- [ ] 비대칭 3D scene의 known transform·drift, 평면/무대응 거부, 부족한 fragment, raw 보존 테스트 작성/실패 확인.
  ```python
  rejected = register_clouds(flat_source, flat_target, np.eye(4))
  assert rejected['accepted'] is False
  assert result['accuracy_verified_on_real_camera'] is False
  assert result['optimization_status'] in {'optimized_unverified', 'prior_only', 'insufficient_fragments'}
  ```
- [ ] map별 streaming fragments·bounded candidates·양방향 robust ICP/관측성 gate·pose graph·correction SLERP 구현.
- [ ] 동일 depth·설정의 baseline/optimized 재융합과 상태/진단/CLI 작성.
- [ ] `python -m pytest tests/handheld_mapping/test_global_reconstruct.py -q`와 실제 604-keyframe record 재처리.

## Task 3: Reference core와 편집 UI

Files: new `reference.py`, `reference_editor.py`, `tests/handheld_mapping/test_reference.py`, `test_reference_editor.py`.

Interfaces: source legacy/global/explicit/reference 선택; reference.json metadata; spec registration helper; CLI `python -m experiments.handheld_mapping.reference_editor TARGET`.

- [ ] 경사·중력·좌표/시작점·crop·raw 보존·재로드·merge 거부/수용 tests를 작성/실패 확인.
  ```python
  assert np.isclose(np.linalg.norm(aligned_step), np.linalg.norm(source_step))
  assert saved['units'] == 'm'
  assert rejected_merge_did_not_modify_source
  ```
- [ ] IMU provenance/정지/시간/pose 조건을 검증해 중력 후보 계산, 부족하면 명시 vector/manual 방식만 허용.
- [ ] 위 3D/아래 XY 화면, polygon/height ROI, 두 클릭 start+heading, origin/trajectory, reset/preview/save 구현.
- [ ] 새 map 초기 SE3 지정→검증된 정합 preview→새 version 저장 구현. 큰 계산은 worker thread.
- [ ] `python -m pytest tests/handheld_mapping/test_reference.py tests/handheld_mapping/test_reference_editor.py -q`.

## Task 4: 앱·뷰어 연결과 사용 설명

Files: `controller.py`, `app.py`, `viewer.py`, new `tools/reference_mapping.sh`, `tools/edit_reference_map.sh`, docs, integration tests.

- [ ] RGB flag 전송, checkbox 상태, 전역재구성 후처리와 오류 fallback, explicit result 뷰어 선택 tests.
- [ ] source backend/map별 재구성, RGB ON과 global checkbox 기본, 후처리 editor 버튼·기존기록 재처리 연결.
- [ ] 복수 map은 각각의 결과로 보존하고 성공한 다른 map도 fallback 전체와 혼동하지 않음.
- [ ] `python -m pytest tests/handheld_mapping -q`; `git diff --check`; shell syntax 검사.
- [ ] 실제 기록 재처리와 offscreen UI screenshot·저장/재로드·추가 map preview 결과를 보존. 장비미연결 한계를 문서화.

## Task 5: 검토와 실행 폴더 반영

- [ ] 구현 diff 전체를 별도 reviewer에게 읽기 전용 검토. 중요한 지적을 고치고 해당 검사를 재실행.
- [ ] 새 commit을 shared repo에 fast-forward하고 `/home/su/rover`의 대응 소스를 동일하게 반영. 원본 기록/기존 결과는 변경하지 않음.
- [ ] 실행 앱이 idle임을 확인한 뒤 새 코드로 실행, 기록 중이면 저장 완료까지 유지.
- [ ] 실제 동작한 항목·미연결 센서 검증·실측 정확도 한계를 구분해 사용법과 결과 전달.

## 진행 기록

- 설계 확장과 구현 계획 작성. 사용자 일괄 구현 지시를 기준으로 실행하며 중간 승인 대기로 멈추지 않는다.
- 분업은 파일 소유권으로 제한: Task1 RGB/융합, Task2 정합/전역재구성, Task3 reference/editor. Root가 Task4~5를 연결한다. 모든 commit은 통합 담당이 검증 후 생성한다.
- 기준 검사: 236 passed, 8 skipped, 2 failed. 지난 1 m 변경 뒤 `test_offline.py`의 2 m fixture 가정이 남은 실패다. backend 분리 검사는 3 m를 명시하고 실제 기본 경로 fixture는 0.8 m로 수정해 원래 목적을 유지한다.
- Task 1~4 구현 완료. RGB/전역 재구성/편집 모듈 및 앱·wrapper·사용 설명서 통합. 기존 fixture 수정 후 전체 범위 385 passed, 10 skipped, 최종 편집 검사 28 passed. shell syntax·diff check 정상.
- 실제 604-keyframe 재처리는 전역 보정 1.398 m·24.997°를 거부하고 baseline poses를 유지했다. 실제 정확도 향상은 미입증. 편집 crop·출발 prior·처리·저장/재로드 및 원본 hash 불변 확인.
- 독립 검토의 stale merge, crop/processing 재적용, 잘린 경로 재등장, 재처리 상태 잔류, 수동 up 누적, Taubin NaN 문제에 회귀 검사와 수정을 추가했다. 장비 검증은 하지 않았다.
- 최종 검토: 미해결 주요 문제 없음, 독립 관련 검사 78 passed. 최종 전체 검사 387 passed, 10 skipped. 실제 실행 폴더 소스 동기화와 idle UI smoke를 확인했다. 기존 runtime 파일은 별도 backup을 남겼고 촬영·센서·모터는 시작하지 않았다.
