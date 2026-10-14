이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../../data-assets.md)를 따른다.

# 입력 공백 계측 변경 독립 review

범위: `before/{controller,storage,pi_worker}.py`와 현재 production diff, 새 `test_capture_timing.py` / `test_capture_metrics.py`, 계획 `docs/superpowers/plans/2026-10-14-input-gap-debug-progress.md`. Production은 수정하지 않았다. 이 review 파일과 재현 산출물만 forensics 하위에 생성했다.

## 검토 당시 Important finding — 수정 및 재검증 완료

현재 미해결 Critical/Important finding은 없다. 아래는 처음 발견한 수정 전 현상이며, root가 finished의 dropped를 즉시 반영하도록 수정한 뒤 이 reviewer가 같은 실제 subprocess 재현을 다시 실행했다. 현재 `state=failed,dropped=5,transport_queue.dropped=5`이고 새 회귀도 통과한다.

**[P2] 후처리 실패 시 finished의 최종 dropped가 session.json에 반영되지 않는다.**

- 위치: `experiments/handheld_mapping/controller.py:238`의 finished 처리 및 `:293`~`:297` 후처리 완료/실패 경로.
- `finished` 수신 즉시 queue, source_frames, capture_metrics는 반영하지만 `dropped`는 reconstruct/compare까지 성공한 뒤 `:295`에서만 반영한다. 마지막 주기 status 이후 추가 drop이 생겼고 촬영은 정상 종료했지만 후처리가 실패하면 최종 queue와 최종 drop counter가 서로 달라진다.
- 실제 subprocess 최소 재현: status `dropped=0/source_frames=20` → image 1개 → finished `dropped=5/source_frames=22/queue.dropped=5/queued_bytes=0` → reconstruct가 RuntimeError. 저장된 session.json은 `state=failed,dropped=0,source_frames=22,transport_queue.dropped=5`다. raw PNG와 frame index는 보존된다.
- 최종 촬영 counters 반영이라는 이번 요구 범위에 남은 결함이다. capture가 실패하지 않았어도 잘못된 drop 수를 표시하고, 새 final queue snapshot과 모순된다. 이전 구현부터 있던 실패경로 누락이지만 이번 finished snapshot 수정에서 함께 해결하는 것이 타당하다.
- 수정 제안: finished 처리의 선택적 필드 갱신에 `dropped`를 포함하여 후처리 전에 보존한다. 마지막 status와 finished drop 값이 다르고 후처리가 실패하는 회귀를 추가한다.
- 수정 전 증거: `review_repro_result_before_fix.json`의 `post_capture_failure`. 수정 후 증거: `review_repro_result.json`, `test_final_drop_counter_survives_reconstruction_failure`.

Critical finding은 없다. 위 1건은 해소됐고 다른 Important finding은 발견하지 못했다.

## 검증 결과

```bash
env PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen \
  .venv-mapping/bin/python -m pytest \
  tests/handheld_mapping/test_capture_timing.py \
  tests/handheld_mapping/test_capture_metrics.py \
  tests/handheld_mapping/test_transport.py \
  tests/handheld_mapping/test_camera_parameters.py -q
```

최종 source_frames와 dropped 수정 이후 **22 passed**, exit 0. source_frames 추가 직후 21 passed였고 dropped 실패경로 회귀 1개가 더해졌다. 최초 실행의 20 passed/1 failed는 capture agent가 추가 중이던 `finished.source_frames` 회귀였고, 변경 완료 후 다시 실행하여 해소를 확인했다. 전체 suite는 root가 실행하므로 중복 실행하지 않았다.

```bash
env PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 QT_QPA_PLATFORM=offscreen \
  .venv-mapping/bin/python \
  outputs/handheld_mapping/20261014_multia_debug/forensics/review_repro.py
```

재현 script exit 0. 별도 case로 새 Pi 진단 필드가 전혀 없는 구버전 worker packet을 실제 subprocess로 받아 정상 완료함을 확인했다. 새 필드가 없는 finished는 과거 status 값을 보존한다. 구버전 worker의 stale queue snapshot을 새로 계산할 정보는 없으므로 이를 억지로 0으로 바꾸지 않는 동작이 맞다.

## 보존 조건 확인

- source `stamp_ns`는 정수 그대로 유지되고 PNG 경로의 stamp도 같다. uint8/uint16 원본의 decode/PNG pixel round trip을 새 테스트가 확인한다.
- Pi 내부 duration은 Pi monotonic끼리만, 노트북 duration은 노트북 monotonic끼리만 계산한다. 두 장치 monotonic 절대값 차감은 없다. `read_wait_ms`에는 sensor pacing·pipe 읽기 대기가 포함되며 latency로 표시하지 않는다.
- CaptureMetrics는 세 stream 고정 dictionary, 두 non-image drop counter, 고정 sender 필드만 유지한다. 노트북 집계도 고정 field 수이며 frame별 누적 배열을 추가하지 않는다.
- ByteQueue의 FIFO/drop 정책, byte/item 한계, QoS, compressed PNG 옵션, live offer, final ORB/TSDF 처리 경로를 바꾸지 않았다. 계측 metadata는 enqueue 전에 byte accounting에 포함된다. dequeue 뒤 추가되는 sender 필드는 대기 queue 밖의 current packet에 해당한다.
- sender의 `sent_packets` 및 compression/write 집계는 transmit thread의 queued packet에만 해당한다. 직접 보내는 calibration/ready/artifact/finished는 포함하지 않는다. scope 문자열도 이 범위를 명시한다.
- final queue/dropped/source_frames/capture_metrics는 모두 후처리 전에 반영된다.

## 실패 경로에서 확인한 집계 범위

PNG write가 두 번째 이미지에서 30 ms 대기 후 False를 반환하게 했다. session은 `state=failed,error=원본 영상 저장 실패`가 되고 첫 번째 PNG와 frames.jsonl row는 보존된다. `laptop_capture`와 per-stream image timing은 마지막 성공 처리 packet/image까지만 집계하므로 실패한 write의 30 ms는 남지 않는다. 실패를 성공으로 표시하거나 기존 raw를 삭제하는 문제는 없다. 이 범위를 보고서에 명시하면 이번 변경의 blocker로 볼 필요는 없다. 에러 packet/EOF/truncated read도 같은 이유로 마지막 완료된 packet 이후 시간은 aggregate에서 빠진다.

## 판단을 유보한 범위

- Pi 실제 ROS callback 순서·USB 오류·zlib/network 병목·실제 성능 overhead: 이번 review에는 하드웨어 접속/촬영이 없으며 runtime 계측 검증으로 남겨야 한다.
- ORB map 분리/재생 품질과 COLMAP 비교: 이번 diff가 해당 알고리즘을 수정하지 않는다.
- 실제 종료 직전 write가 영구 block된 상황: 기존 stop timeout/queue drain 정책을 변경하지 않았고, 이번 targeted 테스트는 정상 drain과 집계만 검사한다.

## 검토한 source SHA256

- controller.py 최종: `bf6ec2b155be7097f3f631c18ad65e73a89e4bb05736db32ac4b24d6c6e82e67` (finding 재현 당시 `37baf45de49669b04b16db782528286dfd7d0059cb4a849506bfeb72e6c4d4b6`)
- storage.py: `c5eb0b648c7665d60c39dcb0dd283ef507a41a35398149ba7869562c960f33a5`
- pi_worker.py: `2ad222de174a13c76baa6db8d8584cc763512ed30a933a2ddaa62900fad3e6d0`

판정: **현재 diff는 진행 가능**. 발견한 최종 dropped 누락은 수정·독립 재검증됐고, 나머지 계측 변경에서 요구한 보존 조건을 깨는 문제는 확인하지 못했다. 실제 Pi 성능/장애 계측 검증은 남아 있다.
