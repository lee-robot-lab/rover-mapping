# 입력 공백·추적 손실 멀티에이전트 디버깅 — 2026-10-14

사용자 요청: 원인 계속 멀티에이전트 디버깅으로 조사. 사용자는 취침하므로 추가 확인 대기 없이 원인 규명과 근거 있는 수정·재생 검증을 진행한다. 실기 주행/모터 명령 금지. 원본 기록과 이미 생성된 지도는 보존한다. Git 없는 workspace에서 임의 init/commit/worktree 없음.

## 분담
- capture_path_debug: 수집·압축·전송·저장 경로 및 bounded큐/backpressure 독립 재현.
- record_forensics: 실제 기록의 timestamp/drop/state timeline과 공백 단계 근거.
- orb_gap_replay: 동일 원본 native 재생 및 ORB timestamp/매칭/복구 기전.
- root: 결과 독립 검증, 확정 원인에 한한 수정/계측·회귀, 통합 보고.

Production 소스 before snapshot: outputs/handheld_mapping/20261014_multia_debug/before/. 각 agent는 별도 output 하위 폴더만 쓰며 production은 read-only로 시작한다.

## 시작 증거
실제 session20261013_204443_766235의 left159.6837→160.6161s .93243s gap 직후 nonOK, depth157.9520→160.9491s2.99708s gap. 총 queue drop101, rawleft3604/right3628/depth3584, trackingnonOK45pairs, maps0/421. 단독 detector1000+features는 map correspondence/inlier가 아니다. Pi SSH 마지막 조회 실패. native replay/synthetic queue로 소프트웨어 재현하고 hardware원인은 분리한다.

## 확인한 현상과 구현 범위
- long 기록의 nominal slot 누락 left35/right11/depth55 = 101, short3/3/7 = 13으로 각 queue drop과 일치. source_frames는 시작 준비 callback도 포함하므로 단순 총수 차감은 금지.
- 실제 CaptureNode/FIFO 경로를 막는 실험에서 bounded queue 포화·stream별 비대칭 누락을 재현. 정상 종료 sentinel은 큐를 비워 추가 손실이 없었다.
- 원본 full ORB 반복에서도 159~164초 취약 구간이 재현되지만 nonOK/map 수는 실행마다 다름. non-inertial stereo는 1초 timestamp jump 정책의 대상이 아니다. 영상 대응/기하 검증 실패 후 약 3초 relocalization 실패로 새 맵을 만드는 경로다.
- 어느 단계가 송신 큐를 막았는지는 기존 기록으로 확정 불가. 큐 크기·FIFO 정책·QoS·ORB 설정을 임의 변경하지 않는다.

### Task 1: 다음 실기에서 원인 단계 구분을 위한 Pi 계측
소유: capture_path_debug. 파일: experiments/handheld_mapping/pi_worker.py, tests/handheld_mapping/test_capture_metrics.py.
- [x] ROS 없이 실제 집계 helper를 검증하는 RED 테스트: 녹화 중 stream별 received/enqueued/dropped, source/receipt gap, sender compression/write/진행 중 age.
- [x] 고정 개수 counter만 사용하여 구현. image header에 Pi monotonic receipt/enqueue/send-start와 queue/compress duration, stream_seq를 추가하고 status/finished에 capture_metrics를 보낸다. source_frames의 기존 의미는 보존.
- [x] GREEN 및 기존 camera_parameters/transport 회귀. 전송 byte 상한과 원본 stamp를 유지.

### Task 2: 노트북 기록 경계 계측과 종료 snapshot 수정
소유: root. 파일: experiments/handheld_mapping/controller.py, storage.py, tests/handheld_mapping/test_capture_timing.py.
- [x] 실제 subprocess packet 수신·파일 저장 RED 테스트: finished가 마지막 status의 queue/capture_metrics/source_frames를 대체해야 한다.
- [x] image/event에 laptop_receipt_monotonic_ns·laptop_read_wait_ms 기록. 실제 PNG 저장 테스트로 source stamp/원본 보존과 decode/PNG write 시간 및 saved monotonic을 검증.
- [x] session.json에 고정 크기의 read/accept 및 stream별 decode/PNG 집계 기록. read wait는 센서 주기·전송 대기도 포함하며 latency로 표시하지 않는다. Pi/laptop monotonic의 절대값을 빼지 않는다.
- [x] GREEN, 전체 mapping 범위 테스트, 정적 검증. 원본·완료된 map 파일을 덮어쓰지 않는다.

### Task 3: 통합 검증 및 다음 촬영 절차
- [x] forensics/ORB 실험 결과를 독립 재검산하고 단회 설정 A/B를 개선으로 주장하지 않는다.
- [x] 독립 review에서 계측 경계·실패 경로·bounded memory·구버전 worker 호환 확인.
- [x] 보고서에 COLMAP은 offline 기준맵 비교 후보로만 추가. 실제 설치/교체는 이번 원인 조사 범위 밖.
- [x] Pi 재연결 후의 검증 절차와 미확정 원인을 기록한다. 실기 주행·모터 명령은 실행하지 않는다.

## 검증 종료 상태
최종 mapping/headless mapping 173 passed(10.11s), 정적 검사 통과. 20초 실제 영상의 Qt/native/TSDF 재생 exit0. 독립 review의 후처리 실패 시 stale dropped를 RED→GREEN으로 수정하고 재검증. 원본/native 등 47파일 root 재해시 변경 없음. 전체 repository pytest는 기존 output snapshot의 sim/libksrc_common.so 누락으로 collection 중단. 실제 Pi 접속/새 촬영은 미실시이며 최초 backpressure 위치는 미확정. [최종 조사 보고](../../experiments/2026-10-14-input-gap-investigation.md).
