# 입력 공백·ORB 맵 분리 조사 — 2026-10-14

현재 가장 강한 증거는 Pi의 bounded 송신 큐 포화와 stream별 enqueue 거절이다. 큐를 처음 막은 압축·SSH/무선·노트북 처리 중 어느 단계인지는 기존 기록으로 구분할 수 없다. 이번 변경은 이 경계를 다음 촬영에서 구분할 계측과, 촬영 종료 counters의 저장 오류 수정이다. 실제 입력 공백이 해결됐다고 판정하지 않는다.

## 실제 기록과 재현

| 기록 | left/right/depth 내부 누락 slot 추정 | 최종 queue drop | 결과 |
|---|---|---|---|
| 20261013_204443_766235 | 35 / 11 / 55 = 101 | 101 | left 0.93243초 공백 직후 추적 손실, 최종 맵 2개 |
| 20261013_203820_847029 | 3 / 3 / 7 = 13 | 13 | 공백은 있으나 추적 유지, 최종 맵 1개 |

약 15 FPS의 measured median period에 대한 내부 slot 추정이며 hardware counter가 아니다. 시작 전·끝 이후의 누락과 비이미지 packet drop은 이미지별로 구분되지 않는다. 합계 일치는 큐 포화를 강하게 지지하지만 USB·ROS 손실 가능성을 완전히 배제하지는 않는다. `source_frames`는 시작 준비의 left callback도 포함한다.

실제 CaptureNode/ByteQueue 경로에 송신 차단을 넣어 91개 입력 중 80개 거절, 수락한 11개 정상 drain을 재현했다. 제한된 서비스에서 right만 계속 통과하고 left/depth가 연속 거절되는 stream 편향도 재현했다. 큐 크기·FIFO/drop-new 정책·QoS·압축 옵션은 변경하지 않았다.

같은 원본 3604 stereo 쌍을 다시 계산하면 baseline도 비정상 frame 76/46개, 맵 3/2개로 결과가 달랐다. 특징 수 1200→2400에서도 주 실패 구간과 맵 분리가 유지되고 처리시간 중앙값은 약 16.5→23.3 ms로 늘었다. 2400 첫 실행은 입력 처리가 끝난 뒤 SIGSEGV로 최종 저장에 실패했고 GDB 반복에서는 재현되지 않았다. 설정 변경을 채택하지 않는다.

현재 runner는 depth를 직접 넣지 않는 non-inertial `STEREO`다. 큰 timestamp jump의 강제 reset은 inertial 분기에만 있다. stereo 영상 대응/기하 검증에 실패한 뒤 약 3초 동안 relocalization에 실패하면 새 맵을 만든다. ORB 검출점 1000개 이상은 충분한 map-point inlier를 보장하지 않는다. 정확히 어느 내부 기준에서 탈락했는지는 아직 계측하지 않았다.

## 다음 촬영부터 남는 진단 정보

| 위치 | 기록 |
|---|---|
| Pi `capture_metrics.streams` | 녹화 중 stream별 received/enqueued/dropped, 첫·마지막 source stamp, 최대 source/receipt gap |
| Pi 각 image header | `stream_seq`, `pi_receipt_monotonic_ns`, `pi_enqueue_monotonic_ns`, `pi_queue_wait_ms`, `pi_compress_ms`, `pi_send_start_monotonic_ns` |
| Pi `capture_metrics.sender` | queued packet의 sent 수, last/max compress·write 시간, 송신 진행 중 age |
| 노트북 `frames.jsonl` | 원래 stamp/PNG 경로와 `laptop_receipt_monotonic_ns`, `laptop_read_wait_ms`, `laptop_decode_ms`, `laptop_png_write_ms`, `laptop_image_saved_monotonic_ns` |
| 노트북 `events.jsonl` | status/finished 등 packet의 노트북 receipt와 read wait |
| 노트북 `session.json.laptop_capture` | 완료 packet의 read/accept 집계, 최대 packet 처리시간·kind·source stamp, stream별 decode/PNG 집계 |

Pi/laptop monotonic의 원점은 다르므로 절대값을 서로 빼지 않는다. 같은 장치의 duration과 대응 source stamp/sequence를 사용한다. `read_wait_ms`에는 sensor pacing·pipe/전송 대기가 포함되고, `write_ms`에는 output lock·쓰기/flush·스케줄 지연이 포함된다. 화면의 live lag도 end-to-end latency가 아니다.

집계는 고정 counter만 유지해 Pi RAM 사용량이 촬영 길이에 따라 늘어나지 않는다. 이미지 metadata는 기존 byte 한계의 accounting에 포함된다. 직접 전송하는 calibration/ready/artifact/finished는 queued sender 수에 포함되지 않는다. 최종 queue/drop/source counter와 capture_metrics는 후처리 전에 session 상태에 반영되며, TSDF가 실패해도 보존한다. 구버전 worker에 새 필드가 없으면 마지막 status 정보를 유지한다.

실패한 PNG write·끊긴 packet·EOF 이후의 시간은 완료 packet/image 집계에 포함되지 않는다. 실패 상태와 이미 저장한 원본은 남는다. 저주파 status는 snapshot이며 모든 drop의 개별 stamp/reason 로그를 추가한 것은 아니다. 원인 분리에 부족하면 이 부분과 driver/kernel·CPU 정보를 추가 계측한다.

## 검증

- 신규 counter·clock·slow PNG·subprocess·구버전·후처리 실패 회귀는 RED→GREEN으로 확인했다. 최종 매핑 범위 **173 passed (10.11초)**. compileall/pyflakes/bash 문법 검사 통과.
- 저장된 실제 D435 20초를 원래 간격으로 재생해 Qt·raw·live ORB·overlay·최종 ORB·TSDF 확인. 원본 이미지 891개 저장, stereo 298쌍 tracking OK, 최종 98 keyframe/depth 융합. 원본에 이미 있던 누락은 그대로다. replay worker의 drop=0은 원래 촬영의 drop=0을 뜻하지 않는다.
- 이 재생의 PNG 최대 write 시간은 left/right/depth 약 11.52/13.26/12.39 ms, packet 최대 처리시간 약 15.88 ms였다. live pair drop은 시작 구간 뒤 10초와 종료 모두 59개, raw 저장과는 별개다. 현재 Pi/USB/무선의 부하·성능이나 전체 지연을 측정한 결과가 아니다.
- 독립 review에서 발견한 후처리 실패 시 stale dropped를 수정하고 같은 subprocess로 재검증했다. 미해결 Critical/Important finding 없음.
- 원본 metadata/log/CSV와 native/runtime/vocabulary 등 47개 파일을 root가 재해시해 변경 없음 확인. 원본 PNG 전체의 hash 검증은 수행하지 않았다.
- repository 전체 `pytest -q --maxfail=1`은 기존 output snapshot의 `pi/common/test_spi_link.py` 수집 시 `sim/libksrc_common.so` 부재로 중단됐다. 전체 repository 통과를 주장하지 않는다.

현재 Pi는 SSH hostname 조회에 실패해 계측을 적용한 새 실기 촬영은 하지 못했다. 모터/주행 명령은 실행하지 않았다.

## 이어서 할 검증과 COLMAP 위치

다음 연결 후 새 프로세스로 매핑 앱을 열고 정지·느린 회전·이동을 구분해 촬영한다. 각 stream received/enqueued/dropped와 저장 frame 수를 대조하고, source gap·receipt gap·queue wait·compression/write·노트북 처리 시간을 같은 구간에서 비교한다. 원지점이 확인되면 그 단계만 수정하고 같은 조건에서 누락 감소를 검증한다. 이후 IMU/encoder 기록과 시각·extrinsic·noise 검증, 실측 거리/단차·재방문 지도 평가로 이어간다.

COLMAP은 여러 이미지의 camera pose/sparse 구조를 추정하는 SfM과 dense MVS를 제공한다. [공식 tutorial](https://colmap.github.io/tutorial.html). 우리 구성에서는 촬영 중 ORB 안내를 유지하고 **촬영 후 기준 맵의 pose/형상을 비교하는 후보**로 둔다. D435 depth와 함께 사용할 때는 pose 좌표·scale·시간 대응·카메라 보정을 확인해야 한다. 같은 기록에서 실측 거리·중복 표면·실패 구간으로 비교하기 전에는 기존 ORB+TSDF보다 정확하다고 정하지 않는다. COLMAP 설치·실행·통합은 이번 원인 조사에 포함하지 않았다.

증거: [forensics](../evidence/20261014_multia_debug/forensics/README.md), [송신 경로 재현](../evidence/20261014_multia_debug/capture/findings.md), [ORB 반복 비교](../evidence/20261014_multia_debug/orb/README.md), [통합 재생](../evidence/20261014_multia_debug/laptop_replay/compact_validation.json), [review](../evidence/20261014_multia_debug/forensics/review.md).
