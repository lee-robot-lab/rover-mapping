이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../../data-assets.md)를 따른다.

# 수집·전송 경로 조사

대상 원본: `outputs/handheld_mapping/sessions/20261013_204443_766235`. 원본은 읽기만 했다. 아래 조사 후 root의 지정에 따라 production `pi_worker.py`와 새 `test_capture_metrics.py`에 계측을 추가했다. Pi SSH·ROS·실기체 실행 없이 `.venv-mapping` Python 3.10.12, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=2`, `OPENBLAS_NUM_THREADS=2`로 분석했다. 조사 본문의 코드라인은 변경 전 기준이다.

## 결론과 근거의 강도

**확정:** Pi 전송 큐는 실제로 포화하여 패킷 101개를 거절했다. `ByteQueue.put_nowait`의 byte/item 상한 검사와 drop-new 동작은 `transport.py:22–31`, 이를 합산하는 callback은 `pi_worker.py:124–130`이다. 원본 `events.jsonl`과 finished에서 cumulative dropped=101이며 main 구간에는 약 3,996KB queue가 이어진다. 전체 큐 용량은 4,194,304B이고 영상 1 cycle raw는 1,228,800B이므로 4MiB는 header를 제외해도 약 0.228s(15Hz)의 대기량이다. **센서 데이터량에 비해 일시 정체 허용시간이 짧다는 계산이지, buffer 증대를 권하는 것은 아니다.**

**매우 강한 연결:** 저장 영상의 nominal missing을 median source-stamp period 0.066592529s로 계산하면 다음과 같다. 각 gap별 `round(gap/period)-1`을 합산한 수치이므로 하드웨어 frame counter 관측값은 아니다.

| 구간 | left | right | depth | 합계 | status dropped 증가 |
|---|---:|---:|---:|---:|---:|
| 시작 약 0.67–1.66s | 3 | 1 | 8 | 12 | 12 |
| 약 157.75–161.08s | 32 | 10 | 47 | 89 | 89 |
| 전체 | 35 | 11 | 55 | 101 | 101 |

공통 origin은 `1791891890550544922ns`이며 저장 마지막 시각은 +242.258044678s이다. right stamp 집합 3,628개에 모든 left/depth stamp가 포함되어 있다. left는 right 대비 24개, depth는 44개 적다. 전체 nominal 누락과 큐 drop이 정확히 맞는 점, 두 군집별로도 일치하는 점이 주 손실 경로를 전송 큐로 좁혀 준다.

주요 left gap은 +159.683682861→160.616112793s, 0.932429932s이다. 그 사이 right 영상은 12개, depth는 0개가 저장되었다. depth는 +157.952049072→160.949125976s, 2.997076904s이며 사이에 left 14개/right 34개가 저장되었다. 전체 카메라 정지보다 혼합 FIFO 포화 중 크기/순서에 따른 선택적 거절 패턴과 잘 맞는다.

**미확정:** 큐가 포화한 원래 지연 위치는 원본 기록으로 구분할 수 없다. `pi_worker.py:247–257`은 한 sender가 zlib level 1 압축 후 송신을 직렬로 수행한다. `protocol.py:24–31`은 각 패킷마다 stdout flush한다. 노트북 `controller.py:192–197`은 다음 패킷을 읽기 전에 `RunWriter.accept`를 끝내며, `storage.py:60–68`은 zlib decode, PNG level 1 쓰기, line-buffered metadata 기록을 동기적으로 수행한다. 따라서 Pi 압축 CPU 지연, SSH/무선 송신 지연, 노트북 PNG/disk/스케줄 지연이 모두 같은 Pi 큐 포화로 보인다. 원본에는 stage별 wall time·송신 byte·receipt timestamp가 없다.

**확정:** 정상 종료에서 이미 enqueue된 영상의 추가 손실은 확인되지 않았다. `pi_worker.py:294–300`은 recording을 끄고 sender에 FIFO sentinel을 넣은 뒤 join한다. 원본 finished.queue.queued_bytes=0이고 finished.status=saved이다. fake ROS 재현에서도 수락된 11개를 모두 보내고 큐가 0이 되었다. ROS/DDS에 남아 아직 callback되지 않은 종료 직전 영상은 이 보장에 포함하지 않는다.

**확정된 표시/기록 오류:** 원본 session.json.transport_queue.queued_bytes=922,019B이지만 finished 패킷의 최종 queue는 0B이다. `controller.py:219–225`가 queue를 status에만 반영하고 finished에서는 갱신하지 않아 종료 후 snapshot이 낡은 값으로 남는다. 누락 원인과 별개의 작은 수정 후보이다.

## fake ROS·sink 재현

`reproduce_capture.py`는 production `CaptureNode`와 `transmit`를 AST로 읽어 그대로 실행한다. ROS Node/subscription과 stdout sink만 fake로 바꾸며 실제 `ByteQueue`, `write_packet`, `zlib.compress`를 사용한다. 생산 파일을 복사하거나 수정하지 않으므로 재현의 로직이 production과 어긋나는 위험을 줄인다.

1. 첫 송신의 flush를 차단한 동안 source image callback 91개를 실행하면 80개 drop(left27/right26/depth27), queue peak=3,995,507B이다. sink를 풀고 `finish`하면 11개(left4/right4/depth3)를 모두 전송하고 queue=0, accepted-frame shutdown loss=0이다.
2. 13개의 IR로 queue를 채운 뒤 15Hz tick마다 IR 한 개분만 dequeue하고 right→left→depth를 offer하면 3초간 right45개를 수락하고 left45/depth45개는 모두 거절한다. 고정 callback 순서와 크기가 편향을 만들 수 있음을 결정적으로 재현한다. **이 순서가 실제 Pi의 정확한 실행 순서라는 주장은 아니다.**
3. ROS 이전에서 사라진 sample을 callback 간 1s stamp gap으로 재현하면 reported transport dropped=0이다. 현재 counter는 카메라/driver/DDS 손실을 세지 않는다.

```bash
env PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  .venv-mapping/bin/python outputs/handheld_mapping/20261014_multia_debug/capture/analyze_capture.py
env PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  .venv-mapping/bin/python outputs/handheld_mapping/20261014_multia_debug/capture/reproduce_capture.py
env PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 \
  .venv-mapping/bin/python -m pytest \
  tests/handheld_mapping/test_transport.py tests/handheld_mapping/test_core.py -q
```

마지막 명령은 19 passed. 처음 존재하지 않는 `test_protocol.py`를 지정한 검증 시도는 collection 전에 실패했으며, 실제 protocol/storage test가 있는 `test_core.py`로 수정해 실행했다.

## ROS QoS·실시간 스케줄 후보

`pi_worker.py:115–119`는 `qos_profile_sensor_data`로 수신한다. 로컬 Humble header `/opt/ros/humble/include/rmw/rmw/qos_profiles.h:25–35`에서는 KEEP_LAST depth=5, BEST_EFFORT, VOLATILE이다. 이는 로컬 참고이며 Pi Jazzy runtime을 직접 조회하지 않았다. 15Hz에서 subscription backlog가 5 frame을 넘는 executor stall이면 callback 전에 유실될 수 있으나, 이번 세션의 nominal 누락 총량과 queue drop이 일치해 별도의 upstream 손실을 주장할 증거는 없다. RELIABLE 전환이나 depth 증가로 포화 원인을 가리는 변경은 권하지 않는다.

`rclpy.spin_once`는 메인 루프의 단일 executor에서 호출되고, image callback은 raw bytes 복사 후 enqueue만 수행한다. 압축·SSH는 sender thread여서 callback이 직접 stdout에 막히지는 않는다. 다만 CPU/스케줄 압박을 정량적으로 보여 주는 callback receipt gap 기록은 없다. hard real-time scheduler·CPU affinity·executor 교체의 근거도 없다.

`orb/camera.log`에는 D435 USB type 2.1 및 reduced-performance 경고가 존재한다. 640×480×(1+1+2)×15=18,432,000B/s(147.456Mb/s) raw이며 USB2/driver 지연 가능성을 배제할 수 없다. 그러나 경고만으로 주 원인을 USB로 단정할 수 없고, 별도 transport 밖 손실의 관측 증거도 없다.

## 가장 작은 다음 변경

1. **per-stream 수집·누락·gap 계측:** recording 이후 callback count, enqueue 성공, queue drop, source-stamp/receipt gap max와 last를 status/finished에 넣는다. gap은 센서 producer stop과 executor 미전달을 구분하지 못하므로 `upstream_gap`/`callback_gap`처럼 범위를 정확히 이름 붙인다.
2. **stage time 계측:** sender의 compression wall time, write/flush wall time, current write age·compressed bytes를 status/finished로 기록한다. 노트북은 packet read elapsed와 writer.accept elapsed를 별도 누적한다. 지연 후에 도착하는 status라도 이미 저장한 bounded cumulative counters로 어느 경계에서 정체했는지 드러낼 수 있다.
3. **finished snapshot 반영:** controller가 finished.queue와 최종 수집 counters를 마지막 status 대신 저장하도록 한다. 위 stale snapshot 값으로 재현·회귀 검증할 수 있다.

현재 증거로 buffer 증가, QoS 변경, 압축 교체, async disk queue 추가를 먼저 적용하면 정체 원인을 구분하지 못한다. 계측 후 write/flush가 길고 notebook accept가 짧으면 SSH·무선 경로, notebook accept가 길면 저장 경로, compression이 길면 Pi CPU 경로로 좁힌다. 추가 개선은 그 결과에 맞춰 한 경계씩 검증한다.

## 승인된 production 계측과 검증

root의 범위 지정에 따라 `experiments/handheld_mapping/pi_worker.py` 및 새 `tests/handheld_mapping/test_capture_metrics.py`만 수정했다. 기존 큐 정책/크기, ROS QoS, zlib level, framing은 유지한다. `controller.py`/`storage.py`는 root 소유이다. 이 작업은 누락을 제거했다고 주장하는 수정이 아니라 다음 실제 촬영에서 정체 위치를 구분하는 계측이다.

- `pi_worker.py:28–89`: `CaptureMetrics`는 fixed 3 streams, status/pose drop, sender 고정 집계와 lock으로 detached snapshot을 반환한다. 히스토리/새 queue는 없다. stream은 recording 중 received/enqueued/dropped, 첫/마지막 source stamp, max source-gap/receipt-gap, 마지막 Pi receipt를 기록한다.
- `pi_worker.py:190–225`: image callback에서 Pi receipt monotonic과 recording stream_seq, enqueue 직전 Pi monotonic을 header에 넣고 enqueue 성공/실패를 구분한다. 기존 `source_frames`는 startup left callbacks를 포함하며 :173 주석과 snapshot scope에 명시했다.
- `pi_worker.py:323–345`: dequeue→compression→send의 실제 경계에서 `pi_queue_wait_ms`, `pi_compress_ms`, `pi_send_start_monotonic_ns`를 기록한다. sender 집계는 전송 완료 packets 수, last/max compression·write wall duration, 현재 send age를 담는다. 직접 보낸 calibration/ready/artifact/finished는 queued sender count에 포함하지 않는다.
- `pi_worker.py:236–240`, `:375–377`, `:395–398`: 두 status 경로 및 finished에 `capture_metrics`, finished에 마지막 `source_frames`를 넣는다. camera source gap은 source stamp 차이, 나머지 시간은 Pi 단일 호스트 monotonic clock이다. 노트북 monotonic과 직접 뺄 수 없다.

송신 wall duration은 `send` wrapper의 output lock 대기, JSON/framing 쓰기 및 flush, 그 사이 scheduler 지연을 포함한다. 압축 wall duration도 scheduler 지연을 포함한다. 따라서 계측값 하나를 순수 CPU나 무선 latency라고 부르면 안 된다. 실제 단계별 상관관계를 보고 원인을 좁혀야 한다.

TDD 기록: `metrics_red.log`는 CaptureMetrics 미존재로 5 failed, `final_counter_red.log`는 finished.source_frames 누락으로 1 failed이다. 구현 후 `metrics_green.log`는 새6개 + transport/core/camera_parameters 기존25개, 합계 **31 passed in 1.60s**이다. root가 전체 pytest 통합 검증을 맡는다.

기존 fake ROS·sink를 새 helper에 연결해 다시 실행했다. 계측 전후 offered91/drop80/sent11와 종료 추가손실0, 3초 right45/left0/depth0 수락 패턴이 동일하다. header 계측이 raw byte 예산에도 포함되어 stalled-sender peak는 기존3,995,507B에서3,996,497B로 약990B 늘었다. 새 `reproduction.json`은 변경 후 결과이다. 실 Pi 성능 overhead와 실제 입력/송신 정체 위치는 Pi 촬영으로 확인해야 하며 이 작업에서는 실행하지 않았다.
