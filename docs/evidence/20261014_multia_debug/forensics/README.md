이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../../data-assets.md)를 따른다.

# 실제 session의 공백·drop·map 분리 포렌식

원본에서 관측되는 내부 이미지 누락 slot 합은 두 실제 session 모두 최종 ByteQueue drop과 정확히 일치한다. 긴 session에서는 left의 0.932430초 공백 직후 첫 입력에서 ORB state가 3이 되고, 약 3.13 source 초 뒤 새 map 421로 재초기화된다. 전송 queue 포화가 visible loss를 설명하는 가장 강한 증거다. 다만 압축/전송/노트북 저장 중 최초 병목과 이미지별 거부 지점은 현 기록으로 분리할 수 없다.

## 실행 및 원본 보존

```bash
cd /home/su/rover
env PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2 \
  .venv-mapping/bin/python \
  outputs/handheld_mapping/20261014_multia_debug/forensics/analyze_records.py
```

Python 3.10.12, 실제 interpreter `/home/su/rover/.venv-mapping/bin/python`. Conda는 이 사후 분석에 사용하지 않는다. script는 forensics 하위에만 쓴다. 최종 실행 exit 0, 원본 metadata/log/CSV/production 파일 45개의 실행 전후 SHA256 동일. `analysis.json`에 hash와 runtime을 보존했다. PNG는 읽기/`stat`만 수행했으며 쓰지 않았다. PNG 내용 전체의 hash 검증은 하지 않았다.

## 1. 센서 source timestamp에서 직접 관측한 사실

시간 원점은 각 session의 첫 기록된 `stamp_ns`다. 정수 ns끼리 뺀 후 초로 변환했으며, 큰 epoch 값을 float로 바꾼 뒤 빼지 않았다.

| session | duration (s) | left/right/depth 기록 수 | median period (ms) | 최대 left/right/depth 공백 (s) | 추정 내부 누락 slot | 최종 queue drop | ORB |
|---|---:|---|---:|---|---|---:|---|
| 20261013_204443_766235 | 242.258045 | 3604 / 3628 / 3584 | 66.592529 | 0.932430 / 0.266406 / 2.997077 | 35 / 11 / 55 = 101 | 101 | map 0, 421; state 2는 3559/3604 = 98.7514% |
| 20261013_203820_847029 | 113.035862 | 1695 / 1695 / 1691 | 66.609619 | 0.266408 / 0.266408 / 0.399611 | 3 / 3 / 7 = 13 | 13 | map 0; state 2는 1695/1695 |
| 20261013_193447_connection_check | 약 8초 | 122 / 122 / 122 | summary.json 참조 | 단일 정상 period | 0 | 0 | offline ORB 기록 없음 |
| 20261013_203449_connection_check | 약 8초 | 120 / 120 / 120 | summary.json 참조 | 단일 정상 period | 0 | 0 | offline ORB 기록 없음 |

긴 session의 기록 timestamp 합집합은 3628개이며 right가 합집합 전체를 덮는다. 같은 timestamp의 right는 있으나 left가 없는 경우는 24개, depth가 없는 경우는 44개다. 세 stream 모두 없는 추정 slot은 11개다. 따라서 `24 + 44 + 3*11 = 101`이다. left/right pairing에서 left의 3604개 모두 같은 stamp의 right를 갖는다.

짧은 session은 left/right가 같은 1695개 timestamp를 갖고, depth만 추가 4개가 없다. 세 stream 모두 없는 추정 slot 3개를 포함하면 `4 + 3*3 = 13`이다. 공통 stereo 공백 80.400005→80.666413초에도 tracking은 유지됐다. 모든 공백이 map 분리로 이어지는 것은 아니다.

긴 session의 내부 추정 누락은 시작 첫 약 1.7초에 12개, 157.75~161.08초 부근에 89개다. 그 밖의 내부 누락 slot은 없다.

### slot 계산의 가정과 한계

stream별 전체 interval의 measured median `P`에 대해 연속 두 기록의 간격 `d`마다 `max(0, round(d/P)-1)`을 계산한다. source가 안정된 약 15 FPS라는 가정이며, hardware frame counter를 복구한 값은 아니다. 최대 rounding residual은 긴 session 413.099 microseconds, 짧은 session 61.4025 microseconds로, 약 66.6 ms period 대비 작아 이번 누락 개수의 반올림은 안정적이다.

첫 기록 이전과 마지막 기록 이후의 slot은 세지 않는다. 세 stream의 첫·끝 stamp는 두 session 모두 같지만 그 경계 밖의 손실은 판정할 수 없다. status 같은 비이미지 packet도 queue에서 거부될 수 있고 그 drop도 같은 누적값에 포함된다. 따라서 정확한 합계 일치는 queue가 모든 visible 내부 누락을 설명한다는 강한 정합성 증거이며, 이미지별 source/ROS/transport 원인을 확정한 증명은 아니다. source 손실·비이미지 drop·경계 drop이 우연히 상쇄된 가능성은 per-drop 기록 없이는 논리적으로 배제할 수 없다.

## 2. tracking/map 전환의 정확한 순서

| timings.csv data row | source t (s) | state | map_id | 의미 |
|---:|---:|---:|---:|---|
| 1 | 0 | 2 | 0 | 정상 tracking 시작 |
| 2379 | 159.683683 | 2 | 0 | 긴 left 공백 직전 마지막 입력, map 0의 마지막 keyframe과도 일치 |
| 2380 | 160.616112793 | 3 | -1 | 0.932429932초 공백 직후 첫 left 입력 |
| 2424 | 163.679791992 | 0 | -1 | state 3 44개 입력 후 reset 상태 |
| 2425 | 163.746393799 | 2 | 421 | 새 map에서 tracking 재개 |

left 공백 `(159.683683,160.616113)`의 열린 구간 안에서 right는 12개 기록됐다. depth는 157.952049→160.949126초에 2.997076904초 동안 비어 있다. 전체 센서/USB가 동일하게 멈췄다고 보기 어려운 비대칭이다. 단, 특정 ROS topic/callback만의 손실은 배제할 수 없다.

map 0 keyframe 417개는 마지막 159.683683초까지, map 421 keyframe 398개는 163.746394초부터 있다. 각 map pose는 독립 좌표계이므로 마지막 map 0 pose와 첫 map 421 pose의 위치 차이를 실제 로봇 이동으로 해석하지 않는다. keyframe `source_stamp`의 decimal text는 Decimal로 읽었고 원본 timestamp와의 최대 차이는 map 0에서 389 ns, map 421에서 343 ns다. 이미 upstream에서 반올림된 decimal precision 이상을 복원했다고 주장하지 않는다.

Offline tracking-call latency는 median 16.353 ms, p95 19.405 ms, max 31.272 ms다. state가 나쁜 입력의 median도 17.501 ms다. 이 값은 촬영 종료 후 노트북 offline ORB 호출 시간으로 Pi CPU/FPS/transport latency가 아니다. `session.json.elapsed_s=370.2`에는 후처리가 포함되므로 sensor capture duration 242.26초와 혼용하면 안 된다. 완료 상태의 `left_receipt_fps=0` 역시 capture 평균 FPS가 아니다.

## 3. queue drop: timestamp 없는 별도 관측축

`events.jsonl` status에는 source stamp, monotonic send/receipt time, 수신 시각이 없다. `frames.jsonl`과 별도 파일이고 둘 사이 전체 packet interleaving도 저장되지 않는다. 따라서 아래 counter를 센서 source 초에 정확히 대응시키지 않았다. plot의 drop 축도 source time과 분리했다.

| events.jsonl line | source_frames | cumulative dropped | 증가 | queued bytes |
|---:|---:|---:|---:|---:|
| 3 | 35 | 6 | 6 | 2766268 |
| 4 | 50 | 12 | 6 | 0 |
| 158 | 2398 | 21 | 9 | 3996115 |
| 159 | 2413 | 53 | 32 | 3996622 |
| 160 | 2428 | 85 | 32 | 3996328 |
| 161 | 2443 | 101 | 16 | 0 |

queue 한계는 4194304 bytes이고 후반 세 status의 snapshot은 약 3.996 MB다. mono8 이미지 한 packet은 약 307 KB, depth는 약 614 KB여서 남은 공간 약 198 KB에서는 둘 다 수용할 수 없다. 실제 `ByteQueue.put_nowait`는 enqueue 시 byte 또는 item 한계에 닿으면 새 packet을 거부하며 누적 drop을 증가시킨다. 이 snapshot의 item 수는 저장되지 않았다.

`source_frames`는 left ROS callback마다 증가하며 recording 전 초기 준비 과정의 callback도 포함한다. 준비 조건은 각 stream 최소 20개지만 정확한 recording-start counter는 기록하지 않는다. 마지막 `finished`에는 최종 source counter가 없고 `session.json`은 마지막 주기 status의 counter를 유지한다. 따라서 `3647 - 3604 = 43`을 left 손실 또는 source 손실 수로 계산하면 안 된다.

후반 drop 급증 동안 source_frames는 2398→2413→2428→2443으로 계속 15개씩 증가한다. 1초 이상일 때 status를 만드는 코드와 맞으며 left callback 자체가 오랫동안 완전히 중단됐다는 증거는 없다. 정확한 callback 도착 간격과 packet별 counter는 저장되지 않았다.

짧은 session은 event 81/82의 source counter 1222/1237에서 cumulative drop이 2/13으로 늘어난다. queue peak는 3995990 bytes다.

## 4. PNG filesystem mtime: 보조증거

PNG `mtime_ns`는 실제 남아 있는 filesystem 쓰기 시각이다. true packet receipt 또는 decode 시작 시각이 아니다. copy/복원 이력, 두 장치 wall-clock 동기, clock drift를 보증하는 별도 기록도 없다. 따라서 절대 `(mtime-source_stamp)`를 network latency로 부르지 않는다.

보조 분석은 첫 기록된 PNG를 기준으로 `(mtime-mtime_first) - (source_stamp-source_first)`를 정수 ns에서 계산했다. 긴 session 10816개와 짧은 session 5081개 파일 모두 mtime이 frames.jsonl 순서로 단조증가하며 ctime==mtime다. 첫 long left PNG는 birth 20:44:50.835693746, mtime/ctime 20:44:50.843690718로 촬영 시각과도 맞는다. 이는 원래 저장 시각이라는 해석과 정합적이나 파일 생성 이력을 완전히 입증하지는 않는다.

긴 session의 left relative write lag peak는 source 158.285056396초에서 +1.564295983초다. 바로 전 정상 155~156초의 median은 -0.194333185초이므로 가까운 구간 대비 약 **1.759초 증가**다. source 161.2초 부근에서 정상 수준으로 돌아온다. 이 증가가 depth/left 누락에 앞서 나타나고 right에서도 비슷한 peak가 보인다. 짧은 session peak는 source 79.933796631초에서 left +0.636691439초다. 두 기록 모두 시간 backlog의 생성과 배출이라는 해석에 맞지만 압축·TCP/SSH·수신/PNG 저장 중 위치를 특정하지 못한다.

plot은 기록된 점만 표시하며 누락 구간을 가짜 관측으로 보간하지 않는다.

## 5. USB/CPU 및 원인 우선순위

두 실제 session의 camera.log 모두 시작 시 `Device USB type: 2.1`, reduced performance 경고가 있다. version은 RealSense ROS v4.58.4/LibRealSense v2.58.4, FW 5.13.0.50이다. 시작 시 depth_units 파라미터 조회 경고 뒤로 정상 종료 전까지 camera timeout/reset/overflow/error 경고는 없다. worker.log는 두 session 모두 비어 있다. dmesg/USB counters, CPU load·temperature·throttling 또는 per-stage 처리시간은 보존되지 않았다. 로그 부재가 USB/CPU 문제 부재의 증명은 아니다.

1. **queue 포화에서 stream별 새 packet 거부**: 직접 drop counter, byte snapshot, 누락 합계 일치, stereo 비대칭으로 가장 강하게 지지된다. 큰 depth packet이 더 오래 비는 현상은 byte 제한/stream 순서의 편향과 정합적이다. 정확한 순서는 per-drop stamp가 필요하다.
2. **일시적 sender/transport/노트북 저장 처리량 저하**: PNG write backlog 증가가 지지한다. 원본 영상의 entropy 증가, zlib CPU, network, host PNG/disk, Python scheduling은 현 기록에서 서로 분리되지 않는다.
3. **USB2 또는 ROS 경로의 source 손실/지연**: USB2 사용은 확실하지만 이번 공백의 직접 원인 증거는 없다. right는 장시간 계속 들어오고 모든 visible slot 손실이 queue counter로 설명되어 우선순위가 낮다. 특정 stream ROS 전달 지연 가능성은 남는다.
4. **ORB 처리시간이 capture를 막음**: 실제 capture는 sensors_only이고 offline 처리이므로 기록된 offline latency로 촬영 공백을 설명할 수 없다. 영상 내용/움직임이 gap 후 tracking 실패에 기여했는지는 별도 replay/content 비교가 필요하다.

현재 기록만으로 source ROS drop과 transport enqueue drop을 이미지별로 분리할 수는 없다. 확실하게 분류되는 것은 101/13개의 enqueue 거부가 있었다는 사실이다.

## 6. 다음 촬영에 필요한 최소 계측

- Recording-start의 stream별 callback counter와 last source stamp, recording-end의 최종 counter. status에도 각 stream counter와 last stamp를 넣는다.
- 각 이미지의 source stamp/hardware frame counter(가능한 경우), per-stream callback sequence, Pi monotonic callback time.
- 거부된 packet의 stream/stamp/sequence/이유(bytes/items), 당시 queued bytes/items, stream별 drop counter. Payload는 추가로 보존할 필요 없다.
- enqueue/dequeue monotonic time, compression start/end·raw/compressed bytes, send start/end. sender 오류도 같은 monotonic 축에 남긴다.
- 노트북 packet-read complete/decode start/end/PNG-write start/end monotonic time. 수신 packet sequence로 Pi와 대응시킨다.
- source USB/ROS 가설 분리가 계속 필요하면 driver frame counters/diagnostics와 저빈도 CPU load/temperature/throttle/USB kernel 이벤트를 함께 수집한다.

## 산출물

- `analyze_records.py`: 원본을 읽는 재현 script.
- `analysis.json`: 전체 정량 요약, runtime, 실행 전후 원본 SHA256.
- session별 `source_timeline.csv`: 실제 기록된 timestamp 합집합과 stream 존재/ORB state.
- session별 `stream_gaps.csv`: 정수 timestamp gap, 추정 누락 slot, rounding residual.
- session별 `drop_status_no_timestamp.csv`: 시각을 만들지 않은 원래 status counter 축.
- session별 `frames_with_write_times.csv`: PNG mtime/ctime/size 보조증거.
- session별 `tracking_transitions.csv`, `keyframes_precise_decimal_parse.csv`.
- long `critical_window_155_166s.csv`, `critical_window.png`, session별 `timeline.png`.
