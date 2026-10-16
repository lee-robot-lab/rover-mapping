# 매핑 구조·품질 검토 — 2026-10-15

검토 대상: 공유 저장소의 Python 수집/전송/저장, native runner 소스, TSDF 융합, 기존 실기 조사 기록. 기존 조이스틱 회전 50% 변경은 유지했다. 이번 검토에서 매핑 실행 코드는 변경하지 않았다.

## 판단

Pi 센서 수집 → 노트북 원본 저장 → live 촬영 안내 → 종료 후 전체 원본 ORB → 최종 keyframe/depth TSDF라는 분리는 기준 맵 제작 목적에 적절하다. 원본 재처리, 실제 CameraInfo/TF 사용, timestamp 기반 매칭, map별 독립 출력도 유지할 가치가 있다.

우선 개선할 것은 원본 입력의 연속성과 최종 맵 품질 판정이다. 현재 구조만으로 실측 정확도를 보증할 수 없다. 공개된 실기 보고는 전송 큐 포화와 입력 공백을 보여주지만, 최초 병목이 Pi 압축·무선/SSH·노트북 저장 중 어디인지는 새 실기 계측이 필요하다.

## 1. 우선: 좌우 영상의 원자적 큐 수락과 원본 보존

- 근거: `pi_worker.py:166,190–225`는 left/right/depth를 개별 항목으로 같은 ByteQueue에 넣는다. `transport.py:22–31`은 상한 초과 시 새 항목을 거절한다.
- 영향: 왼쪽만 남고 오른쪽은 누락될 수 있다. `offline.iter_pairs`는 정확히 같은 timestamp의 좌우만 사용하므로 불완전한 쌍은 추적 입력에서 사라진다. 이 exact matching 규칙은 유지한다.
- 계산: 640×480 mono8 두 장 + Z16 한 장은 1,228,800 B/frame-set, 15 Hz이면 압축 전 18.432 MB/s다. 4 MiB 큐는 metadata 제외 약 3.4세트, 약 0.228초 분량이다. 네트워크 실효 전송량은 압축률과 별도 측정해야 한다.
- 재현: 현재 ByteQueue에 송신 없이 4세트를 넣으면 left `[0,1,2,3]`, right/depth `[0,1,2]`만 수락한다. 실제 카메라 없이 비대칭 누락을 재현했다.
- 제안: 짧고 제한된 timestamp 동기화 버퍼에서 좌우를 묶고, 쌍 단위로 큐 수락/거절한다. depth도 대응 관계를 보존한다. depth 대기가 좌우 추적을 무기한 막지 않게 한다. status/pose는 별도 작은 경로로 보내 계측도 포화에 묻히지 않게 한다.
- 원본 무손실이 필요하면 별도 디스크/SSD의 Pi 로컬 기록을 검증하고 노트북 live 전송과 분리한다. RAM disk는 장시간 원본 보존용이 아니다. 큐만 키워서는 지속적인 처리량 부족을 해결하지 못한다. 쌍 단위 큐도 전체 누락 자체를 없애지는 못한다.

## 2. 우선: 수신과 PNG 저장 분리

- 근거: `controller.py:196–208`은 `read_packet` 다음 `writer.accept`를 같은 스레드에서 실행한다. `storage.py:64–79`는 zlib 해제, PNG 쓰기, 인덱스 기록을 끝낸 뒤 반환한다.
- 영향: 저장/압축 지연이 다음 패킷 수신을 늦추고 pipe/SSH를 통해 Pi 송신 큐로 역압을 전달할 수 있다. 이 전파 경로는 코드로 확인되며, 현재 실기의 최초 병목이라는 판정은 아니다.
- 제안: 수신 담당 → byte 상한이 있는 기록 큐 → 순서 보장 writer로 분리한다. 종료 시 큐 drain과 실제 저장 성공을 검증한다. live 안내는 저장 완료된 경로를 별도 worker에 전달한다.
- 초기화 주의: calibration 수신 처리 중 `LiveMapper` 생성이 `vocabularies()`를 호출한다. 최초 vocabulary 압축 해제가 수신을 멈추지 않도록 촬영 전 자산 준비를 마친다.
- 저장 큐도 지속 포화하면 숨기지 않고 촬영 품질 저하/종료 사유를 기록해야 한다. 장기적으로는 받은 무손실 패킷을 순차 파일로 기록하고 PNG를 별도 생성하는 방식을 비교할 수 있다.

## 3. 우선: 오프라인 재생 속도와 최적화 상태 계측

- 근거: `orbline/runner.cpp:49–85`는 파일을 읽고 TrackStereo를 호출한 뒤 다음 영상을 바로 읽는다. 저장된 촬영 간격만큼 대기하지 않는다.
- 기존 증거: 동일 3604쌍 baseline 반복에서 최종 맵 수가 3개/2개, 최종 keyframe 수가 782/788로 달랐다. async scheduling의 영향은 가능한 설명이며 단독 원인으로 확정되지 않았다.
- 공식 ORB-SLAM3 stereo 예제는 다음 frame timestamp와 처리시간 차이만큼 기다린다. LocalMapping은 새 keyframe 유무와 BA 중단 상태에 따라 처리 경로가 달라진다.
- 제안: 원본 timestamp를 그대로 두고 wall-clock 입력 속도만 조절하는 `replay-rate`를 추가한다. 기준 비교는 촬영 속도 1배 재생으로 시작하고, mapper backlog·BA 완료/중단·loop 처리 상태를 기록한다. 처리 여유가 확인된 뒤 빠른 재생을 비교한다. native 연산이 촬영 간격보다 느리면 frame을 버리지 않고 더 오래 처리한다.
- 종료 패치가 worker 종료를 기다리는 것은 좋다. 다만 worker가 종료됐다는 것과 입력 keyframe/loop queue가 모두 처리됐다는 것은 별도 확인해야 한다. 시간만 기다리는 방식으로 완전한 결정성을 보장하지 않는다.

## 4. 후속: depth 품질 선별과 융합 frame 확장

- 근거: `reconstruct.py:77–118`은 최종적으로 살아남은 keyframe CSV만 읽고, 깊이 범위 0.25–3m와 유효 픽셀 100개 기준을 통과하면 융합한다. 100픽셀은 640×480의 약 0.033%다.
- 판단: 최종 최적화된 keyframe pose 사용은 타당하다. 다만 SLAM의 keyframe 생존 기준은 표면 복원에 필요한 관측 밀도와 같지 않다. 삭제된 keyframe과 일반 frame의 depth는 현재 TSDF에 들어가지 않는다.
- 제안: frame별 depth 유효율·깊이 경계·반복 관측 일관성·pose 품질을 기록하고 불량 frame/pixel을 선별한다. 평면 잡음과 실제 단차를 구분하는 실측 비교가 필요하다.
- 다음 단계: 최종 keyframe을 기준으로 보정한 일반 frame pose도 map ID/추적 유효성과 함께 출력한다. 필요한 frame을 시점/이동량/중첩에 따라 추가 선택해 융합한다. 추적 소실·map 변경·긴 공백을 가로질러 pose를 임의 보간하지 않는다.
- 선택 비교: ORB 궤적을 초기값으로 depth 정합과 pose graph 보정을 적용한 뒤 TSDF를 다시 만든다. 평면·반복 구조에서는 정합이 모호하므로 잔차/관측 가능 방향/과도한 보정 거부 조건을 둔다. 개선 효과는 실측 오차로 판단한다.
- TSDF는 입력 pose에 따라 표면을 융합한다. 현재 이 호출에 궤적 정합/최적화 단계가 추가로 들어 있는 것은 아니다. voxel을 줄이면 출력 해상도와 비용이 바뀌며 정확도가 자동으로 높아지는 것은 아니다.

## 5. 병행: 노출과 회전 blur 계측

- 근거: `pi_worker.py:271–277`은 영상 profile/projector/sync를 설정하지만 노출·gain을 명시적으로 설정하거나 저장하지 않는다.
- 제안: 실제 노출·gain·자동노출 상태와 포화/어두운 영역·영상 선명도 지표를 저장한다. 조이스틱 회전 50% 변경 전후를 같은 조명·경로에서 비교한다. 짧은 노출은 회전 blur를 줄일 수 있지만 어두운 영상과 노이즈를 늘릴 수 있으므로 고정 수치를 임의 채택하지 않는다.
- 입력 연속성을 먼저 확보한 뒤 FPS/해상도를 비교한다. 현재 큐가 포화한 상태에서 FPS/해상도/특징 수를 늘리는 것은 별도 비용을 추가한다. 기존 2400 feature 실험은 분리를 해결하지 못하고 처리시간이 늘었다.

## 6. 우선: 저장 완료와 기준 맵 채택 분리

- 근거: `controller.py:293–295`의 complete/partial은 dense 처리 결과와 전송 drop 등을 바탕으로 정해진다. 최종 map이 여러 개여도 각각 융합되면 실행은 complete일 수 있다. `reconstruct.py:86–89`도 실측 정확도 미검증을 명시한다.
- 제안: 현재 실행 상태는 유지하고 별도 `quality.json`과 화면 판정을 추가한다. 목적에 맞는 한 개의 연속 map, 좌우 pair 누락/최대 공백, 추적 손실 구간, 유효 depth/융합량, 중력 정렬 검증, 실측 거리·단차 오차, 재방문 중복 표면을 포함한다.
- 거리/높이 실측과 맵 정합 잔차를 구분한다. 시작점 재방문/loop closure 자체만으로 독립 정확도 검증을 대체하지 않는다. 임계치는 요구되는 주행 정확도와 실측으로 정한다.
- 서로 다른 map을 좌표가 같다는 가정으로 합치지 않는다. 기준 맵 제작 A를 동결한 뒤 별도 주행 B에서 위치 보정 효과를 평가한다.

## IMU·중력 정렬의 위치

현재 native baseline은 STEREO이며 IMU가 들어가지 않는다. 실기 IMU 기록·camera/MCU 시각 관계·축·장착 변환·noise/bias 확인 후 관성융합을 비교하는 순서는 타당하다. D435에 자체 IMU가 있다고 가정하지 않는다. encoder는 미끄러짐이 있는 상황에서 무조건적인 no-slip 위치 관측으로 쓰지 않는다.

지형/costmap용 좌표는 검증한 중력 방향으로 정렬해야 한다. 현재 coverage는 초기 optical X-Z 투영이다. 이 촬영 안내 구조는 유지하되 수평 지형 지도 품질과 구분한다.

## 권장 구조와 적용 순서

```text
Pi: 카메라 → timestamp 동기화 → 좌우 쌍 단위 수락
                         ├→ 원본 로컬 기록(선택, 저장장치 성능 검증)
                         └→ 전송
노트북: 수신 → bounded 기록 큐 → 원본 writer
                                  └→ live ORB → 저주파 coverage/overlay

촬영 후: 원본 검증 → 속도 제어 offline ORB → 최종 pose/추적 상태
         → 선택적 depth 궤적 보정 → depth/frame 품질 선별 → TSDF 재융합
         → 실측 품질 판정 → 기준 맵 동결
         → 검증한 중력 정렬 → 지형/costmap
```

적용 순서:

1. 현재 계측으로 정지/느린 회전/이동을 나눠 새 촬영하고 최초 병목 확인.
2. 좌우 쌍 큐와 수신/기록 분리를 각각 적용하고 같은 조건에서 비교.
3. 촬영 속도 재생과 최적화 상태 계측, 별도 quality 보고서 추가.
4. depth 품질 선별과 최종 보정 pose 기반 융합 확대.
5. IMU 융합·depth 정합·고급 재구성 후보를 동일 입력에서 비교.

## 이번 검증 범위

`/usr/bin/python3`로 아래 기존 테스트를 실행했다. 결과 **22 passed in 0.46s**.

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=. QT_QPA_PLATFORM=offscreen \
  /usr/bin/python3 -m pytest -q -p no:cacheprovider \
  tests/handheld_mapping/test_transport.py \
  tests/handheld_mapping/test_capture_metrics.py \
  tests/handheld_mapping/test_capture_timing.py \
  tests/handheld_mapping/test_camera_parameters.py
```

추가로 현재 ByteQueue의 frame-set 비대칭 수락을 모의 입력으로 재현했다. 새 실제 RUN, native runner 실행, Open3D 실측 비교, Pi/카메라/모터 실행은 수행하지 않았다. 기존 문서의 실기 수치는 과거 증거이며 현재 하드웨어를 새로 측정한 값이 아니다.

근거:

- [입력 공백 조사](2026-10-14-input-gap-investigation.md), [기존 ORB 반복 실행 조사](../evidence/20261014_multia_debug/orb/README.md), [프로젝트 단계 목표](2026-10-13-reference-map-roadmap.md).
- [고정 버전 ORB-SLAM3 stereo 예제](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo/stereo_euroc.cc#L135): timestamp에 따른 재생 대기.
- [고정 버전 LocalMapping](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L57): 비동기 keyframe 처리와 BA 조건.
- [Open3D TSDF](https://www.open3d.org/docs/release/tutorial/pipelines/rgbd_integration.html): 입력 camera pose에 따른 융합과 voxel/noise 관계.
- [Open3D multiway registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html): 여러 관측의 정합과 pose graph 최적화.
- [RealSense 성능 조정 문서](https://dev.realsenseai.com/docs/tuning-depth-cameras-for-best-performance/): 노출/gain 및 motion artifact 관련 지침.
