# 첫 실기 ORB 수동 지도 생성: 준비·기록·판정

조사일: 2026-10-09. 상태: 공식 자료와 로컬 코드에 근거한 실행 준비 문서. 실제 Pi 센서·조종·지도 생성 시험은 아직 실행하지 않았다. 대상은 Pi 5 / Ubuntu Server 24.04 / native ROS 2 Jazzy / D435 좌우 IR / KSRC Pi↔STM다.

## 이번 시험의 질문

수동 주행 중 stereo 특징점 지도와 위치가 끊기지 않고 생성되는가? 원본을 빠짐없이 저장해 같은 입력을 다시 분석할 수 있는가? 종료 후 Atlas를 불러와 다른 주행에서도 이전 지도를 실제 이용하는가? 첫날에는 자동 주행이나 cm 정확도를 완료 조건으로 삼지 않는다.

## 확인한 사실과 아직 확인할 값

| 항목 | 현재 근거 | 현장에서 남길 것 |
|---|---|---|
| 카메라 장착 | 사용자: 아마 약 35°, 추후 20°; 현재 URDF camera joint도 35° | 수평 차체 기준 측정 각도, 장착 사진, 카메라 위치, 측정/임시 구분 |
| URDF | 실기와 매우 유사하지만 세부 차이가 있음 | 사용 URDF 사본·SHA256, 실제 센서 위치와 차이; 원본 수정 없음 |
| D435 | 과거 한 기기에서 IR 640×480, fx 377.186, baseline 49.9182 mm를 확인 | 현재 serial·firmware·USB 속도·좌우 CameraInfo·extrinsics 재확인 |
| IMU | KSRC ISM330DHCX, Pi 전달 경로 50 Hz; chip→body 변환이 decoder에 있음 | 실제 보드·축·bias·tick/t_ms·결측; D435i 내장 IMU로 표기하지 않음 |
| 조종 | 기존 노트북 pygame 화면 조이스틱 → UDP → Pi SPI → STM | 실제 실행 버전·입력 제한·연결 손실/종료 시험 결과 |
| 장소·전원·Pi 패키지 | 아직 조회하지 못함 | 실제 환경과 지원 여부; 미확인은 미확인으로 남김 |

URDF의 `imu_link_joint`는 yaw -90°, KSRC decoder는 칩 x/y 부호를 반전한다. frame 정의가 다른지 실제로 확인해야 한다. URDF만으로 IMU 축을 다시 회전시키지 않는다. [로컬 URDF](../../ros2_ws/src/rover_description/urdf/rover.urdf), [검토한 KSRC decoder](https://github.com/YunBeeeen/ksrc/blob/4bd158b1125cee32c5851f74ecc0c8f3d71013b1/pi/common/nucleo_link.py).

## 주행 전 준비

- [ ] 카메라·IMU·케이블이 고정돼 있고, 바퀴·조향 동작 중 케이블이 당겨지지 않는다. 카메라 각도 사진을 찍고 차체 수평 기준 각도를 기록한다.
- [ ] 사람이 로버에 접근해 구동 전원을 끊을 방법과 조종 정지 입력을 확인한다. 실제 구동 점검은 먼저 바퀴를 띄워 방향·조향·정지부터 확인하고 낮은 속도로 바닥 시험한다.
- [ ] Pi 입력 전원·D435 USB 전원·모터 전원 구성을 기록한다. USB 3 케이블·포트·연결 속도를 확인한다. 주행 중 USB 재연결·저전압·Pi 재부팅이 없는지 검사한다.
- [ ] 냉각과 저장 공간을 확인한다. 원본 저장 경로는 Pi의 로컬 디스크다. 노트북 연결이 끊겨도 센서 기록이 살아 있는지 정지 시험에서 먼저 확인한다.
- [ ] SSH 주소와 UDP 도달 여부를 각각 확인한다. SSH 접속 성공만으로 조이스틱 UDP나 ROS DDS 통신 성공을 판정하지 않는다.
- [ ] SPI owner는 기존 bridge 하나다. 주행 중 별도 `imu_check.py`/`spi_monitor.py`를 동시에 실행하지 않는다.
- [ ] Pi의 OS·architecture·kernel·Jazzy·RMW·librealsense·ROS wrapper·ORB commit와 patch·KSRC/STM 버전을 기록한다. 시험 직전 firmware를 자동 갱신하지 않는다.

Pi 5 공식 문서상 5 V / 5 A 전원 조건에서 USB 주변장치에 1.6 A를 제공하며, 다른 전원에서는 USB 전류가 600 mA로 제한될 수 있다. D435와 외장 저장장치까지 연결한 실제 부하로 확인해야 한다. 전류 제한을 설정으로 무조건 해제하지 않는다. [Raspberry Pi 전원 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#power-supply).

## 카메라 설정과 확인

첫 기준은 **좌우 IR 640×480 Y8, 실제 지원되는 30 Hz, ORB stereo, projector off**다. Pi 처리율이 부족하면 지원되는 15 Hz를 별도 run으로 시험한다. 요청 profile과 실제 열린 profile이 다를 수 있으므로 실제 영상 크기·encoding·rate·CameraInfo를 검사한다. RGB·depth·alignment·dense pointcloud는 첫 ORB 입력의 필수 스트림이 아니다.

공식 ORB stereo RealSense 예제는 좌우 IR과 emitter off를 사용한다. 우리는 노출 한계 숫자를 기기 지원 확인 없이 그대로 복사하지 않는다. projector 점 패턴은 자연 환경의 고정 특징으로 간주하지 않는다는 판단 아래 baseline을 off로 정한다. depth용 projector on의 유용성과 영상 SLAM의 특징점 안정성은 별도 검증 대상이다. [ORB 공식 RealSense 예제](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo/stereo_realsense_D435i.cc).

- [ ] 좌우 센서가 올바른 순서이고 가려진 렌즈·상하 반전·좌우 미러링이 없다. 화면 라벨과 stream index를 저장한다.
- [ ] 좌우 CameraInfo의 width/height, K/D/R/P, distortion model, baseline을 현재 기기에서 읽는다. 50 mm URDF baseline은 임시값이다.
- [ ] 평면 물체의 좌우 대응점이 같은 행에 놓이는지 확인한다. rectified IR을 두 번 rectify하지 않는다. CameraInfo와 ORB 설정이 같은 영상 profile을 가리켜야 한다.
- [ ] 최초에는 auto exposure로 영상 밝기를 확인하고 exposure·gain·auto 여부를 저장한다. 필요하면 별도 run에서 짧은 노출과 조명을 비교한다. 실제 지원 단위/범위를 조회하고 어두운 영상에 임의로 고정 노출값을 넣지 않는다.
- [ ] 실외는 햇빛 포화·그림자 전환을, 실내는 조명 깜박임·흐림을 확인한다. CLAHE는 원본과 구분한 처리 옵션이며 흰색 포화나 motion blur를 복원한 것으로 설명하지 않는다.
- [ ] 35°의 실제 시야에서 바닥 특징·벽·벽 끝이 보이는지 정지 화면을 저장한다. 지금 장착으로 먼저 시험하고, 20°는 다른 calibration ID로 비교한다.

wrapper 공식 문서는 profile fallback, 좌우 IR 공통 profile, 동기화·QoS·metadata·ROS/optical 축을 설명한다. `enable_sync`는 서로 다른 스트림을 같은 timetag로 보내는 동작도 포함하므로 **이름만 보고 실제 촬영 동시성을 보장한다고 해석하지 않는다**. 실제 설치 버전의 parameter dump와 per-frame metadata를 함께 보존한다. [RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md#parameters).

## 시간과 IMU

원본 sensor stamp, host 수신 ROS 시각, host monotonic 시각, MCU tick/t_ms를 각각 보존한다. 저장 파일의 수신 시각을 영상 촬영 시각으로 바꿔 쓰지 않는다. 좌우 stamp를 덮어써 ExactTime을 통과시키지 않는다.

- [ ] 먼저 좌우 stamp 차이·frame counter·누락·중복·단조 증가를 확인한다. source에서 실제 동기화된 좌우만 ORB에 입력한다.
- [ ] 실기는 `use_sim_time=False`; 녹화도 `--use-sim-time`을 사용하지 않는다. `/clock` 없는 실기에서 sim time 녹화는 데이터를 쓰지 않을 수 있다. [rosbag2 Jazzy 문서](https://github.com/ros2/rosbag2/blob/jazzy/README.md#simulation-time).
- [ ] 외부 STM IMU는 카메라와 다른 clock이다. clock offset·drift·timestamp provenance를 보존한다. 아직 동기화하지 못했으면 receipt-time만 있는 진단 기록으로 표시하고 fused 추정에 넣지 않는다.
- [ ] MCU `t_ms` wrap/reset과 tick 중복·gap을 구분한다. IMU_VALID가 없는 값을 정상 0 측정으로 채우지 않는다. chip raw 값과 body 변환값을 함께 기록한다.
- [ ] 모터 꺼짐 10초 정지, 모터 켜짐 정지, 천천히 전진·회전의 구간을 표시한다. gyro bias, acceleration norm, 진동, 실제 전달률과 방향 부호를 확인한다.
- [ ] ROS IMU는 rad/s·m/s²로 변환한다. gyro dps·acc g와 구분한다. orientation을 추정하지 않았다면 `orientation_covariance[0] = -1`로 unavailable을 표시한다. [ROS Imu 메시지](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Imu.msg).

IMU를 연결하기 전에도 visual stereo 지도 생성은 가능하다. 그 run의 manifest에 `imu_recorded=false`를 기록한다. 208 Hz 칩 ODR를 50 Hz Pi 전달률과 혼동하거나 샘플을 복제해 전달률을 높이지 않는다. 초기 구간을 정지했다고 기록하는 것과 실제 정지 관측을 확인하는 것은 별개다.

## 저장할 데이터

| 데이터 | 저장 내용 | 필요한 이유 |
|---|---|---|
| 원본 좌우 IR | 무손실 원본 Image, source stamp, frame ID | ORB 설정을 바꿔도 같은 입력으로 재시험 |
| CameraInfo·metadata·extrinsics | 좌우 K/D/R/P, device stamp/frame counter/exposure 등 제공되는 항목 | profile·노출·동기화·baseline 확인 |
| 보정 | 실제 사용 T_c1_base·stereo transform·map 정렬, pitch/xyz 출처·확인 상태 | 35°와 20° 및 URDF 업데이트 전후 분리 |
| STM telemetry | tick/t_ms, flags/IMU_VALID, raw IMU·body gyro/acc, wheel/steer feedback·valid, 전압·온도·CRC/error | 입력 진동·슬립·시간·제어 상태 분석 |
| 수동 명령 | 받은 nominal, clamp 뒤 SPI 명령, seq·수신 시각·stale 여부 | 사용자가 준 입력과 실제 적용값 분리 |
| ORB 결과 | pose·추적 상태·active map ID·유효 지도점/키프레임·map change | 추적 실패·새 지도·loop closure 확인 |
| 성능 | 센서/ORB/저장 rate, stamp gap, input age, 처리 시간, CPU/RSS·온도·disk free·주기 초과 | SLAM 정확도 문제와 데이터/계산 문제 분리 |
| run 정보 | 시작/종료 시각, 장소·조명·바닥·속도 제한·배터리, 시험 구간/사건, 버전·설정·URDF 사본 | 원인 비교·재현 |
| 산출물 | bag, Atlas, 최적화 지도 snapshot/키프레임, 궤적, 로그, summary | 화면 녹화만으로 대체할 수 없는 재분석 자료 |

이미지·CameraInfo·TF·diagnostics는 bag으로, MCU 원본/명령과 clock provenance는 같은 run의 JSONL로 보존한다. 둘을 연결하는 run ID와 시각 정보가 있어야 한다. 5 Hz joystick 요약만 저장한 경우 full telemetry가 기록됐다고 표시하지 않는다. rosbag 전체 `-a` 대신 필요한 토픽을 명시해 화면 압축 영상·dense pointcloud 중복 저장을 줄인다.

권장 run 폴더 계약(이 문서에서는 아직 생성하지 않음):

```text
run/
  manifest.json             # 정보·설정·확인 상태·종료 상태
  calibration.json         # 적용한 실제 센서 계약
  orb_settings.yaml
  robot.urdf               # 사용 당시 사본
  bag/                     # raw ROS 입력·출력
  stm_telemetry.jsonl       # 전달 가능할 때만 생성
  events.jsonl             # 정지/직선/회전/복귀/연결 손실 등
  logs/
  atlas.osa                # 저장 성공 후 확인
  atlas_input.osa          # 재사용 run의 입력 사본만
  map_snapshot.ply
  optimized_keyframes.csv
  frame_trajectory.tum
  base_trajectory.tum
  summary.json
```

MCAP은 index를 유지한 무압축 chunk 설정부터 실제 저장 부하를 확인한다. 파일 크기를 줄이는 compression은 CPU 부하와 함께 비교한다. `fastwrite`는 index 일부를 생략하므로 선택 재생·탐색이 필요한 이번 기록에 자동 적용하지 않는다. [MCAP Jazzy 저장 옵션](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_storage_mcap/README.md).

용량은 `2 × width × height × 1 byte × FPS`로 계산한 **이미지 payload만** 다음과 같다. bag/metadata·다른 토픽은 추가된다. 압축률을 미리 보장하지 않는다.

| 원본 stereo | 데이터율 | 1분 | 10분 |
|---|---:|---:|---:|
| 640×480 Y8, 15 Hz | 9.216 MB/s | 0.553 GB | 5.530 GB |
| 640×480 Y8, 30 Hz | 18.432 MB/s | 1.106 GB | 11.059 GB |

첫 run의 실제 bytes/s로 예상 남은 녹화 시간을 계산한다. 저장 경고·buffer drop·source stamp gap을 별도로 확인한다. Jazzy에서는 rolling의 message-loss 통계 CLI가 있다고 가정하지 말고 설치된 `ros2 bag record --help`와 log를 확인한다. [Jazzy record CLI](https://github.com/ros2/rosbag2/blob/jazzy/ros2bag/ros2bag/verb/record.py).

## 첫날 주행 순서

각 단계는 별도 run 또는 명확한 event 구간으로 구분한다. 아래 시간·거리·속도는 초기 시험 제안이며 실제 장소·로버 반응에 맞춰 조정한다.

1. **정지 수집 30초:** 모터를 움직이지 않고 좌우 영상·기록·보정·시각화·처리율 확인. ORB가 정지에서 초기화하지 못하면 상태를 기록하고 다음의 짧은 이동에서 검사한다.
2. **바퀴를 띄운 조종 시험:** 전진/횡이동/회전·space·focus loss·PC 창 종료·UDP 손실 때 실제 정지 확인. STM 전원 reset·bridge crash는 별도 정지 점검. 명령 timeout과 기계적 정지 거리를 같은 것으로 취급하지 않는다.
3. **짧은 직선:** 열린 공간에서 0.05 m/s 정도로 약 1 m 이동·정지·복귀. 영상 흐림·축 방향·tracking/map ID 확인.
4. **완만한 회전·작은 사각형:** 0.1 m/s 이하, 회전 0.3 rad/s 이하를 시작 제한으로 제안한다. 벽·바닥 특징이 보이는 작은 경로를 돌아 시작 부근의 모습을 다시 관측한다. 빠른 제자리 회전부터 시작하지 않는다.
5. **별도 재사용 주행:** 완전히 종료해 저장한 Atlas를 새 프로세스에 load하고 같은 장소의 다른 짧은 주행을 기록한다. 이전 keyframe ID 대응/map merge·좌표 일관성·load log를 검사한다.
6. **부하 비교:** 동일한 짧은 구간에서 센서+ORB, 여기에 녹화, 여기에 시각화를 더해 비교한다. 조종 SPI loop가 함께 실행된 상태의 주기 초과도 기록한다. 필요하면 화면 rate를 먼저 줄이고 센서 원본은 유지한다.

실외/모래라면 평탄하고 회수 가능한 작은 구간에서 먼저 시험한다. 30° 경사·긴 외벽 순회·의도적 큰 흔들림은 기본 지도/기록 확인 이후의 별도 시험이다. 이번 첫 시험에서 무조건 큰 경로를 돌 필요는 없다.

## 종료와 백업

1. 조이스틱 0 입력으로 정지하고 움직임이 멎는지 확인한다. 지도 녹화 종료를 구동 정지 명령으로 대신 쓰지 않는다.
2. ORB를 정상 종료하고 worker 종료·Atlas 저장을 확인한다. 이번 고정 commit은 `Shutdown()` worker 대기가 주석 처리되어 있어 native 빌드에 종료 대기만 복구하는 patch와 검증을 계획했다. 고정 시간 sleep만으로 저장 완료를 추정하지 않는다. [ORB Shutdown 구현](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc).
3. recorder를 정상 종료해 metadata/index를 마무리하고 `ros2 bag info`로 실제 토픽·count·duration을 확인한다. TF/CameraInfo가 없거나 좌우 count/기간이 크게 다르면 incomplete로 기록한다.
4. run 파일 크기·checksum·저장 상태를 남기고 노트북에 복사한 후 checksum을 비교한다. 복사 직후 Pi 원본을 삭제하지 않는다.
5. Pi를 정상 종료한 뒤 전원을 제거한다. `SIGKILL`·정전으로 끝난 run은 `interrupted`로 유지하고 복구한 bag에도 원래 종료 상태를 남긴다.

Atlas는 `./` + 설정 stem + `.osa`로 저장되므로 run을 working directory로 두고 `atlas` stem을 사용한다. input/output은 별도 파일이다. load 이후 새 map을 생성하는 upstream 동작 때문에 파일 load 성공과 이전 지도 위치 재사용 성공을 구분한다. [ORB Atlas 구현](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc).

## 시험 후 보고할 지표

| 질문 | 지표·증거 | 제한 |
|---|---|---|
| 원본이 충분한가 | 좌우 count/rate, frame counter gap, source stamp dt 분포, recorder 경고 | 메시지 count만으로 촬영부터 저장까지 무손실을 보장하지 않음 |
| 추적이 유지되는가 | 처리된 frame 중 OK 비율·연속 LOST 시간·재초기화·map 전환, 처리하지 못한 frame 수 | 떨어진 frame을 분모에서 제외해 좋은 숫자로 만들지 않음 |
| 지도가 정리되는가 | 유효 점/키프레임 수, loop closure 전후 snapshot, 잘못된 겹침/휘어짐 | 점 수가 많다고 정확한 지도는 아님 |
| 지연이 누적되는가 | 처리 시간·입력 age p50/p95/p99/max, sensor와 처리/기록 rate 차이, SPI overrun | 서로 다른 clock의 단순 차이를 지연이라 계산하지 않음 |
| Pi가 안정적인가 | CPU/RSS·온도·clock·전압/throttling·USB reset·저장 여유 | Ubuntu에서 일부 Pi 진단 도구가 없을 수 있음 |
| 저장·재사용되는가 | Atlas 존재·load·별도 run의 이전 지도 대응 증거·input/output checksum | 같은 녹화 재생은 실제 재사용 시험과 분리 |
| 위치가 정확한가 | 줄자로 표시한 점/외부 촬영 등 독립 reference가 있을 때만 위치 오차 | start 복귀 차이는 일관성 지표, 독립 GT 없으면 ATE/cm 정확도 미보고 |

Pi에서 `vcgencmd`가 지원되면 `measure_temp`와 `get_throttled`를 쓰며 현재 bit와 과거 발생 bit를 구분한다. Ubuntu에서 미지원이면 thermal sysfs·kernel log 등 가능한 항목을 저장하고 누락으로 표시한다. [Raspberry Pi 진단 문서](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled).

카메라 각도 변경 후에는 먼저 동일한 짧은 경로·비슷한 조명에서 35°/20°를 별도 run으로 비교한다. 새 변환을 과거 bag에 소급 적용하지 않는다. optical origin 위치도 바뀌었으면 translation까지 다시 기록한다. ORB 지도를 브러시/A*용 free-space 지도로 변환하는 일은 다음 단계다.
