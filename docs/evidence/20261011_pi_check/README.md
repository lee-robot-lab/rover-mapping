이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# Pi + D435 handheld 입력 확인 — 2026-10-11

SSH key 인증으로 `pi@raspberrypi.local`에 실제 접속했다. 이번 시험은 카메라만 들고 이동하기 위한 **stereo 원본 입력 검사**다. ORB 추적·지도 생성·재로컬라이제이션은 실행하지 않았다. STM/SPI/모터/서버는 사용하지 않았으며, 로버 URDF의 35° 장착 보정은 적용하지 않았다.

## 실제 환경

- Pi: Ubuntu Server 24.04.5, aarch64, native Jazzy, Python 3.12.3, conda unset.
- D435: firmware 5.13.0.50, ROS wrapper/librealsense 4.58.4/2.58.4.
- 연결: USB 2.1 / 480 Mbps. 현재 연결에서도 양쪽 IR 640×480 Y8 30 Hz가 지원되며 실제 동시 수신을 확인했다. 장시간 이동 시험이나 depth/color 추가 부하는 미검증이다.
- 기록: stereo만 활성화, color/depth/IMU/pointcloud 비활성화, auto exposure 활성화, emitter 0. 모든 기록 metadata에서도 emitter mode 0이었다.
- Fast DDS: 기존 `fastdds_stereo_local.xml`의 SHM 32 MiB를 카메라·probe·recorder 시작 전에 동일하게 적용했다.
- 설치 확인: RealSense, MCAP, rosbag2_py, native numpy/yaml/OpenCV. clean `/opt/ros/jazzy` 환경에서 ORB adapter와 Foxglove bridge는 검색되지 않았다. `/home/pi/rover_ws/src`에는 기존 `rover_test`가 있으며 변경하지 않았다.

## 검증 결과

- 30.011초 probe: left 897장 / right 896장, 약 29.9 Hz. 마지막 왼쪽 프레임만 종료 경계에 남았다. 공통 source stamp 896쌍, nearest delta 최대 0 ns, source 순서 역전/50 ms 초과 간격/profile 변경 0.
- MCAP 전체 읽기: left 923장 / right 922장, source rate 약 30.03 Hz, 정확히 같은 source stamp 922쌍. 짝이 없는 왼쪽 1장은 오른쪽 첫 영상보다 한 프레임 앞선 시작 경계에 있고, 공통 구간 안의 짝 누락은 0.
- 양쪽 기록 영상: duplicate stamp/순서 역전/50 ms 초과 간격 0. 양쪽 metadata frame counter 1173–2094가 연속이다.
- recorder는 transport loss 1건을 보고했다. 개별 topic 원인은 해당 로그로 특정할 수 없으므로 로그를 보존한다. 위 전체 영상·metadata 검사는 공통 구간 안의 영상 누락을 발견하지 못했다는 뜻이다.
- `validate_mcap_files`의 header/record/footer 검사가 통과했다. 저장 메시지 5,554개를 끝까지 순회하고 IR 영상·CameraInfo·metadata를 역직렬화했다. 전체 payload CRC 검증과는 구분한다.
- 파일 약 543 MiB. 동시 probe/기록 중 마지막 Pi 온도 약 55.1°C. 정지 시험 결과이며 실시간 ORB 부하 결과가 아니다.
- 실제 화면은 천장/형광등을 보고 있었다. `left.png`, `right.png`는 같은 source stamp의 원본 영상이다. 이동 시험은 정면의 벽 모서리·가구 등 정적인 특징을 포함하도록 한다.

## 파일과 종료 상태

Pi 원본: `/home/pi/handheld_mapping/20261011_check01/raw_check/raw_check_0.mcap` 및 `metadata.yaml`.

Pi에는 원본 MCAP을 보존했고, 노트북 이 폴더에는 inventory, probe, metadata, parameters, camera/recorder log, 전체 읽기 audit, 실제 좌우 PNG를 복사했다. 대용량 MCAP 자체는 노트북으로 복사하지 않았다.

카메라는 SIGINT로 정상 종료했고 `Close Sensor - Done`을 확인했다. 전용 tmux session `d435_handheld_check01`은 종료됐다. Recorder는 33초 제한에서 SIGINT로 종료해 cache를 비우고 파일을 닫았다(`timeout` return code 124는 이 제어된 종료이며 recorder 오류 코드로 해석하지 않는다).

다음 실험은 사용자가 카메라를 정면으로 들고 천천히 이동하는 별도 raw run이다. 노트북 ORB 분석은 실제 CameraInfo/baseline을 사용하고 camera trajectory로 취급한다. handheld 카메라와 로버 base의 관계를 identity 또는 기존 URDF로 만들어 채우지 않는다. D435에 외부 IMU는 연결되지 않았으므로 stereo-only로 진행한다.
