이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# Pi D435 실시간 화면과 ORB 실행 재개 — 2026-10-11

이전 대화의 구성은 D435를 Pi에 연결하고 Pi에서 카메라 수집·ORB 연산을 수행하며 컴퓨터에서 화면을 보는 것이다. 사용자는 카메라를 손으로 들고 이동하며 Pi 연산 가능 여부를 시험하고, 계정 없이 영상·성능 검사부터 진행하기로 했다.

## 현재 연결 상태

- SSH `pi@raspberrypi.local` 접속 확인.
- Pi의 `d435_live_camera`, `d435_live_web` tmux session 유지.
- 컴퓨터의 끊어진 SSH 터널을 복구했다. `127.0.0.1:8088` → Pi `127.0.0.1:8080`.
- 기존 좌우 영상 페이지는 `http://127.0.0.1:8089/`이며 브라우저에서 열었다.
- **현재 실시간 영상은 수신되지 않는다.** Pi `lsusb`에는 root hub만 있고 D435가 없다. 카메라 로그에는 `/dev/video0` 없음과 `The device has been disconnected!`가 기록됐다. 브라우저에도 영상이 표시되지 않는 것을 확인했다. 사용자에게 USB 재연결을 요청했다.

## Pi native 빌드

- 기존 전용 폴더 `/home/pi/handheld_mapping/runtime`에서 ORB-SLAM3 ARM 라이브러리 빌드 완료.
- 기존 카메라 전용 adapter를 Jazzy로 빌드하고 `/home/pi/handheld_mapping/ros2_ws/install/orbslam3_ros2`에 설치. 빌드 종료 코드 0, `JAZZY_ADAPTER_BUILD_COMPLETE` 확인.
- 설치 실행 파일은 ARM aarch64 ELF이며 runtime library 경로를 적용한 `ldd`에서 누락된 의존성이 없었다.
- 기존 Pi `rover_ws`, STM/SPI/모터는 사용하지 않았다.
- 소스·알고리즘은 이번 재개 과정에서 수정하지 않았다. 전용 runtime에 빌드 연결 스크립트와 녹화 재생 검사 스크립트를 추가했다. 이 폴더의 스크립트 사본은 당시 실행 기록이며 `build_adapter.sh`의 대기 PID는 당시 빌드 프로세스다.

## 저장된 실제 D435 영상의 Pi 재생 검사

실시간 카메라 연결과 독립적으로 오늘 저장한 정지 영상 MCAP을 Pi에서 1배속 재생했다. **손으로 이동하는 실시간 카메라 시험 결과가 아니다.**

- Pi native Jazzy, `/usr/bin/python3` 3.12.3, conda 없음.
- 별도 `ROS_DOMAIN_ID=65`, `/clock`와 `use_sim_time=true`로 재생. 실시간 영상 서버 domain과 분리.
- 당시 실제 CameraInfo의 rectified P와 baseline으로 `--handheld` 설정 생성. 외부 IMU와 로버 장착 변환 없음.
- 원본 MCAP의 정확히 같은 source stamp를 가진 쌍: 922. ORB 처리: 913쌍. 미처리 9쌍의 발생 위치·원인은 이 검사로 특정하지 않았다.
- 처리한 913쌍 중 tracking OK 912, 비정상 상태 1. 로그에 추적 손실 후 `Relocalized!!` 확인.
- 31개 diagnostics sample의 관측 구간 처리율 약 29.73 Hz.
- 관측된 마지막 처리 시간 p50 32.93 ms, p95 38.60 ms. 모든 프레임의 처리 시간 분포가 아니라 주기적 diagnostics sample이다.
- 재생 clock 기준 마지막 영상 age는 첫 sample 약 55 ms에서 마지막 약 208 ms로 증가했다. p95 약 214 ms. 실제 USB·Wi-Fi 화면의 end-to-end latency가 아니다. 지속적인 30 Hz 무지연 처리를 보장하는 결과로 해석하지 않는다.
- pidstat의 ORB 평균 CPU 약 175.62% (약 1.76개 CPU core), 평균 RSS 약 564 MiB. 카메라 실시간 수집·원본 녹화·화면 압축을 함께 실행한 부하가 아니다.
- 재생 및 ORB 정상 종료 코드 각각 0. Atlas 저장 완료 로그와 `tracking.json`의 `saved` 상태 확인. Atlas 재사용은 미검증.

재생 결과와 로그는 `20261011_recorded_replay01/`, 빌드 로그는 `build_native.log`, `build_adapter.log`에 보존했다. Pi 원본 MCAP은 그대로 보존한다.

## 다음 단계

D435 USB 재연결을 실제 `lsusb`와 영상 수신으로 확인한 뒤 좌우 실시간 화면을 검증한다. 새 live run에서 현재 CameraInfo·source stamp·rate를 다시 검사하고 ORB 단독 부하와 원본 기록·화면을 함께 켠 부하를 비교한다. 카메라 궤적으로 표시하며 로버 base pose로 취급하지 않는다.

## 사용자 재연결 후 점검 — 16:58~17:02 KST

- D435 인식과 브라우저 좌우 IR 표시를 잠시 확인했다. 20초 probe에서 좌우 각각 525장, 동일 source stamp 525쌍을 수신했으나 종료 전 입력이 멈췄다. 이 probe를 안정적인 20초 연속 스트림 성공으로 표시하지 않는다.
- 커널에 USB disconnect, descriptor/address 오류 `-71`, `Cannot enable. Maybe the USB cable is bad?`, completion 오류 `-75`가 반복됐다. 이후 실제 `lsusb -t`에서 D435가 **12M**로 연결된 상태를 확인했다.
- 카메라 로그에는 `IR stream start failure`와 반복되는 `Frames didn't arrived within 5 seconds`. 별도 5초 ROS Image header 수신도 timeout(code 124)이었다. 웹에 앞서 표시된 영상은 현재 갱신을 보장하지 않는다.
- `vcgencmd get_throttled`는 `0x0`이었다. 이 한 항목으로 케이블·포트·카메라 전원/기기 원인을 확정하거나 배제하지 않는다.
- 사용자에게 먼저 같은 케이블을 Pi 파란 USB 3 포트로 옮기도록 요청했다. 이미 해당 포트라면 다른 USB 3 데이터 케이블로 교체할 수 있는지 질문했다. 원인은 아직 확정하지 않았다.
- 기존 native adapter를 `d435_live_orb` tmux에서 실행해 입력 대기까지 확인했다. Pi run은 `/home/pi/handheld_mapping/20261011_live_handheld01`, `use_sim_time=false`, camera-only, stereo-only. 현재 입력이 없어 실시간 추적 성능은 미검증이다. 보정은 위 재연결 probe에서 생성했으며 연결 안정화 후 현재 CameraInfo와 다시 대조한다.
- 당시 커널/카메라/USB tree 로그는 `reconnect_usb_kernel.log`, `reconnect_camera.log`, `reconnect_usb_tree.txt`에 보존했다.
- 오류 해석 참고: [Linux kernel USB error codes](https://docs.kernel.org/driver-api/usb/error-codes.html). 해당 문서는 protocol/overflow 오류가 보통 기기·firmware·케이블 등 하드웨어 문제를 가리킬 수 있다고 설명하며 단일 원인을 특정하지 않는다.
