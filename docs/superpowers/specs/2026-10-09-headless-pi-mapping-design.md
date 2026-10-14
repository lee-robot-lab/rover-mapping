# 간이 실기 수동 지도 생성과 노트북 시각화

작성일: 2026-10-09. 상태: 사용자 설계 승인, 구현 계획 작성 중. 사용자가 “그래 그러자”로 1차 수동 mapping 설계를 승인했고, 첫 실기 준비·기록 항목 조사와 현재 약 35° 카메라 장착을 추가했다. KSRC의 기존 Pi↔STM·수동 조종 경로를 기준으로 한다. 실행 환경은 사용자 답변에 따라 **Docker 없이 Pi 호스트의 ROS 2 Jazzy**로 확정했다. 실제 Raspberry Pi 설치·센서 실행은 아직 하지 않았다.

## 사용자 요청과 첫 목표

간이 실기 장소에는 사전 STL이 없다. 사용자가 수동 주행하며 지도를 만들고 노트북에서 영상·지도·추적 품질을 확인한다. Raspberry Pi는 GUI 없이 사용하고 SSH로 연결한다. 실제 대회에는 사전 STL이 있으므로 기존 STL 벽 보정 실험을 유지한다.

첫 목표는 수동 지도 생성 → 정상 종료·지도 저장 → 새 프로세스에서 불러오기 → 같은 장소의 별도 주행에서 지도 재사용 확인이다. 촬영 데이터는 원본으로 남겨 재생 검사할 수 있어야 한다. 지도 생성 주행 자체를 정답 궤적으로 사용하지 않는다.

## 2026-10-09 대화에서 좁힌 1차 구현 범위

사용자는 Nav2의 역할을 검토한 뒤, 먼저 ORB로 주행 중 지도를 만들고 조이스틱 등으로 수동 조종하자고 요청했다. 이번 단계에서는 Nav2를 실행하지 않는다. 조종은 기존 KSRC의 노트북 화면 조이스틱을 우선 사용한다. 이 코드는 마우스 패드와 Q/E 키를 사용하는 pygame 화면이며 물리 USB 게임패드 지원이 이미 있는 것으로 설명하지 않는다. 물리 게임패드 연결은 필요할 때 별도 입력 지원으로 추가한다.

실기 대상은 Pi의 native Jazzy다. Pi에서 원본 좌우 IR을 수집하고 ORB를 실행하는 배치를 우선 대상으로 삼되 실제 처리율은 접속 후 확인한다. 기존 계산 위치 질문은 미응답 상태이므로 이 배치를 사용자 확정 선택으로 기록하지 않는다. 실기 확인과 무관하게 로컬에서는 adapter·저장 데이터 재생 검증을 준비할 수 있다. Pi의 처리율이 부족하면 같은 출력 계약을 유지하면서 센서 기록을 노트북에서 재생하는 단계부터 검증한다.

| 경로 | 1차 구현 내용 | 완료 확인 |
|---|---|---|
| 수동 조종 | 기존 노트북 KSRC joystick → UDP → Pi의 단일 SPI owner → STM | 기존 조종 경로 재사용, firmware 변경 없음 |
| ORB 입력 | 실제 보정된 좌우 IR → 기존 C++ adapter의 `stereo` 모드; 기존 `stereo_inertial`도 선택 가능하게 유지 | IMU가 없어도 stereo 지도 생성 진행; 잘못된 모드는 시작 시 거부 |
| 좌표 | 카메라 장착 변환으로 정한 local map 축을 모든 지도·키프레임·pose에 동일 적용 | 현재 약 35° 장착을 별도 실기 보정으로 기록; 외부 절대 yaw로 표현하지 않음 |
| 실시간 출력 | `/orbslam3/map_points`의 active map 점군, `/orbslam3/keyframes`의 최적화 키프레임, 기존 camera/base pose·tracking state | pose·점군·키프레임이 같은 frame; map ID 변경 시 화면·궤적 분리 |
| 상태 출력 | map ID·점/키프레임 수·처리 시간·입력 age·동기화 실패·tracking 상태를 stamp와 함께 진단으로 발행 | 오래된 pose와 추적 실패를 정상 주행처럼 갱신하지 않음 |
| 노트북 화면 | 좌우 IR, 위에서 본 지도·궤적·현재 방향, 회전 가능한 3D 지도, 추적 상태 | 노트북에서 확인 가능한 저장 레이아웃; Pi GUI 불필요 |
| 기록·저장 | 원본 bag, 보정·모드·버전, Atlas, 최적화 지도 snapshot, 궤적·진단 | 정상 종료 후 실제 파일 검사; 새 실행의 별도 영상으로 재사용 판정 |

첫 코드 변경은 기존 `orbslam3_ros2` adapter의 stereo 선택·지도 출력·저장 설정과 실기 launch에 한정한다. ROS 센서 입력·시각화·기록을 이어 붙이고, KSRC의 STM·모터 제어는 유지한다. STM IMU의 기존 bridge telemetry 분기와 기록은 이 기본 지도 출력 검증 후 연결하며, IMU가 연결되기 전에는 IMU 입력을 기록했다고 표시하지 않는다. 실시간 ESKF·RL·자동 경로 추종은 이번 mapping 구현과 구분한다.

검증은 기존 stereo-inertial 동작 보존, IMU 없는 stereo 입력, 원본 timestamp 보존, 추적 실패·지도 전환·loop closure의 출력 갱신, 종료 후 Atlas 파일 확인과 새 세션 재사용을 포함한다. 저장 bag 재생 검증과 실제 Pi 주행 검증은 결과에 구분해 기록한다. ORB 지도는 희소 특징점 지도이며, 이 단계의 산출물을 A*용 주행 가능 지도라고 부르지 않는다.

## 첫 실기 추가 조건과 기록 기준

사용자의 실제 카메라 장착은 현재 **약 35°**, 이후 약 20°로 바꿀 예정이다. 35°는 사용자 진술과 현재 URDF의 35° joint를 근거로 한 임시값이며 실측 확정값이 아니다. 기존 시뮬레이션의 20° 선택을 실기 장착값으로 복사하지 않는다. 첫 실기는 현재 장착으로 진행하며 20° 변경을 선행 조건으로 두지 않는다.

사용자는 실기 형상이 URDF와 매우 유사하지만 세부 차이가 있고 이후 URDF를 갱신할 예정이라고 설명했다. URDF를 재생성하거나 20°로 덮어쓰지 않는다. 카메라–차체 변환·IMU 축·좌우 baseline처럼 측위에 직접 필요한 값은 별도 실기 보정 파일에 출처(`measured` 또는 `urdf_provisional`)와 확인 상태를 기록한다. 그 외 형상 차이는 첫 지도 생성의 선행 조건으로 삼지 않는다.

현재 URDF의 `imu_link_joint`는 yaw -90°이고 KSRC의 칩→차체 변환은 x/y 반전이다. 서로 다른 frame 정의일 수 있으므로 실제 축 시험으로 확인하며 이를 일치한다고 가정하거나 두 변환을 연속 적용하지 않는다. 이후 카메라를 20°로 바꾸면 각도뿐 아니라 회전 중심 때문에 optical origin의 위치가 바뀌었는지도 확인하고 새 calibration ID·run을 만든다. 이전 기록의 보정·지도 정렬은 보존한다.

첫 실기 준비·기록·종료·재사용 절차는 [첫 실기 준비와 기록 조사](../../experiments/first-real-mapping-checklist.md)에 상세히 둔다. 원본 영상·보정·노출·기기/소프트웨어 버전·명령/피드백·clock·tracking/map 상태·시스템 부하·시험 구간을 함께 기록한다. Pi 연결·테스트 장소는 아직 확인되지 않았다.

## 사용자 제공 환경과 확정 실행 환경

사용자가 제공한 세팅 문서는 Raspberry Pi 5, Ubuntu Server 24.04.5 LTS, SSH `pi@raspberrypi.local`, Docker의 Ubuntu 22.04 Jammy + ROS 2 Humble을 사용했던 기록이다. 이후 사용자는 **Humble은 이미 삭제됐으며 이번 작업은 Jazzy로 진행한다**고 명확히 답했다. 세팅 문서의 `humble`과 KSRC 문서의 `humble_rover`를 현재 실행 환경으로 사용하지 않는다. 원격 장치의 실제 설치 상태는 아직 조회하지 못했다.

실행 목표는 Pi의 Ubuntu 24.04 호스트에서 native ROS 2 Jazzy를 직접 실행하는 것이다. Docker·Humble을 설치하거나 다시 만드는 단계는 없다. Pi의 Jazzy·의존성·D435 USB·SPI 장치 권한을 접속 후 확인한다. 노트북은 시각화만 담당하면 Foxglove와 SSH로 연결한다. 노트북에도 추정·수집 ROS 노드를 실행하도록 선택할 경우 그 노트북의 Jazzy 환경을 별도로 검증한다.

2026-10-09 읽기 전용 SSH 조회는 `Could not resolve hostname raspberrypi.local: Name or service not known`으로 종료했다. 원격 OS·프로세스·센서·checkout은 아직 확인하지 못했다. 연결 때는 같은 네트워크의 실제 주소를 확인하며 host key·인증·Wi-Fi 설정을 자동 변경하지 않는다.

## 기준 저장소와 현재 확인한 코드

실기 연결의 기준은 [YunBeeeen/ksrc](https://github.com/YunBeeeen/ksrc)다. 검토한 commit은 `4bd158b1125cee32c5851f74ecc0c8f3d71013b1`이고 로컬에서는 다운로드한 snapshot만 읽었다. 실제 Pi의 checkout·실행 프로세스는 접속 후 확인한다. 새로운 STM 프로토콜이나 독립 조종 경로를 만들지 않는다.

- `pi/teleop/teleop_joystick.py --udp`: 노트북의 pygame 조이스틱이 명령을 UDP로 보낸다.
- `pi/common/udp_spi_bridge.py`: Pi의 UDP 5005 수신 → `SpiLink.exchange()` 50 Hz → STM 명령·텔레메트리 교환이다. 명령 stale 0.3초 정지, STM watchdog 500 ms, 종료 시 0 명령을 보내는 기존 경로를 보존한다.
- `pi/common/nucleo_link.py`: CRC·버전 검사, MCU `tick`/`t_ms`, `IMU_VALID`, 차체 축 변환, `gyro_dps`·`acc_g`·엔코더·조향 피드백을 이미 제공한다. 같은 IMU의 칩→차체 회전을 또 적용하지 않는다.
- 기존 bridge의 노트북 응답은 5 Hz이며 명령·조향 명령·duty·flags 요약만 포함한다. IMU·MCU timestamp·엔코더가 빠져 있어 이 응답을 localization 입력으로 쓰지 않는다.
- `firmware/CLI.md`에 `ssh pi@raspberrypi.local`, Ubuntu 24.04 호스트, ROS 2 Humble 컨테이너 `humble_rover`, `~/ksrc`가 기록되어 있다. 이 중 컨테이너 안내는 이번 실행에 적용하지 않는다. 사용자 지시대로 Pi의 native Jazzy와 실제 checkout을 조회한다. 현재 연결 가능 여부·보드 실행 버전은 문서와 별개다.
- KSRC의 `ros/mujoco_bridge.py`와 Nav2 launch는 시뮬레이션 경로다. 참값 `/odom`과 항등 `map→odom`을 실기 측위로 사용하지 않는다.

우리 저장소에서 재사용할 지도 생성·출력 코드는 다음과 같다.

- `ros2_ws/src/orbslam3_ros2/src/stereo_inertial_node.cpp`는 headless ORB 실행, 좌·우 ExactTime 수신, IMU 수신, camera/base pose·path·tracking state 발행을 구현한다. 현재 센서 모드는 `IMU_STEREO`로 고정되어 있다. 지도 점군·키프레임 지도 발행과 원격 시각화 구성이 없다.
- `sand_run.launch.py`는 `use_sim_time=True`로 고정되어 있다. 실기에서는 실제 센서 timestamp 체계와 `use_sim_time=False`를 사용해야 한다.
- 우리 기존 ORB ROS adapter는 Jazzy 기반으로 작성됐고 현재 작업 노트북에는 `/opt/ros/humble`이 있다. Pi 호스트의 Jazzy 환경에 직접 빌드·실행한다. 기존 Docker build가 통과했다는 사실을 native ARM 빌드 성공으로 간주하지 않는다. 노트북의 Humble 환경을 섞어 source하지 않고, 노트북에서 ROS 노드를 실행해야 할 경우 Jazzy 지원 환경과 의존성을 먼저 확인한다.
- 최근 성공한 경로는 `visual_orb.cpp`의 독립 ORB stereo와 `inertial_filter.py`의 ESKF를 offline으로 결합하는 방식이다. 공식 ORB inertial 초기화 성공이나 실시간 ESKF 완성을 주장하지 않는다.
- `fused_localization.py`는 현재 STL을 필수로 읽는다. 간이 실기에서 이 경로를 그대로 실행하거나 가짜 STL을 넣지 않는다.
- D435 기기 보정 기록은 `docs/handoff/gyusa-server/orbslam/camera-and-intrinsics.md`에 있다. 시뮬레이션 선택값은 차체 기준 아래 20°, 현재 실기 장착은 사용자 진술상 약 35°다. 실기 설정은 사용할 기기의 CameraInfo·baseline과 실제 장착 변환을 읽어 생성한다. 시뮬레이션 기본 intrinsics나 IMU 장착 회전을 복사하지 않는다.

## 실행 배치와 통신

주행 경로는 **노트북 KSRC 조이스틱 → UDP → Pi KSRC bridge → SPI → STM**으로 유지한다. SSH는 Pi 프로세스 관리와 시각화 연결에 사용한다. 기존 수동 조종의 UDP를 SSH 로그인으로 대체한 것으로 설명하지 않는다.

STM의 IMU·엔코더·조향 피드백은 **현재 bridge가 이미 받는 텔레메트리**에서 분기한다. 지도 생성 프로세스가 별도 `SpiLink`를 열거나 `imu_check.py`를 주행 bridge와 동시에 polling하지 않는다. SPI에는 하나의 owner를 두어 명령 sequence·텔레메트리 응답이 섞이지 않게 한다.

bridge에 붙일 추가 경로는 비차단 local telemetry 출력으로 제한한다. 별도 수집 프로세스가 받아 ROS 메시지·기록으로 변환한다. 소비자가 느리거나 종료되면 telemetry 출력 누락을 진단하고 제어 루프는 계속 진행한다. bridge 안에서 이미지 처리·SLAM·blocking logging·시각화 전송을 실행하지 않는다. 기존 5 Hz 조이스틱 응답·명령 패킷을 그대로 유지한다.

D435 좌·우 IR은 카메라 수집 경로에서 별도로 받고 IMU와 실제 clock 관계를 맞춘다. 지도 생성 호스트에서 ORB를 GUI 없이 실행한다. 노트북에서 Foxglove로 결과를 시각화한다.

SSH는 원격 명령, 로그 조회, 파일 회수, WebSocket 포트 포워딩에 사용한다. SSH 로그인만으로 ROS 2 DDS나 센서 스트림이 자동 전달되지는 않는다.

| 계산 위치 | 데이터 경로 | 선택 조건 |
|---|---|---|
| Pi에서 ORB 실행 | Pi 센서 → Pi ORB → Pi Foxglove Bridge → SSH tunnel → 노트북 화면 | Pi의 실제 처리 속도·메모리와 사용자 선택 확인 후 |
| 노트북에서 ORB 실행 | Pi 센서·기록 → 같은 네트워크의 ROS 2 입력 또는 파일 재생 → 노트북 ORB·Bridge·화면 | 첫 검증의 계산 부담을 줄이고 싶을 때 |

두 배치를 동시에 구현하지 않는다. 1차 구현은 위의 Pi 실행 배치를 대상으로 검토하며, 실제 접속·사용자 답변·처리율을 확인한 뒤 배포 위치를 확정한다. 노트북 계산을 선택하면 sensor transport도 별도로 정한다. Foxglove 화면용 WebSocket을 ORB의 원본 센서 입력 경로로 대신 쓰지 않는다. SSH만 가능한 네트워크라면 먼저 Pi 기록 → 파일 회수 → 노트북 재생 검증을 사용하며 이를 live mapping으로 표시하지 않는다.

Pi 계산 배치에서 bridge를 Pi localhost에 바인딩하고, 노트북의 localhost로 포워딩한다. 포트는 8765를 제안한다. `ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 <실제 SSH 대상>` 형태로 연결하고 노트북에서 `ws://localhost:8765`를 연다. 실제 대상은 사용자 제공 접속 정보를 사용하며 SSH 설정·키를 임의 생성하거나 바꾸지 않는다. Foxglove의 웹/데스크톱 앱 선택과 계정 요구는 설치 전에 사용자에게 안내한다.

## 추가 검토: ROS 경계·지연·RL 밑단

사용자는 ROS 사용량이 지연을 늘릴지 우려하며 이후 RL을 거쳐 밑단에 명령을 보내는 구조까지 함께 고려하도록 요청했다. 출력은 차체 속도 잔차와 바퀴별 조향·속도 직접 출력 **두 방향 모두 열어두라**고 답했다. 현재 KSRC가 두 명령 형식을 모두 지원하는 것으로 해석하지 않는다.

KSRC `sim/rl/env.py`의 현재 action은 3개 차체 속도 잔차이며 `CTRL_HZ=50`이다. 관측은 프레임당 36개 값의 50프레임 이력이다. `ros/mujoco_bridge.py`는 Nav2 명령·경로와 시뮬 차체 pose에서 경로 관측을 계산한 뒤 정책을 실행한다. 실제 RL 입력에는 추정 pose와 실제 센서값을 제공해야 하며 MuJoCo quaternion·참값 위치·바퀴 상태를 이식하지 않는다.

- ROS 경계는 센서의 외부 입력·기록, 측위·경로·명령 출력, 시각화·진단으로 잡는다. RL 관측 항목마다 subscriber를 만들어 차례로 기다리는 구조나 IMU→ROS→필터→ROS→RL→ROS→SPI의 불필요한 직렬 왕복을 만들지 않는다.
- 현재 KSRC action 방식이라면 수동/Nav2의 선택된 nominal cmd + IMU·실측 바퀴 상태·필요한 pose/경로 → Pi의 local RL inference → 차체 보정 명령 → KSRC SPI owner → STM의 기구학·제어로 연결한다. policy worker와 SPI loop는 비차단 local mailbox로 최신 입력·결과를 교환하고 image/SLAM/기록/시각화 callback이 SPI 제어를 기다리게 하지 않는다.
- RL 실험의 출력 mode는 `body_residual`과 `wheel_targets`로 구분한다. 전자는 3개 잔차를 nominal에 합쳐 기존 `(vx, vy, omega)` SPI 명령으로 보내고 STM IK를 사용한다. 후자는 차체 기준 FL/FR/RL/RR의 목표 조향각(rad)·목표 바퀴 선속도(m/s)를 보내는 별도 명령 지원이 필요하다. 직접 출력에서 STM은 목표 해석·구동을 담당하되 기존 범위 제한·조향 정렬 gate·watchdog을 유지하는 설계로 검토한다. 직접 출력 명령은 현재 펌웨어의 기능이 아니며 이번 수동 mapping 단계에서 펌웨어를 변경하지 않는다.
- 공통으로 유지할 것은 센서 수집, clock/sequence/age, 최신 입력 mailbox, inference 실행·지연 측정, 명령 owner와 진단이다. 관측 layout·정규화·previous action의 차원은 모델별 계약으로 기록한다. 3개 action policy를 8개 action policy에 그대로 연결하거나 없는 바퀴 명령을 임의 역변환해 실행하지 않는다.
- 한 제어 주기에 선택된 mode 하나만 적용한다. mode와 모델·명령 수신 capability가 맞지 않으면 실행을 거부한다. 모드별 비교는 먼저 같은 펌웨어 공통 C 코드 기반 시뮬레이션에서 하고, 향후 바퀴별 명령을 추가하는 실기 제어 변경은 시뮬레이션 검증을 먼저 권고한다.
- 50 Hz는 우선 목표 주기 20 ms이며 end-to-end 응답 보장이 아니다. 현재 policy의 1초 관측 이력은 sliding history로 매 주기 갱신하는 것이며 1초마다 추론한다는 뜻이 아니다. 실제 student 모델·정규화·history layout·각 관측의 획득 시각과 결측 처리를 확인한다.
- command와 화면은 오래된 대기열을 소진하기보다 최신 값·유효 시각을 사용한다. 반면 관성 적분용 IMU는 시각 순서대로 버퍼링하고 sample gap을 기록한다. 모든 토픽에 queue depth 1 또는 무조건 best-effort를 일괄 적용하지 않는다.
- RL 추론 결과에 입력 관측·nominal 명령의 시각/sequence를 붙여 늦은 결과·이전 명령에 대한 보정을 구분한다. 늦은 결과를 언제까지 허용하고 nominal로 복귀하거나 정지할지는 실측·실기 제어 조건으로 정한다. 기존 명령 stale와 MCU watchdog은 이 RL 유효성 판정을 대신하지 않는다.
- image 처리와 loop closure를 매 RL tick에서 기다리지 않는다. 단, 현재 경로추종 policy는 측위와 경로 관측을 사용하므로 그 지연이 무관하다고 주장하지 않는다. 최근 측위·관측 age를 기록하고 연속 local 추정과 map correction을 구분한다.
- `ros/mujoco_bridge.py`의 nominal command filter `tau=0.10 s`는 응답을 평활화하며 ROS 자체의 전달 지연과 다른 요인이다. 이는 fixed 100 ms transport delay라고 해석하지 않는다. 필터를 임의 제거하지 않고 학습 시·실기 관측/명령과의 일치를 검토한다.
- 실제 Pi에서 센서/명령 수신→관측 생성→RL 추론→SPI 전송의 p50/p95/p99, 주기 초과, 관측 age·누락을 측정한다. SLAM·녹화·지도 출력·SSH 화면을 모두 켠 조건도 비교하고 서로 다른 장치의 clock을 동기화하지 않은 차이로 end-to-end 지연을 계산하지 않는다.

ROS QoS 근거: [Jazzy 공식 QoS 문서 원문](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst). QoS의 history/depth/reliability 조절은 큐·재전송 특성을 조절하지만 전체 실행 지연이나 real-time 동작을 자동 보장하지 않는다.

## 노트북에서 확인할 화면

| 화면 | 표시할 내용 | 해석 |
|---|---|---|
| 좌·우 IR | 원본 좌/우 영상, 좌우 구분, timestamp | 실제 입력과 노출·흐림 검사 |
| 특징점 overlay | 추적에 사용되는 특징점 및 대응 상태 | 원본과 처리 영상을 구분; 녹화 입력을 덮어쓰지 않음 |
| 위에서 본 지도 | 지도 점, 키프레임, 현재 base 위치·방향, 궤적 | 실제 overhead 카메라가 아니라 SLAM 추정 결과의 투영 |
| 회전 가능한 3D 지도 | 같은 지도와 camera/base pose | 점군과 높이 분포·지도 왜곡 확인 |
| IMU | 각속도 rad/s, 가속도 m/s², valid 여부와 샘플 주기 | raw 값과 보정·추정값을 구분. 없는 추정값은 표시하지 않음 |
| 상태 | ORB state, active map ID, 특징점·지도점·키프레임 수, 처리 시간, sensor rate, 누락, 입력 시각 경과 | 추적 실패·새 지도 생성·오래된 데이터가 화면에서 드러남 |
| 기록 | run ID, bag 상태, 저장 경로, Atlas 저장·재사용 결과 | 정상 종료 또는 불완전 종료를 구분 |

첫 지도는 ORB의 희소 특징점·키프레임 지도다. 벽 mesh/STL, occupancy grid, Nav2용 지도 완성을 뜻하지 않는다. 노트북 레이아웃은 파일로 저장해 다음 접속·기록 재생에도 동일하게 연다. 원본 영상은 센서 rate로 기록하고 화면용 영상만 필요에 따라 낮은 rate·압축으로 발행한다. 압축한 화면 영상을 ORB 입력으로 재사용하지 않는다.

## 지도 생성·출력 규칙

간이 지도 생성은 독립 ORB stereo로 시작한다. IMU는 동시에 기록·진단한다. ESKF를 추가로 연결하는 단계는 구분하며, 영상 추정에 이미 사용한 IMU를 다시 독립 관측으로 취급해 융합하지 않는다. 공식 Stereo-Inertial 모드도 기존 비교 경로로 보존한다.

ORB upstream의 알고리즘·초기화 gate는 수정하지 않는다. adapter에 sensor mode 선택, 실제 시간 설정, 시각화·지도 출력만 추가한다. 조사한 고정 commit의 `System::Shutdown()`은 worker 종료 대기가 주석 처리되어 있다. 저장 완료 검증을 위해 native 빌드에 종료 대기만 복구하는 작은 patch를 따로 기록하고, 대기 실패 시 Atlas 저장 성공으로 기록하지 않는다. 이 수정은 추적·초기화·특징점 알고리즘을 변경하지 않는다. CLAHE는 기존 `raw|clahe` 선택을 명시적으로 노출하고 적용 여부를 run 정보에 기록한다.

지도 점군은 현재 프레임에 보이는 점을 단순히 누적한 결과가 아니라 active map의 현재 유효 점들로 갱신한다. 키프레임 pose도 최적화된 최신 값으로 다시 발행한다. 고정 ORB commit의 `Map`/`MapPoint` 공개 API를 사용하고 mapping/loop closing과 일관된 snapshot을 얻는 잠금 범위를 구현 시 확인한다. 무리하게 전체 점군을 영상 rate로 발행하지 않고 우선 1 Hz로 표시하며 실측한다.

loop closure와 global BA의 큰 지도 변경을 갱신하고, 실시간 pose 이력과 최적화된 키프레임 경로를 구별한다. 지도가 바뀌면 active map ID를 표시하고 서로 다른 지도들을 임의의 변환으로 이어 붙이지 않는다. 추적 실패 시 마지막 정상 pose의 timestamp를 유지하고 오래된 값임을 표시하며 정상 궤적을 새로 만들어내지 않는다.

ORB optical/world frame과 ROS body의 축 차이를 확인하고, 위에서 보는 시각화용 local map frame에 고정 변환을 명시한다. 카메라 광학 z축을 지도의 중력 z축으로 간주하지 않는다. IMU의 초기 정지 중력 방향은 시각화 정렬에 활용할 수 있지만 외부 절대 yaw가 생긴 것으로 해석하지 않는다. 모든 지도 점·키프레임·pose에 같은 변환을 적용한다.

실기 timestamp의 단조 증가와 stereo 동기화를 검사한다. 좌우 timestamp가 다르면 stamp를 덮어써 맞춘 것처럼 만들지 않는다. IMU 단위·축·실제 주기와 카메라 clock과의 관계를 확인한다. KSRC IMU를 사용한다면 208 Hz sensor ODR와 50 Hz 전달 경로를 구별한다. 샘플 복제로 200 Hz인 것처럼 만들지 않는다.

## 저장과 재사용

실행마다 새 run 폴더를 사용하고 원본 stereo·CameraInfo·IMU·관련 TF·추정 출력·진단을 rosbag으로 기록한다. bag 형식은 실행 호스트의 지원을 확인해서 정한다. Foxglove 파일 재생이 필요한 경우 MCAP 지원도 확인한다.

정상 종료 시 ORB의 설정 기반 Atlas 저장을 사용하고, 궤적·지도 점 snapshot·최적화 키프레임 경로·로그·센서 보정·모드·버전 정보를 남긴다. 궤적 파일을 재사용 가능한 Atlas라고 부르지 않는다. ORB가 자체 생성하는 Atlas 파일의 실제 위치·확장자와 저장 성공 여부를 확인한다.

고정 commit은 Atlas 파일명에 `./` 접두사와 `.osa` 접미사를 붙인다. ORB 프로세스의 작업 디렉터리를 run 폴더로 지정하고 저장 stem은 `atlas`로 사용해 실제 `atlas.osa`를 확인한다. 새 세션의 load 입력은 새 run에 복사한 `atlas_input.osa`와 입력 checksum으로 남기고 출력 `atlas.osa`를 따로 두어 원본을 덮어쓰지 않는다. 정상 종료와 새 프로세스의 load 검사 전에는 재사용 가능한 저장 성공으로 판정하지 않는다.

다음 실행은 새 run에서 저장 Atlas를 읽는다. Atlas load만으로 이전 active map에서 즉시 localization되는 것으로 가정하지 않는다. 고정 ORB 코드가 load 뒤 새 map을 만들 수 있으므로 새 세션에서 이전 지도와 대응·재위치 찾기 또는 map merge가 실제 일어났는지 상태·ID·좌표를 확인한다. 불러오기 오류나 tracking 실패를 성공으로 기록하지 않는다.

SSH·화면 연결이 끊겨도 Pi의 센서 기록이 계속되는 구성을 검증한다. 기존 수동 조종의 정지·연결 손실 처리는 별도로 확인하고 시각화 연결의 생존 여부와 혼동하지 않는다. STM 펌웨어·모터·CAN 제어는 이번 범위에서 수정하지 않는다. KSRC Pi bridge에 비차단 telemetry 출력을 추가한다면 fake SPI/socket 또는 저장 프레임을 이용해 기존 stale 정지·sequence·종료 경로와 출력 소비자 실패를 먼저 검증한 뒤 실기에 반영한다. 실기 제어 변경에 앞서 시뮬레이션 검증을 먼저 권고한다.

## 검증과 완료 기준

1. 사용자와 KSRC 문서의 SSH 대상에서 Pi checkout·실행 프로세스·architecture·OS·native Jazzy·카메라·IMU를 읽기 전용으로 확인한다. 접속 실패는 기록하고 호스트·키·실행 상태를 임의 변경하지 않는다. 계산 위치는 사용자 선택으로 확정한다.
2. recorded stereo에서 adapter를 실행해 지도 점·키프레임과 pose가 같은 frame에 표시되고 loop closure·map change가 반영되는지 확인한다. 기존 ORB/ROS 검사를 필요한 범위로 실행한다.
3. headless 호스트의 bridge → SSH tunnel → 노트북 시각화를 실제 연결한다. Pi 또는 노트북에서 GUI viewer를 구동하지 않아도 지도 생성되는지 확인한다.
4. 실제 정지·직선·회전·되돌아오는 수동 주행을 기록한다. sensor/processing rate, camera·IMU 시간 관계, 추적 상태, map ID와 누락을 보고한다. 처리 지연이 계속 쌓이면 live 성공으로 기록하지 않는다.
5. 정상 종료 뒤 Atlas·bag·보정·지도 snapshot·로그가 생성됐는지 확인한다. 새 프로세스의 별도 주행에서 지도 재사용을 검증한다.
6. 화면 재연결과 bag 재생을 확인한다. 정답 위치를 측정하지 않았으면 ATE/RMSE나 cm 정확도를 주장하지 않는다. 시작점 복귀 차이는 추정의 일관성 지표이며 절대 정확도와 다르다.

## 이번 범위 밖

사전 STL 벽 보정 방식의 교체, 관측 벽 mesh 구축, dense reconstruction, A*/Nav2용 주행 가능 지도·자동 주행·브러시 구역 계획, 공식 ORB IMU 초기화 우회, 실기 성능 수치의 시뮬레이션 대체는 포함하지 않는다. 현재 단계가 끝나면 관측 벽 지도와 자체 경로 추종·RL 연결을 다음 작업으로 검토한다. Nav2는 장애물 회피·재계획·복구가 필요한 단계에서 도입 여부를 다시 판단하며 필수 구성으로 고정하지 않는다.

## 근거

- [Foxglove ROS 2 연결·기록 재생](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2): WebSocket bridge와 노트북 앱 연결, MCAP 재생.
- [Foxglove Bridge](https://docs.foxglove.dev/docs/fleet/bridge): ROS 2용 C++ bridge.
- [OpenSSH ssh](https://man.openbsd.org/ssh): `-L` TCP 포워딩과 `-N` 터널 전용 실행.
- [ORB-SLAM3 고정 commit System.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc): Atlas 저장·불러오기와 세션 생성 동작.
- [ORB-SLAM3 고정 commit System.h](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h): tracking 정보, map change와 localization mode API.

사용자가 1차 설계를 승인했으므로 다음 단계는 [구현 계획](../plans/2026-10-09-headless-pi-mapping.md)의 검토와 실행 방식 선택이다. 현재는 제품 코드 변경·Pi 설치·실기 테스트를 실행하지 않았다. Git 저장소 생성·commit은 하지 않는다.
