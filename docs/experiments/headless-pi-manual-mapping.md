# Pi 수동 ORB 지도 생성 — 실행과 검증

> 2026-10-14: 이 문서의 35°/과거 IMU 자세는 당시 모델 기록이다. 새 run의 기구 기준은 사용자 확인 하향 20°인 [최신 URDF 갱신 기록](2026-10-14-rover-extrinsics-update.md)을 따른다. 과거 run의 보정값은 유지한다.

현재 범위는 **수동 조종 + stereo 희소 지도 + 원본 기록 + 노트북 시각화**다. Nav2/RL/live ESKF는 이번 실행에 연결하지 않는다. 희소 점군은 통행 가능 영역이나 occupancy map이 아니다.

현재 실기는 약 **35°**, `urdf_provisional`, `extrinsic_verified=false`다. 원본 URDF는 유지한다. 기존 영상 재생은 **20° 시뮬레이션 기록**이며 35° 실기 성능을 증명하지 않는다.

후속 도구: [첫 실기 자동 검사·수동 구간 표시·종료 리포트](first-real-mapping-tools.md). 기존 실행에 추가해 사용하는 명령과 판정 범위를 정리했다.

## 환경과 검증 상태

| 항목 | 확인 상태 |
|---|---|
| 노트북 | Ubuntu 22.04, native Humble, Python 3.10.12 `/usr/bin/python3`, conda unset |
| Pi 확인 | 2026-10-11 SSH 접속, Ubuntu 24.04.5/aarch64/native Jazzy, RealSense 4.58.4·MCAP 확인. handheld stereo 원본 검증 완료; Pi ORB 실행은 미실행 |
| 공식 ORB | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`; worker 종료 대기 + Atlas load cache 초기화 patch |
| native 로컬 의존성 | Pangolin 0.8, OpenCV 4.5.4, Eigen 3, ORB C++14 / adapter C++17 |
| Python 검사 | mapping65개 통과; `outputs/headless_mapping/20261009_merge_debug/python-final.log` |
| C++ 검사 / build | gtest13개·6 target 통과; 같은 폴더의 `cpp-cache-tests.log`, `adapter-cache-build.log` (colcon 집계19는 target6개도 포함) |
| native 센서 재생 | `outputs/headless_mapping/20261009_replay/lineage_validation.json`, `negative_validation.json` 및 각 run |
| 실제 조종·35° 영상·Pi 부하·별도 주행 reuse | 미실행 |
| Foxglove | 공식 SDK 0.59.0 모델로 config 직렬화 확인; app import/live 화면은 미확인 |

원본 URDF SHA256: `3c70150888955bb56e33dd45ec35e81c32449fcff2b7fe0bd7233c50aea162fa`.

## 연결 구조

```text
노트북 KSRC pygame 가상 조이스틱 → UDP → Pi 기존 bridge → SPI → STM
                                            └─ optional nonblocking tee
                                                 └─ JSONL + receipt IMU 진단
D435 좌/우 IR → C++ ORB stereo → pose / map snapshot / keyframes / diagnostics
       └─────────────────────→ 별도 MCAP recorder
ROS 토픽 → Pi Foxglove bridge(localhost) → SSH tunnel → 노트북 화면
```

조종 경로는 SLAM/기록/화면의 응답을 기다리지 않는다. SPI owner는 기존 bridge 하나다. 실제 motor command는 사용자가 입력한다. 외부 IMU는 이미 KSRC에서 body 축 변환된 값이며, 시각 동기화 확인 전에는 inertial 입력으로 연결하지 않는다.

## 1. Pi 환경 확인과 native build

아래 Pi 명령은 SSH로 접속한 **Pi의 native Jazzy 셸**에서 실행한다. 노트북 Humble setup을 섞지 않는다. 2026-10-11 `pi@raspberrypi.local` 접속과 D435 좌우 IR 원본 기록을 확인했다. 현재 STM 없이 카메라만 들고 이동할 예정이며, 아래 로버 URDF 기반 run 절차는 handheld에 그대로 적용하지 않는다. 실제 입력 검사 결과와 범위는 [handheld 기록 (별도 자료)](../data-assets.md)을 따른다. 이후 ORB/Pangolin의 Pi build 명령은 아직 실행하지 않은 절차다.

```bash
source /opt/ros/jazzy/setup.bash
uname -m
lsusb -t
df -h
ros2 pkg prefix realsense2_camera
ros2 pkg prefix foxglove_bridge
ros2 bag record --help
```

필요한 ROS dependency: `cv_bridge`, `message_filters`, `diagnostic_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `rosbag2_storage_mcap`, `foxglove_bridge`, Python numpy/yaml/OpenCV. RealSense SDK/driver는 실제 설치 버전을 먼저 확인한다.

ORB source는 위 commit과 정확히 일치해야 한다. 원본 `src/System.cc` SHA256는 `935dd45007a75d6750bb8c3b5c41f4edb91b7ef0b0944f655d4348955bbd168a`다. 다른 source에 patch를 강제로 적용하지 않는다.

```bash
# ORB_ROOT: 실제 공식 ORB source 경로; PKG: 이 adapter package 경로
sha256sum "$ORB_ROOT/src/System.cc"
patch --batch --fuzz=0 --forward -p1 -d "$ORB_ROOT" < "$PKG/patches/orbslam3_shutdown_wait.patch"
sha256sum "$ORB_ROOT/src/MapPoint.cc"
# 원본 MapPoint.cc: 99f2e631cbcedf5bd90359724d0faad64120416e0750bd1b2b78c7945ca19123
patch --batch --fuzz=0 --forward -p1 -d "$ORB_ROOT" < "$PKG/patches/orbslam3_atlas_load_cache.patch"
# ORB의 DBoW2/g2o와 ORB_SLAM3 target을 실제 native 환경에서 재빌드한다.
# Pangolin 설치 prefix는 실제 경로를 지정한다.
cmake -S "$ORB_ROOT/Thirdparty/DBoW2" -B "$ORB_ROOT/Thirdparty/DBoW2/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$ORB_ROOT/Thirdparty/DBoW2/build" --parallel 1
cmake -S "$ORB_ROOT/Thirdparty/g2o" -B "$ORB_ROOT/Thirdparty/g2o/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14
cmake --build "$ORB_ROOT/Thirdparty/g2o/build" --parallel 1
cmake -S "$ORB_ROOT" -B "$ORB_ROOT/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 -DCMAKE_PREFIX_PATH="$PANGOLIN_PREFIX"
cmake --build "$ORB_ROOT/build" --target ORB_SLAM3 --parallel 1
# 추적·BA·특징점·IMU 초기화 코드는 수정하지 않는다.
colcon build --packages-select orbslam3_ros2 --cmake-args -DORB_SLAM3_ROOT="$ORB_ROOT" -DCMAKE_PREFIX_PATH="$PANGOLIN_PREFIX"
source install/setup.bash
colcon test --packages-select orbslam3_ros2
colcon test-result --verbose
```

adapter는 종료 대기와 Atlas cache 초기화 marker symbol을 실제 공유 라이브러리에서 요구한다. patch source만 있고 오래된 unpatched binary를 링크하는 경우에도 실패한다. ORB/Pangolin 공유 라이브러리가 loader 경로에 있어야 한다. 로컬 의존성은 시스템에 덮어쓰지 않고 `/tmp/rover-native-mapping-deps`에서 빌드했다. cache patch는 역직렬화용 MapPoint 생성자의 view flag2개·scale level2개를 초기화한다. 특징점/매칭/최적화 기준은 바꾸지 않는다.

**빌드 ABI 주의:** ORB와 bundled g2o 모두 실제 compiler flags에서 C++14인지 확인한다. GCC 기본 C++17로 빌드된 g2o와 C++14 ORB를 섞었을 때 첫 map 생성 뒤 Eigen 할당/해제 경로에서 abort가 발생했다. adapter는 C++17과 `EIGEN_HAS_CXX17_OVERALIGN=0`을 사용한다. `-march`도 라이브러리와 일치시킨다. 위 Pi 명령은 절차이며 ARM/Jazzy에서 실행한 결과가 아니다.

## 2. 정지 입력 확인 → run 준비

D435의 실제 지원 profile과 topic을 먼저 조회한다. 아래 기본 topic은 wrapper namespace에 따라 달라질 수 있으므로 `ros2 topic list/info -v`, `ros2 param dump` 결과를 보존하고 명령에 실제 이름을 넣는다. IR1=left/cam0, IR2=right/cam1이다.

선택 입력은 640×480 Y8 30 Hz stereo, depth/color는 이번 ORB 입력에서 제외한다. 모래 특징점을 볼 때 IR projector pattern을 자연 특징으로 학습하지 않도록 emitter 상태·노출/게인을 기록한다. 실제 지원되는 parameter만 설정한다. 요청 profile이 아니라 CameraInfo/영상/수신률을 검사한다.

**Fast DDS로 실행할 때는 카메라를 포함한 모든 ROS 프로세스의 시작 전**, 각 SSH/tmux 셸에 아래 환경을 적용한다. 이미 실행 중인 publisher의 공유 메모리는 consumer 셸 설정만 바꿔도 커지지 않는다. 다른 RMW를 쓰는 경우 이 XML은 적용되지 않는다.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$(ros2 pkg prefix orbslam3_ros2)/share/orbslam3_ros2/config/fastdds_stereo_local.xml"
```

이 설정은 같은 호스트의 SHM segment를 participant당 32 MiB로 늘리고 UDP discovery를 유지한다. source stamp·영상·QoS는 바꾸지 않는다. 노트북 Humble에서 기본512 KiB의 stereo 누락을 재현하고 32 MiB 조건에서 개선을 확인했다. Pi Jazzy·USB·Wi-Fi 성능은 현장에서 다시 검사한다. 상세 대조 실험은 [전송·Atlas 조사 기록](stereo-transport-and-atlas-debug.md)을 따른다.

```bash
ros2 run orbslam3_ros2 mapping_probe.py --run-dir "$HOME/mapping_probe/first35" \
  --left /camera/camera/infra1/image_rect_raw --right /camera/camera/infra2/image_rect_raw \
  --left-info /camera/camera/infra1/camera_info --right-info /camera/camera/infra2/camera_info \
  --duration 30
```

`probe.json`의 양쪽 크기/mono8/rate/동일 source stamp, CameraInfo.P/R, input age와 시스템 상태를 확인한다. `--metadata-topics`에 실제 metadata topic을 추가할 수 있다. profile fallback, source stamp 불일치, 빈 CameraInfo는 먼저 해결한다. 임의 stamp 재작성이나 ExactTime 우회는 하지 않는다.

이미 rectified된 IR 입력임을 실제 확인했다면 다음으로 준비할 수 있다. 실제 CameraInfo.P에서 K와 baseline을 얻고 R을 body extrinsic에 합성한다. URDF의 fixed camera 경로는 **임시 장착 보정**으로만 사용한다. 원본 CameraInfo도 함께 저장한다.

```bash
RUN="$HOME/mapping_runs/first35_01"
ros2 run orbslam3_ros2 prepare_mapping_run.py \
  --from-probe "$HOME/mapping_probe/first35/probe.json" --rectified-input \
  --camera-profile 640x480x30:Y8 --urdf "$ROVER_URDF" \
  --vocabulary "$ORB_ROOT/Vocabulary/ORBvoc.txt" --run-dir "$RUN"
cp "$FASTRTPS_DEFAULT_PROFILES_FILE" "$RUN/dds_profile.xml"
```

원본/비rectified 입력 또는 실측 보정이 있으면 `--calibration actual_sensor.json`을 사용한다. 필수 키는 `left/right` CameraInfo, `T_c1_c2`, `T_c1_base`, `pitch_down_deg`, `extrinsic_source`, `extrinsic_verified`, `camera_profile`, `imu_clock_verified`다. `T_a_b`는 b의 좌표를 a로 변환한다. 숫자를 fixture에서 복사하지 않는다.

run은 덮어쓰지 않는다. 한번 mapping을 시작하면 `mapping.started`가 남으며 재시작은 새 run을 준비한다. 20°로 실제 변경한 후에는 새 장착 보정과 run을 만든다. 35° URDF로 20°를 측정 완료 처리하지 않는다.

## 3. 별도 세션에서 기록·추적·화면 실행

SSH 단절과 독립된 tmux 세션에서 실행한다. `RUN`, `PKG`, `ORB_ROOT`는 각 셸에 실제 절대 경로로 설정한다.

```bash
ros2 run orbslam3_ros2 record_mapping_run.py --run-dir "$RUN" \
  --qos "$PKG/config/mapping_record_qos.yaml" --storage-config "$PKG/config/mapping_mcap.yaml" \
  --required-topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
  --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
  /camera/camera/infra1/camera_info /camera/camera/infra2/camera_info /tf /tf_static \
  /orbslam3/base_pose /orbslam3/camera_pose /orbslam3/path /orbslam3/map_points \
  /orbslam3/keyframes /orbslam3/diagnostics /orbslam3/tracked_image \
  /rover/imu/data_raw_receipt /rover/telemetry_diagnostics /mapping/recording_status
```

실제 metadata topic도 `--topics`에 추가한다. CameraInfo/metadata QoS는 publisher와 맞추고 static TF는 transient_local로 기록한다. MCAP은 chunk/index/CRC를 유지하고 첫 기준은 compression None이다. stereo Y8 영상 payload만 약 1.106 GB/min이며 실제 파일 크기·남은 저장 공간은 따로 확인한다.

```bash
ros2 launch orbslam3_ros2 manual_mapping.launch.py run_dir:="$RUN" \
  vocabulary_path:="$ORB_ROOT/Vocabulary/ORBvoc.txt" image_preprocessing:=raw
# 별도 셸: 실제 bridge 설치 버전에서 address/port 인자를 확인한 후
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765
```

노트북에서 실제 SSH 대상에 `ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 USER@PI_IP`로 터널을 열고 Foxglove의 `ws://localhost:8765`에 연결한다. [공식 layout 안내](https://docs.foxglove.dev/docs/visualization/layouts)에 따라 `config/manual_mapping_foxglove.json`을 import한다. 현재 공식 안내상 layout 작성/편집에는 developer seat가 필요하다. 계정 생성/구매는 수행하지 않았다. app 버전에 따른 import 및 실제 화면은 확인이 필요하다.

처음 진단의 `current_map_id`를 보고 두 3D panel의 display/fixed frame을 `orb_map_<id>`로 선택한다. 여기서 id는 **map 원점의 최초 keyframe ID(`GetInitKFid`)**다. merge 중 바뀌는 ORB의 `GetId`는 `orb_runtime_map_id`에 별도로 기록한다. map switch 시 그 frame으로 바꾸고 이전 Path와 연결하지 않는다. 모르는 map 사이에 identity TF를 만들지 않는다. pose의 source stamp와 `last_pose_age_s`를 함께 본다. path/overlay는 5 Hz, map snapshot은 1 Hz이며 원본 영상/pose 기록과 구분한다.

## 4. KSRC telemetry와 조종

KSRC 기준은 `4bd158b1125cee32c5851f74ecc0c8f3d71013b1`. 실제 checkout 버전을 확인한 후 patch를 review/apply한다. 이 세션에서는 검토 snapshot의 새 사본과 fake SPI에만 적용했다.

```bash
# collector 먼저; 기존 SPI owner와 별개이며 SPI를 열지 않는다.
ros2 run orbslam3_ros2 ksrc_telemetry_capture.py --run-dir "$RUN" --socket /tmp/ksrc-mapping.sock
# 기존 bridge를 사용자가 실행할 때 추가하는 선택 인자:
python3 pi/common/udp_spi_bridge.py --telemetry-socket /tmp/ksrc-mapping.sock --run-id "$(basename "$RUN")"
# 노트북의 기존 pygame 조이스틱:
python3 pi/teleop/teleop_joystick.py --udp PI_IP --rate 50 --max-lin 0.15 --max-ang 0.8 --monitor
```

이미 SPI owner가 실행 중이면 새 bridge를 동시에 띄우지 않는다. 조종 변경은 **fake SPI/시뮬레이션 검증 후 실제 바퀴를 띄운 저속·정지 시험부터** 확인한다. collector가 없거나 disk/socket가 포화되어도 bridge는 nonblocking tee drop만 집계한다.

`gyro_dps`는 rad/s, `acc_g`는 m/s²로 변환하고 body 축 변환은 반복하지 않는다. IMU orientation은 unknown이다. `tick` uint8 / `t_ms` uint32와 bridge host receipt·monotonic 시각을 원본 JSONL에 보존한다. receipt topic은 acquisition 시각을 검증한 IMU가 아니다. 기본 stereo launch에는 외부 IMU를 연결하지 않는다.

## 5. 첫 주행과 저장

[첫 실기 checklist](first-real-mapping-checklist.md)의 정지 30초 → 바퀴 띄운 조종 → 저속 직선 → 작은 사각형 순서로 사용자가 운전한다. 기록만/ORB 추가/화면 추가의 입력 rate·age·CPU·온도·disk를 별도 run으로 비교한다. 성능이 밀리면 화면부터 줄이고 지원되는 15 Hz 입력은 새 run으로 비교한다. 50 Hz 제어 루프의 실제 지터는 동시 부하에서 측정해야 한다.

주행을 정지하고 ORB를 Ctrl+C로 정상 종료한다. 종료 대기는 30초 한도이며 worker가 끝나기 전에 Atlas를 저장하지 않는다. recorder와 collector도 Ctrl+C로 종료한 뒤:

```bash
ros2 run orbslam3_ros2 finalize_mapping_run.py --run-dir "$RUN"
```

`tracking.json`의 `saved`, `recording.json`의 `complete`, nonempty Atlas·최종 PLY가 있어야 manifest가 `saved_and_recorded`다. recorder는 metadata뿐 아니라 MCAP 파일의 header·record 길이·DataEnd·footer·summary offset·tail을 검사한다. 이는 전체 payload CRC 검증과는 다르다. 빈 지도·저장 실패·SIGKILL 흔적은 incomplete/failed다. `saved_and_recorded`도 별도 주행 reuse 성공을 뜻하지 않는다.

| 산출물 | 의미 |
|---|---|
| `raw/*.mcap` + metadata | 원본 stereo/보정/ROS 출력 |
| `calibration.json`, `orb.yaml`, `robot.urdf`, `manifest.json` | 장착각·입력 계약·checksum·단계별 상태 |
| `atlas.osa` / `atlas_input.osa` | 새 Atlas / 보존된 load 입력 |
| `map_alignments.json` | map별 표시 변환, initial body pose와 장착 보정의 provenance |
| `map_<id>.ply`, `map_<id>_keyframes.csv` | 종료 시 해당 map의 점과 최적화된 camera keyframes |
| `base_map_<id>.tum` | 그 시점에 발행했던 body pose history; loop 최적화된 최종 궤적과 다름 |
| `tracking.json`, `recording.json`, `telemetry.json[l]`, log | 실패·누락·저장·clock provenance |

새 프로세스 load는 `prepare_mapping_run.py ... --load-atlas OLD_RUN/atlas.osa`로 **새 run**에 준비한다. 입력은 복사하고 같은 vocabulary/영상 보정 계약을 확인한다. 이전 alignment를 이어받고 새 세션 시작 위치로 이전 map을 재원점화하지 않는다. `map_alignment_identity=origin_keyframe_id_v1`이 없는 옛 mutable map ID 기준 run은 거부한다. 실제 parse/load와 이전 map 원점의 tracking/merge를 따로 확인한다. 새로운 장소/별도 주행의 reuse는 아직 확인하지 않았다. 첫 실기는 새 지도 생성부터 진행한다.

## 로컬 재생 재현

기존 `outputs/localization_recovery/20261009/E1/capture/sequence`의 cam0/cam1 CSV·PNG만 사용한다. GT·render pose·정답 경로·depth를 읽지 않는다. replay에서만 sim time을 켠다.

```bash
ros2 launch orbslam3_ros2 manual_mapping.launch.py run_dir:="$REPLAY_RUN" \
  vocabulary_path:="$VOCABULARY" use_sim_time:=true \
  left_topic:=/rover/stereo/left/image_raw right_topic:=/rover/stereo/right/image_raw
ros2 run orbslam3_ros2 replay_stereo_sequence.py --sequence "$SEQUENCE" --max-pairs 360 --rate 0.5 --min-subscribers 2
```

`--min-subscribers 2`는 좌우 모두 ORB와 recorder discovery가 끝난 뒤 입력을 시작한다. `rate=0.5`는 오프라인 기능 검증용 재생 속도다. 실기 30 Hz 처리 성능으로 해석하지 않는다.

## 2026-10-09 로컬 실행 결과

최종 native Humble/x86 빌드·gtest 12개(5 targets), Python 47개를 현재 source로 다시 실행해 통과했다. C++14 g2o 재빌드 전 abort 3회도 로그/실패 bag을 보존했다. 최종 기능 검증은 raw 전처리, IMU 없는 stereo, 기존 **20° 시뮬레이션 기록의 일부**이며 위치 정확도 시험이 아니다.

| run | 입력 / 발행 pose / MCAP 좌·우 | 결과 |
|---|---|---|
| `stereo05` | 360 / 360 / 360·360 | 새 지도, Atlas·PLY·keyframes 저장, `orb_map_0` |
| `load05` | 480 / 480 / 480·480 | 새 프로세스 load, 90 frame 뒤부터 재생, `orb_map_60`→이전 `orb_map_0` 병합, 이전 alignment 정확히 유지 |
| `invalid_stereo` | 72 / 60 / 기록 미실행 | 중복 stamp·잘못된 크기 2개 거부, blank 10개에서 RECENTLY_LOST, pose stamp 미갱신, incomplete/exit 1 |
| `inertial_no_imu` | 60 / 0 / 기록 미실행 | IMU interval 60개 거부, incomplete/exit 1 |
| `storage_failure02` | 기록만 30쌍 / pose 없음 | 파일 크기 1 MB 제한으로 MCAP 잘림, metadata 60개·child exit0이어도 wrapper failed/exit1 |
| `recording_verified01` | 기록만 30쌍 / pose 없음 | 수정된 recorder로 좌우30개·정상 MCAP complete/exit0; 지도 미실행이라 전체 run은 incomplete |

두 최종 run은 `saved_and_recorded`이며 source pose stamp 역행과 좌우 원본 누락은 0이다. `load05`의 `atlas_load_verified=true`, **`atlas_reuse_verified=false`**를 유지한다. 동일 기록을 시작 위치만 바꿔 재생했으므로 별도 실기 주행 재사용을 증명하지 않는다. 전체 142.8초를 재시험한 결과도 아니다.

저장 실패 시험은 실제 disk-full 마운트 대신 child `RLIMIT_FSIZE=1048576`을 사용했다. 첫 시험 `storage_failure01`에서 잘린 MCAP인데 rosbag metadata/exit code만으로 complete 처리하는 결함을 발견했다. 실패 테스트를 먼저 추가하고 container 검사를 구현한 뒤 storage_failure02에서 실패 판정을 확인했다. 기존 stereo05/load05 MCAP도 container 검사를 통과했다. 초기 잘못된 complete 판정 자료는 조사용으로 보존하며 성공 결과로 집계하지 않는다.

지도 미리보기: [stereo05_map_preview.png (별도 자료)](../data-assets.md). 최종 최적화 점군과 실시간 발행 body pose history를 함께 그렸고 GT나 최종 최적화 궤적으로 표시하지 않았다. 재생성은 `PYTHONNOUSERSITE=1 /usr/bin/python3 outputs/headless_mapping/20261009_replay/plot_saved_map.py`로 한다.

1 Hz 진단에서 표본으로 읽은 `process_ms` 중앙값/최댓값은 stereo05 9.49/11.07 ms, load05 10.28/12.59 ms다. source 30 Hz를 wall time 15 pairs/s로 재생한 notebook 값이며 전체 지연·지터·Pi 성능을 뜻하지 않는다.

**병합 종료 후속:** `load04`의 같은 Atlas를 공식 API에 직접 재생해 병합 단계 SIGSEGV를 재현했다. 저장한 core에서 loaded MapPoint의 초기화되지 않은 view flag/scale level로 배열 밖을 읽는 것을 확인했다. 생성자 네 값 초기화 patch와 수정 전 실패/수정 후 통과하는 회귀 검사를 추가했고 같은 Atlas 반복6회는 모두 정상 병합·저장이었다. [전송·Atlas 조사 기록](stereo-transport-and-atlas-debug.md)에 원본 실패와 수정 후 자료를 함께 보존했다. 별도 실제 주행 reuse는 계속 미검증이다.

실제 Pi 접속·Jazzy/ARM 빌드·35° IR 보정·motor/50 Hz 동시 부하·USB/온도·Foxglove import/재연결·별도 실기 주행은 미실행이다. 실측값이 없으므로 cm 오차나 ATE를 보고하지 않는다. 다음 실기 절차는 Pi 연결 후 정지 30초 probe부터다.

결과 모음은 `outputs/headless_mapping/20261009_replay/verification_summary.json`, source/library hash는 `implementation_checksums.json`에 있다. 실제 실행한 검증 driver는 같은 폴더 `verification_tools/`에 보존했다. 이 driver들은 당시 절대 경로/run 이름을 고정한 조사 자료이므로 그대로 재실행하지 말고 새 run 이름을 사용한다. 기존 실패 run도 유지한다.
