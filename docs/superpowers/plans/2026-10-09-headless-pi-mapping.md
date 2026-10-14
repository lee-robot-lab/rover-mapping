# Headless Pi Manual ORB Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** KSRC 수동 조종 중 D435 stereo로 희소 ORB 지도를 생성하고, 노트북에서 관찰하며 원본·보정·Atlas를 저장해 별도 주행의 지도 재사용을 확인한다.

**Architecture:** 기존 KSRC joystick/UDP/SPI 제어 경로를 유지한다. 기존 C++ ORB ROS adapter에 visual stereo·지도 snapshot·진단을 추가하고, 별도 recorder/telemetry collector/Foxglove bridge를 연결한다. 실기 목표는 Pi의 native Jazzy이며 로컬 기록 재생과 실기 배포의 검증 결과를 구분한다.

**Tech Stack:** ROS 2 Jazzy, rclcpp/C++17, ORB-SLAM3 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`, OpenCV, Python, rosbag2 MCAP, Foxglove, 기존 KSRC `4bd158b1125cee32c5851f74ecc0c8f3d71013b1` 기준 API.

**Spec:** [승인된 설계 및 추가 실기 조건](../specs/2026-10-09-headless-pi-mapping-design.md). [첫 실기 준비·기록 조사](../../experiments/first-real-mapping-checklist.md).

**Status:** 사용자 승인 후 직접 구현·notebook native Humble 검증을 진행했다. 최신 판정은 [실행 ledger](2026-10-09-headless-pi-mapping-progress.md)와 [실행 문서](../../experiments/headless-pi-manual-mapping.md)를 따른다. 아래 체크박스는 복합 단계에 Pi 실기/미실행 검사가 포함되어 있어 일괄 완료 처리하지 않는다. 실제 Pi/Jazzy/35°·별도 주행 reuse는 미실행이며 전체 목표는 아직 완료가 아니다.

## Global Constraints

- 실행 환경: **Docker 없이 Pi 호스트의 ROS 2 Jazzy**. 노트북 Humble 환경과 섞어 source하지 않는다.
- 현재 실기 카메라: **약 35°**, 이후 약 20°. 현재값은 `urdf_provisional`; 실제 측정 완료로 표시하지 않는다.
- 원본 URDF는 수정·재생성하지 않는다. run마다 사본·checksum을 보존한다.
- 사전 STL·GT pose/depth·정답 경로를 추정기 입력으로 사용하지 않는다. 희소 ORB 지도를 free-space 지도라고 부르지 않는다.
- ORB 추적·초기화 gate·특징점 알고리즘은 수정하지 않는다. worker 종료 대기만 복구하는 patch는 파일·hash로 별도 추적한다.
- STM firmware·SPI wire protocol·모터 제어는 유지한다. SPI owner는 기존 KSRC bridge 하나다.
- telemetry 출력은 기본 비활성의 비차단 분기다. collector·disk·ROS·화면이 제어 루프를 기다리게 하지 않는다.
- 원본 source stamp를 보존한다. 미동기화 외부 IMU를 촬영 시각이 확인된 측정처럼 ORB/ESKF에 넣지 않는다.
- 정상 종료·저장 검증·새 세션 재사용 검증을 구분한다. 실패·중단은 성공으로 덮어쓰지 않는다.
- 이 폴더에 git init/worktree/commit을 만들지 않는다. 변경 파일·검증 로그로 작업을 추적한다.
- 로컬 Python CPU 검사는 `/usr/bin/python3` 3.10, conda unset을 확인한 뒤 실행한다. Pi에서는 native Jazzy의 지원 Python을 사용한다.

## Review Focus

1. 요청한 IR profile과 실제 profile 불일치: CameraInfo/영상 크기·encoding이 맞지 않으면 ORB 실행을 거부하고 원본을 유지한다. Task 1/2.
2. map merge/reset 또는 LOST 뒤 map 식별 불가: 예전 pose/점군을 새 map 정상 출력처럼 갱신하지 않는다. Task 3.
3. 종료 중 mapping/GBA worker가 살아 있음: 기다린 뒤 저장하고 timeout 시 실패·비정상 종료를 기록한다. Task 4.
4. collector가 없거나 socket/disk가 가득 참: tee는 누락을 집계하고 기존 조종/timeout/종료를 보존한다. Task 6.
5. 새 장착각·다른 CameraInfo·다른 vocabulary로 Atlas load: 이전 보정은 유지하고 입력 계약 차이를 검사하며 load와 지도 재사용을 구분한다. Task 1/4/8.

## 파일 구성

기존 adapter 클래스·실행 파일 이름은 유지한다. 새 executable 이름만 만들기 위해 기존 구조를 갈아엎지 않는다.

| 파일 | 역할 |
|---|---|
| `orbslam3_ros2/include/.../sensor_mode.hpp`, `src/sensor_mode.cpp` | 두 sensor mode의 파싱·IMU 요구 여부 |
| `include/.../map_snapshot.hpp`, `src/map_snapshot.cpp` | ORB 객체에서 유효 지도·키프레임을 값으로 복사 |
| `include/.../map_output.hpp`, `src/map_output.cpp` | snapshot의 ROS 변환·최종 PLY/CSV 출력 |
| 기존 `stereo_inertial_node.hpp/.cpp`, `main.cpp` | stereo 선택·입력 검사·pose/지도/상태·종료 연결 |
| 기존 `pose_conversions.hpp/.cpp` | map 표시 좌표와 body 변환의 합성 |
| 기존 `CMakeLists.txt`, `package.xml` | 새 source/test/diagnostic dependency·scripts/config 설치 |
| `scripts/prepare_mapping_run.py` | 센서 보정·URDF 사본·Atlas 입력·run 설정 준비; 모터/센서 실행 없음 |
| `scripts/mapping_probe.py` | 원본 입력·동기화·크기·rate·노출·시스템 상태를 정지 시험에서 수집 |
| `launch/manual_mapping.launch.py` | 명시적 센서 토픽·real time·run cwd로 ORB 실행 |
| `scripts/record_mapping_run.py` | 필요한 토픽만 MCAP 기록하고 종료·metadata 상태 보존 |
| `scripts/ksrc_telemetry_capture.py` | 단일 owner에서 분기한 telemetry의 JSONL/진단·receipt IMU 출력 |
| `patches/ksrc_telemetry_tee.patch` | 검토 버전 KSRC bridge에 optional 비차단 분기만 추가 |
| `patches/orbslam3_shutdown_wait.patch` | 고정 ORB의 worker 종료 대기만 복구 |
| `config/mapping_record_qos.yaml`, `mapping_mcap.yaml` | 원본/TF 기록 QoS와 MCAP 설정 |
| `config/manual_mapping_foxglove.json` | 좌우 영상·지도·3D·상태의 저장 레이아웃 |
| `scripts/replay_stereo_sequence.py` | 기존 EuRoC 형식 영상만 ROS로 재생하는 검증 입력 |
| `docs/experiments/headless-pi-manual-mapping.md` | 실제 실행 명령과 검증 결과·실기 미확인 사항 |

위 상대 경로 중 `orbslam3_ros2/...`와 이후 package 파일은 `/path/to/rover-mapping/ros2_ws/src/orbslam3_ros2`를 기준으로 한다. 테스트는 package의 `test/`와 `/path/to/rover-mapping/tests/headless_mapping/`에 둔다. KSRC 실제 checkout은 SSH 접속 후 확인하며 임시 검토 snapshot을 실기 checkout으로 수정하지 않는다.

## Task 0: 현재 상태·입력 계약 확인

**Files:** 결과는 `docs/experiments/headless-pi-manual-mapping.md`의 환경 표에 기록.

**Interfaces:** 이후 task가 사용할 실제 topic names·설치 버전·지원 profile·실기 calibration의 확인 상태를 만든다. SSH 실패여도 독립적인 로컬 코드 준비는 진행 가능하다.

- [ ] 기존 URDF에서 카메라 35°·IR optical frames·IMU frame 경로를 확인하고 hash를 보존한다. 실기 보정의 source를 `urdf_provisional`로 기록한다.
- [ ] 사용자가 알려준 SSH 대상에서 읽기 전용 확인을 한다. host key·인증을 자동 변경하지 않는다. motor process를 실행·종료하지 않는다.

```bash
uname -m
cat /etc/os-release
ls /opt/ros
lsusb -t
df -h
timedatectl status
```

- [ ] 실제 Pi checkout에서 버전·SPI owner 유무를 확인하고, 정지 상태의 D435 `rs-enumerate-devices -c`, `ros2 param list/get/dump`, topic `info -v`로 지원 profile·QoS·frame ID를 저장한다. 민감한 전체 환경 변수나 Wi-Fi 비밀번호를 dump하지 않는다.
- [ ] Pi 35°를 측정할 수 없다면 정확도 확인 상태를 false로 유지한다. 전체 URDF 재작성은 요구하지 않는다.

**Deliverable:** 실제 확인한 값과 미확인 값이 분리된 환경 표. 조회 실패는 명령·exit code로 남긴다.

## Task 1: 실기 보정과 run 준비

**Files:** Create `scripts/prepare_mapping_run.py`; Test `tests/headless_mapping/conftest.py`, `test_prepare_mapping_run.py`.

**Interfaces:**

```python
def validate_stereo_info(left: dict, right: dict) -> None: ...
def prepare_run(run_dir: Path, calibration: dict, urdf: Path,
                load_atlas: Path | None = None) -> dict: ...
```

`calibration`에는 좌우 실제 `CameraInfo`와 `T_c1_c2`, `T_c1_base`, `pitch_down_deg`, `extrinsic_source`, `extrinsic_verified`, `camera_profile`, `imu_clock_verified`가 들어간다. helper는 기존 `calibration.py`의 fixed-joint 합성·OpenCV YAML 규칙을 참고하되 simulated intrinsics·기본 IMU 200 Hz를 가져오지 않는다. script는 실제 수집한 입력 JSON을 받고 live sensor·SPI를 열지 않는다.

테스트 fixture는 아래의 명시적 소형 계약을 사용한다. 실제 실기 보정의 대체 입력으로 제공하지 않는다. `pytest`/`copy`/`math`/`numpy as np` import와 script 모듈 경로를 conftest에서 설정한다. `urdf` fixture는 tmp_path에 `<robot name="fixture"><link name="base_link"/></robot>`을 쓴 Path다. 센서 transform은 calibration에서 직접 받으므로 이 fixture의 형상을 센서 보정으로 사용하지 않는다.

```python
@pytest.fixture
def calibration():
    info = {'width': 640, 'height': 480,
            'k': [377.186, 0, 320.542, 0, 377.186, 241.732, 0, 0, 1],
            'd': [0, 0, 0, 0, 0], 'distortion_model': 'plumb_bob'}
    stereo = np.eye(4).tolist()
    stereo[0][3] = 0.0499182
    c, s = math.cos(math.radians(35)), math.sin(math.radians(35))
    T_base_c1 = np.eye(4)
    T_base_c1[:3, :3] = [[0, -s, c], [-1, 0, 0], [0, -c, -s]]
    T_c1_base = np.linalg.inv(T_base_c1).tolist()
    return {'left': copy.deepcopy(info), 'right': copy.deepcopy(info),
            'T_c1_c2': stereo, 'T_c1_base': T_c1_base,
            'pitch_down_deg': 35.0, 'extrinsic_source': 'urdf_provisional',
            'extrinsic_verified': False, 'camera_profile': '640x480x30:Y8',
            'imu_clock_verified': False}
```

- [ ] mismatched profile, 영/음수 focal, 잘못된 4×4 transform, baseline 0, 기존 run 덮어쓰기의 실패 테스트를 작성한다.

```python
def test_existing_run_is_not_overwritten(tmp_path, calibration, urdf):
    prepare_run(tmp_path / 'run', calibration, urdf)
    with pytest.raises(FileExistsError):
        prepare_run(tmp_path / 'run', calibration, urdf)

def test_pitch_is_recorded_as_provisional(tmp_path, calibration, urdf):
    calibration['pitch_down_deg'] = 35.0
    calibration['extrinsic_source'] = 'urdf_provisional'
    calibration['extrinsic_verified'] = False
    result = prepare_run(tmp_path / 'run', calibration, urdf)
    assert result['calibration']['pitch_down_deg'] == 35.0
    assert result['calibration']['extrinsic_verified'] is False
```

- [ ] 먼저 실패를 확인한 뒤 `Path.mkdir(exist_ok=False)`, finite/SO(3)/homogeneous 행 검사, 실제 left/right K별 ORB YAML 출력, SHA256·URDF 사본·초기 manifest 작성을 구현한다. 필수 실측 CameraInfo가 없으면 fixture 값을 대신 넣지 않는다.
- [ ] load Atlas가 있으면 `atlas_input.osa`로 복사하고 hash를 보존한다. `System.LoadAtlasFromFile="atlas_input"`, `System.SaveAtlasToFile="atlas"`를 생성한다. input/output collision은 거부한다. 같은 vocabulary hash와 영상 보정 계약을 확인하고 pitch/xyz 변화는 기록된 실제 보정으로 처리한다.
- [ ] 위 test와 `test_load_input_remains_unchanged`, `test_changed_intrinsics_requires_new_contract`, `test_camera_angle_does_not_edit_original_urdf`를 통과시킨다. 마지막 테스트는 전후 URDF hash가 같은지 검사한다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m pytest tests/headless_mapping/test_prepare_mapping_run.py -q
```

**Deliverable:** 센서 입력을 발명하지 않고 재현 가능한 run을 만드는 도구. 35°→20°에서 이전 설정을 덮어쓰지 않는다.

## Task 2: adapter의 독립 stereo와 실기 입력 검사

**Files:** Create `sensor_mode.hpp/.cpp`, `test/test_sensor_mode.cpp`; Modify 기존 node, CMake/manifest; Create `manual_mapping.launch.py`.

**Interfaces:**

```cpp
enum class SensorMode { Stereo, StereoInertial };
SensorMode parseSensorMode(const std::string & value);
bool requiresImu(SensorMode mode);
```

parameter `sensor_mode`는 `stereo`/`stereo_inertial`, 기존 node default는 `stereo_inertial`; real mapping launch는 `stereo`로 명시한다. `image_preprocessing`은 `raw`/`clahe`; CLAHE는 기존 offline clip 3, tile 8×8 계약을 재사용한다. 원본 publish/record에는 적용하지 않는다.

- [ ] 센서 모드 파싱과 IMU 없는 stereo 경로에 대한 실패 테스트를 추가한다.

```cpp
TEST(SensorMode, StereoDoesNotRequireImu) {
  EXPECT_FALSE(requiresImu(parseSensorMode("stereo")));
}
TEST(SensorMode, InvalidModeIsRejected) {
  EXPECT_THROW(parseSensorMode("rgbd"), std::invalid_argument);
}
```

- [ ] 최소 helper를 구현하고 System 생성의 enum을 선택한다. `Stereo`는 IMU subscription/interval 대기를 만들지 않고 빈 IMU vector로 `TrackStereo`를 호출한다. `StereoInertial`은 기존 버퍼·drop 동작을 유지한다.
- [ ] 영상 stamp 단조 증가, 좌우 동일 stamp, 설정된 width/height·MONO8 conversion 검사를 한다. 잘못된 영상은 count/진단에 남기고 `TrackStereo`에 넣지 않는다. 임의 stamp 재작성과 대기열 catch-up을 하지 않는다.
- [ ] launch는 `run_dir`를 cwd로 설정하고 `use_sim_time=False`, 실제 topic names·mode·보정 파일을 명시한다. 기존 `sand_run.launch.py`도 명시적 `use_sim_time` 인자를 지원하되 기본값 True를 보존한다.

```python
Node(package='orbslam3_ros2', executable='orbslam3_stereo_inertial',
     cwd=LaunchConfiguration('run_dir'),
     parameters=[{'use_sim_time': False, 'sensor_mode': 'stereo',
                  'settings_path': LaunchConfiguration('settings_path'),
                  'vocabulary_path': LaunchConfiguration('vocabulary_path'),
                  'trajectory_dir': LaunchConfiguration('run_dir')}])
```

- [ ] gtest와 ROS input probe로 “IMU 없음+stereo에서는 처리”, “IMU 없음+inertial에서는 정상 pose 미발행”, “다른 크기의 stereo 거부”, “non-increasing stamp 거부”를 실제 확인한다. 잘못된 mode/settings의 node exit code도 검사한다.

**Deliverable:** 초기화 우회 없이 기존 inertial 모드와 visual stereo 모드를 실행하는 adapter.

## Task 3: active map snapshot·최적화 키프레임·상태

**Files:** Create `map_snapshot.hpp/.cpp`, `map_output.hpp/.cpp`, `test/test_map_output.cpp`; Modify node와 pose conversions; `diagnostic_msgs` dependency 추가.

**Interfaces:**

```cpp
struct MapPointValue { std::uint64_t id; Eigen::Vector3f position; };
struct KeyFrameValue { std::uint64_t id; double stamp; Sophus::SE3f T_wc1; };
struct MapSnapshot {
  std::uint64_t map_id;
  int change_index;
  double source_stamp;
  std::vector<MapPointValue> points;
  std::vector<KeyFrameValue> keyframes;
};
std::optional<MapSnapshot> captureMapSnapshot(ORB_SLAM3::System & system,
                                              double source_stamp);
sensor_msgs::msg::PointCloud2 makeMapCloud(const MapSnapshot & snapshot,
                                         const Sophus::SE3f & T_map_orb);
geometry_msgs::msg::PoseArray makeKeyframePoses(const MapSnapshot & snapshot,
                                               const Sophus::SE3f & T_map_orb);
```

- [ ] 점·pose의 실제 출력 좌표를 확인하는 테스트를 추가한다. 같은 변환을 적용하고 비유한 점 제거와 map별 frame을 확인한다.

```cpp
TEST(MapOutput, CloudAndKeyframesUseSameFrameAndTransform) {
  MapSnapshot s{7U, 0, 1.0, {{1U, Eigen::Vector3f(1, 0, 0)}},
                {{2U, 1.0, Sophus::SE3f()}}};
  Sophus::SE3f T(Eigen::Matrix3f::Identity(), Eigen::Vector3f(0, 2, 0));
  const auto cloud = makeMapCloud(s, T);
  const auto poses = makeKeyframePoses(s, T);
  EXPECT_EQ(cloud.header.frame_id, "orb_map_7");
  EXPECT_EQ(poses.header.frame_id, cloud.header.frame_id);
  EXPECT_NEAR(poses.poses.at(0).position.y, 2.0, 1e-6);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y");
  EXPECT_NEAR(*x, 1.0F, 1e-6F);
  EXPECT_NEAR(*y, 2.0F, 1e-6F);
}
```

- [ ] `TrackStereo` 이후 유효한 tracked MapPoint의 `GetMap()`으로 map을 식별한다. null/bad·여러 map 후보·일관성 미확인은 invalid 상태로 표시하며 임의 첫 map/가장 큰 map을 고르지 않는다. 존재하지 않는 `System::GetAtlas()` API나 private 접근 매크로를 사용하지 않는다.
- [ ] 식별된 Map의 `mMutexMapUpdate`를 try-lock하고 `GetAllMapPoints/GetAllKeyFrames`에서 값만 복사한다. 각 객체의 bad/owner/finite 검사를 하고 change index·map ID를 재확인한다. mapping/GBA가 전체 snapshot 동안 정지한 것으로 가정하지 않는다. 불일치·lock 실패는 재시도 예정 진단으로 남기고 오래된 snapshot의 시각을 유지한다. pointer는 callback 밖에 캐시하지 않는다.
- [ ] snapshot rate는 우선 1 Hz; copy/lock 시간과 처리 지연을 측정한다. ROS publish·PLY/CSV 변환은 map lock을 해제한 뒤 실행한다. map 변경 시 중복 누적 대신 전체 snapshot을 교체한다.
- [ ] 각 map의 frame은 `orb_map_<id>`다. pose·점군·키프레임에 같은 저장된 `T_map_orb`를 적용하고 map별 pose history를 분리한다. initial flat-body와 임시 장착 보정으로 정렬했다는 provenance를 남긴다. Atlas 재사용에는 이전 alignment metadata를 이어받으며 새 세션 pose로 기존 map을 재원점화하지 않는다.
- [ ] pose는 tracking OK·유효 map 식별일 때만 새 source stamp로 발행한다. LOST·다중 map 후보·map reset은 진단에 남기고 마지막 정상 pose를 다시 현재 시각으로 발행하지 않는다. map switch 때 이전 Path를 이어 붙이지 않는다.
- [ ] diagnostics에는 tracking state·current/last valid map ID·point/keyframe count·source stamp/last pose age·입력/처리/drop count·map snapshot skip·처리/snapshot 시간·mode를 포함한다. 초기 0점/미추적은 “정상 위치”로 표현하지 않는다.
- [ ] gtest + 입력 재생으로 loop closure에 의한 point/keyframe 변경, map 전환의 path 분리, LOST 때 timestamp 미갱신을 확인한다. 기존 TUM frame history와 최적화된 keyframe 경로를 서로 다른 결과로 표기한다.

**Deliverable:** 현재 화면만 누적한 가짜 map이 아닌 active map snapshot, map ID와 실패가 드러나는 출력.

## Task 4: 종료·Atlas 저장·load 검증

**Files:** Create `patches/orbslam3_shutdown_wait.patch`; Modify node/main, Create `test/test_mapping_shutdown.py`와 최종 snapshot exporter 연결.

**Interfaces:** `shutdownSlam()`은 입력 처리를 종료하고 worker 종료→Atlas 저장→snapshot/trajectory→상태 보고를 수행한다. 예외가 있으면 실패 상태·nonzero exit로 남긴다. `shutdown_complete_`를 저장 성공 의미로 사용하지 않는다.

- [ ] 고정 upstream `System.cc`의 주석 처리된 worker wait를 아래 bounded wait로 복구하는 patch를 작성한다. 이미 적용/다른 source이면 patch 적용을 거부하거나 정확한 동일 patch임을 확인한다. 원본 commit·patch hash를 기록한다.

```cpp
const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
       mpLoopCloser->isRunningGBA()) {
  if (std::chrono::steady_clock::now() >= deadline) {
    throw std::runtime_error("ORB workers did not finish; Atlas not saved");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}
```

이 코드는 finish 요청 다음, SaveAtlas 앞에 둔다. `<chrono>/<thread>/<stdexcept>` 필요 여부를 확인하고 추가한다. tracking·BA·IMU 초기화 조건은 바꾸지 않는다. `System::isFinished()`는 worker 종료 확인 API가 아니므로 쓰지 않는다.

- [ ] 정상 종료 직전에 마지막 유효 map pointer는 종료 처리 안에서만 보유하고, patched worker 종료 후 최종 유효 map snapshot·최적화 keyframes를 읽는다. 빈 map/실패 run에서도 원본 bag·진단은 보존한다. 실제 shutdown 경계에서 pointer 유효성·map 변경을 검사한다.
- [ ] 원자적으로 manifest/summary를 갱신하고 `atlas.osa`가 실제 생성·nonempty인지 검사한다. `.osa` 존재만으로 load 성공을 주장하지 않는다. exporter 출력은 temporary→rename으로 저장한다.
- [ ] shutdown 중 mapping이 진행 중인 recorded sequence를 정상 종료한 후 **새 프로세스**에서 해당 Atlas를 load하고 동일 vocabulary checksum 검사·parse 오류·file 크기를 확인한다. 누락/잘못된 vocabulary/손상 파일은 명시적 실패로 기록한다.
- [ ] 별도 run 영상에서 이전 keyframe ID와의 대응·map merge·좌표 일관성을 검증한다. “기존 파일에서 세션 생성”과 “이전 map을 실제 이용”을 별도 필드로 남긴다. 실제 주행 입력이 아직 없으면 reuse 검증을 미실행으로 유지한다.
- [ ] 강제 종료·worker timeout·empty map에서 summary가 complete가 되지 않는지 확인한다. 장시간 worker 강제 종료는 native 검증 fixture에서 수행하고 실제 구동 루프와 분리한다.

**Deliverable:** race를 고정 시간 sleep으로 숨기지 않는 종료와 별도 load/reuse 검증.

## Task 5: 원본 기록과 입력/시스템 probe

**Files:** Create `mapping_probe.py`, `record_mapping_run.py`, QoS/MCAP config; Test `tests/headless_mapping/test_record_mapping_run.py`.

**Interfaces:**

```python
def build_record_command(run_dir: Path, topics: list[str],
                         qos_path: Path, storage_path: Path) -> list[str]: ...
def finalize_recording(run_dir: Path, return_code: int,
                       topic_counts: dict[str, int]) -> dict: ...
```

- [ ] 원본 topic만 명시, `--use-sim-time` 없음, shell expansion 없음, incomplete 판정에 대한 실패 테스트를 작성한다.

```python
def test_recording_has_real_time_and_explicit_topics(tmp_path):
    cmd = build_record_command(tmp_path, ['/left', '/right'],
                               tmp_path / 'qos.yaml', tmp_path / 'mcap.yaml')
    assert '--use-sim-time' not in cmd and '-a' not in cmd
    assert '/left' in cmd and '/right' in cmd

def test_missing_right_image_is_incomplete(tmp_path):
    result = finalize_recording(tmp_path, 0, {'/left': 100, '/right': 0})
    assert result['recording_status'] == 'incomplete'
```

- [ ] MCAP은 chunk/index/CRC를 유지하고 compression은 처음에 None으로 설정한다. 토픽별 QoS는 실제 publisher와 맞춘다.

```yaml
# mapping_mcap.yaml
noChunking: false
noMessageIndex: false
noSummary: false
noChunkCRC: false
compression: "None"
```

```yaml
# mapping_record_qos.yaml — 실제 remap topic으로 생성/검증
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
```

- [ ] recorder는 `subprocess.Popen(list)`로 별도 실행하고 SIGINT 정상 종료를 전달한다. bag의 실제 metadata/topic count를 읽고 원본 stereo 필수 누락을 판정한다. zero return code라도 0 frame bag은 incomplete다. recorder가 없는 distro-specific 통계 플래그를 가정하지 않는다.
- [ ] probe는 좌우 stamp/counter/영상 크기·encoding/실제 수신률·가까운 frame의 stamp 차이·입력 age를 기록한다. 양쪽 source stamp는 수정하지 않는다. CameraInfo·metadata·TF·parameter snapshot을 raw 기록과 연결한다.
- [ ] 별도 1 Hz 수집으로 CPU/RSS·thermal sysfs·disk free를 기록하고 지원되는 Pi 진단값만 추가한다. 미지원은 unavailable로 남긴다. SPI loop 안에서 `/proc`나 disk를 읽지 않는다.
- [ ] recording-only→ORB 추가→화면 추가의 부하를 구분하고 원본 30 Hz에서 실제 bytes/s·누락·age를 측정한다. disk full fixture에서 명령 경로는 유지되고 recorder는 failure로 보고되는지 확인한다.

**Deliverable:** 화면 녹화와 구별되는 원본 bag, 센서/처리/저장 누락과 시스템 성능 기록.

## Task 6: 기존 SPI owner의 optional telemetry 기록

**Files:** Create KSRC patch, collector script, `tests/headless_mapping/test_telemetry_capture.py`. patch 안에 `pi/common/telemetry_tee.py`를 추가하고 기존 `udp_spi_bridge.py`의 optional 연결만 변경한다. 기존 firmware/decoder의 wire protocol은 변경하지 않는다.

**Interfaces:** bridge의 `--telemetry-socket` optional UNIX datagram path. emitter payload는 JSON이며 schema=1, run_id, host receipt ROS/monotonic ns, UDP rx seq/age, selected command/stale, decoded telemetry, decode failure/tee drop counters를 포함한다. source MCU `tick`/`t_ms`는 원형을 유지한다.

- [ ] recorded decoded frame/fake SPI와 로컬 socket으로 absent/full/slow consumer에서 명령·stale·exit zero 보존을 검사한다. 실제 SPI는 열지 않는다.

```python
def test_absent_collector_does_not_wait(tmp_path):
    tee = TelemetryTee(tmp_path / 'missing.sock')
    assert tee.emit({'schema': 1, 'telemetry': None}) is False
    assert tee.dropped == 1
    tee.close()
```

`TelemetryTee`는 patch에 포함하는 작은 helper이며 `emit(payload: dict) -> bool`, `dropped: int`, `close() -> None`를 제공한다. `setblocking(False)`와 `sendto`를 사용한다. JSON serialization과 loop 전체 시간도 부하 검사에 포함하고 collector 재시작/누락 동안 기다리지 않는다.

- [ ] bridge는 받은 telemetry에서 분기하며 수신 실패에는 `telemetry=null`을 남긴다. 기존 `n_bad`를 모두 CRC 전용 오류로 확정하지 않는다. 5 Hz PC reply·command packet·단일 owner·기존 종료 0 명령을 보존한다. optional flag가 없으면 tee를 만들지 않는다.
- [ ] collector는 run의 JSONL에 decoded 전체 값을 저장하고 유효 IMU를 ROS 진단용 `/rover/imu/data_raw_receipt`로 발행한다. header frame은 이미 body 축 변환된 값에 맞게 `base_link`; receipt stamp를 acquisition stamp로 표시하지 않는다. `imu_clock_verified=false`이면 ORB inertial 입력으로 연결하지 않는다.

```python
gyro_rad_s = [math.radians(v) for v in telemetry['gyro_dps']]
acc_m_s2 = [9.80665 * v for v in telemetry['acc_g']]
# raw는 그대로 JSONL에 남기고 이미 적용된 chip->body 변환을 반복하지 않는다.
```

- [ ] 유효성 flag·gyro/acc 변환·중복 tick·gap·MCU reset/wrap를 시험한다. `tick`은 현 decoder에서 uint8, `t_ms`는 uint32임을 반영하고 tick wrap만으로 reset을 판정하지 않는다. 시간 동기화 미완료는 별도 상태다.
- [ ] emitter가 처리한 명령과 UDP nominal을 구분해 기록한다. 실기 반영 전 fake SPI/KSRC 기존 teleop 검증을 실행하고, 실제 바퀴를 띄운 저속·정지 시험을 먼저 권고한다. Pi checkout 버전이 다르면 patch를 강제로 적용하지 않는다.

**Deliverable:** 제어 경로를 막지 않는 full decoded telemetry/명령·시각 provenance 기록. clock 교정·live ESKF는 추가하지 않는다.

## Task 7: 노트북 시각화 연결

**Files:** Create Foxglove layout; update 실행 문서.

**Interfaces:** source 좌우 IR·map_points·keyframes·pose/path·diagnostics·receipt IMU. live connection은 Pi의 localhost WebSocket→SSH tunnel이며 UDP 조종과 별도다.

- [ ] layout에 좌/우 라벨, 위에서 본 map, 회전 가능한 3D, pose/path, 특징점 overlay, 추적/입력 age/map ID, gyro/acc, 기록 상태를 넣는다. overlay는 node에서 ORB의 공개 tracked keypoints/map associations를 사용해 `/orbslam3/tracked_image` (`sensor_msgs/Image`, left source stamp)의 별도 영상으로 발행하며 원본을 덮어쓰지 않는다. keypoint와 MapPoint vector 길이·invalid association을 검사한다. overlay의 undistorted 좌표와 출력 영상 좌표가 일치하는 rectified 입력에서만 그린다.
- [ ] 화면용 image를 필요하면 낮은 rate·compressed로 보내고 원본 sensor input/bag와 구분한다. map snapshot은 1 Hz다. 압축 영상을 ORB 입력으로 되돌리지 않는다.
- [ ] Foxglove Bridge의 설치 버전·localhost bind parameter를 확인하고 8765에 설정한다. account 필요 여부는 사용자에게 안내한다. SSH 터널을 만들고 실제 지도·좌우 영상·진단을 확인한다.

```text
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 <실제_접속_대상>
```

- [ ] map ID/frame 변경 시 고정 3D frame 선택을 해당 `orb_map_<id>`로 바꾸는 절차를 문서에 넣는다. 변환을 모르는 map들을 임의 identity TF로 합쳐 보이지 않는다.
- [ ] 화면 끊김/재연결과 bag 파일 재생을 확인한다. Pi 센서 기록은 계속 유지하고 기존 KSRC 정지 처리는 실제 입력 연결의 상태로 확인한다.

**Deliverable:** 저장 가능한 노트북 레이아웃과 실기 화면 연결 절차. 실제 UI 연결 전에는 JSON 파일 존재만으로 성공을 주장하지 않는다.

## Task 8: native 검증·첫 실기 실행·결과 보존

**Files:** Create `replay_stereo_sequence.py`; Modify 최종 실행 문서와 handoff.

**Interfaces:** publisher 입력은 기존 `sequence/mav0/cam0/data.csv`, `sequence/mav0/cam1/data.csv`, 원본 이미지와 보정뿐이다. `evaluation/ground_truth.csv`·render manifest pose는 읽지 않는다. 실제 Pi는 real launch, replay는 `use_sim_time=True`와 `/clock`를 명시적으로 구분한다. `manual_mapping.launch.py`는 `use_sim_time` 인자를 default False로 제공하고 Task 2의 예시 고정 False를 이 인자로 연결한다.

- [ ] 로컬 CPU helper 검사와 native Jazzy build/test를 실행한다. ORB include/library 경로는 실제 checkout을 CMake cache로 전달하고 tests의 hardcoded `/opt/ORB_SLAM3/...`도 같은 변수로 맞춘다. ARM 링크·Pangolin headless 의존성·patched ORB 버전을 실제 확인한다.

```text
source /opt/ros/jazzy/setup.bash
colcon build --packages-select orbslam3_ros2 --cmake-args -DORB_SLAM3_ROOT=<실제_ORB_checkout>
colcon test --packages-select orbslam3_ros2
colcon test-result --verbose
```

- [ ] 기존 142.8초 시퀀스 `/path/to/rover-mapping/outputs/localization_recovery/20261009/E1/capture/sequence`의 **센서 파일만**으로 stereo input probe와 adapter replay를 검증한다. 이 폴더가 Pi에 없으면 노트북에서만 수행하며 실기 데이터라고 부르지 않는다. 필요한 범위만 재생하고 렌더링을 다시 하지 않는다.
- [ ] 비교하기 쉬운 짧은 fixture에서 wrong image size, LOST, map transition, shutdown·load, record/replay, consumer failure를 확인한다. 공개 ORB 알고리즘은 실제 라이브러리를 호출하고 mock 결과로 mapping 성공을 주장하지 않는다.
- [ ] 실제 센서가 연결되면 checklist의 정지 수집→바퀴 띄운 조종→직선→작은 사각형→별도 reuse 순서로 진행한다. 실제 조종은 사용자가 하고 자동 motor command를 추가하지 않는다.
- [ ] manual mapping processes는 tmux 등 SSH 연결과 분리된 사용자 세션에서 실행한다. 이미 실행 중인 SPI owner가 있으면 새 bridge를 동시에 띄우지 않는다. live/replay·센서/보정 확인·IMU 기록 유무를 화면과 run manifest에 구분한다.
- [ ] 첫 실제 run은 30초부터 source stamp/rate·처리 age·저장 공간·USB/thermal 상태를 확인한다. 처리 지연이 쌓이면 화면 rate부터 줄이고 별도 run에서 지원 15 Hz를 비교한다. 50 Hz 제어 주기를 SLAM/recording/display 동시 부하로 측정한다.
- [ ] 사용자가 카메라를 20°로 바꾸면 실제 xyz/pitch를 재확인하고 새 calibration ID/run으로 동일한 짧은 경로를 비교한다. 기존 원본 URDF·35° 기록은 유지한다.
- [ ] 출력 경로·수행 명령·exit code·통과/실패/미실행을 실행 문서에 기록한다. 실기 GT가 없으면 cm 오차/ATE를 보고하지 않는다. bag/Atlas/log/checksum을 노트북으로 회수하고 원본은 보존한다.

**Deliverable:** native build·입력 재생·실기 시험·재사용 시험의 증거가 분리된 실행 결과와 재현 명령.

## 완료 판단

- [ ] IMU 없는 stereo에서 실제 ORB pose·희소 map·keyframes를 출력한다.
- [ ] pose·map·keyframes·원본 영상의 frame/stamp/장착 보정이 맞고 LOST/map change가 드러난다.
- [ ] run의 원본 bag·보정·URDF 사본·버전·진단·명령/telemetry 유무·clock provenance를 확인한다.
- [ ] 정상 worker 종료와 Atlas 파일·새 프로세스 load를 확인한다. 별도 주행의 이전 지도 활용 증거를 따로 확인한다.
- [ ] Pi 부하·녹화/시각화 재연결·조종 정지와 단일 SPI owner를 실제 시험한다.
- [ ] 실제 장치/접속이 없어 수행하지 못한 항목은 미실행으로 유지한다. 로컬 helper tests만 통과해서 전체 mapping 완료로 표시하지 않는다.

## 실행 방식 검토

이 계획은 task 사이의 연결을 순서대로 확인해야 하므로 **이 세션에서 직접 구현하는 방식**을 제안한다. git 작업이나 새 worktree는 만들지 않는다. 계획 검토와 실행 방식 답변을 받은 뒤 `superpowers:executing-plans`로 진행한다. 새 subagent는 사용자의 명시적 선택 없이 만들지 않는다.
