# 설치 설명서

기준: Ubuntu 22.04/Python 3.10 노트북, Ubuntu 24.04/ROS 2 Jazzy Pi. 먼저 Python-only 합성 경로를 확인하고 실제 촬영에 필요한 native/Pi 자산을 준비한다. 새 PC의 전체 native·RealSense 설치를 자동 완료하는 저장소는 아니다.

## 1. clone과 노트북 Python 환경

접근 권한이 있는 계정으로 clone하고 저장소 루트에서 실행한다.

```bash
git clone git@github.com:lee-robot-lab/rover-mapping.git
cd rover-mapping
sudo apt update
sudo apt install python3-venv python3-pyqt5 python3-opencv python3-matplotlib python3-pytest python3-yaml
PYTHONNOUSERSITE=1 /usr/bin/python3 -m venv --system-site-packages .venv-mapping
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pip install -r requirements.txt
```

Conda 환경을 활성화하지 않는다. `--system-site-packages`는 apt의 PyQt/OpenCV 등을 사용하기 위함이다. 현재 mapping 요구 버전은 numpy1.26.4/scipy1.13.1/Open3D0.19.0/pygame2.6.1이다. 이미 정상인 환경을 다시 설치할 필요는 없다.

장비 없이 확인:

```bash
bash tools/handheld_mapping.sh --demo
bash tools/terrain_mapping.sh --demo --output outputs/terrain_demo_01
```

GUI에서 매핑 시작 → 잠시 대기 → 종료 및 지도 생성 → 3D 지도 보기 순서로 확인한다. 합성 입력은 알려진 pose를 쓰고 native ORB가 없어도 TSDF를 생성한다. terrain output은 새 폴더를 지정한다.

## 2. native 자산 준비: 기존 검증 환경에서 전달받기

실제 노트북 ORB/live를 사용하려면 아래 자산이 clone 경로 아래에 있어야 한다. [별도 데이터·runtime 안내](data-assets.md)를 참고한다.

```text
outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare
outputs/handheld_mapping/orb3_comparison_build/runtime/*.so
outputs/handheld_mapping/native_dependencies/pangolin/
external/ORB-LINE-SLAM/Vocabulary/ORBvoc.txt.tar.gz
external/ORB-LINE-SLAM/Vocabulary/LSvoc.txt.tar.gz
```

같은 OS/아키텍처/라이브러리 ABI에서 기존 검증 자산을 복사하는 경로가 있다. `-march=native` 산출물은 다른 CPU에서 재빌드가 필요할 수 있다. 파일이 있다는 이유만으로 다른 머신의 실행을 검증했다고 간주하지 않는다. vocabulary text가 없으면 코드가 해당 tar에서 꺼낸다. 현재 point도 line vocabulary 디렉터리를 사용한다.

선택적 line 실행기와 library는 `outputs/handheld_mapping/orbline_build`, `external/ORB-LINE-SLAM/Thirdparty/g2o/lib`에 있다.

## 3. native 소스 재빌드 경로

이 절은 pinned upstream과 저장소의 build script/patch를 조합하는 절차다. 공유 준비에서 **stock source에 대한 patch dry-run**은 확인했지만, 모든 새 PC에서 아래 전체 컴파일·실행을 다시 검증한 것은 아니다. compiler/메모리/그래픽 의존성 차이는 로그로 확인한다.

### 의존성과 외부 소스

```bash
sudo apt install build-essential cmake git pkg-config libopencv-dev libeigen3-dev \
  libboost-serialization-dev libssl-dev libglew-dev libepoxy-dev libgl1-mesa-dev \
  libegl1-mesa-dev libwayland-dev libxkbcommon-dev libpng-dev libjpeg-dev

mkdir -p external
git clone https://github.com/stevenlovegrove/Pangolin.git external/Pangolin
git -C external/Pangolin checkout aff6883c83f3fd7e8268a9715e84266c42e2efe3
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git external/ORB_SLAM3
git -C external/ORB_SLAM3 checkout 4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4
git clone https://github.com/Giannis-Alamanos/ORB-LINE-SLAM.git external/ORB-LINE-SLAM
git -C external/ORB-LINE-SLAM checkout a351dbd1e7cfdba622891723d1b3533359f92264
```

소스가 이미 있으면 clone을 중복 실행하지 않는다. 기존 변경을 지우지 않고 commit과 patch 상태를 확인한다.

### Pangolin과 point core

이후 명령은 저장소 루트에서 실행한다. `MAPPING_REPO_DIR`는 이 절차에서만 쓰는 변수다.

```bash
MAPPING_REPO_DIR="$PWD"
cmake -S external/Pangolin -B outputs/handheld_mapping/pangolin_build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$MAPPING_REPO_DIR/outputs/handheld_mapping/native_dependencies/pangolin" \
  -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
cmake --build outputs/handheld_mapping/pangolin_build --parallel 2
cmake --install outputs/handheld_mapping/pangolin_build

git -C external/ORB_SLAM3 apply --check "$MAPPING_REPO_DIR/third_party/patches/orbslam3-cxx14.patch"
git -C external/ORB_SLAM3 apply "$MAPPING_REPO_DIR/third_party/patches/orbslam3-cxx14.patch"
git -C external/ORB_SLAM3 apply --check "$MAPPING_REPO_DIR/ros2_ws/src/orbslam3_ros2/patches/orbslam3_shutdown_wait.patch"
git -C external/ORB_SLAM3 apply "$MAPPING_REPO_DIR/ros2_ws/src/orbslam3_ros2/patches/orbslam3_shutdown_wait.patch"
git -C external/ORB_SLAM3 apply --check "$MAPPING_REPO_DIR/ros2_ws/src/orbslam3_ros2/patches/orbslam3_atlas_load_cache.patch"
git -C external/ORB_SLAM3 apply "$MAPPING_REPO_DIR/ros2_ws/src/orbslam3_ros2/patches/orbslam3_atlas_load_cache.patch"
git -C external/ORB_SLAM3 apply --check "$MAPPING_REPO_DIR/ros2_ws/src/orbslam3_ros2/patches/orbslam3_stereo_recovery_wait.patch"
git -C external/ORB_SLAM3 apply "$MAPPING_REPO_DIR/ros2_ws/src/orbslam3_ros2/patches/orbslam3_stereo_recovery_wait.patch"

for MAPPING_LIBRARY in DBoW2 g2o; do
  cmake -S "external/ORB_SLAM3/Thirdparty/$MAPPING_LIBRARY" \
    -B "external/ORB_SLAM3/Thirdparty/$MAPPING_LIBRARY/build" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=14 \
    -DCMAKE_CXX_FLAGS="-march=native -DEIGEN_HAS_CXX17_OVERALIGN=0"
  cmake --build "external/ORB_SLAM3/Thirdparty/$MAPPING_LIBRARY/build" --parallel 1
done

cmake -S external/ORB_SLAM3 -B external/ORB_SLAM3/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$MAPPING_REPO_DIR/outputs/handheld_mapping/native_dependencies/pangolin" \
  -DCMAKE_CXX_FLAGS="-march=native -DEIGEN_HAS_CXX17_OVERALIGN=0"
cmake --build external/ORB_SLAM3/build --target ORB_SLAM3 --parallel 1
bash tools/build_mapping_baseline.sh "$MAPPING_REPO_DIR/external/ORB_SLAM3"
```

patch가 이미 적용됐으면 `apply --check`는 실패한다. 실패를 무시해 이중 적용하지 않고 `git diff`와 대상 marker를 확인한다. DBoW2/g2o/core/runner의 Eigen ABI와 arch 설정을 맞춘다. 빌드 메모리를 줄이기 위해 parallel1을 기준으로 한다.

노트북 stereo 설정은 `Tracking.StereoRelocalizationWaitSeconds: 15.0`을 사용한다. 기존 설치도 네 번째 patch 적용 후 core와 비교 실행기를 재빌드해야 한다. 실행기는 `orbslam3_has_stereo_recovery_wait` marker가 없는 core를 거절하며, build wrapper도 이를 검사한다. 복구 대기 설정을 생략한 core 사용처는 기존 stereo 3초 정책을 유지한다. IMU 관성 융합의 정책은 바꾸지 않는다.

### 선택적 point+line

```bash
git -C external/ORB-LINE-SLAM apply --check "$MAPPING_REPO_DIR/experiments/handheld_mapping/orbline/shutdown_wait.patch"
git -C external/ORB-LINE-SLAM apply "$MAPPING_REPO_DIR/experiments/handheld_mapping/orbline/shutdown_wait.patch"
bash tools/build_orbline.sh
```

runner load는 [사용 설명서](user-guide.md)의 회귀 명령으로 확인한다. native 설치 후 GUI를 다시 열어 live/노트북 ORB checkbox가 활성화됐는지 확인한다. point+line은 선택적 비교이며 첫 실기의 필수 조건은 아니다.

## 4. Pi의 기존 환경

Pi에는 Ubuntu24.04/native Jazzy/Python3.12, D435 driver와 아래 실제 runtime이 필요하다. controller가 자동 배포하는 것은 Python worker/helper뿐이다.

| 경로 | 필요한 내용 |
|---|---|
| `/opt/ros/jazzy/setup.bash` | native ROS2 Jazzy |
| `/home/pi/handheld_mapping/runtime` | 기존 native/runtime |
| `/home/pi/handheld_mapping/live/fastdds_stereo_local.xml` | DDS profile |
| `/home/pi/handheld_mapping/ros2_ws/install` | Pi ORB 모드를 사용할 때의 adapter |
| `/home/pi/ksrc` | STM 조종 시의 기존 bridge/firmware 프로젝트 |

새 Pi에 ROS/RealSense/firmware를 설치하는 작업은 이번 공유에서 재실시하지 않았다. 운영 Pi를 인계받거나 위 구성을 실기에서 준비한다. Pi에 Humble을 source하지 않는다. 확인된 device/SDK/배선은 [인수인계서](handoff/d435-rover-mapping-handoff.md)에 있다.

노트북에서:

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 pi@raspberrypi.local hostname
```

hostname이 조회되지 않으면 현재 Pi IP로 시도하고 GUI 연결 대상도 같은 값으로 바꾼다. 촬영 전 기존 camera/ORB와 SPI owner의 중복을 확인한다. 실제 시작·종료 절차는 [사용 설명서](user-guide.md)를 따른다.
