# 실행 기록 — 계획: /path/to/rover-mapping/docs/superpowers/plans/2026-10-09-headless-pi-mapping.md

2026-10-09 사용자 “진행해”: 계획 승인, 이 세션에서 직접 구현.

## 사전 확인과 결정

- Ruling: Git 기반 workspace/commit 도구 대신 이 ledger와 검증 로그를 유지한다 — 이 디렉터리는 git repo가 아니고 사용자가 git init/worktree/commit을 금지했다 — 재개 시 변경 파일과 로그를 함께 확인해야 한다.
- Ruling: 새 subagent 없이 직접 구현·최종 self-review한다 — 승인된 계획의 실행 방식과 사용자 제약을 유지한다 — 독립 리뷰가 없으므로 실패 입력·저장/재사용 검증에 중점을 둔다.
- Task 0: 로컬 Ubuntu 22.04, /usr/bin/python3 3.10.12, conda unset, ROS Humble 확인. Pi native Jazzy와 SSH 주소는 미확인; 로컬 구현은 진행한다.
- Pre-flight: Task 1 run/calibration → Task 2 launch/settings → Task 3 map alignment → Task 4 Atlas: map별 alignment metadata와 input/output 파일 분리가 필요하다.
- Pre-flight: Task 5 recorder/probe → Task 6 telemetry → Task 7 display: raw receipt IMU는 진단용이고 clock verified 이전에 inertial에 연결하지 않는다.
- Pre-flight: Task 8 local sensor replay → real launch: sim-time 인자를 replay 때만 True로 설정한다.

## 상태

Task 1–7의 로컬 코드·설정 구현과 Task 8의 notebook native sensor replay를 진행했다. 실제 Pi/Jazzy/ARM·35° 조종·별도 주행 reuse·Foxglove UI는 미실행이며 계획 전체를 완료로 표시하지 않는다. 최신 명령과 결과는 [실행 문서](../../experiments/headless-pi-manual-mapping.md)를 따른다.

## 구현·로컬 검증

- Task 1: 보정/run 준비 구현. 실패 테스트 → 통과. 기존 run 거부, SE(3)/K/profile, Atlas 계약과 입력 보존, URDF checksum 확인.
- Task 2 초기: sensor mode helper 실패 → gtest 통과. 독립 stereo 입력 경로·real/replay launch 추가. 후속 native 재생 결과는 아래에 기록.
- Task 3: snapshot·map별 alignment/Path·PLY/CSV·diagnostics·overlay 연결. map output 좌표/NaN gtest 통과. worker snapshot은 전체 BA 원자성을 보장하지 않는다.
- Task 4 초기: 고정 upstream Shutdown worker wait patch 작성. finalizer에서 empty/interrupted/변경된 input Atlas 실패 검사 통과. 후속 실제 Atlas 저장/load 결과는 아래에 기록.
- Task 5: recorder/probe/MCAP 설정 구현. 명시 topic·실시각·0 frame·실패 판정 테스트 통과. MCAP native replay 시험 준비 중.
- Task 6: optional KSRC tee patch를 검토 snapshot의 새 사본에 fuzz=0 적용. fake SPI에서 baseline/patch 명령 완전 일치, stale 0.3 s·종료 0 명령 5회 확인. absent/full/slow collector 검사 통과. 실제 checkout/장치는 수정하지 않았다.
- Task 7: Foxglove official SDK 0.59.0 모델로 panel config serialization 확인. 레이아웃 import/live UI는 미실행.
- Task 8: 기존 4285 stereo pair(142.8 s) 센서 파일만 읽는 replay 준비. native Humble C++ gtest target 5개 통과. Pi Jazzy는 미실행.
- Ruling: 최초 실기를 수동 JSON 편집 없이 준비하도록 probe의 실제 rectified CameraInfo.P + 명시적 rectified 확인 + 원본 URDF fixed 경로 변환 helper를 추가했다. 실측으로 표시하지 않으며 실제 profile/rate/exact pair 불일치를 거부한다.
- Ruling: official PinHole stereo 내부 rectification의 좌표 변환을 body extrinsic·overlay에도 적용했다. ORB 알고리즘은 변경하지 않는다.
- Ruling: Path/overlay 화면 출력은 5 Hz, 전체 원본 pose history는 유지한다. map snapshot은 1 Hz.
- 검증 편차: map output/finalizer/replay 일부 테스트는 구현 직후 추가하여 엄격한 RED-first 순서가 아니다. 실제 C++ 컴파일과 실패 입력 검사를 수행했으며 검증 결과만 주장한다.
- 빌드: 별도 /tmp의 upstream ORB + Pangolin 0.8 native 의존성으로 x86/Humble 호환성을 검증한다. Pangolin 헤더는 C++14가 필요하고 upstream bool++는 C++17에서 금지되어 외부 CMake 표준만 C++14로 맞췄다. 첫 시도 C++17 빌드는 실패 로그에 보존. Pi/Jazzy/ARM 검증 대체가 아니다.
- Native replay RED: stereo01–03은 첫 map 생성 후 g2o::HyperGraph::clear에서 abort했다. 실패 bag/log/manifest를 보존했고 complete로 표시하지 않았다. GDB backtrace와 실제 flags로 조사: ORB는 C++14인데 bundled g2o는 표준을 명시하지 않아 GCC 기본 C++17로 빌드되어 Eigen class new/delete ABI가 달랐다. adapter의 arch/legacy allocation만 맞춘 두 시도는 충분하지 않았다. g2o도 C++14로 명시하여 재빌드한다. 추적 알고리즘 변경은 없다.
- 로컬 MCAP plugin은 apt package를 /tmp에만 추출했다. 시스템 ROS/Pi 환경은 설치/변경하지 않았다.
- Replay 초기 recorder discovery 전에 영상이 시작되어 원본 일부가 빠지는 것을 발견했다. replay의 최소 subscriber 수를 지정하고 discovery timeout은 실패로 보고하도록 수정했다. 기존 source stamp는 유지한다.
- Native replay GREEN: bundled g2o도 실제 C++14로 재빌드한 뒤 첫 map 생성 abort는 재현되지 않았다. ORB marker symbol/adapter RPATH/arch/Eigen allocation 계약을 확인했다. 시스템 설치 대신 /tmp 의존성을 사용했다.
- Task 3 후속: ORB는 merge 중 GetId를 바꾸므로 표시 alignment의 identity를 GetInitKFid(원점 keyframe)로 변경했다. runtime GetId는 별도 진단에 남긴다. origin_keyframe_id_v1이 없는 옛 manifest load는 거부한다. load05에서 원점 60→0 전환과 이전 map 0 alignment 동일성을 확인했다.
- Task 4/8: stereo05 360 pairs/360 poses, load05 480 pairs/480 poses, MCAP 원본 좌우 각 360/480, 정상 종료/Atlas/최종 export 확인. 같은 20° 시뮬레이션 기록을 90 frame 뒤부터 load 재생했으며 separate-drive reuse는 false다.
- 미해결 실패: ABI 수정 뒤 load04 merge 중 SIGSEGV 한 번. GDB 3회·일반 2회·최종 load05는 정상 load/merge/save였으나 원인 미확정이다. 실패 자료를 보존한다. stable identity 수정이 crash를 해결했다고 주장하지 않는다.
- Native 실패 입력: invalid_stereo 중복 stamp/크기 2개 거부, blank10 RECENTLY_LOST 중 pose stamp 미갱신. inertial_no_imu 60 interval drop/0 poses. 두 run 모두 incomplete/exit1.
- 최종 fresh 검증: Python47 통과, gtest12/5targets 통과(colcon summary17은 target5도 포함), native adapter build exit0. 로그 python_verified.log/cpp_tests_verified.log/adapter_build_verified.log. 로컬 helper/ABI 검증은 Pi 실기 완료가 아니다.
- Task 5 쓰기 실패: RLIMIT_FSIZE=1 MB에서 rosbag child는 exit0/metadata60인데 실제 MCAP 잘림을 발견했다(storage_failure01). RED 테스트2개→container 검사 추가→storage_failure02 failed/exit1 확인. 정상30쌍 recording_verified01은 complete/exit0, 지도 미실행은 전체 incomplete로 유지. 실제 disk-full 마운트 시험이나 전체 payload CRC 검증으로 주장하지 않는다.
- 원본 URDF SHA256 유지, 설치된 scripts --help·Python compileall 확인. plot은 시스템 matplotlib에 PYTHONNOUSERSITE=1로 실행해 scientific preview를 보존했다. 새 motor command나 실제 SPI 접근은 없다.

## 후속: 첫 실기 검사 도구 (사용자 “다음 것도 미리 만들어줘”)

- 기존 Task0/5/8의 준비·기록 흐름을 보완하는 bounded 변경으로 직접 구현했다. 추가 범위 선택 질문을 남기고 공통으로 필요한 실기 검사부터 진행했다. depth/Nav2/붓 계획은 이 추가 구현에 포함하지 않는다.
- `mapping_preflight.py`: 읽기 전용 host package/환경 확인과 실제 probe의30초·profile/보정·clock·age·동일 stamp 후보·disk 판정. 미확인/임시 extrinsic과 실제 motor 확인을 준비 완료로 바꾸지 않는다.
- `mapping_probe.py`: 최소 age·수집 중 profile 변경·use_sim_time provenance 추가. 기존 source stamp를 유지한다.
- `mapping_session.py`: operator host 시각의 구간 append, 실제 기록/Atlas/export/input hash를 재검사하는 별도 report. 원본 상태나 reuse 검증을 조작하지 않는다.
- RED 신규15개→GREEN, CameraInfo 크기/주점 실패2개와 Atlas input hash 실패1개를 추가로 확인하고 수정. 최종 mapping Python65개 통과, adapter native build/설치 CLI 정상·실패 exit code 확인.
- ROS 기능 fixture64×48/30Hz 정지30초: 정상901/900·exact900 ready; profile 변경2회 not_ready. 실기 센서가 아닌 명시적 합성 입력이다.
- 실패 보존: 초기640×480 합성 publisher bytes setter20.8ms/image로16.5Hz, array로 변경한 다음에도 동시 best-effort 수신에 좌우 누락. strict pair 판정을 완화하지 않고 별도 작은 기능 fixture로 분리했다. 실제 Pi 전송 문제 해결이나 실기30Hz 검증으로 주장하지 않는다.
- 전체 pytest 시도는 기존 mesh script `test_mesh_inertia.py`/`test_mesh_preparation.py`의 trimesh 미설치로 collection error2개. 관련 없는 설치/코드 변경은 하지 않았다.
- 명령/판정/한계: [첫 실기 도구 문서](../../experiments/first-real-mapping-tools.md). 자료: `outputs/headless_mapping/20261009_field_tools`. 원본 URDF hash 유지, 실제 Pi·motor/SPI·35°·UI/Atlas 별도 주행은 미실행.

## 후속: 전송 누락·Atlas 병합 종료 수정 (사용자 “해결해봐”)

- 이전 실패/작은 입력 통과를 최종 해결로 취급하지 않고 원본 크기에서 재현했다. 환경은 local22.04/Humble/x86, conda unset, /usr/bin/python3이며 Pi/실기/모터/SPI는 실행하지 않았다.
- Fast DDS2.6.12 기본 SHM512 KiB에서640×480 stereo 좌우 누락을 확인했다. 같은 XML의 segment만32 MiB로 늘려 두 consumer 모두300/300,60초 시험1800/1800을 확인했다. QoS/시각/판정 기준/해상도를 바꾸지 않았다. 제품 XML을 설치하고 publisher 포함 모든 프로세스의 시작 전 적용하도록 실행 문서를 업데이트했다.
- 실제 크기30초 probe 정상900/899 exact899 준비 통과, profile 변경2회 계속 거부. 종료 callback 경계의1개 차이를 무손실로 주장하지 않는다. 실기Pi 네트워크/USB 검증이 아니다.
- Python Image bytes setter 약19.67ms/image 병목을 array로 수정했다. 같은 원본300쌍의 MCAP 전체 payload/촬영 시각을 비교해 모두 동일, receipt 구간12.8505→9.9684초를 확인했다. raw source/GT 차단은 유지했다.
- 현재 valid Atlas GDB6회에서 load/저장6회, 실제 merge 완료5회였으나 원인 미확정으로 남겨둔 채 실패 당시 Atlas를 공식 API에서 직접 재생했다. Pangolin loader 실패4개는 미실행 증거로 보존하고 환경 수정 후 세 번째240쌍0.5배속에서 SIGSEGV 재현/core 확보.
- core: frame410, loaded point1694의 view flag85/scale1769473, scale 배열8개. ORBmatcher projection에서 범위 밖 접근, LoopClosing은 local BA 중. default MapPoint 생성자/serialize의 transient tracking cache 초기화 누락을 확인했다. 원본 DSO/core/stack/객체 값/생성자 RED 로그를 보존했다.
- default constructor의 view flags2개/scale levels2개만 초기화하는 `orbslam3_atlas_load_cache.patch` 추가. ORB 알고리즘/threshold/gate/Atlas 저장 형식은 유지했다. source + 실제 link marker를 요구하고 오래된 DSO 혼합을 차단한다.
- 같은 실제 생성자 poison-memory 검사 RED(exit1)→GREEN(exit0). 추가한 gtest도 원본 DSO preload에서는 실패, 수정 DSO에서 통과. 같은 실패 Atlas 공식 API 반복6/6이 정상 병합·추적·저장했다. 실제 별도 주행 reuse로 주장하지 않는다.
- Python65 통과, native adapter rebuild exit0, gtest13/6targets 통과(colcon19는 target6 포함). 전체 pytest는 기존 mesh 검사 `test_mesh_inertia.py`/`test_mesh_preparation.py`의 trimesh 미설치로 collection error2개. 관계없는 설치/코드 변경은 하지 않았다.
- 변경/자료 설명: [전송·Atlas 조사 기록](../../experiments/stereo-transport-and-atlas-debug.md). 원본 URDF SHA256 유지. Git init/worktree/commit, 새 subagent, 실제 장치 접근은 없다.
- 수정 후 native ROS2회 각360쌍/360pose/MCAP 좌우각360, conversion/tracking failure0, merge/load/save/PLY/report 통과. 실제 MCAP reader로 영상/pose 시각 전부 원본 일치·finite pose·map60→0·이전 alignment 정확히 유지 확인. 두 run은 `evidence_complete=true`, 실제 별도 주행 reuse는 false다. 자료 `20261009_merge_fixed/`.
