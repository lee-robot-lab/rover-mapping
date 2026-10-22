# Rover Mapping

**가기 전:** [사전 테스트·부하·로그 기록](TESTS_BEFORE_FIELD.md) · [현장 CLI](CLI.md) · [현장 시험 절차서](FIELD_TESTS.md)

**2026-10-22 KSRC 최종 f판 실행:** [현장 CLI 명령어 — S1 plan 분리](CLI.md), [AP 내부망·초기 배치 안내](deployment/ksrc/README.md), [현장 시험 절차서 — TELEOP→MPPI·E-STOP·ORB 자동 복구](FIELD_TESTS.md). Pi 배포 코드는 `deployment/ksrc/`에 있으며 기본 실행은 모터 없는 FAKE다.

D435와 Raspberry Pi로 현장 영상을 수집하고, 노트북에서 ORB-SLAM3·TSDF 기반 3D 지도를 구성해 로버의 대표점 방문 임무를 준비하는 프로젝트다. 촬영 중 coverage/오버레이, 촬영 후 지형 층·prototype costmap 분석, 기존 STM 조이스틱 연동과 STL 기반 경로·임무 도구를 포함한다.

**현재는 지도 품질 검증과 STL 이후 임무 준비 단계다.** IMU 융합 시험, depth 구간 전역 정합·재융합, RGB 컬러 기록, 중력 후보·crop·출발점·반복 지도 병합 편집 경로가 있다. STL 임무 편집기와 pose 기반 상태·속도 요청 실행기, 합성 시나리오 재생도 구현되어 있다. RGB 동시 수집의 실기 성능, 실측 정확도, 고정 지도에 대한 주행 보정은 검증이 남아 있다. 최신 지도 운영 절차는 [기준 지도 워크플로](docs/experiments/reference-map-workflow.md), 임무 운영 절차는 [STL 임무 도구](docs/experiments/site-mission.md)를 따른다.

서버 Isaac 시뮬레이션에는 **로버가 정답 위치 대신 ORB-SLAM3 stereo + IMU(+엔코더) ESKF 추정 위치로 주행하는 닫힌 루프**가 있다. 시뮬레이션 결과이며 실기 예측이 아니다. 구성·측정·한계는 [시뮬 VIO와 닫힌 루프](docs/experiments/2026-10-18-sim-vio-closed-loop.md)에 있다.

## 프로젝트 목표와 대회 시나리오

목표는 출발지·갈림길·경사로 진입점·정상 같은 대표점을 방문하고, 등판 진행이 부족하면 후퇴해 다른 경로로 임무를 이어가는 로버를 준비하는 것이다.

- **B: 여의도 중간 테스트.** 사전 지도가 없는 현장에서 매핑한다. 현재 역할 분담은 팀원이 매핑과 STL 생성을 맡고, 이 프로젝트의 임무 도구는 전달받은 STL부터 사용한다.
- **사천: 실제 대회.** 현장 매핑 품질이 충분하면 사천의 기존 지도와 새 사천 관측 지도를 정합·병합하는 방안을 검토한다. 여의도 지도와 사천 지도를 합치는 계획은 아니다.
- **세 경로와 공통 정상.** 갈림길에서 10°·15°·30°로 부르는 세 길이 같은 정상으로 연결된다. 상행과 하행을 별도 방향으로 다루며, 경로 이름과 STL에서 계산한 실제 경사값은 구분한다.

기본 임무는 **출발 → 갈림길 → 30° 진입 대표점 → 등판 시도 → 정상 → 10° 하행 → 출발지 복귀**다. 30°에서 진행이 부족하면 실제 지나온 경로를 되짚어 갈림길로 후퇴한 뒤 15°로 우회하고, 15°도 실패하면 10°를 사용한다. 30°가 성공하면 그 성공을 기록한다. 진입점 방문·등판 결과·정상 방문·복귀 완료는 각각 기록한다.

## 구현 범위와 남은 연결

| 구분 | 현재 상태 |
|---|---|
| 지도 입력·임무 설정 | STL 단위·up 설정, 지형 격자 생성, 출발·갈림길·정상·세 경로 편집과 저장/재로드 |
| 경로 검사 | 관측 여부·벽·단차·로버 여유 반경·경사 검사. 일반 경로와 시험 경로의 경사 제한 분리 |
| 임무 판단 | 대표점 방문, 등판 진행 부족 판정, 실제 주행 경로 후퇴, 대체 상행, 정상·복귀 결과 관리 |
| 실행 준비 | 정렬된 pose JSONL 입력, 추적 손실·stale·좌표 점프 시 정지 요청, 합성 재생과 결과 로그 |
| 위치 추정(시뮬) | ORB stereo + IMU + 엔코더 ESKF(IMU 공백 방어, map 재닻, 렌더 지연 보상)와 서버 닫힌 루프 주행. **벽 보정은 포함하지 않는다** |
| 남은 실기 통합 | live ORB→STL 지도 좌표 정렬·입력 adapter, 실제 구동기 연결과 독립 정지/watchdog, 기체별 설정·현장 검증 |

현재 임무 실행기는 속도 요청을 **파일로 기록하며 실제 모터에는 송신하지 않는다.** 진행 부족 판정은 pose 변화에 기반하고 바퀴별 slip 실측은 아니다. 합성 재생은 평면 운동학과 주입한 정체를 사용하므로 실제 경사·모래·토크 성능을 검증하지 않는다. 실기 제어 연결은 시뮬레이션/fake 통신 검증 후 저속 실기로 확인한다.

## 먼저 읽을 문서

| 목적 | 문서 |
|---|---|
| 최종 f판 Pi 실행·AP SSH·초기 위치·CLI | **[현장 CLI 안내](deployment/ksrc/README.md)** |
| TELEOP→MPPI·E-STOP·연결 단절·ORB 자동 복구 시험 | **[현장 시험 절차서](FIELD_TESTS.md)** |
| 처음 설치 | [설치 설명서](docs/installation.md) |
| 촬영·조종·지도·costmap 운영 | **[사용 설명서](docs/user-guide.md)** |
| RGB·전역 재구성·중력·crop·출발점·병합 | **[기준 지도 워크플로](docs/experiments/reference-map-workflow.md)** |
| 팀원 STL 이후 대표점·세 경로·우회·복귀 준비 | **[STL 임무 도구](docs/experiments/site-mission.md)** |
| 서버 Isaac 경로 추종·costmap 기록·실시간 관찰 | **[서버 대회 임무 시뮬레이션](docs/experiments/server-site-mission.md)** |
| 시뮬 VO+IMU 추정과 닫힌 루프 주행 | **[시뮬 VIO와 닫힌 루프](docs/experiments/2026-10-18-sim-vio-closed-loop.md)** |
| 구현·검증 상태와 다음 작업 | [통합 인수인계서](docs/handoff/d435-rover-mapping-handoff.md) |
| 새 담당자/팀원 공유 | [공유·기여 안내](docs/sharing.md) |
| 제외된 원본/실행 자산 | [데이터·runtime 안내](docs/data-assets.md) |
| 입력 공백 조사 | [조사 보고](docs/experiments/2026-10-14-input-gap-investigation.md) |
| 최신 20° 카메라 장착 URDF | [기구 기준·보정 적용 범위](docs/experiments/2026-10-14-rover-extrinsics-update.md) |
| 보정 IMU와 D435 동시 기록 | [실행·검증 결과](docs/experiments/2026-10-14-imu-co-recording.md) |
| upstream 버전·patch | [외부 의존성](third_party/README.md) |

## 빠른 시작

GitHub 접근 권한이 있는 계정으로 clone한다.

```bash
git clone git@github.com:lee-robot-lab/rover-mapping.git
cd rover-mapping
```

[설치 설명서](docs/installation.md)의 Python 환경을 준비한 뒤:

```bash
# 장비 없이 GUI → 원본 저장 → 합성 pose TSDF
bash tools/handheld_mapping.sh --demo

# 장비 없이 지형·costmap 파일 생성 (새 output 필요)
bash tools/terrain_mapping.sh --demo --output outputs/terrain_demo_01

# 장비 없이 STL 이후 대표점·세 경로 임무 편집기 확인
bash tools/plan_site_mission.sh --demo

# 저장된 최신 촬영의 3D 지도
bash tools/view_mapping.sh
```

실제 촬영은 native point runner와 Pi runtime을 준비한 뒤 `bash tools/handheld_mapping.sh`로 시작한다. runner가 없으면 live 안내·노트북 ORB 옵션은 활성화되지 않는다. 합성 GUI/terrain demo는 native ORB 없이도 사용할 수 있다.

기본 실제 구성:

```text
Pi: D435 IR좌/우 + depth 640×480, 요청15FPS, projector OFF
  → bounded queue → zlib → SSH
노트북: 원본 저장 ┬→ live ORB → 잠정 coverage/IR 오버레이
                 └→ 종료 후 full IR ORB → 최종 keyframe + depth → TSDF
선택 map + 검증한 중력 정렬 → 지형 층 → 로버별 costmap
```

## 저장소 범위

| 폴더 | 내용 |
|---|---|
| `experiments/handheld_mapping` | Qt·수집/전송·live/offline ORB·TSDF·지형·costmap·STL 대표점 임무 |
| `tools` | GUI/뷰어/terrain·임무 wrapper, native runner build |
| `experiments/site_sim` | 서버 물리 임무·costmap/경험 기록·브라우저 관찰·실행 관리, stereo 기록·ORB 스트리밍·VIO 오프라인 평가 |
| `experiments/localization` | ESKF·오프라인 융합·온라인 추정기(영상+IMU+엔코더, 벽 보정 제외) |
| `ros2_ws/src/orbslam3_ros2` | ROS adapter·headless 준비 도구·native patch |
| `ros2_ws/src/rover_description/urdf` | 좌표/보정 preflight에 사용하는 URDF 참고본 |
| `teleop/ksrc_snapshot` | 확인한 KSRC의 조이스틱·SPI bridge 소스 일부 |
| `tests` | 매핑/headless/fake SPI 회귀 |
| `docs` | 설명서·인수인계·작은 검증 보고서 |
| `third_party` | 외부 프로젝트 버전과 라이선스 안내 |

촬영 PNG/PLY/Atlas, native 바이너리·vocabulary, venv, 빌드 결과, 원본 전체 firmware/대형 로버 mesh는 Git에 포함하지 않는다. 결과는 로컬 `outputs/`, 외부 소스는 로컬 `external/`에 준비한다. [필요한 별도 자산](docs/data-assets.md)을 확인한다.

## 회귀 테스트

```bash
KSRC_REVIEW_ROOT="$PWD/teleop/ksrc_snapshot" \
QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest -q
```

2026-10-18 기준 이 저장소 전체는 **688 passed, 16 skipped**였다(2026-10-14 원본 workspace의 해당 범위는 173 passed). 이 저장소는 optional native binary를 제외하므로 그 실행기 검사 1개는 빌드 전 skip될 수 있다. 실제 하드웨어를 열지 않는 fake SPI 테스트를 포함한다. 공유본의 실제 결과는 [공유본 검증](docs/share-validation.md)에 기록한다.

Python-only demo·회귀 성공은 실측 지도 정확도나 현재 Pi 전체 처리율 검증을 뜻하지 않는다. voxel 1cm/추적 성공률/초록 coverage를 정확도·통행 가능성의 보증으로 사용하지 않는다.

## 환경·버전

노트북 Ubuntu22.04/Python3.10, Pi Ubuntu24.04/native ROS2 Jazzy/Python3.12가 기준이다. 노트북 매핑은 `.venv-mapping`을 쓰며 Conda/ML 환경과 섞지 않는다. Pi에 Humble를 source하지 않는다. 새 머신의 native/RealSense 설치는 별도 확인해야 한다.

이 저장소는 사용자 선택으로 public 공개되어 있다. ORB-SLAM3/ORB-LINE과 ROS adapter 등의 기존 라이선스는 [외부 의존성 안내](third_party/README.md)를 따른다. 별도의 프로젝트 전체 라이선스를 임의로 지정하지 않았다.
