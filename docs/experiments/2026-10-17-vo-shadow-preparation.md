# 표면 실험과 독립적인 VO 준비

## 이번 확인 범위

서버 실행·업로드 없이 로컬 소스만 확인했다. 실기 모터·CAN 코드는 변경하지 않았다. ORB를 실제 실행하거나 VO 정확도를 측정한 결과가 아니다.

## 준비된 부분

- `ros2_ws/src/orbslam3_ros2/src/stereo_inertial_node.cpp`: `sensor_mode`로 stereo / stereo_inertial 선택, ExactTime stereo 처리. 기본 입력은 `/rover/stereo/left/image_raw`, `/rover/stereo/right/image_raw`, `/rover/imu/data`.
- 출력은 `/orbslam3/camera_pose`, `/orbslam3/base_pose`(base 출력 설정 시), `/orbslam3/tracking_state`, `/orbslam3/diagnostics` 등이다. tracking OK와 유한 pose 조건을 거쳐 pose를 발행한다.
- `scripts/replay_stereo_sequence.py`: EuRoC 형태의 `mav0/cam0`, `mav0/cam1` 시퀀스를 ROS 입력으로 재생한다. 좌우 timestamp 일치와 단조 증가를 검사한다.
- `experiments/handheld_mapping/mission_runtime.py`: tracking loss, stale pose, frame/site identity, pose epoch 변경을 검사한다.

## 연결이 필요한 부분과 실행 순서

1. 시뮬의 좌우 stereo 영상을 같은 simulation timestamp로 PNG/원본 이미지에 기록한다. 현재 site_sim 실행기의 관찰 JPEG 두 장(overview/front)은 stereo 데이터셋이 아니다. JPEG publish wall 주기를 sensor sampling rate로 사용하지 않는다.
2. 실제 생성된 camera intrinsics와 stereo extrinsics, 최신 URDF의 camera-to-base 변환을 함께 저장한다. URDF 이름만 바꾸거나 D435 실측 보정값을 시뮬 카메라에 복사하지 않는다.
3. 기존 replay 입력 형식으로 기록하고 `sensor_mode=stereo`로 병행 평가한다. 제어는 GT를 유지하고 VO는 평가에만 쓴다. 처음에는 저장 영상 재생으로 시작하면 Isaac 실행을 반복하지 않고 설정을 검토할 수 있다.
4. 동일 시각 GT와 추정 base pose를 맞춰 평가한다. 초기 정합은 고정된 하나의 SE(3)만 사용한다. timestamp pairing tolerance와 실제 시간 차를 기록한다. 위치·yaw 오차, tracking 유효 비율, 최대 연속 추적 손실 시간, 회복 시 map 변경을 남긴다. 전체 궤적을 매번 다시 정렬해서 drift를 숨기지 않는다.
5. ORB 노드가 새 map마다 reference alignment를 생성하므로 map ID/diagnostics를 확인한다. 이를 임무 입력의 pose_epoch와 연결해야 하며 갑자기 새 원점으로 계속 주행해서는 안 된다. stale/lost/epoch 변경 시 정지하는 경로를 회귀 검사한다.
6. VO 병행 평가 후에 제어 pose 선택 기능을 구현한다. estimator 모드에서 GT fallback을 자동 사용하지 않는다. GT는 평가 로그에만 둔다.
7. VIO는 별도 후속 단계다. stereo와 IMU의 동일 simulation clock, 샘플링·단위·중력·축·noise 설정·외부 변환을 확인한 뒤 stereo_inertial을 평가한다.

## 복귀 로직 준비 결과

`SiteMission.descent_path()`가 10° 하산 경로에 정확히 일치하는 출발 waypoint를 찾으면 거기서 종료한다. 해당 waypoint가 없는 일반 임무는 기존 갈림길 경유 복귀를 유지한다. 가까워 보인다는 이유로 임의의 지형을 가로지르지 않는다.

현재 `outputs/site_sim/20261017c/mission` 기준:

- 기존 10° 하산 길이: 9.8021 m, 끝점 (3.7, 0.35).
- 변경 후 하산 길이: 8.1521 m, 끝점 (2.9, 1.2) = 출발점.
- 일반 경사 20° 기준 strict path validation 통과. 허용오차 slack을 추가하지 않았다.
- 이후 갈림길→출발점 연결 경로 왕복을 생략한다.
- 기존 대표점 `entry10`은 route[1]에 정의돼 있어 새 하산 종료점 뒤에 있을 수 있다. 이 경우 방문했다고 거짓 기록하지 않는다. UI 대표점 정의는 실제 10° 하산 입출구 기준으로 후속 정리해야 한다.

이 변경은 로컬 코드에만 있으며 서버에 반영하지 않았다. 표면·UI·site_sim의 동시 작업 변경도 이번 작업에서 수정하지 않았다.
