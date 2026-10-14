# 최신 로버 URDF와 카메라 장착 좌표

2026-10-14 사용자가 `rocker_bogie_4wd_rover`를 전달했고, 실제 카메라 장착각은 **하향 20°**라고 확인했다. 기존 35° URDF는 과거 기록용으로 보존한다.

후속 갱신: Qt에서 [IMU 동시 기록](2026-10-14-imu-co-recording.md)을 켜면 최신 URDF 사본도 같은 RUN에 보존한다. 현재 지도 좌표는 계속 camera optical이며 자동 수평화·관성 융합은 별도다.

## 반영 파일과 원본

- 최신 참고 사본: [rocker_bogie_4wd_rover.urdf](../../ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf).
- 과거 35° 참고본: [rover.urdf](../../ros2_ws/src/rover_description/urdf/rover.urdf).
- 제공된 원본 경로: `/home/su/rocker_bogie_4wd_rover/urdf/rocker_bogie_4wd_rover.urdf`.
- 제공 원본 SHA-256: `107ed258fd7b1c66d6d170dce13bea0106b1f9f962c61965d2fa79214d44bb73`.
- 사본에서 첫 설명 주석과 카메라 주석의 35° → 20°만 수정했다. 모든 링크·조인트 수치는 제공 원본과 같다.

메시는 별도 `rocker_bogie_4wd_rover` ROS 패키지의 `package://` 경로를 참조한다. 이 저장소의 URDF 사본만으로 RViz 형상을 표시할 수는 없다. 보정 계산은 메시를 읽지 않는다. 원본 패키지와 그 설치본·체크섬은 이번 변경에서 수정하지 않았다.

## base_link 기준 센서 변환

단위는 m, X 전방·Y 왼쪽·Z 위이며, optical 프레임은 X 오른쪽·Y 아래·Z 전방이다.

| 프레임 | 위치 xyz (m) | 자세/의미 |
|---|---|---|
| `camera_link` | `0.12498574268 0.00747451631 0.01521787607` | `rpy = 0 0.349066 0`, 약 20° 하향 |
| `camera_left_optical_frame` | `0.12498574268 0.02497451631 0.01521787607` | camera_link의 +Y 17.5 mm, ORB 좌측 IR 기준 후보 |
| `camera_right_optical_frame` | `0.12498574268 -0.02502548369 0.01521787607` | camera_link의 -Y 32.5 mm |
| `imu_link` | `-0.01800665319 0 -0.021473` | `rpy = 0 0 0` |

`rpy` 수치로 계산한 하향각은 20.00000857°다. 좌측 optical의 전방축은 base 기준 약 `(0.93969257, 0, -0.342020284)`다. 로버 전체 높이 또는 지면 기준 카메라 높이는 위 Z 좌표와 다르다.

`prepare_mapping_run.py`는 고정 조인트 경로에서 기존 `camera_infra1_optical_frame` 또는 최신 `camera_left_optical_frame`을 읽는다. 두 후보가 모두 연결돼 있으면 임의 선택하지 않고 오류를 낸다. 선택한 프레임은 `urdf_left_optical_frame`에 남긴다. 실제 `CameraInfo.R` rectification을 적용한 역변환을 `T_c1_base`로 저장한다.

장착 변환은 계속 **`urdf_provisional`, `extrinsic_verified=false`**다. 새 URDF의 optical 원점은 모델에서 가정한 값이며 D435 내부 좌표와 실측 정합은 별도 확인이 필요하다. stereo baseline과 intrinsics는 URDF의 50 mm를 강제하지 않고 실제 rectified CameraInfo의 P로 구한다. depth↔IR 변환도 카메라의 실제 TF를 계속 사용한다.

## 새 로버 장착 run에서 사용

clone한 저장소 루트에서 다음 경로를 선택한다. Pi에서 실행한다면 사본을 먼저 Pi로 전달한다. 아래 probe는 실제 rectified IR 640×480, 15 FPS로 검사한 결과여야 한다.

```bash
export ROVER_URDF="$PWD/ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf"
python ros2_ws/src/orbslam3_ros2/scripts/mapping_preflight.py sensors \
  --probe /absolute/path/to/probe.json --urdf "$ROVER_URDF" \
  --profile 640x480x15:Y8 --rectified-input
```

이 명령은 준비 상태 검사다. `prepare_mapping_run.py --from-probe ... --urdf "$ROVER_URDF" --camera-profile ... --rectified-input`으로 새 run을 준비하면 보정값과 URDF 사본·SHA-256을 run에 보존한다. vocabulary/run-dir 등의 나머지 인자는 [headless 실행 절차](headless-pi-manual-mapping.md)를 따른다. 과거 run의 URDF·보정값을 바꾸거나 새 각도로 덮어쓰지 않는다.

**현재 Qt 매핑 앱은 camera optical 기준으로 ORB·TSDF를 계산하며 이 URDF를 자동으로 읽지 않는다.** 이번 변경은 headless 로버 pose 보정 입력과 최신 기구 기준을 준비한 것이다. Qt의 지도·coverage가 자동으로 수평화되거나 IMU 융합이 추가된 것은 아니다. 지형 분석의 `grid_from_world` 중력 정렬은 여전히 명시적으로 검증해 제공해야 한다. 카메라 장착각만으로 경사면 위 로버의 중력을 알 수는 없다.

## 전달 패키지에서 발견한 불일치

- 원본 실제 카메라 값은 20°지만 README·offset audit·validation report·카메라 주석은 35°다. 이번 사본과 이 문서는 사용자 확인값 20°를 따른다.
- 원본 `CHECKSUMS.sha256` 검사는 Camera.stl과 URDF에서 실패했다. 수정 후 해시가 갱신되지 않은 상태이며 파일 손상으로 단정하지 않는다.
- 원본과 `install/`의 URDF는 다르다. 설치본은 일부 조향·휠 축 부호와 IMU Z가 이전 값이다. 이번 참고 사본은 `install/`이 아닌 소스 `urdf/`를 사용했다.
- 원본 validation report의 14 links/13 joints와 달리 소스는 15 links/14 joints다. 기존 report의 pass를 이번 모델의 검증 결과로 사용하지 않는다.
- 오른쪽 조향은 이름과 위치가 다르다. `rr_steering_joint`는 +X의 `fr_leg`, `fr_steering_joint`는 -X의 `rr_leg`에 연결된다. 카메라 고정 변환에는 영향이 없지만 향후 joint 이름으로 제어할 때 대응을 확인해야 한다.

실제 STM 제어값·IMU decoder 축은 변경하지 않았다. 새 기구 모델과 제어 부호를 맞추는 작업은 시뮬레이션에서 먼저 확인하고, 이후 바퀴를 띄워 방향·정지를 검증해야 한다.

## 이번 검증

- `check_urdf` 및 ElementTree 파싱 성공. 소스와 사본의 XML 수치/구조가 같고, 제공 원본 SHA-256도 유지됨을 확인했다.
- 새로운 프레임 지원 전 회귀 검사 2개가 의도대로 실패했고, 지원 후 관련 39개 검사가 통과했다.
- 원본 작업 폴더의 매핑 검사 175개 통과. 공유 레포 전체 `pytest`: 175 passed, 1 skipped(native 실행기 선택 검사).
- 합성 rectified 15 FPS CameraInfo로 새 run 생성 성공. 장착각 약 20°, 실제 P에서 읽은 baseline 49.9182 mm, URDF 사본·hash·선택 프레임·ORB YAML 보존 확인. 합성 입력이며 Pi·카메라 실기 검증은 아니다.
- 변경 Python 파일 pyflakes, 공유본 diff 공백 검사, 현재 source manifest의 106개 hash 검사 통과.
