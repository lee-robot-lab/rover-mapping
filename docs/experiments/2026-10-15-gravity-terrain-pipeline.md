# 중력 기준 지도와 지형 처리 — 2026-10-15

목표는 경사면을 평평하게 펴는 것이 아니라, 고정된 중력 좌표 안에서 로버와 카메라의 3D 자세를 반영해 높이·경사·단차를 보존하는 것이다. 경사 진입/이탈로 센서나 위치 추정 모드를 전환하지 않는다.

## 현재 사용할 수 있는 경로

GUI는 stereo 비전 추정을 계속 실행하고 보정 IMU도 계속 기록한다. **중력 기준 지도와 높이·경사·단차 생성**을 켜면 다음을 수행한다.

1. 같은 Pi의 수신 시각으로 카메라와 겹치는 약2초 정지 IMU 구간을 찾는다. 최소40개, 구간 끝 허용120ms, 연속 표본 간격120ms 이하, 같은 reset segment를 요구한다. 평균 가속도 크기0.95–1.05g, 평균 대비 최대 편차0.025g, 최대 자이로 크기1.5°/s를 정지 선별 기준으로 쓴다. 이는 측정 오차 보증이나 움직임 검출의 완전성을 뜻하지 않는다.
2. 보정된 `acc_g`는 이미 로버축 **X 전방, Y 왼쪽, Z 위쪽**이다. 칩→body 부호를 다시 적용하지 않는다. 정지 가속도계의 specific force를 위쪽 벡터로 사용한다. 중력 가속도 자체의 부호와 혼동하지 않는다.
3. 같은 RUN에 보존된20° URDF의 고정 body↔left optical 경로와 실제 CameraInfo의 rectification을 합성한다. 정지 구간의 카메라 자세로 위쪽을 ORB world에 옮긴다. x축은 기준 로버 전방의 수평 성분, z축은 추정 위쪽이다. 원점은 기준 카메라 위치이므로 높이는 지면에서 잰 절대 높이가 아니다.
4. map마다 이 변환을 고정한다. 실시간 coverage와 영상 오버레이는 고정 z-up X-Y 투영을 사용한다. map이 바뀌면 새 기준을 기다리며 다른 map의 기준을 재사용하지 않는다.
5. 종료 후 최종 keyframe으로 기준을 다시 계산한다. live 최적화 상태와 최종 map을 섞지 않는다. TSDF는 변환된 pose로 생성하고 3D 경로에도 동일한 변환을 적용한다.
6. 원본 depth와 최종 pose를 한 장씩 재생해 map별 높이·경사·거칠기·단차·cost를 새 폴더에 저장한다. **높이·경사 보기**로 정적 분석 그림을 연다. 재계산은 기존 지형 폴더를 덮어쓰지 않고 `_v2` 등 새 폴더를 만든다.

정지 구간이 없거나 축·보정 출처가 다르면 해당 map은 중력/지형 `unavailable`로 표시한다. 원본과 기본 3D 지도는 보존하고 세션은 `partial`로 마감한다. 지형 생성 실패를 중력 기준 지도 성공으로 숨기지 않는다. IMU 수신 시각은 정지 구간을 대응시키는 데만 쓰며 취득 시각 동기화로 취급하지 않는다.

실시간 차체 자세는 **비전 pose + 정지 IMU의 중력 기준 + 장착 모델**로 계산한다. 차체 기울기5° 이상 진입,3° 이하 이탈을1초간 유지하면 `orb_live/body_incline.jsonl`에 이벤트를 기록한다. 추적 손실이나 map 변경은 연속성으로 취급하지 않는다. 이 이벤트는 표시용이며 지면 경사·로버 통과 한계·SLAM 모드 전환 기준이 아니다. 서스펜션이 있는 로버에서는 몸체 기울기와 지면 경사가 다를 수 있다.

후속 표시 보완: 가속도 단독 roll/pitch에는0.2초 평활화와 사용자가 평지/정지를 확인한 경우의 **평지 영점**을 추가했다. 이는 IMU 참고 표시만 바꾸며 위의 중력 기준·비전 차체 자세·실제 지형 경사를 바꾸지 않는다. 알려지지 않은 바닥의 초기 기울기를 센서 바이어스로 자동 간주하지 않는다. 수신이 회복돼도 남던 지연 경고를 해제하고, 실제 control packet 공백과 노트북 전달 지연을 분리해 기록한다.

## 실행

기존 bridge/IMU 프로그램을 정상 종료한 뒤 SPI owner 하나만 실행한다. 로버가 꺼져 있는 동안은 실행하지 않는다.

```bash
bash tools/mapping_imu_bridge.sh pi@raspberrypi.local
```

다른 노트북 터미널:

```bash
bash tools/handheld_mapping.sh
```

보정 파일은 `outputs/handheld_mapping/calibration/imu_calibration.json`을 기본으로 사용한다. 이번 작업에서 사용자가 지정한 `/home/parkshinyoung/ksrc/pi/imu_calibration.json`을 검증 후 바이트 단위로 동일하게 복사했다. Git에는 장비별 보정값을 넣지 않는다. 앱의 두 IMU/중력 옵션을 확인하고 **매핑 시작 후 최소3초 정지**해 오른쪽 중력 기준 표시를 기다린다. 추적을 잃고 새 map이 만들어진 경우에도 정지가 필요할 수 있다. 조이스틱은 별도 프로세스이고, 매핑 종료는 모터 정지가 아니다.

기존 RUN에 IMU 기록·보정·장착 사본·Pi 수신 시각·정지 구간이 모두 있을 때:

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.gravity /absolute/path/to/RUN \
  --backend orb_offline --reconstruct --terrain
bash tools/view_mapping.sh /absolute/path/to/RUN
```

원본은 보존하지만 `--reconstruct`는 해당 dense 결과를 갱신하므로 중요한 기존 결과는 RUN 사본으로 실행한다. 중력 파일에는 RUN/backend/map과 keyframe·보정·URDF·IMU 원본의 SHA-256을 묶어 두며, 입력이 바뀐 오래된 기준은 재사용하지 않는다.

## 동적 IMU 융합: 입력 경로 구현, 실기 적용 전

native runner에 **offline `IMU_STEREO`** 입력을 추가했고 재빌드했다. 평지와 경사지형 모두 같은 추정 모드로 실행한다. GUI live는 아직 stereo이며 동적 IMU 융합을 하지 않는다. 검증된 입력이 준비되기 전에 GUI 모드를 임의 전환하지 않는다.

현재 SPI telemetry는 약50Hz control packet이며 `imu_n`이 원시208Hz 표본 여러 개의 평균을 나타낼 수 있다. 이 자료의 MCU/Pi 수신 시각을 개별 센서 취득 시각으로 치환해서 넣지 않는다. 6면 보정과 정지 표준편차만으로 noise density/random walk를 만들어내지도 않는다.

실제 융합에는 같은 RUN의 개별 표본 CSV와 실측 profile이 필요하다. CSV 열은 `stamp_ns,ax,ay,az,gx,gy,gz`, 단위는 m/s²·rad/s, 축은 로버 body 방향으로 보정된 IMU 원점, 시각은 검증된 카메라 clock이다. profile은 다음 필드를 요구한다.

- `schema: 1`, 현재 `run_id`, RUN 내부 `samples` 상대경로.
- `sample_format: per_sample_SI_in_camera_clock`, `frame: rover_body_aligned_imu`, `acc_unit: m/s^2`, `gyro_unit: rad/s`.
- `clock_verified`, `extrinsic_verified`, `sample_timestamps_verified`, `noise_verified`: 실제 측정 검증 후에만 true.
- `T_imu_left`: rectified 좌측 카메라 좌표를 보정 IMU 좌표로 옮기는 실측4×4 변환. URDF provisional 값을 검증 없이 대신 넣지 않는다.
- `frequency_hz`와 `noise`의 `NoiseGyro`, `NoiseAcc`, `GyroWalk`, `AccWalk`: 실측 값.
- `source_sha256`: 표본 CSV, `capture_calibration.json`, `imu_calibration.json`, `robot.urdf`의 RUN 상대경로→checksum.

입력 전체의 시각 단조성·표본 공백·유한 수치·stereo 촬영 전체의 표본 범위와 checksum을 검사한다. native 최종 map의 IMU 초기화도 검사하며 초기화되지 않은 map을 융합 성공으로 내보내지 않는다.

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.offline /absolute/path/to/RUN \
  --inertial-profile /absolute/path/to/verified_run_profile.json
```

결과는 `orb_inertial`에 저장해 기존 `orb_offline` 비교본을 보존한다. `dense_inertial` 생성 시에도 검증된 중력 기준이 필요하다. 현재 기록과 보정 파일만으로 이 모드의 실기 정확도나 동기화를 검증했다고 주장하지 않는다. 실제 개별 표본·시각/장착/잡음 검증과 live 관성 입력 연결은 다음 하드웨어 단계다.

## 해석과 검증 범위

합성15° 경사면을 서로 다른 카메라 pitch로 관측했을 때 경사가 유지되는지, map별 고정 기준, TSDF와 경로의 동일 변환, 정지 실패/시각 공백/reset/출처 불일치 거절, GUI 옵션·부분 성공 마감을 검사한다. native runner 빌드와 IMU 입력 경계도 검사한다. 새 실제 경사면 촬영이나 ground truth 정확도 검증은 아직 하지 않았다.

[검증 수치](../evidence/20261015_gravity_terrain_validation.json): 전체265 passed/9 skipped, 화면 배치 조정 후 관련28 passed. 기존 실촬영120쌍의 새 runner 재생은9 keyframe을 저장하고 정상 종료했다. 640×480 합성 depth와 차체 pitch0/−8/−15°에서 알려진15° 경사면의 TSDF 평면 적합은15.0005°, 지형 층 중앙값은14.8901°(유효1011셀)였다. 80×60 입력에 stride4를 쓴 별도 희소 표본 시험은 유효12셀·16.4532°로1° QA 목표를 충족하지 못했다. 이 수치는 실제 센서/SLAM 정확도나 임의 해상도의 성능 보증이 아니다.

후속 보완으로 실제 재생은 공간·다중 시점 depth 필터와 반복된 아래쪽 높이군/국소 연결성 기반 지면 분리를 기본 적용한다. 지면 경사와 지면 위 장애물 높이를 분리하고, 가려진 바닥이나 지면 연결 근거가 부족한 표면은 unknown으로 남긴다. 전역 평면을 빼지 않는다. 합성 demo의 기존 셀 평균 기하 기준은 유지한다. 지면 종자와 연결 임계값은 heuristic이며, 반복되는 센서 오류와 pose 오차가 남을 수 있다. cost 한계는 실제 로버 한계가 아니며, 차체 경사 이벤트도 지면 분류를 대신하지 않는다. 중력 기준·장착 보정·실측 경사판을 대조한 뒤 주행 판단에 사용한다.
