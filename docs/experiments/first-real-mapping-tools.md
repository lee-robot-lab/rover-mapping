# 첫 실기 검사·구간 기록·종료 리포트

> 2026-10-14: 이 문서의 35°/과거 IMU 자세는 당시 모델 기록이다. 새 run의 기구 기준은 사용자 확인 하향 20°인 [최신 URDF 갱신 기록](2026-10-14-rover-extrinsics-update.md)을 따른다. 과거 run의 보정값은 유지한다.

2026-10-09. 기존 [수동 ORB 실행 문서](headless-pi-manual-mapping.md)에 붙여 사용하는 준비 도구다. 이번 추가는 기존 검사·기록 흐름의 보완이며 모터·SPI owner·STM·ORB 알고리즘을 변경하지 않는다. 실제 Pi 접속, 35° 센서와 조종, Foxglove live UI는 여전히 미실행이다.

## 실제 Pi에서 사용할 순서

각 명령은 native Jazzy와 rover workspace를 source한 **Pi SSH 셸**에서 실행한다. 노트북 Humble setup을 Pi에 복사하지 않는다. 센서 드라이버와 KSRC 조종은 기존 실행 절차를 따른다. 아래 도구는 장치를 자동 설치하거나 드라이버·모터를 실행하지 않는다.

### 1. 읽기 전용 설치 확인

```bash
source /opt/ros/jazzy/setup.bash
source "$ROVER_WS/install/setup.bash"
ros2 run orbslam3_ros2 mapping_preflight.py --output "$HOME/pi_preflight_01.json" host
```

Ubuntu 24.04/aarch64/Jazzy와 RealSense·adapter·MCAP·Foxglove package prefix가 확인돼야 exit0이다. 이 판정은 환경·package 존재 확인이며 실제 binary ABI·장치·USB·firmware·SSH/UDP·단일 SPI owner·모터 정지 검증을 대신하지 않는다. 결과에 sensor/motor 확인은 false로 남는다. 설치·계정·SSH key·네트워크 설정은 변경하지 않는다.

### 2. 정지 30초 수집과 판정

드라이버의 실제 topic 이름을 조회한 뒤 수집한다. 기존 예시 기본 이름을 실제 이름으로 바꾼다.

```bash
PROBE="$HOME/mapping_probe/first35_02"
ros2 run orbslam3_ros2 mapping_probe.py --run-dir "$PROBE" \
  --left /camera/camera/infra1/image_rect_raw --right /camera/camera/infra2/image_rect_raw \
  --left-info /camera/camera/infra1/camera_info --right-info /camera/camera/infra2/camera_info --duration 30
ros2 run orbslam3_ros2 mapping_preflight.py --output "$PROBE/readiness.json" sensors \
  --probe "$PROBE/probe.json" --urdf "$ROVER_URDF" --profile 640x480x30:Y8 --rectified-input
```

`--rectified-input`은 실제 확인한 경우에만 지정한다. 판정은 다음을 검사하며 부족한 자료는 준비 완료로 채우지 않는다.

- 실제 mono8/profile·수신률·CameraInfo projection·baseline과 영상 크기의 일치.
- 수집 시간 30초 이상, real clock, source stamp 역행/중복 0, 수집 중 profile 변경 0.
- 마지막 최대1000개 window에서 좌우 동일 source stamp 후보가 99% 이상. 전체 장시간 frame loss 증명은 아니다.
- 영상 source age가 -0.05~0.5초 범위, 저장 공간 최소2 GB. `--max-age-s`, `--min-free-bytes`를 명시적으로 조정할 수 있으며 적용값을 결과에 남긴다.
- 온도80°C 이상은 경고다. 실제 부하·냉각·저전압 검사는 별도로 한다.

`capture_ready=true`는 이 입력 수집 검사를 통과했다는 뜻이다. URDF 외부 보정·IMU clock·모터 시험이 검증됐다는 뜻은 아니다. URDF 장착각은 임시값으로 표시하고 `extrinsic_verified=false`를 유지한다. 이전 probe에 새 age/clock/profile-change 자료가 없으면 재수집한다. 2 GB는 짧은 첫 검사 기준이며 긴 주행의 저장 용량 보장이 아니다.

실패 시 exit1/`issues`를 확인하고 원본 probe는 유지한다. 임의 stamp 변경·가짜 CameraInfo·장착각 재라벨로 통과시키지 않는다. 위 검사 통과 후 기존 `prepare_mapping_run.py --from-probe ...`로 새 run을 준비하고 recorder/ORB/화면을 실행한다.

### 3. 수동 주행 구간 표시

recording과 ORB가 실행 중인 같은 Pi에서, 실제 사용자가 행동을 시작/종료할 때 표시한다. 이 명령은 모터 명령을 보내지 않는다.

```bash
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase stationary --edge start
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase stationary --edge end
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase straight --edge start --note 'low-speed first straight'
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase straight --edge end
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase turn --edge start
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase turn --edge end
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase return --edge start
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase return --edge end
ros2 run orbslam3_ros2 mapping_session.py event --run-dir "$RUN" --phase stop --note 'operator stopped rover'
```

`events.jsonl`은 append 방식이며 run ID·operator host wall time·monotonic 시각·phase·edge·note를 남긴다. **실제 촬영 시각이나 정지/회전을 센서로 자동 검출한 결과가 아니다.** 잘못 표시했다면 원본을 지우지 말고 `--phase note --note 'correction ...'`를 추가한다. 바퀴를 띄운 저속·정지 검증 후 바닥 주행을 진행한다.

### 4. 정상 종료 후 결과 확인

사용자가 로버를 멈추고 ORB·recorder·collector를 정상 종료한 뒤:

```bash
ros2 run orbslam3_ros2 finalize_mapping_run.py --run-dir "$RUN"
ros2 run orbslam3_ros2 mapping_session.py report --run-dir "$RUN" --output "$RUN/field_report.json"
```

report는 원본 stage/manifest를 수정하지 않고 별도 파일에 쓴다. 파일 존재만으로 성공 처리하지 않고 실제 MCAP 구조·metadata counts·required topics·recorder 상태를 재검사한다. Atlas·최종 map export·tracking 상태·load 입력 checksum도 확인한다. 기존 output을 덮어쓰지 않으므로 다시 만들 때 새 파일명을 쓴다. report도 run checksum 목록에 넣으려면 마지막에 finalizer를 한 번 더 실행한다.

표시 내용은 input/pose 수·발행 비율·추적 state/drop counters·녹화 topic 수·지도 frame 목록·장착 보정 출처·구간 사건·telemetry 유무·load/reuse 구분이다. 발행 비율은 위치 정확도나 전체 경로 추적 성공률이 아니다. 여러 `orb_map_*`를 임의로 합치지 않고 GT가 없으므로 ATE/cm 정확도를 계산하지 않는다. MCAP container 검사는 payload CRC 전체 검증과 구분한다.

병합 종료는 같은 실패 Atlas의 직접 재생에서 MapPoint 초기화 누락으로 재현해 수정했다([조사 기록](stereo-transport-and-atlas-debug.md)). 별도 실제 주행 Atlas reuse는 아직 실험 단계다. 이 도구가 `atlas_reuse_verified`를 자동으로 true로 만들지 않는다. 기존 단계 판정을 그대로 노출한다. 첫 실기는 새 지도 생성부터 시작한다.

## 로컬 검증 자료

**전송 누락 후속 수정:** [전송·Atlas 조사 기록](stereo-transport-and-atlas-debug.md). Fast DDS SHM512 KiB에서640×480 누락을 재현했고 segment만32 MiB로 늘린 대조 시험에서는 좌우300쌍을 두 consumer 모두 수신했다. 실제 크기30초 probe도 정상900/899·exact899로 통과했으며 profile 변경2회는 계속 거부했다. 카메라를 포함한 각 프로세스를 시작하기 전 실행 문서의 DDS 환경을 적용한다. 아래64×48 결과는 이전 기능 시험 이력이다. Pi/Jazzy 실제 전송 성능은 아직 미검증이다.

`outputs/headless_mapping/20261009_field_tools/`에 실패/성공 입력·ROS fixture probe·실제 기존 MCAP의 리포트를 보존한다. 작은 합성 ROS 입력은 **64×48 30 Hz 기능 검사**이며 D435/Pi의 실제640×480 전송률이나 35° 영상 검증이 아니다.

초기 합성640×480 시험에서는 Python publisher bytes 변환에 약20.8 ms/image가 걸려 수신16.5 Hz로 거부됐다. array 전송으로 바꾼 뒤에도 동시 best-effort 전송에서 좌우 누락이 확인돼 준비 완료를 거부했다. 판정 기준을 완화하지 않고 작은 입력으로 기능/부하를 분리했다. 실패 자료도 보존한다.

현재 source의 mapping Python65개와 native adapter build를 재실행해 통과했다. 설치된 CLI에서도 정상 probe exit0, profile 변경 probe exit1, 기존 load05 report exit0을 확인했다. 준비된 새 fixture run에 event를 append하고 촬영 시각을 만들지 않는 것을 확인했다. 기존 stereo05/load05는 evidence_complete=true, 잘린 storage_failure01은 false다.

ROS 합성64×48 30초 시험에서 정상 좌901/우900·exact900개로 capture_ready=true, profile 변경 입력은 변경2회로 capture_ready=false였다. 이 작은 입력의 성공은 앞선 실제 크기640×480 전송 실패를 해결한 것으로 주장하지 않는다. 로컬 Ubuntu22.04/Humble host inventory도 목표 Pi24.04/Jazzy와 다르므로 정상적으로 exit1이었다.

전체 저장소 `python3 -m pytest -q`는 기존 `ros2_ws/src/rover_description/scripts/test_mesh_inertia.py`, `test_mesh_preparation.py`가 `trimesh` 미설치로 collection error2개를 내어 실행되지 못했다. 이번 작업에서 관련 없는 mesh 코드나 시스템 의존성은 수정하지 않았다. mapping 범위의 결과와 전체 저장소 결과를 구분한다.
