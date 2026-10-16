# Stereo ORB + 외부 IMU 자세 보조 실험

ORB 위치는 유지하고, body축 자이로 회전 예측에 ORB 회전·가속도 기반 기울기 피드백을 적용하는 SO(3) 상보 필터다. ORB 내부 IMU 모드를 사용하지 않으며 native 관성 초기화 성공 여부에 종속되지 않는다. 가속도 위치 적분, ORB 내부 특징점 추적 개선, 지도 병합은 수행하지 않는다. offline 실험 CLI이며 GUI·실시간 제어 출력에는 아직 연결하지 않았다.

## 사용법

노트북의 `.venv-mapping` Python3.10 환경을 사용한다. Conda와 Pi 연결은 필요 없다. 원본에는 `imu.jsonl`, `frames.jsonl`, `orb_live/live_poses.csv`, 보정·URDF provenance 파일이 필요하다.

```bash
cd /home/su/rover
bash tools/attitude_fusion.sh /absolute/path/to/RUN --segment 0 --output /absolute/path/to/NEW_OUTPUT
```

기본으로 기존 시간 정렬 진단의 회전 상관 조건을 통과해야 한다. 정지·시각 공백·MCU reset으로 실패하면 해당 입력의 원인을 확인한다. reset을 가로질러 적분하지 않고, `--segment`로 단일 boot를 선택한다. `--correction-s`는 **해당 기록에서 얻은 미검증 실험용 후보**를 수동 지정하는 옵션이다. 다른 기록에 일괄 적용하지 않는다.

이번 실제 기록 실행:

```bash
bash tools/attitude_fusion.sh \
  outputs/handheld_mapping/sessions/20261016_004946_f86cc9 \
  --segment 0 --correction-s -0.04 \
  --output outputs/handheld_mapping/20261016_external_attitude_v2
```

이미 있는 결과 폴더는 거부한다. 재실행은 새로운 `--output`을 지정한다. 이 명령은 결과가 존재하는 현재 상태에서는 덮어쓰지 않고 실패한다.

## 출력과 좌표

- `poses.csv`: 보정 camera→map 자세. 유효 ORB 위치 XYZ는 원본 값과 정확히 같다. `map_id`는 입력 ORB 상태, `orientation_map_id`는 회전이 실제로 속한 마지막 유효 map을 뜻한다.
- `position_valid=0`인 행의 XYZ는 NaN이다. 추적 손실 중 최대0.3초 gyro 회전 예측은 `gyro_only_position_invalid`이며 새 위치로 사용하면 안 된다. 이후 회전도 unavailable로 표시한다.
- `alignment.json`: 시각 모델·선택 boot·보정 후보·원본 hash.
- `summary.json`: 처리시간·fallback 상태·기존 native 관성 실행의 별도 요약.
- `comparison.png`: 같은 map 중력 기준의 roll/pitch와 ORB 대비 회전 변화량. 마지막 패널은 저장된 live stereo 위치, 외부 자세 출력, 기존 offline native 관성 초기화의 가용 구간을 표시한다. 서로 다른 map의 회전을 직접 오차로 비교하지 않는다.

map 변경, 큰 위치/회전 점프, 긴 손실 뒤에는 ORB 좌표로 재기준을 잡는다. IMU 시간 공백에는 visual fallback을 사용한다. 정지 후보는 최근0.5초의 낮은 자이로·가속도 변화로 판단한다. 그 구간의 평균 가속도를 현재 ORB body 회전으로 map에 놓아 중력 기준을 설정한다. 절대 heading이나 세계 좌표 정합을 제공하지 않는다. 움직임 중 가속도 norm gate만으로 선형 가속도를 완전히 구별할 수 없으므로 피드백은 실험용이다.

## 실기 기록 재생 결과

2,449프레임을 별도 노트북 출력에 재생했다. 융합 자세2,138행, 재기준6행, 짧은 gyro 예측17행, unavailable288행이었다. 유효 원본 위치가 모두 정확히 보존되고, 유효 회전이 SO(3)에 속하며, 무효 위치가 NaN인 것을 검사했다. gyro-only 행에는 회전 기준 map ID를 남긴다.

추가 처리시간 중앙값 약0.26ms/p95 약0.33ms는 **노트북에서 필터 계산만** 측정한 값이다. ORB 계산·카메라·전송·그림 저장은 포함하지 않는다. Pi 성능 결과가 아니다. 원본 ORB와 자세 차이의 중앙값 약0.33°/p95 약0.75°는 보정량이며 정확도 오차가 아니다.

기존 native 관성 실행과 live stereo는 처리 조건·추적/reset 결과가 달라 추적 비율만으로 우열을 정하지 않는다. 외부 자세 보조의 위치 유효 비율은 입력 ORB와 동일하다. `imu_clock_verified=false`, `extrinsic_verified=false`, `accuracy_evaluated=false`를 유지한다. 다음 실기에서는 기본 stereo와 같은 동작·센서 입력으로 반복해 기울기 흔들림과 추적 손실 구간의 회전 예측을 독립 기준과 비교한다.
