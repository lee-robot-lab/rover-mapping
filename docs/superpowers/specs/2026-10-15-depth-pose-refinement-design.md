# ORB 궤적과 D435 depth 기반 기준 맵 개선 설계

작성일: 2026-10-15. 상태: 구현 전 검토용 설계.

## 목적과 합의한 방향

실기 장소의 기준 맵을 노트북에서 만들고, 이후 경기 STL처럼 별도 주행의 누적오차 보정에 사용한다. 사용자는 실시간 처리보다 촘촘성·정확성·안정성을 우선하며, 지금은 장비 연결 없이 코드 준비를 요청했다. COLMAP 전환보다 현재 ORB-SLAM3 + D435 depth + TSDF 방식을 먼저 개선한다.

첫 구현 범위는 **저장된 기록의 depth 정합 → ORB 궤적 보정 → TSDF 재생성 → 전후 비교**다. 프로젝터 ON 선택·검증, IMU 시간 정렬, RGB 기록·색상 지도 요청도 유지한다. 이들은 수집 경로의 후속 작업이며 이번 depth 보정에 미검증 IMU나 RGB를 필수 입력으로 넣지 않는다.

## 현재 코드와 실제 입력

- `experiments/handheld_mapping/offline.py`: 저장된 exact timestamp 좌우 IR로 native ORB-SLAM3를 재실행한다.
- `experiments/handheld_mapping/reconstruct.py`: 최종 map별 keyframe pose와 depth를 최대 2ms 차이로 매칭해 Open3D TSDF에 융합한다. 별도 depth 정합이나 pose graph 최적화는 없다.
- 기존 표면은 1cm voxel, 0.25–3m depth 범위, 색상 없는 PLY mesh/cloud다. voxel 크기는 실측 정확도가 아니다.
- 20261015_000854_aa8850: 원본 depth 1016장, offline map0 최종 keyframe 276개.
- 20261015_001142_c10be6: 원본 depth 1703장, offline map0 최종 keyframe 310개.
- 기존 ORB는 non-inertial stereo다. IMU·encoder는 동시 기록이며 관성 융합은 없다. 지도는 map별 초기 camera optical 좌표이고 중력 정렬은 별도다.

## 선택한 방법과 대안

추천은 **ORB를 초기 궤적과 연결 제약으로 유지하고, 검증된 depth 정합만 추가하는 보수적인 후처리**다. 기존 기록으로 실행 가능하고 촬영 중 추적·제어 경로와 독립적이다.

depth만으로 궤적을 처음부터 계산하는 방법은 평평한 바닥에서 위치를 구분하기 어려워 첫 버전으로 선택하지 않는다. COLMAP의 영상 기반 SfM/MVS는 추후 독립 비교 후보로 남긴다. 색상 추가만으로 형상 정확도가 개선됐다고 판단하지 않는다.

## 처리 흐름

1. 입력 RUN·backend·map을 명시적으로 선택하고 최종 keyframe/depth를 순차 매칭한다.
2. 유효 depth에서 범위 밖 값만 제거하고, 정합용으로 downsample한 point cloud와 법선을 만든다. 원본 depth와 작은 단차를 보존하며 hole filling이나 강한 평활화는 기본 적용하지 않는다.
3. 시간상 인접한 keyframe과 ORB 위치상 재방문 후보에서 제한된 수의 쌍을 선택한다. 모든 쌍을 비교하지 않는다.
4. ORB 상대 자세로 시작해 coarse-to-fine robust point-to-plane ICP를 수행한다.
5. 대응 수·양방향 overlap·잔차·보정 크기·기하 퇴화를 검사한다. 통과한 쌍만 depth 제약으로 채택한다.
6. ORB 인접 상대 자세와 채택한 depth 제약으로 map별 pose graph를 최적화한다. 첫 노드를 고정해 원점·좌표계를 유지한다.
7. 보정된 최종 자세로 원본 depth를 다시 TSDF에 융합한다. 입력 pose와 기존 mesh는 덮어쓰지 않는다.
8. 선택·거부된 제약, 보정량, 처리시간, 메모리 범위와 전후 표면 일관성을 보고한다.

정합이 불충분하면 해당 제약을 거부한다. 채택 제약이 없거나 최적화가 비정상인 map은 원래 ORB pose로 남기고 `unchanged` 또는 `rejected`로 보고한다. 빈 결과나 단순 복사 결과를 개선 성공으로 표시하지 않는다.

## 좌표와 시각

`T_A_B`는 B 좌표의 점을 A로 변환한다. 기존 keyframe CSV는 `T_world_left`, 보정값은 `T_left_depth`다.

- ICP 노드: `T_world_depth = T_world_left @ T_left_depth`.
- source i → target j 초기값: `inverse(T_world_depth_j) @ T_world_depth_i`.
- 보정 후 keyframe: `T_world_left_refined = T_world_depth_refined @ inverse(T_left_depth)`.

카메라 baseline·intrinsics·depth↔IR 변환은 RUN의 실측 보정값을 그대로 사용한다. 20° URDF를 중력 기준으로 간주하지 않는다. 서로 다른 ORB map은 독립적으로 처리하고 임의 병합하지 않는다. 매칭은 기존 2ms 허용값과 일대일 depth 사용 규칙을 유지한다.

## 정합 제약과 검증할 초기 설정

다음 값은 장비·지형에서 검증되지 않은 **실험 초기값**이며 결과 metadata에 기록한다.

| 항목 | 초기 설정 |
|---|---|
| 정합 voxel 단계 | 4cm → 2cm → 1cm |
| 대응 거리 단계 | 12cm → 6cm → 3cm |
| robust kernel | point-to-plane Tukey, 최종 scale 2cm |
| 쌍별 최소 대응 수 | 500 |
| 양방향 최소 overlap | 각각 30% |
| 최대 최종 inlier RMSE | 2cm |
| ORB 초기값 대비 최대 보정 | 이동 10cm, 회전 5° |
| 시간상 후보 | 각 노드의 다음 3개 노드까지 |
| 재방문 후보 | 시간 차 3초 이상, ORB 거리 1m 이내, 노드당 최대 5개 |
| 정합 cloud | 노드당 최대 20,000점, 캐시 최대 8개 |

높은 overlap·낮은 잔차만으로 정합을 승인하지 않는다. target 좌표로 옮긴 유효 대응점의 중심을 원점으로 삼은 point-to-plane Jacobian `J = [p × n, n]`에서 회전 열을 대응점의 RMS 반경(m)으로 나눠 길이 단위를 정규화한 뒤 `J.T @ J`를 검사한다. RMS 반경이 1cm 미만이거나 최소/최대 고유값 비가 1e-4 미만이면 거부한다. 이 임계값도 실험 초기 설정이다. 단일 평면 등 6DoF를 구속하지 못하는 쌍은 첫 버전에서 보수적으로 거부하고 ORB 제약을 유지한다. Open3D의 일반 information matrix를 실측 covariance나 평면 퇴화 검증의 대체로 사용하지 않는다.

depth 제약은 불확실한 제약으로 넣고 graph 최적화의 outlier 처리를 적용한다. ORB 연결 가중치와 depth 가중치는 설정·보고되는 heuristic이며 검증된 확률 모델로 표현하지 않는다. ORB 근처에 있다는 이유만으로 재방문 제약을 확정하지 않는다.

## 실행·출력·메모리

첫 버전은 노트북의 명시적 offline CLI로 제공한다. GUI 기본 매핑이나 Pi runtime을 자동 교체하지 않는다. Python 실행 환경은 기존 `.venv-mapping`, Open3D·NumPy를 사용하고 새 ML 환경을 요구하지 않는다.

기존 RUN 하위의 새 결과 디렉터리를 만들고, 이미 존재하면 종료한다. 재융합은 원래 backend 폴더를 수정하지 않고 명시적인 pose 입력·출력 폴더를 받도록 기존 TSDF 함수를 확장한다. 결과에는 다음을 포함한다.

- 입력 RUN/backend/map, calibration hash, 설정과 소프트웨어 버전.
- map별 원래/보정 keyframe CSV, pose graph, 채택·거부 사유를 포함한 pair CSV.
- 보정 TSDF mesh/cloud와 summary, baseline/보정 비교 보고서.
- 원본 변경 없음 확인, baseline 대비 이동·회전 보정 분포, 작업시간과 최대 cache 점 수.

전체 촬영 영상을 RAM에 올리지 않는다. 기존 SQLite 임시 인덱스·순차 depth 매칭을 재사용하고, cloud cache를 제한한다. graph 노드·제약 metadata는 선택 keyframe 수에 비례한다. 첫 버전의 map당 keyframe 한도는 2000개다. 초과 입력은 처리 한도를 명시해 종료하며 조용히 일부 데이터만 사용하지 않는다. Pi에는 정합 cloud·지도·원본을 저장하지 않는다.

## 정확도·퇴화 검증

장비 없이 먼저 다음을 검증한다.

- 알려진 변환으로 만든 여러 방향의 평면·돌출 구조: 변환 방향과 단위, 합성 pose 오차 감소.
- 단일 평면: 관측 불가능한 이동·회전을 만들어내지 않고 depth 제약 거부.
- 겹침 부족·outlier·잘못된 초기 pose: 거부 사유와 기존 pose 보존.
- 둘 이상의 map, depth 누락·시간 불일치, 잘못된 보정·SE3: 오염이나 임의 연결 방지.
- 알려진 drift가 있는 합성 loop: graph 보정 전후 GT pose 오차와 이중 표면 감소.
- 같은 실제 기록의 baseline/후처리: 같은 depth 범위·voxel·평가 구역으로 비교하고 원본 hash 보존.

실제 기록의 정합 잔차 감소는 독립적인 위치 정확도 향상이 아니다. 시각순 keyframe의 매 5번째 관측은 depth 정합에서 제외하고, depth 제약 없이 ORB 연결로만 위치를 정하는 hold-out 관측으로 표면 일관성을 별도 보고한다. 평가용 baseline/보정 TSDF 양쪽 모두 이 hold-out depth를 융합에서 제외한다. 최종 배포 후보 mesh는 평가와 구분해 모든 유효 depth로 다시 만든다. hold-out의 pose 자체도 ORB/graph 추정이므로 독립 GT로 표시하지 않는다. 알려진 거리·단차·시작점 재방문 실측을 확보하기 전에는 GT 정확도 향상을 주장하지 않는다. 독립 실측이 없는 기존 두 기록은 실행 가능성·표면 일관성 비교용이다.

## 후속 수집 변경의 적용 순서

1. 프로젝터 ON/OFF 선택과 실제 emitter parameter 확인·기록. ON이 IR 특징 추적을 악화시킬 수 있어 동일 조건 비교 후 기본값을 정한다.
2. MCU→Pi 시간차·속도 차 추정과 카메라 시간 출처/수신 시각 기록. 기존 기록의 약 0.6–0.7% 속도 차 및 45–55ms 움직임 상관 시차는 예비 진단으로만 보존한다. IMU 평균 구간과 SPI 지연 때문에 실기 검증 전 `imu_clock_verified=false`를 유지한다.
3. RGB 원본·intrinsics·depth↔RGB TF·시간차를 기록하고 색상 TSDF를 만든다. geometry 기준 맵과 색상 유무를 구분한다. 수집 부하와 IR 추적 누락을 함께 측정한다.
4. 시간·축·extrinsic·노이즈를 검증한 뒤 IMU 관성 융합 여부를 별도 결정한다.

## 검토할 결정

첫 구현은 저장 기록 후처리로 한정하고 원래 ORB pose·지도와 비교 가능하게 보존한다. 기하 퇴화 시 개선을 포기하는 보수적인 정책을 선택한다. 위 초기 설정은 실험 설정이며 실기 합격선이 아니다. 이 설계 검토 후 상세 구현 계획과 검증 순서를 작성한다.

## 근거

- 기존 코드: `experiments/handheld_mapping/{offline,reconstruct,recording}.py`.
- 기존 로드맵: `docs/experiments/2026-10-13-reference-map-roadmap.md`.
- [Open3D Multiway registration](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html).
- [Open3D Robust kernels](https://www.open3d.org/docs/release/tutorial/pipelines/robust_kernels.html).
- [RealSense ROS wrapper](https://github.com/realsenseai/realsense-ros): emitter/profile/metadata와 enable_sync 의미.
