# Depth 구간 정합과 전역 최적화를 이용한 사전 지도 제작

상태: 대화에서 방향 승인, 이 문서의 검토 대기. 구현·성능 검증 완료 문서가 아니다.

## 목적과 승인된 방향

실기장에는 사전 지도가 없으므로 D435 기록으로 경기 STL을 대신할 기준 형상을 만든다. 모래 알갱이와 미세 요철의 충실한 재현보다 전체 경사·높낮이·큰 돌·단차·경계의 위치와 형태를 우선한다. 센서 수집은 Pi, 저장과 무거운 재처리는 노트북에서 수행한다.

기존 방식은 ORB 최종 keyframe 위치로 depth를 TSDF에 융합한다. 새 방식은 ORB 위치를 초기값으로 사용하고 depth fragment 정합과 전역 pose graph 최적화를 추가한 뒤 원본 depth를 다시 융합한다. ORB 결과와 원본은 보존한다. COLMAP pose와 동일 depth의 비교는 후속 후보이며 이번 구현에 COLMAP 설치·실행을 포함하지 않는다.

최종 사용자 흐름은 지도 제작 개선 → 중력 정렬·단순화·영역 편집·출발 위치와 방향 지정 → 고정 사전 지도에 대한 독립 주행 보정 시험이다. 이 문서는 첫 번째인 **오프라인 depth 정합·전역 최적화·재융합**에 한정한다. UI 편집, IMU 중력 추정, 다중 촬영 병합, 주행 중 위치 보정은 별도 변경으로 진행한다.

## 현재 코드와 실제 자료

- `experiments/handheld_mapping/recording.py`: 원본 인덱스와 keyframe을 디스크 기반으로 정렬하고 2 ms 이내 depth를 일대일 매칭한다.
- `experiments/handheld_mapping/reconstruct.py`: depth scale·extrinsic·pose를 검증하고 지도별 `ScalableTSDFVolume(NoColor)`를 생성한다. 일반 stereo 결과는 초기 optical 좌표이며 중력 정렬을 적용하지 않는다. 관성 결과의 중력도 검증 완료 상태가 아니다.
- `experiments/handheld_mapping/viewer.py`: 저장한 mesh/cloud를 지도별로 표시한다. 서로 다른 좌표계의 지도는 별도로 유지한다.
- 설치 기준은 Python 3.10, NumPy 1.26.4, SciPy 1.13.1, Open3D 0.19.0이다. 실행 환경은 `/home/su/rover/.venv-mapping`, `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=2`, `OPENBLAS_NUM_THREADS=1`이다. Conda `robot_lab`은 이 변경의 실행 환경이 아니다.
- 첫 실제 재처리 후보: `/home/su/rover/outputs/handheld_mapping/sessions/20261016_002510_fc62bc`. `dense_offline/summary.json`은 map 0, 604 keyframe 융합을 기록한다. 정확도 검증 완료를 뜻하지 않는다.
- 짧은 입력·관성 경로 확인 후보: `20261015_235533_69b722/orb_inertial`의 최종 관성 map 244에는 10 keyframe이 있다. 충분한 fragment·재방문이 없을 때 성공을 과장하지 않는지 확인한다.
- 기존 `dense_colmap_d20` 등 이름의 실험 산출물도 발견됐다. 생성 설정·실행 provenance와 축척은 재확인 전 성능 비교의 근거로 쓰지 않는다. 과거 문서의 COLMAP 실행 여부 설명만으로 현재 산출물의 존재를 부정하지 않는다.

## 입력과 결과 인터페이스

새 진입점은 `experiments.handheld_mapping.global_reconstruct`로 둔다. 필수 입력은 원본 RUN, 명시한 source backend(`orb`, `orb_offline`, `orb_inertial`), 하나의 map ID와 새 output 폴더다. 자동으로 가장 큰 지도·최신 지도를 고르지 않는다. 원본 RUN·다른 결과·기존 output 폴더를 덮어쓰지 않는다.

입력은 `capture_calibration.json`, `frames.jsonl`, `raw/depth` 및 source backend의 `map_<id>_keyframes.csv`다. `orb_inertial`은 기존처럼 최종 export에 관성 초기화가 확인된 map만 받는다. 유효하지 않은 calibration, SE3, 시각 중복, map ID 불일치, 원본 밖 경로는 명시한 오류로 종료한다.

결과 폴더는 다음 파일을 포함한다. geometry 경로는 결과 폴더 기준 상대 경로다.

| 파일 | 내용 |
|---|---|
| `summary.json` | source·map ID·설정·입력 해시·프레임 수·정합/최적화 상태·누락·검증 한계 |
| `fragments/manifest.json` | fragment 시간 범위, 원본 anchor pose, point 수, 소속 keyframe과 사용 depth |
| `fragments/fragment_<id>.ply` | 정합용 경량 local cloud, 노트북 디스크에 저장 |
| `registration.jsonl` | 후보별 initial/refined transform, correspondence·overlap·잔차·관측성, 채택/거부 사유 |
| `pose_graph.json` | 초기·최종 노드 및 연속/재방문 edge의 transform과 정보량 |
| `keyframes.csv` | source와 같은 timestamp·map ID를 유지한 최적화 camera pose |
| `baseline/mesh.ply`, `baseline/cloud.ply` | 새 결과와 동일한 관측·TSDF 설정의 비교 기준 |
| `optimized/mesh.ply`, `optimized/cloud.ply` | 최적화 pose로 원본 depth를 한 번씩 재융합한 결과 |

실행 시작에 `status=processing` manifest를 원자 저장하고, 성공·부분 완료·오류를 마지막 manifest에 기록한다. 중단된 폴더나 한쪽 geometry만 있는 상태를 성공으로 표시하지 않는다. 같은 output 경로에 재시도하지 않고 새 경로를 사용한다.

## 좌표와 fragment 구성

`P_i = T_world_left(i)`는 원본 keyframe pose, `E = T_left_depth`는 기록된 extrinsic이다. fragment f의 anchor는 첫 camera pose `A_f`다. fragment 안의 depth는 `A_f^-1 P_i E`로 local 좌표에 놓는다. 원본 world에 이미 놓인 cloud에 pose를 다시 곱하지 않는다.

keyframe을 timestamp 순으로 읽고 유효하게 매칭된 depth만 사용한다. 기본은 최대 32 keyframe, 인접 fragment의 overlap은 8 keyframe이다. 마지막 짧은 fragment를 허용하되 3개 미만 유효 프레임은 정합용 fragment로 사용하지 않는다. 설정 값과 실제 구성을 기록한다. 누락·시각 공백이 1 s를 초과하면 fragment를 나눈다. 서로 다른 map ID나 재부팅 뒤 다른 좌표계를 자동 연결하지 않는다.

정합용 fragment는 depth 투영과 2 cm voxel 축약으로 구성한다. 원본 depth를 바꾸거나 정합용 축약 cloud를 최종 재융합 입력으로 사용하지 않는다. frame당 점 수 100,000, fragment당 축약 점 수 100,000, fragment 수 256을 상한으로 두고 초과하면 명시한 오류를 낸다. 점 수 상한을 맞추기 위한 몰래 무작위 삭제는 하지 않는다.

이 단계의 world는 원본 ORB map의 world다. 카메라 장착각 20°만으로 중력 정렬됐다고 표시하거나, 가장 큰 지면 평면을 수평으로 펴지 않는다. 실제 경사·단차를 보존한다.

## 정합과 pose graph

인접 fragment의 초기값은 원본 anchor pose에서 계산한다. ICP는 coarse-to-fine point-to-plane에 robust loss를 사용하는 경로를 기본으로 하고 point-to-point를 진단 기준으로 유지한다. depth 기반 edge의 승인과 ORB 연속성 prior를 구분한다.

초기 실험 설정은 다음과 같으며, 실기 합격 기준이나 정확도 보증이 아니다. 모든 설정은 manifest에 저장하고 명시적으로 변경할 수 있게 한다.

| 설정 | 초기 값·정책 |
|---|---|
| 정합 voxel 단계 | 4, 2, 1 cm |
| 단계별 최대 대응 거리 | 12, 6, 3 cm |
| 법선 이웃 | 각 단계 voxel의 3배 반경, 최대 30점 |
| robust loss | Huber, 최종 단계 scale 2 cm; coarse 단계는 대응 거리 비율로 확대 |
| edge 대응점 | 양방향 각각 최소 100 |
| edge overlap | 양방향 각각 최소 0.30, 원래 point 수·유효 대응점도 기록 |
| 최종 inlier RMSE | 양방향 각각 2 cm 이하; 단독 승인 기준으로 사용하지 않음 |
| 초기값 대비 허용 변화 | translation 0.5 m, rotation 20° 이하 |
| 관측성 | 실제 point-to-plane Jacobian을 범위로 정규화한 normal matrix의 최소/최대 고유값 비가 1e-4 미만이면 full-SE3 depth edge 거부 |
| 최적화 제한 | node correction이 0.5 m 또는 20°를 넘거나 유효 rigid transform을 벗어나면 최적화 결과 거부 |

법선·수치 범위·correspondence가 유효하지 않거나 순수 평면 등으로 full-SE3가 관측되지 않으면 depth edge를 거부한다. Open3D helper의 information matrix를 보정된 sensor covariance로 표시하지 않는다. 대응점이 없을 때 RMSE=0인 경우도 명시적으로 실패 처리한다.

연속 prior edge는 원본 anchor 관계이며 depth 정합 성공으로 표시하지 않는다. depth edge가 거부됐다고 identity 측정을 넣지 않는다. 원본 prior만 남으면 기존 궤적과 같은 결과를 보존하고 추가 정합에 의한 개선은 없다고 표시한다.

재방문 후보는 fragment index 차이 3 이상, 원본 anchor 거리 1 m 이하인 범위에서 노드당 최대 8개로 제한한다. 제한된 prior-nearby 탐색이며, 큰 drift를 가진 unknown-start 전역 장소 인식으로 설명하지 않는다. FPFH+RANSAC은 이 후보의 초기값 비교용 선택 경로로 두고, 이후 동일한 양방향 정합·관측성·prior gate를 통과해야 한다. 최초 버전의 기본 경로는 ORB 초기값이다. uncertain loop edge와 연속 edge를 구분하고 첫 노드를 고정해 rigid pose graph를 최적화한다.

API의 edge transform은 source local → target local이며 node pose는 fragment local → world다. 비대칭 합성 형상과 회전·이동을 함께 넣는 검증으로 방향을 고정한다. 단일 평면만으로 transform 방향과 pose 정확도를 통과시키지 않는다.

## 최적화 pose와 재융합

최적화 fragment pose를 `F_f`라 할 때 world correction은 `C_f=F_f A_f^-1`이다. anchor timestamp 사이 correction의 translation은 선형, rotation은 SO(3) SLERP로 보간하고 양 끝은 가장 가까운 correction을 사용한다. 각 camera pose는 `P'_i=C_i P_i`로 생성한다. overlap에 속한 frame도 한 번만 출력하고 한 번만 최종 융합한다. 동일 map의 모든 유효 입력 timestamp와 개수를 대조한다.

최종 TSDF의 baseline과 optimized는 동일 keyframe/depth 집합, 동일 depth range, voxel과 truncation을 사용한다. 기본 비교는 2 cm voxel, 8 cm truncation, 0.25~3 m depth다. 기존 1 cm 결과와 직접 정확도 우위를 비교하지 않는다. 최적화 pose를 평가할 때 TSDF 설정 효과가 섞이지 않도록 새 baseline도 함께 생성한다. 원본 keyframe마다 `P'_i E`를 사용하고 Open3D가 받는 world→depth 방향으로 역변환한다.

강한 평활화, Poisson 보간, mesh decimation은 이 단계의 기본 결과에 적용하지 않는다. 큰 형상을 남기는 후처리 결과는 다음 단계에서 별도로 만든다. 보간된 면·미관측 영역을 실제 관측이나 안전 지면으로 표시하지 않는다.

최적화 결과가 prior gate를 벗어나면 optimized geometry를 채택하지 않고 baseline을 유지한다. fragment가 하나뿐이면 `optimization_status=insufficient_fragments`, prior만 사용했으면 `optimization_status=prior_only`, 유효 depth 제약으로 최적화를 수행했으면 `optimization_status=optimized_unverified`다. 지도 산출 상태와 알고리즘 상태를 구분하고 `accuracy_verified_on_real_camera=false`, `gravity_alignment_verified=false`를 유지한다. loop가 없는 경우 `loop_closure_used=false`다.

## 메모리와 기존 기능 연결

전체 영상이나 전체 frame 인덱스를 RAM에 적재하지 않는다. 기존 `matched_depths`를 재사용하고 depth는 한 장씩 읽는다. fragment cloud는 디스크에서 필요할 때 읽고 최대 두 개만 cache한다. pose graph는 위 상한 안에서만 구성한다. 최종 TSDF는 map별 하나씩 baseline·optimized 순서로 생성한다. TSDF·native allocation을 포함한 전체 RSS가 점 수 상한만으로 고정된다고 주장하지 않고 실제 peak RSS를 기록한다.

`reconstruct.py`에는 명시한 pose iterator와 output destination을 받는 공통 융합 경로를 추가하되, 기존 `reconstruct(...)`의 기본 출력·검증·map 분리 동작은 유지한다. 새 global 모듈이 현재 매핑 앱의 촬영·모터·IMU 수집을 변경하지 않는다. 첫 단계는 수동 CLI 재처리로 제공하고 자동 실행·UI 편집 연결은 후속 설계에서 추가한다.

뷰어는 새 result manifest를 명시적으로 열었을 때 baseline/optimized를 선택하고 source/map ID/최적화 상태를 표시하도록 확장한다. 자동 선택으로 기존 촬영 결과를 새 최적화 결과로 바꾸지 않는다. geometry 파일과 경로를 검증하며 다른 map을 한 world로 그리지 않는다.

## 검증과 완료 조건

1. 기록 입력: timestamp 정렬·일대일 매칭·누락·map ID·경로·calibration·관성 export gate를 기존 동작과 함께 검사한다.
2. 좌표: 알려진 비대칭 3D 모서리와 경사·단차 합성 기록에서 extrinsic과 회전/이동, anchor→world, edge 방향, 최종 world→depth가 맞는지 확인한다.
3. 보정: 알려진 synthetic pose drift를 넣은 기록에서 baseline보다 pose·표면 오차가 감소하는지 확인한다. scale·비강체 정렬로 오차를 숨기지 않는다.
4. 거부: 순수 평면, 대응점 0, 낮은 overlap, 반복 구조의 틀린 loop, prior를 크게 벗어난 후보, 최적화 폭주를 거부한다. baseline을 변경하지 않는다.
5. 연속성: overlap frame 중복 융합 없이 원본 timestamp와 개수를 보존하고 correction 보간 경계의 rigid transform·불연속을 검사한다.
6. 실제 재처리: 위 604 keyframe 기록을 source와 동일 calibration·depth 집합으로 처리한다. 초기/최종 graph와 후보 승인·거부, geometry 변화량·coverage·처리시간·peak RSS를 저장한다. 외관·잔차 개선을 절대 정확도 개선으로 표시하지 않는다.
7. 짧은 기록: 10 keyframe 입력은 부족한 fragment/재방문 상태를 정확히 보고하고, 단순 baseline 생성으로 성공을 과장하지 않는다.
8. 회귀: 기존 매핑·TSDF·viewer 관련 검사를 실행한다. 실제 로버 제어는 이번 변경의 검증에 필요하지 않다.

완료는 새 재처리 경로와 실패 거부가 실행되고 재현 가능한 결과·비교 보고서가 저장되는 상태다. 실기 지형 정확도와 독립 주행 B의 보정 성공은 별도 측정으로 남긴다. 기준 지도 A를 동결한 뒤 B를 시험하며 B 미래 관측이나 종료 후 pose를 주행 중 보정에 넣지 않는다.

## 후속 순서

1. 중력 정렬과 실제 경사 보존 검증, 큰 형상 위주 후처리.
2. 시작점·heading·crop·촬영 경로를 지정/표시하는 UI와 편집 metadata 저장.
3. 새 촬영과 기존 지도의 정합 미리보기·검증·버전 저장.
4. 고정 A/STL에 known-start multi-scale robust ICP를 적용하고 독립 B 위치 오차·오보정을 비교.

## 공식 근거

- [Open3D 0.19 reconstruction system](https://www.open3d.org/docs/0.19.0/tutorial/reconstruction_system/system_overview.html)
- [Multiway registration와 pose graph](https://www.open3d.org/docs/0.19.0/tutorial/pipelines/multiway_registration.html)
- [Multi-scale ICP와 대응점 없는 결과](https://www.open3d.org/docs/0.19.0/tutorial/t_pipelines/t_icp_registration.html)
- [Robust kernels](https://www.open3d.org/docs/0.19.0/tutorial/pipelines/robust_kernels.html)
- [최종 pose로 원본 재통합](https://www.open3d.org/docs/0.19.0/tutorial/reconstruction_system/integrate_scene.html)
- [기존 기준 지도·독립 주행 로드맵](../../experiments/2026-10-13-reference-map-roadmap.md)
