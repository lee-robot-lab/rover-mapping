# 사전 지도 제작과 편집 전체 흐름

2026-10-16 사용자 지시: 전역 재구성과 후속 기능까지 한 번에 구현한다. 기존 depth-fragment 설계를 기반으로 RGB·중력·편집·반복 촬영 병합을 포함한다. 원본 보존·관측 부족 시 거부·실제 경사 보존은 공통 조건이다.

## 사용자 흐름

1. 매핑 앱에서 RGB 수집, IMU 기록, 종료 후 전역 재구성을 선택한다. RGB와 전역 재구성은 새 앱에서 기본 선택하되, 합성 입력·구형 기록에는 RGB가 없어도 기존 경로를 유지한다.
2. Pi는 IR/depth와 선택 RGB를 RAM 기반 전송 큐로 보내고 노트북에 저장한다. RGB 수집 실패를 성공으로 숨기거나 색상 없이 컬러 지도라고 표시하지 않는다. 기본 지도 depth 범위는 0.25~1 m다.
3. 촬영 종료 후 기존 ORB/TSDF 결과를 보존하고 지도별 depth fragment graph와 별도 baseline/optimized 결과를 만든다. 최종 관성 export가 유효하면 해당 결과를 사용할 수 있지만 분리 map을 임의 병합하지 않는다.
4. 지도 후처리 창에서 위쪽 3D 지도, 아래쪽 평면도와 경로를 동시에 본다. 원본 촬영 시작점과 사용자가 지정한 다음 주행의 출발 위치·방향을 구분한다.
5. IMU로 중력 후보를 추정하거나 명시한 위쪽 방향을 지정한다. 정지·시계·좌표 정보가 부족하면 자동 적용을 거부한다. 사용자 확인한 정렬 근거를 metadata에 기록하고 지형 평면을 자동 수평화하지 않는다.
6. 다각형과 높이 범위로 필요한 부분을 남긴다. 삭제 후보/적용 결과를 미리 보고 되돌릴 수 있다. 새 폴더에 mesh/cloud·경로·시작 pose·좌표 변환·crop·출처를 저장한다.
7. 추가 촬영 지도는 초기 위치·방향을 명시해 정합 미리보기를 만든다. 양방향 overlap·잔차·관측성·보정 크기 검증을 통과한 경우에만 새 버전으로 합친다. 이전 버전은 그대로 두며 평가 주행 B 중 기준 지도 A는 바꾸지 않는다.

## RGB와 융합 계약

- Pi worker의 `--rgb`가 color stream을 추가한다. root controller가 `rgb_enabled` config를 flag로 전달한다. worker 자체 기본은 기존 3-stream과 호환한다.
- color calibration에는 intrinsics·distortion·frame ID 및 `T_color_depth`(depth 좌표→color optical)를 저장한다. RGB/depth의 다른 시점을 이미지 크기만 맞춰 혼합하지 않는다.
- 노트북은 `rgb8`와 `bgr8`를 검증해 `raw/color/*.png`에 BGR PNG로 보존하고 원래 encoding을 인덱스에 남긴다. IR와 depth encoding 검증은 유지한다.
- keyframe depth와 가까운 RGB를 제한된 tolerance 안에서 매칭하고 depth 3D 점을 color 카메라에 투영해 depth grid의 색을 만든다. 왜곡 모델·extrinsic·occlusion·미관측 pixel을 처리한다. RGB 누락은 융합 geometry를 잃지 않게 보존하고 color coverage와 부분 상태를 기록한다.
- `reconstruct`는 기존 positional 인자를 유지하고 keyword-only `keyframe_dir=None`, `output_dir=None`, `color=False`를 추가한다. 새 호출은 원본 run에서 calibration/depth를 읽고 명시한 pose와 output을 사용한다. 기존 기본 경로·summary 경로 의미는 유지한다. explicit output의 geometry 경로는 summary 폴더 기준이며 `paths_relative_to='summary'`를 저장한다.

## 전역 재구성 계약

기존 [depth-fragment 설계](2026-10-16-depth-fragment-reconstruction-design.md)를 따른다. `global_reconstruct.run_global(run, output, backend='orb_offline', map_id=0, depth_max=1.0, voxel_size=.02, color=False, progress=None, ...)`를 앱과 CLI에서 사용한다. 새 output만 허용한다. `registration.register_clouds(source, target, initial, voxel_size=.02)`는 `accepted`, `transformation`, `information`, 양방향 fitness/RMSE/correspondence, 관측성·거부 사유를 반환한다. 실패 결과의 변환을 적용하지 않는다.

전체 map을 전역 최적화했다고 무조건 정확도 개선으로 표시하지 않는다. 제약 부족·prior-only·partial·최적화 거부를 구분하며 별도 지도 간 정합과 단일 촬영 내부 최적화를 혼동하지 않는다.

## 후처리와 반복 병합 계약

- `reference.py`는 source 읽기·rigid 정렬·crop·출발 pose·경로·저장·병합을 담당하고 `reference_editor.py`는 Qt+Matplotlib 화면과 이벤트를 담당한다. core는 GUI 없이 검증 가능하게 한다.
- source는 legacy RUN의 명시 map, explicit TSDF output, global result의 baseline/optimized, 저장한 reference version을 지원한다. 경로·map ID·단위·SE3를 검증한다. STL 직접 입력은 단위가 meter라는 사용자 확인과 trajectory 없음 표시를 요구한다.
- `reference.json`에 schema version, source hash·map ID·meter 단위, `T_reference_source`, gravity source/quality, ROI polygon·height, 원본 촬영 시작 pose, 다음 주행 시작 pose, 경로 segment와 parent version을 저장한다. 좌표 변환은 geometry와 pose 모두에 동일 적용하고 이중 적용하지 않는다.
- 경로는 같은 좌표계의 시간 순서 camera pose를 URDF extrinsic으로 base pose에 바꿀 수 있을 때만 rover 경로라 부른다. 정보가 없으면 camera 경로로 표시한다. map reset·긴 시각 공백·잘린 구역 사이를 선으로 연결하지 않는다.
- 시작점은 평면도 두 클릭으로 위치·heading을 지정하고 높이는 선택한 surface hit/명시한 높이로 결정한다. 이는 초기 prior이며 정확한 실측 pose나 localization 성공으로 표시하지 않는다.
- 반복 병합은 후보→기준 변환을 명시한 초기값으로 계산하고 preview 상태를 저장 상태와 분리한다. 검증 실패 시 저장 버튼을 막고 원본·기준을 변경하지 않는다. 기하 병합은 검증된 cloud/mesh의 통합 버전이며 원본 sensor를 공동 재최적화한 결과나 watertight surface라고 주장하지 않는다.
- raw geometry는 보존하고 표시용 경량화와 metric 분석 형상을 구분한다. unknown을 Poisson 보간 등으로 안전 지면으로 만들지 않는다.

## 검증과 배포

합성 RGB/depth에서 다른 해상도·extrinsic·왜곡·가림·누락·시각 불일치와 raw 보존을 검사한다. 비대칭 3D 장면의 known drift로 정합 방향과 보정을 검사하고 평면·0 correspondence·오정합은 거부한다. 경사진 지형의 중력 정렬, 좌표/heading/crop 저장 재로드, 추가 map merge 수용/거부를 검사한다. Qt offscreen에서 실제 버튼·클릭과 비동기 작업을 검증한다.

기존 604 keyframe 기록은 RGB가 없는 실제 데이터 재처리에 사용하고, RGB 수집 및 Pi 전체 부하는 다음 장비 연결 때 확인한다. 문서·시험과 실제 연결 검증을 구분한다. 검증 후 shared repo와 `/home/su/rover` 실행 코드를 일치시키며 모터 명령·firmware 변경·자동 주행은 수행하지 않는다.
