# 기준 지도 만들기·편집 사용법

기준 날짜: 2026-10-16. 노트북 Python 3.10의 `.venv-mapping`에서 실행한다. Pi는 센서를 전송하고 원본 저장·무거운 재구성은 노트북이 담당한다. 기본 유효 depth 범위는 0.25~1 m다.

## 촬영부터 지도까지

```bash
bash tools/handheld_mapping.sh --projector-on
```

1. 기존 Pi·카메라·IMU 연결 절차를 완료한다. 앱에서 실제 보정 JSON과 IMU 기록 여부를 확인한다.
2. **RGB 함께 기록 · 컬러 지도**, **종료 후 depth 구간 정합·전역 재구성**은 실제 입력 모드에서 기본 ON이다. RGB를 끄면 기존 IR+depth 경로를 사용한다.
3. **매핑 시작**으로 기록하고, **종료 및 지도 생성**으로 저장·후처리한다. 종료 버튼은 모터 정지 명령이 아니다.
4. 완료 후 **3D 지도 보기** 또는 **지도 편집·출발점**을 누른다. 여러 ORB map이 있으면 하나를 선택한다.

위 영상은 RGB, 아래는 IR 관측 횟수 오버레이다. ORB 입력은 IR stereo이고 RGB는 컬러 표면용이다. RGB가 없거나 보정·동기 조건이 부족하면 geometry를 보존하고 색상 누락 사유와 coverage를 남긴다. 기존 RGB 없는 기록에서 색상을 복원할 수는 없다.

처리 순서는 전체 IR ORB → depth fragment → 검증된 양방향 ICP 제약 → pose graph → 보정 poses로 원본 depth TSDF 재융합이다. 같은 설정의 baseline도 별도 저장한다. 큰 보정이나 관측성이 부족한 정합은 거부하며, 거부 시 기존 poses로 재융합한다. ORB를 다른 SLAM으로 교체한 구현은 아니다.

## 기존 촬영 재처리

앱의 **기록에서 다시 생성**은 저장된 poses에서 표면을 다시 만든다. ORB 재추정 버튼은 아니다. CLI에서는 backend와 map ID를 명시하고 존재하지 않는 출력 폴더를 지정한다.

```bash
bash tools/reference_mapping.sh /path/to/RUN /path/to/NEW_RESULT \
  --backend orb_offline --map-id 0 --color

bash tools/edit_reference_map.sh /path/to/NEW_RESULT
# 기존 RUN을 직접 열 때:
bash tools/edit_reference_map.sh /path/to/RUN --backend orb_offline --map-id 0
# 경쟁장 STL은 좌표 단위가 실제 meter일 때만:
bash tools/edit_reference_map.sh /path/to/map.stl --stl-meters
```

결과에는 `baseline/`, `optimized/`, `keyframes.csv`, `summary.json`과 정합 진단이 생긴다. 앱 자동 처리 결과는 RUN의 `reference_reconstruction/<job>/map_<id>/`에 분리한다. 원본 촬영과 기존 dense 결과는 유지한다. 메모리 사용은 fragment·후보 수로 제한하지만 긴 기록의 노트북 저장 공간은 별도로 필요하다.

## 중력·crop·출발점 편집

위 화면은 3D, 아래는 XY다. 원본 촬영 시작점과 경로, 다음 주행에 사용할 출발 prior를 구분해서 표시한다.

1. **IMU 중력 후보**로 조건을 검사한다. 시각·장착 검증이 부족한 기록은 기본 거부한다. 미검증 시각을 허용하는 체크박스는 실험용이며 자동 적용하지 않는다. 후보를 살펴보고 **미리보기 적용**한다.
2. 수동 방식은 **불러온 지도 up x,y,z**에 해당 지도 좌표계의 위 방향을 지정하거나 roll/pitch를 적용한다. 지면의 경사를 수평으로 펴는 작업이 아니다. 동일 rigid transform을 geometry와 경로에 적용한다.
3. **ROI 다각형 클릭** 후 아래 XY 화면에 꼭짓점을 찍고 높이 min/max를 지정한다. **Crop 미리보기 → 미리보기 적용**으로 남길 영역을 확정한다.
4. **출발 위치 → heading 두 클릭**에서 첫 클릭은 다음 주행 위치, 두 번째는 바라볼 방향이다. 출발 높이도 지정한다. 녹색 표시는 수동 initial prior이며 자동 위치 인식 결과가 아니다.
5. 선택적으로 triangle 수를 줄이거나 약한 Taubin 처리를 미리 본다. 기본 smoothing은 OFF다. 작은 바위·단차가 사라질 수 있으므로 분석 원본과 비교한다.
6. **새 버전 저장**으로 새 폴더에 저장한다. Undo/원래 상태로 편집을 되돌릴 수 있다.

저장 결과는 `reference.json`, geometry, 경로·변환·crop·출발 prior·출처 hash와 `original/`을 포함한다. 저장본은 재로드할 수 있고 crop·smoothing을 다시 중복 적용하지 않는다. 출발 prior를 ORB 실행기에 자동 주입하는 주행 연결은 아직 없다.

## 반복 촬영 지도 병합

기준 지도를 열고 **후보 지도 열기**로 다른 결과를 선택한다. 후보→기준 초기 pose(x/y/z, yaw/roll/pitch)를 맞춘 뒤 **정합 미리보기**를 실행한다. 양방향 겹침·오차·관측성·보정 크기 조건을 통과해야 **통과 병합을 새 버전 저장**할 수 있다. 평면이나 겹침이 부족한 경우 거부되는 것이 정상이다.

이 병합은 정합된 geometry를 새 버전으로 합친다. 여러 촬영의 원본을 함께 최적화하고 TSDF 재융합하는 기능이나 주행 중 지속 map correction은 후속 단계다. 일반 STL을 ORB Atlas로 직접 불러오는 기능도 아니다.

## 확인한 결과와 남은 검증

- 최종 `tests/handheld_mapping` + `tests/headless_mapping` 검사: **387 passed, 10 skipped**. 독립 리뷰의 관련 검사 78개와 shell syntax·diff check도 통과했다. idle UI의 RGB/global 기본 설정과 원본 수집 없이 창 생성도 확인했다.
- 실제 604-keyframe 기록: 36 fragments, 수용한 depth 제약 10개, loop 0개. 제안된 최대 보정 1.398 m·24.997°가 제한 0.5 m·20°를 넘어 `optimization_rejected`로 기존 경로를 유지했다. 정확도 향상은 입증되지 않았다.
- 같은 기록의 baseline/보정 결과는 각각 60,176 vertices·103,241 triangles다. 약 25.7초, peak RSS 약 541 MiB는 이 노트북·기록의 한 번 측정값이다.
- 실제 지도 편집에서 crop·출발 prior·mesh 처리 후 저장/재로드 geometry·경로 일치와 source hash 불변을 확인했다. 합성 정합·loop·거부 경로도 회귀 검사한다.
- 실제 Pi RGB+IR+depth+IMU 동시 처리율, 색 보정·시간 오차, 중력 장착 검증, 외부 기준으로 측정한 지도 정확도, 별도 주행의 prior map localization은 남아 있다. 이 구현 중에는 장비·모터·firmware를 조작하지 않았다.

COLMAP 재구성, FPFH 기반 무초기값 전역 탐색, multi-session joint TSDF는 이번 구현에 포함하지 않았다. 먼저 고정 지도 A를 만들고 별도 주행 B로 정합·누적오차 보정을 평가한다.
