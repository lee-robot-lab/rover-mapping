# 카메라 오버레이와 저장된 3D 지도 보기

## 촬영 화면

```bash
cd /path/to/rover-mapping
bash tools/handheld_mapping.sh
```

왼쪽 위는 원본 IR, 왼쪽 아래는 같은 프레임의 관측 횟수 오버레이, 오른쪽은 전체 관측 격자다. 두 영상을 동시에 보며 비교할 수 있고 오버레이 ON/OFF 버튼은 추가하지 않았다. native 실시간 옵션을 켜야 관측 색을 계산한다.

노랑은 관측이 적거나 현재 처음 보는 유효 depth 표면, 초록은 해당 X-Z 셀의3회 이상 시점 변화 관측 참고값이다. 원본 grayscale PNG는 변경하지 않는다. depth→left extrinsic·양쪽 intrinsics로 IR에 재투영하고 z-buffer로 가까운 표면만 색칠한다. 색은 정밀3D품질/통행 가능성/독립적인 모든 과거 시점 수를 뜻하지 않는다. 현재 같은 X-Z 셀의 높이가 다른 표면을 구별하지 않는다는 제한도 있다.

유효 depth가 없거나 범위 밖/추적 손실/stale이면 색을 입히지 않는다. overlay 예외는 별도로 표시하고 raw 수집/최종 지도 계산은 유지한다. 처리 결과가 오래되면 최신 raw와 무색 영상으로 돌아간다. 촬영 종료 뒤에는 마지막 유효 원본/overlay 프레임을 정지 화면으로 유지하며 종료 프레임임을 표시한다.

오버레이는 Qt와 같은 최대5Hz로 렌더링한다. ORB는 들어오는15FPS를 계속 추적하고 원본 입력/저장은 줄이지 않는다. 첫 native vocabulary 로딩은 몇 초 필요할 수 있으므로 오른쪽 추적 정상 표시를 확인한 뒤 움직인다. 화면상의 수신 후 지연은 pose 수신까지이고 overlay 계산/센서/네트워크/화면 출력까지 합한 end-to-end 지연이 아니다.

[실제 기록 재생의 새 배치 (별도 자료)](../data-assets.md)

## 저장된 3D 지도

매핑 종료 뒤 앱의 **3D 지도 보기** 버튼으로 현재 촬영 결과를 연다. 새 앱에서 아직 촬영한 기록이 없으면 촬영 폴더를 선택한다. 촬영 중에는 버튼을 비활성화한다.

터미널에서 최신 촬영 결과:

```bash
bash /path/to/rover-mapping/tools/view_mapping.sh
```

특정 촬영 / 특정 지도 / point cloud:

```bash
bash /path/to/rover-mapping/tools/view_mapping.sh /path/to/rover-mapping/outputs/handheld_mapping/sessions/20261013_204443_766235
bash /path/to/rover-mapping/tools/view_mapping.sh /path/to/rover-mapping/outputs/handheld_mapping/sessions/20261013_204443_766235 --map-id 0
bash /path/to/rover-mapping/tools/view_mapping.sh /path/to/rover-mapping/outputs/handheld_mapping/sessions/20261013_204443_766235 --cloud
```

직접 PLY도 지정할 수 있다. 점선 결과는 `--backend orbline`, 기본은 저장된 point 결과(`orb_offline` 우선)다. 최신 촬영이 아직 지도 생성 중이면 이전 촬영을 몰래 표시하지 않고 오류를 안내한다. 출력 폴더를 기본 sessions 밖으로 지정한 촬영은 그 RUN을 명시한다.

## 같은 기록에서 최대 거리 1m·2m·3m 비교

`--depth-max`는 저장된 원본 depth와 최종 ORB pose로 별도 3D 지도·지형을 생성하고 연다. 첫 실행은 재처리 시간이 필요하고, 같은 입력·설정의 완성된 결과가 있으면 재사용한다. 옵션을 생략하면 기존 기본 지도를 연다. 실제 카메라 촬영 설정·원본 영상 저장·ORB 추적을 바꾸는 옵션은 아니다.

```bash
cd /home/parkshinyoung/rover-mapping
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261015_152413_6fea4a --depth-max 1
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261015_152413_6fea4a --depth-max 2
bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261015_152413_6fea4a --depth-max 3
```

각 명령은 해당 거리의 창을 열고 닫을 때까지 실행된다. 동시에 비교하려면 각 터미널에서 저장소로 이동한 뒤 한 거리씩 실행한다. 창 제목의 `depth_1m`·`depth_2m`·`depth_3m`으로 구분할 수 있다. `--cloud`, `--map-id`, `--no-trajectory`, `--no-terrain-colors`, `--backend`도 함께 쓸 수 있다.

창 없이 세 버전만 미리 생성하려면:

```bash
cd /home/parkshinyoung/rover-mapping
for mapping_depth in 1 2 3; do
  bash tools/view_mapping.sh outputs/handheld_mapping/sessions/20261015_152413_6fea4a \
    --depth-max "$mapping_depth" --build-only || break
done
```

결과는 원래 RUN의 `depth_variants/BACKEND/depth_1m/` 등으로 저장된다. 안에는 거리별 `dense*`, `terrain`, `depth_variant.json` 및 카메라 보정·최종 키프레임의 작은 사본이 있다. 원본 PNG를 복사하거나 수정하지 않고 기존 기본 dense/지형/세션 참조도 갱신하지 않는다. pose·보정 사본을 사용하므로 나중에 원본 pose가 변경되어도 이전 버전의 경로와 색이 뒤섞이지 않는다. 입력 인덱스·보정·키프레임·중력 기준이나 처리 코드가 변경되면 `_v2` 등 새 폴더를 만들고 이전 결과를 보존한다.

최대 거리는 **매 프레임의 카메라 정면 방향 optical Z depth**이며, 지도 원점 반경이나 3D 유클리드 거리가 아니다. 최소 거리는0.25m이고 `--depth-max`는0.3m 초과~10m 이하를 받는다. TSDF 융합, 공간·다중 시점 필터, 지형 투영에 같은 최대 거리를 적용한다.1/2/3m 비교는 같은 최종 pose·중력 변환·지형 격자 범위를 사용한다. 더 가까운 거리에서는 관측 범위와 반복 관측 근거가 줄어 unknown 또는 미융합 키프레임이 늘 수 있다. 유효 표면이 전혀 없으면 실패를 안내하며 기본 결과는 보존한다. 중력 기준이 없는 기록은 기본 3D와 경로만 만들고 지면 색상은 표시하지 않는다. 실제 지형 정확도나 한 거리의 우수성을 보증하는 옵션은 아니다.

파이썬 API로 별도 처리를 구성할 때는 `reconstruct(..., depth_max=1., output_root=...)`, `terrain_maps(..., depth_max=1., output_root=...)`를 사용할 수 있다. 낮은 수준의 CLI에도 `reconstruct --depth-max --backend --output-root`, `terrain_mapping.sh --depth-max`를 추가했다. 색상·경로·캐시가 함께 연결되는 일반 사용 경로는 위 `view_mapping.sh --depth-max`다.

최종 키프레임 CSV가 있는 지도는 같은 backend/map의 카메라 추정 경로도 함께 표시한다. 파란 선은 시간순 최종 키프레임 위치, 초록 구는 시작, 빨간 구는 끝이다. 최종 TSDF가 중력 기준 좌표로 저장됐으면 경로에도 summary의 `world_from_slam`을 적용하고 z-up 시점으로 연다. live pose와 섞지 않는다. 로봇 중심이나 지면으로 변환하지 않으므로 실측 로봇 궤적이 아니다. 경로는 화면에만 추가하며 PLY는 변경하지 않는다. `--no-trajectory`로 숨길 수 있다. mesh/cloud 모두 지원하고 임의 PLY 또는 최종 키프레임이 없는 기록은 지도만 연다.

같은 RUN/backend/map에 최신 지면 분리 결과가 있으면 기본적으로 **초록=지면, 주황=장애물 표면, 회색=미분류**로 표시한다. 창 제목과 터미널에 범례가 나온다. XY가 같은 물체 윗면을 지면으로 칠하지 않도록 각 꼭짓점/점의 높이를 국소 지면 기준과 대조하고 셀 내부 경사도 반영한다. 초록은 지면 분류이며 통과 가능 여부는 별도의 cost 판정이다. 기본 지면 높이 허용 차는 지면 높이군 폭의 절반 또는 voxel2개 중 큰 값(현재2cm)이고, 주황은 확인된 가까운 지면보다 단차 기준(현재6cm) 이상 높은 표면이다. 근거가 없는 영역과 지면 아래 이상값은 회색으로 남긴다.

지형의 `grid_from_world`와 3D의 `world_from_slam` 차이를 적용해 서로 다른 저장 좌표도 맞춘다. 최신 revision은 숫자로 비교하고, source/calibration/keyframe checksum 및 depth 처리 설정이 다르면 색을 생략하고 이유를 안내한다. 오래된 다른 map의 결과로 대체하지 않는다. 지형이 없거나 직접 지정한 임의 PLY는 기존 단색으로 표시하며, 구형 지형은 한 번 다시 생성해야 한다. `--no-terrain-colors`로 분류 색을 끌 수 있다. 색은 화면의 메모리에만 입히므로 원본 PLY와 지형 수치·주행 경로는 바뀌지 않는다.

서로 다른 지도는 독립 좌표계의 별도 창으로 표시한다. 합친 지도가 아니며 map ID는 제목에 표시한다. 기본 표시방향도 초기 camera 좌표 기준으로, 중력 정렬된 방향을 주장하지 않는다.

- 왼쪽 마우스 드래그: 회전
- 휠: 확대·축소
- Shift+왼쪽 드래그: 화면 이동
- 창 닫기: 해당 지도만 닫음

`tools/view_mapping.sh`는 `.venv-mapping`을 사용하며 DISPLAY가 있으면 X11로 실행해 이전 Wayland/GLEW 초기화 문제를 피한다. 원본/mesh를 수정하지 않는다.

## 검증과 남은 실기

2026-10-15 거리별 재처리: 관련89 tests passed(6.89초).0.7/1.7/2.7m 혼합 depth의 실제 TSDF가1/2/3m 설정에 맞는 표면만 남기는지, 지형 투영·필터의 동일 제한, 기본 결과 보존, 캐시 재사용, 입력 변경 시 새 revision과 이전 pose 사본 보존, 잘못된 거리·빈 depth 실패, 창 없는 CLI, 버전별 지면 색상 출처를 검증했다. 실제 `20261015_152413_6fea4a`도 세 버전으로 생성했다. 점군은1m109,125 /2m445,564 /3m925,672개였고, 각각252 keyframe을 융합했다. 세 버전의364점 카메라 경로가 기본 결과와 같고 기본 메시·점군 SHA-256이 그대로인 것을 확인했다. [거리별 비교 그림](../../outputs/handheld_mapping/reviews/20261015_152413_6fea4a_review/depth_limits_comparison.png), [검증 근거](../evidence/20261015_depth_variants_validation.json). 점 수나 unknown/차단 비중의 차이는 범위 축소의 영향도 포함하므로 정확도 향상률로 해석하지 않는다.

2026-10-15 지면 색상 표시: 관련57 tests passed(4.54초). 같은 XY의 지면/위쪽 물체, 경사 셀 내부 높이, 저장 좌표 차이, 입력 checksum 불일치, backend/map 구분, revision10/2 순서, mesh/cloud 색과 PLY 보존, 기본 표시·단색 옵션을 검증했다. 기존 실제 기록의 지형을 `map_0_v3`로 생성했고 cost 셀 수는 v2와 같았다. 메시 꼭짓점948,467개 중 지면204,915 / 장애물425,579 / 미분류317,973개이며 원본 메시·점군 SHA-256이 그대로인 것을 확인했다. 이는 표시 분류의 집계이며 실제 지형 정답과의 정확도 평가는 아니다. [색상 미리보기](../../outputs/handheld_mapping/reviews/20261015_152413_6fea4a_review/ground_colors_3d.png), [검증 근거](../evidence/20261015_surface_colors_validation.json).

추가 경사 색상 회귀:10/15/25/35도 각각의 평지→오르막→높은 평지와 경사 위20cm 물체를 합성해 GroundGrid부터3D 색칠까지 검사했다. 지면 표본의 주황 판정0, 초록 비율95% 초과, 물체 주황 비율100%였고 색상 모듈15 tests passed(1.23초).25/35도 지면도3D에서 초록으로 유지되면서 cost에서는20도 기준에 따라 차단됐다. 이는 연속된 충분한 관측의 합성 시험이며, depth 결손·경사 진입부의 가림·잘못된 pose 또는 실제 카메라의 오분류율을 검증하지는 않았다. 관측이 끊겨 지면 연결성이 사라진 곳은 미분류나 물체로 오분류될 가능성이 남는다.

[검증 기록](../evidence/20261014_overlap_validation/README.md). 실제 저장 영상20초를 약15FPS로 재생해 raw/Qt/native ORB/overlay/최종 TSDF를 함께 확인했다. 현재 Pi·카메라·무선 전송을 포함한 실기 전체 부하와 정확도는 별도 확인해야 한다. IMU 융합/중력 정렬은 이번 변경에 포함되지 않는다.

2026-10-14 입력 공백 계측: 새 앱 프로세스부터 `frames.jsonl`에 Pi/노트북 단계 metadata가 추가되고 `session.json`에 `capture_metrics`·`laptop_capture`를 저장한다. Pi 코드는 촬영 시작 때 RAM disk에 배포되고 원본은 계속 노트북에 저장한다. 종료 후 최종 큐/counter를 반영하며, 서로 다른 장치 monotonic을 빼서 지연으로 해석하지 않는다. [계측 해석과 다음 촬영 절차](2026-10-14-input-gap-investigation.md).
