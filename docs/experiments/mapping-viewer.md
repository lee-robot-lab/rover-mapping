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

서로 다른 지도는 독립 좌표계의 별도 창으로 표시한다. 합친 지도가 아니며 map ID는 제목에 표시한다. 기본 표시방향도 초기 camera 좌표 기준으로, 중력 정렬된 방향을 주장하지 않는다.

- 왼쪽 마우스 드래그: 회전
- 휠: 확대·축소
- Shift+왼쪽 드래그: 화면 이동
- 창 닫기: 해당 지도만 닫음

`tools/view_mapping.sh`는 `.venv-mapping`을 사용하며 DISPLAY가 있으면 X11로 실행해 이전 Wayland/GLEW 초기화 문제를 피한다. 원본/mesh를 수정하지 않는다.

## 검증과 남은 실기

[검증 기록](../evidence/20261014_overlap_validation/README.md). 실제 저장 영상20초를 약15FPS로 재생해 raw/Qt/native ORB/overlay/최종 TSDF를 함께 확인했다. 현재 Pi·카메라·무선 전송을 포함한 실기 전체 부하와 정확도는 별도 확인해야 한다. IMU 융합/중력 정렬은 이번 변경에 포함되지 않는다.

2026-10-14 입력 공백 계측: 새 앱 프로세스부터 `frames.jsonl`에 Pi/노트북 단계 metadata가 추가되고 `session.json`에 `capture_metrics`·`laptop_capture`를 저장한다. Pi 코드는 촬영 시작 때 RAM disk에 배포되고 원본은 계속 노트북에 저장한다. 종료 후 최종 큐/counter를 반영하며, 서로 다른 장치 monotonic을 빼서 지연으로 해석하지 않는다. [계측 해석과 다음 촬영 절차](2026-10-14-input-gap-investigation.md).
