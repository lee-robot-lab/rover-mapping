이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# 카메라 상하 비교·저장 지도 viewer 검증 — 2026-10-14

## 검증 자료

- 새 코드: `experiments/handheld_mapping/overlap.py`, `viewer.py`, `tools/view_mapping.sh`; 기존 live/app 확장. raw/카메라/native ORB/모터 code는 변경하지 않는다.
- `ui_replay_validation.json`: 실제 D435 저장 기록20초의 첫 replay. overlay 매frame 렌더링은 약42ms 추가되고10초 이후 live skip5쌍, pose lag 중앙값~99ms여서 그대로 채택하지 않았다.
- `optimized/ui_replay_validation.json`: overlay만 Qt5Hz 주기에 맞춘 후 같은 약15FPS replay. live input skip10초시54/종료54(추가0), pose수신lag UI82samples 중앙값20.52ms/p9525.60ms, overlay 계산표본 중앙값41.72ms/p9548.22ms. 영상/ORB 입력을5FPS로 낮춘 것이 아니다. sensor/network/display를 포함한 end-to-end latency도 아니다.
- `optimized/ui_recorded_during_capture.png`: 원본위/overlay아래/전체지도오른쪽, 같은 프레임·관측색을 직접 확인했다.
- `optimized/ui_recorded_after_save.png`: 마지막 matching frame 유지와 최종 저장 흐름. source는 recorded_real_D435_replay다.
- `viewer.log`: default latest-session 선택으로 실제 원본촬영 `20261013_204443_766235`의 Map0/421 두창 VIEWER_READY. native X11 창 생성 확인, 임의 map 병합 없음.
- `pytest_final.log`: handheld/headless 전체 회귀의 최종 결과. 별도 ML/전 repo 테스트 결과로 확대하지 않는다.

## 계약

Depth .25–3m만 실제 depth-to-left와 intrinsics로 재투영, 가까운 점만 z-buffer로 색칠한다. 원본 IR은변경하지 않는다. initial X-Z coverage의 노랑 적음/초록3시점 이상 참고값으로, 높이가 다른 표면과 실제3D재구성 품질/안전성을 완전히 구분하는 지표는 아니다. tracking/depth/stale/error는 색을 제거하며 새 session에 이전 pixmap을 남기지 않는다. 종료 프레임은 정지화면으로 명시한다.

관측 overlay 한쌍만 보관하고 native/원본 큐 계약은 유지한다. overlay 실패는 별도오류로 기록한다. source calibration은보존하고 최종fullraw ORB/TSDF를 생성한다. 실제 Pi 전송·로버 주행 부하와 IMU 보정은 이번 검증 대상이 아니다.

## 테스트/리뷰

Projection intrinsics/extrinsic·occlusion·유효범위·원본불변4tests RED→GREEN. Matching original/overlay stamp + verticalUI RED→GREEN. 새session pixmapclear/stalefallback RED→GREEN. Overlay5Hz throttle/새mapforcedfresh RED→GREEN. Viewer latest/explicit/backend/cloud/map/pathescape3tests RED→GREEN. 잘못된 summary5종에 UI callback으로 KeyError/TypeError가 나가지 않도록 RED→GREEN 검증했다.

fresh read-only overlap_review는 Critical/Important 없음·진행 가능 판정, 신규3개test파일16passed. 손상된summary스키마의 예외 전파를 Minor로 지적했다. GUI callback으로 전파되는 실제 예외라 해당문제를 수정하고5개regression을 추가했다. 정상파일선택/GL렌더링은 기존과같고 전체suite를 다시실행했다.
