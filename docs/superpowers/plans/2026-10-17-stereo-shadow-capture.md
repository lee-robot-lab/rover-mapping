# Stereo shadow capture implementation plan

**Goal:** 네 표면 조건의 저장 전방 영상 추적을 비교하고 GT 제어와 분리된 stereo 평가용 기록을 만든다.
**Architecture:** CPU OpenCV로 인접 영상 ORB seed/LK 왕복 추적을 평가한다. 선택적 simulation-clock stereo 기록은 같은 render step에서 좌우 PNG, intrinsics, extrinsics, GT를 저장한다. 평가 데이터만 생성하며 제어 pose는 바꾸지 않는다.
**Tech Stack:** `.venv-mapping` Python 3.10/OpenCV, 서버 Isaac 6.1 Python 3.12, 기존 USD/Replicator.

- [x] 네 run의 전체 front_frames를 보존하고 pairwise 추적/시간 간격을 JSON·CSV로 비교.
- [x] 동일 timestamp 좌우 PNG·GT·카메라 보정을 기록하는 CPU writer와 테스트.
- [x] 실행기에 opt-in `--stereo-hz`와 simulation-clock scheduling 연결; 왼쪽과 오른쪽 카메라 설정 일치.
- [x] GPU0 유휴 확인 후 새 run/별도 observer port에서 10초·15Hz 기록; 동기·frame 수·보정·영상 확인.
- [x] stereo VO 추정 실행 여부와 데이터 준비만 끝난 부분을 구분해 보고.

추가 정합 확인 필요: 표본 ORB 좌우 매칭 수 2~21, 세로차 중앙값 최대 3.66px. VO pose는 미실행. 상세 한계는 docs/experiments/2026-10-17-stereo-capture-results.md 참고.
