# Direct return and VO preparation implementation plan

**Goal:** 표면·서버 실험과 독립적으로 불필요한 복귀 왕복을 제거하고 VO 병행 평가 연결 범위를 확인한다.

**Architecture:** 10° 하산 경로에 출발점이 정확한 waypoint로 있으면 그 이후 갈림길 왕복을 생략한다. 그렇지 않은 임무는 기존 복귀 동작을 유지한다. VO는 별도 추정·평가 경로로 준비하며 이번 변경에서 제어 pose를 바꾸지 않는다.

**Tech Stack:** 기존 `.venv-mapping` Python 3.10 CPU 테스트, 기존 ORB-SLAM3 ROS 2 소스 검토.

- [x] `SiteMission.descent_path()`와 runtime의 실제 하산·완료 검사를 회귀 테스트로 확인한다.
- [x] 기존 전체 하산·복귀 fixture는 동작을 유지하는지 검사한다.
- [x] VO 입력/출력과 GT 분리, 시간·좌표 정합, tracking loss 처리의 미구현 연결을 기록한다.

서버 업로드·실행, 표면 파일, 동시 작업 중인 site_sim 파일은 수정하지 않는다.
