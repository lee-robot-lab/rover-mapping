# TELEOP 이동 궤적 구현

범위: 수동 이동을 RUN과 분리해 세그먼트·거리로 저장하고 지도/overview에 노란 점선으로 표시. 초기 바구니 구간은 run0 수동이며 화면 정렬만 수행. AUTO 시작/종료, ANCHOR, 송신ID 변경, 0.5초 수신 공백에서 선을 끊는다. 기존 AUTO/TELEOP 예측과 명령 경로는 유지한다.

- [x] 실제 UDP 경계·좌표단절·거리·저장 TDD RED 확인 및 GREEN.
- [x] Session 기록·서버 mode전환·RUN1과초기transfer분리.
- [x] 메인/overview 점선·거리·TELEOP 지형 누적과 저장.
- [x] 독립리뷰의 transfer지형누적표시 회귀 수정·회귀시험.
- [x] CLI/README 및 실제 리허설 audit 기록, Git반영.

명령·센서·실기실행은 변경하지 않음. 사용자현재시연프로세스는종료하지않음.
