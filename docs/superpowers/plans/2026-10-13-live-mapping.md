# Live Mapping Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:executing-plans로 현재 세션에서 직접 실행한다.

**Goal:** 촬영 중 native ORB pose로 bounded 관측 범위를 표시하고 원본/최종 TSDF 흐름을 보존한다.
**Architecture:** raw writer → bounded exact pair mailbox → stdin native ORB → pose/depth 관측 격자 → Qt. 최종 전체 raw 후처리는 기존 경로다.
**Tech Stack:** native ORB-SLAM3/C++14, Python3.10 .venv-mapping, OpenCV, numpy, PyQt5. Pi native Jazzy3.12.
**Spec:** docs/superpowers/specs/2026-10-13-live-mapping-design.md

## Global Constraints
원본은 노트북만 저장. 모터/SPI 변경 없음. 고정20m×20m/5cm,8stamp pairing/2쌍 mailbox/32depth metadata. 중력 미정렬 X-Z 참고 투영. live 실패가 capture를 중단하지 않음. 기존 offline/native 비교 보존. Git 작업 없음.

## Review Focus
- 같은 위치 정지 프레임은 coverage 횟수를 늘리지 않음: Task1.
- 카메라·depth 축 변환과 unknown/out-of-bounds: Task1.
- 다른 map의 경로/coverage를 섞지 않음: Task1/2.
- native startup 지연·pipe/프로세스 실패·종료가 raw 수집을 막지 않음: Task2/3.
- 오래된 pose 및 live drop을 센서 drop 또는 실제 안전/정확도로 해석하지 않음: Task3.

### Task1: 관측 격자와 pairing
Files: experiments/handheld_mapping/coverage.py, tests/handheld_mapping/test_live.py.
Interfaces: CoverageGrid.update(map_id, pose, depth, info), snapshot(); StereoMailbox.offer(header), take(timeout), depth_for(stamp).
- [x] 테스트 먼저 작성: 정지 후 count1,10cm 이동 후count2, map변경reset, 회전depth-to-left, 범위 밖/invaliddepth, 좌우 exact 매칭과 bounded drop.
- [x] PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_live.py -q: 미구현 import 실패 확인.
- [x] numpy 고정 격자·셀별 마지막시점과 deque8/32+queue2 구현.
- [x] 같은 명령으로 모든 Task1 테스트 통과.

### Task2: native stdin와 실시간 worker
Files: orbline/runner.cpp, live.py, offline.py, test_live.py.
Interfaces: runner argv4 '-'이면 stdin, live_poses.csv flush; LiveMapper(run).offer(header), finish(), snapshot().
- [x] fake native subprocess로 EOF 전 result, 느린 처리 시 mailboxdrop과 빠른offer, native실패와 자식 cleanup 테스트 작성 후 RED.
- [x] runner 입력을 istream으로 선택하고 매frame TrackStereo 반환의 world pose를CSV 출력. 기존 keyframe export 유지.
- [x] live worker가 한쌍 write → pose CSV1행 wait →2ms depthmatch →gridupdate, capture는metadata offer만 수행.
- [x] native build, 실제 D435 저장기록에서100쌍 stdin replay; stdin닫기 전 pose 파일 확인. fake tests GREEN.

### Task3: session와 Qt 통합
Files: controller.py, app.py, live_preview.py, demo.py, tests/handheld_mapping/test_live_session.py, docs/experiments/laptop-terrain-preparation.md.
Interfaces: Session.live_snapshot(), config live_mapping; Qt CoverageView.set_snapshot().
- [x] 합성 knownpose에서 촬영 중 live snapshot 존재, 종료 후 기존TSDF/원본 보존, live 실패에도 capture완료, offscreen widgetrender 테스트 RED.
- [x] calibration 이후 live시작, 이미지 저장 후offer, 종료/finally에서cleanup. demo pose는 합성임을 표시하고 native측정으로 주장하지 않음.
- [x] Qt에서 IR/coverage 나란히, checkbox defaultready, stale/error/map/lag/drop 표시.
- [x] 전체 handheld/headless suite + compileall/pyflakes/bash-n.
- [x] 실제 D435 기록 재생으로 UI preview와 live 처리/수신후 지연 기록, 기존 idle매핑 창만 교체. 실제 Pi가 없으면 실기 전체 부하 미검증을 명시.

## 검토와 완료
현재 디렉터리는 Git이 없으므로 plan 옆 progress.md에 RED/GREEN/판정 근거를 남긴다. 마지막에 fresh reviewer로 주요 실패 조건을 검토하고 중요한 문제는 RED→GREEN으로 수정한다. 실시간 색은 촬영 안내이며 물리적인 주행 costmap과 구분해 문서화한다.
