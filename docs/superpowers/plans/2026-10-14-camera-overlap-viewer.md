# Camera Overlap and Map Viewer Implementation Plan

> **For agentic workers:** superpowers:executing-plans로 현재 세션에서 직접 실행한다.

**Goal:** 원본/관측 오버레이를 위아래로 함께 보여주고 저장된 실제 3D 지도를 재사용 가능한 명령과 버튼으로 연다.
**Architecture:** 기존 live worker에서 같은 stamp의 IR/depth/pose와 fixed coverage를 투영해 RGB overlay를 만든다. UI는 위 원본·아래 overlay·오른쪽 coverage. 별도 Open3D viewer 프로세스가 저장된 map별 PLY를 연다.
**Tech Stack:** Python3.10 .venv-mapping, numpy/OpenCV, PyQt5, Open3D0.19. 기존 native ORB 그대로.
**Spec:** 직전 카메라 위 반투명 관측 색 제안과 사용자의 “온오프 보다는 위아래 배치로 둘다” 승인. 기존 docs/superpowers/specs/2026-10-13-live-mapping-design.md의 raw 보존/지도분리/중력미정렬 계약을 유지한다.

## Global Constraints
- native ORB·Pi·모터 제어 변경 없음. raw grayscale 데이터는 수정하지 않는다.
- overlay는 같은 시각 IR/pose와2ms depth 매칭만 사용. depth→left extrinsic과 양쪽 intrinsics를 적용하고 z-buffer로 가까운 표면만 색칠한다. .25–3m 밖/depth없음/추적손실/stale/error는 색칠하지 않는다.
- 노랑 적은 시점/초록3회 이상 참고값. X-Z coverage라 높이가 다른 표면을 같은 셀로 다룰 수 있으며 정밀3D품질/costmap이 아님을 유지한다.
- 최신 원본/overlay 한쌍만 메모리에 유지. UI raw/overlay를 같은 processed frame으로 표시하며 stale시 최신 raw와 무색 영상으로 복귀한다. 원본 저장/capture를 기다리게 하지 않는다.
- 기본 viewer는 가장 최근 촬영 폴더를 선택한다. 최신 결과가 아직 없으면 이전 지도를 몰래 표시하지 않고 안내한다. map별 독립 창, 절대 임의 병합 금지. explicit RUN/PLY, map-id, cloud 선택 지원.
- 이 workspace는 Git 저장소가 아니다. Git 관련 작업은 수행하지 않는다.

## Review Focus
- depth/IR 광학 중심 이동·회전과 occlusion, invalid depth: Task1 geometric tests.
- 원본 mutation 및 다른 frame/color 잔류: Task1/2 tests.
- summary 경로·없는 map·아직 생성 중인 최신 촬영: Task3 selection tests.
- overlay failure가 capture/live map을 중단하지 않음: Task2 tests.
- 다중 map 창 종료와 Wayland GLFW 초기화: Task3 actual GUI smoke.

### Task1: 오버레이 투영
Files: experiments/handheld_mapping/overlap.py, tests/handheld_mapping/test_overlap.py.
Interface: overlap_image(original,depth,depth_info,left_info,world_left,grid)->RGB ndarray.
- [x] 알려진 pinhole+extrinsic에 한 점, invalid/behind/outside, zbuffer near/far 충돌, 원본 불변 테스트 작성 후 RED.
- [x] numpy projection·coverage lookup·nearest depth·alpha blending 구현. GREEN.

### Task2: live/UI 통합
Files: live.py, app.py, tests/handheld_mapping/test_live_session.py.
- [x] 합성 촬영 중 overlay/original/stamp 일치·raw 보존, 위아래 widget 위치 테스트 RED.
- [x] native/knownpose에서 matching IR+depth로 overlay를 별도 캐시. 예외는 overlay_error로만 기록. snapshot JSON에서 ndarray 제외.
- [x] 위아래 영상 배치, tracking stale/error면 최신raw로 무색표시. Qt 실제 저장영상 replay screenshot 검사, 기존회귀 GREEN.

### Task3: 저장된 맵 viewer
Files: experiments/handheld_mapping/viewer.py, tools/view_mapping.sh, app.py, tests/handheld_mapping/test_viewer.py.
Interface: select_maps(target,backend,map_id,cloud)->list[(label,Path)]. CLI default latest sessions, no auto fallback to older session.
- [x] backend·map filtering·missing latest·path escape·directPLY 테스트 RED.
- [x] summary 기반 map 선택, standalone Open3D 여러창·원본보존·X11 launcher, GUI “3D 지도 보기” 버튼. GREEN.
- [x] 실제 최신 PLY 열기/정상 window 생성 확인. 전체handheld/headless tests·pyflakes/compileall/bash-n·fresh reviewer. 사용법 docs.
