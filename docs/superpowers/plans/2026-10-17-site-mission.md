# STL Representative Mission Implementation Plan

> **For agentic workers:** Use superpowers:executing-plans to implement this plan task-by-task. User approved the written spec and requested implementation in this session.

**Goal:** 팀원 STL에서 세 대체 경로·대표점을 설정하고 등판 시도/후퇴/우회/정상/복귀를 모터 없는 실행기로 준비한다.
**Architecture:** STL→bounded metric grid→저장 임무/방향별 route→timestamped pose 기반 상태기계·추종기. Qt 편집기와 replay는 같은 core를 사용한다.
**Tech Stack:** Python3.10 mapping venv, numpy/scipy/Open3D, PyQt5/Matplotlib, pytest.
**Spec:** docs/superpowers/specs/2026-10-17-site-mapping-route-preparation-design.md

## Global Constraints

- 코드 기준 /home/su/rover-mapping. 원본 STL·촬영·기존 결과는 덮어쓰지 않는다.
- SI/meter, source SHA256/transform/frame/grid origin을 보존하고 미정렬/stale/lost에서는 정지한다.
- 매핑·STL export/firmware/실기 모터 송신은 범위 밖. 합성 입력과 실기 기록을 구분한다.
- 30° 시험은 해당 route에만 적용하고 unknown/장애물/footprint gate는 유지한다.
- 사용자 코드 상단 한글 설명·usage, 클래스 역할, 타입 힌트 규칙을 따른다.

## Review Focus

- winding이 뒤집힌 STL·mm 단위·hole/multilayer가 free로 바뀌지 않음: Task1.
- 경유점 사이가 장애물을 가로지르거나 경계 밖일 때 저장/재로드에서도 거부: Task2.
- 오래된/중복/역행 pose·다른 frame·재개 첫 표본에 거짓 도착/stuck 없음: Task3.
- 30° 부분 등판 후 후퇴가 아직 가보지 않은 정상까지 연결되지 않음: Task3.
- 실사용 창에 시작/정상/경로 입력·저장/재로드가 모두 연결됨: Task4.

### Task 1: STL site grid
**Files:** Create experiments/handheld_mapping/site_map.py; tests/handheld_mapping/test_site_map.py.
**Interfaces:** SiteMap.from_mesh(mesh, source, units, up, resolution, robot_radius); load_stl(path,...); site.height/slope/observed/blocked/origin/resolution; site.validate_path(xy, max_slope_deg); site.save/load(new_folder).
- [ ] RED: 합성 평면/경사, mm/up, hole·수직벽·다층, footprint, invalid/큰 mesh/grid, hash·저장 reload.
- [ ] Run focused tests; expected missing module failure.
- [ ] Implement vertical raycasting in bounded chunks, conservative unknown/multilayer/wall/step masks, hash and atomic metadata/NPZ.
- [ ] Run focused tests; expected all pass. Commit Task1 only.

### Task 2: Mission model and route validation
**Files:** Create site_mission.py; test_site_mission.py.
**Interfaces:** SiteMission(site,start,junction,summit,routes,settings); validate(); save(output); load(output). routes keys 10/15/30 with junction→summit XY polyline; start→junction connection; directed export.
- [ ] RED: actual paths survive roundtrip; mandatory common endpoints; finite config; normal routes slope gate; test30 slope exception only; crossing obstacles/unknown; source tamper; output overlap refusal.
- [ ] Run focused tests; expected missing module failure.
- [ ] Implement dense path sampling/headings, footprint checks, A* connector, directed route bundles and settings validation.
- [ ] Run Task1+2 tests; expected all pass. Commit Task2.

### Task 3: Runtime, feedback and replay
**Files:** Create mission_runtime.py, mission_replay.py; test_mission_runtime.py, test_mission_replay.py.
**Interfaces:** MissionRuntime(mission).update(sample, now_s)→status/command/event dict. replay(mission,output,scenario)→summary. JSONL CLI consumes explicit aligned site pose.
- [ ] RED: common summit, success, 30 failure retreat recorded path only, 15 failure fallback10, all fail, descent unavailable, lost/stale/nonmonotonic/frame mismatch stop, resume no false progress, speed bounded and arrival only valid pose.
- [ ] Implement state machine, reverse driven breadcrumbs, bounded planar follower, persistent progress timeout with max attempt budget and explicit failure events.
- [ ] Implement fake kinematic plant and injected stalls for demo, append event/command CSV and preview; JSONL runner logs stdout/output and zero at EOF.
- [ ] Run Tasks1–3 tests, expected all pass. Commit Task3.

### Task 4: Editor, wrappers and complete validation
**Files:** Create mission_editor.py, tools/plan_site_mission.sh, tools/run_site_mission.sh; tests/handheld_mapping/test_mission_editor.py; docs/experiments/site-mission.md.
**Interfaces:** MissionEditor(site,mission=None), loading STL with explicit unit/up, UI click modes for start-heading/junction/summit/route10/15/30; save/load/demo/route validation buttons.
- [ ] RED: real offscreen canvas clicks populate state, Undo, save/reload, invalid route error, CLI demo and replay produce usable output.
- [ ] Implement Qt editor and Matplotlib 3D/XY views without expanding old mapping UI; wrappers resolve shared venv when checkout has no venv.
- [ ] Run offscreen tests plus bare pytest with KSRC_REVIEW_ROOT; expected all pass/known native skips.
- [ ] Run real saved mesh if present plus demo/replay scenario matrix, shell syntax and diff checks.
- [ ] Update operating docs and ledger with commands/counts/limitations. Commit Task4.
- [ ] Fresh-context independent whole-branch review, fix real issues with regression tests, rerun whole suite after final changes.
