# 서버 대회 임무·costmap·실시간 관찰 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 서버의 기존 경기장·로버로 대표점 임무를 물리 구동하고 costmap·주행 로그를 저장하며 사용자가 브라우저로 진행을 관찰한다.

**Architecture:** 노트북 mapping 환경에서 metric 지도와 검증된 `SiteMission` bundle을 준비한다. 서버 Isaac에서 grid-only `SiteMission`과 기존 `MissionRuntime`을 읽고 simulator pose feedback을 swerve joint 명령으로 연결한다. 기록과 loopback HTTP 관찰 화면을 같은 run에 두고 SSH tunnel로 연결한다.

**Tech Stack:** 노트북 `.venv-mapping` Python 3.10/Open3D/NumPy/SciPy/pytest; 서버 기존 Isaac Sim 6.1.0 이미지 Python 3.12.13/NumPy/SciPy/Pillow/Matplotlib, stdlib HTTP, OpenSSH, Docker. 2026-10-17 CPU container probe에서 서버 Open3D/pytest 부재를 확인했다. 서버에 추가 패키지를 설치하지 않는다.

**Spec:** [승인한 설계](../specs/2026-10-17-server-competition-mission-design.md)

## Global Constraints

- 코드 기준: `/home/su/rover-mapping`, `codex/site-mission`. 원본 `/home/su/rover`와 서버 기존 코드/결과는 보존한다.
- GPU 1만 사용한다. 시작 직전 GPU 사용량을 다시 확인하고 기존 container·학습·`latest` 링크를 변경하지 않는다.
- 기존 `nvcr.io/nvidia/isaac-sim:6.1.0` 이미지와 `/home/sumee/gyusa/arena_sand_v15_m5.usd`를 사용한다. 변형 모래 모델로 표시하지 않는다.
- 서버 실행은 `/home/sumee/gyusa/runs/site_mission/<unique_run>/`에 격리한다. 입력 자산은 read-only로 mount한다.
- 기존 STL scale `0.1`, sand NPZ `(x,y)`를 명시적으로 meter·`(row=y,col=x)`에 변환한다. 경로, 지도와 sim world의 좌표 계약을 저장한다.
- 물리 dt 0.005초, 제어/telemetry 목표 0.05초. 화면 갱신 목표 wall 0.2~0.5초. 실제 갱신률을 기록한다.
- 정답 pose는 `sim_ground_truth`로 명시한다. 임무 중 pose teleport, 실기 모터 송신, MCU/firmware/CAN/Pi 변경은 하지 않는다.
- 자연 주행과 `fault_injection` 실행을 분리한다. 30°의 성공을 실패로 바꾸지 않는다.
- unknown=255, blocked=254, risk=0..252를 보존한다. 시험 경사 예외는 벽·단차·unknown을 허용하지 않는다.
- 새 Python 파일은 한국어 파일 상단 설명·사용법, 클래스 역할 docstring과 타입 힌트를 포함한다.

## Review Focus

1. 서버 Open3D 부재에서도 grid-only 로딩이 같은 hash·geometry 안전 검사를 유지하는가 — Task 1의 import 차단 subprocess와 asset 변조 검사.
2. 기존 NPZ의 XY 축과 높이 offset 때문에 잘못된 경로가 통과하지 않는가 — Task 1의 비대칭 높이 fixture와 실제 anchor 경로 검사.
3. 안착 실패·추적 거부·예외에서 stale wheel target이 남지 않는가 — Task 2의 fake robot/clock과 예외 주입 검사.
4. 자연 등판 성공·주입 정체·timeout을 같은 완료로 기록하지 않는가 — Task 2/4의 서로 다른 실행과 completion 대조.
5. 관찰자가 끊기거나 파일이 갱신 중일 때 화면이 오래된 성공을 현재 진행처럼 보이지 않는가 — Task 3의 HTTP polling·stale 표시·atomic publish 검사.

## 파일 구조

| 파일 | 역할 |
|---|---|
| `experiments/handheld_mapping/site_map.py`, `site_mission.py` | 기존 full mesh 기본 동작 유지, grid-only load 추가 |
| `experiments/site_sim/__init__.py` | 서버 임무 package |
| `experiments/site_sim/prepare.py` | 현장 자산→meter mesh·세 경로·검증 bundle |
| `experiments/site_sim/drive.py` | simulator pose→runtime→joint command, 정지와 주입 정체 |
| `experiments/site_sim/recording.py` | costmap·경험 층·telemetry·event·snapshot·completion |
| `experiments/site_sim/viewer.py`, `viewer.html` | 파일 기반 read-only HTTP 상태·영상·지도와 브라우저 표시 |
| `experiments/site_sim/run.py` | Isaac scene·robot·camera·물리 loop와 finalization |
| `tools/run_site_sim_server.sh` | 준비 bundle·코드 upload, GPU 1 launch, status/tunnel/stop |
| `tests/site_sim/test_prepare.py`, `test_drive.py`, `test_recording.py`, `test_viewer.py` | CPU/fake 동작 검증 |
| `docs/experiments/server-site-mission.md` | 실행·결과·한계·종료 절차 |

기존 `swerve.py`, `drive_runtime.py`, `isaac_scene.py`와 그 장착 변환 의존 코드는 원본에서 필요한 파일만 `experiments/orbslam3_ros2/`, `experiments/orbslam3_offline/`에 가져온다. 기존 파일과 이름이 겹치면 먼저 diff하고 관련 부분만 적용한다. 원본 경로와 SHA256을 실행 문서에 기록한다. 테스트/문서와 무관한 기존 실험 package 전체를 복사하지 않는다.

## Task 1: 서버에서 읽을 지도와 실제 경기장 경로

**Files:** `site_map.py`, `site_mission.py`, `experiments/site_sim/{__init__,prepare}.py`, `tests/site_sim/test_prepare.py`, `tests/handheld_mapping/test_site_map.py`.

**Interfaces:**
- `SiteMap.load(folder: Path, *, load_mesh: bool=True) -> SiteMap`; `SiteMission.load(folder: Path, *, load_mesh: bool=True) -> SiteMission`.
- `load_mesh=False`이면 `mesh=None`. metadata와 grid/mesh 파일 hash 검사는 생략하지 않는다. mesh 없이 save하면 명확한 ValueError를 반환한다.
- `metric_arena_mesh(stl: Path, sand: Path) -> open3d.geometry.TriangleMesh`.
- `route_through(site: SiteMap, anchors: list[list[float]], max_slope_deg: float) -> np.ndarray`.
- `prepare(stl: Path, sand: Path, output: Path) -> SiteMission`; CLI는 `python -m experiments.site_sim.prepare --stl PATH --sand PATH --output NEW_FOLDER`.

- [ ] **RED: grid-only 로딩과 축 변환 검사를 먼저 추가한다.** 기존 `tests/handheld_mapping/site_mission_helpers.py`의 fixture로 full/grid-only bundle을 각각 읽고 같은 `validate()` 결과와 tampered `mesh.ply` 거부를 확인한다. 새로운 subprocess에서 Open3D import를 차단한다.

```python
script = '''
import builtins
original = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] == 'open3d':
        raise ImportError('Open3D intentionally unavailable')
    return original(name, *args, **kwargs)
builtins.__import__ = guarded
from pathlib import Path
from experiments.handheld_mapping.site_mission import SiteMission
m = SiteMission.load(Path(__import__('sys').argv[1]), load_mesh=False)
assert m.site.mesh is None and m.validate()['accepted']
'''
```

`height[x,y] = 0.1*x + 0.03*y`인 비대칭 fixture로 `metric_arena_mesh`의 X/Y와 cell center를 확인한다. NaN 칸과 단차를 가로질러 면을 생성하지 않는지도 검사한다. Run: `.venv-mapping/bin/python -m pytest tests/site_sim/test_prepare.py tests/handheld_mapping/test_site_map.py -q`; 새 API 부재로 FAIL을 확인한다.

- [ ] **구현: 무거운 import를 지도 생성 경로로 한정한다.** `site_map.py`의 Open3D와 `reference.manual_gravity` import를 `from_mesh`, `_rasterize`, `save`, `load_stl`, full `load`에서 필요한 곳으로 이동한다. `load()`의 hash 검사는 그대로 유지하고 다음 분기만 추가한다. `SiteMission.load()`는 keyword를 전달한다.

```python
self.mesh = None
if load_mesh:
    import open3d as o3d
    self.mesh = o3d.io.read_triangle_mesh(str(folder / 'mesh.ply'))
```

- [ ] **구현: metric 입력과 경로 bundle을 준비한다.** STL 사본의 vertices를 `0.1`배하고 `sand['height'].T`를 row/column 순서로 변환한다. NPZ의 `x0,y0`는 grid 모서리, surface vertices는 `(i+.5)*cell`이다. sand는 2.5cm 이하 간격으로 샘플링하고 유한한 인접 vertices/단차 제한 안에서만 삼각형을 만든다. 칸막이 형상은 유지한다. metric mesh와 원본 두 파일의 hash·scale·축·`T_site_sim_world=I`를 저장한다.

```python
cells = site.cells(np.asarray(anchors, float))
safe = site.safe_mask(max_slope_deg)
parts = []
for a, b in zip(cells[:-1], cells[1:]):
    path = astar(safe, tuple(a[::-1]), tuple(b[::-1]))
    if path is None:
        raise ValueError('route anchor is unreachable')
    parts.append(site.origin + (np.asarray(path)[:, ::-1] + .5) * site.resolution)
```

`astar`는 기존 `coverage_path.astar`를 쓴다. 정확한 anchors를 양 끝에 넣고 중복을 제거한 후, 긴 선분 shortcut은 `site.validate_path()`가 통과한 것만 수용한다. 각 route는 같은 junction/summit을 사용하고 30° 첫 내부점은 trial 진입점이다. 후보 위치는 basin 출발 `(3.65,-0.3)`, junction `(3.65,0.35)`, summit `(5.35,0.4)`, trial anchor `(4.5,0.4)`, 15° 상부 anchor `(4.7,1.25)`, 10° 하부 anchor `(4.7,-1.25)`로 시작한다. 실제 safe grid가 거부하면 명시적으로 후보를 조정·기록하며, 여유 반경이나 장애물 검사를 줄여 통과시키지 않는다. 로버 collision bounds에 근거한 원형 반경·추종 tolerance·여유를 먼저 정하고 경로를 검사한다. 상행 시간은 경로 길이/속도를 근거로 설정하며 timeout은 항상 유한하다. `site_name='demo'`, pose source는 `sim_ground_truth`다.

- [ ] **GREEN·실제 자료 준비·commit.** 새 검사와 기존 site/mission/runtime/replay 검사를 실행한다. local mapping Python으로 실제 `gyusa_fence40.stl`과 `sand_height_v15_m5.npz`를 새 `outputs/site_sim/<label>/mission`에 준비한다. 세 경로의 좌표·길이·최대 경사·clearance와 preview를 검토한다. `git diff --check` 후 Task 1 파일을 commit한다.

## Task 2: 물리 명령과 costmap·경험 기록

**Files:** `experiments/site_sim/{drive,recording}.py`, `tests/site_sim/{test_drive,test_recording}.py`, 필요한 기존 swerve/scene 모듈.

**Interfaces:**
- `SimulationDrive(robot, session, mission, *, fault_injection: bool=False)`; `step(sim_s: float) -> dict`; `stop() -> None`. session은 기존 `DriveSession`이며 `controller`, `_action_type`, `steering_indices`, `wheel_indices`, `geometry`를 사용한다.
- `RunRecorder(mission, output: Path, *, max_snapshots: int=1200)`; `observe(row: dict, events: list[dict]) -> None`; `snapshot(stamp_s: float, reason: str) -> Path`; `finalize(status: str, reason: str) -> dict`.
- row: `stamp_s,wall_s,position_m,quaternion_wxyz,joint_positions,joint_velocities,requested_command,applied_command,mission,pose_source,fault_injection`. 최대 sim time 900초·wall time 3600초로 제한한다.

- [ ] **RED: fake joint 구동·정지 및 costmap sentinel을 검사한다.** fake robot의 실제 pose만 변화시키고 runtime 출력이 swerve target으로 전달되는지 확인한다. invalid pose, session apply 예외, 종료에서 wheel 4개의 zero target을 확인한다. 자연 mode는 어떤 route에서도 주입을 하지 않고, 주입 mode는 30° 상행 trial 구간에서만 전진 target을 제한하며 retreat 상태에서는 원래 명령을 적용한다.

```python
normal = SimulationDrive(robot, session, mission, fault_injection=False)
row = normal.step(0.05)
assert row['applied_command'] == row['requested_command']
normal.stop()
assert np.array_equal(session.last_wheel_targets, np.zeros(4))
```

`RunRecorder`에는 observed flat/slope/blocked/unknown fixture를 넣어 normal/trial cost가 경사에서만 달라지고 unknown/벽이 동일한지 검사한다. 방문·정체 이벤트는 해당 칸에만 누적하며 중복 이벤트를 이중 기록하지 않는다. snapshot 수 상한과 최종 snapshot 확보, EOF/예외 시 실패 summary도 검사한다. Run: `.venv-mapping/bin/python -m pytest tests/site_sim/test_drive.py tests/site_sim/test_recording.py -q`; 새 모듈 부재 FAIL을 확인한다.

- [ ] **구현: runtime 출력을 joint에 적용한다.** simulator time으로 stamp/now를 같이 넣고 `frame='site'`, 저장 identity, `pose_epoch=0`, `tracking_valid=True`, 실제 world pose와 yaw를 전달한다. 반송 값의 requested/applied를 분리한다. joint 명령은 다음 기존 계산을 재사용한다.

```python
modules = body_twist_to_modules(session.geometry, vx=command['vx_m_s'], vy=0.,
                                yaw_rate=command['yaw_rate_rad_s'])
session.controller.apply_action(session._action_type(
    joint_positions=np.asarray([m.steering_angle_rad for m in modules]),
    joint_indices=session.steering_indices))
session.controller.apply_action(session._action_type(
    joint_velocities=np.asarray([m.wheel_angular_velocity_rad_s for m in modules]),
    joint_indices=session.wheel_indices))
```

주입은 30° 상행의 정해진 위치 이후 wheel 전진 target을 제한하는 explicit fault다. 조건·시각·적용값을 event에 기록하고 runtime의 pose를 조작하지 않는다. 주입 종료는 retreat 전환으로 한다. 기체가 그 조건에서 후퇴하지 못하면 성공으로 조작하지 않고 실제 실패를 기록한다.

- [ ] **구현: recorder를 추가한다.** `SiteMap`에서 `build_costmap` 입력 층을 만들고 normal/trial 두 배열을 저장한다. footprint clearance는 `site.safe_mask()`와 일치시키고 normal/trial blocked에는 각각 동일한 제한을 적용한다. 경험 층은 `visit_count`, `last_seen_s`, `stuck_count`다. 방문하지 않은 칸의 `last_seen_s`는 NaN이며 JSON에서는 null로 바꾼다. 주행 event cursor로 incremental JSONL을 쓴다. 1 sim 초와 상태 전환, 시작/종료에 compressed NPZ를 저장하고, 상한에 도달하면 주기 snapshot만 생략하고 종료 snapshot 공간을 예약한다.

```python
np.savez_compressed(temporary, stamp_s=stamp_s, origin=site.origin,
    resolution=site.resolution, normal_cost=normal, trial_cost=trial,
    visit_count=visits, last_seen_s=last_seen, stuck_count=stuck)
temporary.replace(destination)
```

telemetry는 line마다 유한값을 검증하고 flush 주기를 제한한다. `summary.json`과 `completion.json`은 atomic write하고 status는 `complete/failed/timeout/interrupted`를 구분한다. 원본 source/config/code/image/frame/physics manifest를 저장한다. slip/sinkage는 unavailable로 둔다.

- [ ] **GREEN·commit.** Task 1/2 검사와 기존 임무 regression을 실행한다. fake step의 requested/applied와 실제 joint 배열, snapshot 좌표를 직접 대조하고 diff check 후 commit한다.

## Task 3: 파일 기반 실시간 관찰 화면과 Isaac 실행기

**Files:** `experiments/site_sim/{viewer,run}.py`, `viewer.html`, `tests/site_sim/test_viewer.py`, `tests/site_sim/test_drive.py`.

**Interfaces:**
- `serve(output: Path, port: int) -> ThreadingHTTPServer`; routes `/`, `/state`, `/map`, `/frame/overview.jpg`, `/frame/front.jpg`만 제공한다.
- `run.py` CLI: `--mission PATH --urdf PATH --geometry PATH --scene-usd PATH --output NEW_PATH --max-sim-s 900 --max-wall-s 3600 [--fault-injection]`.
- live files: `live/state.json`, `live/map.json`, `live/overview.jpg`, `live/front.jpg`; publisher는 temp→replace, HTTP reader는 완성본만 읽는다.

- [ ] **RED: HTTP·stale·부분 publish를 검사한다.** localhost 임의 port로 viewer를 띄우고 urllib로 실제 HTTP를 요청한다. state/이미지 미준비는 503, 잘못된 path는 404이며 부모 경로 접근은 허용하지 않는다. `updated_wall_s`가 오래된 state는 stale로 표시하고 최종 completion은 완료 상태로 표시한다. 파일 교체 중에도 잘린 JSON이 200으로 반환되지 않는지 검사한다. Run: `.venv-mapping/bin/python -m pytest tests/site_sim/test_viewer.py -q`; 새 모듈 부재 FAIL을 확인한다.

- [ ] **구현: viewer를 물리 thread와 분리한다.** HTTP는 파일만 읽고 Isaac API를 호출하지 않는다. browser는 `fetch('/state',{cache:'no-store'})`를 약 0.3초 간격으로 실행하고 실패/age를 명시한다. canvas에서 meter origin·resolution을 사용해 normal/trial map·경로·실제 궤적·현재 heading·대표점을 그린다. image는 최신 overview/front JPEG를 요청한다. legend와 pose source·fault flag를 항상 표시한다. script/CDN 등 외부 자원은 사용하지 않는다.

```python
class Handler(BaseHTTPRequestHandler):
    """현재 run의 허용된 관찰 파일만 읽어서 응답한다."""
    def do_GET(self):
        relative = {'/state':'live/state.json', '/map':'live/map.json',
            '/frame/overview.jpg':'live/overview.jpg',
            '/frame/front.jpg':'live/front.jpg'}.get(self.path.split('?')[0])
        # 루트 HTML과 위 경로 외에는 404를 반환한다.
```

- [ ] **구현: 기존 scene과 camera를 연결한다.** 서버 설치본의 공식 camera 예제/API를 확인한 뒤 기존 `build_isaac_scene`와 `DriveSession`을 초기화한다. 새 overview camera와 기존 전방 camera의 render product를 만든다. meter 임무 출발 XY와 surface 높이를 사용해 로버를 생성하고 수초간 zero target으로 안착시킨다. finite pose·출발 tolerance·낙하 여부를 확인한 뒤 runtime을 시작한다. world step은 0.005초, drive/record는 0.05초, 렌더링은 낮은 wall 주기로 처리한다. 카메라 failure는 상태에 기록하며 주행 실패와 구분한다. 전체 camera 불가 상태를 실시간 영상 성공으로 주장하지 않는다.

```python
try:
    while app.is_running() and sim_s < args.max_sim_s and wall_s < args.max_wall_s:
        if sim_s >= next_control_s:
            row = drive.step(sim_s)
            recorder.observe(row, drive.runtime.events)
            next_control_s += .05
        world.step(render=render_due)
finally:
    drive.stop()
    recorder.finalize(status, reason)
    app.close()
```

최종화는 초기화 단계에 따라 존재하는 자원만 처리하며 원래 예외를 감추지 않는다. SIGTERM/KeyboardInterrupt는 interrupted로 남긴다. terminal 상태/timeout/낙하/비정상 pose 이후 기록과 zero target을 확인한다. viewer는 별도 CPU 프로세스로 띄워 Isaac 종료 후에도 최종 결과를 보여준다.

- [ ] **GREEN·commit.** HTTP/fake loop 검사를 실행하고 camera/scene import가 CPU unit test 수집에서 강제되지 않는지 확인한다. 완성된 HTML을 browser에서 열어 지도 축·범례·stale 상태를 확인한다. diff check 후 commit한다.

## Task 4: 서버 upload·실행·실시간 관찰·검증 기록

**Files:** `tools/run_site_sim_server.sh`, `docs/experiments/server-site-mission.md`, `README.md`, 필요한 wrapper 검사 `tests/site_sim/test_server_wrapper.py`.

**Interfaces:**
- `bash tools/run_site_sim_server.sh upload LABEL MISSION_FOLDER`.
- `bash tools/run_site_sim_server.sh start LABEL [--fault-injection] [--max-sim-s N] [--max-wall-s N]`.
- `bash tools/run_site_sim_server.sh status LABEL` / `tunnel LABEL LOCAL_PORT` / `stop LABEL`.
- `start`는 명명된 run이 없거나 이미 실행 중이면 명확히 거부한다. label은 `[A-Za-z0-9_-]+`만 허용한다.

- [ ] **RED: wrapper의 경계 검사를 추가한다.** 잘못된 label, 존재하지 않는 bundle, GPU 1 메모리 부족, 중복 run에서 새 container를 시작하지 않는지 fake ssh/docker 응답으로 확인한다. `bash -n`과 fake 원격 명령의 shell quoting을 검사한다. Run: `.venv-mapping/bin/python -m pytest tests/site_sim/test_server_wrapper.py -q`; wrapper 부재 FAIL을 확인한다.

- [ ] **구현: 전용 run과 loopback observer를 실행한다.** upload는 필요한 package·검증 bundle·portable URDF·geometry·mesh 자산만 전용 디렉터리에 담고 hash를 비교한다. 기존 arena는 `/arena:ro`로 mount한다. 서버 viewer는 host Python3 stdlib로 `127.0.0.1:<remote_port>`에 bind한다. simulator는 GPU 1만 사용하고 현재 GPU 사용량·image digest·정확한 launch argv를 기록한다. 설정/label은 제한된 값만 받으며 임의 문자열을 원격 command에 삽입하지 않는다.

```bash
ssh -o BatchMode=yes -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 \
  -N -L 127.0.0.1:8765:127.0.0.1:8765 hpi-lab
```

기본 포트가 사용 중이면 다른 명시 port를 고르고 기록한다. stop은 해당 label의 container·viewer·tunnel만 종료한다. 결과 폴더를 삭제하지 않는다. 서버 시작·기록 경로·상태와 로컬 URL을 출력한다.

- [ ] **CPU 검증과 GPU smoke.** `tests/site_sim` 및 기존 전체 regression, `bash -n`, `git diff --check`를 실행한다. 실제 준비 bundle로 GPU 1에서 10~20 sim 초 smoke를 수행해 안착, joint 운동, front/overview 이미지, telemetry, costmap snapshot을 확인한다. 로그·GPU memory·sim/wall 시간은 실제 값으로 기록한다. 실패 시 systematic-debugging을 적용하고 수정한 범위를 재검증한다.

- [ ] **전체 자연 주행과 별도 정체 주입 주행.** natural과 fault를 별도 label/run으로 실행한다. runtime event·최종 pose·joint 운동·계획 대비 궤적·경사 구간 cost sentinel·종료 zero request를 검사한다. 자연30°가 성공한 경우 그 사실을 보존하며 별도 fault run에서 후퇴/우회를 확인한다. 실제로 실패한 실행은 원인과 부분 결과를 남기고 timeout으로 끝난 것을 complete로 바꾸지 않는다.

- [ ] **사용자 화면 확인과 문서·최종 검토.** browser skill을 읽고 SSH tunnel URL을 Codex browser pane에 연다. 실제 갱신되는 서버 화면의 pose/time/image를 확인한다. 사용자가 보고 있는 동안 run이 종료되어도 최종 costmap·궤적을 유지하고 다시 실행하는 명령을 문서화한다. 실행 명령, 입력 hash, pose source, natural/fault 구분, 완료/실패 근거, 로그 경로·종료 절차·고정 모래 한계를 기록한다. Native 실행은 전체 branch의 독립 reviewer 한 명에게 검토를 맡기고 중요 문제를 수정·재검증한 뒤 최종 commit한다. push는 현재 요청의 자동 후속으로 가정하지 않고 로컬 commit과 실제 결과를 먼저 제공한다.

## 계획 자체 검토

- 승인 spec의 좌표·자산·물리 feedback은 Task 1/2, 기록·실패 구분은 Task 2, 영상·지도 화면은 Task 3, 서버 격리·사용자 관찰·종료는 Task 4에 대응한다.
- 다섯 Review Focus는 각 owning task의 RED 검사와 실제 서버 인수 단계에 배치했다.
- 서버에서 Open3D 설치를 피하는 grid-only 계약은 기존 full mesh load/save 기본값과 hash 검사를 유지한다.
- 실제 자산 경로와 후보 경로는 실행 전에 검증한다. 안전 반경/경사 제한을 결과에 맞춰 임의로 낮추지 않는다.
- 권장 실행 방식은 Native다. 네 task가 지도→runtime→기록·관찰→서버 실행 순서로 의존하며 같은 GPU/runtime을 다루므로 한 구현자가 순차 진행하고 마지막에 독립 검토하는 편이 적절하다.
