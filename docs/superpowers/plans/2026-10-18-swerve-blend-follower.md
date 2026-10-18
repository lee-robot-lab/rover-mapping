# Swerve yaw+vy 혼합 추종기(blend) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `MissionRuntime`에 `(vx, vy, yaw_rate)`를 내는 `blend` 추종 모드를 추가하고, 시뮬(IK·RL 경로)까지 `vy`를 전달해 기존 `unicycle` 모드와 시뮬에서 비교한다.

**Architecture:** 순수 함수형 상태 객체 `BlendFollower`(새 파일)가 spec의 혼합 법칙(ψ_ref 슬루, 몸 좌표 변환, 병진 방향 슬루, 램프 게이트)을 계산하고, `MissionRuntime.update()`가 `settings['follower_blend']>0`일 때만 이를 호출한다. 기본 모드의 출력은 비트 단위로 같게 유지한다. 시뮬 구동(`drive.py`)은 `applied.get('vy_m_s',0.)`로 vy를 읽고, 별도 지표 도구가 텔레메트리에서 수용 조건 수치를 뽑는다.

**Tech Stack:** Python 3.10 (`.venv-mapping`), numpy, pytest. 서버 검증은 Isaac Sim 도커(`tools/run_site_sim_server.sh`).

**Spec:** `docs/superpowers/specs/2026-10-18-swerve-blend-follower-design.md`

## Global Constraints

- 범위는 시뮬 검증까지다. 실기(Pi) 반영·실기 명령 프로토콜 변경·경로 생성 교체·벽 거리 사용·Stanley 이득/ORB/ESKF 설정 변경은 하지 않는다.
- 기본 설정(`follower_blend=0`)에서 기존 실행 결과가 달라지면 실패다. unicycle 명령 dict는 키(`vx_m_s`, `yaw_rate_rad_s`)와 값이 그대로여야 하고 `vy_m_s`·`follower` 키가 생기면 안 된다.
- 제어 주기 0.02 s, 최대 속도 0.08 m/s(임무 번들 설정), RL 정책 `res8_mlp_v1_rc4000` 고정. RL은 `lin_vel_y` ±0.20 m/s로 학습되었다.
- 초기 설정값: `blend_max_yaw_rate_rad_s` 0.09, `blend_ramp_slope_deg` 8, `blend_ramp_resume_deg` 15, `blend_ramp_stop_deg` 25, `blend_dir_rate_rad_s` 1.0(서보 한계 `CMD_DIR_RATE=1.5`보다 낮게), 방향 유지 속력 하한 0.01 m/s, yaw 이득 2.
- 명령 dict의 `vy_m_s`는 없으면 0으로 취급한다(`mission_replay.py`, 기록 재생, 뷰어, 옛 기록이 깨지면 안 됨).
- 시뮬 ORB가 회전에 강해도 "ORB가 좋아졌다"고 결론내지 않는다. 이 검증은 "각속도 상한 아래서 임무를 완주할 수 있는가"만 답한다.
- 코딩 규칙(사용자 CLAUDE.md): 새 코드 파일 첫 블록은 `# ====...` / `# <파일명>` / `# 설명:` / `# 사용법:` / `# ====...` 한국어 형식, 클래스 바로 아래 한 줄 한국어 docstring, 타입 힌트, 요청 범위 밖 리팩터링 금지, 가장 단순한 구현.
- 테스트는 관련 파일만 돌린다(`tests/handheld_mapping` 전체는 GUI 창을 연다). 명령: `QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest <files> -q` (`tests/site_sim`은 `KSRC_ROOT=outputs/site_sim/ksrc_20261016`도 앞에 붙인다). 작업 디렉터리는 `/home/su/rover-mapping`.
- 모든 커밋 메시지 끝에 `Co-Authored-By: Claude Sonnet 5.5 <noreply@anthropic.com>` 줄을 넣는다. push하지 않는다.
- 서버(GPU) 단계는 사용자에게 설정을 알리고 go를 받은 뒤에만 실행한다(Task 7).

## Review Focus

각 줄은 "입력/조건 → 기대 동작 (검증하는 Task)" 순서다. 가능성이 큰 것부터다.

1. 속력 ≈ 0(게이트 정지, 경로 끝, 일시정지)에서 병진 방향이 뒤집히면 서보가 헛돈다 → 속력 0.01 m/s 미만이면 직전 몸 좌표 방향 유지 (Task 2).
2. yaw가 ±π를 넘나들거나 추정기 yaw가 한 번에 튈 때 → 짧은 쪽으로 회전, `yaw_rate`는 항상 ±ω_max 안, 게이트 정지 중에도 유한 (Task 2).
3. 경로에 길이 0 구간·중복점이 있거나 당근 점이 현재 위치와 같을 때(`atan2(0,0)`) → 방향을 0으로 튀기지 않고 현재 yaw를 목표 방향으로 사용 (Task 3).
4. stuck 판정이 `abs(command['vx_m_s'])`만 보면, 몸이 옆으로 병진(vx≈0, vy≠0)할 때 진행 실패를 못 센다 → `hypot(vx,vy)` 사용, 게이트 정지 중에는 effort가 쌓이지 않음 (Task 3).
5. 일시정지·watchdog 출력 → blend에서만 `vy_m_s=0.`을 싣고 unicycle에서는 키 자체가 없음 (Task 3).
6. 일시정지에서 재개되거나 단계가 바뀔 때 ψ_ref·방향 상태가 낡은 값으로 남으면 → 정지 시 `invalidate()`로 재초기화(ψ_ref=현재 yaw), 단계 전환 시 방향 슬루 한계 유지 (Task 2·3).
7. 텔레메트리 소비자: `recording.py`는 `row['requested_command']['vx_m_s']`만 읽는다 → vy가 있어도 깨지지 않고, `follower`가 없는 옛 기록도 지표 도구가 처리 (Task 4·6).
8. 램프에서 게이트로 멈춘 동안 추정기 pose가 튀는 경우 → `pose_jump` 규칙은 기존대로 일시정지, 게이트는 yaw만 돌며 멈춘 채 유한 출력 (Task 3).
9. 경사 지도 밖·NaN 위치 → 게이트 비활성(경사 0 취급), 예외 없음 (Task 3).
10. 낮은 속도 + RL 추종 지연으로 방향 슬루가 목표보다 뒤처진 채 벽 쪽으로 쓸리는 경우 → 단위 테스트로는 못 잡으므로 Task 7의 최소 벽 거리 수용 조건으로만 확인한다.

## 파일 구조

| 파일 | 역할 |
|---|---|
| `experiments/handheld_mapping/blend_follower.py` (신규) | 혼합 법칙 순수 계산. 지도·미션 의존 없음 |
| `experiments/handheld_mapping/site_mission.py` | 새 설정 기본값·검증 |
| `experiments/handheld_mapping/mission_runtime.py` | blend 분기, 슬로프 조회, 영(零) 명령, stuck 판정 |
| `experiments/handheld_mapping/mission_replay.py` | 비물리 plant에 vy 적분 |
| `experiments/site_sim/drive.py` | IK·RL 경로에 vy 전달 |
| `experiments/site_sim/run.py`, `server.py`, `viewer.py` | `--follower`, `--blend-max-yaw-rate` 옵션 |
| `experiments/site_sim/follower_metrics.py` (신규) | 텔레메트리에서 수용 조건 지표 계산 + 비교표 CLI |
| `docs/experiments/2026-10-18-swerve-blend-follower.md` (신규) | 검증 결과 기록 |

---

### Task 0: 보류 중인 실험 옵션 커밋

**Files:**
- Commit only: `experiments/site_sim/run.py`, `experiments/site_sim/server.py`, `tests/site_sim/test_realism_options.py` (이미 수정·테스트됨)

**Interfaces:** 없음 (이후 Task가 이 커밋 위에서 `run.py`/`server.py`를 수정한다).

- [ ] **Step 1: 테스트가 통과하는지 확인**

Run: `KSRC_ROOT=outputs/site_sim/ksrc_20261016 QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/site_sim/test_realism_options.py -q`
Expected: 전부 PASS

- [ ] **Step 2: 정확히 이 세 파일만 커밋**

```bash
git status --short   # 세 파일 외에 M/?? 가 있으면 건드리지 말고 그대로 둔다
git add experiments/site_sim/run.py experiments/site_sim/server.py tests/site_sim/test_realism_options.py
git commit -m "sim: add --visual-turn-scale/--filter-acc-noise experiment options"
```
Expected: `git show --stat HEAD`가 위 세 파일만 보인다.

---

### Task 1: 새 설정과 검증

**Files:**
- Modify: `experiments/handheld_mapping/site_mission.py:18-22` (DEFAULT_SETTINGS), `:55-59` (검증)
- Test: `tests/handheld_mapping/test_site_mission.py`

**Interfaces:**
- Produces: `DEFAULT_SETTINGS`에 키 추가 — `follower_blend: 0.`, `blend_heading_hold: 0.`, `blend_max_yaw_rate_rad_s: .09`, `blend_ramp_slope_deg: 8.`, `blend_ramp_resume_deg: 15.`, `blend_ramp_stop_deg: 25.`, `blend_dir_rate_rad_s: 1.`. 값은 모두 int/float(문자열 금지). `SiteMission.settings`로 읽는다.

- [ ] **Step 1: 실패하는 테스트 작성** (`mission_fixture()` 사용, 기존 테스트 스타일)
  - `test_blend_settings_default_to_off_and_spec_values`: `m.settings['follower_blend']==0`, `blend_heading_hold==0`, `blend_max_yaw_rate_rad_s==.09`, `blend_ramp_slope_deg==8`, `blend_ramp_resume_deg==15`, `blend_ramp_stop_deg==25`, `blend_dir_rate_rad_s==1.0`.
  - `test_blend_settings_accept_one`: `SiteMission(..., settings={'follower_blend':1,'blend_heading_hold':1})` 생성 성공(fixture의 인자를 재사용).
  - `test_blend_settings_reject_invalid` (parametrize, 모두 `ValueError('invalid mission settings')`): `follower_blend=2`, `blend_heading_hold=.5`, `blend_max_yaw_rate_rad_s=.7`(> `max_yaw_rate_rad_s` .6), `blend_ramp_resume_deg=25`(resume < stop 위반), `blend_ramp_resume_deg=30`, `blend_dir_rate_rad_s=0`, `blend_max_yaw_rate_rad_s=0`.
  - `test_old_mission_json_without_blend_keys_loads`: `mission_fixture()`을 저장(`save`)한 `mission.json`에서 위 7개 키를 지운 뒤 `SiteMission.load(..., load_mesh=False)` 성공, 7개 키는 기본값. (저장/불러오기 API는 기존 테스트의 사용 방식을 따른다.)

- [ ] **Step 2: 실패 확인**

Run: `QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_site_mission.py -q`
Expected: 새 테스트만 FAIL (KeyError / ValueError 안 나옴)

- [ ] **Step 3: 구현**
  - `DEFAULT_SETTINGS['stanley_gain']=0.` 줄 아래에 새 7개 키를 같은 방식으로 추가한다.
  - 검증의 `v<(0 if k=='stanley_gain' else 1e-12)`를 0 허용 키 집합(`stanley_gain`, `follower_blend`, `blend_heading_hold`)을 쓰도록 바꾼다.
  - 추가 조건(기존 `or` 체인에 이어서): `follower_blend`와 `blend_heading_hold`는 0 또는 1, `blend_max_yaw_rate_rad_s<=max_yaw_rate_rad_s`, `blend_ramp_resume_deg<blend_ramp_stop_deg<90`.

- [ ] **Step 4: 통과 확인** — 같은 명령, Expected: 전부 PASS

- [ ] **Step 5: Commit**

```bash
git add experiments/handheld_mapping/site_mission.py tests/handheld_mapping/test_site_mission.py
git commit -m "mission: add blend follower settings with validation"
```

---

### Task 2: 순수 혼합 법칙 모듈 `blend_follower.py`

**Files:**
- Create: `experiments/handheld_mapping/blend_follower.py` (한국어 헤더 블록 필수)
- Test: `tests/handheld_mapping/test_blend_follower.py`

**Interfaces:**
- Consumes: 없음 (numpy, math, dataclasses만)
- Produces:
  - `@dataclass(frozen=True) class BlendParams`: `max_yaw_rate: float=.09`, `yaw_gain: float=2.`, `ramp_slope_deg: float=8.`, `resume_deg: float=15.`, `stop_deg: float=25.`, `dir_rate: float=1.`, `min_dir_speed: float=.01`, `heading_hold: bool=False`.
  - `@dataclass(frozen=True) class BlendOutput`: `vx: float`, `vy: float`, `yaw_rate: float`, `psi_ref: float`, `heading_gap_rad: float`(= `|wrap(psi_ref−yaw)|`, 텔레메트리·수용 조건의 제외 구간용), `travel_gap_rad: float`(= `|wrap(travel_heading−yaw)|`, 게이트 판정값), `gated: bool`.
  - `class BlendFollower`: `__init__(self, params: BlendParams=BlendParams())`, `invalidate(self) -> None`(다음 `step`에서 ψ_ref=현재 yaw, 몸 좌표 방향=목표 방향으로 재초기화), `step(self, *, yaw: float, travel_heading: float, speed: float, slope_deg: float, dt: float, retreat: bool=False) -> BlendOutput`. `speed`는 호출자가 코너 감속·안전 배율을 곱하기 전/후 어느 쪽이든 비음수 스칼라이며 `cos(error)` 게이트는 적용하지 않는다.
  - 법칙(spec 내가 정한 것 2·3·4·5): (a) 재초기화 직후가 아니면 `psi_ref += clip(wrap(target−psi_ref), ±max_yaw_rate·dt)`, target은 `travel_heading`(`heading_hold`면 갱신 안 함, `retreat`면 `psi_ref=yaw`로 고정); (b) `yaw_rate=clip(yaw_gain·wrap(psi_ref−yaw), ±max_yaw_rate)`; (c) 게이트: `slope_deg>=ramp_slope_deg`일 때만, 정지 상태에서는 `travel_gap>stop_deg`면 정지하고 정지 상태에서는 `travel_gap<=resume_deg`일 때 해제, 경사가 낮으면 항상 해제, `retreat`에서는 게이트 없음; (d) 게이트면 유효 속력 0; (e) 몸 좌표 방향 `θ=wrap(travel_heading−yaw)`를 직전 적용 방향에서 `dir_rate·dt` 이하로 슬루, 유효 속력이 `min_dir_speed` 미만이면 직전 방향 유지; (f) `vx=speed_eff·cosθ_app`, `vy=speed_eff·sinθ_app`. `retreat`에서는 `yaw_rate=0`이고 `travel_heading`은 호출자가 후진 방향(몸 뒤쪽 포함) 그대로 넘긴다.

- [ ] **Step 1: 실패하는 테스트 작성** (`BlendFollower`를 직접 구동, 모두 `dt=.02`)
  - `test_defaults_match_spec`: `BlendParams()`가 위 기본값과 같다.
  - `test_psi_ref_slews_at_omega_max_and_yaw_rate_is_clipped`: 최초 `step(yaw=0, travel_heading=π/2, speed=.08, slope_deg=0, dt=.02)` 뒤 `psi_ref==0`(재초기화 직후), 이후 100회 yaw 고정 호출에서 매 스텝 `psi_ref` 증가량 `<=.09*.02+1e-12`, `abs(yaw_rate)<=.09+1e-12`, 마지막 `yaw_rate==pytest.approx(.09)`.
  - `test_body_velocity_is_world_direction_rotated_into_body`: 재초기화 직후 `yaw=π/2, travel_heading=0, speed=.08` → `vx≈0`, `vy≈-.08`, `hypot(vx,vy)==approx(.08)`.
  - `test_body_direction_slew_is_limited_to_1_rad_s`: yaw=0, travel_heading=0으로 시작해 travel_heading을 π/2로 바꾼 뒤 매 스텝 몸 좌표 방향 변화 `<=1.0*.02+1e-9`, 1.7 s(85 스텝) 안에 `atan2(vy,vx)≈π/2`.
  - `test_direction_is_held_below_min_speed`: 방향이 π/2로 정착한 뒤 `speed=.005`, travel_heading를 반대로 바꿔도 `atan2(vy,vx)`가 직전 값과 같고 `hypot==approx(.005)`.
  - `test_yaw_wrap_at_pi_takes_short_way`: `yaw=π-.01, travel_heading=-π+.01`에서 `heading_gap_rad`와 `travel_gap_rad`가 `approx(.02, abs=1e-9)`, 몇 스텝 후 `yaw_rate>0`이고 `abs(yaw_rate)<=.09`.
  - `test_estimator_yaw_jump_keeps_output_bounded`: 정상 주행 중 `yaw`를 한 스텝에 +90° 점프시켜도 모든 출력이 유한하고 `abs(yaw_rate)<=.09`, 방향 변화 `<=.02`.
  - `test_gate_has_hysteresis_on_ramp`: `slope_deg=10`. gap 20° → 이동(`gated False`, `hypot>0`), 26° → `gated True`이고 `vx==vy==0`이며 `yaw_rate`는 부호가 gap 쪽, 20°로 줄어도 계속 `gated True`, 14° → `gated False`. 같은 gap 60°라도 `slope_deg=5`면 `gated False`.
  - `test_gated_direction_is_held_not_zeroed`: 게이트 정지 해제 직후 첫 `vx,vy` 방향이 정지 전 방향과 `dir_rate*dt` 이내로 연속.
  - `test_retreat_keeps_psi_ref_at_yaw_and_drives_backward`: `yaw=.7, travel_heading=.7+π, retreat=True, slope_deg=15` → `yaw_rate==0`, `psi_ref==yaw`, `gated False`, `vx≈-speed`, `vy≈0`.
  - `test_heading_hold_does_not_chase_travel_direction`: `BlendParams(heading_hold=True)`, 재초기화 후 yaw=.3 고정, travel_heading=2.0으로 50회 → `psi_ref==.3`, `yaw_rate==0`.
  - `test_invalidate_reinitializes_to_current_yaw`: 주행 후 `invalidate()`하고 `yaw=1.0, travel_heading=-1.0` → `psi_ref==1.0`, 방향이 슬루 없이 `wrap(-1.0-1.0)`.

- [ ] **Step 2: 실패 확인**

Run: `QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_blend_follower.py -q`
Expected: ImportError / FAIL

- [ ] **Step 3: 구현** — 위 Interfaces의 법칙을 그대로 구현한다. 상태는 `psi_ref`, 직전 몸 좌표 방향 `theta_app`, `gated`, `initialized` 네 개뿐이다. 각도 wrap은 `math.atan2(sin,cos)` 한 가지로 통일(`mission_runtime.angle_wrap`과 같은 정의, 순환 import를 피하려 이 모듈 안에 복사).

- [ ] **Step 4: 통과 확인** — 같은 명령, Expected: 전부 PASS

- [ ] **Step 5: Commit**

```bash
git add experiments/handheld_mapping/blend_follower.py tests/handheld_mapping/test_blend_follower.py
git commit -m "mission: add pure blend follower (psi_ref slew, body vy, ramp gate)"
```

---

### Task 3: `MissionRuntime` 통합

**Files:**
- Modify: `experiments/handheld_mapping/mission_runtime.py` (`__init__` 약 37-48행, `_output` 약 56-70행, `_pause`, `update()`의 stuck 판정 약 142행과 끝부분 약 163-173행)
- Test: `tests/handheld_mapping/test_mission_runtime_blend.py` (신규, 헤더 블록 포함. `sample()` 헬퍼는 `test_mission_runtime.py`의 것을 복사)

**Interfaces:**
- Consumes: Task 1 설정 키, Task 2 `BlendFollower`, `BlendParams`, `BlendOutput`
- Produces:
  - blend 모드 결과 dict: `result['command']=={'vx_m_s','vy_m_s','yaw_rate_rad_s'}`, `result['follower']=={'psi_ref','heading_gap_rad','travel_gap_rad','gated'}`(float/bool, 유한). 활성 `update()`에서만 `follower`가 있고 일시정지·watchdog·phase_transition 결과에는 `follower`가 없다.
  - unicycle 결과는 현재와 동일(키·값).
  - 내부: `MissionRuntime._slope_deg(self, position: np.ndarray) -> float`(지도 밖·NaN이면 0.), `MissionRuntime._zero_command(self) -> dict`(blend면 `vy_m_s=0.` 포함), `self.follower: BlendFollower | None`.

- [ ] **Step 1: 기본 모드 골든 값 캡처 (구현 전, Task 1 커밋 상태에서)** — 테스트 파일에 `scripted_run(mission) -> list[dict]` 헬퍼를 작성한다: `mission_fixture()`로 `MissionRuntime`을 만들고 비물리 unicycle plant(`xy+=vx*dt*[cosyaw,sinyaw]`, `yaw+=yaw_rate*dt`, `dt=.1`)로 400회 `update`한 모든 결과 dict를 모은다. 임시 스크립트로 `hashlib.sha256(json.dumps(results, sort_keys=True).encode()).hexdigest()`를 구해 상수 `GOLDEN_UNICYCLE_SHA256`로 테스트에 박는다.

- [ ] **Step 2: 실패하는 테스트 작성**
  - `test_default_mode_output_is_bit_identical`: `sha256(scripted_run(...))==GOLDEN_UNICYCLE_SHA256`, 모든 결과의 `command` 키 집합이 `{'vx_m_s','yaw_rate_rad_s'}`이고 `'follower' not in result`.
  - `test_unicycle_pause_has_no_vy_key`: 일시정지 결과(`tracking_valid False`)와 `watchdog()` 결과의 `command`에 `vy_m_s`가 없다.
  - `test_blend_pause_and_watchdog_carry_zero_vy`: `follower_blend=1` 설정의 같은 상황에서 `command=={'vx_m_s':0.,'vy_m_s':0.,'yaw_rate_rad_s':0.}`.
  - `test_blend_corner_keeps_yaw_rate_under_limit_and_uses_vy`: 평지(`monkeypatch.setattr(run,'_slope_deg',lambda p:0.)`)에서 경로를 약 90° 꺾이는 `to_junction` 점열로 두고(`run._set_path`) plant 적분(vy 포함) 600회 → 모든 스텝 `abs(yaw_rate)<=.09+1e-12`, 어느 스텝에서든 `abs(vy)>.01`, 도착(`state`가 `trial_entry`로 전환) 성공.
  - `test_blend_retreating_reverses_without_rotating`: 첫 `update` 뒤 `run.state='retreating'; run.direction='down'; run._set_path([[x,y],[x-.5,y]])`(yaw 0, 경로가 몸 뒤쪽) → `yaw_rate==0`, `vx<0`, `abs(vy)<1e-6`, `follower['gated'] is False`.
  - `test_blend_summit_180_turn_is_slow_and_not_gated`: 평지, `state='descending'`, yaw 0, 경로 방위 π → 매 스텝 `abs(yaw_rate)<=.09`, `follower['gated'] is False`, 병진 속력>0.01; yaw를 plant로 적분해 `abs(wrap(yaw-π))<.1`이 되는 시각이 30~45 s.
  - `test_blend_ramp_gate_stops_then_resumes`: `_slope_deg`를 15로 패치, yaw 0·경로 방위 40°에서 `command` 병진이 0, `yaw_rate>0`, `follower['gated'] is True`; plant yaw 적분으로 gap이 15° 이하가 되면 `gated False`이고 병진>0; 그 사이 gap 20° 구간에서는 계속 `gated True`.
  - `test_blend_stuck_counts_sideways_motion`: 평지, 위치 고정, `yaw_rad=π/2`, 경로 방위 0(몸 좌표 vx≈0, vy<0) → 매 스텝 위치를 같은 값으로 넣어도 `run.effort>0`으로 증가(`stuck_window_s` 도달 시 `failed`/`ascent_failed` 전이까지 확인).
  - `test_blend_gate_stop_does_not_accumulate_effort`: 게이트 정지 구간 동안 `run.effort`가 증가하지 않는다.
  - `test_zero_length_and_duplicate_path_points`: 경로에 같은 점을 연속 3개 넣고 당근 점이 현재 위치와 같은 `update`에서 예외 없이 유한한 `command`, `follower['psi_ref']`가 현재 yaw에서 `.09*dt` 초과로 튀지 않음.
  - `test_blend_pose_jump_while_gated_still_pauses`: 게이트 중 `max_pose_jump_m` 초과 점프 → `reason=='pose_jump'`, `command` 영(零).
  - `test_slope_lookup_outside_map_is_zero`: 지도 밖 좌표와 NaN 경사 칸에서 `_slope_deg==0.`.
  - `test_safety_scale_applies_to_vx_vy_and_yaw_rate`: `safety_status={'scale':.5,...}` 설정 시 세 성분 모두 절반(단, `follower` dict는 배율 전 값).

- [ ] **Step 3: 실패 확인**

Run: `QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_mission_runtime_blend.py -q`
Expected: 골든 테스트는 PASS, 나머지 FAIL

- [ ] **Step 4: 구현** (`update()`의 기존 줄은 `if not blend:` 아래로 이동하거나 분기 앞에서 그대로 두고, unicycle 수식은 한 글자도 바꾸지 않는다)
  - `__init__`: `self.follower=BlendFollower(BlendParams(...settings 매핑...)) if settings['follower_blend']>0 else None`, `self.command=self._zero_command()`.
  - `_output`의 기본값 `{'vx_m_s':0.,'yaw_rate_rad_s':0.}`를 `self._zero_command()`로 교체.
  - `_pause`: `self.follower`가 있으면 `invalidate()`.
  - stuck 판정: `abs(self.command['vx_m_s'])`를 `math.hypot(self.command['vx_m_s'],self.command.get('vy_m_s',0.))`로 바꾼다(vy 키 없으면 값이 같음).
  - `update()` 끝: blend면 (a) `heading`은 기존처럼 계산하되 `reverse`일 때는 `+π`를 붙인 방향이 아니라 "당근 방위 그대로"를 `travel_heading`으로 넘기고 `retreat=reverse`; 후진은 `vx<0`이 되도록 BlendFollower가 몸 좌표 변환으로 만든다. (b) `|carrot−position|<1e-9`이면 `travel_heading=yaw`. (c) `speed=max_speed*carrot.speed_factor(...)`(cos 게이트 없음). (d) `out=self.follower.step(yaw=yaw,travel_heading=...,speed=speed,slope_deg=self._slope_deg(position),dt=dt,retreat=reverse)`. (e) `command={'vx_m_s':out.vx*scale,'vy_m_s':out.vy*scale,'yaw_rate_rad_s':out.yaw_rate*scale}`로 `_output`을 부른 뒤 `result['follower']`를 채워 반환. Stanley는 `not reverse`일 때만 기존대로 `travel_heading`을 대체한다.
  - `_slope_deg`: `site.cells(position[None])[0]`로 (col,row)를 얻고 `site.slope_deg[row,col]`, 범위 밖·NaN이면 0.

- [ ] **Step 5: 통과 확인** — 같은 명령 + `tests/handheld_mapping/test_mission_runtime.py tests/handheld_mapping/test_site_mission.py`, Expected: 전부 PASS (기존 테스트 변경 없이 통과)

- [ ] **Step 6: Commit**

```bash
git add experiments/handheld_mapping/mission_runtime.py tests/handheld_mapping/test_mission_runtime_blend.py
git commit -m "mission: add blend follower mode to MissionRuntime (default unchanged)"
```

---

### Task 4: 시뮬 구동·replay에 vy 전달

**Files:**
- Modify: `experiments/site_sim/drive.py` (약 127, 130행), `experiments/handheld_mapping/mission_replay.py` (약 60행 plant)
- Test: `tests/site_sim/test_blend_drive.py` (신규; `tests/site_sim/test_realism_options.py`처럼 `tests.site_sim.test_drive`의 Robot·Session fake 재사용), `tests/handheld_mapping/test_mission_replay.py` (추가)

**Interfaces:**
- Consumes: Task 3의 `result['command']`(`vy_m_s` 선택적)와 `result['follower']`
- Produces: `mission_replay.step_plant(xy: np.ndarray, yaw: float, cmd: dict, dt: float) -> tuple[np.ndarray, float]` — `xy += (vx·[cosψ, sinψ] + vy·[−sinψ, cosψ])·dt`, `vy=cmd.get('vy_m_s',0.)`, `yaw=angle_wrap(yaw+yaw_rate·dt)`. 텔레메트리 행은 이미 `'mission':result`를 담으므로 `row['mission']['follower']`가 자동으로 기록된다(별도 로깅 코드 없음).

- [ ] **Step 1: 실패하는 테스트 작성**
  - `test_drive_ik_path_passes_vy_to_modules` (monkeypatch `drive.body_twist_to_modules`로 인자 캡처): 가짜 runtime이 `command={'vx_m_s':.02,'vy_m_s':.05,'yaw_rate_rad_s':.03}`을 내면 `vy==.05`.
  - `test_drive_rl_path_passes_vy_to_policy` (가짜 `rl.step`): `rl.step`의 첫 인자가 `(.02,.05,.03)`.
  - `test_drive_without_vy_key_uses_zero`: `command`에 `vy_m_s`가 없으면 두 경로 모두 0.
  - `test_hold_and_fault_paths_apply_zero_vy`: `sim_s<hold_s`와 fault에서 vy 0.
  - `test_telemetry_row_carries_follower_and_commands`: 반환 행에 `row['mission']['follower']`, `row['requested_command']['vy_m_s']`가 있고 `recording.RunRecorder.observe`가 예외 없이 기록한다(`vy` 있음/없음 둘 다).
  - mission_replay: `test_step_plant_integrates_vy_in_body_frame`: yaw=π/2, `vx=0, vy=.1`, dt=1 → xy 변화 `≈(-.1, 0)`; `vy` 키 없는 cmd는 기존 결과와 같다. 기존 `replay()` 시나리오 테스트(`test_mission_replay.py`)가 그대로 통과한다.

- [ ] **Step 2: 실패 확인**

Run: `KSRC_ROOT=outputs/site_sim/ksrc_20261016 QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/site_sim/test_blend_drive.py tests/handheld_mapping/test_mission_replay.py -q`
Expected: 새 테스트 FAIL

- [ ] **Step 3: 구현** — `drive.py`의 두 하드코딩 `vy=0.`/`0.`을 `applied.get('vy_m_s',0.)`로 바꾼다(hold/fault dict는 그대로 두면 `get`이 0을 준다). `mission_replay.py`는 plant 두 줄을 `step_plant`로 추출해 `replay()`가 호출하게 한다(CSV 열은 `**cmd` 그대로라 unicycle 출력 불변).

- [ ] **Step 4: 통과 확인** — 같은 명령 + `tests/site_sim/test_drive.py tests/site_sim/test_realism_options.py`, Expected: 전부 PASS

- [ ] **Step 5: Commit**

```bash
git add experiments/site_sim/drive.py experiments/handheld_mapping/mission_replay.py tests/site_sim/test_blend_drive.py tests/handheld_mapping/test_mission_replay.py
git commit -m "sim: pass vy_m_s through drive (IK and RL) and replay plant"
```

---

### Task 5: CLI 옵션 `--follower`, `--blend-max-yaw-rate`

**Files:**
- Modify: `experiments/site_sim/run.py` (약 33행 stanley 패턴 옆, argparse 약 335-365행), `experiments/site_sim/server.py` (약 291행 flag 튜플, argparse 약 275-290행), `experiments/site_sim/viewer.py:18-19` (`OPTION_LABELS`)
- Test: `tests/site_sim/test_realism_options.py` (추가)

**Interfaces:**
- Consumes: Task 1 설정 키
- Produces: `run.py` 인자 `--follower {unicycle,blend}`(기본 `None`=unicycle, run_options.json에 기록되며 뷰어는 None을 표시하지 않음), `--blend-max-yaw-rate FLOAT`(기본 `None`). `run()`은 `args.follower=='blend'`이면 `mission.settings['follower_blend']=1.`, `args.blend_max_yaw_rate`가 있으면 `mission.settings['blend_max_yaw_rate_rad_s']=float(...)`(0 초과 `max_yaw_rate_rad_s` 이하가 아니면 `ValueError`). `server.py start`는 두 플래그를 `extra_args`로 그대로 전달(문자열도 `str()`로 같은 루프에서 처리되므로 별도 분기 불필요).

- [ ] **Step 1: 실패하는 테스트 작성**
  - `test_server_forwards_follower_flags`: `server.main(['start','lbl','--follower','blend','--blend-max-yaw-rate','0.09'])` → `seen['extra']==['--blend-max-yaw-rate','0.09','--follower','blend']`(extra 순서는 구현의 튜플 순서에 맞춰 테스트에서 고정), `--follower foo`는 argparse 에러(`SystemExit`).
  - `test_run_applies_follower_settings`: `run.py`의 설정 적용 부분을 작은 함수 `apply_follower_args(mission, args)`로 빼서 테스트한다(전체 `run()`은 Isaac이 필요). `follower='blend'` → `settings['follower_blend']==1.`, `blend_max_yaw_rate=.05` → `.05`; `.7`과 `0`은 `ValueError`; `follower=None`이면 설정이 변하지 않는다.
  - `test_viewer_option_text_shows_follower`: `run_options.json`에 `{'follower':'blend'}`이면 `run_options_text`가 `추종기 blend`를 포함하고 `{'follower':None}`이면 포함하지 않는다.

- [ ] **Step 2: 실패 확인**

Run: `KSRC_ROOT=outputs/site_sim/ksrc_20261016 QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/site_sim/test_realism_options.py -q`
Expected: 새 테스트 FAIL

- [ ] **Step 3: 구현** — `apply_follower_args(mission: SiteMission, args: argparse.Namespace) -> None`을 `run.py`에 추가하고 `run()`에서 stanley 줄 바로 아래에서 호출한다. argparse·서버 튜플·`OPTION_LABELS`(`('follower','추종기'),('blend_max_yaw_rate','blend ω')`)를 추가한다. 서버 쪽 `--follower`는 `choices=('unicycle','blend')`.

- [ ] **Step 4: 통과 확인** — 같은 명령, Expected: 전부 PASS

- [ ] **Step 5: Commit**

```bash
git add experiments/site_sim/run.py experiments/site_sim/server.py experiments/site_sim/viewer.py tests/site_sim/test_realism_options.py
git commit -m "sim: add --follower/--blend-max-yaw-rate options"
```

---

### Task 6: 지표 도구 `follower_metrics.py`

**Files:**
- Create: `experiments/site_sim/follower_metrics.py` (헤더 블록 필수. 사용법: `python -m experiments.site_sim.follower_metrics MISSION_FOLDER RUN_DIR [RUN_DIR ...]`)
- Test: `tests/site_sim/test_follower_metrics.py` (합성 텔레메트리, `tmp_path`)

**Interfaces:**
- Consumes: `telemetry.jsonl` 행의 `stamp_s`, `position_m`, `quaternion_wxyz`(GT), `estimated_position_m`, `estimator_status`, `estimator_quality`, `applied_command`, `mission{state,route,follower?}`; `clearance.evaluate(output, mission_folder)`(`completed`, `min_clearance_m`, `contact_samples`, `duration_sim_s`)
- Produces:
  - `summarize(output: Path, mission_folder: Path | None=None, *, omega_limit: float=.11, smooth_s: float=.5, gap_exclude_deg: float=60., moving_speed: float=.01) -> dict` — 키: `omega_mean_rad_s`, `omega_max_rad_s`(이동 중), `over_limit_fraction`(이동 시간 중 smoothed GT |ω| > `omega_limit`인 비율, `follower.heading_gap_rad`>`gap_exclude_deg`인 구간은 분자·분모에서 제외, `follower`가 없는 행은 제외하지 않음), `moving_s`, `excluded_s`, `rotation_after_start_deg`(첫 이동 행 이후 GT yaw의 `max|wrap(yaw−yaw0)|`), `max_estimate_error_m`(XY), `estimator_degraded_s`(`estimator_status in ('imu_gap','recovering')` 또는 `estimator_quality=='degraded'`인 시간), `gate_stop_s_by_phase: dict[str,float]`·`gate_resume_by_phase: dict[str,int]`(구간 = 연속한 같은 `(state,route)`, 키 `"state:route#k"`), `gate_stop_s_total`, 그리고 `mission_folder`가 있으면 `clearance.evaluate` 결과의 `completed`, `min_clearance_m`, `contact_samples`, `duration_sim_s`.
  - GT |ω|는 GT yaw(쿼터니언에서)를 unwrap한 뒤 차분하고 `smooth_s` 이동평균을 한다. 이동 중 = `hypot(applied vx, vy)>moving_speed`.
  - `format_table(summaries: dict[str, dict]) -> str`(run 이름 → 열 정렬된 비교표, 한국어 헤더 허용), `main(argv: list[str] | None=None) -> int`.

- [ ] **Step 1: 실패하는 테스트 작성** (헬퍼로 50 Hz 합성 행 생성)
  - `test_constant_rotation_gives_expected_omega`: GT yaw를 0.2 rad/s로 돌리며 이동 속도 .05 → `omega_mean_rad_s≈.2`, `over_limit_fraction==1.0`; 0.05 rad/s면 0.0.
  - `test_wrap_at_pi_does_not_create_spike`: yaw가 π를 지나가도 `omega_max_rad_s<.06`(0.05 rad/s 입력).
  - `test_heading_gap_over_60deg_is_excluded`: 중간 10 s 동안 `follower.heading_gap_rad=π/2`, 이 구간 ω .5 → `excluded_s≈10`, `over_limit_fraction==0`.
  - `test_old_rows_without_follower_or_vy`: `follower`·`vy_m_s` 키가 없는 행에서도 예외 없이 계산(이동 속도는 vx만), `gate_*`는 0/빈 dict.
  - `test_not_moving_rows_are_not_counted`: 정지 구간(속도 0) ω는 `over_limit_fraction`에 영향 없음.
  - `test_rotation_after_start_and_estimate_error`: 이동 시작 후 yaw가 +30° 도는 행 → `rotation_after_start_deg≈30`, 추정 위치 오차 14 cm가 들어간 행 → `max_estimate_error_m≈.14`.
  - `test_gate_stop_time_and_resume_count_per_phase`: `ascending:15` 구간에서 `gated` True 8 s → False → True 5 s → False → `gate_stop_s_by_phase['ascending:15#0']≈13`, `gate_resume_by_phase['ascending:15#0']==2`, 다른 구간과 섞이지 않음; `gate_stop_s_total≈13`.
  - `test_estimator_degraded_time`: 상태 문자열 `imu_gap` 2 s + `recovering` 1 s → `estimator_degraded_s≈3`.
  - `test_clearance_fields_are_merged`: `monkeypatch.setattr(follower_metrics.clearance,'evaluate',lambda o,m:{...})` 후 병합 키 확인.
  - `test_format_table_lists_each_run_and_metric`: 두 run 이름과 `over_limit_fraction` 열이 문자열에 있다. `main([...])`은 합성 run 두 개로 0을 반환하고 표를 stdout에 출력.

- [ ] **Step 2: 실패 확인**

Run: `KSRC_ROOT=outputs/site_sim/ksrc_20261016 QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m pytest tests/site_sim/test_follower_metrics.py -q`
Expected: ImportError / FAIL

- [ ] **Step 3: 구현** — numpy만 사용(이동평균은 `np.convolve`, 가장자리는 `mode='same'`이 아니라 유효 구간 정규화). `mission_folder`가 `None`이면 clearance를 부르지 않는다. 모든 지표는 JSON 직렬화 가능한 float/int로 반환.

- [ ] **Step 4: 통과 확인** — 같은 명령, Expected: 전부 PASS

- [ ] **Step 5: Commit**

```bash
git add experiments/site_sim/follower_metrics.py tests/site_sim/test_follower_metrics.py
git commit -m "sim: add follower comparison metrics tool"
```

---

### Task 7: 시뮬 검증 실행서와 결과 문서 (서버 사용, 사용자 go 필요)

**Files:**
- Create: `docs/experiments/2026-10-18-swerve-blend-follower.md`
- 산출물(커밋 안 함): `outputs/site_sim/<LABEL>/`

**Interfaces:**
- Consumes: Task 0~6 전부(코드가 커밋된 HEAD에서 업로드), `follower_metrics`
- Produces: 결과 문서(비교표, 수용 조건 판정, 코드 커밋 해시, 서버 run 라벨, 한계 서술)

**공통 규칙**
- 두 모드(`unicycle`, `blend`)는 같은 코드 커밋, 같은 새 connector 경로(`SiteMission.connector()`가 런타임에 계산, `abD1018/mission_bundle` 사용), 같은 seed로 돌린다. 실행 전 `git rev-parse HEAD`를 결과 문서에 기록한다.
- 한 GPU에 한 번에 한 실행, 최대 2실행 동시(`--gpu 0`, `--gpu 1`). 라벨 규칙: 14 s는 `bl14-{uni|bln}-s{N}`, 전체는 `blf-{uni|bln}-s{N}`. 포트는 라벨마다 다른 값(8801부터).
- 공통 start 플래그(`SEED`는 `N`으로 imu/delay/start seed 모두 같게): `--gpu G --port P --stereo-hz 15 --start-hold-s 3 --estimator --adaptive-visual-noise --control-dt 0.02 --rl-policy res8_mlp_v1_rc4000 --imu-realism --imu-drop-prob 0.008 --imu-seed N --command-delay-ms 17.6 46.5 --delay-seed N --start-noise-xy-m 0.02 --start-noise-yaw-deg 2 --start-seed N --wheel-effort 1.96`. 14 s 실행은 `--max-sim-s 14 --max-wall-s 1200`, 전체 임무는 `--max-sim-s 600 --max-wall-s 3600`. blend 실행에는 추가로 `--follower blend`(상한 0.09는 기본값, `--blend-max-yaw-rate`는 지정하지 않음).

- [ ] **Step 1: 사전 점검 (서버 미사용)**

Run: `git status --short`(깨끗), `git rev-parse HEAD`, 그리고 Task 1~6 관련 테스트 파일 전체를 위 명령으로 다시 실행.
Expected: 전부 PASS, 작업 트리 깨끗. 하나라도 실패하면 서버 단계로 가지 않는다.

- [ ] **Step 2: 사용자에게 설정을 알리고 go를 기다린다** — 실행할 라벨 수(14 s 6개 + 전체 4개), 위 플래그 전문, 예상 소요(14 s 몇 분, 전체 약 22분/개), GPU 사용을 알린다. go를 받기 전에는 어떤 `upload`/`start`도 실행하지 않는다.

- [ ] **Step 3: 14 s 비교 (모드 2 × seed 3 = 6개, 2개씩)**

각 라벨마다(예: `LABEL=bl14-bln-s1`):
```bash
bash tools/run_site_sim_server.sh upload LABEL outputs/site_sim/abD1018/mission_bundle --texture outputs/site_sim/sand_texture/sand_high.jpg --ksrc outputs/site_sim/ksrc_20261016
bash tools/run_site_sim_server.sh start LABEL --gpu 0 --port 8801 --max-sim-s 14 --max-wall-s 1200 --stereo-hz 15 --start-hold-s 3 --estimator --adaptive-visual-noise --control-dt 0.02 --rl-policy res8_mlp_v1_rc4000 --imu-realism --imu-drop-prob 0.008 --imu-seed 1 --command-delay-ms 17.6 46.5 --delay-seed 1 --start-noise-xy-m 0.02 --start-noise-yaw-deg 2 --start-seed 1 --wheel-effort 1.96 --follower blend
```
(unicycle 실행은 마지막 `--follower blend`만 뺀다.) 진행은 호스트 `hpi-lab`의 `docker ps`로 확인, 끝나면 `bash tools/run_site_sim_server.sh fetch LABEL`(`outputs/site_sim/LABEL`로 복사). 같은 seed의 두 모드 쌍을 같은 시각에 GPU 0/1에 나눠 돌린다.

- [ ] **Step 4: 14 s 지표 집계**

Run: `.venv-mapping/bin/python -m experiments.site_sim.follower_metrics outputs/site_sim/abD1018/mission_bundle outputs/site_sim/bl14-uni-s1 outputs/site_sim/bl14-bln-s1 ...`(6개 모두)
Expected: 표 출력. 초반 판정 — 같은 seed에서 `max_estimate_error_m`(blend) ≤ (unicycle), `over_limit_fraction` ≤ 0.05. 하나라도 어긋나면 전체 임무로 넘어가기 전에 사용자에게 보고하고 멈춘다.

- [ ] **Step 5: 전체 임무 (모드 2 × seed 2 = 4개, 2개씩; seed 1, 2)**

Step 3의 명령에서 `--max-sim-s 600 --max-wall-s 3600`으로 바꾸고 라벨은 `blf-uni-s1`, `blf-bln-s1`, `blf-uni-s2`, `blf-bln-s2`. 같은 seed 쌍을 동시에(GPU 0/1) 돌리고 `fetch`로 복사한다. 임무 타임라인은 기준과 같아야 한다: `to_junction → trial_entry → ascending(30) → retreating → ascending(15) → descending(10, 정상 180° 포함) → returning → complete`.

- [ ] **Step 6: 전체 임무 지표와 수용 조건 판정**

Run: `.venv-mapping/bin/python -m experiments.site_sim.follower_metrics outputs/site_sim/abD1018/mission_bundle outputs/site_sim/blf-uni-s1 outputs/site_sim/blf-bln-s1 outputs/site_sim/blf-uni-s2 outputs/site_sim/blf-bln-s2`
수용 조건(spec 그대로, blend 실행마다 판정):
  - 완주하고 접촉 표본 0.
  - 최소 벽 거리가 같은 seed의 `unicycle`보다 5 cm 넘게 나쁘지 않을 것.
  - 이동 중 GT |ω|(0.5 s 평활)가 0.11 rad/s(상한 0.09의 125%)를 넘는 시간이 5% 이하일 것. 명령이 아니라 GT로 보고, 진동과 RL 추종 오차 여유로 25%를 둔다. `ψ_ref`와 실제 yaw의 차이(`heading_gap`)가 60°를 넘는 구간(정상 180° 전환)은 텔레메트리에 기록한 값으로 식별해 제외한다.
  - 초반 위치 오차가 `unicycle`보다 나빠지지 않을 것.
  - 램프 게이트는 램프당 재개 2회 이하, 정지 40초 이하(정상 180° 전환은 평지라 해당 없음).
  참고값(옛 경로라 비교 기준 아님): `repeat1018a` 완주 253.6 s, 최소 벽 거리 30.9 cm.

- [ ] **Step 7: 결과 문서 작성 후 커밋**

`docs/experiments/2026-10-18-swerve-blend-follower.md`에 다음을 쓴다: 코드 커밋 해시, 라벨·seed 표, 14 s 비교표, 전체 임무 비교표(완주·최소 벽 거리·접촉·소요·|ω| 평균/최대·추정기 degraded 시간·게이트 정지/재개), 수용 조건별 PASS/FAIL, 한계(시뮬 ORB는 회전에 강해 ORB 개선의 근거가 아님, seed 2~3개의 통계적 한계, 실기 미검증). 수용 조건이 하나라도 FAIL이면 숫자와 실행 로그 위치를 그대로 적고 설정 조정은 사용자 결정으로 남긴다.

```bash
git add docs/experiments/2026-10-18-swerve-blend-follower.md
git commit -m "docs: swerve blend follower simulation results"
```

---

## Self-Review

- **Spec 커버리지:** 목표 1(blend 모드, 기본 불변) Task 1·3, 목표 2(vy 전달 IK·RL, replay, `vy` 없으면 0) Task 4, 목표 3(시뮬 비교) Task 6·7. 결정 1(ψ_ref, hold 옵션) Task 2, 결정 2(명령 계산, 이득 2, cos 게이트 제거, 코너 감속 유지) Task 2·3, 결정 3(램프 게이트 8/15/25) Task 1·2·3, 결정 4(상태별: retreating 후진·정상 180°) Task 3 테스트, 결정 5(방향 슬루 1.0, 0.01 미만 유지) Task 2, 결정 6(수용 조건) Task 7 Step 6. 빌더 과제인 stuck 오작동 방지는 Task 3의 `hypot` 판정과 게이트 중 effort 비누적 테스트로 덮는다. 비목표(벽 거리, 경로 생성, 실기)는 어떤 Task에도 없다.
- **타입 일관성:** `BlendFollower.step` 인자·`BlendOutput` 필드명(`heading_gap_rad`, `travel_gap_rad`, `gated`, `psi_ref`)이 Task 3의 `result['follower']`와 Task 6의 지표 입력 키에 같다. `step_plant`·`apply_follower_args`·`summarize` 시그니처는 정의 Task와 사용 Task가 같다.
- **Review Focus 대응:** 1·2·6은 Task 2, 3·4·5·8·9는 Task 3, 7은 Task 4·6, 10은 Task 7 수용 조건.
