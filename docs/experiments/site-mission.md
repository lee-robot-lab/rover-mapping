# 팀원 STL 이후 대표점 방문·우회·복귀 준비

B는 **여의도 중간 테스트 기회**다. 매핑과 STL 생성은 팀원 담당이며 이 도구는 받은 STL부터 사용한다. 실제 대회 사천에서는 사천 prior와 사천 관측 지도를 별도로 보존·정합한다.

현재 코드는 STL 지형 입력, 대표점·세 경로 편집·저장, pose 기반 임무 상태·속도 요청, 합성 재생을 제공한다. **실제 모터에 송신하거나 MCU/firmware를 변경하지 않는다.** ORB pose를 site 좌표로 정렬하는 현장 adapter와 실제 제어 연결·모래 등판 성능은 별도 실기 통합/검증 대상이다.

## 시작

코드 기준은 `/home/su/rover-mapping`이며 기존 `.venv-mapping`을 사용한다. Conda 환경은 섞지 않는다. 창을 실행할 때 현재 Qt 매핑 앱과 별도 프로세스가 열린다.

```bash
cd /home/su/rover-mapping
# 장비 없이 설정 창 확인
bash tools/plan_site_mission.sh --demo

# 실제 팀원 STL. 원본 단위를 반드시 확인한다.
bash tools/plan_site_mission.sh --stl /absolute/path/team_map.stl --units m
# mm 단위라면 --units mm
```

다른 실행 폴더 `/home/su/rover`에서 같은 이름의 예전 wrapper를 찾지 않는다. 별도 Python 경로가 필요할 때만 `ROVER_MAPPING_PYTHON=/absolute/path/to/mapping/python`으로 명시한다. 기본값은 이 코드 checkout의 `.venv-mapping/bin/python`이다.

## 지도와 임무 설정

1. **STL 단위·up·격자·로버 여유 반경**을 확인한다. up은 원본 STL 좌표의 위쪽 벡터이며 기본 `(0,0,1)`이다. 전체 rigid 회전만 적용하고 경사를 수평화하지 않는다. 예를 들어 원본의 +Y가 위쪽이면 up을 `0,1,0`으로 지정한다. CLI는 `--up 0 1 0`이다.
2. 기본 **다층 보수 검사**는 겹친 표면을 unknown으로 둔다. 두꺼운 CAD 바닥의 아랫면 때문에 지면이 전부 거부되면 먼저 단위/up과 실제 geometry를 확인한다. 상부 표면만 쓰겠다고 판단한 경우에만 **상부 표면 명시 선택** 또는 `--surface-mode top`을 사용한다. 이 선택은 저장 provenance에 남는다. 다층 공간의 아래쪽 통로는 표현하지 못한다. 벽/단차/구멍/footprint 검사는 유지한다.
3. **출발 위치→방향 두 클릭**: 첫 클릭은 출발 XY, 두 번째는 heading이다. 이 값은 수동 prior이며 자동 localization 성공이 아니다.
4. **갈림길**, **정상 대표점**을 지정한다. 세 길은 같은 갈림길과 정상에 연결된다.
5. 각 **10°/15°/30° 경유점** 모드에서 갈림길 이후부터 정상 이전까지 중간점을 순서대로 클릭한다. 양 끝은 자동 연결되므로 다시 찍지 않는다. 30° 첫 중간점은 등판 시험 전 방문할 진입 대표점이다. 각도 이름은 사용자가 부른 경로 이름이며 실제 grid의 경사값과 별개다.
6. 필요하면 **Undo** 또는 **선택 경로 지우기**로 수정한다. 지도 zoom/pan 도구가 켜져 있으면 클릭 편집하지 않는다.
7. **경로 검사**: 모든 구간의 지도 범위·관측·벽/단차·footprint·경사를 검사한다. 일반 경로 제한 기본 20°, 30° 시험 경로 기본 35°이며 실기 한계의 확정값이 아니다. 시험 경로 예외가 unknown/장애물까지 허용하지 않는다.
8. 출력 폴더를 지정하고 **새 임무 저장**한다. 기존 폴더는 덮어쓰지 않는다. **저장 임무 열기**로 다시 열어 수정 후 새 버전으로 저장할 수 있다.

출력은 `mission.json`, `site/site.json`, `site/grid.npz`, `site/mesh.ply`, 방향별 `routes/*_up.csv`와 `*_down.csv`, `preview.png`다. source STL·단위·변환·grid·설정 해시를 함께 보존한다. 원본 STL이 변경되거나 bundle geometry/grid 해시가 다르면 재로드를 거부한다. 원본 STL 경로를 옮겼더라도 bundle 형상과 provenance는 유지된다.

## 임무 정책

기본 순서는 출발 → 갈림길 → 30° 진입 대표점 → 등판 시도다. 진행이 부족하면 실제 지나온 구간을 역방향으로 후퇴해 갈림길에 돌아오고, 15° 상행으로 전환한다. 15°도 실패하면 10° 상행을 사용한다. 정상 방문 후에는 10° 하행과 출발지 복귀를 실행한다. 30°가 성공하면 실패를 연출하지 않고 실제 성공을 기록한다.

진입 대표점 방문, 등판 결과, 정상 도착, 복귀 완료는 별도다. 한 상행의 실패로 정상 전체를 도달 불가로 바꾸지 않는다. 모든 상행이 실패하거나 복귀 경로가 없으면 이유와 함께 실패로 기록한다. 전체 면적 coverage를 수행하는 임무가 아니다.

도착 tolerance·속도·시도 상한·진행 부족 창·최소 진행량은 설정 UI/JSON에 있다. 이 기본값은 시뮬레이션 시작값이고 실기에서 검증한 로버 한계가 아니다. 후퇴는 지도에서 아직 가보지 않은 정상 구간을 사용하지 않는다. 추정 pose의 진행 부족을 관측한 결과는 stuck이며 wheel slip 실측치가 아니다.

## 합성 시나리오 재생

```bash
# GUI 없이 합성 임무를 새 폴더에 준비
bash tools/plan_site_mission.sh --demo --save-only --output outputs/site_demo_01

# 30° 중단 → 15° 우회 → 정상 → 10° 하행 → 복귀
bash tools/run_site_mission.sh outputs/site_demo_01 outputs/site_replay_01 --scenario stall30
```

`success`, `stall30`, `stall30_15`, `stall_all`, `tracking_loss`, `return_stall`, `descent_blocked`, `stall30_entry` 시나리오를 지원한다. 임무 폴더의 실제 경로를 사용하되 demo plant는 평면 운동학과 명시적으로 주입한 정체다. 경로 이름 10°/15°/30°가 있는 합성 평면이며 물리적 경사·모래·토크를 시뮬레이션하지 않는다. 실제 임무를 재생해도 실기 등판 성능이 검증되는 것은 아니다.

각 새 결과 폴더에 `commands.csv`, `events.jsonl`, `replay.png`, `summary.json`을 저장한다. 성공 여부뿐 아니라 실패 경로·후퇴 이벤트·정상/복귀 상태·관측 손실을 확인한다.

## 실제 pose 로그로 실행 준비

```bash
bash tools/run_site_mission.sh --input /absolute/path/aligned_poses.jsonl \
  /absolute/path/mission /absolute/path/new_result
```

입력 JSONL의 한 행은 다음 구조다. `site_identity`는 저장한 `mission.json`의 값이고, `position_m`과 `yaw_rad`는 **실제로 site 좌표로 정렬된 pose**여야 한다. 단순히 raw ORB frame 이름만 바꾸면 안 된다.

```json
{"stamp_s":12.0,"now_s":12.03,"frame":"site","site_identity":"저장 임무의 site_identity","pose_epoch":0,"position_m":[0.5,0.5],"yaw_rad":0.0,"tracking_valid":true}
```

`stamp_s`와 수신 `now_s`는 선언한 같은 clock 기준이어야 한다. MCU/Pi/노트북의 시각 숫자를 임의로 섞지 않는다. `pose_epoch`는 같은 정렬을 유지하는 map/reset 구간 ID다. epoch 변경, 큰 pose 점프, lost, stale, 역행·중복 시각에서 정지 요청을 낸다. epoch 변경은 새 정렬과 runtime 재시작이 필요하며, 통신 단절 뒤 늦은 입력으로 도착·등판 실패를 확정하지 않는다.

출력 `commands.jsonl`에는 상태·대표점 상태·현재 경로/방향·유효시각·bounded `vx_m_s`, `yaw_rate_rad_s` 요청이 있다. EOF나 입력 파싱 오류에도 마지막에 0 속도 요청을 기록한다. **이 실행기는 파일 출력만 하며 실제 모터를 움직이지 않는다.** live 제어 연결 시 독립 watchdog/정지 경로를 두고 시뮬레이션/fake 통신 검증 후 저속 실기로 검증한다.

## 현재 한계

- 실제 팀원 STL, 여의도 센서/단위/장착/추적 정확도와 모래 등판·후퇴는 미검증이다.
- 2.5D 표현이며 다층·가려진 바닥·얇은 구조·grid보다 작은 형상에는 한계가 있다. footprint는 보수적인 원형 근사다.
- 실시간 STL 정합·고정 지도 기반 pose 보정, live ORB→site adapter, CAN/모터 송신, 바퀴별 slip 추정은 이번 도구가 자동으로 제공하는 기능이 아니다. 준비한 timestamp/frame/validity 계약에 별도로 연결한다.
