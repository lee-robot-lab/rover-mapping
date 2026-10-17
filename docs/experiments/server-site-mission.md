# 서버 Isaac 대회 임무·costmap·실시간 관찰

이 도구는 서버의 기존 경기장·4륜 조향 로버에 `SiteMission`을 연결한다. simulator의 실제 pose feedback을 사용해 joint를 구동하며 경로 중간에 pose를 강제로 이동시키지 않는다. 입력 pose 출처는 **sim_ground_truth**, 모래는 **기존 MPM 결과를 구운 고정 표면**이다. ORB 정확도·변형 모래·실제 로버 등판 성능 검증과 구분한다.

## 경로와 임무

출발 → 갈림길 → 30° 진입 대표점 → 등판 시도 → 공통 정상 → 10° 하행 → 출발지 복귀를 수행한다. 진행 부족 시 실제 지나온 경로로 후퇴해 15°를 선택하고, 15°도 실패하면 10°를 사용한다. 자연 주행에서 30°가 성공하면 그대로 성공으로 기록한다.

서버 경기장과 일치하는 meter 형상에서 검증한 경로를 사용한다. 기존 STL은 원본 좌표에 **0.1**을 곱하며, sand NPZ는 **height[x,y]**다. 팀원 STL 일반 입력의 m/mm와 혼동하지 않는다. 입력 두 파일의 hash와 좌표 변환은 `simulation.json`에 저장한다.

2026-10-17 준비 bundle `outputs/site_sim/20261017c/mission` 기준: 30° 경로 1.66m/최대 32.53°, 15° 4.90m/15.00°, 10° 9.80m/10.38°. 이름과 실제 최대 경사는 다르다. 사용자 요청으로 출발점은 왼쪽 상단 `(2.9,1.2,0 rad)`이며 갈림길 연결·복귀도 재검사했다. 원형 footprint 0.23m·clearance 0.02m·도착 여유 0.05m를 합한 grid 반경 0.30m와 추가 셀 여유를 사용했다. 이는 현재 asset용 보수 근사이며 실기 한계가 아니다.

명시적 top surface 모드의 grid revision 3은 모래 아래에 묻힌 CAD 내부 수직면을 표면 장애물로 투영하지 않는다. 돌출한 벽과 미관측 영역은 계속 차단한다. grid revision 2 이전 bundle은 다시 준비한다.

## 노트북에서 준비

코드는 `/home/su/rover-mapping`, Python은 `.venv-mapping`이다. 서버 Python과 패키지 환경은 변경하지 않는다.

```bash
cd /home/su/rover-mapping
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.site_sim.prepare \
  --stl /home/su/rover/gyusa_fence40.stl \
  --sand /home/su/rover/results/mpm_map/sand_height_v15_m5.npz \
  --output outputs/site_sim/my_run/mission
```

`preview.png`, `route_report.json`, `mission.json`, site geometry/grid, 방향별 route CSV와 provenance가 생성된다. 안전 경로가 없으면 bundle을 생성하지 않는다. 범위를 줄이거나 장애물 검사를 끄는 대신 실제 후보 경로와 footprint를 검토한다.

## 서버 실행과 실시간 관찰

먼저 서버 GPU 사용량을 확인한다. 기본은 GPU 1이며 다른 작업이 사용 중이면 실행을 거부한다. 2026-10-17 사용자가 이번 실행을 유휴 GPU 0으로 변경하도록 승인했으므로 아래 명령은 `--gpu 0`을 명시한다. 기존 작업·TensorBoard·`latest` 링크를 변경하지 않는다.

```bash
ssh hpi-lab nvidia-smi
bash tools/run_site_sim_server.sh upload my_smoke outputs/site_sim/my_run/mission
bash tools/run_site_sim_server.sh start my_smoke --gpu 0 --max-sim-s 15 --max-wall-s 300 --port 8765
bash tools/run_site_sim_server.sh tunnel my_smoke 8765
bash tools/run_site_sim_server.sh status my_smoke
```

노트북 브라우저에서 **http://127.0.0.1:8765**를 연다. 이 주소는 해당 노트북에서만 접근하며 다른 PC에서는 별도 SSH tunnel이 필요하다. renderer 초기화·URDF import·안착 중에는 영상과 상태가 아직 준비되지 않을 수 있다. `output/ready.json`과 실제 영상 갱신을 확인한다.

화면에는 전체 경기장/전방 영상, normal 또는 trial costmap, 세 계획 경로와 실제 궤적·현재 pose, 대표점 상태, sim/wall 시간, 기록 수와 age가 표시된다. 좁은 Codex 패널에서는 두 영상을 나란히 배치한다. 화면 갱신 지연은 stale로 표시하며 이미지 age도 별도다. 브라우저를 닫아도 서버 주행과 기록은 계속된다.

짧은 smoke가 통과하면 새 label로 전체 자연 주행을 실행한다. 기존 run 폴더를 덮어쓰지 않는다.

```bash
bash tools/run_site_sim_server.sh upload my_natural outputs/site_sim/my_run/mission
bash tools/run_site_sim_server.sh start my_natural --gpu 0 --port 8767
bash tools/run_site_sim_server.sh tunnel my_natural 8767
```

후퇴·우회 상태 전이만 확인할 별도 정체 주입 시험:

```bash
bash tools/run_site_sim_server.sh upload my_fault outputs/site_sim/my_run/mission
bash tools/run_site_sim_server.sh start my_fault --gpu 0 --fault-injection --port 8768
bash tools/run_site_sim_server.sh tunnel my_fault 8768
```

`--fault-injection`은 30° 상행 진입 후 wheel velocity target을 0으로 제한한다. requested command와 applied command, 주입 event를 각각 기록하며 화면에 시험임을 표시한다. 실제 pose를 조작하지 않고, retreat 상태부터는 원래 명령을 적용한다. 이것을 자연적인 모래 등판 실패로 해석하지 않는다.

## 기록과 종료

서버 실행 한 번의 위치는 `/home/sumee/gyusa/runs/site_mission/LABEL/`이다.

| 파일 | 의미 |
|---|---|
| `source_manifest.json` | 코드 commit·업로드 소스 파일별 hash·입력 자산 provenance |
| `launch.json`, `image_id.txt` | 정확한 container argv·GPU·시간 상한·observer port·이미지 식별자 |
| `mission/`, `rover/` | 임무와 portable rover 입력 사본 |
| `run.log`, `exit.txt` | simulator 출력과 프로세스 종료 코드 |
| `output/manifest.json` | 지도 identity·frame·physics/control 주기·costmap 출처 |
| `output/telemetry.jsonl`, `.csv` | sim/wall 시각·실제 pose·joint 관측·요청/적용 명령·임무 상태 |
| `output/events.jsonl` | 대표점·등판 실패·후퇴·대체 선택·주입·복귀 이벤트 |
| `output/snapshots/*.npz` | normal/trial cost, visit_count·last_seen·stuck_count와 시각·좌표 |
| `output/live/` | 독립 observer가 읽는 최신 상태·지도·JPEG |
| `output/stop.json` | 종료 시 적용한 zero wheel target 여부. 실제 wheel 속도가 0임을 뜻하지는 않음 |
| `output/summary.json`, `completion.json`, `final.png` | 실제 완료/실패/timeout과 최종 지도·궤적 |

cost 값은 risk 0..252, blocked 254, unknown 255다. normal/trial은 경사 허용값만 다르고 벽·미관측을 허용하지 않는다. 주행 경험은 실제 관측한 방문·정체 이력으로 별도 보존하며, 이 첫 시험에서 경험 층으로 자동 재계획하지 않는다. slip·sinkage는 계산하지 않아 unavailable이다.

physics dt는 0.005초, 제어 기록 목표는 0.05초다. 영상은 wall 기준 낮은 주기로 렌더링한다. startup, sim time과 wall time은 별도이며 실제 배속 1을 보장하지 않는다. sim 최대 900초, wall 최대 3600초, snapshot 최대 1200개로 제한한다.

성공 여부는 종료 코드나 `status` 명령의 성공 여부 대신 **completion의 status와 대표점·정상·복귀 결과**로 판단한다. 중간 실패도 부분 로그를 남긴다. simulator 종료 후에도 observer는 최종 결과를 보여주므로 검토 뒤 다음 명령으로 해당 run만 정리한다.

```bash
bash tools/run_site_sim_server.sh stop my_natural
```

결과 폴더는 보존하며 다른 container/observer에는 영향을 주지 않는다. 동일한 label로 다시 실행하지 말고 새 label을 사용한다.

## 기존 소스의 출처

다음 파일은 `/home/su/rover`의 검증된 기존 실험에서 가져왔다. source 사본과 이후 수정 여부는 Git 이력으로 확인한다.

| 상대 경로 | 가져온 원본 SHA256 |
|---|---|
| `experiments/orbslam3_ros2/swerve.py` | `872338e44c738825f9e2ba0c66ae3d6d3ace4563b073366057d013fc6e785545` |
| `experiments/orbslam3_ros2/drive_runtime.py` | `3cc26e8488883c6c9d64596058bcd0f370b133c21193380d8268c4325609c1ac` |
| `experiments/orbslam3_ros2/isaac_scene.py` | `87e41529a95ce2e3cfd88b9b0eaf39f9de10a8f6677d76e25fe138041172a9b0` |
| `experiments/orbslam3_offline/calibration.py` | `d0163ca4ee81ea32a3f75d11061894dd420104d43fe16fcd3b7c6a02dd0dbe21` |

Camera RGB annotator 연결은 서버 이미지에 설치된 공식 예제 `/isaac-sim/standalone_examples/replicator/augmentation/annotator_augmentation.py`의 render_product/annotator attach/get_data 경로를 기준으로 한다. 새 엔진 API를 다른 버전에 그대로 적용하지 않는다.

## 남는 범위

live ORB→site 정렬, 실제 모터 adapter, 동적 장애물 회피·온라인 재계획, 변형 모래·견인력 보정, RL 비교, 사천 prior 병합은 별도다. 서버 물리 주행과 영상·costmap 기록이 실제 실행된 범위만 아래 검증 기록에 남긴다.

## 최신 URDF·모래 질감·stereo 기록·추정 위치 주행 (2026-10-17~18)

- **로버 자산**: 기본 `--assets`는 `outputs/site_sim/rover_assets_new`다. `python -m experiments.site_sim.rover_assets --output outputs/site_sim/rover_assets_new`가 최신 `rocker_bogie_4wd_rover` URDF를 미터 단위·상대 경로 mesh로 바꾸고 FK로 `swerve_geometry.json`(조향 부호, 구름 방향)을 만든다. Isaac import에서 로버 visual mesh가 관찰 카메라에 그려지지 않아(원인 미규명) 관찰용 주황 상자를 대신 쓴다. 물리와 무관하다.
- **모래 질감**: `upload ... --texture FILE.jpg`. `texture_quilt.py`로 사진 모래에서 5.6×4.2 m 비반복 이미지와 대비 단계를 만든다. 질감은 외관만 바꾼다(모래는 고정 메시).
- **stereo 기록**: `start ... --stereo-hz 15 --start-hold-s 3`은 좌우 PNG·GT·보정을 `output/stereo`에 저장하고, 시작 3초 동안 정지해 IMU 영점을 허용한다. 영상 장면은 timestamp보다 약 0.1 s 늦다.
- **추정 위치 주행**: `start ... --estimator`는 서버에 ORB 보조 컨테이너(`rover-orb-<label>`)를 같이 띄우고 임무 실행기가 **추정 위치만** 본다. GT는 기록과 비교용이다. 사전 준비: 서버에 `stream_orb`(`experiments/site_sim/stream_orb.cpp`를 ORB 이미지에서 빌드)와 `assets/orb_stereo.yaml`. 자세한 구성은 [시뮬 VIO와 닫힌 루프](2026-10-18-sim-vio-closed-loop.md)를 본다.
- **터널**: 터널은 약 10분 끊김을 견딘다(`ServerAliveCountMax=40`). 죽으면 자동 재연결되지 않으므로 같은 `tunnel` 명령으로 다시 연다. 서버의 8765 등은 다른 실행의 viewer가 잡고 있을 수 있으니 새 포트를 쓴다.
