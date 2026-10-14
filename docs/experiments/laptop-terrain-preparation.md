# 노트북 지형 정보·costmap 사전 구현

최신 통합 상태·사용법: [D435·Pi·로버 매핑 인수인계](../handoff/d435-rover-mapping-handoff.md) (2026-10-14). 아래 초기 실기 미검증 표현은 작성 당시 상태이며 이후 실제 ORB/TSDF 촬영 검증이 추가됐다. 실제 중력 정렬·지형/costmap 정확도 검증은 여전히 남아 있다.

## 지금 사용할 수 있는 흐름

```text
Pi: D435 센서 수집 → byte/item 상한 전송 큐 → SSH
노트북: 원본 저장 → 종료 후 native ORB → map별 TSDF
        명시한 map + 중력 정렬 → depth 순차 재생 → 지형 층 + costmap
```

Pi에서 ORB를 계속 실행하는 기존 방식도 유지한다. 새 Qt 옵션 **Pi는 센서 전송만 · 종료 후 노트북에서 ORB 계산**을 선택하면 Pi의 ORB/Atlas 생성을 생략한다. 노트북 native 실행기가 설치돼 있으면 이 옵션이 기본 선택된다. 합성 입력은 알려진 pose를 사용하므로 해당 옵션을 선택해도 합성 영상을 실제 ORB로 추정했다고 표시하지 않는다.

2026-10-13 19:34 KST 실제 Pi/D435 sensors-only로 준비 후 약8초 촬영해 IR좌/우/depth 각122프레임, source약15.01FPS, 전송 누락0, 정상 종료와 노트북 저장을 확인했다. USB2이며 큐 최대약0.88MiB였다. [실기 연결 검사 기록](../evidence/20261013_193447_connection_check/README.md). 이동 촬영 후 실제 노트북 ORB/TSDF와 지형 정확도는 아직 검증하지 않았다. 순수 연산/Qt 모의 촬영·합성 지형·기존 Isaac Sim 영상의 native 실행과 구분한다.

## 실행

기존 매핑 창:

```bash
cd /path/to/rover-mapping
bash tools/handheld_mapping.sh
```

장비 없이 지형·costmap 산출물 생성:

```bash
bash tools/terrain_mapping.sh --demo --output outputs/handheld_mapping/my_terrain_demo
```

이미 촬영한 sensors-only 원본에 대해 노트북 ORB만 실행:

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python -m experiments.handheld_mapping.offline /absolute/path/to/RUN
```

Qt sensors-only 모드에서는 종료 후 이 계산과 TSDF 생성이 자동으로 이어진다. CLI는 ORB 결과만 생성한다. TSDF를 따로 만들 때는 Python의 `reconstruct(run, backend='orb_offline')`를 사용한다. 기존 결과를 덮어쓰지 않으며 실패한 계산 폴더를 재사용해 성공 결과처럼 처리하지 않는다. 실패한 폴더를 보존해 별도 이름으로 옮긴 뒤 재시도할 수 있다.

최종 keyframe과 depth가 있는 기록에서 지형 생성:

```bash
bash tools/terrain_mapping.sh /absolute/path/to/RUN \
  --backend orb_offline --map-id 0 \
  --alignment /absolute/path/to/grid_from_world.json \
  --bounds -5 5 -5 5 --resolution 0.05 --stride 4 \
  --output /absolute/path/to/NEW_TERRAIN_OUTPUT
```

`--alignment`는 원래 ORB world 좌표를 z-up 중력 좌표로 옮기는 **4×4 rigid transform JSON 배열**이다. 실제 촬영의 장착 자세/중력/원점에 맞게 정해야 하며, 알려진 합성 예제의 변환을 실기에 그대로 사용하면 안 된다. 기존 Pi ORB 결과에는 `--backend orb`, 점선 결과에는 `--backend orbline`을 사용한다. 실제 map ID는 해당 기록의 `map_*_keyframes.csv` 파일명에서 확인한다.

## 저장 파일과 지형 정의

- `terrain.npz`: 모든 지형 층과 `cost`; `np.load(..., allow_pickle=False)`로 읽는다.
- `summary.json`: map/backend, 정렬 변환, grid/로버 설정, 관측·누락·범위 밖 집계, 상태와 메모리 범위.
- `preview.png`: matplotlib이 있는 노트북에서 높이·경사·거칠기·cost 그림을 생성한다.

격자 축은 `(y, x)`, 높이·단차·거칠기는 m, 경사·방향은 degree다. 셀 중심은 원점에 `(index+0.5)*resolution`을 더한 위치다. `height_m`은 셀의 프레임별 평균 높이를 누적한 값이고, `observations`는 point 수가 아닌 관측 update 수다. `variance_m2`는 프레임별 평균 높이의 표본 분산이며 검증된 pose/depth covariance가 아니다.

`slope_deg`/`heading_deg`/법선은 양쪽 이웃의 높이 차로 계산한다. `roughness_m`은 3×3 이웃의 국소 경사 평면에 대한 RMS 잔차다. `step_m`은 그 이웃의 최대 높이 차이로, 센서 해상도/잡음과 격자 해상도의 영향을 받는다. 관측 부족/이웃 부족/지도 경계는 cost가 unknown으로 남는다.

한 셀에 여러 높이의 점이 있으면 `vertical_span_m`으로 보수적인 차단 판단을 한다. 이것은 정밀한 지면 분할이나 3D footprint 충돌 검사 전체가 아니다. 벽/돌출부가 평균 높이 하나로 안전한 바닥이 되지 않도록 한 첫 기준 구현이다.

## cost 설정

`RoverLimits` 기본값은 실기에서 조정할 초기 시험값이다:

```json
{
  "max_slope_deg": 20.0,
  "max_step_m": 0.06,
  "max_roughness_m": 0.025,
  "safety_radius_m": 0.15
}
```

파일로 저장해 `--rover-limits /path/to/limits.json`으로 전달한다. `safety_radius_m`은 차체 외곽과 여유를 고려한 원형 여유이며 완전한 조향/차체 모델은 아니다. 비용은 경사0.4/거칠기0.3/높이 차0.3의 정규화 가중합이다. 실제 에너지 비용이나 위험 확률을 측정한 값이 아니다. 통과 한계를 넘으면 blocked=254, 정보가 부족하면 unknown=255, 나머지는0..252다. unknown을 평탄한 지면으로 채우지 않는다.

## 메모리와 저장 공간

- Pi 전송 큐: 기본 **압축 전 payload+header 4MiB, 최대24패킷**. 둘 중 하나가 차면 새 패킷을 거부하고 누락을 집계한다. ROS 버퍼/현재 프레임/Python 객체 overhead까지 포함한 전체 RSS 상한은 아니다. worker `--queue-mib` 또는 Python `remote_command(..., queue_mib=...)`로 조절할 수 있다.
- Pi 원본 영상은 파일로 쌓지 않는다. 코드/로그는 `/dev/shm`에 있어 RAM을 사용하며, RAM disk 여유32MiB 미만이면 촬영을 실패 종료한다. 로그의 전체 용량을 별도로 제한하거나 이전 실행을 자동 삭제하는 기능은 없다. 노트북 저장을 확인한 이전 실행의 RAM 폴더만 이후 정리한다.
- sensors-only는 Pi의 Atlas 성장을 피한다. 기존 Pi ORB 모드의 내부 맵 메모리를 이 큐가 제한하는 것은 아니다.
- 지형은 고정 XY 범위의 float32 통계와 정수 count/timestamp만 유지하며 프레임/점군을 누적 보관하지 않는다. **10m×10m, 5cm 격자의 지속 통계 배열은1,120,000B(약1.07MiB)**다. 파생 층/임시 배열/한 프레임/Python·라이브러리 메모리는 별도로 필요하다.
- 기본 `max_cells=250000`, update당 `max_points=100000`. 초과하면 할당/투영 전에 거부한다. `--max-cells`, `--max-points`, `--stride`로 조절한다. 범위 밖 관측은 버리고 개수를 보고하며 자동 확장하지 않는다.
- 촬영 인덱스는 순차 스캔, native keyframe 순서와 처리시간 분위수는 노트북 임시 SQLite 인덱스로 처리한다. SQLite page cache는 각2MiB다. 원본 CSV/영상은 수정하지 않으며 소유 임시 인덱스는 종료 후 삭제한다.
- native ORB 내부 맵과 TSDF volume은 노트북에서 지도 크기에 따라 커진다. 이번 구현이 그 전체 메모리를 고정 크기로 제한하지는 않는다. 원본 영상은 노트북 디스크에 보존하며 GUI의 기존 여유 공간 검사를 유지한다.

## 다음 실기 순서

1. Pi sensors-only로 정지/작은 왕복 촬영. 실제 IR/depth 수신률·누락·RAM 여유·전송 종료를 확인한다.
2. 노트북 ORB/TSDF 후 알려진 거리와 단차를 검사한다. 중력 정렬을 정하고 지형 층을 생성한다.
3. 실제 로버 조건으로 경사/단차/거칠기/원형 여유 값을 조정한다.
4. 이후 별도 주행의 형상 위치 보정, IMU·encoder의 실제 topic/축/clock, live pose-depth 동기화와 주기적 costmap 갱신을 연결한다.

`TerrainGrid.update(points, stamp_ns)`는 이미 주기별 관측을 받을 수 있다. 현재 Qt가 live pose와 depth를 자동 동기화해 주기별 costmap을 표시하는 기능까지 연결된 것은 아니다. 최종 pose가 바뀌면 원본으로 재융합해야 하며, 슬립·물성·Nav2·경로 제어도 별도 단계다.

검증 기록: [합성 지형·기록 재생·native ORB 결과](../evidence/20261013_terrain_validation/README.md)

## 2026-10-14 추가: 실시간 촬영 범위 안내

매핑 창의 **실시간 촬영 범위 표시 · 노트북에서 ORB 동시 실행**을 선택하면 IR 옆에 관측 격자·카메라 위치/경로·시야 방향이 나온다. native baseline 실행기가 있으면 기본 ON이다. `bash tools/handheld_mapping.sh`로 실행한다. 이미 열려 있던 이전 버전의 창은 촬영/저장 종료 후 닫고 다시 열어야 한다.

Pi는 기존 센서 전송을 유지하고 노트북 native ORB가 원본 저장 뒤의 파일 경로를 받아 추적한다. 왼쪽 미리보기와 raw 저장이 live ORB를 기다리지 않는다. live 입력은 최신2쌍, 좌우 미매칭 각8stamp, depth metadata32개만 보유한다. 계산이 밀리면 오래된 live 입력만 건너뛰며 UI에 별도로 집계한다. 전송 누락과 live 건너뜀은 다른 지표다.

처음 vocabulary 로딩에는 몇 초가 걸릴 수 있다. **오른쪽의 추적 정상 표시를 확인한 뒤 움직이면 초기 관측 안내 누락을 줄일 수 있다.** live가 실패해도 원본 기록/종료 후 전체 영상 ORB·TSDF 계산은 유지한다. 종료 후 full raw로 재계산한 최종 지도와 실시간 잠정 격자는 다를 수 있다.

색은 회색=미관측, 노랑=1–2회, 초록=3회 이상 시점 변화 관측 참고값이다. 셀별 이전 관측 카메라에서 투영 위치10cm 또는 방향10° 이상 바뀌어야 횟수를 늘린다. 정지 프레임 개수를 그대로 세지 않는다. 전체 과거 시점의 독립성/정확도/완전한 촬영/주행 가능성을 보장하는 값은 아니다. 유효 depth 표면을 stride8, .25–3m로 투영하며 벽도 표면 관측에 포함한다. 시야 부채꼴은 방향 설명이고 회색을 관측 상태로 채우지 않는다.

현재 격자는 **초기 optical world의 X-Z 투영**, 20m×20m/5cm 고정 범위다. IMU 중력 정렬 전이므로 물리적 수평 지형 맵이나 costmap이라고 부르지 않는다. 범위 밖 점은 집계 후 버리고, 카메라 위치를 로버 중심으로 바꾸지 않는다. map ID가 바뀌면 이전 격자를 저장하고 새 격자로 시작하며 서로 겹치지 않는다. 최적화/loop closure 뒤 실시간 격자를 재융합하는 기능은 아직 없으므로 촬영 안내용 잠정값이다.

추가 산출물은 `orb_live/live_poses.csv`, `timings.csv`, `map_*_keyframes.csv`, `coverage_map_*.npz`, `summary.json`, `runner.log`다. 관측 상태 배열은2,240,000B이며 RGB snapshot/임시 projection/Python/native ORB RSS는 별도다. 원본/최종 TSDF는 기존 폴더에 보존한다.

UI의 **수신 후 ms**는 노트북 stereo 저장 후 pose 수신까지이며 센서·SSH·화면 렌더링 지연은 포함하지 않는다. 1초 이상 오래된 결과는 오래됨으로, 추적 실패는 갱신 중지로 표시한다. live 종료 오류는 별도로 기록하며 저장 영상 재생은 calibration의 source를 따라 실제 Pi 촬영과 구분한다.

검증: 실제 D435 저장 영상20초를 원래 약15FPS로 재생하면서 Qt/raw저장/live ORB/final ORB/TSDF까지 실행했다. live245쌍 처리, 초기 vocabulary warmup53쌍 건너뜀,10초 이후 추가 건너뜀0, tracking/coverage 갱신·최종98keyframe TSDF 생성 확인. UI 상태82샘플의 수신 후 lag 중앙값20.51ms/p9520.71ms였다. **현장 Pi와 네트워크를 포함한 실시간 시험이나 전체 end-to-end latency/GT 정확도 검증은 아니다.** [검증 산출물](../evidence/20261013_live_validation/README.md).

IMU·encoder 동시 기록/clock·장착 보정/stereo-inertial 적용은 여전히 후속 작업이다. 이번 실시간 안내 기능이 IMU를 사용한다고 표시하지 않는다.

2026-10-14 추가: 카메라 원본 위/관측 색 오버레이 아래를 함께 표시하고 저장된3D지도를 여는 전용 CLI·앱 버튼을 추가했다. [카메라 오버레이·3D 지도 보기 사용법](mapping-viewer.md).
