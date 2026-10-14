# 사용 설명서

기준 날짜: 2026-10-14. 모든 노트북 명령은 **clone한 `rover-mapping` 루트**에서 실행한다. 먼저 [설치 설명서](installation.md)를 따른다. 이 문서는 운영 절차이고, 구현/검증 상태는 [인수인계서](handoff/d435-rover-mapping-handoff.md)에 있다.

## 1. 장비 없이 먼저 확인

```bash
bash tools/handheld_mapping.sh --demo
```

1. **합성 입력으로 시험**이 선택됐는지 확인한다.
2. **매핑 시작**을 누르고 원본 IR과 frame 수가 갱신되는지 확인한다.
3. **종료 및 지도 생성**을 누르고 저장 완료를 기다린다.
4. **저장 폴더 열기**, **3D 지도 보기**로 원본과 TSDF 결과를 확인한다.

합성 입력은 알려진 pose를 사용한다. native ORB 없이 실행 가능하지만 native 실행기가 없으면 live 안내 옵션은 비활성화된다. demo가 실제 카메라/SLAM 정확도 시험은 아니다.

## 2. 실제 촬영 준비

| 준비 | 확인 |
|---|---|
| 카메라 | D435를 Pi에 연결. 케이블·포트·실제 USB 속도 확인 |
| 노트북 native | `outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare`와 runtime/vocabulary 준비 |
| Pi | native Jazzy/RealSense 설치, 기존 runtime/profile 경로 확인 |
| 통신 | 노트북→Pi SSH 접속. 조종할 경우 UDP5005도 별도 확인 |
| 중복 실행 | 이전 camera/ORB 정상 종료. SPI bridge는 단일 owner |
| 저장 | 시작 전 노트북 여유2GB 이상. 원본과 지도를 보존할 공간 준비 |

```bash
ssh -o BatchMode=yes -o ConnectTimeout=8 pi@raspberrypi.local hostname
bash tools/handheld_mapping.sh
```

Pi IP가 달라졌으면 앱의 **Pi 연결**을 `pi@현재_IP`로 바꾼다. 과거 hotspot IP를 그대로 쓰지 않는다. 앱을 열기만 해서는 카메라·모터가 시작되지 않는다.

native가 준비되면 다음 두 옵션을 사용한다.

- **Pi는 센서 전송만 · 종료 후 노트북에서 ORB 계산**: ON.
- **실시간 촬영 범위 표시 · 노트북에서 ORB 동시 실행**: ON.
- 처리 방식은 **ORB-SLAM3 + TSDF**로 시작한다. 점선 비교는 선택적 후처리다.

## 3. 촬영 시작·이동·종료

1. 저장 폴더를 선택한다. 기본값은 clone 경로의 `outputs/handheld_mapping/sessions`다.
2. **매핑 시작**을 누른다. 매번 새 RUN을 만들고 worker를 Pi RAM disk에 배포한다.
3. 정지 상태에서 카메라 준비와 오른쪽 추적 정상 표시를 기다린다. 초기 vocabulary 로딩으로 live pair를 일부 건너뛸 수 있다.
4. 작은 구역을 천천히 이동하며 표면을 여러 위치에서 겹쳐 본다. 시작점으로 돌아와 재방문한다. 거리/단차를 실측할 수 있으면 함께 기록한다.
5. 로버를 사용했다면 **먼저 조이스틱으로 정지**한다. 매핑 종료는 모터 정지가 아니다.
6. **종료 및 지도 생성**을 누른다. 정상 마감·전송·offline ORB·TSDF가 끝날 때까지 Pi 전원을 유지한다.
7. `complete`/`partial`/`failed`와 RUN 경로를 확인한다. 종료 상태와 지도 품질 합격은 다르다.

창 닫기/Ctrl+C도 소유 세션의 정상 마감을 요청한다. 촬영 중 강제 전원 차단이나 임의 killall을 정상 종료로 대신하지 않는다. 실패해도 저장된 원본은 남는다.

## 4. 화면과 지표 읽기

| 화면 / 지표 | 의미 |
|---|---|
| 왼쪽 위 | 원본 IR |
| 왼쪽 아래 | 같은 frame의 관측색 오버레이 |
| 오른쪽 | 잠정 coverage·camera 위치/경로·시야 방향 |
| 회색/노랑/초록 | 미관측 /1~2회 /3회 이상 시점 변화 관측 참고값 |
| 저장 frame 수 | 노트북에 실제 저장한 stream별 수 |
| 전송 drop | Pi queue 거절 누적 |
| live 건너뜀 | 촬영 안내의 오래된 pair 생략. 전송 drop과 별개 |
| 수신 후 ms | 노트북 stereo 저장 후 pose 수신까지. 전체 지연 아님 |

카메라 입력 요청15FPS, 화면/overlay는 최대5Hz다. 초록은 정확도·통행 가능성·충분한 모든 시점의 보증이 아니다. coverage는 초기 optical X-Z 투영이며 아직 중력 정렬된 수평 맵이 아니다. 벽이나 서로 다른 높이의 표면도 투영에 들어온다.

추적 손실/유효 depth 없음/오래된 결과에서는 색을 제거한다. live가 실패해도 원본 기록과 종료 후 전체 재계산을 유지한다. 최종 맵은 live 잠정 격자와 다를 수 있다.

## 5. 조이스틱 조종

매핑 앱과 별도다. 기존 Pi/STM 준비·배선·정지 응답을 확인한 뒤 사용자가 조작한다. 코드 변경은 fake/시뮬레이션을 먼저 검증하고 실제 기체는 바퀴를 띄워 방향·정지부터 확인하는 것을 권고한다.

Pi 별도 터미널:

```bash
cd /home/pi/ksrc
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u pi/common/udp_spi_bridge.py \
  --hz 50 --max-lin 0.05 --max-ang 0.3
```

노트북 저장소 루트:

```bash
PYTHONNOUSERSITE=1 .venv-mapping/bin/python \
  teleop/ksrc_snapshot/pi/teleop/teleop_joystick.py \
  --udp raspberrypi.local --rate 50 --max-lin 0.05 --max-ang 0.3 --monitor
```

- 왼쪽 패드: `vx/vy` 병진. 오른쪽 패드: 회전. Q/E를 누르며 병진을 줄 수 있다.
- 마우스를 놓으면 해당 패드0, Space/포커스 상실은 전체0, Esc는 종료다. 실제 정지 응답도 확인한다.
- 속도 제한은 bridge와 조이스틱 **양쪽**에 있다. 병진 m/s, 회전 rad/s다. 위 값은 저속 시작 예시다.
- SPI owner는 하나만 실행한다. bridge 중에 별도 SpiLink monitor를 동시에 열지 않는다.
- `--monitor` 요약은 전체 IMU/encoder 동시 기록이나 관성융합이 아니다.
- Pi의 원래 `/home/pi/ksrc`와 snapshot의 버전/firmware가 다르면 먼저 대조한다. 이 저장소는 전체 STM firmware를 포함하지 않는다.

SPI 배선: Pi 물리23(SCLK)→PB13,19(MOSI)→PC3,21(MISO)→PC2,24(CE0)→PB12,20 또는25(GND)→GND. 상세는 [인수인계서](handoff/d435-rover-mapping-handoff.md)를 따른다.

## 6. 저장 결과 확인·3D 보기

```text
RUN/
  session.json, capture_calibration.json, frames.jsonl, events.jsonl
  raw/{left,right,depth}/*.png
  orb/camera.log, worker.log
  orb_live/                    잠정 live pose/coverage
  orb_offline/                 최종 point pose/Atlas/keyframe
  dense_offline/               sensors-only 기본 TSDF
  dense/                      Pi ORB 또는 합성 TSDF
  orbline/, dense_orbline/      선택적 점선 결과
```

`session.json.state`, drop/counts, native `runner_complete.json`, dense `summary.json`의 융합 수·누락·map 수를 확인한다. `complete`는 절차 완료이며 실측 정확도 판정이 아니다. map ID `0/421`은 두 맵의 이름이다.

```bash
# 최신 RUN
bash tools/view_mapping.sh
# 실제 본인 RUN으로 교체
bash tools/view_mapping.sh /absolute/path/to/RUN
bash tools/view_mapping.sh /absolute/path/to/RUN --map-id 0
bash tools/view_mapping.sh /absolute/path/to/RUN --cloud
bash tools/view_mapping.sh /absolute/path/to/RUN --backend orbline
# 직접 PLY: backend/map-id를 함께 주지 않음
bash tools/view_mapping.sh /absolute/path/to/map_0_mesh.ply
```

서로 다른 map은 별도 창·독립 좌표계다. 근거 없이 합치지 않는다. 왼쪽 드래그 회전, 휠 확대·축소, Shift+왼쪽 드래그 이동이다. 최신 RUN에 아직 지도가 없으면 오류를 보여주며 이전 RUN으로 자동 돌아가지 않는다.

## 7. 재계산과 점선 비교

중요한 기준 맵은 RUN 전체를 보존한 사본에서 재계산한다.

```bash
# ORB만 새로 계산. orb_offline이 있으면 덮어쓰지 않음
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.offline /absolute/path/to/RUN

# 선택적 point+line. 기존 orbline이 있으면 덮어쓰지 않음
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m experiments.handheld_mapping.compare /absolute/path/to/RUN
```

TSDF만 다시 만들려면 GUI **기록에서 다시 생성**을 사용한다. 이는 ORB 재추정 버튼이 아니다. sensors-only RUN의 backend를 명시할 때:

```bash
PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python - /absolute/path/to/RUN <<'PY'
from pathlib import Path
import sys
from experiments.handheld_mapping.reconstruct import reconstruct
print(reconstruct(Path(sys.argv[1]), backend="orb_offline"))
PY
```

TSDF는 해당 dense 출력을 갱신하고 raw는 보존한다. `reconstruct` CLI의 default는 `orb`이며 backend flag는 없으므로 위 API 또는 GUI를 사용한다. point+line의 정확도 이득은 아직 입증하지 않았다.

## 8. 지형·costmap 생성

```bash
# 합성 예제. 새 output 필요
bash tools/terrain_mapping.sh --demo --output outputs/terrain_demo_02

# 실제 기록: 검증한 z-up 정렬과 map ID 필요
bash tools/terrain_mapping.sh /absolute/path/to/RUN \
  --backend orb_offline --map-id 0 \
  --alignment /absolute/path/to/grid_from_world.json \
  --bounds -5 5 -5 5 --resolution 0.05 --stride 4 \
  --output outputs/terrain_real_01
```

alignment는 ORB world→z-up gravity frame의 4×4 rigid transform JSON 배열이다. 자동 중력 검출은 없으며 identity나 다른 RUN의 변환을 임의 사용하지 않는다. 현재 실제 지면 분할·지형 정확도·로버 통과 한계 검증은 남아 있다.

결과는 `terrain.npz`, `summary.json`, `preview.png`다. NPZ는 `np.load(path, allow_pickle=False)`로 읽고 배열 축은 `(y,x)`다. 높이/단차/거칠기는 m, 경사/방향은 degree다. cost는 유효0..252, blocked254, unknown255이고 unknown을 평지로 채우지 않는다.

prototype 기본 한계는 경사20°/단차0.06m/거칠기0.025m/원형여유0.15m이며 실제 로버 한계로 확정하지 않았다. `--rover-limits FILE.json`으로 조정한다. coverage와 별개 offline 도구이며 live costmap/Nav2/슬립·토질 추정은 미연결이다.

## 9. 문제 해결·자료 이전

| 증상 | 먼저 확인 |
|---|---|
| hostname 조회 실패 | 현재 Pi IP·네트워크·SSH 인증. GUI host 수정 |
| D435 미검출/USB -71 | 케이블·접촉·포트·전원·USB 속도·camera/kernel 로그 |
| 이동 중 끊김/drop | source/receipt gap → queue/compress/write → 노트북 PNG 시간 |
| 특징점은 많은데 손실 | keypoint 수와 map-point inlier는 다름. 공백/blur/시야 변화 확인 |
| 지도 기울어짐 | IMU 융합·자동 중력 정렬 없음. alignment 검증 |
| 지도 두 창 | map 분리. 각 map 좌표계를 임의 합치지 않음 |
| Open3D 창 오류 | wrapper와 로컬 DISPLAY/X11 사용 |
| FileExistsError | 기존 결과 보호. 새 사본/output 사용 |
| 조종 명령과 STM 적용 불일치 | UDP/SPI/CRC/watchdog/조향 gate 확인 |

새 계측은 `session.json.capture_metrics`, `laptop_capture`, `frames.jsonl`에 남는다. Pi/laptop monotonic 절대값을 서로 빼지 않는다. 최종 drop은 후처리 실패에도 보존된다. 자세한 판독은 [입력 공백 조사](experiments/2026-10-14-input-gap-investigation.md)를 따른다.

RUN을 옮길 때 raw/보정/index/최종pose/summary/PLY/log를 함께 보존한다. native `pairs.tsv`의 절대경로는 새 위치에 맞춰 재생성한다. 이전 실기 RUN과 native 자산은 Git에 없으며 [별도 자산 안내](data-assets.md)에 정리돼 있다.

## 10. 개발 검증과 다음 순서

```bash
KSRC_REVIEW_ROOT="$PWD/teleop/ksrc_snapshot" \
QT_QPA_PLATFORM=offscreen PYTHONNOUSERSITE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 \
  .venv-mapping/bin/python -m pytest -q
```

fake SPI/합성 테스트이며 실제 모터를 움직이지 않는다. optional native 실행기가 없으면 해당 load test는 skip된다. source snapshot과 공유본 테스트 결과는 [공유본 검증](share-validation.md)을 본다.

다음 작업은 **입력 연속성 계측 → IMU/encoder 기록·시각/장착 검증 → 실측 지도 품질 → 중력/지형 검증 → 고정 스캔/STL을 사용한 별도 주행 보정** 순서다. COLMAP은 offline 비교 후보이며 아직 통합하지 않았다.
