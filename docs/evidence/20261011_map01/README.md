이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# D435 handheld 간이 지도 — 2026-10-11

사용자 요청: Pi에서 간이 지도를 만들되 저장 공간을 아끼도록 결과는 노트북에 저장한다. 기존 stereo-only ORB와 저장 명령을 재사용한다. 모터, 로버 장착 변환, 외부 IMU는 사용하지 않는다.

## 시험 구성

- 카메라: Pi의 기존 좌우 IR 640×480 Y8 15 FPS. 특징선 화면은 유지한다. 선은 LSD 표시이고 지도 추정은 ORB 점 특징이다.
- 새 지도: Pi `/dev/shm/d435_handheld_map01`. `tmpfs` RAM 경로이며 새 시험의 Atlas, 로그, 보정, 녹화 파일을 SD 카드에 저장하지 않는다.
- Pi tmux: `d435_map_orb`, `d435_map_record`. 사용자 종료 요청 후 두 세션 모두 정상 종료됐다.
- 노트북 tmux: `d435_map_sync`. 실행 중 `sync.sh`가 5초 단위로 닫힌 MCAP 조각과 설정을 이 폴더로 가져왔으며 최종 복사 전에 종료했다. 시작 직후 RAM의 live 폴더 생성 전 발생한 find/rsync 오류는 `sync.log`에 남아 있으며 이후 실제 복사는 진행됐다.
- `live/`: 지도 점군, keyframes, path, camera_pose, tracking_state, diagnostics 기록. 원본 영상은 녹화하지 않는다.
- `run/`: Pi RAM run의 설정, 로그 및 정상 종료 후 저장된 Atlas/PLY/TUM 최종 파일 사본.

노트북과 Pi 사이 ROS discovery 수신은 확인되지 않아 SSH 복사를 사용했다. 연결 단절 또는 Pi 전원 차단 시 아직 노트북에 복사되지 않은 RAM 자료는 손실될 수 있다. RAM 조각은 이번 짧은 시험 동안 유지하며 자동 삭제하지 않는다.

## 시작 검증

노트북의 닫힌 첫 MCAP 파일이 실제로 생성됐고 Pi 원본과 SHA-256이 같았다:

`3c2c3f011aaca45490301ed7fb077d8fd8261d1413c561f50dac7901b327e057`

Pi의 `ros2 bag info -s mcap`으로 첫 조각을 열어 지도·경로 토픽 기록을 확인했다. 노트북 Humble의 MCAP plugin 및 native Python `mcap` 패키지는 없으므로 노트북에서 MCAP 전체 역직렬화 검증을 수행한 것은 아니다.

첫 15초 audit(`run/live_audit.json`)은 IR 약 14.86 FPS, 선 약 14.89 FPS, pose 약 11.05 FPS였다. tracking state는 OK(2) 157개, RECENTLY_LOST(3) 55개, LOST(4) 1개였다. 마지막 sample은 OK이며 지도 점 551개, keyframe 31개였다. 입력 영상 간격이 100 ms를 넘은 경우는 2회(최대 약 133 ms)였고 pose 최대 공백은 약 3.2초였다. 연속된 단일 지도나 정확한 이동 거리를 검증한 결과가 아니다. 지도 reset으로 frame이 달라진 조각을 임의로 합치지 않는다.

## 사용자 이동과 마무리

벽 모서리·가구가 보이게 정면을 향하고 천천히 1~2 m 이동한 뒤 시작 지점으로 돌아온다. 사용자가 “끝”이라고 하면:

1. Pi의 `d435_map_orb`에 SIGINT를 보내 정상 종료와 `tracking.json` 상태를 확인한다. Atlas 저장에는 시간이 걸릴 수 있다.
2. Pi의 `d435_map_record`에 SIGINT를 보내 MCAP/metadata를 닫는다.
3. RAM run 전체를 노트북 `run/`으로, live bag 전체를 `live/`로 복사한다. 마지막 열린 조각과 `metadata.yaml`도 이때 포함한다.
4. 노트북 사본의 Atlas, 지도 PLY, 카메라 TUM 경로와 tracking 상태를 검증한다. `d435_map_sync`를 종료한다. 확인 없이 Pi RAM 결과를 지우거나 Pi를 끄지 않는다.

기존 시험은 정상 종료됐고 `previous_run/`에 복사했다. 해당 지도는 새 이동 시험과 별개이며 기존 Pi 원본은 삭제하지 않았다. 카메라와 특징선 화면은 계속 실행한다.

## 종료 결과

사용자가 “끝”이라고 한 뒤 ORB와 recorder를 SIGINT로 정상 종료했다. `tracking.json`의 상태는 `saved`이며 Atlas binary 저장 완료 로그를 확인했다. 최종 bag 조각과 metadata를 포함해 노트북으로 모두 복사했다. `run/SHA256SUMS`의 18개 파일, `live/SHA256SUMS`의 MCAP 31개와 metadata가 Pi RAM 원본과 모두 일치했다.

- Atlas: `run/atlas.osa`, 약 25 MiB. 별도 실행에서의 Atlas 재사용은 미검증이다.
- 최종 지도: `run/map_7.ply`, 좌표계 `orb_map_7`, 점 3,734개, keyframe 111개.
- 최종 조각 카메라 궤적: `run/camera_map_7.tum`, pose 1,864개. 이전 두 조각은 `camera_map_0.tum`, `camera_map_3.tum`으로 각각 보존했다.
- ORB 처리: 2,267쌍, pose 2,156개. tracking OK 2,156 / RECENTLY_LOST 109 / LOST 2. 정상 추적 비율 약 95.1%이며 지도 정확도를 뜻하지 않는다.
- Pi native Jazzy `rosbag2_py`로 MCAP 31개 전체를 순회하여 5,390개 메시지를 역직렬화했다. 기록된 pose/state는 각각 2,155 / 2,266개로 ORB 처리 집계보다 1개씩 적다. 완전 무누락 기록으로 주장하지 않는다. 파일 사본은 SHA-256으로 확인했으며 payload CRC 전체 검증이나 GT 정확도 평가와 구분한다.
- 현재 시험 최종 자료: `run/` 약 25 MiB, `live/` 약 39 MiB. `previous_run/`은 별도 기존 시험 백업이다. 새 시험 자료의 Pi SD 저장은 없고 RAM 사본은 그대로 남겨 두었다.

`map_preview.png`는 최종 지도 조각만 시각화한다. 처음 카메라 좌표계의 X/Z 투영이며 바닥 평면이나 중력 정렬 좌표가 아니다. 원점에서 20 m 밖 점 55개를 그림에서만 제외했다(최대 반경 약 1,045 m인 튄 점 존재). 원본 PLY는 변경하지 않았다. 200 ms를 초과하는 카메라 경로 공백 1개는 선을 끊어 표시했다. 추적 reset으로 좌표계가 다른 조각을 임의로 이어 붙이지 않았다. 정밀 공간 지도 또는 검증된 재방문 지도라고 해석하지 않는다.

미리보기 생성은 native Python 3.10에서 `PYTHONNOUSERSITE=1`로 system numpy 1.21.5 / matplotlib 3.5.1을 사용했다. user site matplotlib과 system mplot3d의 혼합 import를 피했으며 패키지는 변경하지 않았다. 요약은 `map_summary.json`이다.
