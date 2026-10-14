이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# 노트북 D435 실시간 시험 — 2026-10-11

사용자 요청으로 D435 연결을 Pi에서 노트북으로 전환했다. 현재 영상 수집과 ORB 연산은 모두 노트북에서 수행한다.

- 실제 기기: D435 serial `027322073612`, USB 480 Mbps. USB 3 SuperSpeed로 인식된 것은 아니다.
- 환경: 노트북 native ROS 2 Humble, `/usr/bin/python3` 3.10, conda 없음, `ROS_DOMAIN_ID=66`.
- 카메라: 좌우 IR 640×480 Y8 30 Hz, emitter 0, auto exposure. color/depth/IMU 비활성화.
- 15초 입력 probe: left 451 / right 450, 각각 약 30.01 / 29.94 Hz. 공통 source stamp 450쌍, nearest delta 최대 0 ns. 각 stream의 역전·50 ms 초과 source 간격·profile 변경 0. 이 짧은 구간의 결과이며 장시간 이동 안정성 검증이 아니다.
- `orb01/`은 이 probe의 실제 rectified CameraInfo와 baseline으로 준비했다. camera-only, stereo-only, `use_sim_time=false`. 로버 장착 변환을 적용하지 않는다.
- 첫 시험은 기존 노트북 native ORB 실행 파일과 vocabulary를 사용했다. 이후 특징점 영상의 5 FPS 제한을 제거하고 native ORB 실행 파일을 다시 빌드했다. 추적 알고리즘 변경은 없다.
- 브라우저에서 좌우 영상과 초록색 ORB 추적 특징점 표시를 확인했다. 초기 천장/사람을 관측하는 구간에 추적 손실과 map 전환이 있었으므로 전체 실행이 안정적이라고 표시하지 않는다.
- 이후 9개 diagnostics sample의 약 8초 구간에서 처리 프레임 3093→3333, pose 발행 2202→2442, 관측 상태 `2`(OK). 구간 내 처리율 약 30 Hz, 두 끝 sample의 처리 시간 약 14.77 / 13.61 ms, 마지막 유효 pose age 약 48 / 51 ms. 전체 프레임의 시간 분포나 웹 화면 end-to-end latency가 아니다.
- 화면 서버는 공식 ROS deb 두 개를 `runtime/prefix`로 추출해 실행한다. 시스템 package 설치 상태는 변경하지 않았다. 첫 실행의 package resource 검색 실패는 `AMENT_PREFIX_PATH`를 해당 prefix로 설정해 해결했다.
- Pi 정상 종료를 시도했지만 `raspberrypi.local` DNS 조회 실패로 명령 전달/정상 종료는 확인하지 못했다. Pi에서 live 데이터/Atlas가 정상 저장됐다고 주장하지 않는다.

화면: <http://127.0.0.1:8090/>. 영상 API는 localhost:8087이며, 컴퓨터와 Pi 사이의 영상 터널은 사용하지 않는다.

실행 중인 tmux session:

- `d435_laptop_camera`: 카메라 드라이버, `camera.log`.
- `d435_laptop_orb`: 현재 카메라 추적, `orb03/orb.log`.
- `d435_laptop_native`: ROS 직접 구독 OpenCV 창, `native_viewer.py`, `native_viewer.log`. Esc 또는 창 닫기로 뷰어만 종료한다.
- `d435_laptop_lines`: 원본 좌측 IR의 특징선 전용 창, `line_viewer.log`. `native_viewer.py --ros-args -r __node:=line_feature_viewer -p show_lines:=true`로 실행한다.
- `d435_laptop_web`: 영상 서버, `web.log`.
- `d435_laptop_page`: 로컬 화면 페이지, `page.log`.

ORB 결과를 저장하고 끝내려면 `tmux send-keys -t d435_laptop_orb C-c`로 정상 종료한 뒤 Atlas/report 저장 로그를 확인한다. 현재 `orb03`은 실행 중이므로 Atlas 최종 저장·장시간 handheld 이동·재사용은 아직 검증하지 않았다. 원본 영상 bag 녹화는 이번 노트북 live run에서 시작하지 않았다.

## 네이티브 30 FPS 표시

- ORB 특징점 영상은 처리한 각 stereo frame마다 발행한다. 추적 손실 시 영상은 계속 갱신하고 유효하지 않은 특징점은 그리지 않는다.
- `rqt_image_view`는 설치된 image transport 플러그인의 `subscribeImpl` 오류로 구독하지 못했다. 대신 system Python/OpenCV 창에서 `/orbslam3/tracked_image`를 sensor QoS로 직접 구독한다. NumPy 2와 기존 Python `cv_bridge`의 ABI 충돌을 피하도록 이미 BGR8인 ROS 데이터를 직접 배열로 표시한다. 시스템 패키지 변경은 없다.
- `orb03` 시작 후 12초 관측의 실제 영상 수신 구간: 235 frames, arrival 30.013 FPS, source 30.029 FPS. 관측 tracking state 236개 모두 `2`(OK), 마지막 처리 23.36 ms, camera-only. 상세 결과는 `native_fps_check.json`.
- 뷰어 자체의 시작 완료 후 연속 5초 구간 로그: 수신/표시 호출 각각 30.20, 29.97 FPS. 모니터의 물리적 refresh나 end-to-end latency 측정은 아니다.
- 재시작 전 `orb01`에 SIGINT를 전달했으나 최종 Atlas/report 파일은 확인되지 않았다. `orb02`는 Atlas 58 MB를 저장했지만 종료 시 유효한 최종 map 검증 실패로 report가 `incomplete`다. 어느 시험도 완전 저장/재사용 성공으로 간주하지 않는다.

## 특징선 표시

원본 `/camera/camera/infra1/image_rect_raw`에서 OpenCV LSD로 선분을 검출하고 40 px 이상인 선을 주황색으로 표시한다. ORB 점 특징 창과 별도로 실행되며 위치 추정에는 선을 사용하지 않는다. 실제 첫 두 5초 구간 수신/표시 호출은 각각 29.77 / 30.19 FPS, 마지막 frame의 표시 선은 60 / 59개였다. 검출 결과 예시는 `line_features_preview.jpg`다. 모니터 refresh나 선 기반 SLAM 성능을 측정한 결과는 아니다.

## Pi 15 FPS 시험으로 전환 준비

2026-10-11 노트북의 카메라, 특징선 뷰어, 웹 서버와 페이지 서버를 종료했다. 점 특징 뷰어는 이미 종료된 상태였다. USB 목록에서 D435는 보이지 않았다. `orb03`은 SIGINT 수신 후에도 연산을 계속하며 Atlas/report를 생성하지 않았다. 지도 메모리를 보존하고 CPU 사용을 멈추기 위해 PID `1140546`과 그 process group에 SIGSTOP을 전달했으나 후속 조회에서도 `Rsl+` 상태여서 일시 정지를 확인하지 못했다. 해당 프로세스는 정상 종료/저장 성공 상태가 아니며 `d435_laptop_orb` 세션 안에 남아 있다. 후속 조사 전 PID와 명령을 다시 확인한다.

다음 목표는 Pi의 좌우 IR 640×480 Y8 15 FPS 시험이다. Pi 연결 및 지원 profile 확인 전에는 적용하지 않았다. 위 실행 중 세션 목록과 `orb03` 실행 중 설명은 전환 전 이력이다.
