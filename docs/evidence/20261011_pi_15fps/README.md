이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# Pi USB 2.0 / 15 FPS handheld 시험 — 2026-10-11

사용자가 D435를 Pi로 옮겨 USB 3.0 케이블 도착 전 낮은 FPS로 시험하도록 요청했다. Pi 5 native Jazzy/system Python 3.12, conda unset, ROS_DOMAIN_ID=67에서 카메라와 ORB, 특징선 시각화를 실행했다. Hailo-8 PCIe 장치는 인식되지만 현재 ORB/OpenCV 코드는 NPU를 사용하지 않는다. 로버 모터/STM/장착 변환/외부 IMU는 사용하지 않았다.

## 설정과 초기 결과

- D435 serial 027322073612, USB 480 Mbps, 좌우 IR 640×480 Y8 15 FPS. 현재 장치 profile enumeration에서 양쪽 15 FPS 지원을 확인했다. color/depth off, emitter 0, auto exposure on.
- 카메라 30초 probe: 좌우 각각 449장, receipt 14.965 Hz, 공통 source stamp 449쌍, nearest delta 최대 0 ns. 순서 역전/profile 변경 0.
- 기존 probe의 `gaps_over_50ms=448`은 15 FPS의 정상 간격 약 66.7 ms도 포함하는 고정 임계값 지표다. 이 값 자체를 프레임 누락으로 해석하지 않는다. 후속 `audit_live.py`는 실제 최대 간격과 100 ms 초과 횟수를 별도로 기록한다.
- 이 probe의 실제 CameraInfo/baseline과 `640x480x15:Y8`로 `/home/pi/handheld_mapping/20261011_15fps/orb01`을 준비했다. `Camera.fps=15`, camera-only/stereo-only, use_sim_time=false.
- 특징선은 Pi에서 OpenCV LSD로 검출하고 40 px 이상을 주황색으로 그려 `/d435/line_features`로 발행했다. 선을 SLAM 추정에 사용하는 구현은 아니다. 초기 5초 구간은 약 15 Hz, 검출 처리 sample 약 17–24 ms였다.
- Pi의 기존 web_video_server를 localhost:8080에서 사용한다. 최초 실행의 boost library 경로 누락은 기존 추출 prefix의 aarch64 library 경로를 추가해 해결했다. 추가 package 설치는 없다.
- 노트북 SSH 터널 localhost:8091 → Pi localhost:8080과 ffplay 네이티브 창을 실행했다. 실제 Pi 특징선 snapshot은 `line_snapshot.jpg`다.

## 중단 및 원인 미확정

ORB/특징선/화면 동시 실행 후 15초 audit에서 좌우 영상 각각 61장, source 약 5.5 Hz, 최대 source 간격 약 4667 ms였다. 선 영상 67장, camera pose 78장, tracking state OK 77 / recently lost 1 / lost 2였다. 마지막 diagnostics의 영상 age 약 2.88초. 따라서 안정적인 15 FPS 동시 실행 성공으로 간주하지 않는다.

17:39:12 KST 커널 `Undervoltage detected!`, 17:39:14 `Voltage normalised`. 이후 `get_throttled=0x50000`은 저전압 및 throttling 발생 이력이며 조회 시점 현재 저전압을 뜻하지 않는다.

17:40:42–17:40:53에 USB disconnect, descriptor/address `-71`, 반복 재인식 및 `unable to enumerate USB device`를 확인했다. 이후 USB 목록에서 D435가 사라졌고 특징선 publisher는 0 FPS였다. 연산 부하만으로 설명할 수 없는 실제 USB 연결 손실이다. 케이블/포트/전원 중 단일 원인은 아직 확정하지 않았다.

ORB는 SIGINT로 종료하고 Atlas 저장 완료 및 tracking.json `saved`를 확인했다. Atlas는 Pi에 약 520 KiB로 남아 있다. report의 전체 처리 1338쌍 중 pose 1335개는 전체 실행 카운터이며, 최종 작은 map의 안정적인 장시간 주행/재사용/정확도 성공을 뜻하지 않는다. 원본 rosbag 녹화는 시작하지 않았다.

고정된 마지막 화면을 실시간으로 오해하지 않도록 노트북 ffplay 창은 종료했다. Pi의 camera/line/web 프로세스와 노트북 SSH 터널은 재연결을 기다리는 상태다. 사용자는 기타 어댑터/USB 전원을 사용한다고 답했으며 정확한 OUTPUT 표기와 다른 USB 포트 재연결 결과를 요청했다. 15 FPS보다 더 낮추거나 전원 제한 설정을 강제로 바꾸지는 않았다.

사용자는 이후 PD 충전기로 Pi를 공급한다고 답했다. 정확한 5V 출력 전류는 아직 모른다. Pi 조회의 `usb_max_current_enable=0`은 USB 저전류 한도 모드이며, 공식 문서상 해당 USB 예산은 전체 600 mA다. 충전기의 최대 W 표기만으로 5V 5A 공급을 확인할 수 없다. 이 설정만으로 정확한 충전기 정격이나 USB 장애의 단일 원인을 확정하지 않는다.

## 재개

- Pi scripts: `/home/pi/handheld_mapping/20261011_15fps/`에 이 폴더의 shell/Python 실행 파일 사본이 있다.
- Pi tmux: `d435_15_camera`, `d435_15_lines`, `d435_15_web`. `d435_15_orb`는 저장 후 종료됐다.
- 새 ORB 실행 시 이미 시작된 orb01을 재사용하지 않고 새 probe와 새 run 디렉터리를 준비한다.
- 노트북의 이전 `d435_laptop_orb` PID 1140546은 별도 종료 지연 상태로 남아 있으며 Pi 시험과 domain이 다르다. 그 노트북 run의 지도 저장 성공은 확인하지 않았다.

참고: [AI HAT의 신경망 추론 범위](https://www.raspberrypi.com/documentation/computers/ai.html), [Pi USB 전원 예산](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html), [get_throttled 비트 의미](https://www.raspberrypi.com/documentation/computers/os.html#get_throttled).
