이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# Pi/D435 실제 sensors-only 연결 검사

2026-10-13 19:34 KST. Pi raspberrypi.local(172.20.10.11), Ubuntu24.04.5/aarch64, native Jazzy/Python3.12.3. 노트북은 .venv-mapping/Python3.10.12.

- D435 USB2(480Mbps), IR좌/우+depth640×480@15FPS, projector OFF 파라미터 별도 검증.
- 준비 후 약8초 수집. 좌/우/depth 각122프레임을 실제 SSH로 노트북 저장.
- source15.009FPS, 수신 약15.17~15.19FPS. 각 stream 최대 source 간격0.066632초.
- worker exit0, finished=saved, 전송 큐 누락0. 큐 최대922310B(약0.88MiB), 설정 상한4MiB.
- depth nonzero 평균82.83%. 유효한 거리 정확도를 검증한 수치는 아니다.
- Pi SD 여유약16GB, RAM available약6.5GiB, /dev/shm약3.9GiB 여유. 검사 전후 throttled=0x0.
- 카메라/worker를 정상 종료했다. 조이스틱·STM·모터 구동, 이동 SLAM·실기 TSDF/costmap 정확도는 이번 검사 대상이 아니다.

첫 검사 `../20261013_193129_connection_check`는 선택 파라미터 depth_module.depth_units가 없는 드라이버에서 GetParameters가 빈 응답을 반환해 실패했다. 필수 emitter 검증과 선택 units 조회를 분리하고, 선택 값은 nullable metadata로 유지하도록 수정했다. projector 상태를 확인하지 못하거나 켜져 있으면 계속 거부한다. 회귀6개 RED→GREEN, 관련 전체132 passed.

`connection_check.json`, `capture_calibration.json`, `frames.jsonl`, 원본 `raw/`, `events.jsonl`, `worker.log`, 복사된 `orb/camera.log`를 보존한다. 실제 센서 기록이며 synthetic 자료와 구분한다. 센서 단위 metadata는 null이지만 ROS Z16 단위의 기존 wrapper mm 규약은 변경하지 않았다.
