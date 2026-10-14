# 보정 IMU와 D435 동시 기록 준비

사용자 요청: 조이스틱을 유지하면서 보정된 외부 IMU를 매핑 앱에 연결해 다음 촬영을 준비한다.

1. Pi의 단일 SPI owner인 joystick bridge에 기존 비차단 telemetry tee를 적용한다. 원본 KSRC 파일은 보존하고 RAM 사본으로 실행한다. fake에서 명령·정지 경로 동등성을 확인한다.
2. 매핑 앱은 별도 SSH listener로 IMU 원본·보정값을 같은 노트북 RUN에 저장한다. Pi에는 원본을 저장하지 않는다. 시작 전 유효 IMU 확인, 오류·중복·오래된 시각·종료 처리를 검증한다.
3. 화면에 실제 IMU 수신 상태와 값, 보정/URDF provenance를 표시한다. 현재 ORB stereo와 IMU 동시 기록을 명확히 구분하며 clock/fusion 검증값을 자동으로 true로 만들지 않는다.
4. 전체 매핑 검사와 실제 IMU 연결 시험 후 앱을 띄운다. D435 연결 후 짧은 촬영·종료·원본/IMU 저장까지 확인하고 사용자 촬영을 준비한다.

검증 기준: 조종 명령·watchdog 동등, SPI owner 하나, IMU 데이터가 영상 queue를 막지 않음, 같은 RUN에 imu.jsonl·보정/URDF 사본·hash 저장, 보정 축 중복 적용 없음, demo/기존 non-IMU 기록 유지.
