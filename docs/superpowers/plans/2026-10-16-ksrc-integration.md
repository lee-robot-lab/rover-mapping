# KSRC 호환 통합 계획

목표: 이미 main에 반영된 reference 브랜치는 중복 merge하지 않고, KSRC `07161349eb1cfd765d3efff1da43a5cbac57e24f`의 실제 변경과 mapping 접점을 통합한다. 장비·모터·firmware flash는 실행하지 않는다.

사용자 정정: 비교 대상 추가 변경은 `sy00107/rover-mapping` main `ff82f9954b2547616898a739cc3ee8fa30c1a6c8`이다. 공통 base `1e87ee5` 이후 depth filtering/ground/gravity/IMU display/atomic capture group/receiver queue 변경을 현재 RGB/global/editor 경로와 3-way merge하고 검사한다. 기존 실험용 IMU replay를 verified-per-sample 입력과 혼동하지 않도록 별도 이름으로 유지한다. 전역 poses에 중력 변환을 두 번 적용하지 않는 회귀가 필요하다.

## 확인한 설계

- 기존 144-byte/version2 명령·CRC·평균 IMU 값·20° URDF를 유지한다. upstream URDF와 joystick/bridge는 현재 사본과 동일하다.
- 최신 decoder의 residual/config 명령·PI flags·imu_n을 가져오되, flags bit12가 있으면 끝 2-byte는 timing으로 해석한다. timing frame의 `wpi_corr`는 None으로 표시하여 signed offset을 PI 설정으로 오해하지 않는다. IMU_VALID가 없으면 timing도 무효다.
- 최신 firmware patch는 같은 2-byte를 flag로 구분한다. flag 없는 frame에서는 wpi_corr를 유지한다. timing frame에는 두 값을 동시에 담을 수 없으며 PI readback은 미제공이다. command/config 동작은 유지한다. 구형 KSRC decoder와 timing firmware의 혼용은 금지한다.
- 기존 40499bc 전용 패치는 그대로 보존한다. 별도 latest baseline/hash/patch를 만들고 generator에 명시 profile/output을 추가한다. common과 board SPI 복사본을 함께 수정하여 빌드 소스가 어긋나지 않게 한다.
- RL student v3 VO 관측 입력은 존재하지만 실제 run_policy의 VO 전달은 연결돼 있지 않다. 별도 실시간 odometry 출력 계약 없이 RL 시스템 전체를 이번 매핑 저장소로 가져오지 않는다.

## 작업과 검사

1. 끝 필드 충돌, 유효 flag·CRC·명령 byte 유지 회귀를 먼저 실패시킨다.
2. 최신 nucleo_link와 기존 joystick/bridge 사본을 대조·통합하고 timing 해석을 추가한다.
3. 최신 baseline용 패치를 생성하고 C pack/parse↔Python 왕복, 구형 frame, signed 경계, IMU invalid, residual/config byte 동일성을 검사한다. 재생성 hash와 board/common 동일성도 검사한다.
4. 기존 telemetry tee 패치를 최신 bridge에 적용해 수신자 부재/포화에서도 명령·watchdog·종료 경로 동일성을 확인한다.
5. 전체 pytest·shell syntax·독립 리뷰 후 문서/출처와 함께 main에 합쳐 push한다. runtime 반영은 파일별 원본 비교·backup 후 수행하며 실행 중 센서는 재시작하지 않는다.

Python: `/home/su/rover/.venv-mapping/bin/python`; Conda를 사용하지 않는다. upstream은 읽기 전용 기준으로 두고 patch 검사는 임시 복사본에서만 수행한다. 실기 전에는 host/fake 검증을 우선하고 모터를 차단한 bench 검증을 권고한다.
