이 파일은 원본 실험의 검증 기록 사본이다. 당시 명령/절대경로와 현재 clone 환경을 구분한다. 촬영 원본·큰 실행 자산은 포함하지 않았으며 [별도 자산 안내](../../data-assets.md)를 따른다.

# 실제 로버 조종·매핑 준비 — 2026-10-13

사용자는 Pi/D435가 로버에 장착되고 STM·모터 전원도 켜졌으며 Pi↔STM은 GPIO SPI로 연결됐다고 확인했다.

## 확인한 상태

- Pi `/home/pi/ksrc` commit4bd158b, clean checkout. 기존 joystick → UDP5005 → SPI0.0/1MHz/50Hz → STM 경로를 유지한다.
- 확인 시 기존 SPI owner/UDP5005 listener/조종 프로세스 없음. Pi 계정은 dialout 그룹이며 spidev0.0 열기 성공. `/boot/firmware/config.txt`의 `dtparam=spi=on` 확인.
- 속도 명령을 보내지 않는 SPI POLL100회에서 valid0. 추가12개 raw 응답은 각각144바이트 모두0이었다. CRC·버전 decoder 문제로 단정할 근거가 없고 배선/STM 초기화·실행 상태 확인이 필요하다.
- STM RESET 후 재확인을 사용자에게 요청했다. 정상 텔레메트리 확인 전 실제 조종 bridge와 모터 속도 명령을 시작하지 않았다.
- 사용자가 RESET을 누른 뒤 재검사100회도 valid0/raw prefix 전부0. 100kHz/10kHz 저속 POLL 각20회도 valid0. STM 전원 LED 상태를 확인 요청했고 실제 주행은 대기 중이다.

## 준비한 노트북 구성

- Pi checkout의 원본 teleop_joystick.py/joystick_math.py/nucleo_link.py 등을 `ksrc_snapshot`으로 읽기 복사. 원격 checkout/firmware 수정 없음.
- `.venv-mapping`에 pygame2.6.1 wheel을 `--no-deps`로 설치. conda/ML 환경은 변경하지 않았다.
- 기존 joystick_math script 직접 실행은 ALL TESTS PASSED. pytest 함수형 테스트가 아니라 top-level self-check script다.
- SDL dummy display + localhost UDP listener로 기존 GUI 실행. 정지 입력30패킷/전체0/QUIT 후 정상 종료 확인(`loopback.json`). 실제 로버에는 보내지 않았다.
- 원본 SPI C/Python 대조 suite는 snapshot에 `sim/libksrc_common.so`가 없어 실행되지 않았다. 이를 protocol suite 통과로 표시하지 않는다.
- 노트북 기존 매핑 창을 실행해 수동 시작/종료 준비. 실제 촬영은 아직 시작하지 않았다.

## STM 응답 후 사용할 제한

```bash
# Pi: 기존 단일 SPI owner, 별도 monitor를 동시에 띄우지 않는다.
cd /home/pi/ksrc
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -u pi/common/udp_spi_bridge.py --hz 50 --max-lin 0.05 --max-ang 0.3

# 노트북: 실제 target으로 연결하기 전 STM 응답/정지 처리를 확인한다.
PYTHONNOUSERSITE=1 /home/su/rover/.venv-mapping/bin/python \
  /home/su/rover/outputs/handheld_mapping/20261013_rover_teleop_prepare/ksrc_snapshot/pi/teleop/teleop_joystick.py \
  --udp raspberrypi.local --rate 50 --max-lin 0.05 --max-ang 0.3 --monitor
```

사용자 입력으로만 이동한다. 최초 저속/정지 처리는 바퀴를 띄운 상태에서 확인한 뒤 바닥 주행으로 진행한다. 이 명령을 실행한 것과 실제 시험 완료를 구분한다.

## 현재 checkout의 배선 근거

`/home/pi/ksrc/firmware/CLI.md:212` 및 `firmware/src2026-rover-firmware/CLI.md:148`:

| Pi 물리 핀 | 신호 | Nucleo |
|---|---|---|
|24|CE0|PB12 / CN10-16|
|23|SCLK|PB13 / CN10-30|
|21|MISO|PC2 / CN7-35|
|19|MOSI|PC3 / CN7-37|
|25|GND|Nucleo GND 직결|

PB14/PB15는 현재 firmware에서 모터 TIM12에 사용되므로 일반적인 SPI2 예제 핀으로 혼동하지 않는다. 실제 장착 보드/배선이 이 checkout과 같은지 확인해야 한다.

## 배선 수정 후 실제 확인 결과

- 위의 전부 0 응답 기록은 배선 수정 전 상태다. 수정 후 POLL100회 중99회, 추가150회 중150회 유효 응답과 진행하는 STM 시각을 확인했다.
- 기존 UDP/SPI bridge를 단일 SPI owner로 실행했고 사용자 화면 joystick 조작에서 실제 명령/정지 응답을 받았다. 제한은 병진0.05m/s, 회전0.3rad/s. 에이전트는 비영 속도 명령을 생성하지 않았다.
- 촬영 중 D435가 USB 목록에서 사라져 joystick을 SIGINT로 종료했다. bridge는 입력 timeout 시 정지 상태를 유지한다. 최신 zero-only UDP 재검사22응답 모두 cmd/duty0, crc_bad0(`latest_zero_check.json`). latest flags153에는 GATE_TIMEOUT 비트가 있어 전체 구동계가 정상이라는 판정은 하지 않는다.
- D435 마지막 확인 시 lsusb에 없음, throttled0x0. camera.log는 No such device 반복. Pi 전원 지표만으로 USB 전원/접촉/케이블/간섭 원인을 단정할 수 없다.
- 부분 촬영 원본: `../sessions/20261013_201134_fd506c/`. 좌우 IR/depth 각각939장, 약62.5초, 전송 drop0. 카메라 오류로 session은 failed 상태를 유지한다.
- 보존된 부분 촬영을 노트북 ORB로 계산:939쌍, tracking OK100%, map0 하나, 최종18keyframe. 이는 GT 정확도나 Pi 처리 FPS 검증이 아니다.
- offline TSDF:18/18keyframe 융합, 최대 depth timestamp 차305ns, voxel1cm, mesh67473vertices/104996triangles. `dense_offline/map_0_mesh.ply`와 `map_0_cloud.ply` 저장 후 mesh 재열기 검증. 초기 camera optical 좌표이며 중력 정렬/정확도 검증 전이므로 아직 지형 경사/costmap 기준맵으로 확정하지 않는다.
- 모터/firmware 소스 변경 없음. GPIO 통신과 정지 검증 성공을 카메라 재연결 또는 조향 gate 문제 해결로 확대 해석하지 않는다.
