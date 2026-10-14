# KSRC 소스 사본

기준: Pi `/home/pi/ksrc`의 확인된 `4bd158b1125cee32c5851f74ecc0c8f3d71013b1`.

`teleop_joystick.py`, `joystick_math.py`, `test_joystick_math.py`, `nucleo_link.py`는 실제 확인한 Pi 소스를 보존한 노트북 사본에서 가져왔다. `udp_spi_bridge.py`는 같은 commit의 고정 review 소스에서 가져왔다. 파일 내용과 기존 주석은 바꾸지 않았고 hash는 `docs/source-snapshot.json`에 있다.

전체 KSRC 프로젝트·STM firmware·sim/libksrc_common.so는 포함하지 않는다. 실제 Pi checkout/firmware/배선과 반드시 대조한다. bridge는 SPI를 직접 여는 실행기이므로 별도 owner와 동시에 띄우지 않는다. 사용법은 `docs/user-guide.md`의 조종 절을 따른다.
