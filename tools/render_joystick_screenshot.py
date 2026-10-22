# ================================================================
# render_joystick_screenshot.py
# 설명: 조이스틱 GUI를 화면 없이(SDL dummy) 실행하고 가짜 gateway 상태를 보낸 뒤, 번호 주석을 덧그려 CLI.md용 PNG로 저장한다.
# 사용법:
#   python tools/render_joystick_screenshot.py docs/images/joystick_gui.png
# ================================================================
import json
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'teleop/ksrc_snapshot/pi/teleop'))

import pygame  # noqa: E402
import gateway_client  # noqa: E402
import teleop_joystick  # noqa: E402

STATUS = {'mode': 'TELEOP', 'run_id': 0, 'run': 0, 'missing': [], 'last_reject': '', 'link_ok': True, 'stationary_s': 5.2, 'anchor_count': 0}
MARKS = {1: (60, 118), 2: (580, 118), 3: (610, 40), 4: (330, 361), 5: (628, 374), 6: (610, 470)}   # 번호 -> 말풍선 위치(글자와 안 겹치는 빈 곳)


def main(output: Path) -> None:
    gateway = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gateway.bind(('127.0.0.1', 0))
    gateway.settimeout(.05)
    args = SimpleNamespace(udp='127.0.0.1:%d' % gateway.getsockname()[1], gateway=True, grid_keys=True, key_speed=.5, rate=50.,
                           max_lin=.15, max_ang=.8, deadzone=.1, monitor=False, ros=False, spi=False)
    threading.Thread(target=teleop_joystick.run, args=(args,), daemon=True).start()
    stop = threading.Event()

    def fake_gateway():
        while not stop.is_set():
            try:
                data, address = gateway.recvfrom(256)
            except socket.timeout:
                continue
            if len(data) == struct.calcsize('<4sIB3f') and struct.unpack('<4sIB3f', data)[2] == gateway_client.ANCHOR:
                STATUS['anchor_count'] += 1   # 실제 gateway처럼 ANCHOR 패킷을 받으면 runner의 anchor_count가 늘어난다
            gateway.sendto(json.dumps(STATUS).encode(), address)

    threading.Thread(target=fake_gateway, daemon=True).start()
    time.sleep(1.)
    left = teleop_joystick.LEFT_CENTER
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=pygame.Rect(*teleop_joystick.GW_BUTTONS['anchor']).center, button=1))
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, pos=(0, 0), button=1))   # ANCHOR만 누르고 패드는 놓는다
    time.sleep(.4)
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=(left[0] + 35, left[1] - 70), button=1))
    time.sleep(.5)
    shot = pygame.display.get_surface().copy()
    stop.set()
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    font = pygame.font.SysFont(None, 22)
    for number, center in MARKS.items():
        pygame.draw.circle(shot, (255, 190, 60), center, 11)
        label = font.render(str(number), True, (20, 20, 20))
        shot.blit(label, (center[0] - label.get_width() // 2, center[1] - label.get_height() // 2))
    output.parent.mkdir(parents=True, exist_ok=True)
    pygame.image.save(shot, str(output))
    print('saved', output, shot.get_size())


if __name__ == '__main__':
    main(Path(sys.argv[1]))
