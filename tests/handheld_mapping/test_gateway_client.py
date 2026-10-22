# ================================================================
# test_gateway_client.py
# 설명: 조이스틱 화면의 중계 클라이언트(하트비트·비상·시작·상태 표시)와, 화면 없이(SDL dummy) 돌린 조이스틱 창이
#       버튼·키 입력을 control_gateway가 읽는 KCTL 패킷으로 보내는지 검사한다.
# 사용법: python -m pytest tests/handheld_mapping/test_gateway_client.py -q
# ================================================================
import math
import os
from pathlib import Path
import socket
import sys
import threading
import time
from types import SimpleNamespace

import pytest

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
TELEOP = Path(__file__).resolve().parents[2]/'teleop/ksrc_snapshot/pi/teleop'
sys.path.insert(0, str(TELEOP))

import gateway_client as gc  # noqa: E402
from experiments.handheld_mapping import control_gateway as cg  # noqa: E402


class Capture:
    """보낸 패킷을 모아 두는 가짜 소켓."""

    def __init__(self):
        self.packets = []

    def sendto(self, data, dest):
        self.packets.append((data, dest))


def test_control_packets_are_readable_by_the_gateway_and_seq_increases():
    sock = Capture()
    client = gc.GatewayClient(sock, ('pi', 5006))
    client.start(2)
    client.start(1)
    parsed = [cg.parse_ctl(data) for data, _ in sock.packets]
    assert [p[1] for p in parsed] == [cg.START, cg.START]
    assert parsed[0][2][0] == 2. and parsed[1][2][0] == 1.
    assert parsed[0][0] < parsed[1][0]


def test_estop_is_repeated():
    sock = Capture()
    gc.GatewayClient(sock, ('pi', 5006)).estop()
    assert [cg.parse_ctl(d)[1] for d, _ in sock.packets] == [cg.ESTOP]*gc.ESTOP_REPEAT


def test_anchor_packet_is_readable_by_the_gateway():
    sock = Capture()
    gc.GatewayClient(sock, ('pi', 5006)).anchor()
    assert [cg.parse_ctl(d)[1] for d, _ in sock.packets] == [cg.ANCHOR]


def test_heartbeat_is_sent_only_every_interval():
    sock = Capture()
    client = gc.GatewayClient(sock, ('pi', 5006), heartbeat_s=.2)
    for now in (0., .05, .1, .21, .3, .45):
        client.tick(now)
    assert len(sock.packets) == 3


def test_status_is_stored_but_stm_telemetry_is_not_consumed():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    assert not client.consume({'cmd': [0, 0, 0], 'flags': 0}, 1.)
    assert client.consume({'mode': 'TELEOP', 'missing': ['orb_ok'], 'link_ok': True}, 1.)
    texts = [t for t, _ in client.lines(1.1)]
    assert 'MODE TELEOP' in texts[0] and any('orb_ok' in t for t in texts)


def test_lines_show_anchor_applied_only_in_teleop():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    client.consume({'mode': 'TELEOP', 'missing': [], 'link_ok': True, 'anchored': True}, 1.)
    assert any('anchor applied' in t for t, _ in client.lines(1.1))
    client.consume({'mode': 'AUTO', 'run_id': 1, 'missing': [], 'link_ok': True, 'anchored': True}, 2.)
    assert not any('anchor applied' in t for t, _ in client.lines(2.1))


def test_lines_report_stale_status_auto_mode_and_rejection():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    assert client.lines(1.)[0][1] == 'bad'
    client.consume({'mode': 'AUTO', 'run_id': 2, 'missing': [], 'link_ok': False, 'last_reject': 'orb_ok'}, 1.)
    lines = client.lines(1.1)
    assert 'AUTO' in lines[0][0] and 'RUN 2' in lines[0][0]
    assert any('LOST' in t for t, _ in lines) and any('rejected' in t for t, _ in lines)
    assert client.lines(5.)[0][1] == 'bad'


@pytest.fixture
def window():
    """화면 없이 조이스틱 창을 스레드로 돌리고, 가짜 중계 소켓으로 나오는 패킷을 받는다."""
    pygame = pytest.importorskip('pygame')
    import teleop_joystick
    gateway = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gateway.bind(('127.0.0.1', 0))
    gateway.settimeout(.05)
    args = SimpleNamespace(udp='127.0.0.1:%d' % gateway.getsockname()[1], gateway=True, rate=50., max_lin=.15,
                           max_ang=.8, deadzone=0., monitor=False, ros=False, spi=False)
    thread = threading.Thread(target=teleop_joystick.run, args=(args,), daemon=True)
    thread.start()
    yield SimpleNamespace(pygame=pygame, module=teleop_joystick, gateway=gateway, thread=thread)
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    thread.join(2.)
    gateway.close()


def collect(gateway, seconds):
    """seconds 동안 받은 KCTL 패킷을 (동작, 값) 목록으로 돌려준다."""
    found, end = [], time.time()+seconds
    while time.time() < end:
        try:
            data, _ = gateway.recvfrom(256)
        except socket.timeout:
            continue
        parsed = cg.parse_ctl(data)
        if parsed:
            found.append((parsed[1], parsed[2]))
    return found


def post_click(window, name):
    pygame = window.pygame
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=pygame.Rect(*window.module.GW_BUTTONS[name]).center, button=1))


def test_window_sends_heartbeat_estop_and_start(window):
    pygame = window.pygame
    assert any(op == cg.HEARTBEAT for op, _ in collect(window.gateway, 1.))
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_SPACE, unicode=' ', mod=0))
    assert any(op == cg.ESTOP for op, _ in collect(window.gateway, .6))
    post_click(window, 'start1')
    starts = [args for op, args in collect(window.gateway, .6) if op == cg.START]
    assert starts and starts[0][0] == 1.
    post_click(window, 'start2')
    assert [args[0] for op, args in collect(window.gateway, .6) if op == cg.START][:1] == [2.]
    post_click(window, 'anchor')
    assert any(op == cg.ANCHOR for op, _ in collect(window.gateway, .6))


def test_window_without_gateway_flag_keeps_old_behaviour():
    pygame = pytest.importorskip('pygame')
    import teleop_joystick
    gateway = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gateway.bind(('127.0.0.1', 0))
    gateway.settimeout(.05)
    args = SimpleNamespace(udp='127.0.0.1:%d' % gateway.getsockname()[1], rate=50., max_lin=.15, max_ang=.8,
                           deadzone=0., monitor=False, ros=False, spi=False)
    thread = threading.Thread(target=teleop_joystick.run, args=(args,), daemon=True)
    thread.start()
    time.sleep(.5)
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_SPACE, unicode=' ', mod=0))
    controls = collect(gateway, .5)
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    thread.join(2.)
    gateway.close()
    assert controls == []                         # --gateway가 없으면 KCTL을 한 개도 보내지 않는다
