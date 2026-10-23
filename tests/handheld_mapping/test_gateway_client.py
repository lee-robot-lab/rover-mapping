# ================================================================
# test_gateway_client.py
# 설명: 조이스틱 화면의 중계 클라이언트(하트비트·비상·시작·상태 표시)와, 화면 없이(SDL dummy) 돌린 조이스틱 창이
#       버튼·키 입력을 control_gateway가 읽는 KCTL 패킷으로 보내는지 검사한다.
# 사용법: python -m pytest tests/handheld_mapping/test_gateway_client.py -q
# ================================================================
import json
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


@pytest.mark.parametrize('run', [1, 2, 3, 4])
def test_client_sends_each_supported_run(run):
    sock = Capture()
    gc.GatewayClient(sock, ('pi', 5006)).start(run)
    assert cg.parse_ctl(sock.packets[0][0])[2][0] == run


@pytest.mark.parametrize('run', [0, 5, 1.5])
def test_client_rejects_unsupported_runs_without_sending(run):
    sock = Capture()
    with pytest.raises(ValueError):
        gc.GatewayClient(sock, ('pi', 5006)).start(run)
    assert not sock.packets


def test_estop_is_repeated():
    sock = Capture()
    gc.GatewayClient(sock, ('pi', 5006)).estop()
    assert [cg.parse_ctl(d)[1] for d, _ in sock.packets] == [cg.ESTOP]*gc.ESTOP_REPEAT


def test_anchor_is_allowed_only_when_teleop_stationary_orb_ok_and_status_fresh():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    assert not client.anchor_allowed(0.)                                           # 상태를 못 받음
    client.consume({'mode': 'TELEOP', 'missing': ['stationary_s'], 'link_ok': True}, 1.)
    assert not client.anchor_allowed(1.1)
    client.consume({'mode': 'TELEOP', 'missing': ['orb_ok'], 'link_ok': True}, 2.)
    assert not client.anchor_allowed(2.1)
    client.consume({'mode': 'TELEOP', 'missing': ['status'], 'link_ok': True}, 3.)
    assert not client.anchor_allowed(3.1)
    client.consume({'mode': 'TELEOP', 'missing': ['in_region1'], 'link_ok': True}, 4.)
    assert client.anchor_allowed(4.1)                                              # 출발 구역 밖이어도 anchor는 가능
    assert not client.anchor_allowed(6.)                                           # 상태가 오래됨
    client.consume({'mode': 'AUTO', 'run': 1, 'missing': [], 'link_ok': True}, 7.)
    assert not client.anchor_allowed(7.1)


def test_orb_reset_packet_allowed_only_in_teleop_and_note_fades():
    sock = Capture()
    client = gc.GatewayClient(sock, ('pi', 5006))
    assert not client.orb_reset_allowed(0.)                                         # 상태를 못 받음
    client.consume({'mode': 'TELEOP', 'missing': ['stationary_s', 'orb_ok'], 'link_ok': True}, 1.)
    assert client.orb_reset_allowed(1.1)                                            # ORB가 안 되는 때가 바로 쓰는 때다
    client.orb_reset(1.1)
    assert [cg.parse_ctl(d)[1] for d, _ in sock.packets] == [cg.ORB_RESET]
    assert any('ORB reset requested' in t for t, _ in client.lines(1.2))
    client.consume({'mode': 'TELEOP', 'missing': [], 'link_ok': True}, 5.)
    assert not any('ORB reset requested' in t for t, _ in client.lines(5.1))
    client.consume({'mode': 'AUTO', 'run': 1, 'missing': [], 'link_ok': True}, 6.)
    assert not client.orb_reset_allowed(6.1)


def test_anchor_packet_is_readable_by_the_gateway():
    sock = Capture()
    gc.GatewayClient(sock, ('pi', 5006)).anchor(0.)
    assert [cg.parse_ctl(d)[1] for d, _ in sock.packets] == [cg.ANCHOR]


@pytest.mark.parametrize('preset, op', [(1, 5), (2, 7), (3, 8), (4, 9)])
def test_anchor_preset_selects_its_wire_operation(preset, op):
    sock = Capture()
    gc.GatewayClient(sock, ('pi', 5006)).anchor(0., preset=preset)
    assert [gc.CTL.unpack(data)[2:] for data, _ in sock.packets] == [(op, 0., 0., 0.)]


@pytest.mark.parametrize('preset', [0, 5, 1.5, '2', None])
def test_anchor_rejects_invalid_preset_without_sending(preset):
    sock = Capture()
    with pytest.raises(ValueError):
        gc.GatewayClient(sock, ('pi', 5006)).anchor(0., preset=preset)
    assert not sock.packets


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
    assert 'MODE TELEOP' in texts[0] and any('NOT READY' in t and 'ORB' in t for t in texts)


def teleop(**fields):
    return dict({'mode': 'TELEOP', 'missing': [], 'link_ok': True, 'run': 0, 'anchor_count': 0}, **fields)


def texts_of(client, now):
    return [(t, tone) for t, tone in client.lines(now)]


def test_ready_and_not_ready_reasons_are_spelled_out():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    client.consume(teleop(), 1.)
    assert ('READY: START allowed', 'ok') in texts_of(client, 1.1)
    client.consume(teleop(missing=['stationary_s', 'orb_ok', 'in_region1'], stationary_s=1.24), 2.)
    line = next(t for t, _ in texts_of(client, 2.1) if t.startswith('NOT READY'))
    assert 'hold still 1.2/3.0 s' in line and 'ORB not tracking' in line and 'press ANCHOR' in line
    client.consume(teleop(missing=['status']), 3.)
    assert any('no runner status' in t for t, _ in texts_of(client, 3.1))


def test_anchor_feedback_applied_then_cleared_when_auto_starts():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    client.consume(teleop(anchor_count=4), 1.)
    client.anchor(1.)
    assert ('ANCHOR sent...', 'warn') in texts_of(client, 1.1)
    client.consume(teleop(anchor_count=5), 1.2)
    assert ('ANCHOR applied (current pose = start preset)', 'ok') in texts_of(client, 1.3)
    client.consume({'mode': 'AUTO', 'run': 1, 'stage': 'entry30', 'missing': [], 'link_ok': True, 'anchor_count': 5}, 2.)
    assert 'RUN 1  entry30' in texts_of(client, 2.1)[0][0]
    client.consume(teleop(anchor_count=5), 3.)
    assert not any('ANCHOR' in t for t, _ in texts_of(client, 3.1))      # 이전 회차의 anchor 결과가 남지 않는다


@pytest.mark.parametrize('preset, label', [(1, 'ANCHOR1 -Y'), (2, 'ANCHOR2 -X'),
                                          (3, 'ANCHOR3 +Y'), (4, 'ANCHOR4 +X')])
def test_current_anchor_preset_is_visible_from_gateway_status(preset, label):
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    client.consume(teleop(anchor_count=4, anchor_id=preset), 1.)
    assert label in texts_of(client, 1.1)[0][0]
    client.anchor(1., preset=preset)
    client.consume(teleop(anchor_count=5, anchor_id=preset), 1.2)
    assert any(label in text and 'applied' in text and tone == 'ok'
               for text, tone in texts_of(client, 1.3))
    client.consume({'mode': 'AUTO', 'run': 1, 'anchor_id': preset}, 2.)
    assert label in texts_of(client, 2.1)[0][0]


def test_anchor_feedback_rejected_when_count_does_not_change():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    client.consume(teleop(anchor_count=1), 1.)
    client.anchor(1.)
    client.consume(teleop(anchor_count=1), 2.4)
    assert any(t.startswith('ANCHOR rejected') and tone == 'bad' for t, tone in texts_of(client, 2.6))


def test_anchor_feedback_says_it_cannot_confirm_for_an_old_stack():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    client.consume({'mode': 'TELEOP', 'missing': [], 'link_ok': True}, 1.)     # anchor_count 필드가 없는 옛 실행기
    client.anchor(1.)
    assert any('cannot confirm' in t for t, _ in texts_of(client, 1.1))


def test_previous_run_result_is_shown_in_teleop():
    client = gc.GatewayClient(Capture(), ('pi', 5006))
    client.consume(teleop(run=1, done=True, reason='complete'), 1.)
    assert ('RUN 1 COMPLETE - back to TELEOP', 'ok') in texts_of(client, 1.1)
    client.consume(teleop(run=1, done=True, reason='local_plan_blocked_timeout'), 2.)
    assert ('RUN 1 FAILED (local_plan_blocked_timeout) - back to TELEOP', 'bad') in texts_of(client, 2.1)
    client.consume(teleop(run=1, done=False, reason='operator'), 3.)
    assert ('RUN 1 stopped by operator', 'warn') in texts_of(client, 3.1)


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


def test_anchor_buttons_follow_map_axes_in_a_cross():
    import teleop_joystick as joystick
    boxes = joystick.GW_BUTTONS
    centers = {name: (x + width / 2, y + height / 2)
               for name, (x, y, width, height) in boxes.items()}
    assert centers['anchor4'][0] == centers['anchor2'][0]
    assert centers['anchor3'][1] == centers['anchor'][1]
    assert centers['anchor3'][0] < centers['anchor4'][0] < centers['anchor'][0]
    assert centers['anchor4'][1] < centers['anchor'][1] < centers['anchor2'][1]
    assert centers['start1'][1] > centers['anchor2'][1]
    assert joystick.GW_LABELS['anchor'] == 'ANCHOR1 -Y'
    assert joystick.GW_LABELS['anchor2'] == 'ANCHOR2 -X'
    assert joystick.GW_LABELS['anchor3'] == 'ANCHOR3 +Y'
    assert joystick.GW_LABELS['anchor4'] == 'ANCHOR4 +X'


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
    for run in (3, 4):
        post_click(window, f'start{run}')
        assert [args[0] for op, args in collect(window.gateway, .6) if op == cg.START][:1] == [float(run)]


def reply_status(window, **fields):
    """창이 보낸 패킷의 보낸 주소를 알아내 그쪽으로 gateway 상태를 돌려준다."""
    _, address = window.gateway.recvfrom(256)
    window.gateway.sendto(json.dumps(dict({'mode': 'TELEOP', 'missing': [], 'link_ok': True}, **fields)).encode(), address)


@pytest.mark.parametrize('name, op', [('anchor', 5), ('anchor2', 7), ('anchor3', 8), ('anchor4', 9)])
def test_anchor_button_is_ignored_until_stationary_and_orb_ok(window, name, op):
    reply_status(window, missing=['stationary_s', 'in_region1'], stationary_s=1.0)
    time.sleep(.3)
    post_click(window, name)
    assert not any(sent == op for sent, _ in collect(window.gateway, .5))      # 정지 3초 전에는 눌러도 보내지 않는다
    reply_status(window, missing=['in_region1'], stationary_s=3.4)                  # 출발 구역은 anchor 조건이 아니다
    time.sleep(.3)
    post_click(window, name)
    assert any(sent == op for sent, _ in collect(window.gateway, .6))
    reply_status(window, mode='AUTO', run=1)
    time.sleep(.3)
    post_click(window, name)
    assert not any(sent == op for sent, _ in collect(window.gateway, .5))


def test_orb_reset_button_sends_in_teleop_and_is_ignored_in_auto(window):
    reply_status(window)
    time.sleep(.3)
    post_click(window, 'orbreset')
    assert any(op == cg.ORB_RESET for op, _ in collect(window.gateway, .6))
    reply_status(window, mode='AUTO', run=1)
    time.sleep(.3)
    post_click(window, 'orbreset')
    assert not any(op == cg.ORB_RESET for op, _ in collect(window.gateway, .5))


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
