# ================================================================
# test_joystick_keys.py
# 설명: 조이스틱 화면의 qweasdzxc 격자 키 → 병진 속도 변환(방향·대각선 속력·상쇄·정지)과 --grid-keys 창 기동을 검사한다.
# 사용법: python -m pytest tests/handheld_mapping/test_joystick_keys.py -q
# ================================================================
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
os.environ.setdefault('SDL_AUDIODRIVER', 'dummy')
sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'teleop/ksrc_snapshot/pi/teleop'))

from joystick_math import grid_keys_to_linear  # noqa: E402

MAX, SCALE = .15, .5


@pytest.mark.parametrize('keys, expected', [
    ({'w'}, (1, 0)), ({'x'}, (-1, 0)), ({'a'}, (0, 1)), ({'d'}, (0, -1)),
    ({'q'}, (1, 1)), ({'e'}, (1, -1)), ({'z'}, (-1, 1)), ({'c'}, (-1, -1)),
])
def test_each_key_points_in_its_compass_direction_at_the_same_speed(keys, expected):
    vx, vy = grid_keys_to_linear(keys, MAX, SCALE)
    assert math.hypot(vx, vy) == pytest.approx(MAX*SCALE)
    assert math.copysign(1, vx) == math.copysign(1, expected[0]) if expected[0] else vx == 0
    assert math.copysign(1, vy) == math.copysign(1, expected[1]) if expected[1] else vy == 0


def test_two_keys_combine_into_the_diagonal():
    assert grid_keys_to_linear({'w', 'a'}, MAX, SCALE) == pytest.approx(grid_keys_to_linear({'q'}, MAX, SCALE))


def test_opposite_keys_cancel_and_s_stops_and_unknown_keys_are_ignored():
    assert grid_keys_to_linear({'w', 'x'}, MAX, SCALE) == (0., 0.)
    assert grid_keys_to_linear({'a', 'd'}, MAX, SCALE) == (0., 0.)
    assert grid_keys_to_linear({'w', 's'}, MAX, SCALE) == (0., 0.)
    assert grid_keys_to_linear({'p', 'k'}, MAX, SCALE) == (0., 0.)
    assert grid_keys_to_linear(set(), MAX, SCALE) == (0., 0.)


def test_window_starts_and_quits_with_grid_keys():
    pygame = pytest.importorskip('pygame')
    import socket
    import teleop_joystick
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(('127.0.0.1', 0))
    args = SimpleNamespace(udp='127.0.0.1:%d' % sink.getsockname()[1], grid_keys=True, key_speed=.5, rate=50., max_lin=.15,
                           max_ang=.8, deadzone=0., monitor=False, ros=False, spi=False)
    thread = threading.Thread(target=teleop_joystick.run, args=(args,), daemon=True)
    thread.start()
    time.sleep(.5)
    assert thread.is_alive()
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    thread.join(2.)
    sink.close()
    assert not thread.is_alive()


@pytest.mark.parametrize('max_lin, normal, boosted', [(.15, .10, .15), (.30, .10, .15), (.09, .06, .09)])
def test_shift_temporarily_boosts_grid_speed_without_changing_rotation(monkeypatch, max_lin, normal, boosted):
    pygame = pytest.importorskip('pygame')
    import socket
    import teleop_joystick
    keys = FakeKeys()
    monkeypatch.setattr(pygame.key, 'get_pressed', lambda: keys)
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(('127.0.0.1', 0))
    args = SimpleNamespace(udp='127.0.0.1:%d' % sink.getsockname()[1], grid_keys=True,
                           rate=50., max_lin=max_lin, max_ang=.8, deadzone=0., monitor=False, ros=False, spi=False)
    thread = threading.Thread(target=teleop_joystick.run, args=(args,), daemon=True)
    thread.start()
    time.sleep(.3)
    try:
        keys.down = {pygame.K_w, pygame.K_RIGHT}
        assert read_command(sink) == pytest.approx((normal, 0., -.4), abs=1e-5)
        for shift in (pygame.K_LSHIFT, pygame.K_RSHIFT):
            keys.down = {pygame.K_w, pygame.K_RIGHT, shift}
            assert read_command(sink) == pytest.approx((boosted, 0., -.4), abs=1e-5)
            keys.down = {pygame.K_w, pygame.K_RIGHT}
            assert read_command(sink) == pytest.approx((normal, 0., -.4), abs=1e-5)
        keys.down = {pygame.K_q, pygame.K_LSHIFT}
        vx, vy, omega = read_command(sink)
        assert vx == pytest.approx(vy)
        assert math.hypot(vx, vy) == pytest.approx(boosted, abs=1e-5)
        assert omega == 0.
        for stop in (pygame.K_s, pygame.K_SPACE):
            keys.down = {pygame.K_w, pygame.K_LEFT, pygame.K_LSHIFT, stop}
            assert read_command(sink) == (0., 0., 0.)
        keys.down = {pygame.K_LSHIFT}
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(170, 90)))
        assert read_command(sink) == pytest.approx((max_lin, 0., 0.), abs=1e-5)
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=(170, 90)))
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=(570, 190)))
        assert read_command(sink) == pytest.approx((0., 0., -.4), abs=1e-5)
    finally:
        pygame.event.post(pygame.event.Event(pygame.QUIT))
        thread.join(2.)
        sink.close()
    assert not thread.is_alive()


class FakeKeys:
    """pygame.key.get_pressed()가 돌려주는 객체처럼 키 상수로 조회되는 가짜 키 상태."""

    def __init__(self):
        self.down = set()

    def __getitem__(self, key):
        return key in self.down


def read_command(sink, seconds=.3):
    """seconds 동안 받은 KSRC 명령 중 마지막 (vx, vy, w)."""
    import socket
    from experiments.handheld_mapping import control_gateway as cg
    sink.settimeout(.05)
    last, end = None, time.time()+seconds
    while time.time() < end:
        try:
            parsed = cg.parse_cmd(sink.recvfrom(64)[0])
        except socket.timeout:
            continue
        if parsed:
            last = parsed[1]
    return last


def test_held_keys_become_the_velocity_the_window_sends(monkeypatch):
    pygame = pytest.importorskip('pygame')
    import socket
    import teleop_joystick
    keys = FakeKeys()
    monkeypatch.setattr(pygame.key, 'get_pressed', lambda: keys)
    sink = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sink.bind(('127.0.0.1', 0))
    args = SimpleNamespace(udp='127.0.0.1:%d' % sink.getsockname()[1], grid_keys=True, key_speed=.5, rate=50., max_lin=.15,
                           max_ang=.8, deadzone=0., monitor=False, ros=False, spi=False)
    thread = threading.Thread(target=teleop_joystick.run, args=(args,), daemon=True)
    thread.start()
    time.sleep(.3)
    keys.down = {pygame.K_w}
    assert read_command(sink) == pytest.approx((.075, 0., 0.), abs=1e-5)
    keys.down = {pygame.K_a}
    assert read_command(sink) == pytest.approx((0., .075, 0.), abs=1e-5)
    keys.down = {pygame.K_q}
    vx, vy, _ = read_command(sink)
    assert vx == pytest.approx(vy) and math.hypot(vx, vy) == pytest.approx(.075, abs=1e-5)
    keys.down = {pygame.K_LEFT}
    assert read_command(sink) == pytest.approx((0., 0., .4), abs=1e-5)       # 회전은 max_ang의 50%(기존 규칙)
    keys.down = {pygame.K_RIGHT, pygame.K_w}
    assert read_command(sink) == pytest.approx((.075, 0., -.4), abs=1e-5)
    keys.down = {pygame.K_w, pygame.K_s}
    assert read_command(sink) == (0., 0., 0.)                                # s는 정지
    keys.down = {pygame.K_e}                                                 # 격자 모드에서 E는 회전이 아니라 전진-우측 대각선
    vx, vy, w = read_command(sink)
    assert vx > 0 > vy and w == 0.
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    thread.join(2.)
    sink.close()
