# ================================================================
# test_control_gateway.py
# 설명: 텔레옵↔자동 중계(control_gateway)의 모드 규칙, 시작 조건, 비상 정지, 정지 감지, 패킷 형식, 소켓 왕복(순서번호 연속·지연)을 검사한다.
# 사용법: python -m pytest tests/handheld_mapping/test_control_gateway.py -q
# ================================================================
import socket
import threading
import time

import pytest

from experiments.handheld_mapping import control_gateway as cg

READY = dict(stationary_s=3.5, orb_ok=True, in_region1=True)


class Sink:
    """중계가 run_policy로 보낼 명령을 모아 두는 시험용 수신기."""

    def __init__(self):
        self.sent = []

    def __call__(self, vx, vy, w):
        self.sent.append((vx, vy, w))


def make(**kwargs):
    sink = Sink()
    return cg.Gateway(sink, **kwargs), sink


def ready_gateway(now=10.):
    gateway, sink = make()
    gateway.runner_status(dict(READY), now)
    return gateway, sink


def test_teleop_command_is_forwarded_immediately():
    gateway, sink = make()
    gateway.teleop((.1, 0., .2), 1.)
    assert sink.sent == [(.1, 0., .2)]


def test_auto_command_is_dropped_in_teleop_and_teleop_dropped_in_auto():
    gateway, sink = ready_gateway()
    gateway.auto((.05, 0., 0.), 10.1)
    assert sink.sent == []
    assert gateway.control(cg.START, (1, 0, 0), 10.2)
    gateway.teleop((.1, 0., 0.), 10.3)
    assert sink.sent == []
    gateway.auto((.05, 0., 0.), 10.4)
    assert sink.sent == [(.05, 0., 0.)]


@pytest.mark.parametrize('missing', ['stationary_s', 'orb_ok', 'in_region1'])
def test_start_is_rejected_until_every_condition_holds(missing):
    gateway, _ = make()
    status = dict(READY)
    status[missing] = 0. if missing == 'stationary_s' else False
    gateway.runner_status(status, 10.)
    assert not gateway.control(cg.START, (1, 0, 0), 10.1)
    assert gateway.snapshot(10.1)['mode'] == 'TELEOP'
    assert missing in gateway.snapshot(10.1)['missing']


def test_start_is_rejected_when_runner_status_is_stale():
    gateway, _ = ready_gateway(10.)
    assert not gateway.control(cg.START, (1, 0, 0), 12.)


def test_start_calls_runner_with_run_id():
    started = []
    gateway, _ = make(on_start=started.append)
    gateway.runner_status(dict(READY), 10.)
    assert gateway.control(cg.START, (2, 0, 0), 10.1)
    assert started == [2]


def test_estop_returns_to_teleop_and_holds_zero_then_releases():
    stopped = []
    gateway, sink = make(on_stop=lambda: stopped.append(1))
    gateway.runner_status(dict(READY), 10.)
    gateway.control(cg.START, (1, 0, 0), 10.1)
    gateway.auto((.1, 0., 0.), 10.2)
    assert gateway.control(cg.ESTOP, (0, 0, 0), 10.3)
    assert sink.sent[-1] == (0., 0., 0.) and stopped == [1]
    assert gateway.snapshot(10.3)['mode'] == 'TELEOP'
    gateway.teleop((.1, 0., 0.), 10.4)           # 0.5 s 동안은 조이스틱도 무시하고 0을 유지한다
    assert sink.sent[-1] == (0., 0., 0.)
    gateway.teleop((.1, 0., 0.), 10.9)
    assert sink.sent[-1] == (.1, 0., 0.)


def test_keepalive_repeats_last_command_then_zero_after_stale():
    gateway, sink = make(stale_s=.3, keepalive_s=.02)
    gateway.teleop((.1, 0., 0.), 1.)
    gateway.tick(1.005)
    assert len(sink.sent) == 1                    # 20 ms 전에는 다시 보내지 않는다
    gateway.tick(1.03)
    assert sink.sent[-1] == (.1, 0., 0.) and len(sink.sent) == 2
    gateway.tick(1.31)
    assert sink.sent[-1] == (0., 0., 0.)


def test_auto_source_silence_stops_the_robot_but_keeps_auto_mode():
    gateway, sink = ready_gateway()
    gateway.control(cg.START, (1, 0, 0), 10.1)
    gateway.auto((.1, 0., 0.), 10.2)
    gateway.tick(10.6)
    assert sink.sent[-1] == (0., 0., 0.)
    assert gateway.snapshot(10.6)['mode'] == 'AUTO'


def test_heartbeat_loss_does_not_stop_auto():
    gateway, sink = ready_gateway()
    gateway.control(cg.HEARTBEAT, (0, 0, 0), 10.)
    gateway.control(cg.START, (1, 0, 0), 10.1)
    for step in range(100):                       # 하트비트 없이 5 s 동안 자동 명령이 계속 온다
        now = 10.2+step*.05
        gateway.auto((.1, 0., 0.), now)
        gateway.tick(now+.01)
    assert sink.sent[-1] == (.1, 0., 0.)
    assert not gateway.snapshot(15.)['link_ok']


def test_pose_is_forwarded_to_runner():
    poses = []
    gateway, _ = make(on_pose=lambda *p: poses.append(p))
    assert gateway.control(cg.POSE, (2.9, 1.2, 0.), 1.)
    assert poses == [(2.9, 1.2, 0.)]


def test_anchor_is_forwarded_to_runner_only_in_teleop_and_after_estop_hold():
    anchored = []
    gateway, _ = make(on_anchor=lambda: anchored.append(1))
    gateway.runner_status(dict(READY), 10.)
    assert gateway.control(cg.ANCHOR, (0, 0, 0), 10.1) and anchored == [1]
    gateway.control(cg.START, (1, 0, 0), 10.2)
    assert not gateway.control(cg.ANCHOR, (0, 0, 0), 10.3) and anchored == [1]      # AUTO 중에는 받지 않는다
    gateway.control(cg.ESTOP, (0, 0, 0), 10.4)
    assert not gateway.control(cg.ANCHOR, (0, 0, 0), 10.5) and anchored == [1]      # 비상 정지 유지 시간
    assert gateway.control(cg.ANCHOR, (0, 0, 0), 11.) and anchored == [1, 1]


def test_orb_reset_is_forwarded_to_runner_only_in_teleop_and_after_estop_hold():
    resets = []
    gateway, _ = make(on_orb_reset=lambda: resets.append(1))
    gateway.runner_status(dict(READY), 10.)
    assert gateway.control(cg.ORB_RESET, (0, 0, 0), 10.1) and resets == [1]
    gateway.control(cg.START, (1, 0, 0), 10.2)
    assert not gateway.control(cg.ORB_RESET, (0, 0, 0), 10.3) and resets == [1]     # AUTO 중에는 받지 않는다
    gateway.control(cg.ESTOP, (0, 0, 0), 10.4)
    assert not gateway.control(cg.ORB_RESET, (0, 0, 0), 10.5) and resets == [1]     # 비상 정지 유지 시간
    assert gateway.control(cg.ORB_RESET, (0, 0, 0), 11.) and resets == [1, 1]


def test_snapshot_passes_runner_detail_to_the_screen_only_while_fresh():
    gateway, _ = make()
    gateway.runner_status(dict(READY, stage='entry30', run=1, reason='complete', anchor_count=2, done=True, active=False), 10.)
    snapshot = gateway.snapshot(10.1)
    assert (snapshot['anchor_count'], snapshot['stage'], snapshot['run'], snapshot['stationary_s']) == (2, 'entry30', 1, 3.5)
    assert snapshot['done'] and snapshot['reason'] == 'complete'
    assert 'anchor_count' not in gateway.snapshot(12.)    # 실행기 상태가 오래되면 이전 값을 숨긴다


def test_runner_done_returns_to_teleop_with_zero():
    gateway, sink = ready_gateway()
    gateway.control(cg.START, (1, 0, 0), 10.1)
    gateway.runner_status(dict(READY, done=True), 20.)
    assert gateway.snapshot(20.)['mode'] == 'TELEOP' and sink.sent[-1] == (0., 0., 0.)


def test_command_codec_round_trip_and_rejects_bad_packets():
    packet = cg.pack_cmd(7, .1, -.2, .3)
    seq, cmd = cg.parse_cmd(packet)
    assert seq == 7 and cmd == pytest.approx((.1, -.2, .3))
    assert cg.parse_cmd(b'XXXX'+packet[4:]) is None
    assert cg.parse_cmd(packet[:-1]) is None
    assert cg.parse_cmd(cg.pack_cmd(1, float('nan'), 0., 0.)) is None


def test_control_codec_round_trip():
    seq, op, args = cg.parse_ctl(cg.pack_ctl(3, cg.POSE, 2.9, 1.2, .1))
    assert (seq, op) == (3, cg.POSE) and args == pytest.approx((2.9, 1.2, .1))
    assert cg.parse_ctl(cg.pack_cmd(1, 0., 0., 0.)) is None


class FakePolicy:
    """run_policy의 수신부처럼 순서번호가 거꾸로 온 패킷을 버리는 시험용 수신기."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(.05)
        self.last_seq, self.accepted, self.discarded = -1, [], 0
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                data, _ = self.sock.recvfrom(64)
            except socket.timeout:
                continue
            parsed = cg.parse_cmd(data)
            if parsed is None:
                continue
            if parsed[0] <= self.last_seq:
                self.discarded += 1
                continue
            self.last_seq = parsed[0]
            self.accepted.append((time.perf_counter(), parsed[1]))


def test_socket_round_trip_keeps_sequence_across_mode_switches_and_is_fast():
    policy = FakePolicy()
    sockets = cg.open_sockets('127.0.0.1', (0, 0, 0))
    ports = [s.getsockname()[1] for s in sockets]
    stop = threading.Event()
    runner = threading.Thread(target=cg.serve, args=(sockets, policy.sock.getsockname(), stop), daemon=True)
    runner.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    status = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    import json
    seq = 0
    delays = []
    for _ in range(20):                           # 텔레옵 명령의 입력→출력 지연
        seq += 1
        before = len(policy.accepted)
        sent = time.perf_counter()
        client.sendto(cg.pack_cmd(seq, .1, 0., 0.), ('127.0.0.1', ports[0]))
        deadline = time.time()+1.
        while len(policy.accepted) == before and time.time() < deadline:
            time.sleep(.0002)
        assert len(policy.accepted) > before
        delays.append(policy.accepted[before][0]-sent)
        time.sleep(.01)
    status.sendto(json.dumps(dict(READY)).encode(), ('127.0.0.1', ports[2]))
    time.sleep(.05)
    seq += 1
    client.sendto(cg.pack_ctl(seq, cg.START, 1, 0, 0), ('127.0.0.1', ports[0]))
    time.sleep(.05)
    auto = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for index in range(10):
        auto.sendto(cg.pack_cmd(index+1, .05, 0., 0.), ('127.0.0.1', ports[1]))
        time.sleep(.01)
    seq += 1
    client.sendto(cg.pack_ctl(seq, cg.ESTOP, 0, 0, 0), ('127.0.0.1', ports[0]))
    time.sleep(.1)
    stop.set()
    runner.join(1.)
    policy.stop.set()
    assert policy.discarded == 0                  # 모드가 바뀌어도 run_policy가 버린 패킷이 없다
    commands = [c for _, c in policy.accepted]
    assert any(c[0] == pytest.approx(.05) for c in commands) and commands[-1] == (0., 0., 0.)   # float32로 오가므로 근사 비교
    assert sorted(delays)[len(delays)//2] < .005  # 중앙값 5 ms 미만(개발 PC 기준)


def test_status_reply_comes_from_the_port_the_gui_sent_to():
    """상태가 5006이 아닌 다른 소스 포트에서 오면 노트북의 stateful 방화벽·NAT가 응답으로 인정하지 않고 버린다."""
    import json
    sockets = cg.open_sockets('127.0.0.1', (0, 0, 0))
    teleop_port = sockets[0].getsockname()[1]
    stop = threading.Event()
    thread = threading.Thread(target=cg.serve, args=(sockets, ('127.0.0.1', 9), stop), daemon=True)
    thread.start()
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.settimeout(2.)
    client.sendto(cg.pack_ctl(1, cg.HEARTBEAT), ('127.0.0.1', teleop_port))
    data, address = client.recvfrom(2048)
    stop.set()
    thread.join(1.)
    assert 'mode' in json.loads(data) and address[1] == teleop_port
