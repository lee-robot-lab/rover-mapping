# ================================================================
# test_cli.py
# 설명: 오프라인 CLI·보정·MCU timestamp 중복 처리를 검증한다.
# 사용법: python -m pytest tests/pi_navigation -q
# ================================================================
from pathlib import Path
import json
import os
import subprocess
import sys
import socket
import time
import pytest

ROOT = Path(__file__).resolve().parents[2]
DEPLOY = ROOT / 'deployment/ksrc'
sys.path.insert(0, str(DEPLOY))


def test_cli_help_and_check_without_network():
    for args in (['--help'], ['check']):
        result = subprocess.run([sys.executable, str(DEPLOY / 'cli.py'), *args], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr + result.stdout
    assert 'f' in result.stdout


def test_default_launch_is_fake():
    import cli
    args = cli.parser().parse_args(['start'])
    assert not args.real
    assert args.async_plan
    assert (args.upper_cpu, args.lower_cpu, args.orb_cpus, args.rt) == ('0', '3', '1,2', 50)


def test_sync_plan_requires_explicit_option():
    import cli
    args = cli.parser().parse_args(['start', '--sync-plan'])
    assert not args.async_plan


@pytest.mark.parametrize('run', [1, 2, 3, 4])
def test_start_command_sends_selected_run(monkeypatch, run):
    import cli
    sent = []
    monkeypatch.setattr(sys, 'argv', ['ksrc', f'start{run}'])
    monkeypatch.setattr(cli, 'control', lambda op, value: sent.append((op, value)))
    cli.main()
    assert sent == [(3, run)]


@pytest.mark.parametrize('command, preset', [('anchor', 1), ('anchor1', 1), ('anchor2', 2),
                                           ('anchor3', 3), ('anchor4', 4)])
def test_anchor_command_sends_selected_preset_to_runner(monkeypatch, command, preset):
    import cli
    packets = []

    class CaptureSocket:
        """실행 중인 로버에 닿지 않도록 CLI의 UDP 전송만 기록한다."""

        def __enter__(self):
            return self

        def __exit__(self, *_):
            pass

        def sendto(self, payload, destination):
            packets.append((json.loads(payload), destination))

    monkeypatch.setattr(sys, 'argv', ['ksrc', command])
    monkeypatch.setattr(cli.socket, 'socket', lambda *_: CaptureSocket())
    cli.main()
    assert packets == [({'op': 'ANCHOR', 'preset': preset}, ('127.0.0.1', 5010))]


def test_stop_returns_after_owned_processes_finish(tmp_path):
    command = [sys.executable, str(DEPLOY / 'cli.py'), '--state-dir', str(tmp_path)]
    started = subprocess.run([*command, 'start', '--seconds', '30', '--policy-script', str(tmp_path / 'missing.py')],
                             capture_output=True, text=True)
    assert started.returncode == 0, started.stderr
    try:
        result = subprocess.run([*command, 'stop'], capture_output=True, text=True, timeout=15)
        assert result.returncode == 0, result.stderr
        assert (tmp_path / 'stopped.json').is_file()
    finally:
        subprocess.run([*command, 'stop'], capture_output=True, timeout=15)


def test_policy_mcu_duplicates_do_not_generate_new_imu():
    import policy_telemetry
    import numpy as np
    config = dict(calibration={'ROS.T_c1_base': np.eye(4).tolist(), 'IMU.T_b_c1': np.eye(4).tolist()},
                  imu=dict(gyro_bias_dps=[0, 0, 0], acc_offset_g=[0, 0, 0], acc_scale=[1, 1, 1]))
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1', 0))
    receiver.settimeout(.1)
    tee = policy_telemetry.Telemetry(config, receiver.getsockname()[1])
    tlm = dict(t_ms=100, flags=16, gyro_dps=[0, 0, 0], acc_g=[0, 0, 1], wheel_mps=[0, 0, 0, 0])
    try:
        tee.emit(tlm)
        data = json.loads(receiver.recv(4096))
        assert data['force'] == pytest.approx([0, 0, 9.81])
        tee.emit(tlm)
        with pytest.raises(socket.timeout):
            receiver.recv(4096)
    finally:
        receiver.close()
        tee.socket.close()


def test_fake_policy_stop_uses_save_path(tmp_path):
    policy = tmp_path / 'fake_policy.py'
    policy.write_text('''# 테스트 정책: Ctrl+C 종료 시 로그 저장 경로를 검증한다.
import argparse, signal, time
from pathlib import Path
parser = argparse.ArgumentParser()
parser.add_argument('--log', type=Path, required=True)
args, _ = parser.parse_known_args()
signal.signal(signal.SIGINT, signal.default_int_handler)
args.log.with_suffix('.ready').touch()
try:
    while True:
        time.sleep(.01)
except KeyboardInterrupt:
    args.log.write_text('saved after SIGINT')
''')
    state = tmp_path / 'state'
    command = [sys.executable, str(DEPLOY / 'cli.py'), '--state-dir', str(state)]
    subprocess.run([*command, 'start', '--seconds', '30', '--policy-script', str(policy)], check=True)
    try:
        deadline = time.monotonic() + 5
        while not (state / 'policy.ready').exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert (state / 'policy.ready').exists()
        subprocess.run([*command, 'stop'], check=True, timeout=15)
        assert (state / 'policy.npz').read_text() == 'saved after SIGINT'
    finally:
        subprocess.run([*command, 'stop'], timeout=15)


def fake_other_session(directory: Path, **status: object) -> None:
    """이 테스트 프로세스를 supervisor로 기록한 가짜 세션 폴더. 상태가 idle이 아니면 종료 대상이 되지 않는다."""
    import cli
    directory.mkdir()
    (directory / 'supervisor.json').write_text(json.dumps(dict(pid=os.getpid(), starttime=cli.process_identity(os.getpid()))))
    if status:
        (directory / 'status.json').write_text(json.dumps(status))


@pytest.mark.parametrize('status, reason', [(dict(active=True), 'AUTO'), (None, '알 수 없음')])
def test_start_refuses_to_stop_other_session_when_auto_or_unknown(tmp_path, status, reason):
    import cli
    fake_other_session(tmp_path / 'other', **(status or {}))
    with pytest.raises(RuntimeError, match=reason):
        cli.stop_other_sessions(tmp_path / 'new')
    assert cli.process_identity(os.getpid())   # 이 프로세스(가짜 owner)는 종료되지 않았다


def test_start_stops_idle_previous_session_then_starts(tmp_path):
    first, second = tmp_path / 'first', tmp_path / 'second'
    fake = ['--seconds', '60', '--policy-script', str(tmp_path / 'missing.py')]
    command = lambda state: [sys.executable, str(DEPLOY / 'cli.py'), '--state-dir', str(state)]
    assert subprocess.run([*command(first), 'start', *fake], capture_output=True, text=True).returncode == 0
    try:
        deadline = time.time() + 30
        while time.time() < deadline and not (first / 'status.json').exists():
            time.sleep(.2)
        assert (first / 'status.json').exists(), '첫 세션이 상태를 쓰지 않았다'
        result = subprocess.run([*command(second), 'start', *fake], capture_output=True, text=True, timeout=40)
        assert result.returncode == 0, result.stdout + result.stderr
        assert '먼저 종료' in result.stdout and (first / 'stopped.json').is_file()
        assert (second / 'supervisor.json').is_file() and not (second / 'stopped.json').exists()
    finally:
        for state in (second, first):
            subprocess.run([*command(state), 'stop'], capture_output=True, timeout=20)
