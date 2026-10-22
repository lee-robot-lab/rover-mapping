# ================================================================
# test_cli.py
# 설명: 오프라인 CLI·보정·MCU timestamp 중복 처리를 검증한다.
# 사용법: python -m pytest tests/pi_navigation -q
# ================================================================
from pathlib import Path
import json
import subprocess
import sys
import socket
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
