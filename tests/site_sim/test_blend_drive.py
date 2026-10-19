# ================================================================
# test_blend_drive.py
# 설명: 명령의 vy_m_s가 SimulationDrive의 IK·RL 경로와 텔레메트리·기록기까지 전달되고, vy가 없으면 0으로 취급되는지 검사한다.
# 사용법: KSRC_ROOT=outputs/site_sim/ksrc_20261016 .venv-mapping/bin/python -m pytest tests/site_sim/test_blend_drive.py -q
# ================================================================
import numpy as np
import pytest

from tests.site_sim.test_drive import Robot, Session


class RlRobot(Robot):
    """RL 상태 조립에 필요한 속도 측정을 더한 로봇 fixture."""
    def get_linear_velocity(self): return np.zeros(3)
    def get_angular_velocity(self): return np.zeros(3)


class FakeRuntime:
    """정해진 명령 dict를 돌려주는 임무 runtime 대역."""
    def __init__(self, mission, command, state='to_junction'):
        self.mission, self.command, self.state, self.route, self.events = mission, command, state, '30', []

    def update(self, sample, now_s):
        return {'command': dict(self.command), 'state': self.state, 'route': self.route, 'reason': 'active'}

    def _event(self, *args, **kwargs): self.events.append((args, kwargs))


class FakeRl:
    """rl.step의 첫 인자를 기록하고 영 잔차 목표를 돌려주는 대역."""
    def __init__(self): self.commands = []

    def step(self, command, state, sim_s):
        self.commands.append(tuple(command))
        return {'steer_goal': np.zeros(4), 'wheel_mps': np.zeros(4), 'd_steer': np.zeros(4), 'wheel_gain': np.ones(4),
                'action': np.zeros(8), 'obs_frame': np.zeros(3)}


def captured_twists(monkeypatch):
    from experiments.site_sim import drive as drive_module
    seen, real = [], drive_module.body_twist_to_modules
    def spy(geometry, **kwargs):
        seen.append(kwargs);return real(geometry, **kwargs)
    monkeypatch.setattr(drive_module, 'body_twist_to_modules', spy)
    return seen


def make_drive(mission, command, *, rl=None, robot=None, **kwargs):
    from experiments.site_sim.drive import SimulationDrive
    drive = SimulationDrive(robot or Robot(), Session(), mission, rl=rl, **kwargs)
    drive.runtime = FakeRuntime(mission, command)
    return drive


def test_drive_ik_path_passes_vy_to_modules(mission, monkeypatch):
    seen = captured_twists(monkeypatch)
    make_drive(mission, {'vx_m_s': .02, 'vy_m_s': .05, 'yaw_rate_rad_s': .03}).step(.1)
    assert seen[0]['vx'] == .02 and seen[0]['vy'] == .05 and seen[0]['yaw_rate'] == .03


def test_drive_rl_path_passes_vy_to_policy(mission):
    rl = FakeRl()
    make_drive(mission, {'vx_m_s': .02, 'vy_m_s': .05, 'yaw_rate_rad_s': .03}, rl=rl, robot=RlRobot()).step(.1)
    assert rl.commands == [(.02, .05, .03)]


def test_drive_without_vy_key_uses_zero(mission, monkeypatch):
    seen = captured_twists(monkeypatch)
    make_drive(mission, {'vx_m_s': .02, 'yaw_rate_rad_s': .03}).step(.1)
    assert seen[0]['vy'] == 0.
    rl = FakeRl()
    make_drive(mission, {'vx_m_s': .02, 'yaw_rate_rad_s': .03}, rl=rl, robot=RlRobot()).step(.1)
    assert rl.commands == [(.02, 0., .03)]


def test_hold_and_fault_paths_apply_zero_vy(mission, monkeypatch):
    seen = captured_twists(monkeypatch)
    command = {'vx_m_s': .02, 'vy_m_s': .05, 'yaw_rate_rad_s': .03}
    row = make_drive(mission, command, hold_s=1.).step(.1)   # IMU 영점 정지 구간
    assert seen[-1]['vy'] == 0. and seen[-1]['vx'] == 0. and row['requested_command'] == command
    drive = make_drive(mission, command, fault_injection=True)
    drive.runtime.state = 'ascending'
    drive.robot.position = np.array([1.3, 1.5, 0.])
    row = drive.step(1.)
    assert row['fault_active'] and seen[-1]['vy'] == 0. and seen[-1]['vx'] == 0.
    rl = FakeRl()
    make_drive(mission, command, rl=rl, robot=RlRobot(), hold_s=1.).step(.1)
    assert rl.commands == [(0., 0., 0.)]


@pytest.mark.parametrize('blend', [True, False])
def test_telemetry_row_carries_follower_and_commands(mission, tmp_path, blend):
    from experiments.site_sim.drive import SimulationDrive
    from experiments.site_sim.recording import RunRecorder
    mission.settings['follower_blend'] = 1. if blend else 0.
    drive = SimulationDrive(Robot(), Session(), mission)
    recorder = RunRecorder(mission, tmp_path / 'run')
    for k in range(3):
        row = drive.step(.1 * (k + 1))
        recorder.observe(row, drive.runtime.events)
    if blend:
        assert 'vy_m_s' in row['requested_command'] and 'vy_m_s' in row['applied_command']
        assert set(row['mission']['follower']) == {'psi_ref', 'heading_gap_rad', 'travel_gap_rad', 'gated', 'realigning'}
    else:
        assert 'vy_m_s' not in row['requested_command'] and 'follower' not in row['mission']
    recorder.finalize('failed', 'test')
