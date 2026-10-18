# ================================================================
# test_joint_convention.py
# 설명: 펌웨어(KSRC) 규약의 (조향각, 바퀴 속도)와 시뮬 조인트 규약(steering_sign, rolling_heading)이 서로 물리적으로 같은 바퀴 속도 벡터로 바뀌는지, RL 경로가 제자리 회전 방향을 뒤집지 않는지 검사한다.
# 사용법: KSRC_ROOT=/path/to/ksrc .venv-mapping/bin/python -m pytest tests/site_sim/test_joint_convention.py
# ================================================================
import math
import os
from pathlib import Path

import numpy as np
import pytest

from experiments.orbslam3_ros2.swerve import ModuleGeometry, SwerveGeometry, body_twist_to_modules

# 서버 시뮬이 쓰는 rover_assets_new 기하(최신 URDF FK): 조향 부호 -1, 왼쪽 바퀴는 구름 방향이 -x, 조향 joint 이름이 앞뒤 오른쪽에서 서로 바뀌어 있다.
SIM = SwerveGeometry(modules=(
    ModuleGeometry('front_left', 'fl_steering_joint', 'fl_wheel_joint', .08412474402, .1325, -1., -math.pi),
    ModuleGeometry('front_right', 'rr_steering_joint', 'fr_wheel_joint', .08412474402, -.1325, -1., 0.),
    ModuleGeometry('rear_left', 'rl_steering_joint', 'rl_wheel_joint', -.1207380504, .1325, -1., -math.pi),
    ModuleGeometry('rear_right', 'fr_steering_joint', 'rr_wheel_joint', -.1207380504, -.1325, -1., 0.),
), wheel_radius_m=.04743295907974243)
COMMANDS = [(.03, 0., -.6), (0., 0., -.6), (0., 0., .6), (.08, 0., 0.), (-.08, 0., 0.), (.1, 0., .3), (0., .1, 0.), (.15, 0., -.3)]


def physical_vector(module, q, wheel_rad_s, radius):
    """시뮬 조인트 값이 만드는 바퀴 접지점 속도 벡터(body 축). +wheel 속도는 rolling_heading+steering_sign*q 방향으로 굴러간다."""
    heading = module.rolling_heading_rad+module.steering_sign*q
    return wheel_rad_s*radius*np.array([math.cos(heading), math.sin(heading)])


def fitted_twist(vectors):
    """네 바퀴 속도 벡터에 최소제곱으로 맞춘 강체 (vx, vy, yaw_rate). 위치는 시뮬 기하."""
    rows, rhs = [], []
    for m, v in zip(SIM.modules, vectors):
        rows += [[1, 0, -m.y_m], [0, 1, m.x_m]]
        rhs += list(v)
    return np.linalg.lstsq(np.array(rows), np.array(rhs), rcond=None)[0]


def test_to_joint_equals_sim_ik_and_roundtrips_through_from_joint():
    from experiments.site_sim.joint_convention import from_joint, to_joint
    for vx, vy, w in COMMANDS:
        sim = body_twist_to_modules(SIM, vx=vx, vy=vy, yaw_rate=w)
        for m, cmd in zip(SIM.modules, sim):
            mx, my = vx-w*m.y_m, vy+w*m.x_m
            q, wheel = to_joint(m, math.atan2(my, mx), math.hypot(mx, my), SIM.wheel_radius_m)
            assert q == pytest.approx(cmd.steering_angle_rad, abs=1e-9) and wheel*SIM.wheel_radius_m == pytest.approx(cmd.wheel_angular_velocity_rad_s*SIM.wheel_radius_m, abs=1e-9)
            theta, wheel_fw = from_joint(m, q, wheel)
            back = physical_vector(m, q, wheel, SIM.wheel_radius_m)
            assert np.allclose(back, [mx, my], atol=1e-9)                                  # 같은 물리 벡터
            assert -math.pi/2-1e-9 <= theta <= math.pi/2+1e-9
            assert np.allclose(wheel_fw*SIM.wheel_radius_m*np.array([math.cos(theta), math.sin(theta)]), [mx, my], atol=1e-9)


@pytest.mark.skipif(not os.environ.get('KSRC_ROOT') or not (Path(os.environ.get('KSRC_ROOT', ''))/'sim/libksrc_common.so').is_file(),
                    reason='KSRC_ROOT with built sim/libksrc_common.so required')
def test_firmware_targets_map_to_joint_targets_that_turn_the_right_way():
    from experiments.site_sim.joint_convention import to_joint
    from experiments.site_sim.rl_residual import load_student_runtime
    rt = load_student_runtime(Path(os.environ['KSRC_ROOT']))
    for vx, vy, w in COMMANDS:
        goal, speed = rt.FirmwareIK().step(vx, vy, w)
        vectors = []
        for m, th, v in zip(SIM.modules, goal, speed):
            q, wheel = to_joint(m, float(th), float(v), SIM.wheel_radius_m)
            vectors.append(physical_vector(m, q, wheel, SIM.wheel_radius_m))
        twist = fitted_twist(vectors)
        assert twist[0] == pytest.approx(vx, abs=.02) and twist[1] == pytest.approx(vy, abs=.02)
        assert twist[2] == pytest.approx(w, abs=.12)                                       # 회전 방향·크기(펌웨어 기하와 시뮬 기하의 차이 허용)


class SimSession:
    """서버 시뮬과 같은 조인트 규약(SIM 기하)을 가진 가짜 세션: 조향 4개, 바퀴 4개의 목표를 보존한다."""

    def __init__(self):
        from types import SimpleNamespace
        self.geometry = SIM
        self.controller = self
        self._action_type = SimpleNamespace
        self.steering_indices, self.wheel_indices = np.arange(4), np.arange(4, 8)
        self.steering, self.wheels = np.zeros(4), np.zeros(4)

    def apply_action(self, action):
        if hasattr(action, 'joint_positions'):
            self.steering = np.asarray(action.joint_positions, float)
        if hasattr(action, 'joint_velocities'):
            self.wheels = np.asarray(action.joint_velocities, float)


@pytest.mark.skipif(not os.environ.get('KSRC_ROOT') or not (Path(os.environ.get('KSRC_ROOT', ''))/'sim/libksrc_common.so').is_file(),
                    reason='KSRC_ROOT with built sim/libksrc_common.so required')
def test_drive_with_zero_residual_turns_the_commanded_direction_on_sim_joint_conventions(mission):
    """회귀: RL 경로가 펌웨어 목표를 조인트에 그대로 줘서 ω −0.6이 후진으로, 직진이 회전으로 나오던 문제."""
    from experiments.site_sim.drive import SimulationDrive
    from experiments.site_sim.rl_residual import RlResidual
    from tests.site_sim.test_rl_residual import Robot   # 속도·조인트 값을 가진 fixture
    import json
    root = Path(os.environ['KSRC_ROOT'])
    meta = json.loads((root/'pi/policy/res8_mlp_v1_rc4000.json').read_text())
    session = SimSession()
    rl = RlResidual(root, lambda obs: np.zeros(8, np.float32), meta=meta)
    row = SimulationDrive(Robot(), session, mission, rl=rl).step(.1)
    command = row['applied_command']
    vectors = [physical_vector(m, q, w, SIM.wheel_radius_m) for m, q, w in zip(SIM.modules, session.steering, session.wheels)]
    twist = fitted_twist(vectors)
    assert twist[0] == pytest.approx(command['vx_m_s'], abs=.02)
    assert twist[2] == pytest.approx(command['yaw_rate_rad_s'], abs=.12)
    assert abs(command['yaw_rate_rad_s']) > .3                                            # 이 임무 첫 명령은 큰 회전이다
