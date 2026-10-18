# ================================================================
# test_rl_residual.py
# 설명: RlResidual이 실기 run_policy 루프와 같은 순서(관측 조립 → 정지 명령 잔차 0 → 잔차 → 펌웨어 C IK)로 목표를 만들고 SimulationDrive의 조인트 경계에 연결되는지, 끄면 기존 동작이 그대로인지 검사한다. KSRC_ROOT(ksrc 체크아웃, sim/libksrc_common.so 빌드됨)가 없으면 건너뛴다.
# 사용법: KSRC_ROOT=/path/to/ksrc .venv-mapping/bin/python -m pytest tests/site_sim/test_rl_residual.py
# ================================================================
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = os.environ.get('KSRC_ROOT')
pytestmark = pytest.mark.skipif(not ROOT or not (Path(ROOT)/'sim'/'libksrc_common.so').is_file(),
                                reason='KSRC_ROOT with built sim/libksrc_common.so required')
META = lambda: json.loads((Path(ROOT)/'pi/policy/res8_mlp_v1_rc4000.json').read_text())  # noqa: E731
STATE = {'gyro_body': np.zeros(3), 'force_body': np.array([0., 0., 9.81]), 'steer_pos': np.zeros(4),
         'steer_vel': np.zeros(4), 'wheel_vel': np.zeros(4)}


def make(policy, **kw):
    from experiments.site_sim.rl_residual import RlResidual
    return RlResidual(Path(ROOT), policy, meta=META(), **kw)


def test_zero_action_equals_plain_firmware_ik_and_obs_layout_matches_policy():
    seen = []
    rl = make(lambda obs: (seen.append(obs.shape), np.zeros(8, np.float32))[1])
    out = rl.step((.1, 0., .0), STATE)
    assert seen == [(1, 145)]
    assert np.allclose(out['steer_goal'], 0) and np.allclose(out['wheel_mps'], .1, atol=1e-6)
    assert np.allclose(out['d_steer'], 0) and np.allclose(out['wheel_gain'], 1)


def test_full_action_adds_scaled_residual_and_clips_to_firmware_limits():
    rl = make(lambda obs: np.ones(8, np.float32))
    out = rl.step((.1, 0., 0.), STATE)
    scale = rl.runtime.C.RES8_STEER_SCALE_RAD
    assert np.allclose(out['d_steer'], scale) and np.allclose(out['steer_goal'], scale, atol=1e-5)
    assert np.allclose(out['wheel_mps'], .1*(1+rl.runtime.C.RES8_WHEEL_SCALE), atol=1e-5)
    fast = make(lambda obs: np.ones(8, np.float32)).step((.3, 0., 0.), STATE)
    assert np.all(fast['wheel_mps'] <= rl.runtime.FW_MAX_WHEEL_MPS+1e-6)


def test_standing_command_zeroes_residual_and_residual_off_ignores_policy():
    rl = make(lambda obs: np.ones(8, np.float32))
    out = rl.step((0., 0., 0.), STATE)
    assert np.allclose(out['d_steer'], 0) and np.allclose(out['wheel_gain'], 1)
    off = make(lambda obs: np.ones(8, np.float32), residual_on=False).step((.1, 0., 0.), STATE)
    assert np.allclose(off['d_steer'], 0) and np.allclose(off['wheel_mps'], .1, atol=1e-6)


def test_control_dt_must_match_policy_and_history_is_fed_oldest_first():
    from experiments.site_sim.rl_residual import RlResidual
    with pytest.raises(ValueError, match='control dt'):
        RlResidual(Path(ROOT), lambda obs: np.zeros(8), meta=META(), dt=.05)
    captured = []
    rl = make(lambda obs: (captured.append(obs.copy()), np.zeros(8, np.float32))[1])
    rl.step((.1, 0., 0.), STATE)
    rl.step((.0, .0, .3), STATE)
    layout = dict(rl.obs.layout)
    first_term = captured[1][0, :5*layout['command']].reshape(5, layout['command'])
    assert np.allclose(first_term[-1], (0., 0., .3)) and np.allclose(first_term[-2], (.1, 0., 0.))   # 최신이 마지막


class Robot:
    """SimulationDrive가 RL 관측을 만들 때 쓰는 속도·조인트 값을 가진 최소 fixture."""

    def __init__(self):
        self.position, self.quaternion = np.array([.5, .5, 0.]), np.array([1., 0., 0., 0.])

    def get_world_pose(self): return self.position, self.quaternion
    def get_joint_positions(self): return np.zeros(8)
    def get_joint_velocities(self): return np.zeros(8)
    def get_linear_velocity(self): return np.zeros(3)
    def get_angular_velocity(self): return np.zeros(3)


def test_drive_applies_rl_targets_to_joints_and_default_off_is_unchanged(mission):
    from experiments.site_sim.drive import SimulationDrive
    from tests.site_sim.test_drive import Session
    plain_session = Session()
    plain = SimulationDrive(Robot(), plain_session, mission).step(.1)
    assert 'rl_d_steer_rad' not in plain
    session = Session()
    rl = make(lambda obs: np.ones(8, np.float32))
    row = SimulationDrive(Robot(), session, mission, rl=rl).step(.1)
    assert np.allclose(row['rl_d_steer_rad'], rl.runtime.C.RES8_STEER_SCALE_RAD)
    assert np.allclose(session.steering-plain_session.steering, rl.runtime.C.RES8_STEER_SCALE_RAD, atol=.02)   # 같은 명령 + 잔차
    assert np.max(np.abs(session.wheels)) > np.max(np.abs(plain_session.wheels))            # 바퀴 배율 1.6
