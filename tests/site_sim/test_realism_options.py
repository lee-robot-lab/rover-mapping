# ================================================================
# test_realism_options.py
# 설명: 실기 차이를 시뮬에 넣는 옵션(명령 지연, IMU 패킷 누락, 시작 자세 노이즈, RL 부팅 바이어스)과 완주·벽 거리 평가가 의도한 크기·순서로 동작하는지 검사한다.
# 사용법: KSRC_ROOT=/path/to/ksrc .venv-mapping/bin/python -m pytest tests/site_sim/test_realism_options.py
# ================================================================
import json
import math
import os
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.localization.imu_realism import ImuRealism
from experiments.localization.online import OnlineEstimator
from experiments.site_sim.clearance import clearance_map, evaluate
from experiments.site_sim.drive import ActuationDelay, SimulationDrive
from experiments.site_sim.start_noise import start_offset
from tests.site_sim.test_drive import Robot, Session

CALIBRATION = {'ROS.T_c1_base': np.eye(4).tolist(), 'IMU.T_b_c1': np.eye(4).tolist(),
               'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .002, 'IMU.GyroWalk': .00002, 'IMU.AccWalk': .003}


def test_delay_is_within_range_ordered_and_clearable():
    delay = ActuationDelay(.0176, .0465, seed=1)
    assert delay.pop(0.) is None
    for k in range(200):
        delay.push(.02*k, np.full(4, k), np.full(4, k))
    releases = [q[0] for q in delay.queue]
    assert all(b >= a for a, b in zip(releases, releases[1:]))
    assert all(.0176-1e-9 <= r-.02*k <= .0465+.02*200 for k, r in enumerate(releases))   # 순서 보존 때문에 상한만 느슨
    first = delay.queue[0][0]-0.
    assert .0176 <= first <= .0465
    delay.clear()
    assert delay.pop(100.) is None
    with pytest.raises(ValueError):
        ActuationDelay(.1, .05)


def test_drive_applies_targets_only_after_the_delay(mission):
    session = Session()
    drive = SimulationDrive(Robot(), session, mission, delay=ActuationDelay(.03, .03))
    drive.step(.05)
    assert np.array_equal(session.wheels, np.zeros(4))          # 목표는 계산됐지만 아직 조인트에 안 닿음
    drive.step(.10)
    assert np.max(np.abs(session.wheels)) > 0
    drive.stop()
    assert np.array_equal(session.wheels, np.zeros(4)) and not drive.delay.queue


def test_start_offset_is_reproducible_clipped_and_zero_when_disabled():
    assert start_offset(3, 0., 0.) == (0., 0., 0.)
    assert start_offset(3, .02, 2.) == start_offset(3, .02, 2.)
    assert start_offset(3, .02, 2.) != start_offset(4, .02, 2.)
    draws = np.array([start_offset(k, .02, 2.) for k in range(2000)])
    assert np.abs(draws[:, :2]).max() <= .06+1e-12 and np.abs(draws[:, 2]).max() <= math.radians(6)+1e-12
    assert draws[:, 0].std() == pytest.approx(.02, rel=.1) and np.degrees(draws[:, 2]).std() == pytest.approx(2., rel=.1)
    with pytest.raises(ValueError):
        start_offset(0, -1., 0.)


def test_dropped_imu_packets_are_merged_into_contiguous_windows():
    est = OnlineEstimator(np.eye(4), CALIBRATION, imu_realism=ImuRealism(seed=2, drop_probability=.3))
    q = Rotation.identity().as_quat()
    windows, nones = [], 0
    for k in range(1, 400):
        t = .02*k
        sample = est.on_step(t, np.array([.07*t, 0., 0.]), np.r_[q[3], q[:3]])
        if sample is None:
            nones += 1
        else:
            windows.append(sample)
    assert nones > 50
    assert all(b[0] == pytest.approx(a[1]) for a, b in zip(windows, windows[1:]))        # 시간 공백·중복 없음
    assert max(w[1]-w[0] for w in windows) <= .06+.02+1e-9                               # 필터 공백 기준 0.1 s보다 짧음
    assert sum(w[1]-w[0] for w in windows) == pytest.approx(windows[-1][1]-windows[0][0])
    plain = OnlineEstimator(np.eye(4), CALIBRATION, imu_realism=ImuRealism(seed=2))
    assert all(plain.on_step(.02*k, np.array([.0014*k, 0., 0.]), np.r_[q[3], q[:3]]) is not None for k in range(1, 50))


@pytest.mark.skipif(not os.environ.get('KSRC_ROOT') or not (Path(os.environ.get('KSRC_ROOT', ''))/'sim/libksrc_common.so').is_file(),
                    reason='KSRC_ROOT with built sim/libksrc_common.so required')
def test_rl_boot_bias_is_estimated_from_realistic_gyro_and_removed_from_the_observation():
    from experiments.site_sim.rl_residual import RlResidual
    root = Path(os.environ['KSRC_ROOT'])
    meta = json.loads((root/'pi/policy/res8_mlp_v1_rc4000.json').read_text())
    captured = []
    rl = RlResidual(root, lambda obs: (captured.append(obs.copy()), np.zeros(8, np.float32))[1], meta=meta,
                    imu_realism=ImuRealism(seed=3), bias_s=2.)
    state = {'gyro_body': np.zeros(3), 'force_body': np.array([0., 0., 9.81]), 'steer_pos': np.zeros(4),
             'steer_vel': np.zeros(4), 'wheel_vel': np.zeros(4), 'speed': 0.}
    for k in range(250):
        rl.step((0., 0., 0.), state, sim_s=.02*k)
    assert rl.bias == pytest.approx(np.radians((-.630, .485, .488)), abs=3e-4)
    last_gyro = np.array([o[0, 15+12:15+15] for o in captured[200:]])                # gyro 항의 최신 프레임
    assert np.abs(last_gyro.mean(0)).max() < 4e-4                                     # 바이어스가 빠진 뒤 평균 0 근처


def test_clearance_map_and_evaluation(tmp_path, mission):
    blocked = np.zeros((20, 20), bool)
    blocked[10, 10] = True
    distance = clearance_map(blocked, .1)
    assert distance[10, 13] == pytest.approx(.3) and distance[10, 10] == 0
    assert np.isinf(clearance_map(np.zeros((3, 3), bool), .1)).all()
    folder = tmp_path/'output'
    folder.mkdir()
    site = mission.site
    inside = [float(site.origin[0]+.5*site.resolution*(site.height_m.shape[1])), float(site.origin[1]+.5*site.resolution*site.height_m.shape[0])]
    rows = [{'position_m': inside+[0.]}, {'position_m': [site.origin[0]-5., site.origin[1]-5., 0.]}]
    (folder/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    (folder/'summary.json').write_text(json.dumps({'status': 'complete', 'reason': 'phase_transition', 'stuck_events': 1,
                                                    'duration_sim_s': 10., 'mission': {'state': 'complete', 'route_status': {}}}))
    mission_folder = tmp_path/'m'
    mission.save(mission_folder)
    result = evaluate(folder, mission_folder)
    assert result['completed'] is True and result['samples'] == 2 and result['stuck_events'] == 1
    assert result['min_clearance_m'] == 0.0 and result['contact_samples'] >= 1          # 지도 밖 표본은 충돌로 센다
    (folder/'summary.json').write_text(json.dumps({'status': 'complete', 'reason': 'x', 'mission': {'state': 'failed'}}))
    assert evaluate(folder, mission_folder)['completed'] is False                       # 임무 상태가 complete가 아니면 완주 아님
