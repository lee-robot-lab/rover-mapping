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


def test_start_heading_survives_physics_reset():
    from experiments.site_sim import start_noise

    class ResettableRobot:
        """현재 pose와 reset pose를 분리한 articulation 대역."""

        def __init__(self):
            self.position = np.array([3.6, 2.82, .52])
            self.orientation = np.array([1., 0., 0., 0.])
            self.default_position = self.position.copy()
            self.default_orientation = self.orientation.copy()

        def get_world_pose(self):
            return self.position.copy(), self.orientation.copy()

        def set_world_pose(self, position, orientation):
            self.position, self.orientation = position.copy(), orientation.copy()

        def set_default_state(self, position, orientation):
            self.default_position, self.default_orientation = position.copy(), orientation.copy()

        def post_reset(self):
            self.set_world_pose(self.default_position, self.default_orientation)

    robot = ResettableRobot()
    start_noise.set_start_pose(robot, -math.pi / 2, .02, -.01)
    robot.post_reset()
    assert robot.position == pytest.approx([3.62, 2.81, .52])
    rotation = Rotation.from_quat(np.r_[robot.orientation[1:], robot.orientation[0]])
    assert rotation.apply([1., 0., 0.]) == pytest.approx([0., -1., 0.], abs=1e-12)


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


def test_server_forwards_estimator_experiment_flags(monkeypatch):
    """--visual-turn-scale·--filter-acc-noise가 서버 start에서 run.py 인자로 그대로 넘어간다."""
    from experiments.site_sim import server
    seen = {}
    monkeypatch.setattr(server, 'start', lambda *args, **kwargs: seen.update(extra=kwargs['extra_args']))
    assert server.main(['start', 'lbl', '--visual-turn-scale', '1', '--filter-acc-noise', '0.025']) == 0
    assert seen['extra'] == ['--visual-turn-scale', '1.0', '--filter-acc-noise', '0.025']


def test_server_forwards_the_velocity_direction_sigma(monkeypatch):
    """--velocity-direction-sigma가 서버 start에서 run.py 인자로 그대로 넘어간다."""
    from experiments.site_sim import server
    seen = {}
    monkeypatch.setattr(server, 'start', lambda *args, **kwargs: seen.update(extra=kwargs['extra_args']))
    assert server.main(['start', 'lbl', '--velocity-direction-sigma', '0.02']) == 0
    assert seen['extra'] == ['--velocity-direction-sigma', '0.02']


def test_server_forwards_the_imu_profile(monkeypatch):
    """--imu-profile이 서버 start에서 run.py 인자로 그대로 넘어간다."""
    from experiments.site_sim import server
    seen = {}
    monkeypatch.setattr(server, 'start', lambda *args, **kwargs: seen.update(extra=kwargs['extra_args']))
    assert server.main(['start', 'lbl', '--imu-profile', 'stress']) == 0
    assert seen['extra'] == ['--imu-profile', 'stress']



def test_build_estimator_wires_imu_profile_into_a_real_estimator():
    """run()의 추정기 생성 줄을 실제로 실행한다(c98862f에서 imu_realism 대입 누락 → NameError였다)."""
    from types import SimpleNamespace
    from experiments.site_sim.run import build_estimator
    pose = np.eye(4)
    common = dict(imu_realism=True, imu_seed=1, imu_drop_prob=.008, adaptive_visual_noise=True, velocity_direction_sigma=None, visual_turn_scale=None)
    nominal = build_estimator(SimpleNamespace(imu_profile='nominal', **common), pose, pose, CALIBRATION)
    stress = build_estimator(SimpleNamespace(imu_profile='stress', **common), pose, pose, CALIBRATION)
    assert nominal.imu_realism.gyro_density == pytest.approx(7.2e-5)
    assert stress.imu_realism.gyro_density == pytest.approx(1.1e-4)
    assert nominal.imu_realism.drop_probability == pytest.approx(.008)
    off = build_estimator(SimpleNamespace(**{**common, 'imu_realism': False}, imu_profile='nominal'), pose, pose, CALIBRATION)
    assert off.imu_realism is None


def test_policy_imu_record_describes_the_active_model_and_boot_bias():
    """노이즈가 켜진 정책 실행을 이상적 IMU 실행으로 잘못 기록하지 않는다."""
    from types import SimpleNamespace
    from experiments.site_sim.run import policy_imu_metadata
    model = ImuRealism(seed=2, vibration_scale=.5)
    record = policy_imu_metadata(SimpleNamespace(imu_realism=model, bias_s=2.))
    assert record['imu_for_policy'] == 'physics state with simulated IMU noise and bias'
    assert record['imu_realism'] == model.parameters()
    assert record['gyro_bias_estimation_s'] == 2.
    off = policy_imu_metadata(SimpleNamespace(imu_realism=None, bias_s=2.))
    assert off['imu_for_policy'] == 'physics state, no noise or bias'
    assert off['imu_realism'] is None


def test_server_forwards_follower_flags(monkeypatch):
    """--follower·--blend-max-yaw-rate가 서버 start에서 run.py 인자로 넘어가고, 모르는 추종기 이름은 argparse가 거부한다."""
    from experiments.site_sim import server
    seen = {}
    monkeypatch.setattr(server, 'start', lambda *args, **kwargs: seen.update(extra=kwargs['extra_args']))
    assert server.main(['start', 'lbl', '--follower', 'blend', '--blend-max-yaw-rate', '0.09']) == 0
    assert seen['extra'] == ['--blend-max-yaw-rate', '0.09', '--follower', 'blend']
    with pytest.raises(SystemExit):
        server.main(['start', 'lbl', '--follower', 'foo'])


def test_apply_follower_args_sets_and_validates_blend_settings(mission):
    from types import SimpleNamespace
    from experiments.site_sim.run import apply_follower_args
    assert mission.settings['follower_blend'] == 0
    apply_follower_args(mission, SimpleNamespace(follower=None, blend_max_yaw_rate=None))
    apply_follower_args(mission, SimpleNamespace(follower='unicycle', blend_max_yaw_rate=None))
    assert mission.settings['follower_blend'] == 0 and mission.settings['blend_max_yaw_rate_rad_s'] == .09
    apply_follower_args(mission, SimpleNamespace(follower='blend', blend_max_yaw_rate=.05))
    assert mission.settings['follower_blend'] == 1. and mission.settings['blend_max_yaw_rate_rad_s'] == .05
    for bad in (.7, 0., -.1, float('nan')):
        with pytest.raises(ValueError):
            apply_follower_args(mission, SimpleNamespace(follower='blend', blend_max_yaw_rate=bad))


def test_rtx_setting_overrides_default_to_no_change_and_validate_ranges():
    """렌더러 옵션은 안 주면 아무것도 바꾸지 않고, 주면 carb 키로 매핑하며, 범위 밖이면 거부한다."""
    from types import SimpleNamespace
    from experiments.site_sim.run import rtx_setting_overrides
    assert rtx_setting_overrides(SimpleNamespace(rtx_aa_op=None, dlss_exec_mode=None)) == []
    assert rtx_setting_overrides(SimpleNamespace()) == []
    assert rtx_setting_overrides(SimpleNamespace(rtx_aa_op=2, dlss_exec_mode=None)) == [('/rtx/post/aa/op', 2)]
    assert rtx_setting_overrides(SimpleNamespace(rtx_aa_op=None, dlss_exec_mode=2)) == [('/rtx/post/dlss/execMode', 2)]
    for bad in (SimpleNamespace(rtx_aa_op=5), SimpleNamespace(rtx_aa_op=-1), SimpleNamespace(dlss_exec_mode=4), SimpleNamespace(dlss_exec_mode=1.5)):
        with pytest.raises(ValueError):
            rtx_setting_overrides(bad)


def test_server_forwards_rtx_flags(monkeypatch):
    """--rtx-aa-op·--dlss-exec-mode가 서버 start에서 run.py 인자로 넘어가고, 안 주면 넘기지 않는다."""
    from experiments.site_sim import server
    seen = {}
    monkeypatch.setattr(server, 'start', lambda *args, **kwargs: seen.update(extra=kwargs['extra_args']))
    assert server.main(['start', 'lbl', '--rtx-aa-op', '2']) == 0
    assert seen['extra'] == ['--rtx-aa-op', '2']
    assert server.main(['start', 'lbl', '--dlss-exec-mode', '2']) == 0
    assert seen['extra'] == ['--dlss-exec-mode', '2']
    assert server.main(['start', 'lbl']) == 0
    assert seen['extra'] == []


def run_py_args(monkeypatch, tmp_path, *extra):
    """run.py main()의 실제 argparse로 인자를 파싱한다(run()만 대체해 인자를 가로챈다)."""
    import sys
    from experiments.site_sim import run as run_module
    seen = {}
    (tmp_path/'out').mkdir()   # main()이 출력 폴더의 부모에 exit.txt를 쓴다
    monkeypatch.setattr(run_module, 'run', lambda args: seen.update(args=args) or {'status': 'complete'})
    monkeypatch.setattr(sys, 'argv', ['run.py', '--mission', str(tmp_path/'mission'), '--urdf', 'u', '--geometry', 'g',
                                      '--scene-usd', 's', '--output', str(tmp_path/'out'/'run'), *extra])
    with pytest.raises(SystemExit):
        run_module.main()
    return seen['args']


@pytest.mark.parametrize('flags,blend', [(('--follower', 'blend', '--blend-max-yaw-rate', '0.05'), True), ((), False)])
def test_run_py_flags_reach_a_stepping_drive(monkeypatch, tmp_path, mission, flags, blend):
    """run.py의 argparse → load_mission → 실제 SimulationDrive.step까지 통과해 vy와 follower가 텔레메트리에 나온다."""
    from experiments.site_sim.run import load_mission
    mission.save(tmp_path/'mission')
    args = run_py_args(monkeypatch, tmp_path, *flags)
    loaded = load_mission(args)
    assert loaded.settings['follower_blend'] == (1. if blend else 0.)
    row = SimulationDrive(Robot(), Session(), loaded).step(.1)
    if blend:
        assert loaded.settings['blend_max_yaw_rate_rad_s'] == .05
        assert 'vy_m_s' in row['requested_command'] and 'follower' in row['mission']
        assert abs(row['requested_command']['yaw_rate_rad_s']) <= .05+1e-12
    else:
        assert 'vy_m_s' not in row['requested_command'] and 'follower' not in row['mission']


def test_viewer_option_text_shows_follower(tmp_path):
    from experiments.site_sim.viewer import run_options_text
    (tmp_path/'run_options.json').write_text(json.dumps({'follower': 'blend'}))
    assert '추종기 blend' in run_options_text(tmp_path)
    (tmp_path/'run_options.json').write_text(json.dumps({'follower': None, 'blend_max_yaw_rate': None}))
    assert '추종기' not in run_options_text(tmp_path)
    (tmp_path/'run_options.json').write_text(json.dumps({'follower': 'blend', 'blend_max_yaw_rate': .05}))
    assert 'blend ω 0.05' in run_options_text(tmp_path)


def test_filter_noise_overrides_scale_gyro_density_and_default_is_unchanged():
    """--filter-acc-noise는 절댓값, --filter-gyro-noise-scale은 DEFAULT_NOISE 기준 배율이며 미지정이면 아무것도 바꾸지 않는다."""
    from types import SimpleNamespace
    from experiments.localization.online import DEFAULT_NOISE
    from experiments.site_sim.run import filter_noise_overrides
    assert filter_noise_overrides(SimpleNamespace()) == {}
    assert filter_noise_overrides(SimpleNamespace(filter_acc_noise=None, filter_gyro_noise_scale=None)) == {}
    out = filter_noise_overrides(SimpleNamespace(filter_acc_noise=.03, filter_gyro_noise_scale=20.))
    assert out == {'IMU.NoiseAcc': .03, 'IMU.NoiseGyro': pytest.approx(DEFAULT_NOISE['IMU.NoiseGyro'] * 20)}
    only_gyro = filter_noise_overrides(SimpleNamespace(filter_gyro_noise_scale=20.))
    assert set(only_gyro) == {'IMU.NoiseGyro'}
    with pytest.raises(ValueError):
        filter_noise_overrides(SimpleNamespace(filter_gyro_noise_scale=0.))


def test_server_forwards_the_filter_gyro_noise_scale(monkeypatch):
    """--filter-gyro-noise-scale이 서버 start에서 run.py 인자로 그대로 넘어간다."""
    from experiments.site_sim import server
    seen = {}
    monkeypatch.setattr(server, 'start', lambda *args, **kwargs: seen.update(extra=kwargs['extra_args']))
    assert server.main(['start', 'lbl', '--filter-gyro-noise-scale', '20']) == 0
    assert seen['extra'] == ['--filter-gyro-noise-scale', '20.0']


def test_build_estimator_enables_the_gyro_bias_init_from_the_start_hold():
    """--filter-gyro-bias-init-sigma-dps가 있으면 start_hold_s 동안 평균으로 bias를 초기화하는 추정기를 만든다(없으면 꺼짐)."""
    from types import SimpleNamespace
    from experiments.site_sim.run import build_estimator
    pose = np.eye(4)
    common = dict(imu_realism=False, adaptive_visual_noise=True, velocity_direction_sigma=None, visual_turn_scale=None, start_hold_s=3.)
    off = build_estimator(SimpleNamespace(**common), pose, pose, CALIBRATION)
    assert off.gyro_bias_init_hold_s is None
    on = build_estimator(SimpleNamespace(filter_gyro_bias_init_sigma_dps=.03, **common), pose, pose, CALIBRATION)
    assert on.gyro_bias_init_hold_s == 3. and on.gyro_bias_sigma_dps == .03


def test_server_forwards_the_gyro_bias_init_sigma(monkeypatch):
    """--filter-gyro-bias-init-sigma-dps가 서버 start에서 run.py 인자로 그대로 넘어간다."""
    from experiments.site_sim import server
    seen = {}
    monkeypatch.setattr(server, 'start', lambda *args, **kwargs: seen.update(extra=kwargs['extra_args']))
    assert server.main(['start', 'lbl', '--filter-gyro-bias-init-sigma-dps', '0.03']) == 0
    assert seen['extra'] == ['--filter-gyro-bias-init-sigma-dps', '0.03']
