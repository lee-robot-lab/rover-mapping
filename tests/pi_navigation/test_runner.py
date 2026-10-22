# ================================================================
# test_runner.py
# 설명: 최종 실행기의 정지·복구·UDP·임무 시작 동작을 모터 없이 검증한다.
# 사용법: python -m pytest tests/pi_navigation -q
# ================================================================
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
DEPLOY = ROOT / 'deployment/ksrc'
sys.path.insert(0, str(DEPLOY))


def make_runner():
    import runner
    config = {
        'calibration': {'ROS.T_c1_base': np.eye(4).tolist(), 'IMU.T_b_c1': np.eye(4).tolist(),
                        'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .1,
                        'IMU.GyroWalk': 1e-12, 'IMU.AccWalk': .003},
        'start_z_m': .455, 'lag_s': .1,
        'imu': {'gyro_bias_dps': [0, 0, 0], 'acc_offset_g': [0, 0, 0], 'acc_scale': [1, 1, 1]},
        'wall': None,
    }
    return runner.Navigation(DEPLOY / 'mission', config)


def feed(nav, end, start=0.):
    for i in range(round(start * 50) + 1, round(end * 50) + 1):
        t = i / 50
        nav.imu(t, .02, [0, 0, 0], [0, 0, 9.81], 0.)
        if i % 3 == 0:
            nav.frame(t, np.eye(4), 'orb_map_0', 2)
        nav.tick(t)


def test_cold_start_does_not_drive():
    nav = make_runner()
    assert nav.tick(0.)['command'] == (0., 0., 0.)
    assert not nav.status(0.)['orb_ok']
    assert not nav.start(1, 0.)


def test_start_and_stop_require_explicit_request():
    nav = make_runner()
    feed(nav, 3.5)
    assert nav.status(3.5)['stationary_s'] >= 3
    assert nav.start(1, 3.5)
    nav.stop('operator')
    feed(nav, 4., 3.5)
    assert not nav.active
    assert nav.tick(4.)['command'] == (0., 0., 0.)


def test_gap_stops_and_recovery_preserves_pose_and_stage():
    nav = make_runner()
    feed(nav, 3.5)
    assert nav.start(1, 3.5)
    stage = nav.runtime.stage
    checkpoint = nav.checkpoint.copy()
    for i in range(176, 216):
        nav.imu(i / 50, .02, [0, 0, 0], [0, 0, 9.81], 0.)
        out = nav.tick(i / 50)
    assert out['command'] == (0., 0., 0.)
    assert nav.recovering
    # ORB의 새 원점이 100 m 바뀌어도 site pose는 checkpoint에 재연결한다.
    moved_origin = np.eye(4)
    moved_origin[:3, 3] = [100, -20, 5]
    for i in range(216, 296):
        t = i / 50
        nav.imu(t, .02, [0, 0, 0], [0, 0, 9.81], 0.)
        if i % 3 == 0:
            nav.frame(t, moved_origin, 'orb_map_1', 2)
        nav.tick(t)
    assert not nav.recovering
    assert nav.active
    assert nav.runtime.stage == stage
    assert np.linalg.norm(nav.checkpoint[:2, 3] - checkpoint[:2, 3]) < .02


def test_duplicate_imu_does_not_extend_freshness():
    nav = make_runner()
    nav.imu(.02, .02, [0, 0, 0], [0, 0, 9.81], 0.)
    assert not nav.imu(.02, .02, [0, 0, 0], [0, 0, 9.81], 0.)
    assert not nav.status(1.)['orb_ok']


def test_run2_skips_trial_and_slows_descent():
    nav = make_runner()
    feed(nav, 3.5)
    assert nav.start(2, 3.5)
    assert nav.runtime.stage == 'entry15'
    assert nav.runtime.settings['descent_speed_scale'] == .7
    assert nav.runtime.settings['summit_look'] == 0


def test_imu_gap_stops_and_requires_stable_recovery():
    nav = make_runner()
    feed(nav, 3.5)
    assert nav.start(1, 3.5)
    assert nav.tick(3.7)['command'] == (0., 0., 0.)
    assert nav.recovering


def test_new_start_after_estop_requires_start_pose_alignment():
    nav = make_runner()
    feed(nav, 3.5)
    nav.checkpoint[0, 3] -= .3
    assert not nav.start(1, 3.5)


def test_missing_imu_time_is_not_counted_as_stationary():
    nav = make_runner()
    feed(nav, 3.5)
    nav.imu(10., .02, [0, 0, 0], [0, 0, 9.81], 0.)
    assert nav.status(10.)['stationary_s'] == 0.


def test_second_gap_during_recovery_requires_a_new_stable_window():
    nav = make_runner()
    feed(nav, 3.5)
    assert nav.start(1, 3.5)
    nav.fail_tracking(3.5, 'pose_gap')
    feed(nav, 4.1, 3.5)
    assert nav.frame_count >= 5
    for i in range(206, 241):
        t = i / 50
        nav.imu(t, .02, [0, 0, 0], [0, 0, 9.81], 0.)
        nav.tick(t)
    nav.imu(4.82, .02, [0, 0, 0], [0, 0, 9.81], 0.)
    nav.frame(4.82, np.eye(4), 'orb_map_0', 2)
    assert nav.tick(4.82)['command'] == (0., 0., 0.)
    assert nav.recovering


def test_async_worker_exits_even_if_parent_installed_sigterm_handler():
    import os
    import signal
    nav = make_runner()
    feed(nav, 3.5)
    nav.plan_cpu = str(min(os.sched_getaffinity(0)))
    previous = signal.signal(signal.SIGTERM, lambda *_: None)
    worker = None
    try:
        assert nav.start(1, 3.5)
        worker = nav.async_plan.process
        nav.close()
        assert not worker.is_alive()
    finally:
        signal.signal(signal.SIGTERM, previous)
        if worker is not None and worker.is_alive():
            worker.kill()
            worker.join(timeout=2)


def test_vo_has_existing_wire_format():
    import runner
    import struct
    packet = runner.pack_vo(7, 123456789, .1, -.02, True)
    magic, seq, ns, vx, vy, valid = struct.unpack('<4sIqffI', packet)
    assert (magic, seq, ns, valid) == (b'KVO1', 7, 123456789, 1)
    assert vx == pytest.approx(.1)
    assert vy == pytest.approx(-.02)
