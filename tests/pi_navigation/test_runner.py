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


def test_anchor_count_increases_only_when_anchor_is_accepted():
    nav = make_runner()
    assert not nav.anchor(0.) and nav.status(0.)['anchor_count'] == 0     # 센서·정지 조건이 없으면 거절
    feed(nav, 3.5)
    assert nav.anchor(3.5) and nav.status(3.5)['anchor_count'] == 1
    assert nav.start(1, 3.5)
    assert not nav.anchor(3.6) and nav.status(3.6)['anchor_count'] == 1   # AUTO 중에는 거절


def test_gateway_payload_fits_the_gateway_receive_limit_even_with_worst_values():
    import json
    import runner
    nav = make_runner()
    feed(nav, 3.5)
    payload = runner.gateway_payload(nav.status(3.5))
    assert len(payload) <= 256 and json.loads(payload)['orb_ok'] is True
    worst = dict(orb_ok=True, in_region1=True, done=True, start_aligned=True, active=True, recovering=True, run=2,
                 stage='retreat15', anchor_count=99999, stationary_s=12345.678, reason='x' * 80)
    payload = runner.gateway_payload(worst)
    assert len(payload) <= 256
    assert json.loads(payload)['stationary_s'] == 12345.7 and len(json.loads(payload)['reason']) == 30


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


def test_orb_reset_when_idle_keeps_pose_and_recovers_on_new_map_but_is_refused_in_auto():
    nav = make_runner()
    feed(nav, 3.5)
    checkpoint = nav.checkpoint.copy()
    assert nav.orb_reset(3.5) and nav.recovering and not nav.status(3.5)['orb_ok']
    assert nav.events[-1]['type'] == 'recovery_start' and nav.events[-1]['reason'] == 'operator_orb_reset'
    new_origin = np.eye(4)
    new_origin[:3, 3] = [50, -10, 2]
    for i in range(176, 260):                 # 재시작한 ORB가 새 map(새 원점)으로 pose를 내보낸다
        t = i / 50
        nav.imu(t, .02, [0, 0, 0], [0, 0, 9.81], 0.)
        if i % 3 == 0:
            nav.frame(t, new_origin, 'orb_map_1', 2)
        nav.tick(t)
    assert not nav.recovering and nav.status(5.2)['orb_ok']
    assert np.linalg.norm(nav.checkpoint[:2, 3] - checkpoint[:2, 3]) < .02      # 위치는 직전 checkpoint에서 이어진다
    assert nav.start(1, 5.2)
    assert not nav.orb_reset(5.3) and not nav.recovering                         # AUTO 중에는 거절


def test_orb_restart_is_requested_only_when_frames_are_really_gone():
    import runner
    due = runner.orb_restart_due
    assert not due(False, 100., 0., 50., 0., -1.)                       # 복구 중이 아니다
    assert not due(True, 100., 0., 5., 0., -1.)                         # 복구 시작 5 s: ORB가 스스로 복구할 시간
    assert not due(True, 100., 0., 40., 20., 39.)                       # 프레임이 계속 들어온다(복구가 진행 중)
    assert not due(True, 100., 200., 40., 20., 10.)                     # 재시작 직후 유예 중
    assert due(True, 100., 0., 40., 20., 10.)                           # 20 s 복구 중, 30 s 동안 프레임 없음
    assert due(True, 100., 0., 40., 20., float('-inf'))


def test_slow_starting_orb_is_not_killed_before_its_first_pose():
    """ORB가 재시작 후 첫 pose까지 10 s 걸려도, 자동 재시작이 8 s마다 죽이던 때와 달리 한 번에 복구된다."""
    import runner
    next_restart, restarts, orb_started, orb_stuck, last_frame = 0., 0, None, True, float('-inf')
    recovery_since = 0.                                                     # t=0에 추적 상실, ORB는 멈춘 상태
    for step in range(0, 1200):
        t = step / 10
        if runner.orb_restart_due(True, t, next_restart, t, recovery_since, last_frame):
            restarts, orb_started, orb_stuck, next_restart = restarts + 1, t, False, t + runner.ORB_STARTUP_GRACE_S
        if not orb_stuck and t - orb_started >= 10.:                       # 재시작 후 10 s 뒤부터 프레임을 낸다
            last_frame = t
    assert restarts == 1 and last_frame > 100.


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


def test_runner_status_reaches_the_joystick_screen_end_to_end():
    """runner.status → gateway_payload → Gateway.runner_status → snapshot → GatewayClient.lines. anchor·READY·회차 결과가 화면 문장까지 간다."""
    import json
    import runner
    from ksrc_runtime.handheld_mapping.control_gateway import Gateway
    sys.path.insert(0, str(ROOT / 'teleop/ksrc_snapshot/pi/teleop'))
    import gateway_client

    class Sock:
        def sendto(self, data, dest):
            pass

    nav, gateway, client = make_runner(), Gateway(lambda *command: None), gateway_client.GatewayClient(Sock(), ('pi', 5006))

    def screen(t_nav, t_gateway):
        gateway.runner_status(json.loads(runner.gateway_payload(nav.status(t_nav))), t_gateway)
        client.consume(json.loads(json.dumps(gateway.snapshot(t_gateway + .01))), t_gateway)
        return [text for text, _ in client.lines(t_gateway + .02)]

    feed(nav, 3.5)
    assert 'READY: START allowed' in screen(3.5, 10.)
    client.anchor(10.1)
    assert nav.anchor(3.5)
    assert 'ANCHOR1 -Y applied (current pose = start preset)' in screen(3.5, 10.2)
    assert nav.start(1, 3.5)
    nav.runtime.state, nav.active, nav.reason = 'complete', False, 'complete'
    assert 'RUN 1 COMPLETE - back to TELEOP' in screen(3.5, 11.)
