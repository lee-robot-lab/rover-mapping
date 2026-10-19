# ================================================================
# test_follower_metrics.py
# 설명: 합성 텔레메트리로 follower_metrics의 GT 각속도·제외 구간·추정 오차·게이트 정지·추정기 degraded·병합 지표를 검사한다.
# 사용법: KSRC_ROOT=outputs/site_sim/ksrc_20261016 .venv-mapping/bin/python -m pytest tests/site_sim/test_follower_metrics.py -q
# ================================================================
import json
import math

import pytest

HZ = 50


def make_run(root, name, seconds, *, yaw=lambda t: 0., speed=lambda t: .05, vy=None, follower=None, error=None,
             phase=lambda t: ('ascending', '30'), status=None, quality='good', old=False):
    """50 Hz 합성 telemetry.jsonl을 가진 run 폴더를 만든다. old=True면 follower·vy 키를 뺀 옛 형식."""
    out = root / name
    out.mkdir(parents=True)
    with (out / 'telemetry.jsonl').open('w') as stream:
        for k in range(int(seconds * HZ) + 1):
            t = k / HZ
            psi = yaw(t)
            row = {'stamp_s': t, 'position_m': [0., 0., 0.], 'quaternion_wxyz': [math.cos(psi / 2), 0., 0., math.sin(psi / 2)],
                   'applied_command': {'vx_m_s': speed(t), **({} if old or vy is None else {'vy_m_s': vy(t)})},
                   'mission': {'state': phase(t)[0], 'route': phase(t)[1]},
                   'estimator_status': 'ok' if status is None else status(t), 'estimator_quality': quality}
            if error is not None:
                row['estimated_position_m'] = [error(t), 0., 0.]
            if follower is not None and not old:
                row['mission']['follower'] = follower(t)
            stream.write(json.dumps(row) + '\n')
    return out


def follower_row(gap=0., gated=False, realigning=False):
    return {'psi_ref': 0., 'heading_gap_rad': gap, 'travel_gap_rad': gap, 'gated': gated, 'realigning': realigning}


@pytest.mark.parametrize('rate,fraction', [(.2, 1.0), (.05, 0.0)])
def test_constant_rotation_gives_expected_omega(tmp_path, rate, fraction):
    from experiments.site_sim.follower_metrics import summarize
    s = summarize(make_run(tmp_path, 'r', 10, yaw=lambda t: rate * t))
    assert s['omega_mean_rad_s'] == pytest.approx(rate, rel=1e-6) and s['over_limit_fraction'] == fraction
    assert s['omega_max_rad_s'] == pytest.approx(rate, rel=1e-6)


def test_wrap_at_pi_does_not_create_spike(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    s = summarize(make_run(tmp_path, 'r', 20, yaw=lambda t: math.atan2(math.sin(3.0 + .05 * t), math.cos(3.0 + .05 * t))))
    assert s['omega_max_rad_s'] < .06


def test_heading_gap_over_60deg_is_excluded(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    yaw = lambda t: .05 * t + (.5 * (t - 10) if 10 <= t < 20 else .5 * 10 if t >= 20 else 0.)   # 중간 10 s만 ω .55
    follower = lambda t: follower_row(math.pi / 2 if 10 <= t < 20 else 0.)
    s = summarize(make_run(tmp_path, 'r', 30, yaw=yaw, follower=follower))
    assert s['excluded_s'] == pytest.approx(10, abs=.1) and s['over_limit_fraction'] < .03   # 0.5 s 평활이 경계 0.25 s를 번지게 한다
    kept = summarize(make_run(tmp_path, 'k', 30, yaw=yaw, follower=lambda t: follower_row(0.)))   # 제외하지 않으면 걸린다
    assert kept['over_limit_fraction'] > .25 and kept['excluded_s'] == 0


def test_old_rows_without_follower_or_vy(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    s = summarize(make_run(tmp_path, 'r', 5, yaw=lambda t: .02 * t, old=True))
    assert s['moving_s'] == pytest.approx(5, abs=.1) and s['excluded_s'] == 0
    assert s['gate_stop_s_by_phase'] == {} and s['gate_resume_by_phase'] == {} and s['gate_stop_s_total'] == 0
    json.dumps(s)


def test_not_moving_rows_are_not_counted(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    s = summarize(make_run(tmp_path, 'r', 20, yaw=lambda t: 1.0 * t if t < 10 else 10.0, speed=lambda t: 0. if t < 10 else .05))
    assert s['over_limit_fraction'] < .03 and s['moving_s'] == pytest.approx(10, abs=.1)   # 정지 구간의 ω 1.0은 세지 않는다(경계 평활 번짐만)
    moving = summarize(make_run(tmp_path, 'm', 20, yaw=lambda t: 1.0 * t if t < 10 else 10.0, speed=lambda t: .05))
    assert moving['over_limit_fraction'] > .4


def test_sideways_translation_counts_as_moving(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    s = summarize(make_run(tmp_path, 'r', 5, speed=lambda t: 0., vy=lambda t: .05))
    assert s['moving_s'] == pytest.approx(5, abs=.1)


def test_rotation_after_start_and_estimate_error(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    yaw = lambda t: 0. if t < 5 else math.radians(30) * min(1., (t - 5) / 5)
    s = summarize(make_run(tmp_path, 'r', 12, yaw=yaw, speed=lambda t: 0. if t < 3 else .05, error=lambda t: .14 if 7 < t < 8 else .01))
    assert s['rotation_after_start_deg'] == pytest.approx(30, abs=.01) and s['max_estimate_error_m'] == pytest.approx(.14)
    assert summarize(make_run(tmp_path, 'n', 3))['max_estimate_error_m'] is None   # 추정 위치가 없는 run


def test_gate_stop_time_and_resume_count_per_phase(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    def gated(t):   # 8 s 정지, 4 s 이동, 5 s 정지, 이동
        return 2 <= t < 10 or 14 <= t < 19
    phase = lambda t: ('ascending', '15') if t < 25 else ('descending', '10')
    s = summarize(make_run(tmp_path, 'r', 30, follower=lambda t: follower_row(gated=gated(t)), phase=phase))
    assert s['gate_stop_s_by_phase']['ascending:15#0'] == pytest.approx(13, abs=.1)
    assert s['gate_resume_by_phase']['ascending:15#0'] == 2
    assert s['gate_stop_s_by_phase'].get('descending:10#0', 0) == 0 and s['gate_resume_by_phase'].get('descending:10#0', 0) == 0
    assert s['gate_stop_s_total'] == pytest.approx(13, abs=.1)


def test_same_phase_name_twice_gets_separate_segments(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    phase = lambda t: ('ascending', '30') if t < 5 or t >= 10 else ('retreating', '30')
    s = summarize(make_run(tmp_path, 'r', 15, follower=lambda t: follower_row(gated=True), phase=phase))
    assert set(s['gate_stop_s_by_phase']) == {'ascending:30#0', 'retreating:30#0', 'ascending:30#1'}


def test_realign_stop_time_is_reported_separately_from_the_ramp_gate(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    s = summarize(make_run(tmp_path, 'r', 10, follower=lambda t: follower_row(realigning=2 <= t < 4.5)))
    assert s['realign_stop_s'] == pytest.approx(2.5, abs=.05) and s['gate_stop_s_total'] == 0
    assert summarize(make_run(tmp_path, 'o', 3, old=True))['realign_stop_s'] == 0


def test_estimator_degraded_time(tmp_path):
    from experiments.site_sim.follower_metrics import summarize
    status = lambda t: 'imu_gap' if 1 <= t < 3 else 'recovering' if 5 <= t < 6 else 'ok'
    s = summarize(make_run(tmp_path, 'r', 10, status=status))
    assert s['estimator_degraded_s'] == pytest.approx(3, abs=.05)
    assert summarize(make_run(tmp_path, 'q', 10, quality='degraded'))['estimator_degraded_s'] == pytest.approx(10, abs=.05)


def test_clearance_fields_are_merged(tmp_path, monkeypatch):
    from experiments.site_sim import follower_metrics
    seen = []
    monkeypatch.setattr(follower_metrics.clearance, 'evaluate',
                        lambda output, mission: seen.append((output, mission)) or {'completed': True, 'min_clearance_m': .31, 'contact_samples': 0,
                                                                                  'duration_sim_s': 250., 'stuck_events': 3})
    run = make_run(tmp_path, 'r', 3)
    s = follower_metrics.summarize(run, tmp_path / 'mission')
    assert (s['completed'], s['min_clearance_m'], s['contact_samples'], s['duration_sim_s']) == (True, .31, 0, 250.)
    assert 'stuck_events' not in s and seen == [(run, tmp_path / 'mission')]
    assert 'completed' not in follower_metrics.summarize(run)   # mission_folder가 없으면 clearance를 부르지 않는다


def test_format_table_lists_each_run_and_metric_and_main_prints_it(tmp_path, monkeypatch, capsys):
    from experiments.site_sim import follower_metrics
    monkeypatch.setattr(follower_metrics.clearance, 'evaluate',
                        lambda output, mission: {'completed': True, 'min_clearance_m': .31, 'contact_samples': 0, 'duration_sim_s': 250.})
    a = make_run(tmp_path, 'uni-s1', 5, yaw=lambda t: .2 * t)
    b = make_run(tmp_path, 'bln-s1', 5, follower=lambda t: follower_row(gated=t < 1))
    table = follower_metrics.format_table({p.name: follower_metrics.summarize(p) for p in (a, b)})
    assert 'uni-s1' in table and 'bln-s1' in table and 'over_limit_fraction' in table
    assert follower_metrics.main([str(tmp_path / 'mission'), str(a), str(b)]) == 0
    out = capsys.readouterr().out
    assert 'uni-s1' in out and 'bln-s1' in out and 'min_clearance_m' in out
