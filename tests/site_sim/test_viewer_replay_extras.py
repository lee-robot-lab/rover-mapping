# ================================================================
# test_viewer_replay_extras.py
# 설명: 재생 화면에 실행 설정 한 줄과 벽까지 거리(정답 위치 기준)가 정확히 실리는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_viewer_replay_extras.py
# ================================================================
import json
from types import SimpleNamespace

import numpy as np

from experiments.site_sim.viewer import replay_payload, run_options_text


def write_run(folder, positions, options=None):
    """telemetry.jsonl(정답·추정 위치)과 선택적 run_options.json을 가진 최소 run 폴더를 만든다."""
    folder.mkdir()
    rows = [{'stamp_s': .02*k, 'position_m': p+[0.], 'estimated_position_m': [p[0]+.01, p[1]], 'quaternion_wxyz': [1., 0., 0., 0.],
             'mission': {'state': 'ascending', 'route': '15'}} for k, p in enumerate(positions)]
    (folder/'telemetry.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
    if options is not None:
        (folder/'run_options.json').write_text(json.dumps(options))


def test_options_text_lists_only_enabled_settings(tmp_path):
    write_run(tmp_path/'a', [[1., 1.]], {'rl_policy': 'res8_mlp_v1_rc4000', 'imu_realism': True, 'command_delay_ms': [17.6, 46.5],
                                         'mu': .25, 'control_dt': .02, 'estimator': True, 'stanley_gain': 0., 'imu_drop_prob': 0.})
    text = run_options_text(tmp_path/'a')
    for expected in ('RL res8_mlp_v1_rc4000', '실제 IMU', '명령 지연 ms [17.6, 46.5]', 'μ 0.25', '제어 주기 s 0.02', '추정 위치로 주행'):
        assert expected in text
    assert 'Stanley' not in text and 'IMU 누락' not in text
    write_run(tmp_path/'b', [[1., 1.]], {})
    assert run_options_text(tmp_path/'b') == '기본 설정'
    write_run(tmp_path/'c', [[1., 1.]])
    assert run_options_text(tmp_path/'c') == ''


def test_replay_payload_adds_options_and_wall_clearance(tmp_path, monkeypatch):
    blocked = np.zeros((20, 20), bool)
    blocked[10, 10] = True
    site = SimpleNamespace(blocked=blocked, resolution=.1, origin=np.array([0., 0.]),
                           cells=lambda xy: np.floor((np.asarray(xy, float)-np.array([0., 0.]))/.1).astype(int))
    monkeypatch.setattr('experiments.handheld_mapping.site_mission.SiteMission.load', lambda *a, **k: SimpleNamespace(site=site))
    write_run(tmp_path/'run', [[1.35, 1.05], [1.05, 1.05], [5., 5.]], {'rl_policy': 'p', 'control_dt': .02})
    data = replay_payload(tmp_path/'run', mission=tmp_path)
    assert data['options'].startswith('RL p')
    assert data['clear_cm'][0] == 30.0 and data['clear_cm'][1] == 0.0       # 벽 칸 위는 0, 3칸 옆은 30 cm
    assert data['clear_cm'][2] == 0.0                                       # 지도 밖은 충돌(0)로 센다
    assert len(data['clear_cm']) == len(data['t']) == 3
    assert 'clear_cm' not in replay_payload(tmp_path/'run')                  # 임무가 없으면 항목 자체가 없다(서버 뷰어)
