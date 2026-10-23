# ================================================================
# test_teleop_prediction.py
# 설명: 배포 지도에서 수동주행용 표시 MPPI의 방향·안전·정지 숨김을 검증한다.
# 사용법: python -m pytest tests/broadcast_deployment/test_teleop_prediction.py -q
# ================================================================
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope='module')
def site():
    sys.path.insert(0, str(ROOT/'deployment/ksrc'))
    import runner
    from cli import fake_config
    return runner.Navigation(ROOT/'deployment/ksrc/mission', fake_config()).mission.site


@pytest.fixture
def predictor(site):
    name = 'deployment.broadcast.teleop_prediction'
    assert importlib.util.find_spec(name) is not None, 'read-only TELEOP predictor is missing'
    from deployment.broadcast.teleop_prediction import TeleopPrediction
    return TeleopPrediction(site)


def test_forward_prediction_uses_current_pose_and_safe_real_candidates(predictor, site):
    pose = dict(x=3.6, y=2.75, yaw=-math.pi/2, v=.1, w=0., speed_m_s=.1)
    original = pose.copy()
    result = predictor.predict(pose)
    assert pose == original
    assert result is not None
    assert result['source'] == 'teleop_model'
    assert result['lookup_distance_m'] == 0.
    assert result['simulated_compute_ms'] >= 0.
    assert 1 <= len(result['candidates']) <= 16
    selected = np.asarray(result['selected'])
    assert np.allclose(selected[0], [3.6, 2.75])
    assert selected[-1, 1] < 2.75
    for path in result['candidates'] + [result['selected']]:
        assert site.validate_path(path, max_slope_deg=35.)['accepted']
    paths = np.asarray(result['candidates'])
    cells = site.cells(paths.reshape(-1, 2)).reshape(paths.shape)
    expected = site.slope_deg[cells[..., 1], cells[..., 0]].mean(axis=1)
    np.testing.assert_allclose(result['costs'], expected)
    assert json.loads(json.dumps(result, allow_nan=False)) == result


def test_reverse_prediction_and_slip_follow_actual_travel_direction(predictor):
    pose = dict(x=3.6, y=1.7, yaw=-math.pi/2, v=.1, w=0.)
    uphill = predictor.predict(pose)
    downhill = predictor.predict({**pose, 'v': -.1})
    assert uphill is not None and downhill is not None
    assert uphill['selected'][-1][1] < pose['y']
    assert downhill['selected'][-1][1] > pose['y']
    assert 0. < downhill['slip'] < uphill['slip'] < .95


@pytest.mark.parametrize('w', [0., .1, -.1])
def test_lateral_translation_uses_travel_heading_without_changing_pose(predictor, w):
    pose = dict(x=3.6, y=1.7, yaw=0., v=0., vy_m_s=.1, w=w)
    original = pose.copy()
    downhill = predictor.predict(pose)
    uphill = predictor.predict({**pose, 'vy_m_s': -.1})
    assert pose == original
    assert downhill is not None and uphill is not None
    assert downhill['source'] == 'teleop_model'
    assert downhill['selected'][-1][1] > pose['y']
    assert uphill['selected'][-1][1] < pose['y']
    assert downhill['slip'] < uphill['slip']
    if w:
        assert (downhill['selected'][-1][0] - pose['x']) * w < 0.
    json.dumps(downhill, allow_nan=False)


def test_thirty_degree_sand_is_available_outside_mission_cache(predictor):
    result = predictor.predict(dict(x=2.75, y=2., yaw=-math.pi/2, v=.1, w=0.))
    assert result is not None
    assert max(result['costs']) > 25.


@pytest.mark.parametrize('changes', [dict(v=0.), dict(v=.009, w=.3), dict(x=-1.),
                                    dict(x=20.), dict(yaw=float('nan')),
                                    dict(vy_m_s=float('nan'))])
def test_stopped_outside_and_invalid_poses_hide_prediction(predictor, changes):
    pose = dict(x=3.6, y=2.75, yaw=-math.pi/2, v=.1, w=0.)
    assert predictor.predict({**pose, **changes}) is None


def test_identical_pose_reuses_result_but_next_outside_pose_does_not(predictor):
    pose = dict(x=3.6, y=2.75, yaw=-math.pi/2, v=.1, w=0.)
    result = predictor.predict(pose)
    assert result is not None
    assert predictor.predict(pose) == result
    assert predictor.predict({**pose, 'x': -1.}) is None


def test_slow_turn_target_uses_at_most_three_seconds(predictor,monkeypatch):
    targets=[];original=predictor.planner.plan
    def capture(xy,yaw,target,*args,**kwargs):
        targets.append(np.asarray(target)-xy)
        return original(xy,yaw,target,*args,**kwargs)
    monkeypatch.setattr(predictor.planner,'plan',capture)
    predictor.predict(dict(x=3.6,y=1.7,yaw=-math.pi/2,v=.01,w=.3))
    assert np.linalg.norm(targets[0])<=.03+1e-6
