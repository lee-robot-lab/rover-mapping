# ================================================================
# test_online_replay.py
# 설명: 기록된 IMU·영상 이벤트 재생이 원래 추정을 복원하고 GT는 평가에만 쓰는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_online_replay.py -q
# ================================================================
import json
from pathlib import Path

import numpy as np
import pytest

from tests.localization.test_online import CALIBRATION


def test_sensor_event_replay_matches_recording_and_uses_gt_only_for_evaluation(tmp_path: Path) -> None:
    from experiments.site_sim.vio_offline import replay_online
    root = tmp_path/'run'
    root.mkdir()
    (root/'estimator_calibration.json').write_text(json.dumps({
        'calibration': CALIBRATION, 'start_world_from_base': np.eye(4).tolist(), 'render_lag_s': 0.}))
    events = []
    for t, x in ((.05, .00125), (.1, .005), (.15, .01125), (.2, .02)):
        events.append({'kind': 'imu', 'stamp_s': t, 'dt_s': .05,
                       'gyro': [0., 0., 0.], 'force': [1., 0., 9.81]})
        if t in (.05, .15):
            camera = np.eye(4)
            camera[0, 3] = x
            events.append({'kind': 'frame', 'stamp_s': t, 'orb_state': 2, 'map_id': '0',
                           'world_from_camera': camera.tolist()})
        events.append({'kind': 'pose', 'stamp_s': t, 'position_m': [x, 0., 0.],
                       'quaternion_wxyz': [1., 0., 0., 0.], 'estimated_position_m': [x, 0., 0.]})
    path = root/'estimator_events.jsonl'
    path.write_text(''.join(json.dumps(e)+'\n' for e in events))
    report = replay_online(root, tmp_path/'baseline')
    assert report['last_position_m'] == pytest.approx([.02, 0., 0.], abs=1e-12)
    assert report['visual_accepted'] == 1
    assert report['xyz_rmse_m'] < 1e-12
    assert report['recorded_estimate_max_diff_m'] < 1e-12
    for event in events:
        if event['kind'] == 'pose':
            event['position_m'][0] += 10.
    path.write_text(''.join(json.dumps(e)+'\n' for e in events))
    changed = replay_online(root, tmp_path/'changed_gt')
    assert changed['last_position_m'] == pytest.approx(report['last_position_m'], abs=1e-12)
    assert changed['xyz_rmse_m'] == pytest.approx(10.)
