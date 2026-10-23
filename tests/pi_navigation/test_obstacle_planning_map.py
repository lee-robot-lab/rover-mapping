# ================================================================
# test_obstacle_planning_map.py
# 설명: 배포 MPPI가 선택된 rocks10 장애물 격자를 읽고 회차 설정은 보존하는지 확인한다.
# 사용법: python -m pytest tests/pi_navigation/test_obstacle_planning_map.py -q
# ================================================================
import json
import numpy as np
from test_runner import make_runner,DEPLOY


def test_deployed_planning_map_contains_selected_stones_and_preserves_mission():
    meta=json.loads((DEPLOY/'mission/planning_map.json').read_text())
    nav=make_runner();site=nav.mission.site
    assert meta['name']=='rocks10' and meta['stone_diameter_m']==.10
    assert site.identity==meta['site_identity']
    assert site.metadata['source_sha256']==meta['source_sha256']
    cells=site.cells(np.asarray(meta['added_blocked_centers_xy_m']))
    assert len(cells)==56
    assert site.blocked[cells[:,1],cells[:,0]].all()
    assert nav.mission.validate()['accepted']
    assert 'descent15' in nav.scenario['goals'] and 'outer4' in nav.scenario['goals']
    assert np.allclose(nav.mission.start,[3.6,2.82,-np.pi/2])
