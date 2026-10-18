# ================================================================
# test_mission_connector_waypoints.py
# 설명: 명시한 연결 점열의 보존·안전 검증·파일 왕복·실행기 전달을 검사한다.
# 사용법: python -m pytest tests/handheld_mapping/test_mission_connector_waypoints.py -q
# ================================================================
import json

import numpy as np
import pytest

from site_mission_helpers import mission_fixture
from experiments.handheld_mapping.site_mission import SiteMission, dense_path
from experiments.handheld_mapping.mission_runtime import MissionRuntime


PATH = np.array([[.5, .5], [1.5, .5], [1.5, 1.5], [.5, 1.5]])


def mission_with_connector(points):
    """독립 평면 지도 fixture에 선택 연결 점열만 추가한다."""
    old = mission_fixture()
    return SiteMission(old.site, old.start, old.junction, old.summit, old.routes,
                       old.settings, old.site_name, connector_waypoints=points)


def test_explicit_connector_preserves_corners_and_owns_its_array():
    points = PATH.copy()
    mission = mission_with_connector(points)
    points[1] = [3., 2.]
    assert np.array_equal(mission.connector(), PATH)
    returned = mission.connector()
    returned[1] = [3., 2.]
    assert np.array_equal(mission.connector(), PATH)
    assert mission.validate()['accepted']


def test_runtime_uses_the_explicit_connector_without_shortcutting():
    mission = mission_with_connector(PATH)
    runtime = MissionRuntime(mission)
    assert np.array_equal(runtime.points, dense_path(PATH))


def test_connector_roundtrip_and_old_schema_without_optional_field(tmp_path):
    explicit = mission_with_connector(PATH)
    folder = tmp_path / 'explicit'
    explicit.save(folder)
    data = json.loads((folder / 'mission.json').read_text())
    assert data['connector_waypoints'] == PATH.tolist()
    assert np.array_equal(SiteMission.load(folder).connector(), PATH)
    old = mission_fixture()
    old.save(tmp_path / 'legacy')
    legacy = json.loads((tmp_path / 'legacy/mission.json').read_text())
    assert 'connector_waypoints' not in legacy
    assert np.array_equal(SiteMission.load(tmp_path / 'legacy').connector(), old.connector())


@pytest.mark.parametrize('points', [
    [], [.5, .5], [[.5, .5]], [[.5, .5, 0.], [.5, 1.5, 0.]],
    [[.5, .5], [float('nan'), 1.], [.5, 1.5]],
    [[.6, .5], [.5, 1.5]], [[.5, .5], [.6, 1.5]],
    [[.5, .5], [.5, .5], [.5, 1.5]],
])
def test_malformed_explicit_connector_is_rejected(points, tmp_path):
    mission = mission_with_connector(points)
    with pytest.raises(ValueError, match='connector'):
        mission.connector()
    assert not mission.validate()['accepted']
    with pytest.raises(ValueError):
        mission.save(tmp_path / 'bad')
    assert not (tmp_path / 'bad').exists()


def test_safe_waypoints_do_not_allow_crossing_an_unsafe_segment():
    mission = mission_with_connector(PATH)
    mission.site.observed[4:7, 9:12] = False
    slope = mission.settings['normal_slope_deg']
    assert all(mission.site.validate_path([p], slope)['accepted'] for p in PATH)
    with pytest.raises(ValueError, match='connector'):
        mission.connector()


def test_load_rejects_an_unsafe_serialized_connector(tmp_path):
    mission = mission_fixture()
    mission.save(tmp_path / 'mission')
    path = tmp_path / 'mission/mission.json'
    data = json.loads(path.read_text())
    data['connector_waypoints'] = [[.5, .5], [-1., .5], [.5, 1.5]]
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='connector|validation'):
        SiteMission.load(path.parent)
