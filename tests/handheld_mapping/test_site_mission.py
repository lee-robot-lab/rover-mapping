# ================================================================
# test_site_mission.py
# 설명: 공동 정상·대표점·방향별 경로·설정·원본 hash의 임무 계약을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_site_mission.py -q
# ================================================================
import json
import numpy as np
import pytest
from site_mission_helpers import mission_fixture


def test_routes_share_summit_and_descent_reverses_only_selected_route():
    mission=mission_fixture()
    assert mission.validate()['accepted']
    assert np.allclose(mission.path('10','down')[0],[3.5,1.5])
    assert np.allclose(mission.path('10','down')[-1],[.5,1.5])
    assert np.allclose(mission.connector()[0],[.5,.5])
    assert np.allclose(mission.connector()[-1],[.5,1.5])


def test_roundtrip_source_and_directed_csv(tmp_path):
    from experiments.handheld_mapping.site_mission import SiteMission
    original=mission_fixture();folder=tmp_path/'mission';original.save(folder)
    loaded=SiteMission.load(folder)
    assert loaded.site.identity==original.site.identity
    assert loaded.settings==original.settings
    assert np.array_equal(loaded.routes['30'],original.routes['30'])
    assert (folder/'routes/10_down.csv').is_file()
    assert (folder/'routes/30_up.csv').is_file()
    with pytest.raises(FileExistsError):original.save(folder)
    data=json.loads((folder/'mission.json').read_text());data['site_identity']='wrong'
    (folder/'mission.json').write_text(json.dumps(data))
    with pytest.raises(ValueError,match='site'):SiteMission.load(folder)


@pytest.mark.parametrize('change',['endpoint','nan','route_missing','outside','duplicate','short_trial'])
def test_invalid_routes_rejected(change):
    mission=mission_fixture()
    if change=='endpoint':mission.routes['10'][-1]=[3.,1.5]
    if change=='nan':mission.routes['15'][1]=[np.nan,1.]
    if change=='route_missing':del mission.routes['15']
    if change=='outside':mission.routes['10'][1]=[-1.,1.]
    if change=='duplicate':mission.routes['10'][1]=mission.routes['10'][0]
    if change=='short_trial':mission.routes['30']=mission.routes['30'][[0,-1]]
    assert not mission.validate()['accepted']


def test_slope_exception_applies_only_to_trial_and_never_to_unknown():
    mission=mission_fixture();mission.site.slope_deg[:]=30.
    report=mission.validate()
    assert report['routes']['30']['accepted']
    assert not report['routes']['15']['accepted']
    mission.site.observed[14:17,15:18]=False
    assert not mission.validate()['routes']['30']['accepted']


@pytest.mark.parametrize('name,value',[('max_speed_m_s',0),('normal_slope_deg',90),
    ('pose_max_age_s',float('nan')),('unknown_setting',1),('max_attempt_s',.1)])
def test_invalid_settings_raise(name,value):
    from experiments.handheld_mapping.site_mission import SiteMission
    old=mission_fixture()
    with pytest.raises(ValueError):SiteMission(old.site,old.start,old.junction,old.summit,old.routes,{name:value})


def test_save_rejects_invalid_before_creating_output(tmp_path):
    mission=mission_fixture();mission.routes['10'][1]=[-1,0]
    with pytest.raises(ValueError):mission.save(tmp_path/'bad')
    assert not (tmp_path/'bad').exists()
