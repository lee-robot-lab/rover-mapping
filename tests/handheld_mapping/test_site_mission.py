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


def test_connector_around_obstacle_has_no_removable_waypoints():
    """격자 A* 계단 경로가 아니라, 어느 경유점도 이웃 둘을 직선으로 이어 뺄 수 없는 팽팽한 경로여야 한다."""
    mission=mission_fixture()
    mission.start=np.array([.5,.5,0.]);mission.junction=np.array([2.5,.5])
    x0,x1,y0,y1=11,13,2,13
    mission.site.observed[y0:y1+1,x0:x1+1]=False   # 직선 start→junction을 막는 장애물
    slope=mission.settings['normal_slope_deg']
    assert not mission.site.validate_path(np.asarray([mission.start[:2],mission.junction]),slope)['accepted']
    path=mission.connector()
    assert np.allclose(path[0],mission.start[:2]) and np.allclose(path[-1],mission.junction)
    assert mission.site.validate_path(path,slope)['accepted']
    for before,after in zip(path[:-2],path[2:]):
        assert not mission.site.validate_path(np.asarray([before,after]),slope)['accepted']


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


BLEND_KEYS=('follower_blend','blend_heading_hold','blend_max_yaw_rate_rad_s','blend_ramp_slope_deg',
            'blend_ramp_resume_deg','blend_ramp_stop_deg','blend_dir_rate_rad_s','blend_realign_stop_deg','blend_realign_resume_deg')


def test_blend_settings_default_to_off_and_spec_values():
    s=mission_fixture().settings
    assert s['follower_blend']==0 and s['blend_heading_hold']==0
    assert s['blend_max_yaw_rate_rad_s']==.09 and s['blend_ramp_slope_deg']==8
    assert s['blend_ramp_resume_deg']==15 and s['blend_ramp_stop_deg']==25 and s['blend_dir_rate_rad_s']==1.0


def test_blend_realign_defaults_and_validation():
    from experiments.handheld_mapping.site_mission import SiteMission
    old=mission_fixture()
    assert old.settings['blend_realign_stop_deg']==90 and old.settings['blend_realign_resume_deg']==45
    for change in ({'blend_realign_stop_deg':181},{'blend_realign_resume_deg':90},{'blend_realign_resume_deg':100},{'blend_realign_resume_deg':0}):
        with pytest.raises(ValueError,match='invalid mission settings'):
            SiteMission(old.site,old.start,old.junction,old.summit,old.routes,change)
    SiteMission(old.site,old.start,old.junction,old.summit,old.routes,{'blend_realign_stop_deg':180,'blend_realign_resume_deg':10})


def test_blend_settings_accept_one():
    from experiments.handheld_mapping.site_mission import SiteMission
    old=mission_fixture()
    mission=SiteMission(old.site,old.start,old.junction,old.summit,old.routes,{'follower_blend':1,'blend_heading_hold':1})
    assert mission.settings['follower_blend']==1 and mission.settings['blend_heading_hold']==1


@pytest.mark.parametrize('name,value',[('follower_blend',2),('blend_heading_hold',.5),('blend_max_yaw_rate_rad_s',.7),
    ('blend_ramp_resume_deg',25),('blend_ramp_resume_deg',30),('blend_dir_rate_rad_s',0),('blend_max_yaw_rate_rad_s',0)])
def test_blend_settings_reject_invalid(name,value):
    from experiments.handheld_mapping.site_mission import SiteMission
    old=mission_fixture()
    with pytest.raises(ValueError,match='invalid mission settings'):
        SiteMission(old.site,old.start,old.junction,old.summit,old.routes,{name:value})


def test_old_mission_json_without_blend_keys_loads(tmp_path):
    from experiments.handheld_mapping.site_mission import SiteMission
    folder=tmp_path/'mission';mission_fixture().save(folder)
    data=json.loads((folder/'mission.json').read_text())
    for key in BLEND_KEYS:del data['settings'][key]
    (folder/'mission.json').write_text(json.dumps(data))
    loaded=SiteMission.load(folder,load_mesh=False)
    assert loaded.settings['follower_blend']==0 and loaded.settings['blend_max_yaw_rate_rad_s']==.09
    assert all(key in loaded.settings for key in BLEND_KEYS)
