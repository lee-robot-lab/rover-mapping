# ================================================================
# test_mapping_preflight.py
# 설명: 실기 준비 판정이 결측·시간 역행·좌우 누락·지연을 정상으로 승인하지 않는지 검사한다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping/test_mapping_preflight.py
# ================================================================
import copy
from pathlib import Path
import pytest


@pytest.fixture
def live_probe(calibration: dict) -> dict:
    infos=copy.deepcopy({side:calibration[side] for side in ('left','right')})
    for side,info in infos.items():
        k=info['k']; info.update(r=[1,0,0,0,1,0,0,0,1],p=[k[0],0,k[2],0,0,k[4],k[5],0,0,0,1,0])
        if side=='right': info['p'][3]=-k[0]*.05
    stats={side:{'count':900,'receipt_rate_hz':30.,'width':640,'height':480,'encoding':'mono8',
                 'profile_changes':0,'non_increasing':0,'min_age_s':.01,'max_age_s':.04} for side in infos}
    return {'status':'captured','elapsed_s':30.,'camera_info':infos,'stats':stats,
            'exact_pair_candidates':900,'clock':{'use_sim_time':False},
            'system':[{'disk_free_bytes':8_000_000_000,'temperatures_mC':{'cpu':'55000'}}]}


def check(probe: dict) -> dict:
    from mapping_preflight import assess_probe
    urdf=Path(__file__).resolve().parents[2]/'ros2_ws/src/rover_description/urdf/rover.urdf'
    return assess_probe(probe,urdf,'640x480x30:Y8',True)


def test_valid_capture_keeps_provisional_extrinsic_warning(live_probe):
    result=check(live_probe)
    assert result['capture_ready'] is True
    assert result['extrinsic_verified'] is False
    assert result['warnings']


@pytest.mark.parametrize('fault', ['missing','clock','backward','format','age','negative_age','pairs','disk','short'])
def test_bad_sensor_evidence_is_not_ready(live_probe,fault):
    if fault=='missing': del live_probe['stats']['left']['min_age_s']
    elif fault=='clock': live_probe['clock']['use_sim_time']=True
    elif fault=='backward': live_probe['stats']['right']['non_increasing']=1
    elif fault=='format': live_probe['stats']['right']['profile_changes']=1
    elif fault=='age': live_probe['stats']['left']['max_age_s']=2.
    elif fault=='negative_age': live_probe['stats']['left']['min_age_s']=-1.
    elif fault=='pairs': live_probe['exact_pair_candidates']=1
    elif fault=='disk': live_probe['system'][0]['disk_free_bytes']=1000
    elif fault=='short': live_probe['elapsed_s']=1.
    result=check(live_probe)
    assert result['capture_ready'] is False and result['issues']


def test_bad_projection_is_not_ready(live_probe):
    live_probe['camera_info']['right']['p'][3]=50
    assert check(live_probe)['capture_ready'] is False


def test_camera_info_size_cannot_disagree_with_images(live_probe):
    live_probe['camera_info']['left']['width']=1280
    assert check(live_probe)['capture_ready'] is False


def test_invalid_camera_principal_point_is_not_ready(live_probe):
    for side in ('left','right'): live_probe['camera_info'][side]['p'][2]=10000.
    assert check(live_probe)['capture_ready'] is False
