# ================================================================
# test_calibration_probe.py
# 설명: 실제 rectified P와 source CameraInfo를 보존하며 URDF 임시 외부 보정을 만든다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping
# ================================================================
import copy
from pathlib import Path
import numpy as np
import pytest
from prepare_mapping_run import calibration_from_probe

@pytest.fixture
def probe(calibration):
    infos={}
    for side in ('left','right'):
        info=copy.deepcopy(calibration[side]); info['r']=[1,0,0,0,1,0,0,0,1]
        k=info['k']; info['p']=[k[0],0,k[2],0,0,k[4],k[5],0,0,0,1,0]
        if side=='right': info['p'][3]=-k[0]*.05
        infos[side]=info
    return {'camera_info':infos,'stats':{s:{'encoding':'mono8','width':640,'height':480,'receipt_rate_hz':30.} for s in infos},
            'exact_pair_candidates':900}

def test_requires_rectified_confirmation(probe,urdf):
    with pytest.raises(ValueError): calibration_from_probe(probe,urdf,'640x480x30:Y8',False)

def test_mismatched_projection_rejected(probe,urdf):
    probe['camera_info']['right']['p'][4]=1
    with pytest.raises(ValueError): calibration_from_probe(probe,urdf,'640x480x30:Y8',True)

def test_actual_projection_and_provisional_angle(probe,tmp_path):
    urdf=tmp_path/'camera.urdf'
    urdf.write_text('''<robot name="fixture"><link name="base_link"/><link name="mount"/><link name="camera_infra1_optical_frame"/>
    <joint name="pitch" type="fixed"><parent link="base_link"/><child link="mount"/><origin rpy="0 0.6108652381980153 0" xyz="0.1 0.02 0.3"/></joint>
    <joint name="optical" type="fixed"><parent link="mount"/><child link="camera_infra1_optical_frame"/><origin rpy="-1.5707963267948966 0 -1.5707963267948966"/></joint></robot>''')
    result=calibration_from_probe(probe,urdf,'640x480x30:Y8',True)
    assert result['pitch_down_deg']==pytest.approx(35)
    assert result['T_c1_c2'][0][3]==pytest.approx(.05)
    assert result['extrinsic_verified'] is False
    assert result['source_camera_info']==probe['camera_info']

def test_handheld_uses_actual_stereo_without_body_extrinsic(probe):
    result=calibration_from_probe(probe,None,'640x480x30:Y8',True)
    assert result['pose_output']=='camera'
    assert result['T_c1_c2'][0][3]==pytest.approx(.05)
    assert result['source_camera_info']==probe['camera_info']
    assert not {'T_c1_base','pitch_down_deg','extrinsic_source','extrinsic_verified'} & result.keys()


def test_updated_rover_left_optical_mount(probe):
    urdf = Path(__file__).resolve().parents[2] / 'ros2_ws/src/rover_description/urdf/rocker_bogie_4wd_rover.urdf'
    result = calibration_from_probe(probe, urdf, '640x480x30:Y8', True)
    # 독립적으로 계산한 광학 중심: camera_link 원점 + Y 방향 17.5 mm.
    base_from_left = np.linalg.inv(result['T_c1_base'])
    assert base_from_left[:3, 3] == pytest.approx([.12498574268, .02497451631, .01521787607])
    assert base_from_left[:3, 2] == pytest.approx([.93969257, 0, -.342020284], abs=1e-8)
    assert result['pitch_down_deg'] == pytest.approx(20, abs=1e-4)
    assert result['urdf_left_optical_frame'] == 'camera_left_optical_frame'
    assert result['extrinsic_verified'] is False
    assert result['T_c1_c2'][0][3] == pytest.approx(.05)


def test_ambiguous_left_optical_mount_rejected(probe, tmp_path):
    urdf = tmp_path / 'ambiguous.urdf'
    urdf.write_text('''<robot name="fixture"><link name="base_link"/>
    <link name="camera_infra1_optical_frame"/><link name="camera_left_optical_frame"/>
    <joint name="a" type="fixed"><parent link="base_link"/><child link="camera_infra1_optical_frame"/></joint>
    <joint name="b" type="fixed"><parent link="base_link"/><child link="camera_left_optical_frame"/><origin xyz="0.1 0 0"/></joint>
    </robot>''')
    with pytest.raises(ValueError, match='ambiguous'):
        calibration_from_probe(probe, urdf, '640x480x30:Y8', True)
