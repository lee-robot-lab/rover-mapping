# ================================================================
# test_attitude_fusion.py
# 설명: 외부 자세 융합의 회전 방향과 센서 공백·지도 경계 처리를 검사한다.
# 사용법: python -m pytest tests/handheld_mapping/test_attitude_fusion.py
# ================================================================
import numpy as np
from scipy.spatial.transform import Rotation
from experiments.handheld_mapping.attitude_fusion import AttitudeFilter


def inputs(gyro=(0.,0.,0.)):
    t=np.arange(-1.,2.,.02)
    return t,np.tile(gyro,(len(t),1)),np.tile([0.,0.,9.80665],(len(t),1))


def test_gyro_short_prediction_and_expiry():
    t,g,a=inputs((0.,0.,1.))
    f=AttitudeFilter(t,g,a)
    f.update(0.,0,np.eye(3),None)
    r,status=f.update(.2,-1,None,None)
    np.testing.assert_allclose(r,Rotation.from_rotvec([0,0,.2]).as_matrix(),atol=1e-8)
    assert status=='gyro_only_position_invalid'
    assert f.map_id==0
    assert f.update(.4,-1,None,None)==(None,'unavailable')


def test_gap_falls_back_to_visual():
    t,g,a=inputs();keep=(t<.04)|(t>.3)
    f=AttitudeFilter(t[keep],g[keep],a[keep])
    f.update(0.,0,np.eye(3),None)
    visual=Rotation.from_euler('x',.1).as_matrix()
    r,status=f.update(.25,0,visual,None)
    np.testing.assert_allclose(r,visual)
    assert status=='visual_fallback_imu_gap'


def test_map_boundary_reanchors_in_new_coordinates():
    f=AttitudeFilter(*inputs())
    f.update(0.,0,np.eye(3),np.array([0.,0.,1.]))
    visual=Rotation.from_euler('z',1.5).as_matrix()
    r,status=f.update(.05,7,visual,np.array([0.,0.,1.]))
    np.testing.assert_allclose(r,visual)
    assert status=='visual_reanchor'


def test_gravity_correction_has_correct_sign_and_no_yaw_pull():
    f=AttitudeFilter(*inputs(),visual_tau_s=1e9)
    f.update(0.,0,np.eye(3),np.array([0.,0.,1.]))
    f.rotation=Rotation.from_euler('xyz',[.1,0.,.5]).as_matrix()
    before=np.linalg.norm(f.rotation@np.array([0.,0.,1.])-np.array([0.,0.,1.]))
    r,status=f.update(.05,0,np.eye(3),None)
    after=np.linalg.norm(r@np.array([0.,0.,1.])-np.array([0.,0.,1.]))
    assert after<before
    assert abs(Rotation.from_matrix(r).as_euler('xyz')[2]-.5)<1e-3


def test_invalid_acceleration_does_not_tilt():
    t,g,a=inputs();a[:]=[9.,0.,9.]
    f=AttitudeFilter(t,g,a)
    f.update(0.,0,np.eye(3),np.array([0.,0.,1.]))
    r,status=f.update(.05,0,np.eye(3),None)
    np.testing.assert_allclose(r,np.eye(3))


def test_bad_times_are_rejected():
    import pytest
    t,g,a=inputs();t[4]=t[3]
    with pytest.raises(ValueError,match='timestamp'):
        AttitudeFilter(t,g,a)


def test_map_change_without_visual_invalidates_old_rotation():
    f=AttitudeFilter(*inputs())
    f.update(0.,0,np.eye(3),np.array([0.,0.,1.]))
    assert f.update(.05,7,None,None)==(None,'unavailable_map_change')
    assert f.gravity is None
    r,status=f.update(.1,7,np.eye(3),None)
    assert status=='visual_reanchor'
    assert f.map_id==7
