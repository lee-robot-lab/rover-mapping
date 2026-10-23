# ================================================================
# test_anchor_presets.py
# 설명: 같은 출발 좌표의 -y/-x 기준, START 정렬 검사와 AUTO 중 보정 거절을 검증한다.
# 사용법: python -m pytest tests/pi_navigation/test_anchor_presets.py -q
# ================================================================
import math
import numpy as np
import pytest
from test_runner import make_runner,feed


@pytest.mark.parametrize('preset,yaw',[(1,-math.pi/2),(2,-math.pi),(3,math.pi/2),(4,0.)])
def test_anchor_preset_sets_estimator_runtime_and_start_heading(preset,yaw):
    nav=make_runner();feed(nav,3.5)
    old_xy=nav.prior[:2,3].copy()
    assert nav.anchor(3.5,preset=preset)
    assert nav.status(3.5)['anchor_id']==preset
    assert math.atan2(nav.prior[1,0],nav.prior[0,0])==pytest.approx(yaw)
    assert np.allclose(nav.prior[:2,3],old_xy)
    assert nav.status(3.5)['start_aligned']
    assert nav.new_runtime(4).mission.start[2]==pytest.approx(yaw)
    assert nav.start(4,3.5)
    assert not nav.anchor(3.6,preset=1)
    assert nav.status(3.6)['anchor_id']==preset


def test_invalid_or_unready_anchor_does_not_change_preset():
    nav=make_runner()
    assert not nav.anchor(0.,preset=2)
    assert nav.anchor_id==1
    feed(nav,3.5)
    assert not nav.anchor(3.5,preset=5)
    assert nav.anchor_id==1


def test_gateway_preserves_legacy_anchor_and_separates_anchor2():
    from ksrc_runtime.handheld_mapping.control_gateway import Gateway,ANCHOR,ANCHOR2,ANCHOR3,ANCHOR4
    calls=[];gw=Gateway(lambda *args:None,on_anchor=lambda:calls.append(1),on_anchor2=lambda:calls.append(2),
                                      on_anchor3=lambda:calls.append(3),on_anchor4=lambda:calls.append(4))
    assert gw.control(ANCHOR,(0.,0.,0.),1.)
    assert gw.control(ANCHOR2,(0.,0.,0.),1.)
    assert gw.control(ANCHOR3,(0.,0.,0.),1.)
    assert gw.control(ANCHOR4,(0.,0.,0.),1.)
    assert calls==[1,2,3,4]
    gw.mode='AUTO'
    assert not gw.control(ANCHOR2,(0.,0.,0.),2.)
