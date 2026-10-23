# ================================================================
# test_mission_cache.py
# 설명: 본 주행 예측의 회차·단계 분리와 stale/범위 밖 숨김을 검증한다.
# 사용법: python -m pytest tests/broadcast_deployment/test_mission_cache.py -q
# ================================================================
import json
import numpy as np
import pytest
from experiments.broadcast_demo.core import Lookup, decode_packet
from experiments.broadcast_demo.server import Broadcast
from experiments.broadcast_demo.paths import DEFAULT_OUT


def test_decode_keeps_private_stage_for_prediction_selection():
    value=dict(seq=1,timestamp=0,x=3.6,y=2.82,yaw=0,v=.1,w=0,
               operator=dict(active=True,run=3,anchor_count=1,stage='outer6'))
    assert decode_packet(json.dumps(value).encode())['operator']['stage']=='outer6'


def test_lookup_separates_overlapping_ascent_and_descent():
    states=np.array([[1.,1.,0.,.1,0.]]*2)
    lookup=Lookup(states,np.array([3,3]),stages=np.array(['outer6','return6']))
    assert lookup.find(states[0],3,stage='return6')==1
    assert lookup.find(states[0],3,stage='other') is None


@pytest.fixture
def cache_root(tmp_path):
    for name in ('mission_coverage.json','manifest.json','terrain.npz','terrain.png','predictions.npz','teleop_cache.npz'):
        (tmp_path/name).symlink_to(DEFAULT_OUT/name)
    states=np.array([[3.6,2.82,0.,.05,0.]]*4)
    paths=np.array([[[3.6,2.82],[3.62,2.82]]]*4)
    np.savez_compressed(tmp_path/'mission_cache.npz',states=states,runs=np.arange(1,5),
        stages=np.array(['entry30','entry15','entry30','outer6']),
        candidates=paths[:,None],chosen=paths,costs=np.arange(1,5)[:,None],
        slips=np.array([.1,.2,.3,.4]),compute_ms=np.ones(4))
    return tmp_path


@pytest.mark.parametrize('run,stage',[(1,'entry30'),(2,'entry15'),(3,'entry30'),(4,'outer6')])
def test_live_mission_prediction_uses_received_pose_and_correct_run(cache_root,run,stage):
    app=Broadcast(cache_root,'127.0.0.1',0,live=True)
    try:
        app.session.ingest(dict(seq=1,timestamp=0.,x=3.6,y=2.82,yaw=0.,v=.1,w=0.))
        app.sync_operator(dict(active=True,run=run,anchor_count=1,stage=stage))
        state=app.state()
        assert state['prediction'] is not None
        assert state['prediction']['slip']==pytest.approx(run*.1)
        assert state['pose']['yaw']==0.
        app.operator['stage']='unmatched'
        assert app.state()['prediction'] is None
        app.operator['stage']=stage
        app.session.raw['v']=0.
        assert app.state()['prediction']['slip'] is None
        app.session.raw['x']=8.
        assert app.state()['prediction'] is None
        app.session.raw['x']=3.6
        app.session.received-=1.
        assert app.state()['prediction'] is None
    finally:app.close()


def test_teleop_model_is_visible_without_start_and_never_becomes_auto(cache_root):
    app=Broadcast(cache_root,'127.0.0.1',0,live=True)
    try:
        app.session.ingest(dict(seq=1,timestamp=0.,x=3.6,y=1.7,yaw=-np.pi/2,v=.1,w=0.,speed_m_s=.09))
        app.sync_operator(dict(active=False,run=0,anchor_count=1,stage='ready'))
        state=app.state()
        assert state['prediction'] is not None
        assert state['prediction']['source']=='teleop_model'
        assert not app.session.active and app.session.run==0
        assert state['pose']['speed_m_s']==.09
        app.session.raw['v']=0.
        assert app.state()['prediction'] is None
        app.session.raw['v']=.1;app.session.received-=1.
        assert app.state()['prediction'] is None
    finally:app.close()
