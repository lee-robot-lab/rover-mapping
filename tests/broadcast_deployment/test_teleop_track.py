# ================================================================
# test_teleop_track.py
# 설명: 수동 이동 궤적의 회차 분리·ANCHOR/재접속 단절·저장을 실제 UDP로 검증한다.
# 사용법: python -m pytest tests/broadcast_deployment/test_teleop_track.py -q
# ================================================================
import json
import socket
import time
import pytest
from experiments.broadcast_demo.core import Session
from experiments.broadcast_demo.server import Broadcast
from experiments.broadcast_demo.paths import DEFAULT_OUT


def pose(seq,x=3.6,**kw):
    return dict(seq=seq,timestamp=seq*.2,x=x,y=2.82,yaw=0.,v=.1,w=0.,**kw)


def test_manual_track_breaks_on_gap_and_new_source():
    session=Session((3.6,2.82,0.))
    for seq,x,now,source in [(1,3.6,0.,'a'),(2,3.7,.2,'a'),(3,4.,1.,'a'),(4,6.,1.2,'b')]:
        session.ingest(pose(seq,x,session_id=source),now=now)
        session.record_teleop()
    state=session.snapshot(now=1.2)
    assert [len(s) for s in state['teleop_tracks']]==[2,1,1]
    assert state['teleop_distance_m']==pytest.approx(.1)
    assert state['tracks']==[[],[],[],[]]


def test_live_teleop_auto_anchor_segments_and_saved_distance():
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    def send(seq,x,active=False,run=0,anchor=1):
        value=pose(seq,x,operator=dict(active=active,run=run,anchor_count=anchor,stage='ready'))
        sock.sendto(json.dumps(value).encode(),app.socket.getsockname())
        deadline=time.monotonic()+2
        while app.session.accepted<seq:
            assert time.monotonic()<deadline
            time.sleep(.001)
    try:
        send(1,3.6);send(2,3.62)
        send(3,3.64,True,1);send(4,3.66,True,1)
        send(5,3.68,False,1);send(6,3.70,False,1)
        send(7,3.6,False,1,2)
        state=app.state()
        assert [len(s) for s in state['teleop_tracks']]==[2,2,1]
        assert state['teleop_distance_m']==pytest.approx(.04)
        assert state['teleop_explored']
        assert [p[0] for p in state['tracks'][0]]==pytest.approx([3.64,3.66])
        app.save();saved=json.loads((app.log_dir/'runs.json').read_text())
        assert saved['teleop_tracks']==state['teleop_tracks']
        assert saved['teleop_distance_m']==pytest.approx(.04)
    finally:sock.close();app.close()


def test_basket_transfer_is_manual_not_run1():
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    try:
        app.session.ingest(pose(1))
        app.sync_operator(dict(active=False,run=0,anchor_count=0))
        app.session.record_teleop()
        state=app.session.snapshot()
        assert state['run']==0 and not state['active']
        assert state['tracks'][0]==[]
        assert state['teleop_tracks'][0][0][:2]==pytest.approx(app.meta['start'][:2])
    finally:app.close()
