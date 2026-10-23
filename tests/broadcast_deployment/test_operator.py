# ================================================================
# test_operator.py
# 설명: 실기 회차 상태의 자동 전환과 실제 pose 보존·중복 초기화 금지를 검증한다.
# 사용법: python -m pytest tests/broadcast_deployment -q
# ================================================================
import json
import pytest
from experiments.broadcast_demo.server import Broadcast
from experiments.broadcast_demo.paths import DEFAULT_OUT
from experiments.broadcast_demo.core import decode_packet


def packet(seq,x=3.6,y=2.82):
    return dict(seq=seq,timestamp=0.,x=x,y=y,yaw=-1.57,v=.1,w=0.)


def test_operator_start_is_not_a_pose_snap():
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    try:
        app.session.ingest(packet(1,x=3.64))
        op=dict(active=True,run=1,anchor_count=1)
        app.sync_operator(op)
        assert app.session.pose()['x']==pytest.approx(3.64)
        assert app.phase=='run'
        count=len(app.session.tracks[0]);app.sync_operator(op)
        assert len(app.session.tracks[0])==count
        app.sync_operator(dict(op,active=False))
        assert not app.session.active
        app.sync_operator(dict(active=True,run=2,anchor_count=1))
        assert app.session.run==2
        assert len(app.session.tracks[0])==count
        assert app.state()['prediction'] is None
        with pytest.raises(ValueError,match='disabled'):app.play_demo(8765)
    finally:app.close()


def test_operator_metadata_and_ground_speed_survive_decode():
    value=dict(packet(1),speed_m_s=.12,operator=dict(active=False,run=0,anchor_count=0))
    parsed=decode_packet(json.dumps(value).encode())
    assert parsed['speed_m_s']==.12 and parsed['operator']['run']==0


def test_relay_restart_resumes_same_active_run_without_clearing_track():
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    try:
        op=dict(active=True,run=1,anchor_count=1)
        app.session.ingest(dict(packet(100),session_id='old'));app.sync_operator(op)
        app.session.ingest(dict(packet(101,x=3.62),session_id='old'))
        count=len(app.session.tracks[0])
        app.session.ingest(dict(packet(1,x=3.63),session_id='new'));app.sync_operator(op)
        assert app.session.active
        assert len(app.session.tracks[0])==count
        assert app.session.pose()['x']==pytest.approx(3.63)
    finally:app.close()


def test_live_http_cannot_override_operator_state():
    from http.server import ThreadingHTTPServer
    from threading import Thread
    from urllib.request import Request,urlopen
    from urllib.error import HTTPError
    from experiments.broadcast_demo.server import Handler
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.app=app
    thread=Thread(target=server.serve_forever,daemon=True);thread.start()
    try:
        for path in ('/api/run/1','/api/run/2','/api/stop','/api/demo/replay'):
            with pytest.raises(HTTPError) as failure:
                urlopen(Request(f'http://127.0.0.1:{server.server_address[1]}'+path,method='POST'),timeout=2)
            assert failure.value.code==409
    finally:
        server.shutdown();server.server_close();thread.join(timeout=1);app.close()


def test_anchor_packet_is_not_appended_to_old_coordinate_track():
    import socket,time
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    def send(seq,x,anchor):
        value=dict(packet(seq,x=x),operator=dict(active=False,run=0,anchor_count=anchor))
        sock.sendto(json.dumps(value).encode(),app.socket.getsockname())
        deadline=time.monotonic()+2
        while app.session.accepted<seq:
            assert time.monotonic()<deadline
            time.sleep(.01)
    try:
        send(1,3.6,0);send(2,3.62,0)
        length=len(app.session.tracks[0]);distance=app.session.distances[0]
        send(3,3.6,1)
        assert len(app.session.tracks[0])==length
        assert app.session.distances[0]==pytest.approx(distance)
        assert app.phase=='ready'
        assert app.session.pose()['x']==pytest.approx(3.6)
    finally:sock.close();app.close()


def test_old_operator_packet_cannot_stop_active_recording():
    import socket,time
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
        active=dict(packet(10),operator=dict(active=True,run=1,anchor_count=1))
        old=dict(packet(9),operator=dict(active=False,run=0,anchor_count=0))
        sock.sendto(json.dumps(active).encode(),app.socket.getsockname())
        deadline=time.monotonic()+2
        while not app.session.active:
            assert time.monotonic()<deadline;time.sleep(.01)
        sock.sendto(json.dumps(old).encode(),app.socket.getsockname())
        while not app.session.rejected:
            assert time.monotonic()<deadline;time.sleep(.01)
        assert app.session.active and app.operator['run']==1
    finally:sock.close();app.close()


def test_run3_run4_are_recorded_without_overwriting_previous_runs():
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    try:
        for run in (1,2,3,4):
            app.session.ingest(packet(run),record=False)
            app.sync_operator(dict(active=True,run=run,anchor_count=run))
            assert app.session.run==run and app.session.active
        state=app.session.snapshot()
        assert len(state['tracks'])==4
        assert all(len(track)==1 for track in state['tracks'])
        assert decode_packet(json.dumps(dict(packet(5),operator=dict(active=True,run=4,anchor_count=4))).encode())['operator']['run']==4
    finally:app.close()
