# ================================================================
# test_map_guard.py
# 설명: 실기 지도 식별자의 전달과 중계 예측·저장 데이터의 지도 일치를 검증한다.
# 사용법: python -m pytest tests/broadcast_deployment/test_map_guard.py -q
# ================================================================
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]


def test_runner_status_identifies_site_without_enlarging_gateway_message():
    from deployment.ksrc.runner import Navigation, gateway_payload
    from deployment.ksrc.cli import fake_config

    nav = Navigation(ROOT/'deployment/ksrc/mission', fake_config())
    status = nav.status(0.)
    assert status['site_identity'] == nav.mission.site.identity
    payload = gateway_payload(status)
    assert len(payload) <= 256
    assert 'site_identity' not in json.loads(payload)


def test_decoder_preserves_valid_map_identity_and_rejects_invalid():
    from experiments.broadcast_demo.core import decode_packet
    value=dict(seq=1,timestamp=0,x=0,y=0,yaw=0,v=.1,w=0,site_identity='a'*64)
    assert decode_packet(json.dumps(value).encode())['site_identity']=='a'*64
    for invalid in ('wrong',None,12,'g'*64):
        value['site_identity']=invalid
        if invalid is None:
            assert 'site_identity' not in decode_packet(json.dumps(value).encode())
        else:
            with pytest.raises(ValueError):decode_packet(json.dumps(value).encode())


def test_live_server_refuses_mixed_map_cache(tmp_path):
    from experiments.broadcast_demo.server import Broadcast
    from experiments.broadcast_demo.paths import DEFAULT_OUT
    (tmp_path/'manifest.json').write_bytes((DEFAULT_OUT/'manifest.json').read_bytes())
    (tmp_path/'mission_coverage.json').write_text(json.dumps(dict(site_identity='0'*64)))
    with pytest.raises(ValueError,match='mismatch'):
        Broadcast(tmp_path,'127.0.0.1',0,live=True)


def test_wrong_rover_map_preserves_pose_and_hides_predictions():
    from experiments.broadcast_demo.server import Broadcast
    from experiments.broadcast_demo.paths import DEFAULT_OUT
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    try:
        app.session.ingest(dict(seq=1,timestamp=0.,x=3.6,y=1.7,yaw=-1.57,v=.1,w=0.,site_identity='0'*64))
        app.sync_operator(dict(active=False,run=0,anchor_count=1,stage='ready'))
        state=app.state()
        assert state['map_match'] is False and state['prediction'] is None
        assert state['pose']['x']==3.6
        app.session.raw['site_identity']=app.site_identity
        assert app.state()['map_match'] is True
        assert app.state()['prediction'] is not None
    finally:app.close()


def test_display_terrain_records_selected_map_and_all_added_blocked_cells():
    import numpy as np
    from experiments.broadcast_demo.paths import DEFAULT_OUT
    planning=json.loads((ROOT/'deployment/ksrc/mission/planning_map.json').read_text())
    manifest=json.loads((DEFAULT_OUT/'manifest.json').read_text())
    assert manifest['planning_map']['site_identity']==planning['site_identity']
    with np.load(DEFAULT_OUT/'terrain.npz') as terrain:
        cells=np.rint((np.asarray(planning['added_blocked_centers_xy_m'])-terrain['origin'])/float(terrain['cell'])).astype(int)
        assert (terrain['cost'][tuple(cells.T)]==1.).all()
        assert np.isfinite(terrain['cost']).all()


def test_stones_only_visible_inside_current_reveal_radius():
    from experiments.broadcast_demo.server import Broadcast
    from experiments.broadcast_demo.paths import DEFAULT_OUT
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=False)
    try:
        assert app.state()['obstacles']==[]
        app.session.ingest(dict(seq=1,timestamp=0.,x=3.6,y=2.82,yaw=0.,v=0.,w=0.))
        assert app.state()['obstacles']==[]
        app.session.raw.update(x=2.65,y=.85)
        stones=app.state()['obstacles']
        assert len(stones)==1 and stones[0]['x']==2.458
        app.session.raw.update(x=3.6,y=2.82)
        assert app.state()['obstacles']==[]
        app.session.raw.update(x=2.65,y=.85)
        app.session.received-=1.
        assert app.state()['obstacles']==[]
    finally:app.close()


def test_hide_stones_only_changes_marker_visibility():
    from experiments.broadcast_demo.server import Broadcast
    from experiments.broadcast_demo.paths import DEFAULT_OUT
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True,hide_stones=True)
    try:
        app.session.ingest(dict(seq=1,timestamp=0.,x=2.65,y=.85,yaw=-1.57,v=.1,w=0.))
        hidden=app.state()
        assert hidden['obstacles']==[]
        app.hide_stones=False
        visible=app.state()
        assert len(visible['obstacles'])==1
        assert hidden['prediction']==visible['prediction']
        assert hidden['cells']==visible['cells']
        assert hidden['pose']==visible['pose']
    finally:app.close()


@pytest.mark.parametrize('mode',['demo','sim','real'])
def test_launcher_passes_hide_stones_only_to_display_server(mode):
    from argparse import Namespace
    from deployment.broadcast.launch import commands
    args=Namespace(http_port=8765,udp_port=9870,hide_stones=True,demo=mode=='demo',sim=mode=='sim',
                   pi='pi@example',remote_root='/deployment/ksrc',state_dir='')
    jobs=commands(args)
    assert '--hide-stones' in jobs[0]
    assert all('--hide-stones' not in command for command in jobs[1:])
