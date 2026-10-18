# ================================================================
# test_sim_viewer.py
# 설명: 실제 HTTP 경계의 허용 파일·stale·준비 전 상태를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_sim_viewer.py
# ================================================================
import json
import time
from urllib.request import urlopen
from urllib.error import HTTPError
import pytest


def test_viewer_serves_only_atomic_run_assets_and_marks_stale(tmp_path):
    from experiments.site_sim.viewer import serve
    from experiments.handheld_mapping.storage import atomic_json
    output=tmp_path/'run';(output/'live').mkdir(parents=True)
    server=serve(output,0);base=f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base) as response:assert 'text/html' in response.headers['Content-Type']
        for path,status in [('/state',503),('/frame/front.jpg',503),('/../etc/passwd',404),('/manifest.json',404)]:
            with pytest.raises(HTTPError) as error:urlopen(base+path)
            assert error.value.code==status
        atomic_json(output/'live/state.json',{'status':'running','updated_wall_s':time.time()-10,'last':{}})
        with urlopen(base+'/state') as response:data=json.load(response)
        assert data['stale'] and data['age_s']>=9
        atomic_json(output/'live/state.json',{'status':'complete','updated_wall_s':time.time()-10,'last':{}})
        with urlopen(base+'/state') as response:assert not json.load(response)['stale']
        (output/'live/state.json').write_text('{partial')
        with pytest.raises(HTTPError) as error:urlopen(base+'/state')
        assert error.value.code==503
    finally:server.shutdown();server.server_close()


@pytest.mark.parametrize('sim,wall',[(float('nan'),30),(0,30),(10,-1),(901,3600)])
def test_runner_refuses_unbounded_or_invalid_limits(sim,wall):
    from experiments.site_sim.run import validate_limits
    with pytest.raises(ValueError):validate_limits(sim,wall)


def _telemetry(rows):
    return '\n'.join(json.dumps(r) for r in rows)+'\n'


def test_replay_endpoint_serves_a_compact_timeline_and_overview_frames(tmp_path):
    import math
    from experiments.site_sim.viewer import serve
    output=tmp_path/'run';(output/'live').mkdir(parents=True);(output/'overview_frames').mkdir()
    rows=[{'stamp_s':k*.05,'position_m':[k*.01,0.,.4],'estimated_position_m':[k*.01+.02,0.,.4],
           'quaternion_wxyz':[math.cos(.3),0,0,math.sin(.3)],'mission':{'state':'ascending','route':'15'}} for k in range(100)]
    (output/'telemetry.jsonl').write_text(_telemetry(rows))
    (output/'overview_frames/0000.00.jpg').write_bytes(b'A');(output/'overview_frames/0002.50.jpg').write_bytes(b'B')
    server=serve(output,0);base=f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base+'/replay') as response:data=json.load(response)
        assert data['t'][0]==0 and len(data['t'])==len(data['gt'])==len(data['est'])==len(data['yaw'])==len(data['state'])
        assert data['err_cm'][0]==pytest.approx(2.,abs=.01) and data['yaw'][0]==pytest.approx(.6,abs=1e-3)
        assert data['frames']==[0.0,2.5]
        with urlopen(base+'/replay_frame?t=2.4') as response:assert response.read()==b'B'
        with urlopen(base+'/replay_frame?t=0.2') as response:assert response.read()==b'A'
        for path in ('/replay_frame?t=../../etc/passwd','/replay_frame'):
            with pytest.raises(HTTPError) as error:urlopen(base+path)
            assert error.value.code in (400,404)
    finally:server.shutdown();server.server_close()


def test_replay_endpoint_reports_missing_or_partial_telemetry(tmp_path):
    from experiments.site_sim.viewer import serve
    output=tmp_path/'run';(output/'live').mkdir(parents=True)
    server=serve(output,0);base=f'http://127.0.0.1:{server.server_port}'
    try:
        with pytest.raises(HTTPError) as error:urlopen(base+'/replay')
        assert error.value.code==404
        (output/'telemetry.jsonl').write_text('{partial')
        with pytest.raises(HTTPError) as error:urlopen(base+'/replay')
        assert error.value.code==503
    finally:server.shutdown();server.server_close()


def test_replay_serves_front_camera_frames_from_the_saved_stereo_images(tmp_path):
    import math
    from experiments.site_sim.viewer import serve
    output=tmp_path/'run';(output/'live').mkdir(parents=True);data=output/'stereo/mav0/cam0/data';data.mkdir(parents=True)
    (data/'1000000000.png').write_bytes(b'P1');(data/'2000000000.png').write_bytes(b'P2')
    rows=[{'stamp_s':k*.5,'position_m':[0,0,.4],'quaternion_wxyz':[1,0,0,0],'mission':{}} for k in range(6)]
    (output/'telemetry.jsonl').write_text(_telemetry(rows))
    server=serve(output,0);base=f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(base+'/replay') as response:assert json.load(response)['front']==[1.0,2.0]
        with urlopen(base+'/replay_front?t=1.9') as response:assert response.read()==b'P2'
        with urlopen(base+'/replay_front?t=0.1') as response:assert response.read()==b'P1'
        with pytest.raises(HTTPError) as error:urlopen(base+'/replay_front?t=x')
        assert error.value.code==400
    finally:server.shutdown();server.server_close()
