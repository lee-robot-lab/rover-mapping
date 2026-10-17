# ================================================================
# test_viewer.py
# 설명: 실제 HTTP 경계의 허용 파일·stale·준비 전 상태를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_viewer.py
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
