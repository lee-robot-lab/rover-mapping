# ================================================================
# test_viewer.py
# 설명: 실제 지도 선택의 backend·map 분리·최신 미완료·경로 검증을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_viewer.py -q
# ================================================================
import json
from pathlib import Path
import pytest


def maps(run: Path,folder='dense_offline'):
    output=run/folder; output.mkdir(parents=True)
    rows=[]
    for map_id in (0,421):
        mesh=f'{folder}/map_{map_id}_mesh.ply'
        cloud=f'{folder}/map_{map_id}_cloud.ply'
        (run/mesh).touch(); (run/cloud).touch()
        rows.append({'map_id':map_id,'mesh':mesh,'cloud':cloud})
    (output/'summary.json').write_text(json.dumps({'maps':rows,'status':'partial'}))
    return run


def test_viewer_defaults_to_latest_session_and_preserves_multiple_maps(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    maps(tmp_path/'001'); latest=maps(tmp_path/'002')
    selected=select_maps(None,sessions=tmp_path)
    assert len(selected)==2
    assert selected[0]==('Map 0',(latest/'dense_offline/map_0_mesh.ply').resolve())
    assert selected[1][0]=='Map 421'
    (tmp_path/'003').mkdir()
    with pytest.raises(FileNotFoundError,match='아직|없'):
        select_maps(None,sessions=tmp_path)


def test_viewer_explicit_backend_map_cloud_and_direct_ply(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    run=maps(tmp_path/'run'); maps(run,'dense_orbline')
    selected=select_maps(run,backend='orbline',map_id=421,cloud=True)
    assert selected==[('Map 421',(run/'dense_orbline/map_421_cloud.ply').resolve())]
    direct=run/'dense_offline/map_0_mesh.ply'
    assert select_maps(direct)==[(direct.name,direct.resolve())]
    with pytest.raises(ValueError):select_maps(direct,map_id=0)
    with pytest.raises(ValueError):select_maps(run,map_id=99)
    with pytest.raises(ValueError):select_maps(run,backend='unknown')


def test_viewer_rejects_summary_path_escape_and_missing_geometry(tmp_path):
    from experiments.handheld_mapping.viewer import select_maps
    run=maps(tmp_path/'run')
    summary=run/'dense_offline/summary.json'
    value=json.loads(summary.read_text())
    value['maps'][0]['mesh']='../../escape.ply'
    summary.write_text(json.dumps(value))
    with pytest.raises(ValueError):select_maps(run)
    run=maps(tmp_path/'other')
    (run/'dense_offline/map_421_mesh.ply').unlink()
    with pytest.raises(FileNotFoundError):select_maps(run)


@pytest.mark.parametrize('summary',[[],{'maps':None},{'maps':[{}]},
                                     {'maps':[{'map_id':0}]},{'maps':[{'map_id':0,'mesh':None}]}])
def test_malformed_saved_summary_is_a_user_facing_value_error(tmp_path,summary):
    from experiments.handheld_mapping.viewer import select_maps
    run=maps(tmp_path/'run')
    (run/'dense_offline/summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError):select_maps(run)
