# ================================================================
# test_mapping_shutdown.py
# 설명: 강제 종료 흔적·빈 지도·수정된 Atlas 입력을 정상 완료로 표시하지 않는지 검증한다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping
# ================================================================
import json
from prepare_mapping_run import prepare_run
from finalize_mapping_run import finalize_run

def test_interrupted_not_complete(tmp_path,calibration,urdf):
    run=tmp_path/'run'; prepare_run(run,calibration,urdf)
    assert finalize_run(run)['status']=='incomplete'

def test_empty_atlas_not_complete(tmp_path,calibration,urdf):
    run=tmp_path/'run'; prepare_run(run,calibration,urdf)
    (run/'atlas.osa').touch(); (run/'tracking.json').write_text('{"status":"saved"}')
    (run/'recording.json').write_text('{"recording_status":"complete"}')
    assert finalize_run(run)['status']=='incomplete'

def test_load_input_modification_detected(tmp_path,calibration,urdf):
    old=tmp_path/'old'; prepare_run(old,calibration,urdf); (old/'atlas.osa').write_bytes(b'fixture')
    run=tmp_path/'new'; prepare_run(run,calibration,urdf,old/'atlas.osa')
    (run/'atlas_input.osa').write_bytes(b'changed')
    result=finalize_run(run)
    assert result['status']=='failed' and result['atlas_reuse_verified'] is False

def test_file_exists_does_not_prove_loaded(tmp_path,calibration,urdf):
    old=tmp_path/'old'; prepare_run(old,calibration,urdf); (old/'atlas.osa').write_bytes(b'fixture')
    run=tmp_path/'new'; prepare_run(run,calibration,urdf,old/'atlas.osa')
    assert finalize_run(run)['atlas_load_verified'] is False
    (run/'tracking.json').write_text('{"status":"incomplete","atlas_load_verified":true}')
    result=finalize_run(run)
    assert result['atlas_load_verified'] is True and result['atlas_reuse_verified'] is False

def test_raw_mcap_checksums_included(tmp_path,calibration,urdf):
    run=tmp_path/'run'; prepare_run(run,calibration,urdf)
    (run/'raw').mkdir(); (run/'raw/data.mcap').write_bytes(b'fixture-not-real-mcap')
    assert 'raw/data.mcap' in finalize_run(run)['files']
