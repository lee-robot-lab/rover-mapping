# ================================================================
# test_prepare_mapping_run.py
# 설명: 잘못된 실기 보정·덮어쓰기·Atlas 입력 계약 변경을 거부하는지 검증한다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping/test_prepare_mapping_run.py
# ================================================================
import copy
import hashlib
import json
from pathlib import Path
import pytest
from prepare_mapping_run import prepare_run, validate_stereo_info

@pytest.mark.parametrize('bad', ['size', 'focal', 'nan', 'baseline', 'swapped', 'rotation', 'row', 'profile', 'distortion'])
def test_invalid_calibration_rejected(tmp_path, calibration, urdf, bad):
    if bad == 'size': calibration['right']['width'] = 320
    elif bad == 'focal': calibration['left']['k'][0] = 0
    elif bad == 'nan': calibration['left']['k'][4] = float('nan')
    elif bad == 'baseline': calibration['T_c1_c2'][0][3] = 0
    elif bad == 'swapped': calibration['T_c1_c2'][0][3] *= -1
    elif bad == 'rotation': calibration['T_c1_base'][0][0] = 3
    elif bad == 'row': calibration['T_c1_base'][3][0] = 1
    elif bad == 'profile': calibration['camera_profile'] = '1280x720x30:Y8'
    else: calibration['left']['distortion_model'] = 'equidistant'
    with pytest.raises(ValueError): prepare_run(tmp_path / 'run', calibration, urdf)
    assert not (tmp_path / 'run').exists()

def test_existing_run_not_overwritten(tmp_path, calibration, urdf):
    prepare_run(tmp_path / 'run', calibration, urdf)
    with pytest.raises(FileExistsError): prepare_run(tmp_path / 'run', calibration, urdf)

def test_provisional_pitch_and_original_urdf_preserved(tmp_path, calibration, urdf):
    before = urdf.read_bytes()
    run = tmp_path / 'run'
    result = prepare_run(run, calibration, urdf)
    assert result['calibration']['pitch_down_deg'] == 35
    assert result['calibration']['extrinsic_verified'] is False
    assert urdf.read_bytes() == before == (run / 'robot.urdf').read_bytes()
    assert result['urdf_sha256'] == hashlib.sha256(before).hexdigest()
    assert 'IMU.Frequency' not in (run / 'orb.yaml').read_text()

def atlas_fixture(tmp_path, calibration, urdf):
    run = tmp_path / 'old'
    prepare_run(run, calibration, urdf)
    path = run / 'atlas.osa'
    path.write_bytes(b'fixture-atlas-not-real-orb')
    return path

def test_load_input_unchanged(tmp_path, calibration, urdf):
    atlas = atlas_fixture(tmp_path, calibration, urdf)
    original = atlas.read_bytes()
    result = prepare_run(tmp_path / 'new', calibration, urdf, atlas)
    assert atlas.read_bytes() == original
    assert (tmp_path / 'new/atlas_input.osa').read_bytes() == original
    assert result['atlas_reuse_verified'] is False
    assert 'System.LoadAtlasFromFile: "atlas_input"' in (tmp_path / 'new/orb.yaml').read_text()

@pytest.mark.parametrize('changed', ['intrinsics', 'vocabulary'])
def test_changed_contract_rejects_load(tmp_path, calibration, urdf, changed):
    atlas = atlas_fixture(tmp_path, calibration, urdf)
    new = copy.deepcopy(calibration)
    if changed == 'intrinsics': new['left']['k'][0] += 1
    else: new['vocabulary_sha256'] = 'different'
    with pytest.raises(ValueError): prepare_run(tmp_path / 'new', new, urdf, atlas)

def test_new_pitch_retains_prior_alignment(tmp_path, calibration, urdf):
    atlas = atlas_fixture(tmp_path, calibration, urdf)
    (atlas.parent / 'map_alignments.json').write_text(json.dumps({'7':{'T_map_orb':[[1,0,0,1],[0,1,0,2],[0,0,1,3],[0,0,0,1]]}}))
    calibration['pitch_down_deg'] = 20
    prepare_run(tmp_path / 'new', calibration, urdf, atlas)
    assert json.loads((tmp_path / 'new/map_alignments.json').read_text())['7']['T_map_orb'][0][3] == 1

def test_invalid_saved_alignment_rejected(tmp_path, calibration, urdf):
    atlas=atlas_fixture(tmp_path,calibration,urdf)
    (atlas.parent/'map_alignments.json').write_text('{"0":{"T_map_orb":[1,2,3]}}')
    with pytest.raises(ValueError): prepare_run(tmp_path/'new',calibration,urdf,atlas)

def test_legacy_mutable_map_identity_rejected(tmp_path,calibration,urdf):
    atlas=atlas_fixture(tmp_path,calibration,urdf)
    manifest=json.loads((atlas.parent/'manifest.json').read_text())
    manifest.pop('map_alignment_identity')
    (atlas.parent/'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError): prepare_run(tmp_path/'new',calibration,urdf,atlas)

def handheld_calibration(calibration):
    result=copy.deepcopy(calibration)
    for key in ('T_c1_base','pitch_down_deg','extrinsic_source','extrinsic_verified'):
        result.pop(key)
    result['pose_output']='camera'
    return result

def test_handheld_run_has_no_fabricated_robot_geometry(tmp_path,calibration):
    run=tmp_path/'camera'
    result=prepare_run(run,handheld_calibration(calibration),None)
    assert result['pose_output']=='camera'
    assert result['urdf_sha256'] is None
    assert not (run/'robot.urdf').exists()
    assert 'ROS.T_c1_base' not in (run/'orb.yaml').read_text()
    assert 'ROS.pose_output: "camera"' in (run/'orb.yaml').read_text()

def test_handheld_rejects_body_extrinsic(tmp_path,calibration):
    calibration['pose_output']='camera'
    with pytest.raises(ValueError): prepare_run(tmp_path/'run',calibration,None)

def test_atlas_alignment_output_mode_cannot_change(tmp_path,calibration,urdf):
    atlas=atlas_fixture(tmp_path,calibration,urdf)
    with pytest.raises(ValueError,match='contract mismatch'):
        prepare_run(tmp_path/'new',handheld_calibration(calibration),None,atlas)
