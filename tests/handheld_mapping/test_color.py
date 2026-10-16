# ================================================================
# test_color.py
# 설명: RGB 원본 보존·카메라 투영·가림·명시한 TSDF 출력 계약을 검증한다.
# 사용법: python -m pytest tests/handheld_mapping/test_color.py -q
# ================================================================
import json
from pathlib import Path
import zlib

import cv2
import numpy as np
import pytest

from test_core import make_run, calibration
from experiments.handheld_mapping.storage import RunWriter, decode_image
from experiments.handheld_mapping import reconstruct as fusion


def color_info(width: int = 80, height: int = 60) -> dict:
    return {'width': width, 'height': height, 'k': [70, 0, 39.5, 0, 70, 29.5, 0, 0, 1],
            'd': [0]*5, 'distortion_model': 'plumb_bob', 'color_frame': 'color_optical',
            'depth_frame': 'depth_optical', 'T_color_depth': np.eye(4).tolist()}


def add_color(run: Path, stamp: int = 1_000_000_000) -> None:
    meta = json.loads((run/'capture_calibration.json').read_text())
    meta['color'] = color_info()
    (run/'capture_calibration.json').write_text(json.dumps(meta))
    path = run/'raw/color'/f'{stamp}.png'
    path.parent.mkdir(parents=True, exist_ok=True)
    assert cv2.imwrite(str(path), np.full((60, 80, 3), [10, 20, 230], np.uint8))
    with (run/'frames.jsonl').open('a') as stream:
        stream.write(json.dumps({'stream': 'color', 'stamp_ns': stamp, 'frame_id': 'color_optical',
                                 'width': 80, 'height': 60, 'encoding': 'rgb8',
                                 'path': f'raw/color/{stamp}.png'})+'\n')


@pytest.mark.parametrize('encoding,raw', [('rgb8', bytes([10,20,30,40,50,60,0,0])),
                                         ('bgr8', bytes([30,20,10,60,50,40,0,0]))])
def test_padded_color_decodes_and_preserves_original_encoding(tmp_path, encoding, raw):
    header = {'kind': 'image', 'stream': 'color', 'stamp_ns': 1, 'width': 2, 'height': 1,
              'step': 8, 'encoding': encoding, 'compression': 'zlib', 'frame_id': 'color_optical'}
    assert decode_image(header, zlib.compress(raw)).tolist() == [[[30,20,10],[60,50,40]]]
    writer = RunWriter(tmp_path/'run')
    assert 'color' not in writer.counts
    writer.accept(header, zlib.compress(raw))
    writer.close()
    assert cv2.imread(str(writer.path/'raw/color/1.png')).tolist() == [[[30,20,10],[60,50,40]]]
    assert json.loads((writer.path/'frames.jsonl').read_text())['encoding'] == encoding
    assert writer.counts['color'] == 1


@pytest.mark.parametrize('side,encoding', [('left','rgb8'), ('color','mono8'), ('depth','bgr8')])
def test_stream_encoding_mismatch_refused(tmp_path, side, encoding):
    writer = RunWriter(tmp_path/'run')
    header = {'kind':'image','stream':side,'stamp_ns':1,'width':1,'height':1,'step':3,
              'encoding':encoding,'compression':'zlib'}
    with pytest.raises(ValueError, match='encoding'):
        writer.accept(header,zlib.compress(b'123'))
    writer.close()


def test_projection_uses_extrinsic_and_different_color_resolution():
    from experiments.handheld_mapping.color import project_depth_color
    depth = np.array([[1000,1000]],np.uint16)
    depth_info = {'k':[1,0,0,0,1,0,0,0,1], 'depth_scale_m':.001}
    info = color_info(4,1)
    info['k'] = [1,0,0,0,1,0,0,0,1]
    info['T_color_depth'][0][3] = 1
    bgr = np.array([[[0,0,10],[0,0,20],[0,0,30],[0,0,40]]],np.uint8)
    rgb, valid = project_depth_color(depth, depth_info, bgr, info)
    assert rgb.tolist() == [[[20,0,0],[30,0,0]]]
    assert valid.tolist() == [[True,True]]


def test_projection_rejects_occluded_pixels_and_keeps_neutral_unknown():
    from experiments.handheld_mapping.color import project_depth_color
    depth = np.array([[1000,2000,0]],np.uint16)
    dinfo = {'k':[10,0,0,0,10,0,0,0,1], 'depth_scale_m':.001}
    info = color_info(1,1)
    info['k'] = [1,0,0,0,1,0,0,0,1]
    rgb, valid = project_depth_color(depth, dinfo, np.array([[[10,20,30]]],np.uint8), info)
    assert valid.tolist() == [[True,False,False]]
    assert rgb.tolist() == [[[30,20,10],[128,128,128],[128,128,128]]]


@pytest.mark.parametrize('model,coefficients', [('plumb_bob',[1,0,0,0,0]),
                                               ('rational_polynomial',[1,0,0,0,0,0,0,0])])
def test_projection_applies_supported_color_distortion(model,coefficients):
    from experiments.handheld_mapping.color import project_depth_color
    info = color_info(4,1)
    info.update(k=[1,0,0,0,1,0,0,0,1], d=coefficients, distortion_model=model)
    rgb, mask = project_depth_color(np.array([[1000,1000]],np.uint16),
        {'k':[1,0,0,0,1,0,0,0,1],'depth_scale_m':.001},
        np.array([[[0,0,10],[0,0,20],[0,0,30],[0,0,40]]],np.uint8), info)
    assert rgb[0,1].tolist() == [30,0,0] and mask.all()


def test_projection_refuses_unknown_distortion():
    from experiments.handheld_mapping.color import project_depth_color
    info = color_info(1,1)
    info['distortion_model'] = 'equidistant'
    with pytest.raises(ValueError,match='distortion'):
        project_depth_color(np.ones((1,1),np.uint16), calibration(),np.zeros((1,1,3),np.uint8),info)


def test_explicit_pose_output_is_new_relative_and_preserves_source(tmp_path):
    import open3d as o3d
    run = make_run(tmp_path/'run')
    raw = run/'raw/depth/1000000000.png'
    assert cv2.imwrite(str(raw),np.full((60,80),800,np.uint16))
    before = raw.read_bytes()
    poses = tmp_path/'poses'
    poses.mkdir()
    (poses/'map_0_keyframes.csv').write_text('map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n0,0,1.0,3,0,0,0,0,0,1\n')
    output = tmp_path/'output'
    result = fusion.reconstruct(run,voxel_size=.025,keyframe_dir=poses,output_dir=output,color=True)
    assert result['paths_relative_to'] == 'summary'
    assert result['maps'][0]['cloud'] == 'map_0_cloud.ply'
    cloud = o3d.io.read_point_cloud(str(output/result['maps'][0]['cloud']))
    assert np.median(np.asarray(cloud.points)[:,0]) == pytest.approx(3.2,abs=.05)
    assert result['color_status'] == 'unavailable' and not cloud.has_colors()
    assert raw.read_bytes() == before and not (run/'dense').exists()
    with pytest.raises(FileExistsError):
        fusion.reconstruct(run,keyframe_dir=poses,output_dir=output)


@pytest.mark.parametrize('gap,status,has_colors', [(0,'complete',True),(34_000_001,'unavailable',False)])
def test_tsdf_color_preserves_geometry_and_enforces_time_tolerance(tmp_path,gap,status,has_colors):
    import open3d as o3d
    run = make_run(tmp_path/'run')
    raw = run/'raw/depth/1000000000.png'
    assert cv2.imwrite(str(raw),np.full((60,80),800,np.uint16))
    add_color(run,1_000_000_000+gap)
    before = {p:p.read_bytes() for p in (run/'raw').rglob('*.png')}
    base = fusion.reconstruct(run,voxel_size=.025,output_dir=tmp_path/'base')
    result = fusion.reconstruct(run,voxel_size=.025,output_dir=tmp_path/'color',color=True)
    plain = o3d.io.read_point_cloud(str(tmp_path/'base'/base['maps'][0]['cloud']))
    colored = o3d.io.read_point_cloud(str(tmp_path/'color'/result['maps'][0]['cloud']))
    assert np.array_equal(np.asarray(plain.points),np.asarray(colored.points))
    assert colored.has_colors() == has_colors
    assert result['color_status'] == status
    if has_colors:
        assert np.median(np.asarray(colored.colors),axis=0).tolist() == pytest.approx([230/255,20/255,10/255])
        assert result['maps'][0]['color_coverage'] == 1.
    assert all(p.read_bytes()==value for p,value in before.items())


def test_missing_rgb_frame_keeps_all_geometry_and_reports_partial(tmp_path,monkeypatch):
    import open3d as o3d
    run = make_run(tmp_path/'run')
    raw = run/'raw/depth/1000000000.png'
    assert cv2.imwrite(str(raw),np.full((60,80),800,np.uint16))
    second = run/'raw/depth/2000000000.png'
    second.write_bytes(raw.read_bytes())
    first = json.loads((run/'frames.jsonl').read_text())
    first.update(stamp_ns=2_000_000_000,path='raw/depth/2000000000.png')
    with (run/'frames.jsonl').open('a') as stream:
        stream.write(json.dumps(first)+'\n')
    with (run/'orb/map_0_keyframes.csv').open('a') as stream:
        stream.write('0,1,2.0,1.2,0,0,0,0,0,1\n')
    add_color(run)
    read_text = Path.read_text
    def checked(path,*args,**kwargs):
        if path.name == 'frames.jsonl':
            raise AssertionError('full frame index read')
        return read_text(path,*args,**kwargs)
    monkeypatch.setattr(Path,'read_text',checked)
    plain = fusion.reconstruct(run,voxel_size=.025,output_dir=tmp_path/'plain')
    result = fusion.reconstruct(run,voxel_size=.025,output_dir=tmp_path/'colored',color=True)
    assert result['status'] == 'complete' and result['color_status'] == 'partial'
    item = result['maps'][0]
    assert item['integrated_frames'] == 2 and item['colored_frames'] == 1
    assert item['color_coverage'] == pytest.approx(.5,abs=.01)
    cloud = o3d.io.read_point_cloud(str(tmp_path/'colored'/item['cloud']))
    original = o3d.io.read_point_cloud(str(tmp_path/'plain'/plain['maps'][0]['cloud']))
    assert np.array_equal(np.asarray(cloud.points),np.asarray(original.points))


def test_explicit_inertial_poses_cannot_bypass_source_initialization_gate(tmp_path):
    run = make_run(tmp_path/'run')
    (run/'orb_inertial').mkdir()
    (run/'orb_inertial/summary.json').write_text(json.dumps({'orb_fused':False,'exported_imu_initialized_maps':[]}))
    with pytest.raises(ValueError,match='initialized'):
        fusion.reconstruct(run,keyframe_dir=run/'orb',output_dir=tmp_path/'out',backend='orb_inertial')
    assert not (tmp_path/'out').exists()


@pytest.mark.parametrize('bad', ['path','frame_id','encoding','profile','distortion'])
def test_invalid_color_provenance_never_creates_success_summary(tmp_path,bad):
    run = make_run(tmp_path/'run')
    add_color(run)
    index = run/'frames.jsonl'
    rows = [json.loads(line) for line in index.read_text().splitlines()]
    if bad in ('path','frame_id','encoding'):
        rows[-1][bad] = {'path':'../outside.png','frame_id':'wrong','encoding':'mono8'}[bad]
        index.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    elif bad == 'profile':
        assert cv2.imwrite(str(run/'raw/color/1000000000.png'),np.zeros((20,20,3),np.uint8))
    else:
        path = run/'capture_calibration.json'
        info = json.loads(path.read_text())
        info['color']['distortion_model'] = 'equidistant'
        path.write_text(json.dumps(info))
    assert cv2.imwrite(str(run/'raw/depth/1000000000.png'),np.full((60,80),800,np.uint16))
    with pytest.raises(ValueError):
        fusion.reconstruct(run,output_dir=tmp_path/'out',color=True)
    assert not (tmp_path/'out/summary.json').exists()


def test_no_rgb_diagnostics_explain_fallback(tmp_path):
    run = make_run(tmp_path/'run')
    assert cv2.imwrite(str(run/'raw/depth/1000000000.png'),np.full((60,80),800,np.uint16))
    result = fusion.reconstruct(run,output_dir=tmp_path/'out',color=True)
    assert result['maps'][0]['color_unavailable_reason'] == 'missing_color_calibration'
    assert result['maps'][0]['color_volume_type'] == 'NoColor'


def test_color_frame_metadata_profile_must_agree_with_calibration(tmp_path):
    run = make_run(tmp_path/'run')
    add_color(run)
    index = run/'frames.jsonl'
    rows = [json.loads(line) for line in index.read_text().splitlines()]
    rows[-1]['width'] = 79
    index.write_text(''.join(json.dumps(row)+'\n' for row in rows))
    with pytest.raises(ValueError,match='profile'):
        fusion.reconstruct(run,depth_max=3.,output_dir=tmp_path/'out',color=True)


def test_color_symlink_cannot_escape_raw_recording(tmp_path):
    run = make_run(tmp_path/'run')
    add_color(run)
    target = tmp_path/'outside'
    target.mkdir()
    source = run/'raw/color/1000000000.png'
    (target/source.name).write_bytes(source.read_bytes())
    source.unlink()
    source.parent.rmdir()
    (run/'raw/color').symlink_to(target,target_is_directory=True)
    with pytest.raises(ValueError,match='path'):
        fusion.reconstruct(run,depth_max=3.,output_dir=tmp_path/'out',color=True)


def test_reconstruct_cli_selects_rgb_backend_and_explicit_pose_output(tmp_path,monkeypatch,capsys):
    run = make_run(tmp_path/'run')
    assert cv2.imwrite(str(run/'raw/depth/1000000000.png'),np.full((60,80),800,np.uint16))
    add_color(run)
    output = tmp_path/'cli'
    monkeypatch.setattr('sys.argv',['reconstruct',str(run),'--rgb','--backend','orb_offline',
                                   '--output',str(output),'--keyframe-dir',str(run/'orb')])
    fusion.main()
    result = json.loads(capsys.readouterr().out)
    assert result['color_status'] == 'complete'
    assert result['paths_relative_to'] == 'summary'
    assert (output/result['maps'][0]['mesh']).is_file()
    assert not (run/'dense_offline').exists()
