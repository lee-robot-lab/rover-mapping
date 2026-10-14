# ================================================================
# test_overlap.py
# 설명: IR 위 관측 색의 카메라 투영·occlusion·invalid depth·원본 보존을 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_overlap.py -q
# ================================================================
import numpy as np
import pytest
from experiments.handheld_mapping.coverage import CoverageGrid


def info(width=4,height=3,focal=2.):
    return {'width':width,'height':height,'k':[focal,0,1.,0,focal,1.,0,0,1.],
            'depth_scale_m':.001,'T_left_depth':np.eye(4).tolist(),'d':[0.]*5}


def test_overlay_intrinsics_extrinsic_and_original_preserved():
    from experiments.handheld_mapping.overlap import overlap_image
    original=np.full((3,4),100,np.uint8)
    depth=np.zeros((3,4),np.uint16); depth[1,1]=1000
    depth_before=depth.copy(); before=original.copy()
    calibration=info(); calibration['T_left_depth'][0][3]=.5
    grid=CoverageGrid()
    result=overlap_image(original,depth,calibration,info(),np.eye(4),grid)
    assert result.shape==(3,4,3) and result.dtype==np.uint8
    assert np.array_equal(original,before) and np.array_equal(depth,depth_before)
    assert np.array_equal(result[1,1],[100]*3)
    assert not np.array_equal(result[1,2],[100]*3)  # fx*.5m/1m=1px right
    assert result[1,2,0]>result[1,2,2]  # first view yellow
    grid.counts.fill(3)
    green=overlap_image(original,depth,calibration,info(),np.eye(4),grid)
    assert green[1,2,1]>green[1,2,0]


def test_projection_zbuffer_only_colors_nearest_surface():
    from experiments.handheld_mapping.overlap import overlap_image
    original=np.full((3,4),100,np.uint8)
    depth=np.zeros((3,4),np.uint16)
    depth[1,0]=1000; depth[1,1]=2000
    depth_info=info(); left_info=info(focal=.01)
    grid=CoverageGrid(resolution=.25)
    grid.counts.fill(1)
    # Near point(-.5,0,1) yellow, far point(0,0,2) green. Both project into left(1,1).
    row=int((2-grid.bounds[2])/grid.resolution); col=int((0-grid.bounds[0])/grid.resolution)
    grid.counts[row,col]=3
    result=overlap_image(original,depth,depth_info,left_info,np.eye(4),grid)
    assert result[1,1,0]>result[1,1,2]
    assert result[1,1,0]>result[1,1,1]


def test_invalid_depth_outside_grid_or_behind_camera_remains_uncolored():
    from experiments.handheld_mapping.overlap import overlap_image
    original=np.full((3,4),100,np.uint8)
    expected=np.repeat(original[:,:,None],3,axis=2)
    grid=CoverageGrid()
    for value in (0,200,4000):
        result=overlap_image(original,np.full((3,4),value,np.uint16),info(),info(),np.eye(4),grid)
        assert np.array_equal(result,expected)
    calibration=info(); calibration['T_left_depth'][2][3]=-5.
    result=overlap_image(original,np.full((3,4),1000,np.uint16),calibration,info(),np.eye(4),grid)
    assert np.array_equal(result,expected)
    pose=np.eye(4); pose[0,3]=100.
    result=overlap_image(original,np.full((3,4),1000,np.uint16),info(),info(),pose,grid)
    assert np.array_equal(result,expected)


def test_overlay_rejects_nonrectified_ir_and_profile_mismatch():
    from experiments.handheld_mapping.overlap import overlap_image
    calibration=info(); calibration['d']=[.1,0,0,0,0]
    with pytest.raises(ValueError):
        overlap_image(np.zeros((3,4),np.uint8),np.zeros((3,4),np.uint16),info(),calibration,np.eye(4),CoverageGrid())


def test_live_caches_matching_original_overlay_and_stamp_without_changing_saved_ir(tmp_path):
    import cv2
    import json
    import zlib
    from experiments.handheld_mapping.live import LiveMapper
    from experiments.handheld_mapping.storage import RunWriter
    writer=RunWriter(tmp_path/'run')
    calibration=info()
    writer.accept({'kind':'calibration','depth':calibration,'stereo':{'left':calibration}},b'')
    mapper=LiveMapper(writer.path,demo=True)
    original=np.arange(12,dtype=np.uint8).reshape(3,4)+100
    try:
        for side,image in [('left',original),('depth',np.full((3,4),1000,np.uint16))]:
            header={'kind':'image','stream':side,'stamp_ns':1000000000,'width':4,'height':3,
                    'step':image.strides[0],'encoding':'mono8' if side=='left' else '16UC1','compression':'zlib'}
            writer.accept(header,zlib.compress(image.tobytes()))
            mapper.offer(header)
        mapper.offer_pose({'stamp_ns':1000000000,'map_id':0,'position':[0,0,0],'quaternion_xyzw':[0,0,0,1]})
        snapshot=mapper.snapshot()
        assert snapshot['overlay_stamp_ns']==1000000000
        assert np.array_equal(snapshot['overlay_original'],original)
        assert snapshot['overlay'].shape==(3,4,3)
        assert not np.array_equal(snapshot['overlay'],np.repeat(original[:,:,None],3,axis=2))
        assert np.array_equal(cv2.imread(str(writer.path/'raw/left/1000000000.png'),0),original)
    finally:
        mapper.finish(); writer.close()
    summary=json.loads((writer.path/'orb_live/summary.json').read_text())
    assert summary['error'] is None
    assert 'overlay' not in summary and 'overlay_original' not in summary


def test_overlay_render_is_throttled_but_new_map_forces_fresh_frame(tmp_path):
    import zlib
    from experiments.handheld_mapping.live import LiveMapper
    from experiments.handheld_mapping.storage import RunWriter
    writer=RunWriter(tmp_path/'run')
    calibration=info()
    writer.accept({'kind':'calibration','depth':calibration,'stereo':{'left':calibration}},b'')
    mapper=LiveMapper(writer.path,demo=True)
    def frame(stamp,value,map_id):
        for side,image in [('left',np.full((3,4),value,np.uint8)),('depth',np.full((3,4),1000,np.uint16))]:
            header={'kind':'image','stream':side,'stamp_ns':stamp,'width':4,'height':3,'step':image.strides[0],
                    'encoding':'mono8' if side=='left' else '16UC1','compression':'zlib'}
            writer.accept(header,zlib.compress(image.tobytes())); mapper.offer(header)
        mapper.offer_pose({'stamp_ns':stamp,'map_id':map_id,'position':[0,0,0],'quaternion_xyzw':[0,0,0,1]})
    try:
        frame(1000000000,100,0)
        frame(1000000001,120,0)
        assert mapper.snapshot()['overlay_stamp_ns']==1000000000
        assert np.all(mapper.snapshot()['overlay_original']==100)
        frame(1000000002,140,421)
        assert mapper.snapshot()['overlay_stamp_ns']==1000000002
        assert np.all(mapper.snapshot()['overlay_original']==140)
    finally:mapper.finish(); writer.close()
