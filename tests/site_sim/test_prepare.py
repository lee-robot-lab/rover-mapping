# ================================================================
# test_prepare.py
# 설명: 서버 grid-only 로딩의 안전성, 원본 축·단위와 경로 연결을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_prepare.py
# ================================================================
import subprocess
import sys

import numpy as np
import open3d as o3d
import pytest
from experiments.handheld_mapping.site_mission import SiteMission


def test_grid_only_load_works_without_open3d_and_preserves_validation(mission,tmp_path):
    folder=tmp_path/'mission';mission.save(folder)
    script="""
import builtins,sys
original=builtins.__import__
def guarded(name,*args,**kwargs):
    if name.split('.')[0]=='open3d':raise ImportError('Open3D unavailable')
    return original(name,*args,**kwargs)
builtins.__import__=guarded
from pathlib import Path
from experiments.handheld_mapping.site_mission import SiteMission
m=SiteMission.load(Path(sys.argv[1]),load_mesh=False)
assert m.site.mesh is None
assert m.validate()['accepted']
assert m.site.validate_path([[.5,.5],[.01,.01]],20)['accepted'] is False
"""
    result=subprocess.run([sys.executable,'-c',script,str(folder)],capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_grid_only_load_still_rejects_tampered_mesh(mission,tmp_path):
    folder=tmp_path/'mission';mission.save(folder)
    with (folder/'site/mesh.ply').open('ab') as out:out.write(b'changed')
    with pytest.raises(ValueError,match='hash'):SiteMission.load(folder,load_mesh=False)


def test_grid_only_save_requires_mesh_before_creating_folder(mission,tmp_path):
    folder=tmp_path/'mission';mission.save(folder)
    loaded=SiteMission.load(folder,load_mesh=False)
    with pytest.raises(ValueError,match='mesh'):loaded.site.save(tmp_path/'copy')
    assert not (tmp_path/'copy').exists()


def test_metric_mesh_uses_xy_centers_and_declared_stl_scale(tmp_path):
    from experiments.site_sim.prepare import metric_arena_mesh
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[100,0,-10],[110,0,-10],[100,10,-10]]),
        o3d.utility.Vector3iVector([[0,1,2]]));mesh.compute_triangle_normals()
    stl=tmp_path/'arena.stl';o3d.io.write_triangle_mesh(str(stl),mesh)
    h=np.array([[0,.003,.006],[.01,.013,.016],[.02,.023,.026],[.03,.033,.036]])
    sand=tmp_path/'sand.npz';np.savez(sand,height=h,x0=1.,y0=2.,cell=.1)
    result=metric_arena_mesh(stl,sand);p=np.asarray(result.vertices)
    assert np.any(np.all(np.isclose(p,[10,0,-1]),axis=1))
    assert np.any(np.all(np.isclose(p,[1.15,2.25,.016]),axis=1))


def test_sand_nan_and_discontinuity_are_not_bridged(tmp_path):
    from experiments.site_sim.prepare import metric_arena_mesh
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[100,0,-10],[110,0,-10],[100,10,-10]]),
        o3d.utility.Vector3iVector([[0,1,2]]));mesh.compute_triangle_normals()
    stl=tmp_path/'arena.stl';o3d.io.write_triangle_mesh(str(stl),mesh)
    h=np.zeros((8,8));h[3,:]=np.nan;h[5:]=1.
    sand=tmp_path/'sand.npz';np.savez(sand,height=h,x0=0.,y0=0.,cell=.01)
    result=metric_arena_mesh(stl,sand);p=np.asarray(result.vertices);tri=np.asarray(result.triangles)
    for face in p[tri]:
        if face[:,0].max()<1:
            assert np.isfinite(face).all()
            assert np.ptp(face[:,2])<=.02
            assert not (face[:,0].min()<.035<face[:,0].max())


def test_route_anchors_and_every_segment_are_validated(mission):
    from experiments.site_sim.prepare import route_through
    route=route_through(mission.site,[[.5,.5],[.5,2.4],[3.5,2.4]],20)
    assert np.allclose(route[0],[.5,.5]) and np.allclose(route[-1],[3.5,2.4])
    assert any(np.allclose(p,[.5,2.4]) for p in route)
    assert mission.site.validate_path(route,20)['accepted']
    with pytest.raises(ValueError):route_through(mission.site,[[.01,.01],[3,2]],20)


def test_prepare_writes_reloadable_bundle_with_finite_attempt_budget(tmp_path,monkeypatch):
    from experiments.site_sim import prepare as module
    plane=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[2.5,-1.61,0],[6.51,-1.61,0],
        [6.51,1.6,0],[2.5,1.6,0]]),o3d.utility.Vector3iVector([[0,1,2],[0,2,3]]))
    monkeypatch.setattr(module,'metric_arena_mesh',lambda *args:plane)
    stl=tmp_path/'input.stl';sand=tmp_path/'input.npz';stl.write_bytes(b'source1');sand.write_bytes(b'source2')
    output=tmp_path/'prepared';module.prepare(stl,sand,output)
    loaded=SiteMission.load(output,load_mesh=False)
    assert loaded.validate()['accepted']
    assert np.allclose(loaded.start,[2.9,1.2,0.])
    assert 120<=loaded.settings['max_attempt_s']<900
    assert (output/'preview.png').is_file()
