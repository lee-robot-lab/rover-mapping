# ================================================================
# test_site_map.py
# 설명: STL 단위·좌표·구멍·벽·경사와 저장본 무결성을 실제 mesh로 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_site_map.py -q
# ================================================================
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest


def plane(slope: float = 0., size: float = 4.) -> o3d.geometry.TriangleMesh:
    """+X로 오르는 비대칭 직사각형 평면."""
    z = size*np.tan(np.deg2rad(slope))
    return o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[0,0,0],[size,0,z],[size,3, z],[0,3,0]]),
                                    o3d.utility.Vector3iVector([[0,1,2],[0,2,3]]))


def make_site(mesh=None, **kwargs):
    from experiments.handheld_mapping.site_map import SiteMap
    return SiteMap.from_mesh(plane() if mesh is None else mesh, resolution=.1, robot_radius=.05, **kwargs)


def test_mm_and_up_preserve_metric_bounds_and_slope():
    mesh=plane(15)
    mesh.rotate(o3d.geometry.get_rotation_matrix_from_xyz([np.pi/2,0,0]),center=[0,0,0])
    mesh.scale(1000,center=[0,0,0])
    site=make_site(mesh,units='mm',up=[0,-1,0])
    assert np.ptp(np.asarray(site.mesh.vertices)[:,0]) == pytest.approx(4.)
    assert site.slope_deg[site.observed].mean() == pytest.approx(15.,abs=.01)
    assert site.validate_path([[.5,.5],[3.5,2.5]],20)['accepted']
    assert not site.validate_path([[.5,.5],[3.5,2.5]],10)['accepted']


def test_reversed_winding_is_not_mistaken_for_no_floor():
    mesh=plane();mesh.triangles=o3d.utility.Vector3iVector(np.asarray(mesh.triangles)[:,::-1])
    assert make_site(mesh).validate_path([[.5,.5],[3.,2.]],20)['accepted']


def test_hole_and_footprint_unknown_are_rejected():
    # 가운데 빈 구간은 미관측이며 양쪽 분리 표면만 있다.
    left=plane(size=1.)
    island=plane(size=1.);island.translate([3.,0,0])
    site=make_site(left+island)
    assert not site.validate_path([[.5,1.],[3.5,1.]],20)['accepted']
    assert not site.validate_path([[.01,1.],[.5,1.]],20)['accepted']


def test_vertical_wall_is_obstacle_even_when_ray_misses_its_zero_width():
    wall=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[2,0,0],[2,3,0],[2,3,1],[2,0,1]]),
                                  o3d.utility.Vector3iVector([[0,1,2],[0,2,3]]))
    site=make_site(plane()+wall)
    assert not site.validate_path([[1.,1.],[3.,1.]],20)['accepted']


def test_overlapping_surfaces_are_unknown_instead_of_free():
    upper=plane(size=1.);upper.translate([1.,0,.5])
    site=make_site(plane()+upper)
    assert not site.validate_path([[1.2,1.],[1.8,1.]],35)['accepted']


@pytest.mark.parametrize('kwargs',[{'units':'cm'},{'up':[0,0,0]},{'resolution':0}, {'robot_radius':-1}])
def test_invalid_parameters_rejected(kwargs):
    from experiments.handheld_mapping.site_map import SiteMap
    with pytest.raises(ValueError):SiteMap.from_mesh(plane(),**kwargs)


def test_outside_nonfinite_and_huge_grid_rejected():
    site=make_site()
    for path in [[[-.1,1.],[1,1]],[[np.nan,1],[1,1]]]:
        assert not site.validate_path(path,20)['accepted']
    from experiments.handheld_mapping.site_map import SiteMap
    with pytest.raises(ValueError,match='grid'):SiteMap.from_mesh(plane(),resolution=.0001)


def test_save_reload_and_tamper_detection(tmp_path):
    from experiments.handheld_mapping.site_map import SiteMap
    site=make_site(plane(10));folder=tmp_path/'site';site.save(folder)
    loaded=SiteMap.load(folder)
    assert np.allclose(loaded.height_m,site.height_m,equal_nan=True)
    assert loaded.identity==site.identity
    with pytest.raises(FileExistsError):site.save(folder)
    with (folder/'grid.npz').open('ab') as stream:stream.write(b'tamper')
    with pytest.raises(ValueError,match='hash'):SiteMap.load(folder)


def test_stl_source_change_is_detected(tmp_path):
    from experiments.handheld_mapping.site_map import load_stl,SiteMap
    source=tmp_path/'team.stl';mesh=plane();mesh.compute_triangle_normals()
    assert o3d.io.write_triangle_mesh(str(source),mesh)
    site=load_stl(source,units='m',resolution=.1,robot_radius=.05)
    site.save(tmp_path/'site')
    source.write_bytes(source.read_bytes()+b'changed')
    with pytest.raises(ValueError,match='source'):SiteMap.load(tmp_path/'site')


def test_subcell_position_does_not_bypass_small_radius_wall_clearance():
    wall=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[2,0,0],[2,3,0],[2,3,1],[2,0,1]]),
                                  o3d.utility.Vector3iVector([[0,1,2],[0,2,3]]))
    from experiments.handheld_mapping.site_map import SiteMap
    site=SiteMap.from_mesh(plane()+wall,resolution=.1,robot_radius=.01)
    assert not site.validate_path([[1.999,1.],[1.999,2.]],20)['accepted']


def test_overlapping_full_surfaces_require_explicit_top_selection():
    from experiments.handheld_mapping.site_map import SiteMap
    upper=plane();upper.translate([0,0,.5]);mesh=plane()+upper
    with pytest.raises(ValueError,match='surface'):SiteMap.from_mesh(mesh,resolution=.1,robot_radius=.05)
    selected=SiteMap.from_mesh(mesh,resolution=.1,robot_radius=.05,surface_mode='top')
    assert selected.validate_path([[.5,.5],[3.5,2.5]],20)['accepted']
    assert np.nanmedian(selected.height_m)==pytest.approx(.5)
    assert selected.metadata['surface_mode']=='top'


def test_near_duplicate_upper_surface_cannot_hide_lower_floor():
    near=plane();near.translate([0,0,.49])
    top=plane();top.translate([0,0,.50])
    with pytest.raises(ValueError,match='surface'):make_site(plane()+near+top)


def test_old_grid_revision_requires_rebuild(tmp_path):
    import hashlib
    from experiments.handheld_mapping.site_map import SiteMap
    make_site().save(tmp_path/'site')
    path=tmp_path/'site/site.json';data=json.loads(path.read_text());data.pop('grid_revision',None)
    meta={k:v for k,v in data.items() if k not in ('identity','code_commit','assets')}
    data['identity']=hashlib.sha256(json.dumps(meta,sort_keys=True).encode()).hexdigest()
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError,match='revision'):SiteMap.load(tmp_path/'site')


@pytest.mark.parametrize('wall_top,accepted',[(.8,True),(1.5,False)])
def test_explicit_top_surface_ignores_only_buried_vertical_faces(wall_top,accepted):
    from experiments.handheld_mapping.site_map import SiteMap
    surface=plane();surface.translate([0,0,1.])
    wall=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(
        [[2,.5,0],[2,2.5,0],[2,2.5,wall_top],[2,.5,wall_top]]),
        o3d.utility.Vector3iVector([[0,1,2],[0,2,3]]))
    site=SiteMap.from_mesh(surface+wall,resolution=.1,robot_radius=.05,surface_mode='top')
    assert site.validate_path([[1,1.5],[3,1.5]],20)['accepted'] is accepted
