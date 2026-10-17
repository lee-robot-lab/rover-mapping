# ================================================================
# conftest.py
# 설명: 서버 임무 시험용 평면 지형·독립 경로 fixture를 제공한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim
# ================================================================
import numpy as np
import open3d as o3d
import pytest
from experiments.handheld_mapping.site_map import SiteMap
from experiments.handheld_mapping.site_mission import SiteMission


@pytest.fixture
def mission():
    """벽 없는 평면에서 공통 정상으로 이르는 세 경로를 제공한다."""
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[0,0,0],[4,0,0],[4,3,0],[0,3,0]]),
        o3d.utility.Vector3iVector([[0,1,2],[0,2,3]]))
    site=SiteMap.from_mesh(mesh,resolution=.1,robot_radius=.05)
    return SiteMission(site,[.5,.5,0],[.5,1.5],[3.5,1.5],{
        '30':[[.5,1.5],[1,1.5],[3.5,1.5]],
        '15':[[.5,1.5],[1,2.4],[3,2.4],[3.5,1.5]],
        '10':[[.5,1.5],[1,.5],[3,.5],[3.5,1.5]]})
