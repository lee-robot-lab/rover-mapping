# ================================================================
# site_mission_helpers.py
# 설명: 독립적으로 정의한 평면·세 경로를 임무 시험의 입력 fixture로 제공한다.
# 사용법: from site_mission_helpers import mission_fixture
# ================================================================
import numpy as np
import open3d as o3d


def mission_fixture():
    """출발부·갈림길·같은 정상을 잇는 세 경로."""
    from experiments.handheld_mapping.site_map import SiteMap
    from experiments.handheld_mapping.site_mission import SiteMission
    mesh=o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector([[0,0,0],[4,0,0],[4,3,0],[0,3,0]]),
                                  o3d.utility.Vector3iVector([[0,1,2],[0,2,3]]))
    site=SiteMap.from_mesh(mesh,resolution=.1,robot_radius=.05)
    routes={'10':[[.5,1.5],[1,.5],[3,.5],[3.5,1.5]],
            '15':[[.5,1.5],[1,2.5],[3,2.5],[3.5,1.5]],
            '30':[[.5,1.5],[1,1.5],[3.5,1.5]]}
    return SiteMission(site,[.5,.5,0],[.5,1.5],[3.5,1.5],routes)
