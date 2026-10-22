# ================================================================
# rebuild_mission.py
# 설명: STL(구조물 또는 모래 포함)과 anchor 설정(JSON)에서 지도·세 경로·모서리 둥근 연결 경로를 한 번에 다시 만든다.
#       모래가 STL에 없으면 --sand-level로 평평한 모래 면을 더한다. 안전 검증을 못 통과하면 bundle을 만들지 않는다.
# 사용법:
#   python -m experiments.site_sim.rebuild_mission --stl NEW.stl --units m --anchors anchors.json --output NEW_FOLDER
#   python -m experiments.site_sim.rebuild_mission --stl NEW.stl --units m --sand-level 0.24 --fillet 0.25 --anchors anchors.json --output NEW_FOLDER
#   anchors.json 만들기: python -m experiments.site_sim.rebuild_mission --dump-anchors OLD_MISSION_FOLDER > anchors.json
# ================================================================
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np

from experiments.handheld_mapping.site_map import SiteMap, file_hash
from experiments.handheld_mapping.site_mission import SiteMission, dense_path
from experiments.handheld_mapping.storage import atomic_json

ANCHOR_KEYS = ('site_name', 'start', 'junction', 'summit', 'routes', 'connector_waypoints', 'settings')


def decollinear(points: np.ndarray) -> np.ndarray:
    """일직선 위의 중간점을 지우고 꺾이는 점만 남긴다."""
    points = [np.asarray(p, float) for p in points]
    out = [points[0]]
    for a, b, c in zip(points[:-2], points[1:-1], points[2:]):
        u, v = b-a, c-b
        if abs(u[0]*v[1]-u[1]*v[0]) > 1e-9 or np.dot(u, v) < 0:
            out.append(b)
    out.append(points[-1])
    return np.array(out)


def fillet(points: np.ndarray, radius: float, step_deg: float = 15.) -> np.ndarray:
    """90° 코너(U턴 제외)를 반지름 radius 원호로 바꾼다. 인접 구간이 모자라면 ValueError."""
    points = decollinear(points)
    out = [points[0]]
    for i in range(1, len(points)-1):
        a, v, c = points[i-1], points[i], points[i+1]
        d1, d2 = (v-a)/np.linalg.norm(v-a), (c-v)/np.linalg.norm(c-v)
        cross, dot = d1[0]*d2[1]-d1[1]*d2[0], float(np.dot(d1, d2))
        if abs(cross) < 1e-9 or dot < -.5:   # 직진 또는 U턴(≥120°)은 그대로
            out.append(v)
            continue
        theta = np.arccos(np.clip(dot, -1, 1))
        t = radius*np.tan(theta/2)
        if t > np.linalg.norm(v-a)/(2 if i > 1 else 1)+1e-9 and t > np.linalg.norm(v-a)-1e-9 or t > np.linalg.norm(c-v)-1e-9:
            raise ValueError(f'segment too short for R={radius} at {v.tolist()}')
        start, sign = v-d1*t, np.sign(cross)
        center = start+sign*np.array([-d1[1], d1[0]])*radius
        first = np.arctan2(*(start-center)[::-1])
        count = max(2, int(np.ceil(np.degrees(theta)/step_deg)))
        for k in range(count+1):
            angle = first+sign*theta*k/count
            out.append(center+radius*np.array([np.cos(angle), np.sin(angle)]))
    out.append(points[-1])
    out = np.array(out)
    return out[np.r_[True, np.linalg.norm(np.diff(out, axis=0), axis=1) > 1e-6]]


def flat_sand_mesh(mesh, level_m: float):
    """STL 범위 전체를 덮는 평평한 모래 면(삼각형 2개)을 STL 위에 더한다. mesh는 meter 단위여야 한다."""
    import open3d as o3d
    low, high = np.asarray(mesh.get_min_bound()), np.asarray(mesh.get_max_bound())
    corners = np.array([[low[0], low[1], level_m], [high[0], low[1], level_m],
                        [high[0], high[1], level_m], [low[0], high[1], level_m]])
    sand = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(corners),
                                     o3d.utility.Vector3iVector([[0, 1, 2], [0, 2, 3]]))
    return mesh+sand


def unsafe_spots(mission: SiteMission, report: dict) -> dict:
    """거부된 경로마다 안전하지 않은 표본의 개수와 x·y 범위를 알려 준다(anchor 좌표를 어디로 옮길지 판단용)."""
    site, spots = mission.site, {}
    for name, row in report['routes'].items():
        if row['accepted'] or row['reason'] != 'unknown_obstacle_clearance_or_slope':
            continue
        limit = mission.settings['trial_slope_deg' if name == '30' else 'normal_slope_deg']
        xy = dense_path(mission.routes[name], site.resolution/2)
        cells = site.cells(xy)
        bad = xy[~site.safe_mask(limit)[cells[:, 1], cells[:, 0]]]
        spots[name] = {'count': len(bad), 'x': [round(float(bad[:, 0].min()), 2), round(float(bad[:, 0].max()), 2)],
                       'y': [round(float(bad[:, 1].min()), 2), round(float(bad[:, 1].max()), 2)]}
    return spots


def crop_mesh(mesh, box: tuple[float, float, float, float]) -> int:
    """XY 경계 상자가 box(xmin, ymin, xmax, ymax)와 겹치지 않는 삼각형을 지운다(지도에 안 쓰는 큰 구역을 빼서 삼각형 수 한도를 지킨다).
    큰 평면 삼각형은 일부만 걸쳐도 남긴다. 지운 개수를 돌려준다."""
    vertices, triangles = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    corners = vertices[triangles][:, :, :2]
    low, high = corners.min(axis=1), corners.max(axis=1)
    outside = (high[:, 0] < box[0]) | (high[:, 1] < box[1]) | (low[:, 0] > box[2]) | (low[:, 1] > box[3])
    mesh.remove_triangles_by_mask(outside)
    mesh.remove_unreferenced_vertices()
    return int(outside.sum())


def drop_sliver_faces(mesh, max_height_m: float) -> int:
    """높이가 max_height_m보다 낮은 수직 면(CAD 이음매 등)을 지운다. 로버가 넘을 수 있는 높이라 벽이 아니다. mesh는 meter 단위이고 제자리에서 바뀐다."""
    mesh.compute_triangle_normals()
    vertices, triangles = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    height = np.ptp(vertices[triangles][:, :, 2], axis=1)
    drop = (np.abs(np.asarray(mesh.triangle_normals)[:, 2]) < .35) & (height < max_height_m)
    mesh.remove_triangles_by_mask(drop)
    mesh.remove_unreferenced_vertices()
    return int(drop.sum())


def build_mission(site: SiteMap, anchors: dict, radius: float, output: Path) -> SiteMission:
    """anchor 직각 경로를 둥글게 만들고 검증한 뒤 bundle로 저장한다."""
    routes = {key: np.asarray(value, float) for key, value in anchors['routes'].items()}
    connector = np.asarray(anchors['connector_waypoints'], float)
    if radius > 0:
        routes['15'], routes['10'] = fillet(routes['15'], radius), fillet(routes['10'], radius)   # 30° 경로는 직선이라 그대로 둔다
        connector = fillet(connector, radius)
    mission = SiteMission(site, anchors['start'], anchors['junction'], anchors['summit'], routes,
                          anchors['settings'], anchors.get('site_name', 'demo'), connector_waypoints=connector)
    report = mission.validate()
    if not report['accepted']:
        raise ValueError(json.dumps({**report, 'unsafe_spots': unsafe_spots(mission, report)}, ensure_ascii=False))
    mission.save(output)
    return mission


def write_provenance(mission: SiteMission, output: Path, stl: Path, units: str, sand_level: float | None,
                     seam_tol: float, dropped: int) -> None:
    """입력 STL 해시와 모래 처리 방식, 경로별 길이·최대 경사·벽 여유를 기록하고 preview를 그린다."""
    site, report = mission.site, {}
    from scipy.ndimage import distance_transform_edt
    clear = site.observed & ~site.blocked
    wall = distance_transform_edt(np.pad(clear, 1))[1:-1, 1:-1]*site.resolution
    for key, path in mission.routes.items():
        cells = site.cells(dense_path(path, site.resolution/2))
        report[key] = {'length_m': float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                       'max_slope_deg': float(site.slope_deg[cells[:, 1], cells[:, 0]].max()),
                       'min_wall_m': float(wall[cells[:, 1], cells[:, 0]].min())}
    atomic_json(output/'route_report.json', report)
    atomic_json(output/'simulation.json', {'simulation_only': True, 'pose_source': 'sim_ground_truth',
        'source_stl': str(stl.resolve()), 'source_stl_sha256': file_hash(stl), 'source_units': units,
        'flat_sand_level_m': sand_level, 'sliver_face_max_height_m': seam_tol, 'sliver_faces_dropped': dropped, 'surface_model': 'fixed mesh; no deformable soil'})
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 7))
    extent = [site.origin[0], site.origin[0]+site.observed.shape[1]*site.resolution,
              site.origin[1], site.origin[1]+site.observed.shape[0]*site.resolution]
    image = ax.imshow(np.where(site.observed, site.height_m, np.nan), origin='lower', extent=extent, cmap='terrain')
    fig.colorbar(image, ax=ax, label='Surface height (m)')
    for key, path in mission.routes.items():
        ax.plot(path[:, 0], path[:, 1], '-o', ms=3, label=key+' deg route')
    ax.plot(mission.start[0], mission.start[1], 'ks', label='start')
    ax.set(xlabel='X (m)', ylabel='Y (m)', aspect='equal')
    ax.legend()
    fig.tight_layout()
    fig.savefig(output/'preview.png', dpi=130)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--dump-anchors', type=Path, help='기존 mission 폴더에서 anchor JSON을 표준출력으로 낸다')
    parser.add_argument('--stl', type=Path)
    parser.add_argument('--units', choices=('m', 'mm'), default='m')
    parser.add_argument('--sand-level', type=float, default=None, help='STL에 모래가 없으면 이 높이(m)의 평평한 모래 면을 더한다')
    parser.add_argument('--crop', type=float, nargs=4, default=None, metavar=('XMIN', 'YMIN', 'XMAX', 'YMAX'), help='이 상자(m) 밖 삼각형은 지운다')
    parser.add_argument('--robot-radius', type=float, default=.21, help='지도의 로버 반지름(m). yw 시뮬 실행에 쓴 0.21(스워브 회전 반경 약 0.23 m에서 격자 오차 여유를 뺀 값)이 기본')
    parser.add_argument('--seam-tol', type=float, default=.02, help='이 높이(m)보다 낮은 수직 면은 벽으로 보지 않는다 (0이면 끔)')
    parser.add_argument('--anchors', type=Path)
    parser.add_argument('--fillet', type=float, default=.25, help='모서리 반지름 m (0이면 직각)')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.dump_anchors:
        data = json.loads((args.dump_anchors/'mission.json').read_text())
        json.dump({key: data[key] for key in ANCHOR_KEYS if key in data}, sys.stdout, indent=2)
        return
    if not (args.stl and args.anchors and args.output):
        parser.error('--stl, --anchors, --output required')
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(output)
    import open3d as o3d
    mesh = o3d.io.read_triangle_mesh(str(args.stl))
    mesh.scale(1. if args.units == 'm' else .001, center=[0, 0, 0])
    if args.sand_level is not None:
        mesh = flat_sand_mesh(mesh, args.sand_level)
    if args.crop:
        print('crop으로 지운 삼각형:', crop_mesh(mesh, tuple(args.crop)), file=sys.stderr)
    dropped = drop_sliver_faces(mesh, args.seam_tol) if args.seam_tol > 0 else 0
    site = SiteMap.from_mesh(mesh, units='m', source=args.stl.resolve(), resolution=.025, robot_radius=args.robot_radius, surface_mode='top')
    mission = build_mission(site, json.loads(args.anchors.read_text()), args.fillet, output)
    write_provenance(mission, output, args.stl, args.units, args.sand_level, args.seam_tol, dropped)
    print(json.dumps({'mission': str(output), 'sliver_faces_dropped': dropped, 'validation': mission.validate()['accepted'],
                      'route_report': json.loads((output/'route_report.json').read_text())}, ensure_ascii=False))


if __name__ == '__main__':
    main()
