# ================================================================
# viewer.py
# 설명: 저장된 map별 TSDF mesh/point cloud를 독립된 Open3D 창으로 연다.
# 사용법: bash tools/view_mapping.sh [RUN_OR_PLY] [--map-id 0] [--cloud]
# ================================================================
import argparse
import json
from pathlib import Path
import time

ROOT = Path(__file__).resolve().parents[2]
FOLDERS = {'orb_offline':'dense_offline','orb':'dense','orbline':'dense_orbline','orb_inertial':'dense_inertial'}


def select_maps(target: Path | None, backend: str = 'auto', map_id: int | None = None,
                cloud: bool = False, sessions: Path | None = None) -> list[tuple[str,Path]]:
    """최신 촬영의 저장 결과만 선택하며 좌표계가 다른 map을 별도로 반환한다."""
    if backend not in ('auto',*FOLDERS):
        raise ValueError('unknown viewer backend')
    if target is None:
        root = sessions or ROOT/'outputs/handheld_mapping/sessions'
        candidates = [path for path in root.iterdir() if path.is_dir()] if root.is_dir() else []
        if not candidates:
            raise FileNotFoundError('촬영 기록이 없습니다. RUN 또는 PLY를 지정하세요.')
        target = max(candidates,key=lambda path:path.name)
    target = Path(target).expanduser().resolve()
    if target.is_file():
        if target.suffix.lower() != '.ply' or map_id is not None or backend != 'auto':
            raise ValueError('직접 PLY를 열 때는 backend/map-id를 지정하지 않습니다.')
        return [(target.name,target)]
    if not target.is_dir():
        raise FileNotFoundError(str(target))
    folders = list(FOLDERS.values()) if backend == 'auto' else [FOLDERS[backend]]
    folder = next((target/name for name in folders if (target/name/'summary.json').is_file()),None)
    if folder is None:
        raise FileNotFoundError(f'아직 저장된 3D 지도가 없습니다. 촬영 종료·지도 생성 후 열어주세요: {target}')
    summary = json.loads((folder/'summary.json').read_text())
    if not isinstance(summary,dict) or not isinstance(summary.get('maps'),list):
        raise ValueError('저장 지도 summary의 maps 형식이 올바르지 않습니다.')
    selected = []
    for row in summary['maps']:
        if not isinstance(row,dict):
            raise ValueError('저장 지도 map 항목 형식이 올바르지 않습니다.')
        identity = row.get('map_id')
        if not isinstance(identity,int) or isinstance(identity,bool) or identity < 0:
            raise ValueError('invalid map identity')
        if map_id is not None and identity != map_id:
            continue
        geometry = row.get('cloud' if cloud else 'mesh')
        if not isinstance(geometry,str):
            raise ValueError('저장 지도 summary에 유효한 PLY 경로가 없습니다.')
        path = (target/geometry).resolve()
        if not path.is_relative_to(folder.resolve()) or path.suffix.lower() != '.ply':
            raise ValueError('invalid saved map path')
        if not path.is_file():
            raise FileNotFoundError(str(path))
        selected.append((f'Map {identity}',path))
    if not selected:
        raise ValueError(f'요청한 map {map_id}의 저장 결과가 없습니다.')
    return selected


def show_maps(selected: list[tuple[str,Path]], cloud: bool = False) -> None:
    import open3d as o3d
    windows = []
    try:
        for label,path in selected:
            geometry = o3d.io.read_point_cloud(str(path)) if cloud else o3d.io.read_triangle_mesh(str(path))
            if cloud or len(geometry.triangles) == 0:
                if not cloud: geometry = o3d.io.read_point_cloud(str(path))
                if len(geometry.points) == 0: raise ValueError(f'빈 point cloud: {path}')
                if not geometry.has_normals(): geometry.estimate_normals()
            else:
                geometry.compute_vertex_normals()
            geometry.paint_uniform_color([.65,.78,.85])
            viewer = o3d.visualization.Visualizer()
            if not viewer.create_window(window_name=f'Rover 3D | {path.parent.parent.name} | {label}',
                                        width=1100,height=780):
                raise RuntimeError('3D 창 초기화 실패. tools/view_mapping.sh로 DISPLAY/X11 환경에서 실행하세요.')
            windows.append((viewer,geometry))
            if not viewer.add_geometry(geometry): raise RuntimeError(f'3D geometry 표시 실패: {path}')
            viewer.get_render_option().background_color=[.07,.09,.12]
            viewer.get_render_option().mesh_show_back_face=True
            control=viewer.get_view_control()
            control.set_lookat(geometry.get_center()); control.set_up([0,-1,0])
            control.set_front([0,0,-1]); control.set_zoom(.7)
            viewer.poll_events(); viewer.update_renderer()
            print('VIEWER_READY',label,path,flush=True)
        while windows:
            for viewer,geometry in windows[:]:
                if viewer.poll_events():
                    viewer.update_renderer()
                else:
                    viewer.destroy_window()
                    windows.remove((viewer,geometry))
            time.sleep(.02)
    finally:
        for viewer,_geometry in windows: viewer.destroy_window()


def main() -> None:
    parser=argparse.ArgumentParser(description='최신 촬영 또는 지정 RUN/PLY의 저장된 3D 지도 보기')
    parser.add_argument('target',nargs='?',type=Path)
    parser.add_argument('--backend',choices=['auto',*FOLDERS],default='auto')
    parser.add_argument('--map-id',type=int)
    parser.add_argument('--cloud',action='store_true',help='mesh 대신 point cloud 표시')
    args=parser.parse_args()
    try:
        selected=select_maps(args.target,args.backend,args.map_id,args.cloud)
        if len(selected)>1: print('여러 map은 각각 별도 좌표계·창으로 표시합니다.',flush=True)
        print('왼쪽 드래그 회전 | 휠 확대 | Shift+왼쪽 드래그 이동',flush=True)
        show_maps(selected,args.cloud)
    except (ValueError,FileNotFoundError,RuntimeError) as error:
        parser.exit(1,str(error)+'\n')


if __name__ == '__main__':
    main()
