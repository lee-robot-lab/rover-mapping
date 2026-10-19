# ================================================================
# build_rect_mission.py
# 설명: abD1018 번들에서 갈림길을 램프 중심선(y=0.40)으로 옮기고 모든 경로를 축 평행 직선·90° 코너로 바꾼 번들을 만든다(지도 검증 포함).
# 사용법:
#   .venv-mapping/bin/python -I docs/experiments/handoff-2026-10-19-scripts/build_rect_mission.py [출력폴더]
#   기본 출력: outputs/site_sim/cy_missions/rect_mission
# ================================================================
import sys
sys.path.insert(0, '.')
import json
import shutil
from pathlib import Path
import numpy as np
from experiments.handheld_mapping.site_mission import SiteMission, path_with_heading

SRC = Path('outputs/site_sim/abD1018/mission_bundle')
OUT = Path(sys.argv[1] if len(sys.argv) > 1 else 'outputs/site_sim/cy_missions/rect_mission')
J, S, XR = [3.7, 0.4], [5.35, 0.4], 6.125   # 갈림길, 정상, 오른쪽 세로 통로 x(x=6.15는 y≈-0.1에서 안전 마스크에 막힌다)
ROUTES = {'30': [J, [3.8, 0.4], [4.5, 0.4], S],          # route 30의 두 번째 점이 trial_entry
          '15': [J, [3.7, 1.2], [4.7, 1.2], [XR, 1.2], [XR, 0.4], S],
          '10': [J, [3.7, 1.2], [2.9, 1.2], [2.9, -1.25], [4.7, -1.25], [XR, -1.25], [XR, 0.4], S]}
CONNECTOR = [[2.9, 1.2], [3.7, 1.2], J]

if OUT.exists():
    shutil.rmtree(OUT)
shutil.copytree(SRC, OUT)
orig = SiteMission.load(OUT, load_mesh=False)
new = SiteMission(orig.site, orig.start, J, S, ROUTES, orig.settings, orig.site_name, connector_waypoints=CONNECTOR)
report = new.validate()
assert report['accepted'], report
mission = json.load(open(OUT / 'mission.json'))
mission.update(junction=J, routes=ROUTES, connector_waypoints=CONNECTOR, validation=report)
mission['policy']['trial_entry'] = ROUTES['30'][1]
json.dump(mission, open(OUT / 'mission.json', 'w'), indent=2, ensure_ascii=False)
for name in ('10', '15', '30'):
    for direction in ('up', 'down'):
        np.savetxt(OUT / 'routes' / f'{name}_{direction}.csv', path_with_heading(new.path(name, direction)),
                   delimiter=',', header='x_m,y_m,yaw_rad', comments='')
SiteMission.load(OUT, load_mesh=False)
print('saved', OUT)
