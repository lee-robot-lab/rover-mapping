# ================================================================
# harness_kinematic.py
# 설명: Isaac 없이 RegionMission(+MPPI)을 점질량(vx·yaw rate를 그대로 적분) 로버와 폐루프로 돌려 단계 순서·경로 연결을 확인한다.
#       마찰·미끄러짐·ORB 오차가 없는 이상적 조건이라 임무 논리와 지도·구역 연결만 시험한다.
# 사용법: PYTHONPATH=code_yw17 python harness_kinematic.py MISSION_DIR run1|run2 [--out events.json]
# ================================================================
import json, math, sys, time
from pathlib import Path
import numpy as np
from experiments.handheld_mapping.site_mission import SiteMission
from experiments.handheld_mapping.region_mission import RegionMission

def run(folder, profile, max_s=900., dt=.05):
    mission = SiteMission.load(Path(folder), load_mesh=False)
    scenario = json.loads((Path(folder)/'scenario.json').read_text())
    data = json.loads((Path(folder)/scenario['regions_file']).read_text()); scenario['regions'], scenario['portals'] = data['regions'], data['portals']
    mission.settings.update(summit_look=1. if profile == 'run1' else 0.,
                            trial30_forward_limit_s=10. if profile == 'run1' else .5, trial30_elapsed_limit_s=25. if profile == 'run1' else .5)
    runtime = RegionMission(mission, hold_s=0., scenario=scenario)
    x, y, yaw = (float(v) for v in mission.start)
    t, log, last = 0., [], None
    while t < max_s and runtime.state not in ('complete', 'failed'):
        sample = dict(stamp_s=t, position_m=[x, y, 0.], yaw_rad=yaw, pitch_rad=0., roll_rad=0., frame='site', site_identity=mission.site.identity,
                      tracking_valid=True, pose_epoch=0, tracking_quality='good', filter_status='ok')
        out = runtime.update(sample, t)
        cmd = out['command']; vx, w = float(cmd['vx_m_s']), float(cmd['yaw_rate_rad_s'])
        x += vx*math.cos(yaw)*dt; y += vx*math.sin(yaw)*dt; yaw = (yaw+w*dt+math.pi) % (2*math.pi)-math.pi; t += dt
        if int(t/dt) % 20 == 0: log.append((round(t, 2), round(x, 3), round(y, 3), round(math.degrees(yaw), 1), runtime.stage))
    stages = [(round(e['stamp_s'], 1), e['stage']) for e in runtime.events if e['type'] == 'region_phase']
    return dict(profile=profile, final_state=runtime.state, sim_s=round(t, 1), stages=stages, events=[e for e in runtime.events if e['type'] != 'region_phase'], path=log, end=(round(x, 3), round(y, 3), round(math.degrees(yaw), 1)))

if __name__ == '__main__':
    result = run(sys.argv[1], sys.argv[2])
    print(json.dumps({k: v for k, v in result.items() if k not in ('path', 'events')}, ensure_ascii=False))
    for e in result['events']: print(' ', e['type'], round(e['stamp_s'], 1), e.get('reason', ''))
    if '--out' in sys.argv: json.dump(result, open(sys.argv[sys.argv.index('--out')+1], 'w'))
