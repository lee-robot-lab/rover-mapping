# ================================================================
# run_variants_kinematic.py
# 설명: 실제 배포 MPPI와 회차 상태기를 점 로버 기구학으로 연결해 RUN3/4를 시험한다.
# 사용법: python deployment/ksrc/validation/run_variants_kinematic.py
# ================================================================
from pathlib import Path
import json
import math
import sys
import time
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'deployment/ksrc'))
import runner
from cli import fake_config
(ROOT/'outputs/run_variants').mkdir(parents=True,exist_ok=True)


def check(run: int) -> dict:
    nav=runner.Navigation(ROOT/'deployment/ksrc/mission',fake_config());rt=nav.new_runtime(run);rt.safety=None
    pose=np.array(rt.mission.start,float);steps=[];events=[];began=time.monotonic();last_stage=None
    for index in range(5000):
        stamp=index*.1
        sample=dict(stamp_s=stamp,frame='site',site_identity=rt.mission.site.identity,
                    position_m=[*pose[:2],0.],yaw_rad=float(pose[2]),tracking_valid=True,
                    tracking_quality='good',filter_status='ok',roll_rad=0.,pitch_rad=0.)
        out=rt.update(sample,now_s=stamp)
        if rt.stage!=last_stage:
            events.append(dict(stamp=stamp,stage=rt.stage,pose=pose.tolist()))
            print('RUN',run,round(stamp,1),rt.stage,pose.round(3).tolist(),flush=True);last_stage=rt.stage
        steps.append([stamp,*pose])
        if rt.state in ('complete','failed'):break
        v,w=out['command']['vx_m_s'],out['command']['yaw_rate_rad_s']
        # 30도 시도는 기존 trial_budget 정책으로 중단되며 실패 측정으로 부르지 않는다.
        mid=pose[2]+w*.05
        pose[:2]+=v*.1*np.array([math.cos(mid),math.sin(mid)])
        pose[2]=math.atan2(math.sin(pose[2]+w*.1),math.cos(pose[2]+w*.1))
        if time.monotonic()-began>120:break
    result=dict(run=run,state=rt.state,stage=rt.stage,sim_s=stamp,wall_s=time.monotonic()-began,
                reason=out['reason'],pose=pose.tolist(),stages=events,
                route_status=rt.route_status,validation='actual deployed MPPI + point kinematics; no wheel/contact physics')
    np.save(ROOT/f'outputs/run_variants/run{run}_track.npy',np.array(steps))
    (ROOT/f'outputs/run_variants/run{run}_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2),flush=True)
    return result


if __name__=='__main__':
    for number in (4,3):check(number)
