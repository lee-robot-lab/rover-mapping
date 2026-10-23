# ================================================================
# build_mission_cache.py
# 설명: 배포 RUN1~4 MPPI를 점기구학으로 실행하고 주변 pose의 표시 예측을 사전 계산한다.
# 사용법: OPENBLAS_NUM_THREADS=1 python deployment/broadcast/build_mission_cache.py
# ================================================================
from __future__ import annotations
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from scipy.ndimage import distance_transform_edt

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'deployment/ksrc'))
import runner
from cli import fake_config

OUT=Path(__file__).with_name('data')


def slip_estimate(site, xy: np.ndarray, yaw: float, v: float, gradients: tuple) -> float:
    """기존 미교정 견인 모델을 방향별 지면 경사에 적용한다."""
    cell=site.cells([xy])[0]
    gx,gy=gradients
    x,y=np.clip(cell,[0,0],[gx.shape[1]-1,gx.shape[0]-1])
    angle=yaw+(math.pi if v<0 else 0.)
    grade=float(gx[y,x]*math.cos(angle)+gy[y,x]*math.sin(angle))
    demand=np.clip((max(grade,0.)+.03)/.6,0.,.995)
    return float(np.clip(-.15*np.log1p(-demand),0.,.95))


def build() -> None:
    rows={key:[] for key in ('states','runs','stages','times','candidates','chosen','costs','slips','compute_ms')}
    report={};begin_all=time.perf_counter()
    for anchor,run in [(a,r) for a in (1,2) for r in range(1,5)]:
        nav=runner.Navigation(ROOT/'deployment/ksrc/mission',fake_config())
        nav.anchor_id=anchor
        if anchor==2:nav.anchor_yaw=-math.pi
        rt=nav.new_runtime(run)
        rt.safety=None
        site=rt.mission.site
        height=site.height_m.copy();valid=np.isfinite(height)
        nearest=distance_transform_edt(~valid,return_distances=False,return_indices=True)
        gy,gx=np.gradient(height[tuple(nearest)],site.resolution)
        worker=copy.copy(rt.planner);worker.rng=np.random.default_rng(700+run)
        original=rt.planner.plan
        previous=None;last_stage=None;stage_changes=[];start_count=len(rows['states']);tried=0
        pose=np.array(rt.mission.start,float)
        def plan(xy, yaw, *args, **kwargs):
            nonlocal previous,tried
            result=original(xy,yaw,*args,**kwargs)
            # 동일 단계에서 4cm 또는 8도 이상 이동했을 때 주변 상태를 갱신한다.
            state=np.array([*xy,yaw])
            if previous is not None and previous[0]==rt.stage:
                delta=math.atan2(math.sin(yaw-previous[1][2]),math.cos(yaw-previous[1][2]))
                if np.linalg.norm(state[:2]-previous[1][:2])<.04 and abs(delta)<math.radians(8):return result
            previous=(rt.stage,state.copy())
            normal=np.array([-math.sin(yaw),math.cos(yaw)])
            for lateral in (-.08,-.04,0.,.04,.08):
                for degrees in (-20,-10,0,10,20):
                    tried+=1
                    position=np.asarray(xy)+lateral*normal;angle=yaw+math.radians(degrees)
                    worker.controls=rt.planner.controls.copy()
                    start=time.perf_counter()
                    candidate=worker.plan(position,angle,*args,**kwargs)
                    elapsed=(time.perf_counter()-start)*1000
                    if not candidate['valid']:continue
                    selected=np.asarray(candidate['selected_trajectory'],np.float32)
                    paths=np.asarray(candidate['candidate_trajectories'],np.float32)
                    if len(paths)==0:continue
                    # 표시 배열 길이를 고정하고 부족한 후보는 선택 궤적으로 채운다.
                    if len(paths)<16:paths=np.concatenate([paths,np.repeat(selected[None],16-len(paths),axis=0)])
                    cells=site.cells(paths.reshape(-1,2)).reshape(paths.shape[:-1]+(2,))
                    costs=site.slope_deg[cells[...,1],cells[...,0]].mean(axis=1)
                    cmd=rt.command
                    v,w=float(cmd['vx_m_s']),float(cmd['yaw_rate_rad_s'])
                    values=dict(states=[*position,angle,v,w],runs=run,stages=rt.stage,times=stamp,
                                candidates=paths[:16].astype(np.float16),chosen=selected.astype(np.float16),
                                costs=costs[:16],slips=slip_estimate(site,position,angle,v,(gx,gy)),compute_ms=elapsed)
                    for key,value in values.items():rows[key].append(value)
            return result
        rt.planner.plan=plan
        for index in range(5000):
            stamp=index*.1
            sample=dict(stamp_s=stamp,frame='site',site_identity=site.identity,position_m=[*pose[:2],0.],
                        yaw_rad=float(pose[2]),tracking_valid=True,tracking_quality='good',filter_status='ok',
                        roll_rad=0.,pitch_rad=0.)
            result=rt.update(sample,now_s=stamp)
            if last_stage!=rt.stage:
                stage_changes.append(dict(stage=rt.stage,sim_s=stamp))
                print(f'ANCHOR{anchor} RUN{run} {stamp:.1f}s {rt.stage}: {len(rows["states"])-start_count} states',flush=True)
                last_stage=rt.stage
            if rt.state in ('complete','failed'):break
            v,w=result['command']['vx_m_s'],result['command']['yaw_rate_rad_s']
            vy=result['command'].get('vy_m_s',0.)
            mid=pose[2]+w*.05
            pose[:2]+=.1*np.array([v*math.cos(mid)-vy*math.sin(mid),v*math.sin(mid)+vy*math.cos(mid)])
            pose[2]=math.atan2(math.sin(pose[2]+w*.1),math.cos(pose[2]+w*.1))
        report[f'{anchor}:{run}']=dict(state=rt.state,sim_s=stamp,stages=stage_changes,tried=tried,cached=len(rows['states'])-start_count)
        if rt.state!='complete':raise RuntimeError(f'RUN{run} incomplete: {rt.state}/{rt.stage}')
    arrays={key:np.asarray(value,dtype='U32' if key=='stages' else np.int16 if key=='runs'
                          else np.float16 if key in ('candidates','chosen') else np.float32) for key,value in rows.items()}
    assert all(np.isfinite(value).all() for key,value in arrays.items() if key!='stages')
    np.savez_compressed(OUT/'mission_cache.npz',**arrays)
    size=(OUT/'mission_cache.npz').stat().st_size
    assert size<95*1024**2,'asset exceeds Git per-file budget'
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [ROOT/'deployment/ksrc/runner.py',ROOT/'deployment/ksrc/mission/scenario.json',
             ROOT/'deployment/ksrc/mission/mission.json',ROOT/'deployment/ksrc/mission/planning_map.json',
             ROOT/'deployment/ksrc/mission/site/site.json',ROOT/'deployment/ksrc/mission/site/grid.npz',
             ROOT/'deployment/ksrc/runtime/ksrc_runtime/handheld_mapping/region_mission.py',
             ROOT/'deployment/ksrc/runtime/ksrc_runtime/handheld_mapping/terrain_mppi.py']}
    summary=dict(site_identity=site.identity,planning_map='rocks10',runs={str(r):report[f'1:{r}'] for r in range(1,5)},
        anchor2_runs={str(r):report[f'2:{r}'] for r in range(1,5)},
        anchor_presets_rad={'1':-math.pi/2,'2':-math.pi},states=len(arrays['states']),bytes=size,wall_s=time.perf_counter()-begin_all,
        lateral_offsets_m=[-.08,-.04,0.,.04,.08],yaw_offsets_deg=[-20,-10,0,10,20],
        position_tolerance_m=.04,yaw_tolerance_deg=5,velocity_tolerance_m_s=.08,yaw_rate_tolerance_rad_s=.3,
        costs='mean map slope along candidate, for display color only; not MPPI objective',
        simulation='deployed MPPI + ideal SE2 kinematics; no wheel/contact/sensor physics',
        slip='uncalibrated directional traction model; no measured wheel slip',
        candidate_points=int(arrays['chosen'].shape[1]),source_sha256=hashes)
    (OUT/'mission_coverage.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':build()
