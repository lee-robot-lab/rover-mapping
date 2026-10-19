# ================================================================
# test_mission_runtime_blend.py
# 설명: MissionRuntime의 blend 추종 모드와 기본(unicycle) 모드 비트 단위 불변을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_mission_runtime_blend.py -q
# ================================================================
import hashlib
import json
import math

import numpy as np
import pytest
from site_mission_helpers import mission_fixture

GOLDEN_UNICYCLE_SHA256='f83354c9fc2647c440364f6b27510bd8a36c151e1ab4d7fff2d30b9aaaf77b94'   # 구현 전(f7a30ac 이후 HEAD)에서 캡처
GOLDEN_STALL_SHA256='62b69a0a2937f5db652704c3599819d90eda13227b4cf55f78aaae6f2f00fdab'   # 등판 실패·후퇴·재등판·하강·복귀 포함
GOLDEN_REPLAY_SHA256='83e5d6aa1d44365e7b7f6db2c60a66e18297c1f81485a12fff994ade57b61899'   # mission_replay stall30 commands.csv


def sample(mission,t=0.,xy=None,**extra):
    return {'stamp_s':t,'frame':'site','site_identity':mission.site.identity,
            'position_m':mission.start[:2].tolist() if xy is None else list(xy),
            'yaw_rad':0.,'tracking_valid':True,**extra}


def scripted_run(mission,steps=400,stall=False,pause_steps=()):
    """비물리 unicycle plant로 steps회 update한 모든 결과 dict. stall이면 30° 등판이 막혀 후퇴까지 간다."""
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    run=MissionRuntime(mission);xy=mission.start[:2].copy();yaw=0.;dt=.1;results=[]
    for k in range(steps):
        t=round(k*dt,6)
        out=run.update(sample(mission,t,xy.tolist(),yaw_rad=yaw,tracking_valid=k not in pause_steps),t)
        results.append(out)
        if run.state in ('complete','failed'):break
        if stall and run.state=='ascending' and run.route=='30' and xy[0]>1.3:continue
        c=out['command'];xy=xy+c['vx_m_s']*dt*np.array([math.cos(yaw),math.sin(yaw)]);yaw+=c['yaw_rate_rad_s']*dt
    return results


def digest(results):
    return hashlib.sha256(json.dumps(results,sort_keys=True).encode()).hexdigest()


def test_default_mode_output_is_bit_identical():
    """follower_blend=0이면 정상·일시정지·후퇴 경로 모두 구현 전 결과와 sha256이 같고 vy·follower 키가 없다."""
    first=scripted_run(mission_fixture(),pause_steps=(50,51))
    assert digest(first)==GOLDEN_UNICYCLE_SHA256
    second=scripted_run(mission_fixture(),steps=1200,stall=True,pause_steps=(30,))
    assert digest(second)==GOLDEN_STALL_SHA256
    assert {x['state'] for x in second}>={'retreating','returning','complete'}
    for result in first+second:
        assert set(result['command'])=={'vx_m_s','yaw_rate_rad_s'} and 'follower' not in result


def test_default_mode_replay_commands_csv_is_bit_identical(tmp_path):
    from experiments.handheld_mapping.mission_replay import demo_mission,replay
    replay(demo_mission(),tmp_path/'out','stall30')
    assert hashlib.sha256((tmp_path/'out'/'commands.csv').read_bytes()).hexdigest()==GOLDEN_REPLAY_SHA256


def wrap(value):
    return math.atan2(math.sin(value),math.cos(value))


def blend_mission(**settings):
    mission=mission_fixture();mission.settings.update({'follower_blend':1.,**settings});return mission


def plant(xy,yaw,cmd,dt):
    """비물리 planar plant: 몸 좌표 (vx, vy)를 세계로 돌려 적분한다. vy 키가 없으면 0."""
    vy=cmd.get('vy_m_s',0.);c,s=math.cos(yaw),math.sin(yaw)
    return xy+cmd['vx_m_s']*dt*np.array([c,s])+vy*dt*np.array([-s,c]),wrap(yaw+cmd['yaw_rate_rad_s']*dt)


def simulate(run,mission,steps,*,xy=None,yaw=0.,dt=.1,stop=None):
    """run.update를 steps회 돌리며 (결과, xy, yaw) 목록을 돌려준다. stop(결과)가 참이면 거기서 끝낸다."""
    xy=mission.start[:2].copy() if xy is None else np.array(xy,float);rows=[]
    for k in range(steps):
        t=round(k*dt,6)
        out=run.update(sample(mission,t,xy.tolist(),yaw_rad=yaw),t)
        rows.append((out,xy.copy(),yaw))
        if (stop is not None and stop(out)) or run.state in ('complete','failed'):break
        xy,yaw=plant(xy,yaw,out['command'],dt)
    return rows


def flat(run,degrees=0.):
    run._slope_deg=lambda position:degrees


def test_unicycle_pause_has_no_vy_key():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission)
    paused=run.update(sample(mission,tracking_valid=False),0.)
    assert paused['reason']=='tracking_lost' and paused['command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.}
    assert run.watchdog(1.)['command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.} and run.command=={'vx_m_s':0.,'yaw_rate_rad_s':0.}


def test_blend_pause_and_watchdog_carry_zero_vy():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    zero={'vx_m_s':0.,'vy_m_s':0.,'yaw_rate_rad_s':0.}
    mission=blend_mission();run=MissionRuntime(mission);assert run.command==zero
    assert run.update(sample(mission,tracking_valid=False),0.)['command']==zero
    assert run.watchdog(1.)['command']==zero
    run.update(sample(mission,2.),2.);run.update(sample(mission,2.1),2.1)
    paused=run.update(sample(mission,2.2,tracking_valid=False),2.2)
    assert paused['command']==zero and 'follower' not in paused and run.follower.initialized is False


def test_blend_active_result_carries_vy_and_follower_fields():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run)
    rows=simulate(run,mission,3)
    out=rows[-1][0]
    assert set(out['command'])=={'vx_m_s','vy_m_s','yaw_rate_rad_s'}
    assert set(out['follower'])=={'psi_ref','heading_gap_rad','travel_gap_rad','gated','realigning'}
    assert all(math.isfinite(v) for k,v in out['follower'].items() if k!='gated') and out['follower']['gated'] is False
    json.dumps(out,allow_nan=False)


def test_blend_corner_keeps_yaw_rate_under_limit_and_uses_vy():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run)
    run._set_path([[.5,.5],[1.5,.5],[1.5,1.5]])
    rows=simulate(run,mission,600,stop=lambda out:out['reason']=='phase_transition')
    assert run.state=='trial_entry',rows[-1][0]['reason']
    commands=[out['command'] for out,_,_ in rows if out['reason']=='active']
    assert max(abs(c['yaw_rate_rad_s']) for c in commands)<=.09+1e-12
    assert max(abs(c['vy_m_s']) for c in commands)>.01


def test_blend_retreating_reverses_without_rotating():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run)
    run.update(sample(mission,0.),0.)
    run.state='retreating';run.direction='down';run._set_path([[.5,.5],[.5-.4,.5]]);run.follower.invalidate()   # ascent_failed가 하는 전환; 병진 방향을 후진(θ=π)으로 곧바로 잡는다
    out=run.update(sample(mission,.1),.1)
    assert out['command']['yaw_rate_rad_s']==0 and out['command']['vx_m_s']<0 and abs(out['command']['vy_m_s'])<1e-6
    assert out['follower']['gated'] is False


def forward_then(mission,run,*,pre_path,pre_steps,dt=.1):
    """pre_path를 전진 주행(plant 적분)하고 (xy, yaw, t)를 돌려준다. follower는 초기화 없이 그대로 둔다."""
    run._set_path(pre_path);xy=mission.start[:2].copy();yaw=0.
    for k in range(pre_steps):
        t=round(k*dt,6);out=run.update(sample(mission,t,xy.tolist(),yaw_rad=yaw),t);xy,yaw=plant(xy,yaw,out['command'],dt)
    return xy,yaw,pre_steps*dt


def test_blend_retreat_after_forward_travel_stops_realigns_then_reverses_straight():
    """전진 중이던 상태(θ_app≈0)에서 invalidate 없이 후퇴로 넘어가면 옆걸음 대신 정지-재정렬-직선 후진한다."""
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();mission.start=np.array([1.5,1.5,0.]);run=MissionRuntime(mission);flat(run)
    xy,yaw,t=forward_then(mission,run,pre_path=[[1.5,1.5],[3.5,1.5]],pre_steps=40)
    assert run.follower.initialized and abs(run.follower.theta_app)<1e-3 and run.command['vx_m_s']>0
    run.state='retreating';run.direction='down';run._set_path([xy.tolist(),[.5,1.5]])
    rows=[]
    for k in range(150):
        t+=.1;out=run.update(sample(mission,t,xy.tolist(),yaw_rad=yaw),t)
        if out['reason']!='active':break   # 후퇴 경로 끝 도착(다음 단계로 전환)
        c=out['command'];rows.append(out);assert c['yaw_rate_rad_s']==0
        xy,yaw=plant(xy,yaw,c,.1)
    realign=[o for o in rows if o['follower']['realigning']]
    assert len(realign)>=20 and all(o['command']['vx_m_s']==o['command']['vy_m_s']==0 for o in realign)   # 정지 재정렬
    assert rows[0]['follower']['realigning'] and run.effort<run.settings['stuck_window_s']
    moving=[o['command'] for o in rows if math.hypot(o['command']['vx_m_s'],o['command']['vy_m_s'])>0]
    assert all(abs(wrap(math.atan2(c['vy_m_s'],c['vx_m_s'])-math.pi))<=math.radians(45)+.03 for c in moving)   # 옆걸음(90° 근처) 없음
    assert abs(wrap(math.atan2(moving[-1]['vy_m_s'],moving[-1]['vx_m_s'])-math.pi))<.02 and run.state=='ascending'   # 후퇴를 끝내고 15° 재등판으로 넘어갔다
    assert xy[0]<1.2 and abs(wrap(yaw))<1e-9   # 몸은 돌지 않고 뒤로 물러났다


def test_blend_realign_does_not_accumulate_effort_while_stopped():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();mission.start=np.array([1.5,1.5,0.]);run=MissionRuntime(mission);flat(run)
    xy,yaw,t=forward_then(mission,run,pre_path=[[1.5,1.5],[3.5,1.5]],pre_steps=30)
    run.state='retreating';run.direction='down';run._set_path([xy.tolist(),[.5,1.5]])
    efforts=[]
    for k in range(20):   # 위치 고정: 정지 재정렬 구간(약 2.4 s) 동안 effort는 0이어야 한다
        t+=.1;out=run.update(sample(mission,t,xy.tolist(),yaw_rad=yaw),t)
        if not out['follower']['realigning']:break
        efforts.append(run.effort)
    assert len(efforts)>=20 and max(efforts)<=.1+1e-9   # 첫 스텝만 직전 전진 명령 때문에 dt가 더해질 수 있다


def test_blend_summit_180_reversal_between_phases_has_no_sideways_travel():
    """ascending 끝(방위 0°) → 하강 시작(방위 180°): 정지-재정렬 뒤 후진 방향으로 출발하고 실패하지 않는다."""
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission(max_speed_m_s=.08);mission.routes['10']=np.array([[.5,1.5],[1.5,1.5],[3.5,1.5]])
    assert mission.validate()['accepted'];run=MissionRuntime(mission);flat(run)
    rows=simulate(run,mission,3000,dt=.1,stop=lambda out:out['state']=='complete')
    descent=[(o,y) for o,_,y in rows if o['state']=='descending' and o['reason']=='active']
    assert run.state=='complete' and descent
    realign=[o for o,_ in descent if o['follower']['realigning']]
    assert 15<=len(realign)<=30 and all(o['command']['vx_m_s']==o['command']['vy_m_s']==0 for o in realign)
    for o,yaw in descent:   # 출발 뒤 몸 좌표 방향은 후진(π) 45° 안, 각속도는 상한 안
        c=o['command'];assert abs(c['yaw_rate_rad_s'])<=.09+1e-12
        if math.hypot(c['vx_m_s'],c['vy_m_s'])>0 and o['follower']['travel_gap_rad']>math.pi-.5:
            assert abs(wrap(math.atan2(c['vy_m_s'],c['vx_m_s'])-math.pi))<=math.radians(60)   # 재개 오차 45° + 이후 yaw 변화


def test_blend_summit_180_turn_is_slow_and_not_gated():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission(max_speed_m_s=.08);mission.start=np.array([3.7,1.5,0.]);run=MissionRuntime(mission);flat(run)
    run.state='descending';run.route='10';run.direction='down';run._set_path([[3.7,1.5],[.4,1.5]])
    rows=simulate(run,mission,700,dt=.1,stop=lambda out:out['reason']=='phase_transition')
    active=[(out,yaw) for out,_,yaw in rows if out['reason']=='active']
    assert all(abs(out['command']['yaw_rate_rad_s'])<=.09+1e-12 and out['follower']['gated'] is False
               and math.hypot(out['command']['vx_m_s'],out['command']['vy_m_s'])>.01 for out,_ in active)
    times=[k*.1 for k,(out,_,yaw) in enumerate(rows) if abs(wrap(yaw-math.pi))<.1]
    assert times and 30<=times[0]<=45 and run.state!='failed'


def test_blend_ramp_gate_stops_then_resumes():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run,15.)
    run._set_path([[.5,.5],[1.5,.5+math.tan(math.radians(40))]])
    rows=simulate(run,mission,200,stop=lambda out:out['follower']['gated'] is False if 'follower' in out else False)
    states=[(out['follower']['travel_gap_rad'],out['follower']['gated']) for out,_,_ in rows]
    assert states[0][1] is True and rows[1][0]['command']['yaw_rate_rad_s']>0
    assert all(out['command']['vx_m_s']==out['command']['vy_m_s']==0 for out,_,_ in rows[:-1])
    assert all(gated for gap,gated in states[:-1] if gap>math.radians(15)) and any(math.radians(15)<gap<math.radians(25) for gap,_ in states)   # 15°~25° 구간에서는 계속 정지
    gap,gated=states[-1]
    assert gated is False and gap<=math.radians(15)+1e-9 and math.hypot(rows[-1][0]['command']['vx_m_s'],rows[-1][0]['command']['vy_m_s'])>0


def test_blend_stuck_counts_sideways_motion():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run)
    run._set_path([[.5,.5],[2.5,.5]]);run.state='ascending';run.route='30'
    efforts=[];commands=[]
    for k in range(60):   # 위치 고정, 몸은 +y를 보고 경로는 +x: 병진은 거의 순수 vy
        t=k*.1;out=run.update(sample(mission,t,yaw_rad=math.pi/2),t);efforts.append(run.effort);commands.append(out['command'])
        if run.state!='ascending':break
    c=commands[10]
    assert abs(c['vx_m_s'])<1e-6 and abs(c['vy_m_s'])>.01
    assert efforts[3]>0 and efforts[3]<efforts[10] and run.state=='retreating' and run.failed_routes==['30']


def test_blend_gate_stop_does_not_accumulate_effort():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run,15.)
    run._set_path([[.5,.5],[1.5,.5+math.tan(math.radians(40))]])
    efforts=[];states=[]
    for k in range(80):   # yaw 고정이라 계속 게이트 정지; stuck_window(3 s)보다 오래 둔다
        t=k*.1;out=run.update(sample(mission,t),t);efforts.append(run.effort);states.append(out['follower']['gated'])
    assert all(states) and max(efforts)==0 and run.state=='to_junction'


def test_zero_length_and_duplicate_path_points():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run)
    run._set_path([[.5,.5],[.5,.5],[.5,.5],[.5,.5],[.5,1.5]])
    first=run.update(sample(mission,0.,yaw_rad=.3),0.);assert math.isfinite(first['command']['vx_m_s'])
    here=mission.start[:2].copy();run.carrot.target=lambda index,lookahead:here   # 당근 점이 현재 위치와 같다(atan2(0,0))
    for k in range(1,6):
        out=run.update(sample(mission,k*.02,yaw_rad=.3),k*.02)
        assert all(math.isfinite(v) for v in out['command'].values())
        assert abs(wrap(out['follower']['psi_ref']-.3))<=.09*.02*5+1e-9


def test_blend_pose_jump_while_gated_still_pauses():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);flat(run,15.)
    run._set_path([[.5,.5],[1.5,.5+math.tan(math.radians(40))]])
    for k in range(5):out=run.update(sample(mission,k*.1),k*.1)
    assert out['follower']['gated'] is True
    jumped=run.update(sample(mission,.6,[.5,1.2]),.6)   # 0.7 m > max_pose_jump_m
    assert jumped['reason']=='pose_jump' and jumped['command']=={'vx_m_s':0.,'vy_m_s':0.,'yaw_rate_rad_s':0.}


def test_slope_lookup_outside_map_is_zero():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=blend_mission();run=MissionRuntime(mission);site=mission.site
    assert run._slope_deg(np.array([-5.,-5.]))==0. and run._slope_deg(np.array([50.,50.]))==0.
    assert run._slope_deg(np.array([np.nan,1.]))==0.
    col,row=site.cells(np.array([1.5,1.5]))
    site.slope_deg[row,col]=np.nan;assert run._slope_deg(np.array([1.5,1.5]))==0.
    site.slope_deg[row,col]=12.5;assert run._slope_deg(np.array([1.5,1.5]))==12.5


def test_safety_scale_applies_to_vx_vy_and_yaw_rate():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    outs=[]
    for scale in (None,.5):
        mission=blend_mission();run=MissionRuntime(mission);flat(run);run._set_path([[.5,.5],[1.5,.5],[1.5,1.5]])
        if scale is not None:run.safety_status={'scale':scale,'valid_until_s':None,'mode':'slow','reason':'test'}
        for k in range(4):out=run.update(sample(mission,k*.1,yaw_rad=.5),k*.1)   # 위치·yaw 고정: 같은 입력이면 배율만 다르다
        outs.append(out)
    full,half=outs
    assert full['command']['vx_m_s']>0 and full['command']['vy_m_s']<0 and full['command']['yaw_rate_rad_s']<0
    for key in full['command']:assert half['command'][key]==pytest.approx(full['command'][key]*.5)
    assert half['follower']==full['follower']
