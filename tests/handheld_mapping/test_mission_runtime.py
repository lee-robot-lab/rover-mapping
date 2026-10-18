# ================================================================
# test_mission_runtime.py
# 설명: 유효 site pose만으로 방문·등판 실패·후퇴·우회·복귀 상태를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_mission_runtime.py -q
# ================================================================
import numpy as np
import pytest
from site_mission_helpers import mission_fixture


def sample(mission,t=0.,xy=None,**extra):
    return {'stamp_s':t,'frame':'site','site_identity':mission.site.identity,
            'position_m':mission.start[:2].tolist() if xy is None else list(xy),
            'yaw_rad':0.,'tracking_valid':True,**extra}


def test_first_pose_must_be_near_declared_start():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission)
    result=run.update(sample(mission,xy=mission.summit),0.)
    assert result['command']['vx_m_s']==0
    assert result['reason']=='initial_pose_away_from_start'
    assert result['visits']['summit']=='pending'


@pytest.mark.parametrize('extra,now,reason',[
    ({'tracking_valid':False},0.,'tracking_lost'),
    ({'stamp_s':-1.},1.,'stale_pose'),
    ({'frame':'orb_world'},0.,'unaligned_frame'),
    ({'site_identity':'other'},0.,'unaligned_frame'),
    ({'yaw_rad':float('nan')},0.,'invalid_pose'),
])
def test_invalid_pose_never_commands_motion(extra,now,reason):
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission)
    out=run.update(sample(mission,**extra),now)
    assert out['reason']==reason
    assert out['command']['vx_m_s']==out['command']['yaw_rate_rad_s']==0.
    assert out['visits']['summit']=='pending'


def test_duplicate_and_teleport_do_not_complete_goal():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission)
    run.update(sample(mission),0.)
    assert run.update(sample(mission),0.)['reason']=='nonmonotonic_pose'
    assert run.update(sample(mission,.1,mission.summit),.1)['reason']=='pose_jump'
    assert run.visits['summit']=='pending'


def test_command_limits_and_tracking_pause_do_not_create_stuck_event():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission)
    for t in np.arange(0,2,.1):
        out=run.update(sample(mission,float(t)),float(t))
        assert abs(out['command']['vx_m_s'])<=mission.settings['max_speed_m_s']
        assert abs(out['command']['yaw_rate_rad_s'])<=mission.settings['max_yaw_rate_rad_s']
    run.update(sample(mission,2.,tracking_valid=False),2.)
    out=run.update(sample(mission,100.),100.)
    assert out['state']!='retreating'
    assert not any(event['type']=='ascent_failed' for event in run.events)


def test_pose_epoch_change_requires_realignment_even_without_large_jump():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission)
    run.update(sample(mission,pose_epoch=1),0.)
    out=run.update(sample(mission,.1,pose_epoch=2),.1)
    assert out['reason']=='pose_epoch_changed'
    assert out['command']['vx_m_s']==0.


def test_jsonl_error_always_appends_stop_command(tmp_path):
    import json
    from experiments.handheld_mapping.mission_runtime import run_jsonl
    mission=mission_fixture();source=tmp_path/'input.jsonl'
    source.write_text(json.dumps({**sample(mission),'now_s':0.})+'\n{broken\n')
    with pytest.raises(ValueError):run_jsonl(mission,source,tmp_path/'output')
    last=json.loads((tmp_path/'output/commands.jsonl').read_text().splitlines()[-1])
    assert last['command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.}


def advance_trial(run,mission,end_x):
    """독립 관측열을 출발→갈림길→30° 직선 구간으로 이동시킨다."""
    points=[[.5,.5]]+[[.5,y] for y in np.arange(.6,1.51,.1)]+[[x,1.5] for x in np.arange(.6,end_x+.01,.1)]
    for index,point in enumerate(points):
        t=round(index*.1,6);out=run.update(sample(mission,t,point),t)
    return t,out


def test_tracking_resume_cannot_skip_unsafe_segment_to_summit():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    from experiments.handheld_mapping.mission_replay import demo_mission
    mission=demo_mission();mission.site.robot_radius=.05;mission.site.blocked[30,65]=True
    mission.routes['30']=np.array([mission.junction,[1,1.5],[3.1,1.5],[3.1,2.1],[3.6,2.1],[3.6,1.5],mission.summit])
    assert mission.validate()['accepted']
    run=MissionRuntime(mission);t,_=advance_trial(run,mission,3.1)
    run.update(sample(mission,t+.1,[3.1,1.5],tracking_valid=False),t+.1)
    out=run.update(sample(mission,t+.2,[3.5,1.5]),t+.2)
    assert out['reason']=='unsafe_observed_motion'
    assert out['visits']['summit']=='pending'
    out=run.update(sample(mission,t+.3,[3.5,1.5]),t+.3)
    assert out['visits']['summit']=='pending'
    assert out['command']['vx_m_s']==0.


def test_ascent_keeps_trial_permission_until_safe_descent_handoff():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    from experiments.handheld_mapping.mission_replay import demo_mission
    mission=demo_mission();mission.site.robot_radius=.05;mission.site.slope_deg[30:31,20:67]=30.
    assert mission.validate()['accepted']
    run=MissionRuntime(mission);t,out=advance_trial(run,mission,3.4)
    assert out['state']=='ascending'
    assert out['visits']['summit']=='pending'
    for offset,x in enumerate([3.45,3.5],1):out=run.update(sample(mission,t+.1*offset,[x,1.5]),t+.1*offset)
    assert out['state']=='descending'
    assert out['visits']['summit']=='reached'
    out=run.update(sample(mission,t+.3,[3.5,1.5]),t+.3)
    assert out['reason']=='active'


def test_observed_pose_within_arrival_tolerance_of_safe_boundary_is_not_unsafe():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();site=mission.site;site.robot_radius=max(site.robot_radius,.15)
    limit=mission.settings['normal_slope_deg']
    strict=site.safe_mask(limit);loose=site.safe_mask(limit,mission.settings['arrival_tolerance_m'])
    edge=loose&~strict
    assert edge.any()
    row,col=np.argwhere(edge)[0];p=site.origin+(np.array([col,row])+.5)*site.resolution
    assert not site.validate_path([p,p],limit)['accepted']
    assert site.validate_path([p,p],limit,mission.settings['arrival_tolerance_m'])['accepted']
    run=MissionRuntime(mission);run.update(sample(mission,0.),0.)
    run.last_position=p.copy()
    out=run.update(sample(mission,.1,p),.1)
    assert out['reason'] not in ('unsafe_observed_motion','unsafe_local_segment')


def test_descent_finishes_at_start_without_revisiting_junction():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();mission.start[:2]=mission.routes['10'][1]
    descent=mission.descent_path()
    assert np.allclose(descent[-1],mission.start[:2])
    assert not any(np.allclose(p,mission.junction) for p in descent)
    run=MissionRuntime(mission);run.state='ascending';run.route='15'
    run._arrived(mission.summit,1.)
    assert run.state=='descending' and np.allclose(run.points[-1],mission.start[:2])
    out=run._arrived(mission.start[:2],2.)
    assert out['state']=='complete'
    assert out['visits']['return']=='reached'
    assert out['command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.}


def test_descent_preserves_junction_return_when_start_is_not_on_route():
    mission=mission_fixture()
    assert np.array_equal(mission.descent_path(),mission.path('10','down'))


def test_follower_does_not_weave_on_grid_quantised_path_with_lagged_pose():
    """2 cm 지그재그 경로와 0.4 s 늦은 pose·1차 지연 조향에서도 경로에서 4 cm 넘게 벗어나지 않고 끝점에 닿는다."""
    import math
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission)
    xs=np.arange(.5,3.0,.025);run._set_path(np.c_[xs,.5+np.where(np.arange(len(xs))%2==0,0.,.02)])
    dt,x,yaw,w,hist,commands,trace=.05,np.array(mission.start[:2],float),0.,0.,[],[],[]
    for k in range(1600):
        t=k*dt;hist.append((x.copy(),yaw));px,pyaw=hist[max(0,len(hist)-9)]
        out=run.update({'stamp_s':t,'frame':'site','site_identity':mission.site.identity,
                        'position_m':px.tolist(),'yaw_rad':pyaw,'tracking_valid':True},t)
        if out['state']!='to_junction':break
        c=out['command'];w+=(c['yaw_rate_rad_s']-w)*dt/(.5+dt);yaw+=w*dt
        x=x+c['vx_m_s']*dt*np.array([math.cos(yaw),math.sin(yaw)]);commands.append(c['yaw_rate_rad_s']);trace.append((t,x[1]))
    commands=np.array(commands)
    assert out['state']!='to_junction',out['reason']
    assert max(abs(y-.51) for _,y in trace)<=.04 and trace[-1][0]<=40.


def test_slip_slowdown_does_not_count_as_stuck_ascent():
    """감속(×0.3)으로 진행이 느려져도 stuck_window를 감속 배율만큼 늘려 센다."""
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    from experiments.handheld_mapping.sensor_safety import SensorSafety
    mission=mission_fixture();safety=SensorSafety();run=MissionRuntime(mission,safety=safety)
    window=mission.settings['stuck_window_s']
    for k in range(int(window*1.6/.1)):
        t=k*.1;safety.observe_slip(.5,t)
        out=run.update(sample(mission,t,tracking_quality='good',yaw_rad=np.pi/2),t)
    assert run.state=='to_junction' and out['sensor_safety']['scale']==.3


def test_stuck_is_detected_even_when_slowdown_pushes_command_below_ten_mm_per_s():
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    from experiments.handheld_mapping.sensor_safety import SensorSafety
    mission=mission_fixture();mission.settings['max_speed_m_s']=.03   # ×0.3 → 0.009 m/s
    safety=SensorSafety();run=MissionRuntime(mission,safety=safety)
    for k in range(int(40/.1)):
        t=k*.1;safety.observe_slip(.5,t)
        out=run.update(sample(mission,t,yaw_rad=np.pi/2),t)
        if run.state=='failed':break
    assert run.state=='failed' and 'no_progress' in out['reason']


def test_steering_segment_to_the_carrot_is_safety_checked(monkeypatch):
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission);second=MissionRuntime(mission);real=mission.site.validate_path;calls=[]
    def spy(points,limit,slack=0.):
        calls.append(np.linalg.norm(np.asarray(points[-1])-np.asarray(points[0])));return real(points,limit,slack)
    monkeypatch.setattr(mission.site,'validate_path',spy)
    run.update(sample(mission,0.),0.);run.update(sample(mission,.1),.1)
    assert max(calls)>=.15      # 바로 앞 점(≈5 cm)이 아니라 20 cm 앞 당근까지 검사
    monkeypatch.setattr(mission.site,'validate_path',lambda points,limit,slack=0.:{'accepted':np.linalg.norm(np.asarray(points[-1])-np.asarray(points[0]))<.12})
    second.update(sample(mission,0.),0.)
    assert second.update(sample(mission,.1),.1)['reason']=='unsafe_local_segment'


def test_stanley_gain_is_a_validated_setting_and_steers_back_toward_the_path() -> None:
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    from experiments.handheld_mapping.site_mission import DEFAULT_SETTINGS
    assert DEFAULT_SETTINGS['stanley_gain']==0.        # 기본은 꺼 둔다(기존 동작 유지)
    results={}
    for gain in (0.,3.):
        mission=mission_fixture();mission.settings['stanley_gain']=gain;run=MissionRuntime(mission)
        out=run.update(sample(mission,0.,xy=mission.start[:2]+np.array([.06,0.]),yaw_rad=np.pi/2),0.)    # 경로(x=.5)에서 오른쪽 6 cm, 정면을 향함
        results[gain]=out['command']['yaw_rate_rad_s']
    assert results[3.]>results[0.]-1e-9 and results[3.]!=results[0.]   # 횡오차 항이 명령을 바꾼다(왼쪽으로 더 틀어 경로로 복귀)
