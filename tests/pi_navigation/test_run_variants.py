# ================================================================
# test_run_variants.py
# 설명: 실제 배포 런타임의 RUN3/4 외곽 상승·30도 후퇴·안전 중단을 모터 없이 검증한다.
# 사용법: python -m pytest tests/pi_navigation/test_run_variants.py -q
# ================================================================
import numpy as np
import pytest
from test_runner import make_runner,feed


class Planner:
    """전이 시험에서 속도 명령만 반환하고 실제 경로 계산은 별도 시험에 맡긴다."""
    def plan(self,*args,**kwargs):
        return dict(valid=True,command=dict(vx_m_s=.1,yaw_rate_rad_s=0.),selected_trajectory=[])


def sample(runtime,xy,stamp=1.,**kw):
    runtime.last_position=np.array(xy)
    return dict(stamp_s=stamp,frame='site',site_identity=runtime.mission.site.identity,
                position_m=[*xy,0.],yaw_rad=0.,tracking_valid=True,**kw)


@pytest.mark.parametrize('run,stage,route',[(1,'entry30','30'),(2,'entry15','15'),(3,'entry30','30'),(4,'outer6','10')])
def test_four_run_initial_policies(run,stage,route):
    nav=make_runner();feed(nav,3.5)
    assert nav.start(run,3.5)
    assert nav.runtime.stage==stage and nav.runtime.route==route


def test_run3_retreat_uses_outer_but_run1_keeps_15():
    nav=make_runner()
    for run,next_stage in [(1,'entry15'),(3,'outer6')]:
        runtime=nav.new_runtime(run);runtime.safety=None;runtime.planner=Planner()
        runtime._phase('ascent30','ascending',0.)
        runtime._fail_ascent(.1,'trial_budget')
        assert runtime.stage=='retreat30'
        assert runtime.route_status['30']['up']=='aborted'
        point=runtime._goal()[0]
        runtime.update(sample(runtime,point),now_s=1.)
        assert runtime.stage==next_stage
        assert runtime.visits['entry15']!='active' if run==3 else runtime.visits['entry15']=='active'


def test_outer_ascent_uses_connected_regions_and_reaches_summit():
    runtime=make_runner().new_runtime(4);runtime.safety=None;runtime.planner=Planner()
    stages=['outer6','outer7','outer8','outer9','outer10','outer4']
    edges=[(1,6),(6,7),(7,8),(8,9),(9,10),(10,4)]
    for index,(stage,edge) in enumerate(zip(stages,edges)):
        assert runtime.stage==stage
        target,regions,portal,reverse=runtime._goal()
        assert tuple(portal)==edge and not reverse
        assert set(regions)==set(edge)
        assert runtime.graph.distance(edge[1],target)<=1e-6
        runtime.update(sample(runtime,target,stamp=1+index*.1),now_s=1+index*.1)
    assert runtime.stage=='summit' and runtime.route_status['10']['up']=='succeeded'
    assert runtime._goal()[1]==[4]


def test_outer_ascent_tilt_stops_instead_of_retreating_on_wrong_ramp():
    runtime=make_runner().new_runtime(4);runtime.safety=None;runtime.planner=Planner()
    point=runtime.mission.start[:2]
    out=runtime.update(sample(runtime,point,roll_rad=np.deg2rad(35)),now_s=1.)
    assert runtime.state=='failed'
    assert out['command']['vx_m_s']==0.
    assert out['reason']=='outer_ascent_tilt_limit'


@pytest.mark.parametrize('run',[3,4])
def test_deployed_gateway_accepts_new_runs_with_existing_readiness(run):
    from ksrc_runtime.handheld_mapping.control_gateway import Gateway,START
    started=[];gateway=Gateway(lambda *args:None,on_start=started.append)
    gateway.runner_status(dict(stationary_s=3.,orb_ok=True,in_region1=True),1.)
    assert gateway.control(START,(float(run),0.,0.),1.01)
    assert started==[run] and gateway.mode=='AUTO'


@pytest.mark.parametrize('run',[0.,5.,2.5])
def test_deployed_gateway_rejects_invalid_run_without_entering_auto(run):
    from ksrc_runtime.handheld_mapping.control_gateway import Gateway,START
    gateway=Gateway(lambda *args:None)
    gateway.runner_status(dict(stationary_s=3.,orb_ok=True,in_region1=True),1.)
    assert not gateway.control(START,(run,0.,0.),1.01)
    assert gateway.mode=='TELEOP' and gateway.last_reject=='run'


@pytest.mark.parametrize('run,route,stage',[(1,'10','return10'),(2,'10','return10'),
                                          (3,'15','descent15_entry'),(4,'15','descent15_entry')])
def test_descent_policy_preserves_run1_2_and_uses_15_for_run3_4(run,route,stage):
    runtime=make_runner().new_runtime(run)
    runtime._begin_descent(10.)
    assert runtime.route==route and runtime.direction=='down'
    assert runtime.stage==stage


@pytest.mark.parametrize('run',[3,4])
def test_15_descent_reaches_region1_and_marks_correct_route(run):
    runtime=make_runner().new_runtime(run);runtime.safety=None;runtime.planner=Planner()
    runtime._begin_descent(0.)
    for index,stage in enumerate(('descent15_entry','descent15','return15')):
        assert runtime.stage==stage
        target,regions,edge,reverse=runtime._goal()
        assert not reverse
        assert 10 not in regions and 6 not in regions
        assert runtime.graph.distance(4 if index==0 else 1,target)<=1e-6
        runtime.update(sample(runtime,target,stamp=1+index*.1),now_s=1+index*.1)
    assert runtime.state=='complete'
    assert runtime.route_status['15']['down']=='succeeded'
    assert runtime.route_status['10']['down']=='unknown'
