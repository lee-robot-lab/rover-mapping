# ================================================================
# test_sensor_safety.py
# 설명: pose/IMU 끊김과 추적 저하의 감속·정지·안정 복구를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_sensor_safety.py
# ================================================================
import pytest
from experiments.handheld_mapping.sensor_safety import SensorSafety


def observe(s,t,quality='good',valid=True):
    s.observe_pose(t,t,valid,quality)
    if s.require_imu:s.observe_imu(t,t)
    return s.status(t)


def ready(s):
    for t in (0.,.1,.2,.3,.4,.5):observe(s,t)
    assert s.status(.5)['mode']=='normal'


def test_degraded_slows_and_loss_stops_then_requires_stable_recovery():
    s=SensorSafety();ready(s)
    assert observe(s,.6,'degraded')['scale']==.3
    assert observe(s,.7,valid=False)['mode']=='stop'
    assert observe(s,.8)['reason']=='recovering'
    assert observe(s,.9)['mode']=='stop'
    for t in (1.,1.1,1.2,1.3):observe(s,t)
    assert s.status(1.3)['mode']=='normal'


def test_watchdog_stops_without_any_new_sensor_callback():
    s=SensorSafety(require_imu=True);ready(s)
    assert s.status(.61)['reason']=='imu_stale'
    s.observe_imu(.62,.62)
    assert s.status(.62)['mode']=='stop'
    assert s.status(1.1)['reason']=='pose_stale'


def test_imu_required_is_explicit_not_assumed_in_visual_only_mode():
    s=SensorSafety();ready(s);assert s.status(.55)['mode']=='normal'
    s=SensorSafety(require_imu=True)
    s.observe_pose(0.,0.,True,'good');assert s.status(0.)['reason']=='imu_missing'


@pytest.mark.parametrize('bad',[float('nan'),-1.,1.])
def test_invalid_or_future_imu_stops(bad):
    s=SensorSafety(require_imu=True);ready(s);s.observe_imu(bad,.55)
    assert s.status(.55)['mode']=='stop'


def test_replayed_pose_cannot_satisfy_recovery():
    s=SensorSafety();s.observe_pose(0.,0.,True,'good')
    for _ in range(10):s.observe_pose(0.,.1,True,'good')
    assert s.status(.1)['mode']=='stop'


def test_runtime_applies_speed_limit_and_watchdog_zeroes_both_axes():
    from site_mission_helpers import mission_fixture
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    mission=mission_fixture();run=MissionRuntime(mission,safety=SensorSafety())
    def packet(t,**extra):
        return {'stamp_s':t,'frame':'site','site_identity':mission.site.identity,
                'position_m':mission.start[:2].tolist(),'yaw_rad':1.5707963267948966,
                'tracking_valid':True,**extra}
    for t in (0.,.1,.2,.3,.4,.5):out=run.update(packet(t),t)
    normal=out['command']['vx_m_s'];assert normal>0
    slow=run.update(packet(.6,tracking_quality='degraded'),.6)
    assert slow['command']['vx_m_s']==pytest.approx(normal*.3)
    stopped=run.watchdog(1.11)
    assert stopped['reason']=='pose_stale'
    assert stopped['command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.}
    assert run.failed_routes==[]


def test_runtime_requires_fresh_imu_even_with_fresh_visual_pose():
    from site_mission_helpers import mission_fixture
    from experiments.handheld_mapping.mission_runtime import MissionRuntime
    m=mission_fixture();run=MissionRuntime(m,safety=SensorSafety(require_imu=True))
    def p(t):return {'stamp_s':t,'frame':'site','site_identity':m.site.identity,
        'position_m':m.start[:2].tolist(),'yaw_rad':0.,'tracking_valid':True}
    for t in (0.,.1,.2,.3,.4,.5):
        run.observe_imu(t,t);out=run.update(p(t),t)
    assert out['sensor_safety']['mode']=='normal'
    out=run.update(p(.65),.65)
    assert out['reason']=='imu_stale' and out['command']['yaw_rate_rad_s']==0
    assert run.failed_routes==[]


def test_jsonl_imu_events_and_silent_input_tick_emit_stop(tmp_path):
    import json
    from site_mission_helpers import mission_fixture
    from experiments.handheld_mapping.mission_runtime import run_jsonl
    m=mission_fixture();packets=[]
    for t in (0.,.1,.2,.3,.4,.5):
        packets.append({'sensor_event':'imu','imu_stamp_s':t,'now_s':t})
        packets.append({'stamp_s':t,'now_s':t,'frame':'site','site_identity':m.site.identity,
                        'position_m':m.start[:2].tolist(),'yaw_rad':0.,'tracking_valid':True})
    packets.append({'now_s':.65,'watchdog':True})
    source=tmp_path/'input.jsonl';source.write_text(''.join(json.dumps(p)+'\n' for p in packets))
    out=tmp_path/'out';run_jsonl(m,source,out,safety=SensorSafety(require_imu=True))
    commands=[json.loads(l) for l in (out/'commands.jsonl').read_text().splitlines()]
    assert commands[-2]['reason']=='imu_stale'
    assert commands[-2]['command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.}
    assert commands[-1]['reason']=='input_end_stop'


def test_recovery_needs_new_poses_not_just_elapsed_wall_time():
    s=SensorSafety();observe(s,0.)
    assert s.status(.5)['reason']=='recovering'
    assert s.status(.6)['reason']=='pose_stale'


def test_optional_imu_does_not_shorten_motion_deadline():
    s=SensorSafety();ready(s)
    assert s.status(.5)['valid_until_s']==pytest.approx(1.)
    s=SensorSafety(require_imu=True);ready(s)
    assert s.status(.5)['valid_until_s']==pytest.approx(.6)


def test_new_imu_after_gap_does_not_reenable_motion_immediately():
    s=SensorSafety(require_imu=True);ready(s)
    s.observe_imu(.7,.7)
    s.observe_pose(.7,.7,True,'good')
    assert s.status(.7)['reason']=='recovering'


def test_optional_imu_fault_does_not_block_visual_only_mode():
    s=SensorSafety();ready(s);s.observe_imu(float('nan'),.55)
    assert s.status(.55)['mode']=='normal'


def test_filter_imu_gap_stops_and_recovering_slows():
    s=SensorSafety();ready(s)
    s.observe_filter('imu_gap',.6);out=observe(s,.6)
    assert out['mode']=='stop' and out['reason']=='filter_imu_gap'
    for t in (.7,.8,.9,1.,1.1,1.2):s.observe_filter('recovering',t);out=observe(s,t)
    assert out['mode']=='slow' and out['reason']=='filter_recovering' and out['scale']==.3
    s.observe_filter('ok',1.3);assert observe(s,1.3)['mode']=='normal'


@pytest.mark.parametrize('ratio,scale',[(1.,1.),(.85,1.),(.79,.5),(.59,.3)])
def test_slip_ratio_scales_speed(ratio,scale):
    s=SensorSafety();ready(s);s.observe_slip(ratio,.6)
    out=observe(s,.6);assert out['scale']==scale
    assert out['reason']==('sensors_ok' if scale==1. else 'slip')


def test_slip_release_needs_margin_and_slowest_limit_wins():
    s=SensorSafety();ready(s);s.observe_slip(.79,.6);assert observe(s,.6)['scale']==.5
    s.observe_slip(.82,.7);assert observe(s,.7)['scale']==.5     # 0.8 바로 위는 아직 해제하지 않는다
    s.observe_slip(.86,.8);assert observe(s,.8)['scale']==1.
    s.observe_slip(.79,.9);assert observe(s,.9,'degraded')['scale']==.3   # 추정 품질 감속과는 곱하지 않고 작은 값


@pytest.mark.parametrize('bad',[float('nan'),None,'x'])
def test_unreadable_slip_ratio_takes_the_slowest_limit(bad):
    s=SensorSafety();ready(s);s.observe_slip(bad,.6);assert observe(s,.6)['scale']==.3


def test_stale_filter_status_stops_and_stale_slip_slows_but_absent_inputs_are_ignored():
    s=SensorSafety();ready(s);assert s.status(.5)['reason']=='sensors_ok'   # 한 번도 안 들어온 입력은 선택 사항
    s.observe_filter('ok',.5);s.observe_slip(1.,.5)
    for t in (.6,.7,.8,.9,1.,1.1):s.observe_filter('ok',t);out=observe(s,t)
    assert out['mode']=='slow' and out['reason']=='slip_stale'     # 슬립 공급이 0.5 s 넘게 끊김
    for t in (1.2,1.3,1.4,1.5,1.6,1.7):out=observe(s,t)
    assert out['mode']=='stop' and out['reason']=='filter_stale'   # 필터 상태가 끊기면 정지
