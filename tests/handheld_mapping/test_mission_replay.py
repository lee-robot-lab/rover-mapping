# ================================================================
# test_mission_replay.py
# 설명: 실제 runtime+합성 운동학을 연결해 대체 경로·후퇴·복귀의 종단 동작을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_mission_replay.py -q
# ================================================================
import json
import numpy as np
import pytest
from site_mission_helpers import mission_fixture


@pytest.mark.parametrize('scenario,failed_routes',[
    ('success',[]),('stall30',['30']),('stall30_15',['30','15']),
])
def test_replay_reaches_shared_summit_and_returns_by10(tmp_path,scenario,failed_routes):
    from experiments.handheld_mapping.mission_replay import replay
    mission=mission_fixture();out=replay(mission,tmp_path/scenario,scenario)
    assert out['state']=='complete',out
    assert out['visits']['summit']=='reached'
    assert out['visits']['return']=='reached'
    assert out['failed_ascent_routes']==failed_routes
    assert out['descent_route']=='10'
    assert out['simulation_only'] and not out['hardware_commands_enabled']
    assert (tmp_path/scenario/'commands.csv').is_file()
    events=[json.loads(line) for line in (tmp_path/scenario/'events.jsonl').read_text().splitlines()]
    assert sum(e['type']=='summit_reached' for e in events)==1
    retreat=[e for e in events if e['type']=='retreat_started']
    assert len(retreat)==len(failed_routes)
    for event in retreat:
        assert np.linalg.norm(np.asarray(event['path'])[-1]-mission.junction)<.13
        assert max(p[0] for p in event['path'])<mission.summit[0]-.5


def test_all_ascent_failures_preserve_unreached_summit(tmp_path):
    from experiments.handheld_mapping.mission_replay import replay
    out=replay(mission_fixture(),tmp_path/'fail','stall_all')
    assert out['state']=='failed'
    assert out['visits']['summit']=='failed'
    assert out['failed_ascent_routes']==['30','15','10']


def test_sensor_loss_resume_does_not_become_climb_failure(tmp_path):
    from experiments.handheld_mapping.mission_replay import replay
    out=replay(mission_fixture(),tmp_path/'loss','tracking_loss')
    assert out['state']=='complete'
    assert out['failed_ascent_routes']==[]
    assert out['paused_samples']>0


def test_return_stall_fails_without_marking_return_complete(tmp_path):
    from experiments.handheld_mapping.mission_replay import replay
    out=replay(mission_fixture(),tmp_path/'return','return_stall')
    assert out['state']=='failed'
    assert out['visits']['summit']=='reached'
    assert out['visits']['return']=='failed'


def test_trial_slope_permission_is_retained_while_retracing(tmp_path):
    from experiments.handheld_mapping.mission_replay import replay
    mission=mission_fixture()
    mission.site.slope_deg[13:18,11:29]=30.
    out=replay(mission,tmp_path/'steep_retreat','stall30')
    assert out['state']=='complete',out
    assert out['failed_ascent_routes']==['30']
    assert out['paused_samples']==0


def test_unavailable_descent_does_not_claim_return_success(tmp_path):
    from experiments.handheld_mapping.mission_replay import replay
    out=replay(mission_fixture(),tmp_path/'blocked_return','descent_blocked')
    assert out['state']=='failed'
    assert out['visits']['summit']=='reached'
    assert out['visits']['return']=='failed'
    assert out['reason']=='descent_route_unavailable'


def test_stall_before_trial_representative_still_retreats_and_falls_back(tmp_path):
    from experiments.handheld_mapping.mission_replay import replay
    out=replay(mission_fixture(),tmp_path/'entry_stall','stall30_entry')
    assert out['state']=='complete',out
    assert out['visits']['entry30']=='failed'
    assert out['visits']['summit']=='reached'
    assert out['failed_ascent_routes']==['30']


def test_step_plant_integrates_vy_in_body_frame():
    from experiments.handheld_mapping.mission_replay import step_plant
    xy,yaw=step_plant(np.zeros(2),np.pi/2,{'vx_m_s':0.,'vy_m_s':.1,'yaw_rate_rad_s':0.},1.)
    assert np.allclose(xy,[-.1,0.]) and yaw==pytest.approx(np.pi/2)
    start=np.array([.3,.2]);cmd={'vx_m_s':.1,'yaw_rate_rad_s':.2}   # vy 키가 없으면 기존 unicycle 적분과 비트 단위로 같다
    got_xy,got_yaw=step_plant(start,.7,cmd,.1)
    assert got_xy.tobytes()==(start+cmd['vx_m_s']*.1*np.array([np.cos(.7),np.sin(.7)])).tobytes()
    assert got_yaw==np.arctan2(np.sin(.7+.02),np.cos(.7+.02))


def test_replay_with_blend_follower_completes_the_mission(tmp_path):
    from experiments.handheld_mapping.mission_replay import demo_mission,replay
    mission=demo_mission();mission.settings['follower_blend']=1.
    out=replay(mission,tmp_path/'blend','success')
    assert out['state']=='complete' and out['visits']['summit']=='reached',out
    header=(tmp_path/'blend'/'commands.csv').read_text().splitlines()[0].split(',')
    assert 'vy_m_s' in header


def reversal_mission(kind):
    """kind: 'line'은 10° 하강을 30° 등판과 같은 선으로, 'retrace15'는 10° 경로를 15° 경로와 같게 해 정상에서 정확히 180° 반전시킨다."""
    mission=mission_fixture();mission.routes['10']=np.array(mission.routes['30' if kind=='line' else '15'])
    if kind=='line':mission.routes['10']=np.array([[.5,1.5],[1.5,1.5],[3.5,1.5]])
    assert mission.validate()['accepted']
    return mission


@pytest.mark.parametrize('name,scenario',[('fixture','success'),('fixture','stall30'),('fixture','stall30_15'),
                                          ('demo','success'),('demo','stall30'),('demo','stall30_15'),
                                          ('line180','success'),('retrace15','stall30')])
def test_blend_follower_replays_complete_including_180_degree_transitions(tmp_path,name,scenario):
    """후퇴·정상 반전·복귀 방향 전환(각 약 180°)이 정지 판정 없이 정지-재정렬-출발로 끝까지 간다."""
    from experiments.handheld_mapping.mission_replay import demo_mission,replay
    mission={'fixture':mission_fixture,'demo':demo_mission,'line180':lambda:reversal_mission('line'),
             'retrace15':lambda:reversal_mission('retrace15')}[name]()
    mission.settings['follower_blend']=1.
    out=replay(mission,tmp_path/'run',scenario)
    assert out['state']=='complete' and out['visits']['return']=='reached',(name,scenario,out['reason'],out['duration_s'])
    if scenario!='success':assert out['failed_ascent_routes']
