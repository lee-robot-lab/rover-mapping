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
