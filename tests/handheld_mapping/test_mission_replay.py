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
