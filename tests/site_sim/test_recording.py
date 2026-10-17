# ================================================================
# test_recording.py
# 설명: costmap sentinel·주행 경험·기록 상한·실패 결과를 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_recording.py
# ================================================================
import json
import numpy as np
import pytest


def packet(t,state='ascending'):
    return {'stamp_s':t,'wall_s':t,'position_m':[1.,1.5,0.],'quaternion_wxyz':[1.,0.,0.,0.],
        'joint_positions':[0.]*8,'joint_velocities':[0.]*8,
        'requested_command':{'vx_m_s':.08,'yaw_rate_rad_s':0.},
        'applied_command':{'vx_m_s':.08,'yaw_rate_rad_s':0.},
        'mission':{'state':state,'route':'30','direction':'up','visits':{},'reason':'active'},
        'pose_source':'sim_ground_truth','fault_injection':False,'fault_active':False}


def test_costs_keep_unknown_walls_and_only_allow_trial_slope(mission):
    from experiments.site_sim.recording import geometry_costs
    s=mission.site;s.observed[10,10]=False;s.blocked[10,25]=True;s.slope_deg[5:10,15:21]=30.
    normal,trial=geometry_costs(mission)
    assert normal[10,10]==trial[10,10]==255
    assert normal[10,25]==trial[10,25]==254
    assert normal[7,18]==254 and trial[7,18]<254


def test_observed_experience_snapshots_and_event_deduplication(mission,tmp_path):
    from experiments.site_sim.recording import RunRecorder
    out=tmp_path/'run';r=RunRecorder(mission,out,max_snapshots=4)
    events=[{'type':'ascent_failed','stamp_s':0.,'state':'ascending','route':'30'}]
    for i in range(5):r.observe(packet(float(i)),events)
    summary=r.finalize('failed','test_no_progress')
    assert summary['status']=='failed' and summary['reason']=='test_no_progress'
    files=list((out/'snapshots').glob('*.npz'));assert len(files)<=4
    final=np.load(out/'snapshots/final.npz');visits=final['visit_count'];stuck=final['stuck_count']
    assert visits.sum()==5 and stuck.sum()==1
    assert np.isnan(final['last_seen_s'][0,0])
    assert len((out/'events.jsonl').read_text().splitlines())==1
    assert len((out/'telemetry.jsonl').read_text().splitlines())==5
    assert json.loads((out/'completion.json').read_text())['status']=='failed'
    assert (out/'final.png').is_file()


def test_recording_rejects_nonmonotonic_or_nonfinite_pose(mission,tmp_path):
    from experiments.site_sim.recording import RunRecorder
    r=RunRecorder(mission,tmp_path/'run');r.observe(packet(1.),[])
    with pytest.raises(ValueError):r.observe(packet(1.),[])
    row=packet(2.);row['position_m'][0]=float('nan')
    with pytest.raises(ValueError):r.observe(row,[])
    r.finalize('failed','invalid_input')


def test_live_experience_exposes_only_observed_cells(mission,tmp_path):
    from experiments.site_sim.recording import RunRecorder
    r=RunRecorder(mission,tmp_path/'run');r.observe(packet(0.),[]);r.finalize('timeout','limit')
    live=json.loads((tmp_path/'run/live/state.json').read_text())
    assert live['experience']==[[10,15,1,0]]


def test_estimator_run_does_not_report_gt_as_the_control_pose_source(mission,tmp_path) -> None:
    from experiments.site_sim.recording import RunRecorder
    out=tmp_path/'run';r=RunRecorder(mission,out,pose_source='orb_eskf_estimate')
    row=packet(0.);row['pose_source']='orb_eskf_estimate'
    r.observe(row,[]);summary=r.finalize('timeout','limit')
    assert summary['pose_source']=='orb_eskf_estimate'
    assert json.loads((out/'manifest.json').read_text())['pose_source']=='orb_eskf_estimate'
    assert json.loads((out/'completion.json').read_text())['pose_source']=='orb_eskf_estimate'
