# ================================================================
# test_relay.py
# 설명: 실제 pose 기반 속도 추정, stale 차단, ANCHOR 시 점프 제거를 검증한다.
# 사용법: python -m pytest tests/broadcast_deployment -q
# ================================================================
import importlib.util
from pathlib import Path
import pytest
path=Path(__file__).resolve().parents[2]/'deployment/broadcast/relay.py'
spec=importlib.util.spec_from_file_location('broadcast_relay',path)
relay=importlib.util.module_from_spec(spec);spec.loader.exec_module(relay)


def row(t,x=0.,anchor=0,**kw):
    return dict(_file_age_s=0.,_source_session='test',orb_ok=True,pose_age_s=.1,stamp_s=t,
                pose=[[1,0,0,x],[0,1,0,0],[0,0,1,0],[0,0,0,1]],anchor_count=anchor,**kw)


def test_estimated_velocity_and_stale():
    state=relay.Telemetry();state.packet(row(0))
    packet=state.packet(row(.2,.02,active=True,run=1))
    assert packet['v']==pytest.approx(.1)
    assert packet['speed_m_s']==pytest.approx(.1)
    assert packet['operator']['active']
    assert state.packet(dict(row(.4),_file_age_s=3)) is None


def test_anchor_is_not_a_speed_spike_and_duplicate_is_ignored():
    state=relay.Telemetry();state.packet(row(0))
    packet=state.packet(row(.2,3.6,anchor=1))
    assert packet['v']==0 and packet['w']==0
    assert state.packet(row(.2,3.6,anchor=1)) is None


def test_runner_restart_in_same_folder_resets_time_epoch():
    state=relay.Telemetry();first=state.packet(row(100))
    restarted=state.packet(row(1))
    assert restarted is not None
    assert restarted['session_id']!=first['session_id']
    assert restarted['v']==0.
    assert state.packet(row(1.2,.02))['v']==pytest.approx(.1)


def test_lateral_pose_speed_and_anchor_id_are_preserved():
    from experiments.broadcast_demo.core import decode_packet
    import json
    state=relay.Telemetry();state.packet(row(0))
    status=row(.2,anchor_id=4);status['pose'][1][3]=.02
    packet=decode_packet(json.dumps(state.packet(status)).encode())
    assert packet['v']==pytest.approx(0.)
    assert packet['vy_m_s']==pytest.approx(.1)
    assert packet['operator']['anchor_id']==4


def test_relay_transmits_planning_map_identity():
    from experiments.broadcast_demo.core import decode_packet
    import json
    state=relay.Telemetry()
    packet=state.packet(row(0,site_identity='a'*64))
    assert decode_packet(json.dumps(packet).encode())['site_identity']=='a'*64
