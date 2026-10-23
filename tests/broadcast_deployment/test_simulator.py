# ================================================================
# test_simulator.py
# 설명: 조이스틱 명령·START·E-STOP을 실제 gateway와 기구학 로버에 연결해 확인한다.
# 사용법: python -m pytest tests/broadcast_deployment/test_simulator.py -q
# ================================================================
import importlib.util
from pathlib import Path
import numpy as np
import pytest
path=Path(__file__).resolve().parents[2]/'deployment/broadcast/simulate.py'


def load_sim():
    spec=importlib.util.spec_from_file_location('broadcast_simulate',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def test_joystick_motion_and_release_stop():
    sim=load_sim().KinematicRover()
    start=sim.pose.copy()
    for _ in range(10):
        sim.gateway.teleop((.1,0.,0.),sim.time)
        sim.step(.1)
    assert np.linalg.norm(sim.pose[:2]-start[:2])==pytest.approx(.1)
    for _ in range(5):sim.step(.1)
    stopped=sim.pose.copy()
    for _ in range(5):sim.step(.1)
    assert np.allclose(sim.pose,stopped)


@pytest.mark.parametrize('run,stage',[(1,'entry30'),(2,'entry15'),(3,'entry30'),(4,'outer6')])
def test_start_and_estop_follow_real_gateway(run,stage):
    module=load_sim();sim=module.KinematicRover()
    sim.step(3.1)
    assert sim.gateway.control(module.START,(run,0.,0.),sim.time)
    assert sim.runtime.stage==stage
    sim.step(.1)
    assert sim.packet(1)['operator']['run']==run
    sim.gateway.control(module.ESTOP,(0.,0.,0.),sim.time)
    assert sim.gateway.mode=='TELEOP'
    assert not sim.packet(2)['operator']['active']
    assert sim.command==(0.,0.,0.)


def test_anchor2_requires_actual_sim_heading_and_updates_start_check():
    module=load_sim();sim=module.KinematicRover();sim.step(3.1)
    sim.anchor(preset=2)
    assert sim.anchor_id==1  # truth pose is still -y, do not teleport the heading
    sim.pose[2]=-np.pi
    sim.anchor(preset=2)
    sim.step(.02)
    assert sim.anchor_id==2 and sim.status()['in_region1']
    assert sim.gateway.control(module.START,(4,0.,0.),sim.time)
    assert sim.runtime.mission.start[2]==pytest.approx(-np.pi)
