# ================================================================
# test_drive.py
# 설명: 실제 임무 판단과 운동학을 fake articulation에 연결해 정지·정체 주입을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_drive.py
# ================================================================
from types import SimpleNamespace
import numpy as np
import pytest
from experiments.orbslam3_ros2.swerve import ModuleGeometry,SwerveGeometry


class Robot:
    """외부 physics에서 받은 pose와 실제 joint 측정값의 최소 fixture."""
    def __init__(self):
        self.position=np.array([.5,.5,0.]);self.quaternion=np.array([1.,0.,0.,0.])
    def get_world_pose(self):return self.position,self.quaternion
    def get_joint_positions(self):return np.zeros(8)
    def get_joint_velocities(self):return np.zeros(8)


class Session:
    """명령 경계에서 실제 적용된 joint target을 보존한다."""
    def __init__(self):
        self.geometry=SwerveGeometry(tuple(ModuleGeometry(str(i),f's{i}',f'w{i}',x,y)
            for i,(x,y) in enumerate(((.1,.13),(.1,-.13),(-.1,.13),(-.1,-.13)))))
        self.controller=self;self._action_type=SimpleNamespace
        self.steering_indices=np.arange(4);self.wheel_indices=np.arange(4,8)
        self.wheels=np.ones(4);self.steering=np.zeros(4);self.fail_next=False
    def apply_action(self,action):
        if self.fail_next:self.fail_next=False;raise RuntimeError('actuator failed')
        if hasattr(action,'joint_positions'):self.steering=action.joint_positions
        if hasattr(action,'joint_velocities'):self.wheels=action.joint_velocities


def test_real_runtime_commands_reach_wheel_joint_boundary(mission):
    from experiments.site_sim.drive import SimulationDrive
    robot,session=Robot(),Session();drive=SimulationDrive(robot,session,mission)
    row=drive.step(.05)
    assert row['pose_source']=='sim_ground_truth' and not row['fault_injection']
    assert row['applied_command']==row['requested_command']
    assert np.max(np.abs(session.wheels))>0
    assert abs(row['requested_command']['yaw_rate_rad_s'])<=.6
    drive.stop();assert np.array_equal(session.wheels,np.zeros(4))


def test_invalid_pose_or_actuation_exception_clears_old_targets(mission):
    from experiments.site_sim.drive import SimulationDrive
    robot,session=Robot(),Session();drive=SimulationDrive(robot,session,mission)
    drive.step(.05);robot.position[0]=np.nan
    row=drive.step(.1)
    assert row['mission']['reason']=='invalid_pose'
    assert np.array_equal(session.wheels,np.zeros(4))
    robot.position[0]=.5;session.fail_next=True
    with pytest.raises(RuntimeError,match='actuator'):drive.step(.15)
    assert np.array_equal(session.wheels,np.zeros(4))


def test_only_explicit_trial_fault_is_injected_and_retreat_is_unblocked(mission):
    from experiments.site_sim.drive import SimulationDrive
    robot,session=Robot(),Session();drive=SimulationDrive(robot,session,mission,fault_injection=True)
    drive.runtime.started=True;drive.runtime.state='ascending';drive.runtime.route='30'
    robot.position=np.array([1.3,1.5,0.]);drive.runtime._set_path(mission.routes['30'][1:])
    row=drive.step(1.)
    assert row['fault_active'] and np.array_equal(session.wheels,np.zeros(4))
    assert row['applied_command']!=row['requested_command']
    drive.runtime.state='retreating';drive.runtime._set_path(np.array([[1.3,1.5],[.5,1.5]]))
    row=drive.step(1.05)
    assert not row['fault_active']
    assert row['applied_command']==row['requested_command']
