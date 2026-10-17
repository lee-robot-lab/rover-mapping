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


@pytest.mark.parametrize('twist',[(.08,0.,0.),(-.08,0.,0.),(0.,0.,.4),(.05,.02,-.3)])
def test_new_urdf_modules_roll_in_the_commanded_body_direction(twist):
    from xml.etree import ElementTree as ET
    from experiments.orbslam3_ros2.swerve import body_twist_to_modules
    from experiments.site_sim.rover_assets import SOURCE_URDF,swerve_geometry
    data=swerve_geometry(ET.parse(SOURCE_URDF).getroot(),.0475)
    geometry=SwerveGeometry(tuple(ModuleGeometry(m['name'],m['steering_joint'],m['wheel_joint'],
        *m['steering_pivot_base_link_m'][:2],m['steering_sign'],m['rolling_heading_at_zero_rad']) for m in data['modules']),.0475)
    vx,vy,wz=twist
    for module,command in zip(geometry.modules,body_twist_to_modules(geometry,vx=vx,vy=vy,yaw_rate=wz)):
        heading=module.rolling_heading_rad+module.steering_sign*command.steering_angle_rad
        speed=command.wheel_angular_velocity_rad_s*.0475
        got=speed*np.array([np.cos(heading),np.sin(heading)])
        want=np.array([vx-wz*module.y_m,vy+wz*module.x_m])
        assert np.allclose(got,want,atol=1e-9)
        assert abs(command.steering_angle_rad)<=np.pi/2+1e-9


def test_start_hold_zeroes_applied_command_but_keeps_the_request(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    robot,session=Robot(),Session();drive=SimulationDrive(robot,session,mission,hold_s=1.)
    row=drive.step(.05)
    assert row['applied_command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.}
    assert np.allclose(session.wheels,0.)
    later=SimulationDrive(Robot(),Session(),mission,hold_s=0.).step(.05)
    assert later['applied_command']==later['requested_command']


@pytest.mark.parametrize('twist',[(.08,0.,0.),(-.08,0.,0.),(0.,0.,.4),(.05,.02,-.3),(.1,0.,.5)])
def test_wheel_encoders_recover_the_commanded_body_twist(twist) -> None:
    from xml.etree import ElementTree as ET
    from experiments.orbslam3_ros2.swerve import body_twist_from_modules,body_twist_to_modules
    from experiments.site_sim.rover_assets import SOURCE_URDF,swerve_geometry
    data=swerve_geometry(ET.parse(SOURCE_URDF).getroot(),.0475)
    geometry=SwerveGeometry(tuple(ModuleGeometry(m['name'],m['steering_joint'],m['wheel_joint'],
        *m['steering_pivot_base_link_m'][:2],m['steering_sign'],m['rolling_heading_at_zero_rad']) for m in data['modules']),.0475)
    modules=body_twist_to_modules(geometry,vx=twist[0],vy=twist[1],yaw_rate=twist[2])
    vx,vy,wz,residual=body_twist_from_modules(geometry,[m.steering_angle_rad for m in modules],
                                              [m.wheel_angular_velocity_rad_s for m in modules])
    assert (vx,vy,wz)==pytest.approx(twist,abs=1e-9) and residual<1e-9
