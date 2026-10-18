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


class FakeEstimator:
    """GT와 다른 위치를 돌려주는 추정기 대역."""
    def __init__(self, valid=True, quality='good', status='ok'):
        self.valid,self.quality,self.status=valid,quality,status
    def pose(self, now):
        pose=np.eye(4);pose[:3,3]=[.4,.6,.0]
        return pose,self.valid,self.quality,self.status


def test_mission_decisions_use_the_estimate_not_gt_and_log_both(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    robot,session=Robot(),Session();drive=SimulationDrive(robot,session,mission,estimator=FakeEstimator())
    row=drive.step(.05)
    assert row['position_m']==pytest.approx([.5,.5,0.])               # 기록은 GT
    assert row['estimated_position_m']==pytest.approx([.4,.6,0.])      # 판단은 추정
    assert row['pose_source']=='orb_eskf_estimate' and row['estimator_status']=='ok'


def test_invalid_estimate_stops_the_rover(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    robot,session=Robot(),Session();drive=SimulationDrive(robot,session,mission,estimator=FakeEstimator(valid=False))
    row=drive.step(.05)
    assert row['applied_command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.} and row['mission']['reason']=='tracking_lost'


def test_estimated_yaw_is_recorded_separately_from_ground_truth(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    robot=Robot();robot.quaternion=np.array([np.cos(.25),0.,0.,np.sin(.25)])
    row=SimulationDrive(robot,Session(),mission,estimator=FakeEstimator()).step(.05)
    assert row['estimated_yaw_rad']==pytest.approx(0.)
    assert row['quaternion_wxyz']==pytest.approx(robot.quaternion)


def test_filter_status_reaches_sensor_safety_even_when_pose_is_valid(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    estimator=FakeEstimator();drive=SimulationDrive(Robot(),Session(),mission,estimator=estimator)
    for t in (.05,.3,.6):drive.step(t)
    estimator.status='recovering'
    recovering=drive.step(.65)
    assert recovering['mission']['sensor_safety']['scale']==pytest.approx(.3)
    estimator.status='imu_gap'
    gap=drive.step(.7)
    assert gap['mission']['sensor_safety']['mode']=='stop'
    assert gap['applied_command']=={'vx_m_s':0.,'yaw_rate_rad_s':0.}


def test_wheel_slip_scales_only_the_chosen_side_inside_its_time_window(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive, SlipInjection
    base=SimulationDrive(Robot(),Session(),mission,hold_s=0.);base.step(.05)
    reference=base.session.wheels.copy()
    drive=SimulationDrive(Robot(),Session(),mission,slip=SlipInjection(left=.5,right=1.,start_s=1.,end_s=2.))
    before=drive.step(.05);assert before['wheel_gain']==[1.,1.,1.,1.]
    inside=drive.step(1.5)
    assert inside['wheel_gain']==[.5,1.,.5,1.]                       # y>0 쪽(왼쪽) 모듈만
    after=drive.step(2.5);assert after['wheel_gain']==[1.,1.,1.,1.]
    # 같은 지령에서 왼쪽 바퀴 목표가 절반이 된다
    fresh=SimulationDrive(Robot(),Session(),mission,slip=SlipInjection(left=.5,right=1.,start_s=0.,end_s=9.));fresh.step(.05)
    assert fresh.session.wheels[[0,2]]==pytest.approx(reference[[0,2]]*.5) and fresh.session.wheels[[1,3]]==pytest.approx(reference[[1,3]])


def test_no_slip_injection_keeps_unit_gains_in_the_log(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    row=SimulationDrive(Robot(),Session(),mission).step(.05)
    assert row['wheel_gain']==[1.,1.,1.,1.]


def test_invalid_slip_gains_are_rejected() -> None:
    from experiments.site_sim.drive import SlipInjection
    for bad in (-.1,float('nan'),2.5):
        with pytest.raises(ValueError):SlipInjection(left=bad)


class RecordingEstimator(FakeEstimator):
    """on_motion 호출을 기록하는 추정기 대역."""
    def __init__(self):
        super().__init__();self.motions=[]
    def on_motion(self, stamp_s, vx, vy):
        self.motions.append((stamp_s,vx,vy))


class MovingRobot(Robot):
    """조향·바퀴 조인트 측정값을 지정한 body 속도에 맞춰 돌려주는 로봇(조인트 8개: 조향 4, 바퀴 4)."""
    def __init__(self, twist, geometry):
        super().__init__()
        from experiments.orbslam3_ros2.swerve import body_twist_to_modules
        commands=body_twist_to_modules(geometry,vx=twist[0],vy=twist[1],yaw_rate=twist[2])
        self.q=np.array([c.steering_angle_rad for c in commands]+[0.]*4)
        self.dq=np.array([0.]*4+[c.wheel_angular_velocity_rad_s for c in commands])
    def get_joint_positions(self):return self.q
    def get_joint_velocities(self):return self.dq


def test_measured_joints_reach_the_estimator_as_a_body_velocity(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    session=Session();estimator=RecordingEstimator()
    drive=SimulationDrive(MovingRobot((.06,.05,0.),session.geometry),session,mission,estimator=estimator)
    drive.step(.05)
    assert len(estimator.motions)==1
    stamp,vx,vy=estimator.motions[0]
    assert stamp==pytest.approx(.05) and (vx,vy)==pytest.approx((.06,.05),abs=1e-6)


def test_estimators_without_on_motion_are_left_alone(mission) -> None:
    from experiments.site_sim.drive import SimulationDrive
    drive=SimulationDrive(Robot(),Session(),mission,estimator=FakeEstimator())   # on_motion이 없어도 기존처럼 동작해야 한다
    assert drive.step(.05)['estimator_status']=='ok'
