# ================================================================
# drive.py
# 설명: simulator 정답 pose를 임무 판단과 swerve joint target에 연결한다.
# 사용법: from experiments.site_sim.drive import SimulationDrive
# ================================================================
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any
import numpy as np
from scipy.spatial.transform import Rotation
from experiments.handheld_mapping.mission_runtime import MissionRuntime
from experiments.handheld_mapping.sensor_safety import SensorSafety
from experiments.handheld_mapping.site_mission import SiteMission
from experiments.orbslam3_ros2.drive_runtime import yaw_from_wxyz
from experiments.orbslam3_ros2.swerve import body_twist_to_modules


@dataclass(frozen=True)
class SlipInjection:
    """지정한 시간 구간에 왼쪽(y>0)·오른쪽 바퀴 속도 목표에 배율을 곱해 슬립이나 좌우 속도 차를 흉내낸다."""
    left: float = 1.
    right: float = 1.
    start_s: float = 0.
    end_s: float = math.inf

    def __post_init__(self) -> None:
        for gain in (self.left,self.right):
            if not math.isfinite(gain) or not 0<=gain<=2:raise ValueError('wheel gain must be in [0, 2]')

    def gains(self, geometry: Any, sim_s: float) -> np.ndarray:
        """geometry 모듈 순서대로의 배율(구간 밖이면 모두 1)."""
        if not self.start_s<=sim_s<self.end_s:return np.ones(len(geometry.modules))
        return np.array([self.left if module.y_m>0 else self.right for module in geometry.modules],float)


class SimulationDrive:
    """물리 pose feedback만으로 임무를 진행하고 실제 적용 명령을 따로 보존한다."""

    def __init__(self, robot: Any, session: Any, mission: SiteMission, *, fault_injection: bool=False, hold_s: float=0.,
                 estimator: Any=None, slip: SlipInjection | None=None) -> None:
        self.slip=slip;self.robot,self.session=robot,session;self.hold_s=float(hold_s);self.estimator=estimator
        self.runtime=MissionRuntime(mission,safety=SensorSafety() if estimator is not None else None)
        self.fault_injection=fault_injection
        self.fault_started=False;self.wall_start=time.monotonic()

    def stop(self) -> None:
        """네 wheel velocity target을 즉시 0으로 만든다."""
        s=self.session
        s.controller.apply_action(s._action_type(joint_velocities=np.zeros(4),joint_indices=s.wheel_indices))

    def step(self, sim_s: float) -> dict:
        """한 simulation timestamp의 판단과 joint 적용 결과를 반환한다."""
        try:
            position,quaternion=(np.asarray(a,float) for a in self.robot.get_world_pose())
            sample={'stamp_s':sim_s,'frame':'site','site_identity':self.runtime.mission.site.identity,
                    'pose_epoch':0,'position_m':position.tolist(),'yaw_rad':yaw_from_wxyz(quaternion),
                    'tracking_valid':True}
            estimate=None
            if self.estimator is not None:
                # 임무 판단은 추정 위치만 본다. GT(position, quaternion)는 기록과 비교용이다.
                pose,valid,quality,status=self.estimator.pose(sim_s)
                q=Rotation.from_matrix(pose[:3,:3]).as_quat()
                sample.update(position_m=pose[:3,3].tolist(),yaw_rad=yaw_from_wxyz(np.array([q[3],q[0],q[1],q[2]])),
                              tracking_valid=bool(valid),tracking_quality=quality,filter_status=status)
                estimate={'estimated_position_m':pose[:3,3].tolist(),'estimated_yaw_rad':sample['yaw_rad'],
                          'estimator_status':status,'estimator_quality':quality,
                          'estimator_valid':bool(valid)}
            result=self.runtime.update(sample,now_s=sim_s)
            requested=result['command'];applied=requested.copy()
            if sim_s<self.hold_s:applied={'vx_m_s':0.,'yaw_rate_rad_s':0.}  # IMU 영점용 정지 구간
            fault=(self.fault_injection and self.runtime.state=='ascending' and self.runtime.route=='30'
                   and np.linalg.norm(position[:2]-self.runtime.mission.routes['30'][1])>=.18)
            if fault:
                applied={'vx_m_s':0.,'yaw_rate_rad_s':0.}
                if not self.fault_started:
                    self.runtime._event('fault_injected',sim_s,fault_type='zero_wheel_targets',position_m=position.tolist())
                    self.fault_started=True
            s=self.session
            modules=body_twist_to_modules(s.geometry,vx=applied['vx_m_s'],vy=0.,yaw_rate=applied['yaw_rate_rad_s'])
            s.controller.apply_action(s._action_type(joint_positions=np.array([m.steering_angle_rad for m in modules]),
                                                    joint_indices=s.steering_indices))
            gain=np.ones(len(modules)) if self.slip is None else self.slip.gains(s.geometry,sim_s)
            s.controller.apply_action(s._action_type(joint_velocities=np.array([m.wheel_angular_velocity_rad_s for m in modules])*gain,
                                                    joint_indices=s.wheel_indices))
            return {'stamp_s':float(sim_s),'wall_s':time.monotonic()-self.wall_start,
                    'position_m':position.tolist(),'quaternion_wxyz':quaternion.tolist(),
                    'joint_positions':np.asarray(self.robot.get_joint_positions(),float).tolist(),
                    'joint_velocities':np.asarray(self.robot.get_joint_velocities(),float).tolist(),
                    'requested_command':requested,'applied_command':applied,'mission':result,
                    'pose_source':'orb_eskf_estimate' if estimate else 'sim_ground_truth',
                    'fault_injection':self.fault_injection,'fault_active':bool(fault),'wheel_gain':gain.tolist(),**(estimate or {})}
        except BaseException:
            try:self.stop()
            except Exception:pass
            raise
