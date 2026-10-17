# ================================================================
# online.py
# 설명: 시뮬 GT pose에서 만든 인과적 IMU와 ORB stereo pose를 ESKF로 한 스텝씩 융합하는 온라인 추정기.
#       영상이 실제 장면보다 lag_s 늦게 도착하는 렌더 지연을 필터 시각 지연으로 보상한다.
# 사용법:
#   est = OnlineEstimator(start_world_from_base, calibration, lag_s=.1)
#   est.on_step(t, position, quaternion_wxyz)      # 물리 step마다
#   est.on_frame(stamp_s, state, map_id, world_from_camera)   # stereo 프레임마다
#   est.pose(now)                                   # 제어 입력: (4x4 world_from_base, valid, quality, status)
# ================================================================
from collections import deque
import copy

import numpy as np
from scipy.spatial.transform import Rotation

from experiments.localization.fusion import move_baseline, needs_reanchor
from experiments.localization.inertial_filter import InertialFilter

GRAVITY = np.array([0., 0., 9.81])
# ORB 기본 설정의 IMU 잡음 시작값(Allan variance로 보정한 값이 아니다).
DEFAULT_NOISE = {'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .002, 'IMU.GyroWalk': 1.9393e-05, 'IMU.AccWalk': .003}


class CausalImu:
    """지난 GT pose 세 개에서 가장 최근 구간의 body-frame 자이로·비력을 인과적으로 만든다(시뮬 전용)."""

    def __init__(self) -> None:
        self.history = deque(maxlen=3)

    def push(self, t: float, position: np.ndarray, quaternion_wxyz: np.ndarray):
        """(구간 시작, 끝, 자이로, 비력)을 반환하고 표본이 모자라면 None을 반환한다."""
        q = np.asarray(quaternion_wxyz, float)
        rotation = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        self.history.append((float(t), np.asarray(position, float), rotation))
        if len(self.history) < 3:
            return None
        (t0, p0, r0), (t1, p1, r1), (t2, p2, r2) = self.history
        v01, v12 = (p1-p0)/(t1-t0), (p2-p1)/(t2-t1)
        acceleration = (v12-v01)/(.5*(t2-t0))
        delta = (r1.inv()*r2).as_rotvec()
        middle = r1*Rotation.from_rotvec(delta/2)
        return t1, t2, delta/(t2-t1), middle.inv().apply(acceleration+GRAVITY)


class OnlineEstimator:
    """IMU는 lag_s만큼 늦춰 필터에 넣고(영상과 시각을 맞춤), 제어용 pose는 남은 IMU로 현재까지 외삽한다."""

    def __init__(self, start_world_from_base: np.ndarray, calibration: dict, lag_s: float = .1,
                 good_s: float = 1., invalid_s: float = 4.) -> None:
        self.lag_s, self.good_s, self.invalid_s = lag_s, good_s, invalid_s
        left = np.linalg.inv(np.asarray(calibration['ROS.T_c1_base']))
        self.cam_from_base = np.asarray(calibration['ROS.T_c1_base'])   # world_from_base(ORB 세계) = world_from_cam @ cam_from_base
        self.base_from_imu = left @ np.linalg.inv(np.asarray(calibration['IMU.T_b_c1']))
        self.imu_from_base = np.linalg.inv(self.base_from_imu)
        sigma = np.array([.03]*3+[.1]*3+[.01]*3+[.005]*3+[.05]*3)
        self.filter = InertialFilter(start_world_from_base @ self.base_from_imu, calibration, np.diag(sigma**2))
        self.imu = CausalImu()
        self.pending = deque()                                # (t_end, dt, gyro, force) 아직 필터에 넣지 않은 표본
        self.filter_time = 0.
        self.reference = None
        self.previous_ok, self.map_id, self.last_accept = True, None, 0.
        self.update_time = -1.
        self.last_info: dict = {}

    def on_step(self, t: float, position: np.ndarray, quaternion_wxyz: np.ndarray) -> None:
        """물리 step마다 GT pose에서 IMU 표본을 만들고, 영상과 시각이 맞는 만큼만 필터를 전진시킨다."""
        sample = self.imu.push(t, position, quaternion_wxyz)
        if sample is not None:
            start, end, gyro, force = sample
            self.pending.append((end, end-start, gyro, force))
        while self.pending and self.pending[0][0] <= t-self.lag_s+1e-9:
            end, dt, gyro, force = self.pending.popleft()
            self.filter.propagate(gyro, force, dt)
            self.filter_time = end

    def on_frame(self, stamp_s: float, state: int, map_id: str, world_from_camera: np.ndarray) -> dict:
        """stereo 프레임의 ORB 결과를 상대 관측으로 융합한다(필터 시각은 stamp - lag)."""
        ok = state == 2 and int(map_id) >= 0
        info = {'accepted': False, 'reason': 'no_visual'}
        if ok:
            measured = world_from_camera @ self.cam_from_base @ self.base_from_imu
            if self.reference is None or needs_reanchor(self.previous_ok, ok, str(self.map_id), str(map_id)):
                self.filter.clone_current()
                self.reference, self.map_id = measured, map_id
                info = {'accepted': False, 'reason': 'reanchored'}
            elif stamp_s-self.lag_s-self.update_time >= .1-1e-8:
                info = self.filter.relative_update(np.linalg.inv(self.reference) @ measured)
                if move_baseline(self.filter, info['accepted'], True):
                    self.reference = measured
                self.update_time = stamp_s-self.lag_s
                if info['accepted']:
                    self.last_accept = stamp_s-self.lag_s
        self.previous_ok = ok
        self.last_info = info
        return info

    def pose(self, now: float) -> tuple[np.ndarray, bool, str, str]:
        """남은 IMU 표본으로 현재 시각까지 외삽한 base pose와 (valid, quality, filter status)를 반환한다."""
        projected = copy.deepcopy(self.filter)
        for _, dt, gyro, force in self.pending:
            projected.propagate(gyro, force, dt)
        since = now-self.last_accept
        quality = 'good' if since <= self.good_s else 'degraded'
        valid = since <= self.invalid_s and projected.status != 'imu_gap'
        return projected.pose @ self.imu_from_base, valid, quality, projected.status
