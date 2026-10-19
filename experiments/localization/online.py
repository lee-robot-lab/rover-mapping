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


# 경사·회전 중 영상 관측 표준편차 배율(adaptive_visual_noise를 켰을 때만 쓴다).
VISUAL_SCHEDULE = {'tilt_deg': 8., 'slope_scale': 2., 'turn_rate': .3, 'turn_rotation_scale': 4., 'turn_position_scale': 4.}


class CausalImu:
    """GT pose에서 구간별 IMU를 만든다. 첫 구간 직전 속도는 정지 시작 조건에 따라 0으로 둔다(시뮬 전용)."""

    def __init__(self) -> None:
        self.history = deque(maxlen=3)
        self.velocity = np.zeros(3)

    def push(self, t: float, position: np.ndarray, quaternion_wxyz: np.ndarray) -> tuple[float, float, np.ndarray, np.ndarray] | None:
        """(구간 시작, 끝, 자이로, 비력)을 반환하고 표본이 모자라면 None을 반환한다."""
        if self.history and not t > self.history[-1][0]:
            return None  # 같은 시각·역전 시각은 dt=0 나눗셈으로 상태를 오염시키므로 무시한다.
        q = np.asarray(quaternion_wxyz, float)
        rotation = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        self.history.append((float(t), np.asarray(position, float), rotation))
        if len(self.history) < 2:
            return None
        (t1, p1, r1), (t2, p2, r2) = list(self.history)[-2:]
        v12 = (p2-p1)/(t2-t1)
        velocity = v12
        if len(self.history) == 3:
            t0, p0, _ = self.history[0]
            v01 = (p1-p0)/(t1-t0)
            velocity = v12+(v12-v01)*(t2-t1)/(t2-t0)
        # 3점 이차식의 구간 끝 속도를 차분해야 불규칙 dt에도 적분한 속도가 일관된다.
        # 중앙 가속도를 최신 dt에 곱하면 5/50 ms가 섞일 때 진동이 누적 이동으로 바뀐다.
        acceleration = (velocity-self.velocity)/(t2-t1)
        self.velocity = velocity
        delta = (r1.inv()*r2).as_rotvec()
        middle = r1*Rotation.from_rotvec(delta/2)
        return t1, t2, delta/(t2-t1), middle.inv().apply(acceleration+GRAVITY)


class OnlineEstimator:
    """IMU는 lag_s만큼 늦춰 필터에 넣고(영상과 시각을 맞춤), 제어용 pose는 남은 IMU로 현재까지 외삽한다."""

    def __init__(self, start_world_from_base: np.ndarray, calibration: dict, lag_s: float = .1,
                 good_s: float = 1., invalid_s: float = 4., adaptive_visual_noise: bool = False,
                 visual_schedule: dict | None = None, imu_realism: object | None = None, direction_sigma: float | None = None,
                 sensor_start_world_from_base: np.ndarray | None = None, gyro_bias_init_hold_s: float | None = None,
                 gyro_bias_sigma_dps: float = .03) -> None:
        self.lag_s, self.good_s, self.invalid_s = lag_s, good_s, invalid_s
        self.imu_realism = imu_realism                       # ImuRealism이면 GT에서 만든 IMU에 실기 특성을 더한다(기본 None)
        self._held_imu = None                                # 누락된 IMU 구간(시작, 끝, gyro, force)
        self.adaptive_visual_noise = adaptive_visual_noise
        self.visual_schedule = {**VISUAL_SCHEDULE, **(visual_schedule or {})}
        self.filter_gyro = np.zeros(3)
        left = np.linalg.inv(np.asarray(calibration['ROS.T_c1_base']))
        self.cam_from_base = np.asarray(calibration['ROS.T_c1_base'])   # world_from_base(ORB 세계) = world_from_cam @ cam_from_base
        self.base_from_imu = left @ np.linalg.inv(np.asarray(calibration['IMU.T_b_c1']))
        self.imu_from_base = np.linalg.inv(self.base_from_imu)
        sigma = np.array([.03]*3+[.1]*3+[.01]*3+[.005]*3+[.05]*3)
        self.filter = InertialFilter(start_world_from_base @ self.base_from_imu, calibration, np.diag(sigma**2))
        self.imu = CausalImu()
        # 필터 prior는 추정기가 믿는 시작 자세, 센서(IMU) 이력은 로버의 실제 시작 자세에서 시작한다(없으면 둘이 같다).
        # 시작 노이즈가 있을 때 prior에서 GT로의 점프를 미분하면 첫 구간에 가짜 가속도(수십 m/s²)가 생긴다.
        sensor_start = start_world_from_base if sensor_start_world_from_base is None else np.asarray(sensor_start_world_from_base, float)
        start_imu = sensor_start @ self.base_from_imu
        start_q = Rotation.from_matrix(start_imu[:3, :3]).as_quat()
        self.imu.push(0., start_imu[:3, 3], np.r_[start_q[3], start_q[:3]])
        self.pending = deque()                                # (t_end, dt, gyro, force) 아직 필터에 넣지 않은 표본
        self.direction_sigma = direction_sigma                # 속도 방향 관측 σ [m/s]. None이면 꺼짐
        self.motions = deque()                                # (stamp_s, base 축 단위 이동 방향) 아직 필터 시각에 닿지 않은 측정
        self.direction_updates = self.direction_rejected = 0
        self.gyro_bias_init_hold_s, self.gyro_bias_sigma_dps = gyro_bias_init_hold_s, gyro_bias_sigma_dps   # 정지 구간 평균으로 gyro bias를 한 번 초기화(None이면 꺼짐)
        self._bias_sum, self._bias_time, self._bias_done = np.zeros(3), 0., gyro_bias_init_hold_s is None
        self.filter_time = 0.
        self.reference = None
        self.previous_ok, self.map_id, self.last_accept = True, None, 0.
        self.update_time = -1.
        self.last_info: dict = {}

    def on_step(self, t: float, position: np.ndarray, quaternion_wxyz: np.ndarray) -> tuple[float, float, np.ndarray, np.ndarray] | None:
        """GT에서 만든 IMU 구간을 반환하고, 영상과 시각이 맞는 만큼만 필터를 전진시킨다."""
        if t == 0. and len(self.imu.history) == 1:
            return  # 생성자가 알려진 t=0 pose를 이미 넣었다.
        q = np.asarray(quaternion_wxyz, float)
        world_from_base = Rotation.from_quat([q[1], q[2], q[3], q[0]])
        imu_position = np.asarray(position, float)+world_from_base.apply(self.base_from_imu[:3, 3])
        imu_rotation = world_from_base*Rotation.from_matrix(self.base_from_imu[:3, :3])
        imu_q = imu_rotation.as_quat()
        sample = self.imu.push(t, imu_position, np.r_[imu_q[3], imu_q[:3]])
        if sample is not None:
            start, end, gyro, force = sample
            if self.imu_realism is not None:
                gyro, force = self.imu_realism.apply(end-start, float(np.linalg.norm(self.imu.velocity)),
                                                     self.base_from_imu, gyro, force)
                held = self._held_imu
                if self.imu_realism.should_drop(0. if held is None else held[1]-held[0], end-start):
                    # 패킷 누락: 구간을 잡아 두었다가 다음 패킷에 합친다(구간 평균이라 시간 가중 평균).
                    self._held_imu = (start, end, gyro, force) if held is None else (
                        held[0], end, (held[2]*(held[1]-held[0])+gyro*(end-start))/(end-held[0]),
                        (held[3]*(held[1]-held[0])+force*(end-start))/(end-held[0]))
                    return None
                if held is not None:
                    total = end-held[0]
                    gyro = (held[2]*(held[1]-held[0])+gyro*(end-start))/total
                    force = (held[3]*(held[1]-held[0])+force*(end-start))/total
                    start, self._held_imu = held[0], None
                sample = (start, end, gyro, force)
            self.on_imu(end, end-start, gyro, force)
        return sample

    def on_imu(self, end_s: float, dt_s: float, gyro: np.ndarray, force: np.ndarray) -> None:
        """생성한 구간 평균 IMU를 같은 지연 규칙으로 수용한다. 기록 재생은 GT를 다시 미분하지 않는다."""
        self._init_gyro_bias(end_s, dt_s, gyro)
        self.pending.append((end_s, dt_s, gyro, force))
        while self.pending and self.pending[0][0] <= end_s-self.lag_s+1e-9:
            end, dt, gyro, force = self.pending.popleft()
            self.filter.propagate(gyro, force, dt)
            self.filter_gyro = gyro
            self.filter_time = end
            self._apply_motion(end)

    def _init_gyro_bias(self, end_s: float, dt_s: float, gyro: np.ndarray) -> None:
        """시작 정지 구간(hold-0.3 s까지)의 gyro 시간 평균을 필터 bias로 한 번 넣고 z bias 사전 분산을 줄인다."""
        if self._bias_done:
            return
        if end_s <= self.gyro_bias_init_hold_s-.3:
            self._bias_sum += np.asarray(gyro, float)*dt_s
            self._bias_time += dt_s
        elif self._bias_time > 0.:
            self.filter.gyro_bias[:] = self._bias_sum/self._bias_time
            self.filter.covariance[11, :] = 0.      # 교차공분산까지 지워야 깨끗한 사전분포가 된다(대각만 줄이면 다른 상태와의 상관이 남는다)
            self.filter.covariance[:, 11] = 0.
            self.filter.covariance[11, 11] = np.deg2rad(self.gyro_bias_sigma_dps)**2
            self._bias_done = True

    def on_motion(self, stamp_s: float, vx: float, vy: float, min_speed: float = .03) -> None:
        """base 축 평면 속도 측정(조향·바퀴 조인트)의 방향을 보관한다. 느린 속도는 방향이 불안정해 버리고, 크기는 쓰지 않는다."""
        speed = float(np.hypot(vx, vy))
        if self.direction_sigma is None or not np.isfinite(speed) or speed < min_speed:
            return
        self.motions.append((float(stamp_s), np.array([vx, vy])/speed))

    def _apply_motion(self, filter_end_s: float) -> None:
        """필터 시각까지 도착한 측정 중 가장 최근 것으로 속도 방향 관측을 적용한다."""
        latest = None
        while self.motions and self.motions[0][0] <= filter_end_s+1e-9:
            latest = self.motions.popleft()
        if latest is None:
            return
        direction = (self.imu_from_base[:3, :3] @ np.r_[latest[1], 0.])[:2]   # base 축 방향 → IMU 축
        if float(np.linalg.norm(direction)) < 1e-6:
            return
        info = self.filter.direction_update(direction, self.direction_sigma)
        self.direction_updates += 1
        self.direction_rejected += 0 if info['accepted'] else 1

    def on_frame(self, stamp_s: float, state: int, map_id: str, world_from_camera: np.ndarray) -> dict:
        """stereo 프레임의 ORB 결과를 상대 관측으로 융합한다(필터 시각은 stamp - lag)."""
        ok = state == 2 and int(map_id) >= 0
        info = {'accepted': False, 'reason': 'not_due' if ok else 'no_visual'}
        if ok:
            measured = world_from_camera @ self.cam_from_base @ self.base_from_imu
            if self.reference is None or needs_reanchor(self.previous_ok, ok, str(self.map_id), str(map_id)):
                self.filter.clone_current()
                self.reference, self.map_id = measured, map_id
                info = {'accepted': False, 'reason': 'reanchored'}
            elif stamp_s-self.lag_s-self.update_time >= .1-1e-8:
                position_scale = rotation_scale = 1.
                if self.adaptive_visual_noise:
                    # 시뮬 비교용 휴리스틱. 실기 noise/슬립 보정값으로 간주하지 않는다.
                    body = self.filter.pose @ self.imu_from_base
                    tilt = max(abs(Rotation.from_matrix(body[:3, :3]).as_euler('xyz')[:2]))
                    gyro = self.base_from_imu[:3, :3] @ (self.filter_gyro-self.filter.gyro_bias)
                    plan = self.visual_schedule
                    if tilt > np.deg2rad(plan['tilt_deg']):
                        position_scale = rotation_scale = plan['slope_scale']
                    elif abs(gyro[2]) > plan['turn_rate']:
                        rotation_scale, position_scale = plan['turn_rotation_scale'], plan['turn_position_scale']
                info = self.filter.relative_update(np.linalg.inv(self.reference) @ measured,
                                                   position_scale=position_scale, rotation_scale=rotation_scale)
                info.update(position_scale=position_scale, rotation_scale=rotation_scale)
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
