# ================================================================
# test_online.py
# 설명: 온라인 추정기가 렌더 지연이 있는 영상과 인과적 IMU로 GT 경로를 따라가는지 합성 주행으로 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/localization/test_online.py
# ================================================================
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.localization.online import CausalImu, OnlineEstimator

CALIBRATION = {'ROS.T_c1_base': np.eye(4).tolist(), 'IMU.T_b_c1': np.eye(4).tolist(),
               'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .002, 'IMU.GyroWalk': .00002, 'IMU.AccWalk': .003}
ORB_WORLD = np.eye(4)
ORB_WORLD[:3, :3] = Rotation.from_euler('z', 37, degrees=True).as_matrix()
ORB_WORLD[:3, 3] = [3., -2., .5]


def gt_pose(t: float) -> tuple[np.ndarray, np.ndarray]:
    """0.07 m/s로 직진하다 4초부터 0.4 rad/s로 도는 호(반지름 0.175 m)."""
    if t < 4:
        x, y, yaw = .07*t, 0., 0.
    else:
        s = t-4.
        x, y, yaw = .28+(.07/.4)*np.sin(.4*s), (.07/.4)*(1-np.cos(.4*s)), .4*s
    q = Rotation.from_euler('z', yaw).as_quat()
    return np.array([x, y, .4]), np.array([q[3], q[0], q[1], q[2]])


def world_from_base(t: float) -> np.ndarray:
    p, q = gt_pose(t)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()
    pose[:3, 3] = p
    return pose


def run(lag: float, compensate: bool, seconds: float = 12., dt: float = .05) -> float:
    start = world_from_base(0.)
    estimator = OnlineEstimator(start, CALIBRATION, lag_s=lag if compensate else 0.)
    worst = 0.
    for k in range(int(seconds/dt)+1):
        t = round(k*dt, 6)
        p, q = gt_pose(t)
        estimator.on_step(t, p, q)
        content = max(0., t-lag)                              # 영상은 lag 전의 장면을 보여 준다
        estimator.on_frame(t, 2, '0', ORB_WORLD @ world_from_base(content))
        if t > 2.:
            pose, valid, quality, status = estimator.pose(t)
            worst = max(worst, float(np.linalg.norm(pose[:2, 3]-p[:2])))
    return worst


def test_causal_imu_recovers_gravity_and_rates_from_poses() -> None:
    imu = CausalImu()
    out = None
    for k in range(120):
        t = k*.05
        p, q = gt_pose(t)
        out = imu.push(t, p, q) or out
    start, end, gyro, force = out
    assert gyro[2] == pytest.approx(.4, abs=1e-6) and end-start == pytest.approx(.05)
    assert force[2] == pytest.approx(9.81, abs=.05) and force[1] == pytest.approx(.07*.4, abs=.02)


def test_estimator_tracks_the_gt_path_with_lag_compensation() -> None:
    assert run(lag=.1, compensate=True) < .03


def test_ignoring_the_render_lag_is_clearly_worse() -> None:
    assert run(lag=.1, compensate=False) > run(lag=.1, compensate=True)
