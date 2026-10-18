# ================================================================
# test_direction_update.py
# 설명: 필터의 속도 방향 관측(direction_update)과 추정기의 on_motion이 크기는 믿지 않고 방향만으로 속도를 당겨 오는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/localization/test_direction_update.py -q
# ================================================================
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.localization.inertial_filter import InertialFilter
from experiments.localization.online import OnlineEstimator

NOISE = {'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .002, 'IMU.GyroWalk': .00002, 'IMU.AccWalk': .003}
CALIBRATION = {'ROS.T_c1_base': np.eye(4).tolist(), 'IMU.T_b_c1': np.eye(4).tolist(), **NOISE}


def filter_with_velocity(velocity, yaw_deg=0.) -> InertialFilter:
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('z', yaw_deg, degrees=True).as_matrix()
    f = InertialFilter(pose, NOISE)
    f.velocity[:] = velocity
    f.covariance[3:6, 3:6] = np.eye(3)*.05**2          # 속도 불확실성 5 cm/s
    return f


def test_direction_update_removes_sideways_velocity_but_keeps_speed_along_direction() -> None:
    f = filter_with_velocity([.08, .04, 0.])
    info = f.direction_update(np.array([1., 0.]), sigma=.01)
    assert info['accepted']
    assert abs(f.velocity[1]) < .01                       # 옆 성분이 크게 줄었다
    assert f.velocity[0] == pytest.approx(.08, abs=.01)   # 방향 관측은 크기를 바꾸지 않는다


def test_direction_update_leaves_a_velocity_already_along_the_direction() -> None:
    f = filter_with_velocity([.08, 0., 0.])
    f.direction_update(np.array([1., 0.]), sigma=.01)
    np.testing.assert_allclose(f.velocity, [.08, 0., 0.], atol=1e-6)


def test_direction_update_uses_the_imu_frame_so_rotation_is_respected() -> None:
    f = filter_with_velocity([.0, .08, 0.], yaw_deg=90.)  # 몸이 +y를 보고 있으므로 IMU 앞(+x)이 세계 +y
    f.direction_update(np.array([1., 0.]), sigma=.01)
    np.testing.assert_allclose(f.velocity, [.0, .08, 0.], atol=1e-6)
    g = filter_with_velocity([.08, .0, 0.], yaw_deg=90.)  # 세계 +x로 가는 속도는 IMU 기준 옆 성분
    g.direction_update(np.array([1., 0.]), sigma=.01)
    assert abs(g.velocity[0]) < .02


def test_direction_update_gates_an_inconsistent_observation() -> None:
    f = filter_with_velocity([.08, .08, 0.])
    f.covariance[3:6, 3:6] = np.eye(3)*1e-8               # 속도를 매우 확신하는데 관측이 45° 어긋남
    before = f.velocity.copy()
    info = f.direction_update(np.array([1., 0.]), sigma=.001)
    assert not info['accepted']
    np.testing.assert_allclose(f.velocity, before)


def test_direction_update_rejects_a_non_unit_or_zero_direction() -> None:
    f = filter_with_velocity([.08, .0, 0.])
    for bad in (np.zeros(2), np.array([np.nan, 1.])):
        with pytest.raises(ValueError):
            f.direction_update(bad, sigma=.01)


def estimator(sigma) -> OnlineEstimator:
    return OnlineEstimator(np.eye(4), CALIBRATION, lag_s=.1, direction_sigma=sigma)


def advance(est: OnlineEstimator, until_s: float, start_s: float = 0.) -> None:
    t = start_s
    while t < until_s-1e-9:
        t += .02
        est.on_imu(t, .02, np.zeros(3), np.array([0., 0., 9.81]))


def test_on_motion_is_ignored_when_direction_sigma_is_none() -> None:
    est = estimator(None)
    est.filter.velocity[:] = [.08, .04, 0.]
    est.on_motion(.1, .08, 0.)
    advance(est, .6)
    assert est.filter.velocity[1] == pytest.approx(.04, abs=1e-6)


def test_on_motion_pulls_the_filter_velocity_direction_after_the_vision_lag() -> None:
    est = estimator(.01)
    est.filter.velocity[:] = [.08, .04, 0.]
    est.on_motion(.1, .08, 0.)
    advance(est, .08)                                     # 필터 시각이 0.1 s 늦어 아직 관측 시각에 못 닿는다
    assert est.filter.velocity[1] == pytest.approx(.04, abs=1e-3)
    advance(est, .5, start_s=.08)
    assert abs(est.filter.velocity[1]) < .015


def test_on_motion_ignores_a_measurement_slower_than_the_speed_gate() -> None:
    est = estimator(.01)
    est.filter.velocity[:] = [.08, .04, 0.]
    est.on_motion(.1, .01, 0.)                            # 0.01 m/s는 방향이 불안정한 속도
    advance(est, .6)
    assert est.filter.velocity[1] == pytest.approx(.04, abs=1e-3)


def test_on_motion_direction_is_given_in_the_base_frame() -> None:
    est = estimator(.01)
    est.filter.velocity[:] = [.0, .08, 0.]                # IMU 앞=+x, 세계 +y로 가고 있음
    est.on_motion(.1, .0, .08)                            # 몸체 기준 옆(+y)으로 가고 있다고 측정
    advance(est, .6)
    assert est.filter.velocity[1] == pytest.approx(.08, abs=.01)
    assert abs(est.filter.velocity[0]) < .015
