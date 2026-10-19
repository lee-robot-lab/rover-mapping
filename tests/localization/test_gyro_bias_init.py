# ================================================================
# test_gyro_bias_init.py
# 설명: 정지 구간 gyro 평균으로 필터 gyro bias를 한 번 초기화하고 z 사전 분산을 줄이는 옵션이 의도대로, 켰을 때만 동작하는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/localization/test_gyro_bias_init.py -q
# ================================================================
import numpy as np
import pytest

from experiments.localization.online import OnlineEstimator

NOISE = {'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .002, 'IMU.GyroWalk': .00002, 'IMU.AccWalk': .003}
CALIBRATION = {'ROS.T_c1_base': np.eye(4).tolist(), 'IMU.T_b_c1': np.eye(4).tolist(), **NOISE}
BIAS = np.deg2rad([-.637, .485, .486])


def run(est: OnlineEstimator, until_s: float) -> None:
    t = 0.
    while t < until_s-1e-9:
        t += .02
        est.on_imu(t, .02, BIAS, np.array([0., 0., 9.81]))


def test_bias_init_uses_the_stationary_mean_and_shrinks_the_z_prior() -> None:
    est = OnlineEstimator(np.eye(4), CALIBRATION, lag_s=.1, gyro_bias_init_hold_s=3., gyro_bias_sigma_dps=.03)
    prior_z = est.filter.covariance[11, 11]
    run(est, 2.5)
    assert np.allclose(est.filter.gyro_bias, 0.)            # 정지 구간 안에서는 아직 적용하지 않는다
    t = 2.5
    while t < 3.5:
        t += .02
        est.on_imu(t, .02, BIAS, np.array([0., 0., 9.81]))
    np.testing.assert_allclose(est.filter.gyro_bias, BIAS, atol=1e-6)
    assert est.filter.covariance[11, 11] == pytest.approx(np.deg2rad(.03)**2, rel=.05)   # 적용 뒤 전파가 랜덤워크로 조금 키운다
    assert est.filter.covariance[11, 11] < prior_z


def test_bias_init_is_applied_only_once() -> None:
    est = OnlineEstimator(np.eye(4), CALIBRATION, lag_s=.1, gyro_bias_init_hold_s=3.)
    run(est, 3.5)
    est.filter.gyro_bias[:] = [.1, .2, .3]                  # 이후 필터가 바꾼 값을 다시 덮어쓰면 안 된다
    t = 3.5
    while t < 5.:
        t += .02
        est.on_imu(t, .02, BIAS*3, np.array([0., 0., 9.81]))
    assert not np.allclose(est.filter.gyro_bias, BIAS)


def test_default_leaves_the_filter_bias_alone() -> None:
    est = OnlineEstimator(np.eye(4), CALIBRATION, lag_s=.1)
    run(est, 4.)
    assert est.filter.covariance[11, 11] > np.deg2rad(.03)**2      # 사전 분산이 줄지 않았다
    assert not np.allclose(est.filter.gyro_bias, BIAS, atol=1e-6)


def test_bias_init_clears_the_cross_covariance_of_the_gyro_bias_z() -> None:
    """gb_z 사전 분산만 줄이고 교차공분산을 남기면 다른 상태와의 상관이 그대로라 오프라인 평가(깨끗한 초기화)와 다른 필터가 된다."""
    est = OnlineEstimator(np.eye(4), CALIBRATION, lag_s=.1, gyro_bias_init_hold_s=3., gyro_bias_sigma_dps=.03)
    est.filter.covariance[11, 8] = est.filter.covariance[8, 11] = .5*np.sqrt(est.filter.covariance[11, 11]*est.filter.covariance[8, 8])
    t = 0.
    while t < 2.8:
        t += .02
        est.on_imu(t, .02, BIAS, np.array([0., 0., 9.81]))
    assert abs(est.filter.covariance[11, 8]) < 1e-6
    assert abs(est.filter.covariance[8, 11]) < 1e-6
