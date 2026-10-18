# ================================================================
# test_imu_realism.py
# 설명: ImuRealism이 정지 바이어스·백색 잡음·이동 진동을 body 축 기준으로 정확한 크기로 더하고, 꺼 두면 기존 추정기 동작이 변하지 않는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/localization/test_imu_realism.py
# ================================================================
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.localization.imu_realism import G, ImuRealism
from experiments.localization.online import OnlineEstimator

EYE = np.eye(4)
CALIBRATION = {'ROS.T_c1_base': np.eye(4).tolist(), 'IMU.T_b_c1': np.eye(4).tolist(),
               'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .002, 'IMU.GyroWalk': .00002, 'IMU.AccWalk': .003}


def draw(model: ImuRealism, n: int, dt: float, speed: float, base_from_imu=EYE):
    """같은 입력(gyro 0, 정지 비력)에 n번 적용한 결과를 (n,3) 배열 둘로 돌려준다."""
    gyro = np.zeros(3)
    force = np.array([0., 0., G])
    out = [model.apply(dt, speed, base_from_imu, gyro, force) for _ in range(n)]
    return np.array([o[0] for o in out]), np.array([o[1] for o in out])


def test_still_has_bias_and_white_noise_scaled_by_dt():
    gyro, force = draw(ImuRealism(seed=1), 20000, .02, 0.)
    assert gyro.mean(0) == pytest.approx(np.radians((-.630, .485, .488)), abs=2e-5)
    assert gyro.std(0) == pytest.approx(1.1e-4/np.sqrt(.02), rel=.05)          # 7.8e-4 rad/s = 0.0446 dps
    assert (force-[0, 0, G]).std(0) == pytest.approx(.025/np.sqrt(.02), rel=.05)  # 0.177 m/s² = 0.018 g
    gyro50, _ = draw(ImuRealism(seed=1), 20000, .05, 0.)
    assert gyro50.std(0)[0] == pytest.approx(gyro.std(0)[0]*np.sqrt(.02/.05), rel=.06)


def test_moving_uses_measured_vibration_std():
    gyro, force = draw(ImuRealism(seed=2), 20000, .02, .1)
    assert np.degrees(gyro.std(0)) == pytest.approx((1.44, 2.51, 2.87), rel=.05)
    assert (force-[0, 0, G]).std(0)/G == pytest.approx((.034, .022, .026), rel=.05)
    half, _ = draw(ImuRealism(seed=2), 20000, .02, .04)                        # 속도 절반이면 표준편차도 중간
    assert 0 < np.degrees(half.std(0)[2]) < 2.87 and np.degrees(half.std(0)[2]) > 1.2


def test_vibration_scale_only_scales_motion_noise():
    still_a, _ = draw(ImuRealism(seed=3, vibration_scale=2.), 5000, .02, 0.)
    still_b, _ = draw(ImuRealism(seed=3, vibration_scale=1.), 5000, .02, 0.)
    assert still_a.std(0) == pytest.approx(still_b.std(0))
    moving, _ = draw(ImuRealism(seed=3, vibration_scale=2.), 20000, .02, .1)
    assert np.degrees(moving.std(0)[2]) == pytest.approx(5.74, rel=.05)


def test_noise_is_added_in_body_frame_and_returned_to_imu_axes():
    r = Rotation.from_euler('z', 90, degrees=True).as_matrix()                   # body = R @ imu
    base_from_imu = np.eye(4)
    base_from_imu[:3, :3] = r
    gyro, _ = draw(ImuRealism(seed=4), 20000, .02, 0., base_from_imu)
    assert gyro.mean(0) == pytest.approx(r.T @ np.radians((-.630, .485, .488)), abs=2e-5)
    moving, _ = draw(ImuRealism(seed=4), 20000, .02, .1, base_from_imu)
    body = moving @ r.T
    assert np.degrees(body.std(0)) == pytest.approx((1.44, 2.51, 2.87), rel=.05)


def test_seed_is_reproducible_and_invalid_dt_rejected():
    a, _ = draw(ImuRealism(seed=5), 10, .02, 0.)
    b, _ = draw(ImuRealism(seed=5), 10, .02, 0.)
    c, _ = draw(ImuRealism(seed=6), 10, .02, 0.)
    assert np.array_equal(a, b) and not np.array_equal(a, c)
    with pytest.raises(ValueError):
        ImuRealism().apply(0., 0., EYE, np.zeros(3), np.zeros(3))
    assert ImuRealism(seed=7).parameters()['gyro_bias_dps'] == (-.630, .485, .488)


def test_online_estimator_default_off_unchanged_and_on_changes_sample_and_filter_input():
    def run(realism):
        est = OnlineEstimator(EYE, CALIBRATION, imu_realism=realism)
        samples = []
        for k in range(1, 40):
            t = .02*k
            q = Rotation.identity().as_quat()
            samples.append(est.on_step(t, np.array([.07*t, 0., 0.]), np.r_[q[3], q[:3]]))
        return [s for s in samples if s is not None], est
    base, _ = run(None)
    again, _ = run(None)
    noisy, est = run(ImuRealism(seed=1))
    assert all(np.array_equal(a[2], b[2]) and np.array_equal(a[3], b[3]) for a, b in zip(base, again))
    assert not np.allclose(base[-1][2], noisy[-1][2]) and np.allclose(base[0][:2], noisy[0][:2])
    assert est.pending or est.filter_time > 0                                  # 잡음 낀 구간도 필터로 들어간다
