# ================================================================
# test_imu_from_poses.py
# 설명: 20Hz GT pose에서 만든 이상적 200Hz IMU가 원운동의 해석 해와 맞는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_imu_from_poses.py
# ================================================================
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.site_sim.imu_from_poses import ideal_imu_from_poses


def circle(noise=0., hz=20., duration=20.):
    omega, radius = .4, .8
    t = np.arange(0, duration, 1/hz)
    p = np.column_stack([radius*np.cos(omega*t), radius*np.sin(omega*t), np.full_like(t, .4)])
    p = p+np.random.default_rng(0).normal(0, noise, p.shape)
    rot = Rotation.from_euler('z', omega*t+np.pi/2)
    q = rot.as_quat()
    return omega, radius, t, p, np.column_stack([q[:, 3], q[:, :3]])


def test_circle_gives_centripetal_force_gravity_and_body_z_rate() -> None:
    omega, radius, t, p, q = circle()
    times = np.arange(2., 18., .005)
    stamps, signal = ideal_imu_from_poses(t, p, q, times)
    assert np.array_equal(stamps, times)
    assert signal[:, :3] == pytest.approx(np.tile([0, 0, omega], (len(times), 1)), abs=5e-3)
    # body x is tangent, body y points to the circle center: centripetal acceleration on +y
    assert signal[:, 3] == pytest.approx(0., abs=.02)
    assert signal[:, 4] == pytest.approx(omega**2*radius, abs=.02)
    assert signal[:, 5] == pytest.approx(9.81, abs=.02)


def test_small_position_noise_does_not_create_large_acceleration_error() -> None:
    omega, radius, t, p, q = circle(noise=1e-4)
    stamps, signal = ideal_imu_from_poses(t, p, q, np.arange(2., 18., .005))
    assert abs(signal[:, 3:] - [0, omega**2*radius, 9.81]).max() < .2


def test_rejects_outside_range_and_bad_inputs() -> None:
    omega, radius, t, p, q = circle()
    with pytest.raises(ValueError):
        ideal_imu_from_poses(t, p, q, np.array([-1., 1.]))
    with pytest.raises(ValueError):
        ideal_imu_from_poses(t[::-1], p, q, np.array([1.]))
