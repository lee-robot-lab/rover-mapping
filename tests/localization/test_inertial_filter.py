# ================================================================
# test_inertial_filter.py
# 설명: IMU 좌표계·stochastic clone의 gauge·잘못된 영상 관측 거부를 검사한다.
# 사용법: python3 -m pytest tests/orbslam3_offline/test_inertial_filter.py -q
# ================================================================
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.localization.inertial_filter import InertialFilter

NOISE = {'IMU.NoiseGyro': .00017, 'IMU.NoiseAcc': .002,
         'IMU.GyroWalk': .00002, 'IMU.AccWalk': .003}


def test_tilted_static_imu_does_not_create_translation() -> None:
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', [.3, -.2, .7]).as_matrix()
    f = InertialFilter(pose, NOISE)
    accel = pose[:3, :3].T @ [0, 0, 9.81]
    for _ in range(400):
        f.propagate(np.zeros(3), accel, .005)
    np.testing.assert_allclose(f.pose, pose, atol=1e-10)
    assert np.linalg.eigvalsh(f.covariance).min() > -1e-10


def test_relative_vision_preserves_global_translation_uncertainty() -> None:
    f = InertialFilter(np.eye(4), NOISE)
    initial_variance = f.covariance[0, 0]
    for _ in range(10):
        for _ in range(20):
            f.propagate(np.zeros(3), np.array([0., 0., 9.81]), .005)
        assert f.relative_update(np.eye(4))['accepted']
    assert f.covariance[0, 0] >= initial_variance*.99
    assert f.covariance[3, 3] < .1**2


def test_relative_vision_does_not_replace_pose_with_a_jump() -> None:
    f = InertialFilter(np.eye(4), NOISE)
    f.propagate(np.zeros(3), np.array([0., 0., 9.81]), .1)
    bad = np.eye(4)
    bad[0, 3] = 10
    result = f.relative_update(bad)
    assert not result['accepted']
    np.testing.assert_allclose(f.pose, np.eye(4), atol=1e-12)


def test_relative_vision_is_independent_of_world_origin() -> None:
    pose = np.eye(4)
    pose[:3, 3] = [100, -20, 3]
    a, b = InertialFilter(np.eye(4), NOISE), InertialFilter(pose, NOISE)
    z = np.eye(4)
    z[0, 3] = .01
    for f in (a, b):
        for _ in range(20):
            f.propagate(np.zeros(3), np.array([0., 0., 9.81]), .005)
        assert f.relative_update(z)['accepted']
    np.testing.assert_allclose(b.pose[:3, 3]-a.pose[:3, 3], pose[:3, 3], atol=1e-8)


def test_nonpositive_time_step_is_rejected() -> None:
    f = InertialFilter(np.eye(4), NOISE)
    with pytest.raises(ValueError):
        f.propagate(np.zeros(3), np.ones(3), 0)


def settled_filter() -> InertialFilter:
    f = InertialFilter(np.eye(4), NOISE)
    for _ in range(20):
        f.propagate(np.zeros(3), np.array([0., 0., 9.81]), .005)
    return f


def test_imu_gap_stops_integration_and_marks_degraded() -> None:
    f = settled_filter()
    f.velocity[:] = [.1, 0., 0.]
    before = f.pose.copy()
    f.imu_gap(1.0)
    np.testing.assert_allclose(f.pose, before, atol=1e-12)
    np.testing.assert_allclose(f.velocity, 0.)
    assert f.status == 'imu_gap' and f.degraded
    assert f.covariance[0, 0] >= (.3*1.0)**2
    assert np.linalg.eigvalsh(f.covariance).min() > -1e-10


def test_imu_gap_uncertainty_is_cumulative_not_summed_per_call() -> None:
    a, b = settled_filter(), settled_filter()
    a.imu_gap(1.0)
    for _ in range(10):
        b.imu_gap(.1)
    assert b.covariance[0, 0] == pytest.approx(a.covariance[0, 0], rel=.05)
    assert b.covariance[6:9, 6:9].trace() == pytest.approx(a.covariance[6:9, 6:9].trace(), rel=.05)


def test_vision_recovers_after_motion_during_imu_gap_but_not_without_gap() -> None:
    moved = np.eye(4)
    moved[0, 3] = .3
    plain = settled_filter()
    assert not plain.relative_update(moved)['accepted']
    f = settled_filter()
    f.imu_gap(2.0)
    f.propagate(np.zeros(3), np.array([0., 0., 9.81]), .02)
    assert f.status == 'recovering'
    assert f.relative_update(moved)['accepted']
    assert f.pose[0, 3] == pytest.approx(.3, abs=.05)
    assert f.status == 'ok' and not f.degraded


def test_integration_step_above_100ms_is_still_rejected() -> None:
    with pytest.raises(ValueError):
        settled_filter().propagate(np.zeros(3), np.array([0., 0., 9.81]), .2)


def test_gravity_realigns_tilt_after_gap_when_vision_is_also_unavailable() -> None:
    f = settled_filter()
    f.imu_gap(2.0)
    true = Rotation.from_euler('xyz', [.5, -.3, 0.]).as_matrix()  # rover tilted while no IMU arrived
    accel = true.T @ [0., 0., 9.81]
    for _ in range(50):
        f.propagate(np.zeros(3), accel, .02)
    assert np.linalg.norm(f.pose[:3, 3]) < .02
    assert np.degrees(Rotation.from_matrix(f.pose[:3, :3].T @ true).magnitude()) < 1.
    assert f.covariance[6, 6] <= np.deg2rad(5.)**2 and f.covariance[7, 7] <= np.deg2rad(5.)**2


def test_velocity_update_pulls_a_wrong_velocity_toward_the_encoder_value() -> None:
    f = settled_filter()
    f.velocity[:] = [.2, 0., 0.]
    f.covariance[3:6, 3:6] = np.eye(3)*.1**2
    info = f.velocity_update(np.array([.07, 0.]), sigma=.02)
    assert info['accepted']
    assert f.velocity[0] == pytest.approx(.07, abs=.04) and abs(f.velocity[1]) < .02


def test_velocity_update_uses_the_body_frame_of_the_current_attitude() -> None:
    f = InertialFilter(np.eye(4), NOISE)
    f.pose[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()   # body x points to world +y
    f.velocity[:] = [0., .1, 0.]
    f.covariance[3:6, 3:6] = np.eye(3)*.05**2
    before = f.velocity.copy()
    assert f.velocity_update(np.array([.1, 0.]), sigma=.02)['accepted']
    assert np.linalg.norm(f.velocity-before) < .01     # already consistent: no change


def test_velocity_update_rejects_an_implausible_measurement() -> None:
    f = settled_filter()
    assert not f.velocity_update(np.array([3., 0.]), sigma=.02)['accepted']


def test_gap_dead_reckons_with_the_encoder_twist_instead_of_standing_still() -> None:
    f = settled_filter()
    for _ in range(10):
        f.imu_gap(.1, twist=(.07, 0., 0.))
    assert f.pose[0, 3] == pytest.approx(.07, abs=1e-6) and f.velocity[0] == pytest.approx(.07)
    assert f.covariance[0, 0] < (.3*1.)**2        # encoder bounds the growth below the no-information assumption


def test_gap_encoder_yaw_rate_rotates_the_attitude() -> None:
    f = settled_filter()
    for _ in range(10):
        f.imu_gap(.1, twist=(0., 0., .5))
    assert Rotation.from_matrix(f.pose[:3, :3]).as_euler('xyz')[2] == pytest.approx(.5, abs=1e-6)


def test_larger_visual_noise_scale_gives_a_smaller_correction() -> None:
    def corrected(scale: float) -> float:
        f = settled_filter()
        moved = np.eye(4)
        moved[0, 3] = .05
        f.relative_update(moved, position_scale=scale)
        return f.pose[0, 3]
    assert 0. < corrected(5.) < corrected(1.) <= .05


def test_rotation_scale_softens_the_attitude_correction_but_accepts_larger_disagreement() -> None:
    def trial(rotation_scale: float) -> tuple[bool, float]:
        f = settled_filter()
        moved = np.eye(4)
        moved[:3, :3] = Rotation.from_euler('z', 3, degrees=True).as_matrix()
        info = f.relative_update(moved, rotation_scale=rotation_scale)
        return info['accepted'], abs(Rotation.from_matrix(f.pose[:3, :3]).as_euler('xyz')[2])
    strict_ok, strict_yaw = trial(1.)
    loose_ok, loose_yaw = trial(4.)
    assert loose_ok                       # 영상 회전을 덜 믿으면 3도 불일치도 거부하지 않는다
    assert (not strict_ok) or loose_yaw < strict_yaw
