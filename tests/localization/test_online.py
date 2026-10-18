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


@pytest.mark.parametrize('offset,rpy', [([-.018, 0., -.021], [0., 0., 0.]),
                                      ([0., 0., 0.], [30., 0., 0.])])
def test_in_place_turn_preserves_base_position_with_nonidentity_imu_mount(offset: list[float], rpy: list[float]) -> None:
    """장착 위치·방향이 달라도 제자리 회전을 차체의 병진으로 적분하지 않는다."""
    base_from_imu = np.eye(4)
    base_from_imu[:3, 3] = offset
    base_from_imu[:3, :3] = Rotation.from_euler('xyz', rpy, degrees=True).as_matrix()
    calibration = {**CALIBRATION, 'IMU.T_b_c1': np.linalg.inv(base_from_imu).tolist()}
    estimator = OnlineEstimator(np.eye(4), calibration, lag_s=0.)
    for k in range(601):
        t = k*.01
        q = Rotation.from_euler('z', max(0., t-1.)*.6).as_quat()
        estimator.on_step(t, np.zeros(3), np.r_[q[3], q[:3]])
    pose, _, _, _ = estimator.pose(6.)
    assert np.linalg.norm(pose[:3, 3]) < .003


@pytest.mark.parametrize('include_zero_step', [True, False])
def test_initial_tilt_change_does_not_become_horizontal_acceleration(include_zero_step: bool) -> None:
    """시작 pose와 첫 물리 step 사이 회전도 적분해 정지 중 중력이 수평으로 새지 않는다."""
    estimator = OnlineEstimator(np.eye(4), CALIBRATION, lag_s=.1)
    for k in range(0 if include_zero_step else 1, 101):
        t = k*.05
        q = Rotation.from_euler('x', .001 if k else 0.).as_quat()
        estimator.on_step(t, np.zeros(3), np.r_[q[3], q[:3]])
    pose, _, _, _ = estimator.pose(5.)
    assert np.linalg.norm(pose[:3, 3]) < 1e-5
    assert Rotation.from_matrix(pose[:3, :3]).as_rotvec() == pytest.approx([.001, 0., 0.], abs=1e-7)


def test_optional_turn_schedule_reduces_noisy_visual_attitude_error() -> None:
    """회전 중 흔들리는 영상 자세가 정확한 IMU 자세를 끌어가는 정도를 줄인다."""
    errors = []
    for adaptive in (False, True):
        estimator = OnlineEstimator(np.eye(4), CALIBRATION, lag_s=0., adaptive_visual_noise=adaptive)
        angles = []
        for k in range(161):
            t = k*.05
            rotation = Rotation.from_euler('z', max(0., t-1.)*.6)
            q = rotation.as_quat()
            estimator.on_step(t, np.zeros(3), np.r_[q[3], q[:3]])
            if k % 2 == 0:
                observed = np.eye(4)
                observed[:3, :3] = (rotation*Rotation.from_euler('x', .04*np.sin(6*t))).as_matrix()
                estimator.on_frame(t, 2, '0', observed)
            if t > 2.:
                pose, _, _, _ = estimator.pose(t)
                angles.append((rotation.inv()*Rotation.from_matrix(pose[:3, :3])).magnitude())
        errors.append(float(np.sqrt(np.mean(np.square(angles)))))
    assert errors[1] < errors[0]*.7


def test_irregular_sim_steps_do_not_turn_small_vibration_into_position_drift() -> None:
    """5/50 ms가 번갈아 들어와도 0.4 mm 진동을 수 cm의 누적 이동으로 만들지 않는다."""
    estimator = OnlineEstimator(np.eye(4), CALIBRATION, lag_s=0.)
    t, worst = 0., 0.
    for k in range(400):
        t += (.005, .05)[k % 2]
        p = np.array([.0002*(1-np.cos(40*t)), 0., 0.])
        estimator.on_step(t, p, np.array([1., 0., 0., 0.]))
        pose, _, _, _ = estimator.pose(t)
        worst = max(worst, float(np.linalg.norm(pose[:3, 3]-p)))
    assert worst < .0002


def test_recorded_imu_intervals_replay_the_same_online_fusion() -> None:
    """GT를 다시 미분하지 않고 생성 당시 IMU만 재입력해 같은 필터 상태를 복원한다."""
    direct = OnlineEstimator(world_from_base(0.), CALIBRATION, lag_s=.1)
    replay = OnlineEstimator(world_from_base(0.), CALIBRATION, lag_s=.1)
    for k in range(1, 121):
        t = k*.05
        p, q = gt_pose(t)
        sample = direct.on_step(t, p, q)
        assert sample is not None
        start, end, gyro, force = sample
        replay.on_imu(end, end-start, gyro, force)
        if k % 2 == 0:
            observed = ORB_WORLD @ world_from_base(max(0., t-.1))
            direct.on_frame(t, 2, '0', observed)
            replay.on_frame(t, 2, '0', observed)
        assert direct.pose(t)[0] == pytest.approx(replay.pose(t)[0], abs=1e-12)


def test_causal_imu_ignores_repeated_or_reversed_timestamps() -> None:
    imu = CausalImu()
    for k in range(5):
        p, q = gt_pose(k*.05)
        imu.push(k*.05, p, q)
    p, q = gt_pose(.2)
    assert imu.push(.2, p, q) is None and imu.push(.1, p, q) is None
    p, q = gt_pose(.25)
    out = imu.push(.25, p, q)
    assert out is not None and np.isfinite(out[2]).all() and np.isfinite(out[3]).all()


def start_noise_estimator(**kwargs) -> tuple[OnlineEstimator, np.ndarray]:
    """추정기 prior는 원점이고 실제 로버는 (0.69 cm, 1.64 cm, 0.66°) 틀어져 정지해 있는 상황."""
    prior = np.eye(4)
    actual = np.eye(4)
    actual[:3, :3] = Rotation.from_euler('z', .66, degrees=True).as_matrix()
    actual[:3, 3] = [.00691, .01643, 0.]
    return OnlineEstimator(prior, CALIBRATION, **kwargs), actual


def still_samples(estimator: OnlineEstimator, actual: np.ndarray, steps: int = 8) -> list[tuple]:
    q = Rotation.from_matrix(actual[:3, :3]).as_quat()
    out = []
    for k in range(1, steps+1):
        sample = estimator.on_step(.02*k, actual[:3, 3], np.array([q[3], q[0], q[1], q[2]]))
        if sample is not None:
            out.append(sample)
    return out


def test_start_noise_does_not_create_a_startup_acceleration_spike_when_sensor_start_is_given() -> None:
    estimator, actual = start_noise_estimator(sensor_start_world_from_base=None)
    spiky = still_samples(estimator, actual)
    assert np.linalg.norm(spiky[0][3]) > 20.                    # 기존 방식: prior에서 GT로 점프를 미분해 가짜 가속도가 생긴다(재현)
    estimator, actual = start_noise_estimator()
    estimator, _ = OnlineEstimator(np.eye(4), CALIBRATION, sensor_start_world_from_base=actual), None
    fixed = still_samples(estimator, actual)
    for _, _, gyro, force in fixed:
        assert np.linalg.norm(force) == pytest.approx(9.81, abs=.1)
        assert np.linalg.norm(gyro) < .01


def test_sensor_start_pose_does_not_change_the_filter_prior() -> None:
    actual = np.eye(4)
    actual[:3, 3] = [.00691, .01643, 0.]
    estimator = OnlineEstimator(np.eye(4), CALIBRATION, sensor_start_world_from_base=actual)
    np.testing.assert_allclose(estimator.filter.pose[:3, 3], 0., atol=1e-12)   # 추정기는 여전히 이상적인 시작을 믿는다
    assert estimator.imu.velocity.shape == (3,)


def test_default_without_start_noise_is_unchanged() -> None:
    estimator, actual = start_noise_estimator()
    actual_same = np.eye(4)
    est_a = OnlineEstimator(np.eye(4), CALIBRATION)
    est_b = OnlineEstimator(np.eye(4), CALIBRATION, sensor_start_world_from_base=np.eye(4))
    for est in (est_a, est_b):
        samples = still_samples(est, actual_same)
        for _, _, _, force in samples:
            assert np.linalg.norm(force) == pytest.approx(9.81, abs=.01)
