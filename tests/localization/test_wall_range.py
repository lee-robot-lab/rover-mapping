# ================================================================
# test_wall_range.py
# 설명: STL 벽 지도 로드, 벽 깊이 예측, 벽 거리(R) 보정이 알려진 위치 오차를 줄이는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/localization/test_wall_range.py -q
# ================================================================
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from experiments.localization.inertial_filter import InertialFilter
from experiments.localization.online import DEFAULT_NOISE
from experiments.localization.wall_range import (Camera, DEPTH_RANGE, Scene, add_depth_noise, column_scan, load_stl,
                                                  range_update)

DEPTH_MIN = DEPTH_RANGE[0]


def camera() -> Camera:
    """차체 앞 x, 광축이 아래 20°인 왼쪽 광학 프레임(z 전방)을 base에서 본 변환."""
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    pitch = Rotation.from_euler('y', np.deg2rad(20.)).as_matrix()
    transform = np.eye(4)
    transform[:3, :3] = pitch @ optical
    transform[:3, 3] = [.12, .03, .02]
    return Camera(377.186, 377.186, 320., 240., .05, transform)


def base_pose(x: float, y: float, yaw: float = 0.) -> np.ndarray:
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    pose[:3, 3] = [x, y, .4]
    return pose


def test_fence_stl_is_in_metres_and_matches_the_arena() -> None:
    triangles = load_stl()
    assert len(triangles) == 180
    low, high = triangles.reshape(-1, 3).min(0), triangles.reshape(-1, 3).max(0)
    assert low[:2] == pytest.approx([2.5, -1.61], abs=.02) and high[:2] == pytest.approx([6.51, 1.6], abs=.02)


def test_scene_predicts_the_distance_to_a_wall_ahead() -> None:
    scene = Scene(camera())
    z = scene.depth(base_pose(4.5, 0.), scene.sparse, True)
    assert np.isfinite(z).sum() > 5
    assert np.nanmedian(z) == pytest.approx(1.85, abs=.15)     # 정면 벽 x≈6.5까지 약 2 m, 카메라가 12 cm 앞
    assert np.nanmin(z) > DEPTH_MIN and np.nanmax(z) <= 3.0


def test_range_update_pulls_a_wrong_position_back_toward_the_wall_measurement() -> None:
    scene = Scene(camera())
    true_pose = base_pose(4.5, .1)
    rays = scene.fine
    z_meas = scene.depth(true_pose, rays, True)
    imu_from_base = np.eye(4)
    wrong = base_pose(4.5+.05, .1-.04)
    filt = InertialFilter(wrong, DEFAULT_NOISE)
    before = np.linalg.norm(filt.pose[:2, 3]-true_pose[:2, 3])
    info = range_update(filt, scene, imu_from_base, z_meas, rays)
    after = np.linalg.norm(filt.pose[:2, 3]-true_pose[:2, 3])
    assert info['accepted'] and info['n'] >= 6
    assert after < before*.8      # 벽 거리는 벽에 수직한 성분(x)을 크게 줄인다


def test_column_scan_recovers_the_true_depth_from_an_exact_depth_image() -> None:
    scene = Scene(camera())
    pose = base_pose(4.5, 0.)
    image = np.full((480, 640), np.nan, np.float32)
    c = scene.camera
    full = scene._rays(np.arange(640.), np.arange(480.))
    z = scene.depth(pose, full, False)
    image[:] = z.reshape(480, 640)
    rays, z_meas = column_scan(scene, pose, image, bias=1.0)
    z_pred = scene.depth(pose, rays, True)
    assert len(z_meas) >= 6 and np.nanmax(abs(z_meas-z_pred)) < .02


def test_added_depth_noise_has_the_requested_size_dropout_and_outliers() -> None:
    z = np.full(20000, 2.)
    noisy = add_depth_noise(z, np.random.default_rng(1), scale=1., dropout=.15, outlier=.03)
    valid = noisy[np.isfinite(noisy)]
    assert np.isnan(noisy).mean() == pytest.approx(.15, abs=.02)
    core = valid[abs(valid-2.) < .1]
    assert core.std() == pytest.approx(.01*2.+.005, rel=.2)            # σ = 1%·z + 5 mm
    assert (abs(valid-2.) > .1).mean() == pytest.approx(.03, abs=.015)   # 이상치는 ±0.3 m 안에서 퍼진다
    assert np.array_equal(add_depth_noise(z, np.random.default_rng(1), scale=0., dropout=0., outlier=0.), z)
