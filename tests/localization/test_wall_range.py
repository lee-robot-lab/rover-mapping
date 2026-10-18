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


def test_wall_top_detector_finds_the_void_to_wall_boundary_per_column() -> None:
    from experiments.localization.wall_range import detect_wall_top
    image = np.full((120, 160), 50, np.uint8)
    image[60:, :] = 140                       # 위쪽 허공(어두움), 아래 벽·모래(밝음)
    image[:, :20] = 140                       # 맨 위가 허공이 아닌 열은 검출하지 않는다
    top = detect_wall_top(image, np.arange(4, 160, 8))
    assert np.isnan(top[:2]).all() and np.nanmedian(top) == pytest.approx(60, abs=1.5)


def test_wall_top_gate_keeps_close_walls_and_widens_only_when_too_few() -> None:
    from experiments.localization.wall_range import gate_wall_top
    distance = np.array([.5, .6, .7, 1.4, 1.8, 2.5])
    keep, limit = gate_wall_top(distance, min_columns=3)
    assert limit == 1. and keep.tolist() == [True, True, True, False, False, False]
    keep, limit = gate_wall_top(distance, min_columns=5)
    assert limit == 2. and keep.sum() == 5                                  # 1.5 m도 모자라 2 m까지 넓힌다
    keep, limit = gate_wall_top(np.array([2.5, 2.8]), min_columns=3)
    assert limit == 3. and keep.sum() == 2                                  # 끝까지 모자라면 가진 만큼만


def test_wall_top_residual_rejects_columns_that_are_far_from_the_prediction() -> None:
    from experiments.localization.wall_range import wall_top_residual
    predicted = np.array([100., 100., 100., 100.])
    measured = np.array([103., 98., 140., np.nan])
    rows, keep = wall_top_residual(measured, predicted, limit_px=15.)
    assert keep.tolist() == [True, True, False, False] and rows.tolist() == pytest.approx([3., -2.], abs=1e-9)


def test_wall_top_update_pulls_a_wrong_position_toward_the_measured_wall_top() -> None:
    from experiments.localization.wall_range import predicted_wall_top, wall_top_update
    scene = Scene(camera())
    columns = np.arange(8, 640, 8.)
    true_pose = base_pose(3.2, -.6)                                   # 윗선이 화면 중간(행≈62, 약 1.5 m)
    measured, _ = predicted_wall_top(scene, true_pose, columns)       # 정답 pose에서 보일 윗선(진단용 합성)
    filt = InertialFilter(base_pose(3.2+.05, -.6), DEFAULT_NOISE)      # 벽 정면(x) 방향 오차: 윗선 높이가 거리에 민감하다
    before = np.linalg.norm(filt.pose[:2, 3]-true_pose[:2, 3])
    info = wall_top_update(filt, scene, np.eye(4), measured, columns)
    after = np.linalg.norm(filt.pose[:2, 3]-true_pose[:2, 3])
    assert info['accepted'] and info['n'] >= 4
    assert after < before*.7


def _room_mesh():
    """바닥 없이 4 m x 3 m 방의 벽과 가운데 낮은 칸막이를 가진 삼각형 메쉬."""
    import open3d as o3d
    def wall(a, b, height):
        (x0, y0), (x1, y1) = a, b
        return [[x0, y0, 0.], [x1, y1, 0.], [x1, y1, height], [x0, y0, height]]
    quads = [wall((0, 0), (4, 0), .7), wall((4, 0), (4, 3), .7), wall((4, 3), (0, 3), .7), wall((0, 3), (0, 0), .7), wall((0.5, 1), (3.5, 1), .6)]
    vertices, faces = [], []
    for quad in quads:
        base = len(vertices)
        vertices += quad
        faces += [[base, base+1, base+2], [base, base+2, base+3]]
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(faces))
    return mesh, np.asarray(vertices)[np.asarray(faces)]


def test_mesh_raycasting_matches_the_brute_force_triangle_casting() -> None:
    mesh, triangles = _room_mesh()
    brute, fast = Scene(camera(), triangles=triangles), Scene(camera(), mesh=mesh)
    pose = base_pose(1., 2., .4)
    rays = brute.grid
    a, b = brute.depth(pose, rays, False), fast.depth(pose, rays, False)
    assert np.array_equal(np.isfinite(a), np.isfinite(b)) and np.nanmax(abs(a-b)) < 1e-3
    wa, wb = brute.depth(pose, rays, True), fast.depth(pose, rays, True)
    assert np.array_equal(np.isfinite(wa), np.isfinite(wb))


def test_interior_partition_is_part_of_the_predicted_wall_top() -> None:
    from experiments.localization.wall_range import predicted_wall_top
    mesh, triangles = _room_mesh()
    scene = Scene(camera(), mesh=mesh)
    pose = base_pose(2., .3, np.pi/2)                         # 칸막이(y=1, 높이 0.6 m)를 정면으로 바라본다
    rows, dist = predicted_wall_top(scene, pose, np.array([320.]))
    assert np.isfinite(rows).all() and dist[0] < 1.0          # 먼 외벽(약 2.7 m)이 아니라 가까운 칸막이가 윗선


def test_wall_top_frame_is_used_only_when_enough_columns_agree_with_the_map() -> None:
    from experiments.localization.wall_range import wall_top_agrees
    predicted = np.full(40, 100.)
    good = predicted+np.random.default_rng(0).normal(0, 1.2, 40)
    assert wall_top_agrees(good, predicted, tolerance_px=4., min_fraction=.6)
    mixed = good.copy();mixed[:20] += 30                        # 절반이 지도에 없는 구조물에 걸림
    assert not wall_top_agrees(mixed, predicted, tolerance_px=4., min_fraction=.6)
    assert not wall_top_agrees(np.full(40, np.nan), predicted)  # 검출이 없으면 쓰지 않는다
