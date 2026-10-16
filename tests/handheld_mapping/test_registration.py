# ================================================================
# test_registration.py
# 설명: 비대칭 3D 정합의 방향과 관측 부족·잘못된 prior 거부를 검증한다.
# 사용법: python -m pytest tests/handheld_mapping/test_registration.py -q
# ================================================================
import copy

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation


def corner_cloud() -> o3d.geometry.PointCloud:
    rng = np.random.default_rng(10)
    uv = rng.uniform([-.35, -.25], [.4, .3], (2200, 2))
    points = np.concatenate([np.column_stack((uv, np.full(len(uv), .12))),
                             np.column_stack((uv[:, 0], np.full(len(uv), -.25), uv[:, 1]+.5)),
                             np.column_stack((np.full(len(uv), .4), uv[:, 0], uv[:, 1]+.5))])
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))


def test_asymmetric_registration_recovers_source_to_target_without_mutation():
    from experiments.handheld_mapping.registration import register_clouds
    source = corner_cloud()
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler('xyz', [4, -6, 8], degrees=True).as_matrix()
    expected[:3, 3] = [.07, -.03, .04]
    target = copy.deepcopy(source).transform(expected)
    original = np.asarray(source.points).copy()
    initial = expected.copy()
    initial[:3, 3] += [.025, -.02, .015]
    result = register_clouds(source, target, initial)
    assert result['accepted'], result
    assert np.allclose(result['transformation'], expected, atol=.003)
    assert result['forward']['correspondences'] >= 100
    assert result['backward']['fitness'] >= .3
    assert result['observability']['ratio'] >= 1e-4
    assert np.array_equal(original, np.asarray(source.points))
    assert np.asarray(result['information']).shape == (6, 6)
    assert result['point_to_point_diagnostic']['correspondences'] >= 100


def test_plane_is_not_a_full_se3_constraint():
    from experiments.handheld_mapping.registration import register_clouds
    x, y = np.meshgrid(np.linspace(-.4, .4, 45), np.linspace(-.4, .4, 45))
    plane = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.column_stack((x.ravel(), y.ravel(), x.ravel()*0))))
    result = register_clouds(plane, plane, np.eye(4))
    assert not result['accepted']
    assert result['reason'] == 'unobservable'


def test_zero_correspondences_cannot_succeed_with_zero_rmse():
    from experiments.handheld_mapping.registration import register_clouds
    source = corner_cloud()
    target = copy.deepcopy(source).translate([4, 0, 0])
    result = register_clouds(source, target, np.eye(4))
    assert not result['accepted']
    assert result['forward']['correspondences'] == 0


def test_invalid_se3_is_rejected():
    from experiments.handheld_mapping.registration import register_clouds
    bad = np.eye(4)
    bad[0, 0] = 2
    result = register_clouds(corner_cloud(), corner_cloud(), bad)
    assert not result['accepted']
    assert result['reason'] == 'invalid_initial_transform'


def test_low_bidirectional_overlap_is_rejected():
    from experiments.handheld_mapping.registration import register_clouds
    source = corner_cloud()
    target = copy.deepcopy(source)
    for i in range(1, 8):
        target += copy.deepcopy(source).translate([i*2, 0, 0])
    result = register_clouds(source, target, np.eye(4))
    assert not result['accepted']
    assert result['reason'] == 'low_overlap'


def test_initial_prior_cannot_be_corrected_more_than_twenty_degrees():
    from experiments.handheld_mapping.registration import register_clouds
    source = corner_cloud()
    initial = np.eye(4)
    initial[:3, :3] = Rotation.from_euler('z', 30, degrees=True).as_matrix()
    result = register_clouds(source, source, initial)
    assert not result['accepted']
    assert result['reason'] == 'prior_correction_exceeded'
