# ================================================================
# test_reference.py
# 설명: 지도 정렬·crop·저장과 불확실한 중력/병합 거부를 합성 입력으로 검증한다.
# 사용법: python -m pytest tests/handheld_mapping/test_reference.py
# ================================================================
import copy
import json
from pathlib import Path

import numpy as np
import open3d as o3d
import pytest

from experiments.handheld_mapping.reference import (
    ReferenceMap, load_source, manual_gravity, point_mask, rigid_pose, estimate_gravity,
)


def cloud(points: list) -> o3d.geometry.PointCloud:
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(np.asarray(points, float))
    return result


def model() -> ReferenceMap:
    poses = np.repeat(np.eye(4)[None], 5, axis=0)
    poses[:, :3, 3] = [[0, 0, 0], [.2, 0, .1], [2, 0, 1], [.4, 0, .2], [.5, 0, .25]]
    return ReferenceMap(cloud([[0, 0, 0], [.2, 0, .1], [.4, 0, .2], [2, 0, 1]]),
                        poses=poses, stamps=np.array([0, .1, .2, .3, 4.]), map_id=3)


def test_manual_gravity_preserves_slope_and_step_lengths() -> None:
    value = model()
    initial = np.asarray(value.cloud.points).copy()
    result = manual_gravity([0, -1, 0])
    value.apply_transform(result['transformation'], result)
    aligned = np.asarray(value.geometry()[0].points)
    assert np.allclose(np.linalg.norm(np.diff(initial, axis=0), axis=1),
                       np.linalg.norm(np.diff(aligned, axis=0), axis=1))
    assert result['verified'] is False
    assert np.allclose(np.asarray(result['transformation'])[:3, :3] @ [0, -1, 0], [0, 0, 1])
    assert np.ptp(aligned[:, 2]) == pytest.approx(0)


def test_crop_splits_path_and_height_preserves_original() -> None:
    value = model()
    original = np.asarray(value.cloud.points).copy()
    value.set_crop([[-.1, -.1], [.6, -.1], [.6, .1], [-.1, .1]], -.1, .5)
    kept, _ = value.geometry()
    assert len(kept.points) == 3
    assert [len(segment) for segment in value.trajectory_segments()] == [2, 1, 1]
    assert np.array_equal(value.cloud.points, original)
    assert point_mask(np.array([[0, 0, 0], [1, 1, 0]]), value.crop).tolist() == [True, False]


def test_save_reload_no_double_transform_and_source_immutable(tmp_path: Path) -> None:
    path = tmp_path/'source.ply'
    o3d.io.write_point_cloud(str(path), model().cloud)
    original = path.read_bytes()
    value = load_source(path)
    transform = rigid_pose(1, 2, 3, 40, 5, 10)
    value.apply_transform(transform, manual_gravity([0, 0, 1]))
    value.set_start([1, 2, 3], [2, 3, 3])
    destination = tmp_path/'version1'
    value.save(destination)
    loaded = load_source(destination)
    assert np.allclose(loaded.geometry()[0].points, value.geometry()[0].points)
    assert np.allclose(loaded.reference_transform, transform)
    assert loaded.start_pose['provenance'] == 'manual_initial_prior_not_measured'
    assert loaded.metadata()['units'] == 'm'
    assert path.read_bytes() == original
    assert (destination/'original'/'cloud.ply').exists()
    with pytest.raises(FileExistsError):
        value.save(destination)


def test_mesh_crop_keeps_only_triangles_with_all_vertices_inside() -> None:
    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector([[0, 0, 0], [.1, 0, 0], [0, .1, 0], [2, 2, 0]])
    mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2], [1, 2, 3]])
    value = ReferenceMap(cloud(np.asarray(mesh.vertices)), mesh=mesh)
    value.set_crop([[-1, -1], [1, -1], [1, 1], [-1, 1]], -1, 1)
    assert len(value.geometry()[1].triangles) == 1
    assert value.metadata()['crop']['mesh_boundary_policy'] == 'all_vertices_inside'


def test_source_requires_explicit_multi_map_and_safe_paths(tmp_path: Path) -> None:
    o3d.io.write_point_cloud(str(tmp_path/'cloud.ply'), model().cloud)
    summary = {'paths_relative_to': 'summary', 'maps': [
        {'map_id': 2, 'cloud': 'cloud.ply'}, {'map_id': 3, 'cloud': 'cloud.ply'}]}
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='map'):
        load_source(tmp_path)
    assert load_source(tmp_path, map_id=3).map_id == 3
    summary['maps'][1]['cloud'] = '../cloud.ply'
    (tmp_path/'summary.json').write_text(json.dumps(summary))
    with pytest.raises(ValueError, match='path'):
        load_source(tmp_path, map_id=3)


def test_legacy_run_without_rgb_reads_trajectory_and_map_reset(tmp_path: Path) -> None:
    (tmp_path/'dense').mkdir()
    (tmp_path/'orb').mkdir()
    o3d.io.write_point_cloud(str(tmp_path/'dense/map_3_cloud.ply'), model().cloud)
    (tmp_path/'dense/summary.json').write_text(json.dumps({'maps': [{'map_id': 3, 'cloud': 'dense/map_3_cloud.ply'}]}))
    (tmp_path/'orb/map_3_keyframes.csv').write_text('map_id,source_stamp,x,y,z,qx,qy,qz,qw\n3,0,0,0,0,0,0,0,1\n3,.1,.2,0,.1,0,0,0,1\n3,5,.4,0,.2,0,0,0,1\n')
    value = load_source(tmp_path, map_id=3)
    assert value.path_kind == 'camera'
    assert [len(s) for s in value.trajectory_segments()] == [2, 1]
    assert value.original_start is not None


def test_gravity_refuses_missing_provenance(tmp_path: Path) -> None:
    result = estimate_gravity(tmp_path, model())
    assert result['accepted'] is False
    assert result['verified'] is False
    assert result['reason']


def test_merge_reject_preserves_reference_and_accepted_union(tmp_path: Path) -> None:
    reference, candidate = model(), model()
    before = np.asarray(reference.geometry()[0].points).copy()
    original = np.asarray(candidate.cloud.points).copy()
    rejected = reference.preview_merge(candidate, np.eye(4), registrar=lambda *a, **k: {'accepted': False, 'reason': 'plane'})
    with pytest.raises(ValueError, match='accepted'):
        reference.merged(candidate, rejected)
    assert np.array_equal(reference.geometry()[0].points, before)
    accepted = reference.preview_merge(candidate, np.eye(4), registrar=lambda *a, **k: {'accepted': True, 'transformation': rigid_pose(2, 0, 0, 0).tolist()})
    merged = reference.merged(candidate, accepted)
    assert len(merged.geometry()[0].points) == len(before)*2
    assert np.array_equal(reference.geometry()[0].points, before)
    assert np.array_equal(candidate.cloud.points, original)
    assert merged.metadata()['merge']['method'] == 'registered_geometry_union_not_joint_tsdf'
    assert len(merged.trajectory_segments()) >= 2


def test_non_rigid_and_unconfirmed_stl_rejected(tmp_path: Path) -> None:
    value = model()
    matrix = np.eye(4)
    matrix[0, 0] = 2
    with pytest.raises(ValueError):
        value.apply_transform(matrix)
    path = tmp_path/'mesh.stl'
    o3d.io.write_triangle_mesh(str(path), o3d.geometry.TriangleMesh.create_box().compute_vertex_normals())
    with pytest.raises(ValueError, match='meter'):
        load_source(path)
    assert load_source(path, units_confirmed=True).path_kind == 'none'


def test_static_gravity_uses_camera_orientation_and_rejects_motion_and_gaps() -> None:
    from experiments.handheld_mapping.reference import static_gravity
    times = np.arange(101)*.02
    gyro = np.zeros((101, 3))
    acc = np.tile([0., 0., 9.80665], (101, 1))
    poses = np.repeat(rigid_pose(0, 0, 0, 0, 30)[None], 101, axis=0)
    result = static_gravity(times, gyro, acc, times, poses, np.eye(4))
    expected = poses[0, :3, :3] @ [0, 0, 1]
    assert np.allclose(np.asarray(result['transformation'])[:3, :3] @ expected, [0, 0, 1])
    assert result['quality']['static_samples'] == 101
    with pytest.raises(ValueError, match='static'):
        static_gravity(times, gyro + .1, acc, times, poses, np.eye(4))
    with pytest.raises(ValueError, match='static'):
        static_gravity(times*10., gyro, acc, times*10., poses, np.eye(4))
    with pytest.raises(ValueError, match='orientations'):
        static_gravity(times, gyro, acc, times+10., poses, np.eye(4))


def test_crop_path_and_metadata_persist_reload(tmp_path: Path) -> None:
    value = model()
    value.set_crop([[-.1, -.1], [.6, -.1], [.6, .1], [-.1, .1]], -.1, .5)
    value.save(tmp_path/'cropped')
    loaded = load_source(tmp_path/'cropped')
    assert loaded.trajectory_segments() == value.trajectory_segments()
    assert loaded.metadata()['crop'] == value.metadata()['crop']
    loaded.apply_transform(rigid_pose(1, 0, 0, 0))
    assert [len(segment) for segment in loaded.trajectory_segments()] == [2, 1, 1]


def test_explicit_and_global_summaries_use_correct_variant_and_pose(tmp_path: Path) -> None:
    run = tmp_path/'run'
    (run/'orb_offline').mkdir(parents=True)
    header = 'map_id,source_stamp,x,y,z,qx,qy,qz,qw\n'
    (run/'orb_offline/map_3_keyframes.csv').write_text(header + '3,1,0,0,0,0,0,0,1\n')
    job = tmp_path/'job'
    for variant, x in [('baseline', 0.), ('optimized', 1.)]:
        (job/variant).mkdir(parents=True)
        geometry = cloud([[x, 0, 0], [x+.1, .1, .1]])
        o3d.io.write_point_cloud(str(job/variant/'cloud.ply'), geometry)
        (job/variant/'summary.json').write_text(json.dumps({'paths_relative_to': 'summary', 'maps': [{'map_id': 3, 'cloud': 'cloud.ply'}]}))
    (job/'keyframes.csv').write_text(header + '3,1,1,0,0,0,0,0,1\n')
    (job/'summary.json').write_text(json.dumps({'source_run': str(run), 'backend': 'orb_offline',
                                             'baseline_summary': 'baseline/summary.json', 'optimized_summary': 'optimized/summary.json'}))
    optimized, baseline = load_source(job), load_source(job, variant='baseline')
    assert optimized.poses[0, 0, 3] == 1.
    assert baseline.poses[0, 0, 3] == 0.
    assert np.asarray(optimized.cloud.points)[0, 0] == 1.
    assert np.asarray(baseline.cloud.points)[0, 0] == 0.


def test_mesh_processing_preview_preserves_original_and_does_not_repeat_reload(tmp_path: Path) -> None:
    mesh = o3d.geometry.TriangleMesh.create_sphere(resolution=12).compute_vertex_normals()
    value = ReferenceMap(cloud(np.asarray(mesh.vertices)), mesh=mesh)
    original = np.asarray(value.mesh.vertices).copy()
    value.set_processing(max_triangles=100, smooth_iterations=2)
    result = value.geometry()[1]
    assert len(result.triangles) <= 100
    assert np.array_equal(value.mesh.vertices, original)
    value.save(tmp_path/'processed')
    loaded = load_source(tmp_path/'processed')
    assert np.allclose(loaded.geometry()[1].vertices, result.vertices)
    assert loaded.metadata()['processing']['smooth_iterations'] == 2
    with pytest.raises(ValueError):
        value.set_processing(smooth_iterations=11)


def gravity_fixture(run: Path) -> ReferenceMap:
    from experiments.handheld_mapping.reference import file_hash
    run.mkdir()
    cal = {'version': 1, 'sensor': 'ISM330DHCX', 'frame': 'rover_body', 'acc_unit': 'g', 'gyro_unit': 'deg/s',
           'chip_to_body': [[0, 1], [1, 1], [2, 1]], 'gyro_bias_dps': [0., 0., 0.],
           'acc_offset_g': [0., 0., 0.], 'acc_scale': [1., 1., 1.]}
    (run/'imu_calibration.json').write_text(json.dumps(cal))
    (run/'robot.urdf').write_text('''<robot>
    <joint name="base_to_imu" type="fixed"><parent link="base_link"/><child link="imu_link"/><origin xyz="0 0 0"/></joint>
    <joint name="base_to_camera" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz="0 0 0"/></joint>
    <joint name="camera_to_left_optical_frame" type="fixed"><parent link="camera_link"/><child link="camera_left_optical_frame"/><origin xyz="0 0 0"/></joint></robot>''')
    manifest = {'calibration_sha256': file_hash(run/'imu_calibration.json'), 'urdf_sha256': file_hash(run/'robot.urdf'),
                'chip_to_body_actual': cal['chip_to_body'], 'chip_to_body_expected': cal['chip_to_body']}
    (run/'imu_manifest.json').write_text(json.dumps(manifest))
    (run/'capture_calibration.json').write_text(json.dumps({'stereo': {'source_camera_info': {'left': {'r': np.eye(3).ravel().tolist()}}}}))
    folder = run/'orb_inertial'
    folder.mkdir()
    names = ['imu_calibration.json', 'imu_manifest.json', 'robot.urdf', 'capture_calibration.json']
    hashes = {name: file_hash(run/name) for name in names}
    alignment = {'clock_verified': False, 'sensor_acquisition_time_verified': False,
                 'clock_domain': 'pi_monotonic_raw', 'clock_fit': {'scale': 1., 'receipt_residual_trend_suspect': False},
                 'imu_time_correction_s': 0., 'imu_time_source': 'mcu_mean_sample_read', 'input_sha256': hashes}
    (folder/'alignment.json').write_text(json.dumps(alignment))
    (folder/'summary.json').write_text(json.dumps({'T_imu_c1': np.eye(4).tolist(), 'input_sha256': hashes}))
    times = np.arange(101)*.02 + 1
    (folder/'camera_stamp_map.json').write_text(json.dumps({str(round(t*1e9)): round((t+100)*1e9) for t in times}))
    (folder/'imu.csv').write_text('timestamp_ns,gx_rad_s,gy_rad_s,gz_rad_s,ax_m_s2,ay_m_s2,az_m_s2\n' +
                                 ''.join(f'{round(t*1e9)},0,0,0,0,0,9.80665\n' for t in times))
    return ReferenceMap(cloud([[0, 0, 0], [.1, .1, .1]]), poses=np.repeat(np.eye(4)[None], len(times), axis=0),
                        stamps=times+100, map_id=0, source={'trajectory_source': 'synthetic', 'run': str(run)})


def test_experimental_gravity_candidate_requires_opt_in_and_reports_unverified_clock(tmp_path: Path) -> None:
    run = tmp_path/'run'
    value = gravity_fixture(run)
    assert not estimate_gravity(run, value)['accepted']
    before = value.local_transform.copy()
    result = estimate_gravity(run, value, allow_unverified_clock=True)
    assert result['accepted']
    assert result['verified'] is False
    assert result['quality']['clock_verified'] is False
    assert result['quality']['sensor_acquisition_time_verified'] is False
    assert np.array_equal(value.local_transform, before)
    alignment = json.loads((run/'orb_inertial/alignment.json').read_text())
    alignment.pop('clock_fit')
    (run/'orb_inertial/alignment.json').write_text(json.dumps(alignment))
    assert not estimate_gravity(run, value, allow_unverified_clock=True)['accepted']


def test_repeated_merge_retains_cropped_path_gaps(tmp_path: Path) -> None:
    a, b = model(), model()
    a.set_crop([[-.1, -.1], [.6, -.1], [.6, .1], [-.1, .1]], -.1, .5)
    b.set_crop([[-.1, -.1], [.6, -.1], [.6, .1], [-.1, .1]], -.1, .5)
    result = {'accepted': True, 'transformation': np.eye(4).tolist()}
    merged = a.merged(b, result)
    assert [len(segment) for segment in merged.trajectory_segments()] == [2, 1, 1, 2, 1, 1]
    merged.save(tmp_path/'merged')
    assert load_source(tmp_path/'merged').trajectory_segments() == merged.trajectory_segments()


def test_actual_registration_merge_known_transform_and_plane_rejection() -> None:
    from test_registration import corner_cloud
    points = np.asarray(corner_cloud().points)
    target = ReferenceMap(cloud(points))
    matrix = rigid_pose(.12, -.07, .03, 8., 2., -3.)
    candidate = ReferenceMap(cloud(points).transform(np.linalg.inv(matrix)))
    before = np.asarray(target.cloud.points).copy()
    result = target.preview_merge(candidate, matrix)
    assert result['accepted'], result['reason']
    assert np.allclose(result['transformation'], matrix, atol=.003)
    merged = target.merged(candidate, result)
    assert np.allclose(np.asarray(merged.geometry()[0].points)[len(points):], before, atol=.003)
    assert np.array_equal(target.cloud.points, before)
    xx, yy = np.meshgrid(np.linspace(-.6, .6, 40), np.linspace(-.6, .6, 40))
    flat = ReferenceMap(cloud(np.c_[xx.ravel(), yy.ravel(), np.zeros(xx.size)]))
    assert not flat.preview_merge(flat, np.eye(4))['accepted']


def test_geometry_cache_cannot_be_modified_by_preview_consumer() -> None:
    value = model()
    a, _ = value.geometry()
    a.translate([100, 0, 0])
    assert np.asarray(value.geometry()[0].points)[0, 0] == 0.
    value.apply_transform(rigid_pose(1, 0, 0, 0))
    assert np.asarray(value.geometry()[0].points)[0, 0] == 1.


def test_reload_processed_geometry_treats_crop_as_provenance_only(tmp_path: Path) -> None:
    mesh = o3d.geometry.TriangleMesh.create_sphere(resolution=20).compute_vertex_normals()
    value = ReferenceMap(cloud(np.asarray(mesh.vertices)), mesh)
    value.set_crop([[-.7, -.7], [.5, -.7], [.5, .5], [-.7, .5]], -1, 1)
    value.set_processing(100, 1)
    expected_mesh = value.geometry()[1]
    value.save(tmp_path/'version')
    loaded = load_source(tmp_path/'version')
    # 저장된 형상에는 crop가 이미 적용됨. 후속 smoothing 경계에도 crop 재적용은 금지한다.
    assert np.allclose(loaded.geometry()[1].vertices, expected_mesh.vertices)
    assert np.array_equal(loaded.geometry()[1].triangles, expected_mesh.triangles)
    assert loaded.metadata()['crop'] == value.crop


def test_taubin_without_crop_or_decimation_removes_isolated_vertices() -> None:
    mesh = o3d.geometry.TriangleMesh.create_box()
    mesh.vertices = o3d.utility.Vector3dVector(np.vstack([np.asarray(mesh.vertices), [3, 3, 3]]))
    mesh.triangles = o3d.utility.Vector3iVector(np.vstack([np.asarray(mesh.triangles), [0, 0, 1]]))
    value = ReferenceMap(cloud(np.asarray(mesh.vertices)), mesh)
    value.set_processing(smooth_iterations=1)
    processed = value.geometry()[1]
    assert np.isfinite(np.asarray(processed.vertices)).all()
    assert len(processed.vertices) == 8
    assert len(processed.triangles) == 12
    assert len(value.mesh.vertices) == 9


def test_nonfinite_mesh_cannot_be_returned_or_saved(tmp_path: Path) -> None:
    mesh = o3d.geometry.TriangleMesh.create_box()
    value = ReferenceMap(cloud(np.asarray(mesh.vertices)), mesh)
    np.asarray(value.mesh.vertices)[0, 0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        value.geometry()
    with pytest.raises(ValueError, match='finite'):
        value.save(tmp_path/'bad')
    assert not (tmp_path/'bad/reference.json').exists()
