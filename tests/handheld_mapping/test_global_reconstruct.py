# ================================================================
# test_global_reconstruct.py
# 설명: 전역 재구성의 보정·입력 보존·짧은 기록과 실패 상태를 검증한다.
# 사용법: python -m pytest tests/handheld_mapping/test_global_reconstruct.py -q
# ================================================================
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import pytest
from scipy.spatial.transform import Rotation

from test_registration import corner_cloud


def recorded_plane(path: Path, count: int = 10) -> Path:
    (path/'raw/depth').mkdir(parents=True)
    (path/'orb_offline').mkdir()
    info = {'width': 80, 'height': 60, 'k': [70, 0, 39.5, 0, 70, 29.5, 0, 0, 1],
            'depth_scale_m': .001, 'T_left_depth': np.eye(4).tolist(), 'depth_frame': 'depth', 'd': [0.]*5}
    (path/'capture_calibration.json').write_text(json.dumps({'depth': info}))
    with (path/'frames.jsonl').open('w') as frames, (path/'orb_offline/map_0_keyframes.csv').open('w') as poses:
        poses.write('map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n')
        for index in range(count):
            stamp = 1_000_000_000+index*100_000_000
            relative = f'raw/depth/{stamp}.png'
            cv2.imwrite(str(path/relative), np.full((60, 80), 750, np.uint16))
            frames.write(json.dumps({'stream': 'depth', 'stamp_ns': stamp, 'frame_id': 'depth', 'path': relative})+'\n')
            poses.write(f'0,{index},{stamp/1e9:.9f},0,0,0,0,0,0,1\n')
    return path


def hashes(path: Path) -> dict:
    return {str(file.relative_to(path)): hashlib.sha256(file.read_bytes()).hexdigest()
            for file in path.rglob('*') if file.is_file()}


def test_short_record_preserves_input_and_emits_both_identical_observation_sets(tmp_path):
    from experiments.handheld_mapping.global_reconstruct import run_global
    run = recorded_plane(tmp_path/'run')
    before = hashes(run)
    output = tmp_path/'result'
    result = run_global(run, output)
    assert result['status'] == 'complete'
    assert result['optimization_status'] == 'insufficient_fragments'
    assert result['accuracy_verified_on_real_camera'] is False
    assert result['gravity_alignment_verified'] is False
    assert result['baseline_summary'] == 'baseline/summary.json'
    baseline = json.loads((output/result['baseline_summary']).read_text())
    optimized = json.loads((output/result['optimized_summary']).read_text())
    assert baseline['maps'][0]['integrated_frames'] == optimized['maps'][0]['integrated_frames'] == 10
    original = list(csv.DictReader((run/'orb_offline/map_0_keyframes.csv').open()))
    updated = list(csv.DictReader((output/'keyframes.csv').open()))
    assert [row['source_stamp'] for row in updated] == [row['source_stamp'] for row in original]
    assert hashes(run) == before
    assert len(json.loads((output/'fragments/manifest.json').read_text())['fragments']) == 1
    with pytest.raises(FileExistsError):
        run_global(run, output)


def test_overlap_is_fused_once_and_planar_graph_stays_prior_only(tmp_path):
    from experiments.handheld_mapping.global_reconstruct import run_global
    run = recorded_plane(tmp_path/'run', 19)
    output = tmp_path/'result'
    result = run_global(run, output, fragment_size=8, fragment_overlap=2)
    assert result['optimization_status'] == 'prior_only'
    assert result['source_keyframes'] == result['optimized_keyframes'] == 19
    assert result['integrated_frames'] == 19
    manifest = json.loads((output/'fragments/manifest.json').read_text())
    assert len(manifest['fragments']) == 3
    assert manifest['fragments'][0]['keyframe_stamps_ns'][-2:] == manifest['fragments'][1]['keyframe_stamps_ns'][:2]
    edges = json.loads((output/'pose_graph.json').read_text())['edges']
    assert all(edge['kind'] == 'orb_prior' for edge in edges)


def test_graph_corrects_known_fragment_drift_and_anchors_first_node(tmp_path):
    from experiments.handheld_mapping.global_reconstruct import optimize_fragments
    fragments = []
    for index in range(3):
        anchor = np.eye(4)
        anchor[:3, 3] = [index*.08, index*.005, 0]
        anchor[:3, :3] = Rotation.from_euler('z', index*2, degrees=True).as_matrix()
        cloud_file = tmp_path/f'{index}.ply'
        # 같은 물리 관측에 ORB anchor drift만 추가한다.
        o3d.io.write_point_cloud(str(cloud_file), corner_cloud())
        fragments.append({'id': index, 'anchor_pose': anchor.tolist(), 'cloud': str(cloud_file),
                          'anchor_stamp_ns': index*1_000_000_000, 'segment': 0})
    poses, report = optimize_fragments(fragments, tmp_path, .02)
    assert report['optimization_status'] == 'optimized_unverified', report
    assert np.allclose(poses[0], np.eye(4), atol=1e-8)
    assert np.linalg.norm(poses[-1][:3, 3]) < .04
    assert report['accepted_depth_edges'] == 2


def test_correction_interpolation_is_rigid_and_preserves_timestamp_endpoints():
    from experiments.handheld_mapping.global_reconstruct import interpolate_correction
    first, last = np.eye(4), np.eye(4)
    last[:3, :3] = Rotation.from_euler('z', 20, degrees=True).as_matrix()
    last[:3, 3] = [.2, .1, 0]
    value = interpolate_correction([10, 30], [first, last], 20)
    assert np.allclose(value[:3, 3], [.1, .05, 0])
    assert Rotation.from_matrix(value[:3, :3]).as_euler('xyz', degrees=True)[2] == pytest.approx(10)
    assert np.allclose(value[:3, :3].T@value[:3, :3], np.eye(3))
    assert np.allclose(interpolate_correction([10, 30], [first, last], 0), first)
    assert np.allclose(interpolate_correction([10, 30], [first, last], 40), last)


def test_invalid_source_is_manifested_and_raw_is_preserved(tmp_path):
    from experiments.handheld_mapping.global_reconstruct import run_global
    run = recorded_plane(tmp_path/'run')
    before = hashes(run)
    output = tmp_path/'failed'
    with pytest.raises(ValueError, match='map'):
        run_global(run, output, map_id=2)
    assert json.loads((output/'summary.json').read_text())['status'] == 'failed'
    assert hashes(run) == before


def test_source_map_cannot_be_replaced_with_uninitialized_inertial_poses(tmp_path):
    from experiments.handheld_mapping.global_reconstruct import run_global
    run = recorded_plane(tmp_path/'run')
    (run/'orb_offline').rename(run/'orb_inertial')
    (run/'orb_inertial/summary.json').write_text('{"orb_fused":false,"exported_imu_initialized_maps":[]}')
    with pytest.raises(ValueError, match='initialized'):
        run_global(run, tmp_path/'result', backend='orb_inertial')


def test_graph_rejects_optimizer_escape_and_preserves_original_poses(tmp_path, monkeypatch):
    from experiments.handheld_mapping.global_reconstruct import optimize_fragments
    fragments = []
    for index in range(2):
        path = tmp_path/f'{index}.ply'
        o3d.io.write_point_cloud(str(path), corner_cloud())
        fragments.append({'id': index, 'anchor_pose': np.eye(4).tolist(), 'cloud': str(path), 'segment': 0})
    def escape(graph, *args):
        pose = np.asarray(graph.nodes[1].pose).copy()
        pose[0, 3] = 2.
        graph.nodes[1].pose = pose
    monkeypatch.setattr(o3d.pipelines.registration, 'global_optimization', escape)
    poses, report = optimize_fragments(fragments, tmp_path)
    assert report['optimization_status'] == 'optimization_rejected'
    assert all(np.allclose(pose, np.eye(4)) for pose in poses)


def test_baseline_survives_late_failure_as_partial_manifest(tmp_path, monkeypatch):
    from experiments.handheld_mapping import global_reconstruct
    run = recorded_plane(tmp_path/'run')
    output = tmp_path/'result'
    def fail(*args, **kwargs):
        raise RuntimeError('synthetic optimization failure')
    monkeypatch.setattr(global_reconstruct, 'optimize_fragments', fail)
    with pytest.raises(RuntimeError, match='synthetic'):
        global_reconstruct.run_global(run, output)
    result = json.loads((output/'summary.json').read_text())
    assert result['status'] == 'partial'
    assert result['baseline_available'] is True
    baseline = json.loads((output/'baseline/summary.json').read_text())
    assert (output/'baseline'/baseline['maps'][0]['mesh']).is_file()
    assert not (output/'optimized/summary.json').exists()


def test_fragment_cloud_uses_anchor_local_extrinsic_once(tmp_path):
    from experiments.handheld_mapping.global_reconstruct import run_global
    run = recorded_plane(tmp_path/'run', 3)
    calibration = json.loads((run/'capture_calibration.json').read_text())
    calibration['depth']['T_left_depth'][0][3] = .1
    (run/'capture_calibration.json').write_text(json.dumps(calibration))
    path = run/'orb_offline/map_0_keyframes.csv'
    path.write_text(path.read_text().replace(',0,0,0,0,0,0,1\n', ',2,0,0,0,0,0,1\n'))
    output = tmp_path/'result'
    run_global(run, output)
    fragment = o3d.io.read_point_cloud(str(output/'fragments/fragment_0000.ply'))
    assert np.median(np.asarray(fragment.points)[:, 0]) == pytest.approx(.1, abs=.02)
    baseline = json.loads((output/'baseline/summary.json').read_text())
    geometry = o3d.io.read_point_cloud(str(output/'baseline'/baseline['maps'][0]['cloud']))
    assert np.median(np.asarray(geometry.points)[:, 0]) == pytest.approx(2.1, abs=.02)


def test_raw_directory_symlink_cannot_escape_recording(tmp_path, monkeypatch):
    from experiments.handheld_mapping import global_reconstruct
    run_global = global_reconstruct.run_global
    def unexpected_fusion(*args, **kwargs):
        raise AssertionError("escaped raw input reached fusion")
    monkeypatch.setattr(global_reconstruct, "reconstruct", unexpected_fusion)
    run = recorded_plane(tmp_path/'run')
    external = tmp_path/'external_raw'
    (run/'raw').rename(external)
    (run/'raw').symlink_to(external, target_is_directory=True)
    with pytest.raises(ValueError, match='depth path'):
        run_global(run, tmp_path/'result')


def test_source_input_digest_includes_depth_bytes(tmp_path):
    from experiments.handheld_mapping.global_reconstruct import run_global
    run = recorded_plane(tmp_path/'run', 3)
    first = run_global(run, tmp_path/'first')
    image = run/'raw/depth/1000000000.png'
    cv2.imwrite(str(image), np.full((60, 80), 760, np.uint16))
    second = run_global(run, tmp_path/'second')
    assert first['source_input_sha256'] != second['source_input_sha256']


def test_rejected_graph_retains_proposed_corrections_for_diagnosis(tmp_path, monkeypatch):
    from experiments.handheld_mapping.global_reconstruct import optimize_fragments
    fragments = []
    for index in range(2):
        path = tmp_path/f'{index}.ply'
        o3d.io.write_point_cloud(str(path), corner_cloud())
        fragments.append({'id': index, 'anchor_pose': np.eye(4).tolist(), 'cloud': str(path), 'segment': 0})
    def escape(graph, *args):
        pose = np.eye(4)
        pose[0, 3] = 2.
        graph.nodes[1].pose = pose
    monkeypatch.setattr(o3d.pipelines.registration, 'global_optimization', escape)
    _, report = optimize_fragments(fragments, tmp_path)
    assert report['maximum_proposed_translation_correction_m'] == pytest.approx(2.)
    assert report['maximum_proposed_rotation_correction_deg'] == pytest.approx(0.)
    assert report['node_corrections'][1]['translation_correction_m'] == pytest.approx(2.)
    saved = json.loads((tmp_path/'pose_graph.json').read_text())
    assert saved['proposed_nodes'][1][0][3] == 2.
    assert saved['final_nodes'][1][0][3] == 0.
