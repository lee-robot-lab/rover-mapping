# ================================================================
# test_terrain_replay.py
# 설명: 기록 depth와 최종 pose의 좌표·단위·map 분리 및 지형 결과 저장을 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_terrain_replay.py -q
# ================================================================
import json

import numpy as np
import pytest

from test_core import make_run


def alignment() -> np.ndarray:
    matrix = np.diag([1., -1., -1., 1.])
    matrix[2, 3] = 2.
    return matrix


def test_replay_known_depth_pose_and_units(tmp_path):
    from experiments.handheld_mapping.terrain import GridSpec
    from experiments.handheld_mapping.terrain_replay import replay
    run = make_run(tmp_path/'run', (0, 7))
    output = tmp_path/'terrain'
    result = replay(run, output, 0, GridSpec((0., 2.5, -1., 1.), .1), alignment(),
                    depth_stride=1, min_observations=1)
    assert result['map_id'] == 0
    assert result['integrated_frames'] == 1
    assert result['status'] == 'complete'
    with np.load(output/'terrain.npz', allow_pickle=False) as arrays:
        assert np.nanmax(np.abs(arrays['height_m'])) < 1e-6
        assert arrays['cost'].dtype == np.uint8
        assert np.any(arrays['cost'] == 0)
        assert np.any(arrays['cost'] == 255)
        assert np.nanmedian(arrays['last_stamp_ns'][arrays['observed']]) == 1_000_000_000
    assert json.loads((output/'summary.json').read_text())['coordinate_system'] == 'user-aligned gravity frame; axis order y,x'
    with pytest.raises(FileExistsError):
        replay(run, output, 0, GridSpec((0., 2.5, -1., 1.), .1), alignment())


def test_bad_alignment_or_map_identity_refused(tmp_path):
    from experiments.handheld_mapping.terrain import GridSpec
    from experiments.handheld_mapping.terrain_replay import replay
    run = make_run(tmp_path/'run')
    spec = GridSpec((0., 3., -1., 1.), .1)
    with pytest.raises(ValueError):
        replay(run, tmp_path/'bad', 0, spec, np.diag([2., 1., 1., 1.]))
    keyfile = run/'orb/map_0_keyframes.csv'
    keyfile.write_text(keyfile.read_text().replace('0,0,1.0', '7,0,1.0'))
    with pytest.raises(ValueError, match='map'):
        replay(run, tmp_path/'badmap', 0, spec, alignment())


def test_missing_depth_match_fails_without_success_artifact(tmp_path):
    from experiments.handheld_mapping.terrain import GridSpec
    from experiments.handheld_mapping.terrain_replay import replay
    run = make_run(tmp_path/'run')
    keyfile = run/'orb/map_0_keyframes.csv'
    keyfile.write_text(keyfile.read_text().replace('0,0,1.0', '0,0,9.0'))
    with pytest.raises(ValueError, match='depth'):
        replay(run, tmp_path/'output', 0, GridSpec((0., 3., -1., 1.), .1), alignment())
    assert not (tmp_path/'output/summary.json').exists()


def test_depth_projection_obeys_stride_limits_and_zero_invalid(tmp_path):
    from experiments.handheld_mapping.terrain_replay import project_depth
    info = {'width': 4, 'height': 3, 'depth_scale_m': .001,
            'k': [2., 0., 1., 0., 2., 1., 0., 0., 1.], 'd': [0.]*5}
    depth = np.full((3, 4), 1000, np.uint16)
    depth[0, 0] = 0
    points = project_depth(depth, info, np.eye(4), stride=1, max_points=12)
    assert len(points) == 11
    assert np.allclose(points[:, 2], 1.)
    with pytest.raises(ValueError, match='points'):
        project_depth(depth, info, np.eye(4), stride=1, max_points=4)


def test_native_unordered_keyframes_are_sorted_without_changing_source(tmp_path):
    from experiments.handheld_mapping.terrain import GridSpec
    from experiments.handheld_mapping.terrain_replay import replay
    run = make_run(tmp_path/'run', (0, 0))
    keyfile = run/'orb/map_0_keyframes.csv'
    text = ('map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n'
            '0,2,2.0,1,0,0,0,0,0,1\n0,1,1.0,1,0,0,0,0,0,1\n')
    keyfile.write_text(text)
    result = replay(run, tmp_path/'terrain', 0, GridSpec((0., 2.5, -1., 1.), .1), alignment(), depth_stride=1)
    assert result['integrated_frames'] == 2
    assert keyfile.read_text() == text


def test_duplicate_keyframe_stamp_refused(tmp_path):
    from experiments.handheld_mapping.terrain import GridSpec
    from experiments.handheld_mapping.terrain_replay import replay
    run = make_run(tmp_path/'run')
    keyfile = run/'orb/map_0_keyframes.csv'
    keyfile.write_text(keyfile.read_text()+'0,1,1.0,1,0,0,0,0,0,1\n')
    with pytest.raises(ValueError, match='stamp'):
        replay(run, tmp_path/'terrain', 0, GridSpec((0., 2.5, -1., 1.), .1), alignment())
