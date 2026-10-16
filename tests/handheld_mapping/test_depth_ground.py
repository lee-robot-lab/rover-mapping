"""반복 관측으로 depth 잡음을 제거하고 실제 경사·장애물·미관측을 보존한다."""
import numpy as np
import pytest

from experiments.handheld_mapping.depth_filter import DepthFilter, multiview_filter, spatial_filter
from experiments.handheld_mapping.ground import GroundGrid
from experiments.handheld_mapping.terrain import GridSpec
from experiments.handheld_mapping.costmap import RoverLimits, build_costmap


def test_depth_filter_rejects_spikes_and_discontinuities_without_filling_holes():
    depth = np.full((30, 40), 1000, np.uint16)
    depth[10, 10] = 2500
    depth[20, 10] = 0
    depth[:, 25:] = 2000
    original = depth.copy()
    result, _ = spatial_filter(depth, .001, DepthFilter())
    assert result[10, 10] == result[20, 10] == 0
    assert not result[:, 24:26].any()
    assert result[5, 5] == 1000 and result[5, 30] == 2000
    np.testing.assert_array_equal(depth, original)


def test_multiview_removes_inconsistent_patch_and_accounts_for_camera_translation():
    info = {'k': [40., 0., 19.5, 0., 40., 14.5, 0., 0., 1.], 'depth_scale_m': .001}
    depth = np.full((30, 40), 2000, np.uint16)
    depth[8:15, 10:18] = 1500
    reference = np.full_like(depth, 2000)
    pose = np.eye(4)
    pose[0, 3] = .1
    filtered, stats = multiview_filter(depth, info, pose, [(reference, np.eye(4))], DepthFilter())
    assert not filtered[8:15, 10:18].any()
    assert filtered[5, 5] == 2000
    assert stats['reference_frames'] == 1
    fallback, stats = multiview_filter(depth, info, pose, [], DepthFilter())
    np.testing.assert_array_equal(fallback, depth)
    assert stats['reference_frames'] == 0


def scene(slope=.0):
    x, y = np.meshgrid(np.arange(.025, 3., .05), np.arange(.025, 2., .05))
    return np.c_[x.ravel(), y.ravel(), (slope*x+.03*y).ravel()]


def test_ground_preserves_ramp_and_separates_wall_in_same_xy_cells():
    points = scene(.3)
    wall = points[(points[:, 0] > 1.4) & (points[:, 0] < 1.6)
                  & (points[:, 1] > .8) & (points[:, 1] < 1.2)].copy()
    wall[:, 2] += .5
    terrain = GroundGrid(GridSpec((0., 3., 0., 2.), .1))
    for stamp in (1, 2, 3):
        terrain.update(np.r_[points, wall], stamp, [.1, 1., .5])
    layers = terrain.layers()
    assert layers['ground'].sum() > 500
    assert np.nanmedian(layers['slope_deg']) == pytest.approx(np.degrees(np.arctan(np.hypot(.3, .03))), abs=.1)
    assert layers['height_m'][10, 15] == pytest.approx(.3*1.55+.03*1.05, abs=.015)
    assert layers['obstacle_height_m'][10, 15] > .45
    cost = build_costmap(layers, .1, RoverLimits(safety_radius_m=0.))
    assert cost[10, 15] == 254
    assert cost[5, 5] < 254


def test_isolated_high_platform_has_no_invented_ground_and_oneoff_low_echo_does_not_win():
    points = scene()
    platform = points[:, 0] > 2.
    points[platform, 2] += .8
    terrain = GroundGrid(GridSpec((0., 3., 0., 2.), .1))
    for stamp in (1, 2, 3):
        echo = points[(points[:, 0] > .9) & (points[:, 0] < 1.1)].copy()
        echo[:, 2] -= .5
        terrain.update(np.r_[points, echo] if stamp == 1 else points, stamp, [.1, 1., .5])
    layers = terrain.layers()
    assert layers['ground'][10, 10]
    assert layers['height_m'][10, 10] == pytest.approx(.03*1.05, abs=.01)
    assert not layers['ground'][10, 27]
    assert np.isnan(layers['height_m'][10, 27])
    assert build_costmap(layers, .1, RoverLimits())[10, 27] == 255
    size = terrain.persistent_bytes
    terrain.update(points, 4, [.1, 1., .5])
    assert terrain.persistent_bytes == size


def test_no_seed_and_single_observation_stay_unknown():
    terrain = GroundGrid(GridSpec((0., 3., 0., 2.), .1))
    terrain.update(scene(), 1)
    terrain.update(scene(), 2)
    assert not terrain.layers()['ground'].any()
    assert np.all(build_costmap(terrain.layers(), .1, RoverLimits()) == 255)


def test_rendered_depth_ramp_survives_filter_and_ground_separation_at_changing_pitch():
    from scipy.spatial.transform import Rotation
    from experiments.handheld_mapping.terrain_replay import project_depth
    width, height = 320, 240
    info = {'width': width, 'height': height, 'depth_scale_m': .001,
            'k': [190., 0., 159.5, 0., 190., 119.5, 0., 0., 1.]}
    v, u = np.mgrid[:height, :width]
    rays = np.stack(((u-159.5)/190., (v-119.5)/190., np.ones_like(u)), axis=-1)
    slope = np.tan(np.deg2rad(15.))
    optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
    frames = []
    for i, body_pitch in enumerate((0., -8., -15.)):
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler('y', 20.+body_pitch, degrees=True).as_matrix()@optical
        pose[:3, 3] = [i*.1, 0., .5+i*.1*slope]
        direction = rays@pose[:3, :3].T
        denominator = direction[..., 2]-slope*direction[..., 0]
        z = np.divide(slope*pose[0, 3]-pose[2, 3], denominator,
                      out=np.zeros_like(denominator, dtype=float), where=np.abs(denominator) > 1e-8)
        depth = np.rint(np.where((z >= .25) & (z <= 3.), z*1000, 0)).astype(np.uint16)
        depth, _ = spatial_filter(depth, .001, DepthFilter())
        frames.append((depth, pose))
    terrain = GroundGrid(GridSpec((0., 3., -1., 1.), .05))
    for i, (depth, pose) in enumerate(frames):
        depth, _ = multiview_filter(depth, info, pose, [f for j, f in enumerate(frames) if j != i], DepthFilter())
        terrain.update(project_depth(depth, info, pose, stride=2), i+1, pose[:3, 3])
    layers = terrain.layers(2)
    assert np.count_nonzero(layers['valid']) > 500
    assert np.nanmedian(layers['slope_deg']) == pytest.approx(15., abs=.5)
