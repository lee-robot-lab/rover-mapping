# ================================================================
# test_terrain.py
# 설명: 알려진 지형의 경사·단차·빈 구역과 반복 갱신의 메모리 상한을 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_terrain.py -q
# ================================================================
import numpy as np
import pytest


def plane(slope_x: float = 0., slope_y: float = 0.) -> np.ndarray:
    x, y = np.meshgrid(np.arange(20)*.1+.05, np.arange(20)*.1+.05)
    return np.c_[x.ravel(), y.ravel(), (slope_x*x+slope_y*y).ravel()]


def grid():
    from experiments.handheld_mapping.terrain import GridSpec, TerrainGrid
    return TerrainGrid(GridSpec((0., 2., 0., 2.), .1))


def test_plane_slope_and_roughness_are_geometric_not_height_variation():
    terrain = grid()
    terrain.update(plane(.1, .2), 1)
    terrain.update(plane(.1, .2), 2)
    layers = terrain.layers()
    interior = np.s_[3:-3, 3:-3]
    assert np.allclose(layers['slope_deg'][interior], np.degrees(np.arctan(np.hypot(.1, .2))), atol=1e-4)
    assert np.nanmax(layers['roughness_m'][interior]) < 1e-6
    assert np.allclose(layers['heading_deg'][interior], np.degrees(np.arctan2(.2, .1)), atol=1e-4)
    assert layers['height_m'].dtype == np.float32
    assert np.all(layers['observations'] == 2)


def test_unknown_and_insufficient_observation_remain_unknown():
    from experiments.handheld_mapping.costmap import RoverLimits, build_costmap
    terrain = grid()
    terrain.update(plane(), 1)
    assert np.all(build_costmap(terrain.layers(), .1, RoverLimits()) == 255)
    terrain.update(plane()[plane()[:, 0] < 1.], 2)
    cost = build_costmap(terrain.layers(), .1, RoverLimits(safety_radius_m=0.))
    assert cost[5, 5] == 0
    assert cost[5, 15] == 255


def test_step_and_vertical_column_block_with_safety_clearance():
    from experiments.handheld_mapping.costmap import RoverLimits, build_costmap
    points = plane()
    points[points[:, 0] >= 1., 2] = .2
    terrain = grid()
    for stamp in (1, 2):
        terrain.update(points, stamp)
    cost = build_costmap(terrain.layers(), .1, RoverLimits(safety_radius_m=.11))
    assert cost[10, 9] == 254
    assert cost[10, 8] == 254
    assert cost[10, 4] == 0
    terrain = grid()
    column = plane()[[210]].copy()
    column[:, 2] = 1.
    for stamp in (1, 2):
        terrain.update(np.r_[plane(), column], stamp)
    assert build_costmap(terrain.layers(), .1, RoverLimits(safety_radius_m=0.))[10, 10] == 254


def test_rover_configuration_changes_cost_without_changing_terrain():
    from experiments.handheld_mapping.costmap import RoverLimits, build_costmap
    terrain = grid()
    for stamp in (1, 2):
        terrain.update(plane(.3), stamp)
    layers = terrain.layers()
    before = layers['height_m'].copy()
    assert build_costmap(layers, .1, RoverLimits(max_slope_deg=10., safety_radius_m=0.))[8, 8] == 254
    assert build_costmap(layers, .1, RoverLimits(max_slope_deg=30., safety_radius_m=0.))[8, 8] < 254
    assert np.array_equal(layers['height_m'], before)


def test_bounded_state_and_outside_points_are_reported():
    terrain = grid()
    initial = terrain.persistent_bytes
    points = np.r_[plane(), [[-1., 0., 0.], [3., 3., 0.], [np.nan, 0., 0.]]]
    for stamp in range(1, 101):
        terrain.update(points, stamp)
    assert terrain.persistent_bytes == initial
    assert initial <= 400*40
    assert terrain.stats['outside_points'] == 200
    assert terrain.stats['invalid_points'] == 100
    assert terrain.layers()['observations'][5, 5] == 100


def test_many_points_in_one_frame_are_one_observation_and_variance_is_frame_based():
    terrain = grid()
    terrain.update(np.repeat(plane(), 5, axis=0), 1)
    terrain.update(plane()+[0., 0., .02], 2)
    layers = terrain.layers()
    assert layers['observations'][5, 5] == 2
    assert abs(layers['height_m'][5, 5]-.01) < 1e-6
    assert abs(layers['variance_m2'][5, 5]-.0002) < 1e-7


@pytest.mark.parametrize('bounds,resolution', [((0, 10000, 0, 10000), .001), ((0, 0, 0, 2), .1), ((0, 2, 0, 2), 0.)])
def test_invalid_or_oversized_grid_refuses_allocation(bounds: tuple, resolution: float):
    from experiments.handheld_mapping.terrain import GridSpec, TerrainGrid
    with pytest.raises(ValueError):
        TerrainGrid(GridSpec(bounds, resolution, max_cells=1000))


def test_bad_stamp_and_oversized_update_refused():
    from experiments.handheld_mapping.terrain import GridSpec, TerrainGrid
    terrain = TerrainGrid(GridSpec((0., 2., 0., 2.), .1, max_points=400))
    terrain.update(plane(), 10)
    for stamp in (9, 10):
        with pytest.raises(ValueError, match='stamp'):
            terrain.update(plane(), stamp)
    with pytest.raises(ValueError, match='points'):
        terrain.update(np.repeat(plane(), 2, axis=0), 11)


@pytest.mark.parametrize('values', [{'max_slope_deg': 0.}, {'max_step_m': -1.}, {'safety_radius_m': float('nan')}])
def test_invalid_rover_limits(values: dict):
    from experiments.handheld_mapping.costmap import RoverLimits
    with pytest.raises(ValueError):
        RoverLimits(**values)
