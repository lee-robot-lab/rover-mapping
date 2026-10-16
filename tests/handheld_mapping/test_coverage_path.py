# ================================================================
# test_coverage_path.py
# 설명: 칠한 구역을 모두 도는 폐루프 경로 생성기의 성질(닫힘, 안전, 덮음, 장애물 회피, 구역 제한)을 합성·실제 gyusa 지도로 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_coverage_path.py
# ================================================================
from pathlib import Path

import numpy as np
import pytest

from experiments.handheld_mapping.coverage_path import GridMap, coverage_loop, load_nav2_map

RES = 0.02


def empty_room(width_m: float = 2.0, height_m: float = 1.6) -> GridMap:
    """바깥 벽만 막힌 직사각형 방."""
    free = np.ones((round(height_m / RES), round(width_m / RES)), bool)
    free[[0, -1], :] = False
    free[:, [0, -1]] = False
    return GridMap(free=free, resolution=RES, origin=(0.0, 0.0))


def test_empty_room_loop_is_closed_safe_and_covers_most_of_the_region() -> None:
    grid = empty_room()
    result = coverage_loop(grid, region=grid.free, lane=0.3, robot_radius=0.1, start=(0.3, 0.3))
    path = result["waypoints"][:, :2]
    assert np.allclose(path[0], path[-1], atol=0.03)
    assert grid.min_clearance(path) >= 0.1 - RES            # 벽에서 로버 반지름 이상 떨어진다.
    assert result["coverage"] >= 0.97
    assert len(path) > 50


def test_obstacle_is_avoided_and_both_sides_are_visited() -> None:
    grid = empty_room()
    grid.free[30:50, 40:60] = False                          # 가운데 0.4 m 사각 장애물
    result = coverage_loop(grid, region=grid.free, lane=0.3, robot_radius=0.1, start=(0.3, 0.3))
    path = result["waypoints"][:, :2]
    assert grid.min_clearance(path) >= 0.1 - RES
    assert result["coverage"] >= 0.85
    assert (path[:, 0] < 0.8).any() and (path[:, 0] > 1.2).any()


def test_region_limits_where_lanes_are_driven() -> None:
    grid = empty_room()
    region = np.zeros_like(grid.free)
    region[:, 50:] = True                                    # 오른쪽 절반(x ≥ 1.0 m)만 칠한다.
    result = coverage_loop(grid, region=region & grid.free, lane=0.3, robot_radius=0.1, start=(1.2, 0.3))
    path = result["waypoints"][:, :2]
    assert (path[:, 0] >= 1.0 - 0.15).all()                  # 시작도 구역 안이라 연결 경로가 구역 밖으로 나가지 않는다.
    assert result["coverage"] >= 0.85


def test_evenly_spaced_lanes_cover_the_empty_room_completely() -> None:
    grid = empty_room()
    result = coverage_loop(grid, region=grid.free, lane=0.3, robot_radius=0.1, start=(0.3, 0.3))
    assert result["coverage"] >= 0.99
    assert result["length_m"] <= 14.0                          # 전체 거리 상한(후보 중 가장 짧은 순서를 골랐는지 확인)


def test_map_border_counts_as_a_wall_even_when_the_array_has_no_blocked_cells() -> None:
    grid = GridMap(free=np.ones((80, 100), bool), resolution=RES, origin=(0.0, 0.0))
    result = coverage_loop(grid, region=grid.free, lane=0.3, robot_radius=0.1, start=(0.5, 0.5))
    path = result["waypoints"][:, :2]
    assert path.min() >= 0.1 - RES and path[:, 0].max() <= 2.0 - 0.1 + RES and path[:, 1].max() <= 1.6 - 0.1 + RES


def test_lane_phase_is_chosen_so_a_narrow_pocket_is_still_covered() -> None:
    grid = empty_room(width_m=2.0, height_m=1.6)
    grid.free[:30, :] = False
    grid.free[55:, :] = False                                # 폭 0.5 m 통로 하나만 남긴다(안전 여유 0.1 m → 중심 폭 0.3 m).
    result = coverage_loop(grid, region=grid.free, lane=0.4, robot_radius=0.1, start=(0.3, 0.8))
    assert result["coverage"] >= 0.95


def test_unreachable_region_is_reported_not_hidden() -> None:
    grid = empty_room()
    grid.free[:, 49:52] = False                              # 방을 둘로 가르는 벽
    region = np.zeros_like(grid.free)
    region[:, 55:] = True                                    # 시작(왼쪽)에서 못 가는 오른쪽만 칠한다.
    with pytest.raises(ValueError, match="도달"):
        coverage_loop(grid, region=region & grid.free, lane=0.3, robot_radius=0.1, start=(0.3, 0.3))


GYUSA = Path("/home/su/rover/outputs/integration/ksrc-20261016/ros/maps/arena_sand.yaml")


@pytest.mark.skipif(not GYUSA.exists(), reason="gyusa 모래 지도가 없다")
def test_gyusa_sand_arena_loop() -> None:
    grid = load_nav2_map(GYUSA)
    result = coverage_loop(grid, region=grid.free, lane=0.4, robot_radius=0.21, start=None)
    path = result["waypoints"][:, :2]
    assert grid.min_clearance(path) >= 0.21 - grid.resolution * 2
    assert np.allclose(path[0], path[-1], atol=0.05)
    assert result["coverage"] >= 0.8
