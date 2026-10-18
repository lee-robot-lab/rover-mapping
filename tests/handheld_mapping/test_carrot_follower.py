# ================================================================
# test_carrot_follower.py
# 설명: 경로 위 lookahead 당근 점, 급한 꼭짓점 gating, 꼭짓점·끝 근처 감속을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_carrot_follower.py -q
# ================================================================
import numpy as np
import pytest

from experiments.handheld_mapping.site_mission import dense_path


def path(*points):
    from experiments.handheld_mapping.carrot_follower import CarrotPath
    return CarrotPath(dense_path(np.array(points,float)))


def test_straight_carrot_is_lookahead_ahead_of_progress_point():
    p=path([0,0],[2,0])
    assert p.target(index=4,lookahead=.4)[0]==pytest.approx(p.arcs[4]+.4)


def test_carrot_is_clipped_to_path_end():
    p=path([0,0],[1,0])
    assert np.allclose(p.target(index=len(p.points)-3,lookahead=.5),[1,0])


def test_carrot_does_not_cut_a_sharp_corner():
    p=path([0,0],[1,0],[1,1])
    index=int(np.searchsorted(p.arcs,.7))
    assert np.allclose(p.target(index,lookahead=.8),[1,0])
    after=int(np.searchsorted(p.arcs,1.05))
    assert p.target(after,lookahead=.2)[1]>.2


def test_gentle_curve_is_not_a_corner():
    angle=np.linspace(0,np.pi/2,60)
    p=path(*np.c_[np.cos(angle),np.sin(angle)])
    assert p.corners==[]


def test_speed_factor_brakes_toward_corner_and_end_only():
    p=path([0,0],[2,0],[2,2])
    far=p.speed_factor(index=2,brake_m=.3,floor=.35)
    near=p.speed_factor(index=int(np.searchsorted(p.arcs,1.9)),brake_m=.3,floor=.35)
    assert far==1 and .35<=near<1
    end=p.speed_factor(index=len(p.points)-1,brake_m=.3,floor=.35)
    assert end==pytest.approx(.35)


def test_grid_quantisation_jitter_is_not_a_corner():
    x=np.arange(0,2,.025);y=np.where(np.arange(len(x))%2==0,0.,.02)
    from experiments.handheld_mapping.carrot_follower import CarrotPath
    assert CarrotPath(np.c_[x,y]).corners==[]


def test_corner_index_is_the_vertex_not_its_neighbours():
    p=path([0,0],[1,0],[1,1])
    assert len(p.corners)==1 and np.allclose(p.points[p.corners[0]],[1,0],atol=.03)


def test_cross_track_is_signed_left_positive_and_tangent_follows_the_path() -> None:
    p = path([0, 0], [2, 0], [2, 2])
    index = int(np.searchsorted(p.arcs, .5))
    error, tangent = p.cross_track(index, np.array([.5, .1]))
    assert error == pytest.approx(.1, abs=1e-6) and tangent == pytest.approx(0., abs=1e-6)       # 경로 왼쪽 10 cm
    error, _ = p.cross_track(index, np.array([.5, -.07]))
    assert error == pytest.approx(-.07, abs=1e-6)                                                  # 오른쪽은 음수
    index = int(np.searchsorted(p.arcs, 2.6))
    error, tangent = p.cross_track(index, np.array([1.9, .6]))
    assert tangent == pytest.approx(np.pi/2, abs=1e-6) and error == pytest.approx(.1, abs=1e-6)  # 위로 가는 구간, 진행 방향 왼쪽(−x)이 양수


def test_stanley_heading_adds_the_cross_track_correction_and_zero_gain_changes_nothing() -> None:
    from experiments.handheld_mapping.carrot_follower import stanley_heading
    assert stanley_heading(0., .1, speed=.1, gain=0.) == 0.
    left = stanley_heading(0., .1, speed=.1, gain=2.)
    assert left < 0 and stanley_heading(0., -.1, speed=.1, gain=2.) == pytest.approx(-left)      # 왼쪽에 있으면 오른쪽으로 꺾는다
    assert abs(stanley_heading(0., 5., speed=.1, gain=2.)) < np.pi/2+1e-9                          # 큰 오차도 90°를 넘지 않는다
