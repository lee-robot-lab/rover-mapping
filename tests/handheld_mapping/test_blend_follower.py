# ================================================================
# test_blend_follower.py
# 설명: 혼합 추종 법칙(ψ_ref 슬루, 몸 좌표 변환, 방향 슬루, 램프 게이트 히스테리시스, 후진)을 BlendFollower로 직접 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_blend_follower.py -q
# ================================================================
import math

import pytest

DT=.02


def make(**kwargs):
    from experiments.handheld_mapping.blend_follower import BlendFollower,BlendParams
    return BlendFollower(BlendParams(**kwargs))


def drive(follower,n=1,*,yaw=0.,travel_heading=0.,speed=.08,slope_deg=0.,retreat=False):
    out=None
    for _ in range(n):
        out=follower.step(yaw=yaw,travel_heading=travel_heading,speed=speed,slope_deg=slope_deg,dt=DT,retreat=retreat)
    return out


def direction(out):
    return math.atan2(out.vy,out.vx)


def wrap(value):
    return math.atan2(math.sin(value),math.cos(value))


def test_defaults_match_spec():
    from experiments.handheld_mapping.blend_follower import BlendParams
    p=BlendParams()
    assert (p.max_yaw_rate,p.yaw_gain,p.ramp_slope_deg,p.resume_deg,p.stop_deg,p.dir_rate,p.min_dir_speed,p.heading_hold)==(
        .09,2.,8.,15.,25.,1.,.01,False)


def test_psi_ref_slews_at_omega_max_and_yaw_rate_is_clipped():
    f=make()
    first=drive(f,yaw=0.,travel_heading=math.pi/2)
    assert first.psi_ref==0
    previous=first.psi_ref
    for _ in range(100):
        out=drive(f,yaw=0.,travel_heading=math.pi/2)
        assert out.psi_ref-previous<=.09*DT+1e-12 and abs(out.yaw_rate)<=.09+1e-12
        previous=out.psi_ref
    assert out.yaw_rate==pytest.approx(.09)


def test_body_velocity_is_world_direction_rotated_into_body():
    out=drive(make(),yaw=math.pi/2,travel_heading=0.)
    assert out.vx==pytest.approx(0,abs=1e-12) and out.vy==pytest.approx(-.08)
    assert math.hypot(out.vx,out.vy)==pytest.approx(.08)


def test_body_direction_slew_is_limited_to_1_rad_s():
    f=make();drive(f,yaw=0.,travel_heading=0.)
    previous=0.
    for k in range(85):
        out=drive(f,yaw=0.,travel_heading=math.pi/2)
        assert abs(direction(out)-previous)<=1.0*DT+1e-9
        previous=direction(out)
    assert direction(out)==pytest.approx(math.pi/2)


def test_direction_is_held_below_min_speed():
    f=make();drive(f,yaw=0.,travel_heading=0.);drive(f,200,yaw=0.,travel_heading=math.pi/2)
    settled=direction(drive(f,yaw=0.,travel_heading=math.pi/2))
    assert settled==pytest.approx(math.pi/2)
    out=drive(f,yaw=0.,travel_heading=-math.pi/2,speed=.005)
    assert direction(out)==pytest.approx(settled) and math.hypot(out.vx,out.vy)==pytest.approx(.005)


def test_yaw_wrap_at_pi_takes_short_way():
    f=make();yaw=math.pi-.01;heading=-math.pi+.01
    out=drive(f,yaw=yaw,travel_heading=heading)
    assert out.travel_gap_rad==pytest.approx(.02,abs=1e-9) and out.heading_gap_rad==0   # 재초기화 직후 ψ_ref=yaw
    out=drive(f,5,yaw=yaw,travel_heading=heading)
    assert 0<out.yaw_rate<=.09 and out.heading_gap_rad==pytest.approx(5*.09*DT)   # 2π−.02가 아니라 짧은 쪽으로 돈다
    assert direction(out)==pytest.approx(.02,abs=1e-9)   # 몸 좌표 방향도 짧은 쪽


def test_estimator_yaw_jump_keeps_output_bounded():
    f=make();drive(f,50,yaw=0.,travel_heading=0.)
    before=direction(drive(f,yaw=0.,travel_heading=0.))
    out=drive(f,yaw=math.pi/2,travel_heading=0.)
    assert all(map(math.isfinite,(out.vx,out.vy,out.yaw_rate,out.psi_ref)))
    assert abs(out.yaw_rate)<=.09 and abs(wrap(direction(out)-before))<=.02+1e-9


def test_gate_has_hysteresis_on_ramp():
    f=make();deg=math.radians
    out=drive(f,travel_heading=deg(20),slope_deg=10.)
    assert not out.gated and math.hypot(out.vx,out.vy)>0
    out=drive(f,travel_heading=deg(26),slope_deg=10.)
    assert out.gated and out.vx==out.vy==0 and out.yaw_rate>0
    out=drive(f,travel_heading=deg(20),slope_deg=10.)
    assert out.gated and out.vx==out.vy==0
    out=drive(f,travel_heading=deg(14),slope_deg=10.)
    assert not out.gated and math.hypot(out.vx,out.vy)>0
    assert not drive(make(),travel_heading=deg(60),slope_deg=5.).gated


def test_gated_direction_is_held_not_zeroed():
    f=make();deg=math.radians
    drive(f,50,travel_heading=deg(10),slope_deg=10.)
    before=direction(drive(f,travel_heading=deg(10),slope_deg=10.))
    assert drive(f,10,travel_heading=deg(40),slope_deg=10.).gated
    out=drive(f,travel_heading=deg(10),slope_deg=10.)   # 14° 이하로 돌아와 해제된 직후 방향이 끊기지 않는다
    assert not out.gated and abs(direction(out)-before)<=1.0*DT+1e-9


def test_retreat_keeps_psi_ref_at_yaw_and_drives_backward():
    out=drive(make(),yaw=.7,travel_heading=.7+math.pi,retreat=True,slope_deg=15.)
    assert out.yaw_rate==0 and out.psi_ref==pytest.approx(.7) and not out.gated
    assert out.vx==pytest.approx(-.08) and out.vy==pytest.approx(0,abs=1e-9)


def test_heading_hold_does_not_chase_travel_direction():
    f=make(heading_hold=True);drive(f,yaw=.3,travel_heading=2.)
    out=drive(f,50,yaw=.3,travel_heading=2.)
    assert out.psi_ref==.3 and out.yaw_rate==0


def test_invalidate_reinitializes_to_current_yaw():
    f=make();drive(f,100,yaw=0.,travel_heading=1.)
    f.invalidate()
    out=drive(f,yaw=1.,travel_heading=-1.)
    assert out.psi_ref==1. and direction(out)==pytest.approx(wrap(-2.))
