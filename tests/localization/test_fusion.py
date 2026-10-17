# ================================================================
# test_fusion.py
# 설명: IMU 공백 처리와 엔코더 슬립 보정 계수 추정을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/localization/test_fusion.py
# ================================================================
import numpy as np
import pytest

from experiments.localization.fusion import SlipScale, propagate_interval


class RecordingFilter:
    """propagate와 imu_gap 호출만 기록하는 대역."""
    def __init__(self) -> None:
        self.calls = []

    def propagate(self, gyro, accel, dt) -> None:
        self.calls.append(('propagate', round(float(dt), 6)))

    def imu_gap(self, dt, twist=None) -> None:
        self.calls.append(('gap', round(float(dt), 6), twist))


def gapped_imu():
    imu = np.column_stack([np.r_[np.arange(0., 1., .02), np.arange(3., 4., .02)], np.zeros((100, 6))])
    imu[:, 6] = 9.81
    return imu


def test_imu_gap_is_not_bridged_by_interpolation() -> None:
    f = RecordingFilter()
    assert propagate_interval([f], gapped_imu(), .5, .6) is None
    assert f.calls == [('propagate', .1)]
    f = RecordingFilter()
    assert propagate_interval([f], gapped_imu(), 1.5, 1.6) == pytest.approx(.1)
    assert f.calls == [('gap', .1, None)]


def test_gap_passes_the_twist_only_to_the_filter_that_has_one() -> None:
    plain, encoder = RecordingFilter(), RecordingFilter()
    propagate_interval([plain, encoder], gapped_imu(), 1.5, 1.6, gap_twists=[None, (.05, 0., .1)])
    assert plain.calls == [('gap', .1, None)] and encoder.calls == [('gap', .1, (.05, 0., .1))]


def test_slip_scale_converges_to_the_speed_ratio_and_ignores_slow_samples() -> None:
    slip = SlipScale()
    assert slip.value == 1.
    for _ in range(300):
        slip.update(.071, .054)
        slip.update(.01, 5.)            # too slow to trust the ratio
    assert slip.value == pytest.approx(.054/.071, abs=1e-6)


def test_slip_scale_is_clipped_to_a_plausible_range() -> None:
    slip = SlipScale()
    for _ in range(50):
        slip.update(.1, .001)
    assert slip.value == .3


def test_reanchor_after_tracking_loss_even_when_orb_reuses_the_map_id() -> None:
    from experiments.localization.fusion import needs_reanchor
    assert needs_reanchor(False, True, '0', '0')        # 리셋된 ORB가 map id 0을 다시 씀
    assert needs_reanchor(True, True, '3', '4')         # id가 바뀜
    assert not needs_reanchor(True, True, '3', '3')     # 같은 map에서 계속 추적
    assert not needs_reanchor(True, False, '3', '3')    # 추적 상실 중에는 기준을 바꾸지 않음


def test_rejected_update_moves_the_baseline_only_when_refreshing() -> None:
    from experiments.localization.fusion import move_baseline

    class Stub:
        cloned = 0

        def clone_current(self) -> None:
            self.cloned += 1

    a, b, c = Stub(), Stub(), Stub()
    assert move_baseline(a, True, True) and a.cloned == 0          # 수락: clone은 필터가 이미 옮김
    assert move_baseline(b, False, True) and b.cloned == 1         # 거부 + refresh: 기준선을 현재로
    assert not move_baseline(c, False, False) and c.cloned == 0    # 거부 + 유지: 예전 방식
