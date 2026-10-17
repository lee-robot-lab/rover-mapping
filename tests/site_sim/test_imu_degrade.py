# ================================================================
# test_imu_degrade.py
# 설명: IMU 열화(평균·잡음·지연·끊김)의 시각 기준과 통계를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_imu_degrade.py
# ================================================================
import numpy as np
import pytest

from experiments.site_sim.imu_degrade import ACC_DENSITY, degrade


def ideal(n=2000):
    t = np.arange(1, n+1)*.005
    return t, np.column_stack([np.sin(t), np.zeros(n), np.zeros(n), np.zeros(n), np.zeros(n), np.full(n, 9.81)])


def test_average_uses_four_sample_windows_stamped_at_their_center() -> None:
    t, s = ideal()
    stamp, values = degrade(t, s, noise=False)
    assert len(stamp) == 500 and stamp[0] == pytest.approx(t[:4].mean())
    assert values[0, 0] == pytest.approx(s[:4, 0].mean())


def test_noise_matches_measured_density_and_is_reproducible() -> None:
    t, s = ideal(40000)
    a = degrade(t, s, seed=3)[1]
    assert np.array_equal(a, degrade(t, s, seed=3)[1])
    assert a[:, 5].std() == pytest.approx(ACC_DENSITY/np.sqrt(.02), rel=.05)


def test_delay_shifts_stamps_and_gaps_remove_rows() -> None:
    t, s = ideal()
    stamp, values = degrade(t, s, noise=False, delay_s=.047, gaps_s=((3., 4.),))
    base, _ = degrade(t, s, noise=False)
    assert stamp[0] == pytest.approx(base[0]+.047)
    assert not ((stamp >= 3.) & (stamp < 4.)).any() and np.diff(stamp).max() > .9


def test_rejects_bad_shapes_and_nonmonotonic_time() -> None:
    t, s = ideal()
    with pytest.raises(ValueError):
        degrade(t[::-1], s)
    with pytest.raises(ValueError):
        degrade(t, s[:, :5])
