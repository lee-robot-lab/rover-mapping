# ================================================================
# test_stereo_depth_check.py
# 설명: 스테레오 깊이와 정답 벽 깊이의 검출률·편향·이상치 통계 계산을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_stereo_depth_check.py -q
# ================================================================
import numpy as np
import pytest

from experiments.site_sim.stereo_depth_check import depth_agreement


def test_agreement_reports_detection_bias_outliers_per_distance_bin() -> None:
    rng = np.random.default_rng(0)
    truth = np.r_[np.full(1000, .5), np.full(1000, 1.2), np.full(1000, 2.)]
    measured = truth*.95*(1+rng.normal(0, .02, truth.size))       # 5% 짧게 읽고 잡음 2%
    measured[:200] = np.nan                                         # 첫 구간 20% 결측
    measured[1000:1100] *= 1.5                                      # 두 번째 구간 10% 이상치
    rows = {row['bin']: row for row in depth_agreement(truth, measured)}
    assert rows['0.3-0.8']['detection'] == pytest.approx(.8, abs=.01)
    assert rows['0.8-1.5']['outlier_10pct'] == pytest.approx(.1, abs=.02)
    assert rows['1.5-3.1']['median_ratio'] == pytest.approx(.95, abs=.01) and rows['1.5-3.1']['robust_sigma'] == pytest.approx(.02, abs=.01)


def test_bins_without_enough_pixels_are_skipped() -> None:
    assert depth_agreement(np.full(5, 1.), np.full(5, 1.)) == []
