# ================================================================
# test_texture_quilt.py
# 설명: 모래 텍스처 quilting의 크기·재현성·비반복성과 대비 조절을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_texture_quilt.py
# ================================================================
import numpy as np

from experiments.site_sim.texture_quilt import quilt, set_contrast


def source():
    return np.random.default_rng(1).integers(0,255,(256,256,3),dtype=np.uint8)


def test_quilt_has_requested_size_and_is_reproducible():
    a=quilt(source(),300,200,block=64,overlap=16,seed=3);b=quilt(source(),300,200,block=64,overlap=16,seed=3)
    assert a.shape==(200,300,3) and np.array_equal(a,b)
    assert not np.array_equal(a,quilt(source(),300,200,block=64,overlap=16,seed=4))


def test_quilt_does_not_repeat_with_the_source_period():
    out=quilt(source(),600,200,block=64,overlap=16,seed=0).astype(float)
    for shift in (64,128,256):
        assert np.abs(out[:,:-shift]-out[:,shift:]).mean()>40


def test_contrast_keeps_mean_and_scales_deviation():
    image=source();low=set_contrast(image,.5)
    assert np.allclose(low.reshape(-1,3).mean(0),image.reshape(-1,3).mean(0),atol=1.5)
    assert abs(low.std()/image.std()-.5)<.02
