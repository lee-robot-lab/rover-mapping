# ================================================================
# test_tracking_review.py
# 설명: 평탄 영상과 작은 평행 이동 영상에서 특징점 유지율 계산을 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_tracking_review.py
# ================================================================
import cv2
import numpy as np
from experiments.site_sim.tracking_review import review


def test_blank_sequence_has_no_features_and_no_false_retention(tmp_path):
    d=tmp_path/'front_frames';d.mkdir();im=np.zeros((160,200),np.uint8)
    for t in (0,1):cv2.imwrite(str(d/f'{t}.png'),im)
    out=review(tmp_path,False)
    assert out['pairs_without_features']==1 and out['median_retention'] is None


def test_small_translation_preserves_features(tmp_path):
    d=tmp_path/'front_frames';d.mkdir();rng=np.random.default_rng(7)
    im=rng.integers(0,256,(240,320),dtype=np.uint8)
    shifted=cv2.warpAffine(im,np.float32([[1,0,1],[0,1,0]]),(320,240))
    cv2.imwrite(str(d/'0.png'),im);cv2.imwrite(str(d/'0.1.png'),shifted)
    out=review(tmp_path,True)
    assert out['median_seeds']>10 and out['median_retention']>.8
