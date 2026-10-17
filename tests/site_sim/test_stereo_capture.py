# ================================================================
# test_stereo_capture.py
# 설명: stereo 시각 일치·원본 PNG·불완전 쌍 거부를 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_stereo_capture.py
# ================================================================
import csv
import json
import numpy as np
import pytest
from experiments.site_sim.stereo_capture import StereoCapture


def test_complete_pair_uses_identical_timestamp_and_keeps_gt_separate(tmp_path):
    c=StereoCapture(tmp_path/'stereo',{'width':8,'height':6})
    im=np.arange(6*8*3,dtype=np.uint8).reshape(6,8,3)
    c.write(0,im,im,[0,0,0],[1,0,0,0])
    c.write(100000000,im,im,[.01,0,0],[1,0,0,0])
    a=list(csv.reader((tmp_path/'stereo/mav0/cam0/data.csv').open()))
    b=list(csv.reader((tmp_path/'stereo/mav0/cam1/data.csv').open()))
    assert a==b and a[1][0]=='0' and a[2][0]=='100000000'
    gt=json.loads((tmp_path/'stereo/gt.jsonl').read_text().splitlines()[1])
    assert gt['stamp_ns']==100000000 and gt['pose_source']=='sim_ground_truth'
    with pytest.raises(ValueError):c.write(100000000,im,im,[0,0,0],[1,0,0,0])
    with pytest.raises(ValueError):c.write(200000000,im,im[:2],[0,0,0],[1,0,0,0])
    assert len((tmp_path/'stereo/mav0/cam0/data.csv').read_text().splitlines())==3
