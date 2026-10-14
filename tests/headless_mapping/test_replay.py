# ================================================================
# test_replay.py
# 설명: 재생 도구가 일치하는 원본 stereo만 허용하며 GT 없이 동작하는지 확인한다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping
# ================================================================
from pathlib import Path
import pytest
from replay_stereo_sequence import read_pairs

def fixture_sequence(tmp_path):
    for side in ('cam0','cam1'):
        root=tmp_path/'mav0'/side; (root/'data').mkdir(parents=True)
        (root/'data.csv').write_text('#timestamp,filename\n1,a.png\n2,b.png\n')
        for name in ('a.png','b.png'): (root/'data'/name).write_bytes(b'fixture')
    return tmp_path

def test_no_ground_truth_needed(tmp_path):
    assert len(read_pairs(fixture_sequence(tmp_path))) == 2

def test_mismatched_stamp_rejected(tmp_path):
    fixture_sequence(tmp_path)
    (tmp_path/'mav0/cam1/data.csv').write_text('1,a.png\n3,b.png\n')
    with pytest.raises(ValueError): read_pairs(tmp_path)
