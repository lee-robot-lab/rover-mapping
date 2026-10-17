# ================================================================
# test_sim_orb_yaml.py
# 설명: 시뮬 전용 ORB 특징점 설정(저대비 모래 대응)이 지정한 세 값만 바꾸고 나머지는 보존하는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_sim_orb_yaml.py -q
# ================================================================
from experiments.site_sim.server import sim_orb_yaml

BASE = '''%YAML:1.0
Camera.fx: 377.186
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
Viewer.KeyFrameSize: 0.05
'''


def test_only_feature_count_and_fast_thresholds_change() -> None:
    out = sim_orb_yaml(BASE).splitlines()
    assert 'ORBextractor.nFeatures: 2000' in out and 'ORBextractor.iniThFAST: 12' in out
    assert 'ORBextractor.minThFAST: 3' in out
    assert 'Camera.fx: 377.186' in out and 'ORBextractor.nLevels: 8' in out and 'ORBextractor.scaleFactor: 1.2' in out
    assert len(out) == len(BASE.splitlines())


def test_missing_key_is_an_error_not_a_silent_noop() -> None:
    import pytest
    with pytest.raises(ValueError):
        sim_orb_yaml(BASE.replace('ORBextractor.minThFAST: 7\n', ''))
