# ================================================================
# test_visuals.py
# 설명: 직접 표시할 STL 형상에서 단위·삼각형 연결·잘린 파일 거부를 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_visuals.py
# ================================================================
import struct
import numpy as np
import pytest
from experiments.site_sim.visuals import read_binary_stl


def test_stl_preserves_metric_shape_and_shared_vertices(tmp_path):
    triangles=[[(0.,0.,0.),(.24,0.,0.),(0.,.17,0.)],
               [(.24,0.,0.),(.24,.17,0.),(0.,.17,0.)]]
    blob=b'\0'*80+struct.pack('<I',2)
    for triangle in triangles:
        blob+=struct.pack('<12fH',0.,0.,1.,*(v for p in triangle for v in p),0)
    path=tmp_path/'body.stl';path.write_bytes(blob)
    points,faces=read_binary_stl(path)
    assert len(points)==4
    assert np.allclose(points[faces],triangles)
    assert np.allclose(points.max(0),[.24,.17,0.])
    path.write_bytes(blob[:-1])
    with pytest.raises(ValueError,match='truncated'):read_binary_stl(path)
