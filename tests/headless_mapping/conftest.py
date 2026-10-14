# ================================================================
# conftest.py
# 설명: 실기 도구를 테스트하기 위한 합성 보정값을 제공한다. 실제 센서 보정에는 쓰지 않는다.
# 사용법: /usr/bin/python3 -m pytest tests/headless_mapping
# ================================================================
import copy
import math
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/orbslam3_ros2/scripts'))

@pytest.fixture
def calibration() -> dict:
    info = {'width': 640, 'height': 480, 'k': [377.186, 0, 320.542, 0, 377.186, 241.732, 0, 0, 1],
            'd': [0, 0, 0, 0, 0], 'distortion_model': 'plumb_bob'}
    stereo = np.eye(4)
    stereo[0, 3] = 0.0499182
    c, s = math.cos(math.radians(35)), math.sin(math.radians(35))
    base_c1 = np.eye(4)
    base_c1[:3, :3] = [[0, -s, c], [-1, 0, 0], [0, -c, -s]]
    return {'left': copy.deepcopy(info), 'right': copy.deepcopy(info),
            'T_c1_c2': stereo.tolist(), 'T_c1_base': np.linalg.inv(base_c1).tolist(),
            'pitch_down_deg': 35., 'extrinsic_source': 'urdf_provisional',
            'extrinsic_verified': False, 'camera_profile': '640x480x30:Y8',
            'imu_clock_verified': False, 'vocabulary_sha256': 'fixture-only'}

@pytest.fixture
def urdf(tmp_path: Path) -> Path:
    path = tmp_path / 'fixture.urdf'
    path.write_text('<robot name="fixture"><link name="base_link"/></robot>')
    return path
