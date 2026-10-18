# ================================================================
# test_rover_assets_signs.py
# 설명: apply_ksrc_signs가 joint axis의 부호만 KSRC 규약(조향 +z, 바퀴 +y)으로 바꾸고 위치·이름은 그대로 두는지, 그러면 시뮬 기하가 steering_sign +1, rolling_heading 0이 되는지 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_rover_assets_signs.py
# ================================================================
import math
from xml.etree import ElementTree as ET

import numpy as np
import pytest

from experiments.site_sim.rover_assets import SOURCE_URDF, apply_ksrc_signs, swerve_geometry

# 왼쪽 다리는 joint origin이 z로 180° 돌아 있어, 바퀴 local 축 +y가 base에서는 −y가 된다(시뮬 URDF의 실제 구조).
LEGS = (('fl', .1, .13, 3.14159265359, '0 0 -1', '0 1 0'), ('fr', .1, -.13, 0., '0 0 -1', '0 -1 0'),
        ('rl', -.1, .13, 3.14159265359, '0 0 -1', '0 1 0'), ('rr', -.1, -.13, 0., '0 0 -1', '0 -1 0'))


def urdf(steer_lower=-1.7, steer_upper=1.7) -> ET.Element:
    """base 기준 조향축 −z, 바퀴축 −y(시뮬 URDF처럼 KSRC와 반대 부호)를 가진 4모듈 최소 URDF."""
    parts = ['<robot name="r"><link name="base_link"/>']
    for name, x, y, yaw, steer_axis, wheel_axis in LEGS:
        parts.append(f'<link name="{name}_leg"/><link name="{name}_wheel"/>'
                     f'<joint name="{name}_steering_joint" type="revolute"><parent link="base_link"/><child link="{name}_leg"/>'
                     f'<origin xyz="{x} {y} 0" rpy="0 0 {yaw}"/><axis xyz="{steer_axis}"/><limit lower="{steer_lower}" upper="{steer_upper}"/></joint>'
                     f'<joint name="{name}_wheel_joint" type="continuous"><parent link="{name}_leg"/><child link="{name}_wheel"/>'
                     f'<origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="{wheel_axis}"/></joint>')
    return ET.fromstring(''.join(parts)+'</robot>')


def test_only_wrong_signs_are_flipped_and_geometry_becomes_ksrc_convention():
    root = urdf(-1.2, 1.7)
    before = {m['name']: m for m in swerve_geometry(root, .0475)['modules']}
    assert all(m['steering_sign'] == -1 for m in before.values())
    flipped = apply_ksrc_signs(root)
    assert sorted(flipped) == sorted([f'{n}_{kind}_joint' for n, *_ in LEGS for kind in ('steering', 'wheel')])
    after = {m['name']: m for m in swerve_geometry(root, .0475)['modules']}
    for name, module in after.items():
        assert module['steering_sign'] == 1 and module['rolling_heading_at_zero_rad'] == pytest.approx(0., abs=1e-9)
        assert np.allclose(module['wheel_axis_base_link_zero_pose'], [0, 1, 0], atol=1e-9)
        assert module['steering_pivot_base_link_m'] == before[name]['steering_pivot_base_link_m']                      # 위치 그대로
        assert (module['steering_joint'], module['wheel_joint']) == (before[name]['steering_joint'], before[name]['wheel_joint'])
    limit = root.find("joint[@name='fl_steering_joint']/limit")
    assert (float(limit.get('lower')), float(limit.get('upper'))) == pytest.approx((-1.7, 1.2))                         # 축을 뒤집으면 한계도 뒤집힌다
    assert apply_ksrc_signs(root) == []                                                                                 # 두 번째는 변화 없음


@pytest.mark.skipif(not SOURCE_URDF.is_file(), reason='source URDF missing')
def test_real_source_urdf_becomes_ksrc_convention_without_moving_anything():
    root = ET.parse(SOURCE_URDF).getroot()
    before = {m['name']: m for m in swerve_geometry(root, .0475)['modules']}
    apply_ksrc_signs(root)
    after = {m['name']: m for m in swerve_geometry(root, .0475)['modules']}
    assert set(after) == set(before) and len(after) == 4
    for name, module in after.items():
        assert module['steering_sign'] == 1 and abs(math.sin(module['rolling_heading_at_zero_rad'])) < 1e-9 and math.cos(module['rolling_heading_at_zero_rad']) > 0
        assert np.allclose(module['steering_pivot_base_link_m'], before[name]['steering_pivot_base_link_m'])
        assert module['steering_joint'] == before[name]['steering_joint']
