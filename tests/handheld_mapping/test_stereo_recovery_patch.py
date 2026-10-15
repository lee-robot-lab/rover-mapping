# ================================================================
# test_stereo_recovery_patch.py
# 설명: 실제 patch의 C++ 설정 검증 코드를 OpenCV와 컴파일하고 stock 적용을 확인한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_stereo_recovery_patch.py -q
# ================================================================
from pathlib import Path
import shlex
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[2]
PATCH = ROOT / 'ros2_ws/src/orbslam3_ros2/patches/orbslam3_stereo_recovery_wait.patch'
STOCK = Path('/tmp/rover-native-mapping-deps/Tracking.cc.stock-stereo-recovery')


def test_native_stereo_recovery_setting_defaults_and_rejects_invalid_values(tmp_path: Path):
    assert PATCH.is_file(), 'stereo recovery patch required'
    if not shutil.which('c++') or not shutil.which('pkg-config'):
        pytest.skip('native compiler/OpenCV unavailable')
    flags = subprocess.run(['pkg-config', '--cflags', '--libs', 'opencv4'], capture_output=True, text=True)
    if flags.returncode:
        pytest.skip('OpenCV native development package unavailable')
    # 테스트용 복제 구현 대신 배포 patch에 추가된 실제 parser 본문을 컴파일한다.
    added = '\n'.join(line[1:] for line in PATCH.read_text().splitlines()
                      if line.startswith('+') and not line.startswith('+++'))
    start = added.index('double StereoRelocalizationWaitSeconds(')
    opening = added.index('{', start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (added[end] == '{') - (added[end] == '}')
        end += 1
    parser = added[start:end]
    source = tmp_path / 'probe.cc'
    source.write_text('''// ================================================================
// probe.cc
// 설명: patch의 실제 OpenCV YAML 검증 함수 실행용 임시 probe다.
// 사용법: ./probe settings.yaml
// ================================================================
#include <opencv2/core.hpp>
#include <cmath>
#include <stdexcept>
#include <iostream>
''' + parser + '''
int main(int argc, char** argv) {
    try {
        cv::FileStorage settings(argv[1], cv::FileStorage::READ);
        std::cout << StereoRelocalizationWaitSeconds(settings) << std::endl;
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << std::endl;
        return 2;
    }
}
''')
    binary = tmp_path / 'probe'
    subprocess.run(['c++', '-std=c++14', str(source), '-o', str(binary), *shlex.split(flags.stdout)],
                   check=True, capture_output=True, text=True, timeout=60)
    for value, expected in [(None, 3.), ('15', 15.), ('1', 1.), ('60', 60.), ('1.5', 1.5),
                            ('0', None), ('61', None), ('-1', None), ('.Nan', None),
                            ('.Inf', None), ('"15"', None), ('[15]', None)]:
        settings = tmp_path / 'settings.yaml'
        entry = 'Dummy: 1' if value is None else 'Tracking.StereoRelocalizationWaitSeconds: ' + value
        settings.write_text('%YAML:1.0\n---\n' + entry + '\n')
        result = subprocess.run([str(binary), str(settings)], capture_output=True, text=True, timeout=5)
        if expected is None:
            assert result.returncode == 2, (value, result.stdout, result.stderr)
            assert 'Tracking.StereoRelocalizationWaitSeconds' in result.stderr
        else:
            assert result.returncode == 0, (value, result.stderr)
            assert float(result.stdout) == pytest.approx(expected)


def test_patch_applies_to_exact_stock_tracking_source(tmp_path: Path):
    assert PATCH.is_file(), 'stereo recovery patch required'
    if not STOCK.is_file():
        pytest.skip('pinned official native source cache unavailable')
    target = tmp_path / 'src/Tracking.cc'
    target.parent.mkdir()
    target.write_bytes(STOCK.read_bytes())
    result = subprocess.run(['patch', '--dry-run', '-p1', '-i', str(PATCH)], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
