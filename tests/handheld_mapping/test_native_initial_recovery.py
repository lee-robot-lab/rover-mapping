# ================================================================
# test_native_initial_recovery.py
# 설명: 실제 D435 기록과 native 실행기로 초기 지도 복구·timeout·관측 정리의 저장 안전성을 검증한다.
# 사용법: ROVER_STEREO_RECOVERY_TEST_RUN=/absolute/RUN .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_native_initial_recovery.py -q
# ================================================================
import csv
from itertools import islice
import os
from pathlib import Path
import shlex
import shutil
import subprocess

import cv2
import numpy as np
import pytest

from experiments.handheld_mapping.compare import ROOT, runner_environment, vocabularies
from experiments.handheld_mapping.offline import iter_pairs, write_settings


@pytest.mark.parametrize('gap_frames,return_scene', [(61, 'known'), (61, 'unknown'), (241, 'known')])
def test_initial_keyframe_recovery_requires_matching_scene_and_respects_timeout(
        tmp_path: Path, gap_frames: int, return_scene: str) -> None:
    source = os.environ.get('ROVER_STEREO_RECOVERY_TEST_RUN')
    binary = ROOT/'outputs/handheld_mapping/orb3_comparison_build/rover_orb3_compare'
    if not source or not binary.is_file():
        pytest.skip('set ROVER_STEREO_RECOVERY_TEST_RUN and install native assets for integration test')
    run = Path(source).resolve()
    prefix = list(islice(iter_pairs(run), 120))
    assert len(prefix) == 120, 'at least 120 exact stereo pairs required'
    shutil.copy2(run/'capture_calibration.json', tmp_path/'capture_calibration.json')
    settings = write_settings(tmp_path, tmp_path)
    blank = tmp_path/'blank.png'
    image = cv2.imread(prefix[0][1], cv2.IMREAD_GRAYSCALE)
    assert image is not None
    assert cv2.imwrite(str(blank), np.zeros_like(image))
    left, right = prefix[0][1:]
    if return_scene == 'unknown':
        rng = np.random.default_rng(42)
        left, right = str(tmp_path/'unknown_left.png'), str(tmp_path/'unknown_right.png')
        for path in (left, right):
            assert cv2.imwrite(path, rng.integers(0, 256, image.shape, dtype=np.uint8))
    pairs = tmp_path/'pairs.tsv'
    with pairs.open('w') as stream:
        for stamp, a, b in prefix:
            stream.write(f'{stamp}\t{a}\t{b}\n')
        for index in range(1, gap_frames+46):
            stamp = prefix[-1][0]+index*66_666_667
            a, b = (str(blank), str(blank)) if index <= gap_frames else (left, right)
            stream.write(f'{stamp}\t{a}\t{b}\n')
    orb, line = vocabularies()
    with (tmp_path/'runner.log').open('w') as log:
        result = subprocess.run([str(binary), str(orb), str(line), str(settings), str(pairs), str(tmp_path)],
                                cwd=tmp_path, env=runner_environment(True), stdout=log,
                                stderr=subprocess.STDOUT, timeout=90)
    assert result.returncode == 0, (tmp_path/'runner.log').read_text()[-5000:]
    assert (tmp_path/'atlas.osa').is_file(), 'native outputs must remain inside the test directory'
    with (tmp_path/'timings.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 120+gap_frames+45
    assert all(row['state'] == '2' and row['map_id'] == '0' for row in rows[:120])
    assert all(row['state'] != '2' and row['map_id'] == '-1' for row in rows[120:120+gap_frames])
    returned = rows[120+gap_frames:]
    if return_scene == 'unknown':
        assert all(row['state'] != '2' for row in returned), 'unknown scene must not fabricate a recovered pose'
    elif gap_frames == 61:
        assert all(row['state'] == '2' and row['map_id'] == '0' for row in returned), \
            'known first keyframe must relocalize in the original map before timeout'
    else:
        first_valid = next(row for row in returned if row['state'] == '2')
        assert first_valid['map_id'] != '0', 'expired recovery must create a distinct map'


@pytest.fixture(scope='module')
def presave_probe(tmp_path_factory: pytest.TempPathFactory) -> Path:
    build = ROOT/'outputs/handheld_mapping/orb3_comparison_build'
    metadata = build/'CMakeFiles/rover_orb3_compare.dir'
    if not os.environ.get('ROVER_STEREO_RECOVERY_TEST_RUN') or not (metadata/'link.txt').is_file():
        pytest.skip('native CMake build and selected D435 recording required')
    flags = (metadata/'flags.make').read_text().splitlines()
    includes = shlex.split(next(line.split('=', 1)[1] for line in flags if line.startswith('CXX_INCLUDES =')))
    command = shlex.split((metadata/'link.txt').read_text())
    folder = tmp_path_factory.mktemp('native_presave_probe')
    binary = folder/'presave_probe'
    command[1] = str(ROOT/'tests/handheld_mapping/native/presave_probe.cpp')
    command[command.index('-o')+1] = str(binary)
    command[1:1] = ['-std=c++14', '-O1', '-g', '-march=native', '-DEIGEN_HAS_CXX17_OVERALIGN=0',
                    '-DHAVE_GLEW', '-DHAVE_EIGEN', *includes]
    result = subprocess.run(command, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout+result.stderr
    return binary


@pytest.mark.parametrize('mode', ['foreign', 'last', 'missing-keyframe'])
def test_native_presave_cleanup_handles_removed_observations(
        tmp_path: Path, presave_probe: Path, mode: str) -> None:
    run = Path(os.environ['ROVER_STEREO_RECOVERY_TEST_RUN']).resolve()
    _, left, right = next(iter_pairs(run))
    shutil.copy2(run/'capture_calibration.json', tmp_path/'capture_calibration.json')
    settings = write_settings(tmp_path, tmp_path)
    orb, _ = vocabularies()
    environment = runner_environment(True)
    # allocator가 해제한 set node를 채워 iterator 무효화가 조용히 지나가지 않게 한다.
    environment.update(MALLOC_PERTURB_='165', GLIBC_TUNABLES='glibc.malloc.tcache_count=0')
    result = subprocess.run([str(presave_probe), str(orb), str(settings), left, right, mode],
                            cwd=tmp_path, env=environment, capture_output=True, text=True, timeout=90)
    (tmp_path/'probe.log').write_text(result.stdout+result.stderr)
    assert result.returncode == 0, result.stdout[-1500:]+result.stderr[-1500:]
