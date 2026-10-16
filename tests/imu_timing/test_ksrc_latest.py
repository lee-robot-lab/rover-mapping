# ================================================================
# test_ksrc_latest.py
# 설명: 최신 KSRC PI 필드와 기존 IMU timing 확장이 서로 잘못 해석되지 않는지 검사한다.
# 사용법: KSRC_LATEST_BASELINE=PATH .venv-mapping/bin/python -m pytest tests/imu_timing/test_ksrc_latest.py -q
# ================================================================
import ast
import ctypes
import importlib.util
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]


def decoder(path: Path):
    """SPI를 열지 않고 순수 decoder 모듈만 불러온다."""
    spec = importlib.util.spec_from_file_location('ksrc_decoder_fixture', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('flags,value,expected_offset,expected_pi', [
    (16, 240, None, 240),
    (16 | 4096, -7000, -7000, None),
    (16 | 4096, -32768, -32768, None),
    (16 | 4096, 32767, 32767, None),
    (4096, -7000, None, None),
])
def test_snapshot_decoder_discriminates_pi_and_timing(flags, value, expected_offset, expected_pi):
    nl = decoder(ROOT/'teleop/ksrc_snapshot/pi/common/nucleo_link.py')
    frame = bytearray(144)
    frame[:4] = bytes([0xA5, 0x5A, 2, 0x81])
    struct.pack_into('<H', frame, 6, flags)
    frame[139] = 4
    struct.pack_into('<H', frame, 140, value & 0xFFFF)
    struct.pack_into('<H', frame, 142, nl.crc16(frame[2:142]))
    result = nl.decode_spi_telemetry(frame)
    assert result['wpi_corr'] == expected_pi
    assert result['imu_mean_read_offset_us'] == expected_offset
    assert result['imu_timing_valid'] is (expected_offset is not None)
    assert result['imu_n'] == 4
    frame[140] ^= 1
    assert nl.decode_spi_telemetry(frame) is None


@pytest.fixture(scope='module')
def latest(tmp_path_factory):
    """최신 upstream의 필요한 파일만 복사해 생성 patch를 적용한다."""
    source = os.environ.get('KSRC_LATEST_BASELINE')
    if not source:
        pytest.skip('KSRC_LATEST_BASELINE required for upstream integration')
    source = Path(source)
    folder = tmp_path_factory.mktemp('latest_ksrc')
    patch = folder/'latest.patch'
    subprocess.run([sys.executable, str(ROOT/'experiments/imu_timing_patch/build_patch.py'),
                    str(source), '--profile', 'latest', '--output', str(patch)], check=True)
    target = folder/'patched'
    manifest = json.loads((ROOT/'experiments/imu_timing_patch/LATEST_BASELINE.json').read_text())
    for name in manifest['sha256']:
        path = target/name
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source/name, path)
    subprocess.run(['patch', '--batch', '--fuzz=0', '-p1', '-i', str(patch)], cwd=target, check=True,
                   capture_output=True)
    assert patch.read_bytes() == (ROOT/'experiments/imu_timing_patch/ksrc_imu_mean_time_latest.patch').read_bytes()
    return source, target


def test_latest_patch_preserves_config_residual_and_board_copies(latest):
    old_root, new_root = latest
    old = decoder(old_root/'pi/common/nucleo_link.py')
    new = decoder(new_root/'pi/common/nucleo_link.py')
    snapshot = decoder(ROOT/'teleop/ksrc_snapshot/pi/common/nucleo_link.py')
    for module in (new, snapshot):
        assert module.encode_spi_cmd(.1, -.2, .3, 255) == old.encode_spi_cmd(.1, -.2, .3, 255)
        for enabled in (None, True, False):
            assert module.encode_spi_config(3, enabled, 240) == old.encode_spi_config(3, enabled, 240)
        assert module.encode_spi_res_cmd(.1, 0, .2, [.1]*4, [1.1]*4, 4) == old.encode_spi_res_cmd(.1, 0, .2, [.1]*4, [1.1]*4, 4)
    for suffix, board in (('c', 'Src'), ('h', 'Inc')):
        assert (new_root/f'firmware/common/spi_link.{suffix}').read_bytes() == (
            new_root/f'firmware/src2026-rover-firmware/Core/{board}/spi_link.{suffix}').read_bytes()


def test_latest_c_wire_roundtrip_discriminates_fields(latest):
    _, root = latest
    source = root/'probe.c'
    source.write_text('''/* 테스트용 host C probe: 실제 SPI 없이 pack/parse를 교차 검사한다. */
#include "spi_link.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
  spi_link_telemetry_t t={0}, out={0}; unsigned char frame[144];
  t.flags=16; t.wpi_corr=240; t.imu_n=4; t.imu_mean_read_offset_us=-7000;
  spi_link_pack_telemetry(&t,frame);
  assert(spi_link_parse_telemetry(frame,&out)==SPI_LINK_OK);
  assert(out.wpi_corr==240 && out.imu_mean_read_offset_us==0);
  assert(fwrite(frame,1,144,stdout)==144);
  t.flags|=SPI_TLM_FLAG_IMU_READ_TIME_VALID;
  spi_link_pack_telemetry(&t,frame);
  assert(spi_link_parse_telemetry(frame,&out)==SPI_LINK_OK);
  assert(out.imu_mean_read_offset_us==-7000 && out.wpi_corr==0);
  return fwrite(frame,1,144,stdout)==144?0:3;
}
''')
    binary = root/'probe'
    subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', '-fsanitize=undefined',
                    '-I', str(root/'firmware/common'), str(source),
                    str(root/'firmware/common/spi_link.c'), '-o', str(binary)], check=True)
    frames = subprocess.check_output([str(binary)])
    assert len(frames) == 288
    for path in (root/'pi/common/nucleo_link.py', ROOT/'teleop/ksrc_snapshot/pi/common/nucleo_link.py'):
        nl = decoder(path)
        legacy = nl.decode_spi_telemetry(frames[:144])
        timed = nl.decode_spi_telemetry(frames[144:])
        assert legacy['wpi_corr'] == 240 and legacy['imu_timing_valid'] is False
        assert timed['wpi_corr'] is None and timed['imu_mean_read_offset_us'] == -7000


def test_latest_ctypes_telemetry_matches_c_size_and_offset(latest):
    _, root = latest
    source = root/'size.c'
    source.write_text('''/* host C ABI 크기/offset probe. */
#include "spi_link.h"
#include <stddef.h>
#include <stdio.h>
int main(void) { printf("%zu %zu",sizeof(spi_link_telemetry_t),offsetof(spi_link_telemetry_t,imu_mean_read_offset_us)); }
''')
    binary = root/'size'
    subprocess.run(['cc', '-std=c99', '-I', str(root/'firmware/common'), str(source), '-o', str(binary)], check=True)
    size, offset = map(int, subprocess.check_output([str(binary)]).split())
    for name in ('pi/common/fake_nucleo.py', 'pi/common/test_spi_link.py'):
        tree = ast.parse((root/name).read_text())
        definition = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'CTlm')
        namespace = {'ctypes': ctypes}
        exec(compile(ast.Module(body=[definition], type_ignores=[]), name, 'exec'), namespace)
        kind = namespace['CTlm']
        assert ctypes.sizeof(kind) == size
        assert kind.imu_mean_read_offset_us.offset == offset
