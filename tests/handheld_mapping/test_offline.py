# ================================================================
# test_offline.py
# 설명: bounded stereo 인덱스 재생과 sensors-only 노트북 후처리의 저장·실패를 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_offline.py -q
# ================================================================
import json
import hashlib
from pathlib import Path
import sys

import pytest

from test_core import make_run
from test_session import wait_state


def test_stream_pairing_is_exact_and_refuses_backwards_stamp(tmp_path):
    from experiments.handheld_mapping.offline import iter_pairs
    rows = [{'stream': stream, 'stamp_ns': stamp, 'path': f'raw/{stream}/{stamp}.png'}
            for stream, stamp in [('left', 1), ('right', 2), ('left', 3), ('right', 3)]]
    (tmp_path/'frames.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    pairs = list(iter_pairs(tmp_path))
    assert [pair[0] for pair in pairs] == [3]
    rows += [{'stream': 'left', 'stamp_ns': 1, 'path': 'raw/left/1.png'}]
    (tmp_path/'frames.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    with pytest.raises(ValueError, match='stamp'):
        list(iter_pairs(tmp_path))


def test_remote_command_selects_sensors_only_and_byte_limit():
    from experiments.handheld_mapping.controller import remote_command
    command = remote_command('pi@raspberrypi.local', 'fixture', sensors_only=True, queue_mib=2.)
    assert '--sensors-only' in command[-1]
    assert '--queue-mib 2.0' in command[-1]
    assert '--sensors-only' not in remote_command('pi@raspberrypi.local', 'fixture')[-1]
    assert '--projector-on' not in remote_command('pi@raspberrypi.local', 'fixture')[-1]
    assert '--projector-on' in remote_command('pi@raspberrypi.local', 'fixture', projector_on=True)[-1]
    with pytest.raises(ValueError):
        remote_command('pi@raspberrypi.local', 'fixture', queue_mib=float('nan'))


def test_tsdf_accepts_separate_laptop_backend(tmp_path):
    from experiments.handheld_mapping.reconstruct import reconstruct
    run = make_run(tmp_path/'run')
    (run/'orb').rename(run/'orb_offline')
    result = reconstruct(run, depth_max=3., backend='orb_offline')
    assert result['status'] == 'complete'
    assert (run/'dense_offline/map_0_mesh.ply').is_file()
    assert not (run/'dense').exists()


def sensor_worker() -> list[str]:
    # 실제 binary stream을 보내는 fixture이며 ROS/카메라의 동작을 모의하지 않는다.
    code = '''import sys,zlib,numpy as np
from experiments.handheld_mapping.protocol import write_packet
info={'width':80,'height':60,'k':[70,0,39.5,0,70,29.5,0,0,1],'depth_scale_m':.001,'T_left_depth':np.eye(4).tolist(),'depth_frame':'depth','d':[0]*5}
out=sys.stdout.buffer
write_packet(out,{'kind':'calibration','depth':info,'capture_mode':'sensors_only','source':'recorded_fixture'})
write_packet(out,{'kind':'ready'})
write_packet(out,{'kind':'image','stream':'depth','stamp_ns':1000000000,'width':80,'height':60,'step':160,'encoding':'16UC1','frame_id':'depth','compression':'zlib'},zlib.compress(np.full((60,80),800,np.uint16).tobytes()))
write_packet(out,{'kind':'finished','status':'saved','capture_mode':'sensors_only','dropped':0})
'''
    return [sys.executable, '-c', code]


def test_sensors_only_session_uses_laptop_orb_then_tsdf(tmp_path, monkeypatch):
    from experiments.handheld_mapping import offline
    from experiments.handheld_mapping.controller import Session
    calls = []
    def point(run: Path) -> dict:
        calls.append(run)
        (run/'orb_offline').mkdir()
        (run/'orb_offline/map_0_keyframes.csv').write_text(
            'map_id,keyframe_id,source_stamp,x,y,z,qx,qy,qz,qw\n0,0,1.0,0,0,0,0,0,0,1\n')
        return {'status': 'complete'}
    monkeypatch.setattr(offline, 'run_point', point)
    session = Session()
    session.start({'output_root': str(tmp_path), 'worker_command': sensor_worker()})
    result = wait_state(session, {'complete', 'partial', 'failed'})
    session.thread.join(timeout=5)
    assert result['state'] == 'complete', result
    assert result['source'] == 'recorded_fixture'
    run = Path(result['run_dir'])
    assert calls == [run]
    assert (run/'dense_offline/map_0_mesh.ply').is_file()


def test_offline_failure_preserves_raw_capture(tmp_path, monkeypatch):
    from experiments.handheld_mapping import offline
    from experiments.handheld_mapping.controller import Session
    def fail(run: Path) -> dict:
        raise RuntimeError('offline ORB fixture failure')
    monkeypatch.setattr(offline, 'run_point', fail)
    session = Session()
    session.start({'output_root': str(tmp_path), 'worker_command': sensor_worker()})
    result = wait_state(session, {'failed', 'complete'})
    session.thread.join(timeout=5)
    assert result['state'] == 'failed'
    assert 'offline ORB fixture failure' in result['error']
    assert list((Path(result['run_dir'])/'raw/depth').glob('*.png'))


def test_point_line_comparison_reuses_completed_laptop_baseline(tmp_path, monkeypatch):
    from experiments.handheld_mapping import compare
    native_root = tmp_path/'native_root'
    for relative in ('orbline_build/rover_orbline', 'orb3_comparison_build/rover_orb3_compare'):
        binary = native_root/'outputs/handheld_mapping'/relative
        binary.parent.mkdir(parents=True)
        binary.touch()
    monkeypatch.setattr(compare, 'ROOT', native_root)
    rows = [{'stream': side, 'stamp_ns': 100, 'path': f'raw/{side}/100.png'} for side in ('left', 'right')]
    (tmp_path/'frames.jsonl').write_text('\n'.join(json.dumps(row) for row in rows))
    k = [70., 0., 39.5, 0., 70., 29.5, 0., 0., 1.]
    camera = {'width': 80, 'height': 60, 'k': k, 'd': [0.]*5}
    stereo = {'left': camera, 'right': camera, 'T_c1_c2': [[1, 0, 0, .05], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
              'camera_profile': '80x60x15:Y8'}
    (tmp_path/'capture_calibration.json').write_text(json.dumps({'stereo': stereo}))
    point = tmp_path/'orb_offline'
    point.mkdir()
    pairs = compare.stereo_pairs(tmp_path)
    (point/'pairs.tsv').write_text(''.join(f'{stamp}\t{left}\t{right}\n' for stamp, left, right in pairs))
    (point/'runner_complete.json').write_text('{"status":"complete"}')
    (point/'summary.json').write_text(json.dumps({'input_calibration_sha256': hashlib.sha256(
        (tmp_path/'capture_calibration.json').read_bytes()).hexdigest()}))
    (point/'timings.csv').write_text('stamp_ns,state,map_id,process_ms\n100,2,0,1\n')
    calls = []
    def native(args: list, **kwargs):
        calls.append(args)
        folder = Path(args[-1])
        (folder/'runner_complete.json').write_text('{"status":"complete"}')
        (folder/'timings.csv').write_text('stamp_ns,state,map_id,process_ms\n100,2,0,2\n')
    monkeypatch.setattr(compare.subprocess, 'run', native)
    monkeypatch.setattr(compare, 'vocabularies', lambda: (Path('ORBvoc'), Path('LSvoc')))
    original_read = Path.read_text
    def bounded_read(path, *args, **kwargs):
        if path.name in ('frames.jsonl', 'pairs.tsv'):
            raise AssertionError('full recording/pair index read')
        return original_read(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', bounded_read)
    result = compare.run_orbline(tmp_path)
    assert len(calls) == 1
    assert result['same_input_point_baseline']['process_ms_median'] == 1.
