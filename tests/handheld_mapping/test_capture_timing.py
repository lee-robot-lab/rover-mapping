# ================================================================
# test_capture_timing.py
# 설명: 실제 파일·subprocess로 노트북 단계 계측과 종료 시 최종 큐 상태 보존을 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_capture_timing.py -q
# ================================================================
import json
from pathlib import Path
import sys
import time
import zlib

import cv2
import numpy as np
import pytest


@pytest.mark.parametrize('side,dtype,encoding', [
    ('left', np.uint8, 'mono8'), ('depth', np.uint16, '16UC1')])
def test_saved_frame_keeps_original_and_reports_slow_png_stage(
        tmp_path, monkeypatch, side: str, dtype, encoding: str) -> None:
    from experiments.handheld_mapping.storage import RunWriter
    original_write = cv2.imwrite

    def slow_write(*args, **kwargs):
        time.sleep(.02)
        return original_write(*args, **kwargs)

    monkeypatch.setattr(cv2, 'imwrite', slow_write)
    image = np.arange(12, dtype=dtype).reshape(3, 4)
    header = {'kind': 'image', 'stream': side, 'stamp_ns': 10_000_000_000,
              'width': 4, 'height': 3, 'step': image.strides[0], 'encoding': encoding,
              'compression': 'zlib', 'pi_receipt_monotonic_ns': 8_000_000_000_000_000_000,
              'laptop_receipt_monotonic_ns': time.monotonic_ns()}
    before = dict(header)
    writer = RunWriter(tmp_path/'run')
    try:
        decoded = writer.accept(header, zlib.compress(image.tobytes()))
        np.testing.assert_array_equal(decoded, image)
        stats = writer.image_timings[side]
    finally:
        writer.close()
    row = json.loads((tmp_path/'run/frames.jsonl').read_text())
    assert header == before
    assert row['stamp_ns'] == 10_000_000_000
    assert row['pi_receipt_monotonic_ns'] == 8_000_000_000_000_000_000
    assert row['laptop_image_saved_monotonic_ns'] >= row['laptop_receipt_monotonic_ns']
    assert row['laptop_decode_ms'] >= 0
    assert row['laptop_png_write_ms'] >= 15
    assert stats['images'] == 1
    assert stats['png_write_ms_max'] >= 15
    assert stats['decode_ms_total'] >= 0
    stored = cv2.imread(str(tmp_path/'run'/row['path']), cv2.IMREAD_UNCHANGED)
    np.testing.assert_array_equal(stored, image)


def test_final_drop_counter_survives_reconstruction_failure(tmp_path, monkeypatch) -> None:
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import reconstruct

    def fail_map(*args, **kwargs):
        raise RuntimeError('TSDF failure fixture')

    monkeypatch.setattr(reconstruct, 'reconstruct', fail_map)
    worker = '''
import sys
from experiments.handheld_mapping.protocol import write_packet
out = sys.stdout.buffer
write_packet(out, {'kind':'status','dropped':0,'queue':{'dropped':0,'queued_bytes':200}})
write_packet(out, {'kind':'finished','status':'saved','dropped':5,
                  'queue':{'dropped':5,'queued_bytes':0}})
'''
    session = Session()
    session._run({'output_root': str(tmp_path), 'worker_command': [sys.executable, '-c', worker]})
    result = session.snapshot()
    assert result['state'] == 'failed'
    assert 'TSDF failure fixture' in result['error']
    assert result['dropped'] == result['transport_queue']['dropped'] == 5
    saved = json.loads((Path(result['run_dir'])/'session.json').read_text())
    assert saved['dropped'] == 5


@pytest.mark.parametrize('new_worker', [False, True])
def test_finished_snapshot_and_laptop_times_survive_real_packet_session(
        tmp_path, monkeypatch, new_worker: bool) -> None:
    from experiments.handheld_mapping.controller import Session
    from experiments.handheld_mapping import reconstruct
    # 여기서는 실제 packet/storage 경로를 검사하며 TSDF 자체는 다른 실데이터 회귀가 담당한다.
    monkeypatch.setattr(reconstruct, 'reconstruct', lambda *args, **kwargs: {'status': 'complete'})
    worker = '''
import sys, zlib
from experiments.handheld_mapping.protocol import write_packet
out = sys.stdout.buffer
write_packet(out, {'kind':'ready'})
write_packet(out, {'kind':'status','source_frames':20,'dropped':3,
                   'queue':{'queued_bytes':922019},'capture_metrics':{'marker':1}})
write_packet(out, {'kind':'image','stream':'left','stamp_ns':10000000000,
                  'width':4,'height':3,'step':4,'encoding':'mono8','compression':'zlib',
                  'pi_receipt_monotonic_ns':8000000000000000000}, zlib.compress(bytes(range(12))))
final = {'kind':'finished','status':'saved','dropped':3}
if NEW_WORKER:
    final.update(source_frames=21,queue={'queued_bytes':0},capture_metrics={'marker':2})
write_packet(out, final)
'''.replace('NEW_WORKER', repr(new_worker))
    session = Session()
    session._run({'output_root': str(tmp_path), 'worker_command': [sys.executable, '-c', worker]})
    result = session.snapshot()
    assert result['state'] == 'partial', result
    expected_bytes, expected_frames, expected_marker = (0, 21, 2) if new_worker else (922019, 20, 1)
    assert result['transport_queue']['queued_bytes'] == expected_bytes
    assert result['source_frames'] == expected_frames
    assert result['capture_metrics']['marker'] == expected_marker
    run = Path(result['run_dir'])
    saved = json.loads((run/'session.json').read_text())
    assert saved['transport_queue']['queued_bytes'] == expected_bytes
    assert saved['capture_metrics']['marker'] == expected_marker
    times = saved['laptop_capture']
    assert times['packets'] == 4
    assert times['read_wait_ms_max'] >= 0
    assert times['writer_accept_ms_max'] >= 0
    assert times['images']['left']['images'] == 1
    events = [json.loads(line) for line in (run/'events.jsonl').read_text().splitlines()]
    row = json.loads((run/'frames.jsonl').read_text())
    assert all(item['laptop_receipt_monotonic_ns'] > 0 for item in [*events, row])
    assert all(item['laptop_read_wait_ms'] >= 0 for item in [*events, row])
    assert row['pi_receipt_monotonic_ns'] == 8_000_000_000_000_000_000
    assert row['laptop_image_saved_monotonic_ns'] >= row['laptop_receipt_monotonic_ns']
