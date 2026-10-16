"""유한 버퍼의 묶음 원자성·수신 분리·오류 전달을 실제 패킷으로 검증한다."""
import io
import queue
import time

import pytest

from experiments.handheld_mapping.protocol import write_packet
from experiments.handheld_mapping.transport import ByteQueue, FrameBundler, PacketReader


def item(side, stamp, receipt=1_000_000_000):
    return {'kind': 'image', 'stream': side, 'stamp_ns': stamp,
            'pi_receipt_monotonic_ns': receipt}, bytes(20)


def test_full_queue_cannot_accept_half_of_capture_group():
    pending = ByteQueue(max_items=4, max_bytes=10000)
    pending.put_many_nowait([item('left', 1), item('right', 1), item('depth', 1)])
    with pytest.raises(queue.Full):
        pending.put_many_nowait([item('left', 2), item('right', 2), item('depth', 2)])
    pending.finish()
    assert [pending.get()[0]['stamp_ns'] for _ in range(3)] == [1, 1, 1]
    assert pending.get() is None
    assert pending.snapshot()['dropped'] == 3


def test_bundler_handles_callback_order_near_depth_stamps_and_missing_camera_frames():
    groups, dropped = [], []
    bundles = FrameBundler(groups.append, dropped.append)
    bundles.push(item('depth', 1_000_000_318))
    bundles.push(item('right', 1_000_000_000))
    bundles.push(item('left', 1_000_000_000))
    assert [entry[0]['stream'] for entry in groups[0]] == ['left', 'right', 'depth']
    bundles.push(item('left', 1_100_000_000))
    bundles.push(item('right', 1_100_000_000))
    # Depth at 1.1s never arrives; do not pair with the 1.2s surface.
    for side in ('left', 'depth', 'right'):
        bundles.push(item(side, 1_200_000_000))
    assert len(groups) == 2
    assert {entry[0]['stream'] for entry in dropped} == {'left', 'right'}
    assert [entry[0]['stamp_ns'] for entry in groups[1]] == [1_200_000_000]*3
    bundles.finish()
    assert bundles.snapshot()['pending_images'] == 0


def test_absent_depth_cannot_grow_partial_group_buffer():
    groups, dropped = [], []
    bundles = FrameBundler(groups.append, dropped.append)
    for frame in range(100):
        for side in ('left', 'right'):
            bundles.push(item(side, frame*66_666_667, receipt=1_000_000_000+frame*66_666_667))
    assert not groups
    assert bundles.snapshot()['pending_images'] <= 6
    bundles.finish()
    assert len(dropped) == 200
    groups, dropped = [], []
    bundles = FrameBundler(groups.append, dropped.append)
    bundles.push(item('left', 1, receipt=1_000_000_000))
    bundles.push(item('right', 1, receipt=1_000_000_000))
    bundles.push(item('depth', 1, receipt=1_300_000_000))
    assert not groups  # Same stamp cannot revive a group whose callback wait expired.
    bundles.finish()
    assert len(dropped) == 3


def wait_until(predicate):
    deadline = time.monotonic()+3
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.001)


def test_receiver_reads_ahead_of_writer_and_preserves_order_under_backpressure():
    stream = io.BytesIO()
    for sequence in range(80):
        write_packet(stream, {'kind': 'image', 'seq': sequence}, bytes(100))
    stream.seek(0)
    reader = PacketReader(stream, max_bytes=1200, max_items=4)
    try:
        wait_until(lambda: reader.snapshot()['packets'] >= 4)
        # No consumer has run yet: this proves reception does not wait for a PNG write.
        assert reader.snapshot()['queued_bytes'] > 0
        received = []
        while True:
            packet = reader.get()
            if packet is None:
                break
            received.append(packet[0]['seq'])
            assert packet[0]['laptop_receive_queue_wait_ms'] >= 0
        assert received == list(range(80))
        assert reader.snapshot()['peak_bytes'] <= 1200
        assert reader.snapshot()['drops'] == 0
    finally:
        reader.close()
        reader.thread.join(timeout=2)
    assert not reader.thread.is_alive()


def test_truncated_packet_error_is_raised_after_preceding_packets_are_drained():
    stream = io.BytesIO()
    write_packet(stream, {'kind': 'ready'})
    write_packet(stream, {'kind': 'image'}, b'payload')
    reader = PacketReader(io.BytesIO(stream.getvalue()[:-1]))
    try:
        assert reader.get()[0]['kind'] == 'ready'
        with pytest.raises(EOFError):
            reader.get()
    finally:
        reader.close()
        reader.thread.join(timeout=2)


def test_cancel_releases_receiver_waiting_for_full_buffer():
    stream = io.BytesIO()
    for i in range(10):
        write_packet(stream, {'kind': 'status', 'seq': i})
    stream.seek(0)
    reader = PacketReader(stream, max_items=1)
    wait_until(lambda: reader.snapshot()['packets'] == 1)
    reader.close()
    reader.thread.join(timeout=2)
    assert not reader.thread.is_alive()
