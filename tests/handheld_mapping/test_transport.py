# ================================================================
# test_transport.py
# 설명: 전송 큐의 byte·item 상한과 drain/종료 동작을 검증한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/handheld_mapping/test_transport.py -q
# ================================================================
import queue
import threading

import pytest


def test_byte_limit_rejects_without_growing_and_drains():
    from experiments.handheld_mapping.transport import ByteQueue
    pending = ByteQueue(max_items=20, max_bytes=200)
    item = ({'kind': 'image'}, b'a'*80)
    pending.put_nowait(item)
    before = pending.snapshot()['queued_bytes']
    with pytest.raises(queue.Full):
        pending.put_nowait(item)
    assert pending.snapshot()['queued_bytes'] == before
    assert pending.snapshot()['dropped'] == 1
    assert pending.get() == item
    assert pending.snapshot()['queued_bytes'] == 0
    pending.put_nowait(item)
    assert pending.snapshot()['peak_bytes'] <= 200


def test_item_limit_and_finish_wait_for_consumer():
    from experiments.handheld_mapping.transport import ByteQueue
    pending = ByteQueue(max_items=1, max_bytes=1000)
    item = ({'kind': 'pose'}, b'')
    pending.put_nowait(item)
    with pytest.raises(queue.Full):
        pending.put_nowait(item)
    consumer = threading.Thread(target=lambda: pending.get())
    consumer.start()
    pending.finish(timeout=1.)
    consumer.join(timeout=1.)
    assert pending.get() is None
    assert pending.snapshot()['queued_bytes'] == 0


@pytest.mark.parametrize('items,size', [(0, 100), (2, 0), (-1, 100)])
def test_invalid_queue_limits(items: int, size: int):
    from experiments.handheld_mapping.transport import ByteQueue
    with pytest.raises(ValueError):
        ByteQueue(items, size)
