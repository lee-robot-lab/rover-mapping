# ================================================================
# transport.py
# 설명: 패킷 개수와 압축 전 byte 수를 동시에 제한하는 센서 전송 큐다.
# 사용법: from experiments.handheld_mapping.transport import ByteQueue
# ================================================================
import json
import queue
import threading


class ByteQueue:
    """전송 대기의 payload·header byte를 제한하고 포화 누락을 집계한다."""

    def __init__(self, max_items: int = 24, max_bytes: int = 4*1024*1024) -> None:
        if not isinstance(max_items, int) or not isinstance(max_bytes, int) or min(max_items, max_bytes) <= 0:
            raise ValueError('invalid queue limits')
        self.max_bytes = max_bytes
        self.queue: queue.Queue = queue.Queue(maxsize=max_items)
        self.lock = threading.Lock()
        self.queued_bytes = self.peak_bytes = self.dropped = 0

    def put_nowait(self, item: tuple[dict, bytes]) -> None:
        header, payload = item
        size = len(payload)+len(json.dumps(header, allow_nan=False, separators=(',', ':')).encode())+8
        with self.lock:
            if size+self.queued_bytes > self.max_bytes or self.queue.full():
                self.dropped += 1
                raise queue.Full
            self.queue.put_nowait((item, size))
            self.queued_bytes += size
            self.peak_bytes = max(self.peak_bytes, self.queued_bytes)

    def get(self) -> tuple[dict, bytes] | None:
        item, size = self.queue.get()
        with self.lock:
            self.queued_bytes -= size
        return item

    def finish(self, timeout: float = 10.) -> None:
        self.queue.put((None, 0), timeout=timeout)

    def snapshot(self) -> dict:
        with self.lock:
            return {'queued_bytes': self.queued_bytes, 'peak_bytes': self.peak_bytes,
                    'max_bytes': self.max_bytes, 'dropped': self.dropped,
                    'scope': 'queued raw payload/header bytes; excludes ROS/current frame and Python object overhead'}
