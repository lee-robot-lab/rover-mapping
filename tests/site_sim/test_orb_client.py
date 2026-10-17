# ================================================================
# test_orb_client.py
# 설명: ORB 보조 프로세스와의 파일 요청·응답 프로토콜을 가짜 응답자로 검사한다.
# 사용법: .venv-mapping/bin/python -m pytest tests/site_sim/test_orb_client.py
# ================================================================
import threading
import time

import numpy as np
import pytest

from experiments.site_sim.orb_client import OrbClient


def responder(io, stop, state='2'):
    """stream_orb처럼 .req를 보면 .pose를 원자적으로 쓴다."""
    while not stop.is_set():
        for request in io.glob('*.req'):
            stamp = request.stem
            temporary = io/f'{stamp}.pose.tmp'
            temporary.write_text(f'{state} 7 321 1.0 2.0 3.0 0 0 0.7071067811865476 0.7071067811865476\n')
            temporary.rename(io/f'{stamp}.pose')
        time.sleep(.001)


def test_track_round_trip_parses_state_map_and_pose(tmp_path) -> None:
    (tmp_path/'READY').write_text('x')
    stop = threading.Event()
    thread = threading.Thread(target=responder, args=(tmp_path, stop), daemon=True)
    thread.start()
    try:
        result = OrbClient(tmp_path).track(123, tmp_path/'l.png', tmp_path/'r.png')
    finally:
        stop.set()
        thread.join()
    assert result['state'] == 2 and result['map_id'] == '7' and result['keypoints'] == 321
    assert result['world_from_camera'][:3, 3] == pytest.approx([1, 2, 3])
    assert result['world_from_camera'][:3, :3] @ [1, 0, 0] == pytest.approx([0, 1, 0], abs=1e-9)   # z축 90도
    assert not list(tmp_path.glob('123.*'))


def test_missing_response_times_out_and_failed_marker_raises(tmp_path) -> None:
    (tmp_path/'READY').write_text('x')
    with pytest.raises(TimeoutError):
        OrbClient(tmp_path, timeout_s=.05).track(1, tmp_path/'l.png', tmp_path/'r.png')
    (tmp_path/'FAILED').write_text('boom')
    with pytest.raises(RuntimeError, match='boom'):
        OrbClient(tmp_path, timeout_s=.5).track(2, tmp_path/'l.png', tmp_path/'r.png')


def test_not_ready_sidecar_is_reported(tmp_path) -> None:
    with pytest.raises(RuntimeError):
        OrbClient(tmp_path, ready_timeout_s=.05)
