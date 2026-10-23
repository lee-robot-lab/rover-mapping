# ================================================================
# test_core.py
# 설명: UDP 최신성, 좌표 정렬, RUN 보존, stale 조회를 검증한다.
# 사용법: python -m pytest tests/broadcast_demo -q
# ================================================================
import json
import math
import numpy as np
import pytest
from experiments.broadcast_demo.core import Session, decode_packet, Lookup


def packet(seq=1, **kw):
    return dict(seq=seq, timestamp=100., x=10., y=20., yaw=0., v=.1, w=0., **kw)


def test_invalid_packet():
    for value in [b'bad', b'{}', json.dumps(packet(xxx=3)).encode().replace(b'10.0', b'NaN')]:
        with pytest.raises(ValueError):
            decode_packet(value)


def test_alignment_wrap_and_preserve_runs():
    session = Session((1., 2., math.pi/2))
    session.ingest(packet(), now=10.)
    session.start(1, now=10.)
    session.ingest(dict(packet(2), x=11.), now=10.1)
    result = session.snapshot(now=10.1)
    assert result['pose']['x'] == pytest.approx(1.)
    assert result['pose']['y'] == pytest.approx(3.)
    assert result['pose']['yaw'] == pytest.approx(math.pi/2)
    assert len(result['tracks'][0]) == 2
    session.start(2, now=10.2)
    assert len(session.snapshot(now=11.)['tracks'][0]) == 2
    assert session.snapshot(now=11.)['pose']['y'] == pytest.approx(2.)


def test_old_sequence_and_stale_start():
    session = Session((0., 0., 0.))
    assert session.ingest(packet(3), now=0.)
    assert not session.ingest(packet(2), now=.1)
    assert session.snapshot(now=1.)['stale']
    with pytest.raises(ValueError):
        session.start(1, now=1.)


def test_no_record_before_run_and_nonfinite_rejected():
    session = Session((0., 0., 0.))
    session.ingest(packet(), now=0.)
    assert session.snapshot(now=0.)['tracks'] == [[], [], [], []]
    with pytest.raises(ValueError):
        decode_packet(json.dumps(dict(packet(), v=float('nan'))).encode())


def test_bounded_lookup_wrap_and_run_filter():
    lookup = Lookup(np.array([[1., 2., -math.pi+.01, .1, 0.],
                              [1., 2., 0., .1, 0.]]), np.array([1, 2]))
    assert lookup.find([1., 2., math.pi-.01, .1, 0.], 1) == 0
    assert lookup.find([2., 2., math.pi, .1, 0.], 1) is None
    assert lookup.find([1., 2., math.pi, .1, 0.], 2) is None


def test_invalid_utf8_is_rejected():
    with pytest.raises(ValueError):
        decode_packet(b'\xff\xfe')


def test_stop_freezes_elapsed():
    session = Session((0., 0., 0.))
    session.ingest(packet(), now=0.)
    session.start(1, now=0.)
    session.stop(now=.3)
    assert session.snapshot(now=10.)['elapsed_s'] == pytest.approx(.3)


def test_sender_restart_with_new_session_and_late_old_session():
    session = Session((0., 0., 0.))
    session.ingest(dict(packet(100), session_id='a'), now=0.)
    assert session.ingest(dict(packet(0), session_id='b'), now=.1)
    assert not session.ingest(dict(packet(101), session_id='a'), now=.2)
    assert session.raw['seq'] == 0
