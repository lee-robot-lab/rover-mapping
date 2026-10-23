# ================================================================
# verify_live.py
# 설명: 실제 20Hz UDP 송신·10Hz HTTP 조회로 두 RUN과 stale 예측 차단을 통합 검증한다.
# 사용법: python -m experiments.broadcast_demo.verify_live --seconds 6
# ================================================================
from __future__ import annotations
import argparse
import json
import socket
import threading
import time
from urllib.request import Request, urlopen
import uuid
from pathlib import Path
import numpy as np
from experiments.broadcast_demo.paths import DEFAULT_OUT

HTTP_BASE = 'http://127.0.0.1:8765'


def request(path: str, post: bool = False) -> dict:
    with urlopen(Request(HTTP_BASE+path, method='POST' if post else 'GET'), timeout=3) as response:
        return json.load(response)


def main() -> None:
    global HTTP_BASE
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=6.)
    parser.add_argument('--data', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--http-port', type=int, default=8765)
    parser.add_argument('--udp-port', type=int, default=9870)
    args = parser.parse_args()
    HTTP_BASE = f'http://127.0.0.1:{args.http_port}'
    with np.load(args.data/'predictions.npz') as stored:
        dataset = {k:stored[k] for k in stored.files}
    report = dict(runs=[], http_ms=[], ages_ms=[], match_count=0, samples=0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session_id, seq = uuid.uuid4().hex, 0
    for run in np.unique(dataset['runs']).tolist():
        mask = dataset['runs'] == run
        rows, stamps = dataset['states'][mask], dataset['times'][mask]
        stop = threading.Event()
        def send() -> None:
            nonlocal seq
            start = time.monotonic()
            deadline = start
            while not stop.is_set():
                t = min(max(0., time.monotonic()-start-.5), stamps[-1])
                state = [float(np.interp(t, stamps, np.unwrap(rows[:, i]) if i == 2 else rows[:, i])) for i in range(5)]
                packet = dict(zip(('x','y','yaw','v','w'),state))
                packet.update(seq=seq,timestamp=time.time(),session_id=session_id)
                sock.sendto(json.dumps(packet).encode(), ('127.0.0.1',args.udp_port));seq += 1
                deadline += .05
                stop.wait(max(0.,deadline-time.monotonic()))
        worker = threading.Thread(target=send)
        worker.start()
        time.sleep(.15)
        request(f'/api/run/{run}', True)
        begin = time.monotonic()
        while time.monotonic()-begin < args.seconds:
            tick = time.monotonic()
            value = request('/api/state')
            report['http_ms'].append((time.monotonic()-tick)*1000)
            report['ages_ms'].append(value['age_s']*1000)
            report['match_count'] += int(value['prediction'] is not None)
            report['samples'] += 1
            time.sleep(max(0.,.1-(time.monotonic()-tick)))
        request('/api/stop',True)
        stop.set();worker.join()
        value = request('/api/state')
        assert len(value['tracks'][run-1]) > args.seconds*15
        assert len(value['explored'][run-1]) > 0
        if run == 2:
            assert len(value['tracks'][0]) == report['runs'][0]['track_points']
        report['runs'].append(dict(run=run, track_points=len(value['tracks'][run-1]),
                                  distance_m=value['distances'][run-1], explored_cells=len(value['explored'][run-1])))
        print('verified',report['runs'][-1],flush=True)
    time.sleep(.6)
    final = request('/api/state')
    assert final['stale'] and final['prediction'] is None
    sock.close()
    result = dict(runs=report['runs'], samples=report['samples'], match_count=report['match_count'],
                  http_p95_ms=float(np.percentile(report['http_ms'],95)),
                  pose_age_p95_ms=float(np.percentile(report['ages_ms'],95)), stale_prediction_hidden=True,
                  test='local numeric simulation UDP; not remote Isaac physics or network RTT')
    (args.data/'live_verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
