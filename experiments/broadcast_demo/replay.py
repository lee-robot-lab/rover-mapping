# ================================================================
# replay.py
# 설명: 사전 수치 시뮬레이션 pose를 20Hz UDP로 송신해 실시간 입력을 재현한다.
# 사용법: python -m experiments.broadcast_demo.replay --run 1
# ================================================================
from __future__ import annotations
import argparse
import json
from pathlib import Path
import socket
import time
import uuid
import numpy as np
from experiments.broadcast_demo.paths import DEFAULT_OUT


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9870)
    parser.add_argument('--run', type=int, choices=(1,2,3,4), default=1)
    parser.add_argument('--hold', type=float, default=5., help='출발 위치 유지 시간')
    parser.add_argument('--loop', action='store_true')
    args = parser.parse_args()
    data = np.load(args.data/'predictions.npz')
    mask = data['runs'] == args.run
    if not mask.any():raise SystemExit(f'no replay data for RUN {args.run}')
    states, times = data['states'][mask], data['times'][mask]
    yaw = np.unwrap(states[:, 2])
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session_id = uuid.uuid4().hex
    seq = 0
    while True:
        started = time.monotonic()
        while True:
            elapsed = time.monotonic()-started-args.hold
            t = max(0., min(elapsed, times[-1]))
            state = [float(np.interp(t, times, yaw if i == 2 else states[:, i])) for i in range(5)]
            value = dict(zip(('x', 'y', 'yaw', 'v', 'w'), state))
            value.update(seq=seq, timestamp=time.time(), session_id=session_id)
            seq += 1
            sock.sendto(json.dumps(value, allow_nan=False).encode(), (args.host, args.port))
            if elapsed > times[-1]+3:
                break
            time.sleep(.05)
        if not args.loop:
            break
    print(f'RUN {args.run} UDP replay complete', flush=True)


if __name__ == '__main__':
    main()
