# ================================================================
# make_stone_stl.py
# 설명: 원본 KSRC STL(m 단위, 바이너리)에 구형 돌 4개 삼각형을 덧붙여 새 STL을 쓴다. 지도(rebuild_mission) 입력용.
#       돌 위치는 make_rocks_usd.py와 같은 가장자리 규칙(가장자리 + 반지름 + 5 mm)을 쓰고, 지면 높이는 USD 측정값이다.
# 사용법: python3 make_stone_stl.py --diameter 0.10 --out OUT.stl
# ================================================================
import argparse
import struct
import numpy as np

SRC = '/home/su/rover/outputs/ksrc_final_map/20261022_board_level_sand/KSRC_board_level_sand_m.stl'
GROUND = {'1': .6881, '2': .6881, '3': .5606, '4': .3756}    # probe: make_rocks_usd.py 출력의 ground_z
ap = argparse.ArgumentParser(); ap.add_argument('--diameter', type=float, required=True); ap.add_argument('--out', required=True)
a = ap.parse_args(); R = a.diameter / 2; GAP = .005; SINK = .0005
C = {'1': (4.000 - R - GAP, 0.010 + R + GAP), '2': (2.403 + R + GAP, 0.85), '3': (0.610 + R + GAP, 0.005 + R + GAP), '4': (0.610 + R + GAP, 3.145 - R - GAP)}

def icosphere(sub=2):
    t = (1 + 5 ** .5) / 2
    v = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    f = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
         (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    v = [np.array(p, float) / np.linalg.norm(p) for p in v]
    for _ in range(sub):
        cache, nf = {}, []
        def mid(i, j):
            k = (min(i, j), max(i, j))
            if k not in cache:
                m = v[i] + v[j]; v.append(m / np.linalg.norm(m)); cache[k] = len(v) - 1
            return cache[k]
        for p, q, r in f:
            pq, qr, rp = mid(p, q), mid(q, r), mid(r, p)
            nf += [(p, pq, rp), (q, qr, pq), (r, rp, qr), (pq, qr, rp)]
        f = nf
    return np.array(v), np.array(f)

raw = open(SRC, 'rb').read()
n = struct.unpack('<I', raw[80:84])[0]
assert 84 + 50 * n == len(raw)
dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
unit, faces = icosphere(2)
new = []
for k, (x, y) in C.items():
    c = np.array([x, y, GROUND[k] + R - SINK]); P = unit * R + c
    tri = P[faces]                                               # (F,3,3), 바깥 법선 방향 확인 후 필요하면 뒤집는다
    nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    out = (nrm * (tri.mean(1) - c)).sum(1) < 0
    tri[out] = tri[out][:, [0, 2, 1]]
    rec = np.zeros(len(tri), dt); rec['v'] = tri
    nn = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]); rec['n'] = nn / np.linalg.norm(nn, axis=1, keepdims=True)
    new.append(rec)
    print('stone', k, 'center', np.round(c, 4), 'top z', round(c[2] + R, 4), 'tris', len(rec))
new = np.concatenate(new)
with open(a.out, 'wb') as f:
    f.write(raw[:80]); f.write(struct.pack('<I', n + len(new))); f.write(raw[84:]); f.write(new.tobytes())
print('wrote', a.out, 'triangles', n, '->', n + len(new))
