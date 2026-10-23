# ================================================================
# probe_fence.py
# 설명: 장면 USD(/arena/arena.usd)의 단위·up축·메쉬 목록과, 후보 돌 위치 주변의
#       위에서 본 최고 높이(z) 격자를 출력한다. 읽기 전용(GPU 불필요).
# 사용법: Isaac Sim 컨테이너 python으로 실행 (python3 probe_fence.py)
# ================================================================
import numpy as np
from pxr import Usd, UsdGeom

CANDS = {'1': (3.92, 0.04), '2': (2.48, 0.78), '3': (0.65, 0.04), '4': (0.68, 3.11)}
HALF, RES = 0.30, 0.01

s = Usd.Stage.Open("/arena/arena.usd")
print("mpu", UsdGeom.GetStageMetersPerUnit(s), "up", UsdGeom.GetStageUpAxis(s), "default", s.GetDefaultPrim().GetPath() if s.GetDefaultPrim() else None)
cache = UsdGeom.XformCache()
meshes = []
for p in Usd.PrimRange(s.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
    if p.IsA(UsdGeom.Mesh):
        m = UsdGeom.Mesh(p)
        pts = np.array(m.GetPointsAttr().Get(), float)
        if not len(pts):
            continue
        M = np.array(cache.GetLocalToWorldTransform(p))      # row-vector convention
        w = pts @ M[:3, :3] + M[3, :3]
        fv = np.array(m.GetFaceVertexCountsAttr().Get()); fi = np.array(m.GetFaceVertexIndicesAttr().Get())
        if len(fv) and not np.all(fv == 3):
            # 사각형 이상은 팬 삼각분할
            tris, o = [], 0
            for c in fv:
                for k in range(1, c - 1):
                    tris.append((fi[o], fi[o + k], fi[o + k + 1]))
                o += c
            tri = np.array(tris)
        else:
            tri = fi.reshape(-1, 3)
        meshes.append((str(p.GetPath()), w, tri))
        lo, hi = w.min(0), w.max(0)
        print("mesh %-70s tris %7d bbox x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f" % (str(p.GetPath())[-70:], len(tri), lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))

def top_grid(cx, cy):
    xs = np.arange(cx - HALF, cx + HALF + 1e-9, RES); ys = np.arange(cy - HALF, cy + HALF + 1e-9, RES)
    G = np.full((len(ys), len(xs)), np.nan)
    X, Y = np.meshgrid(xs, ys)
    P = np.stack([X.ravel(), Y.ravel()], 1)
    best = np.full(len(P), -np.inf)
    for name, w, tri in meshes:
        A, B, C = w[tri[:, 0]], w[tri[:, 1]], w[tri[:, 2]]
        sel = (np.minimum(np.minimum(A[:, 0], B[:, 0]), C[:, 0]) <= xs[-1]) & (np.maximum(np.maximum(A[:, 0], B[:, 0]), C[:, 0]) >= xs[0]) & \
              (np.minimum(np.minimum(A[:, 1], B[:, 1]), C[:, 1]) <= ys[-1]) & (np.maximum(np.maximum(A[:, 1], B[:, 1]), C[:, 1]) >= ys[0])
        A, B, C = A[sel], B[sel], C[sel]
        for a, b, c in zip(A, B, C):
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (P[:, 0] - c[0]) + (c[0] - b[0]) * (P[:, 1] - c[1])) / d
            l2 = ((c[1] - a[1]) * (P[:, 0] - c[0]) + (a[0] - c[0]) * (P[:, 1] - c[1])) / d
            l3 = 1 - l1 - l2
            ins = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
            if ins.any():
                z = l1[ins] * a[2] + l2[ins] * b[2] + l3[ins] * c[2]
                best[ins] = np.maximum(best[ins], z)
    best[~np.isfinite(best)] = np.nan
    return xs, ys, best.reshape(len(ys), len(xs))

for k, (cx, cy) in CANDS.items():
    xs, ys, G = top_grid(cx, cy)
    print("\n== stone", k, "cand", (cx, cy), "top z (cm) every 5 cm; rows y descending, cols x ascending")
    print("      " + " ".join("%5.2f" % x for x in xs[::5]))
    for j in range(len(ys) - 1, -1, -5):
        print("%5.2f " % ys[j] + " ".join("  .  " if np.isnan(G[j, i]) else "%5.1f" % (G[j, i] * 100) for i in range(0, len(xs), 5)))
