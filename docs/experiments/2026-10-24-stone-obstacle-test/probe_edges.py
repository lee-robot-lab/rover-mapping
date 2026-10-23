# ================================================================
# probe_edges.py
# 설명: 장면 USD 표면의 가장자리(모래가 끝나는 위치)를 후보 돌 근처에서 5 mm 간격 선 스캔으로 찾는다. 읽기 전용.
# 사용법: Isaac Sim 컨테이너 python으로 실행 (python3 probe_edges.py)
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
        if 0: print("mesh %-70s tris %7d bbox x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f" % (str(p.GetPath())[-70:], len(tri), lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))


def top_at(P):
    best = np.full(len(P), -np.inf)
    for name, w, tri in meshes:
        A, B, C = w[tri[:, 0]], w[tri[:, 1]], w[tri[:, 2]]
        lo = P.min(0) - 1e-6; hi = P.max(0) + 1e-6
        sel = (np.minimum(np.minimum(A[:, 0], B[:, 0]), C[:, 0]) <= hi[0]) & (np.maximum(np.maximum(A[:, 0], B[:, 0]), C[:, 0]) >= lo[0]) & \
              (np.minimum(np.minimum(A[:, 1], B[:, 1]), C[:, 1]) <= hi[1]) & (np.maximum(np.maximum(A[:, 1], B[:, 1]), C[:, 1]) >= lo[1])
        for a, b, c in zip(A[sel], B[sel], C[sel]):
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (P[:, 0] - c[0]) + (c[0] - b[0]) * (P[:, 1] - c[1])) / d
            l2 = ((c[1] - a[1]) * (P[:, 0] - c[0]) + (a[0] - c[0]) * (P[:, 1] - c[1])) / d
            l3 = 1 - l1 - l2
            ins = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
            if ins.any():
                best[ins] = np.maximum(best[ins], l1[ins] * a[2] + l2[ins] * b[2] + l3[ins] * c[2])
    best[~np.isfinite(best)] = np.nan
    return best

def scan(name, axis, fixed, lo, hi, step=0.005):
    t = np.arange(lo, hi + 1e-9, step)
    P = np.stack([t, np.full_like(t, fixed)], 1) if axis == 'x' else np.stack([np.full_like(t, fixed), t], 1)
    z = top_at(P)
    print("\n%s: scan along %s at %s=%.3f" % (name, axis, 'y' if axis == 'x' else 'x', fixed))
    print("  " + " ".join("%.3f:%s" % (tt, "--" if np.isnan(zz) else "%.1f" % (zz * 100)) for tt, zz in zip(t, z)))

# 돌 1 (남동): 동쪽 끝 x, 남쪽 끝 y
scan("stone1 east edge", 'x', 0.20, 3.90, 4.02); scan("stone1 south edge", 'y', 3.90, -0.03, 0.10)
# 돌 3 (남서)
scan("stone3 west edge", 'x', 0.20, 0.55, 0.72); scan("stone3 south edge", 'y', 0.70, -0.03, 0.10)
# 돌 4 (북서): 서쪽 끝, 북쪽 경계 근처
scan("stone4 west edge", 'x', 2.95, 0.55, 0.72); scan("stone4 north", 'y', 0.80, 3.00, 3.25)
# 돌 2: 정상 평탄면 모서리
scan("stone2 x-scan y=0.70", 'x', 0.70, 2.38, 2.52); scan("stone2 x-scan y=0.90", 'x', 0.90, 2.38, 2.52)
scan("stone2 y-scan x=2.35", 'y', 2.35, 0.70, 0.90); scan("stone2 y-scan x=2.50", 'y', 2.50, 0.70, 0.90)
