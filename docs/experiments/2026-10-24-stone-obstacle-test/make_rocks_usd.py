# ================================================================
# make_rocks_usd.py
# 설명: 원본 아레나 USD(/arena/arena.usd)를 sublayer로 두고 지름 --diameter(기본 8 cm) 구형 돌 4개(정적 Mesh)를
#       더한 오버레이 USD를 만든다. 원본은 바꾸지 않는다. 돌 아래 지면 높이는 원본 표면에서 측정한다.
# 사용법: Isaac Sim 컨테이너 python으로 /work에서 실행
#   (원본을 /arena/arena.usd로, 출력 폴더를 /work/out으로 마운트) python3 make_rocks_usd.py [--diameter 0.15 --out /work/out/arena_rocks4_d15.usda]
# ================================================================
import json
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt

import argparse
ap = argparse.ArgumentParser()
ap.add_argument('--diameter', type=float, default=0.08, help='돌 지름 [m]')
ap.add_argument('--out', default='/work/out/arena_rocks4_dark.usda')
_a = ap.parse_args()
R = _a.diameter / 2     # 돌 반지름
SINK = 0.0005           # 지면에 0.5 mm 묻어 접촉을 보장
GAP = 0.005             # 가장자리와 돌 표면 사이 간격
# 가장자리 좌표(Isaac 장면 USD 측정, probe_edges.out): 동 x=4.000, 남 y=0.010(남동)·0.005(남서), 서 x=0.610, 북 y=3.145, 정상 절벽 x=2.403
STONES = {'rock1_se': (4.000 - R - GAP, 0.010 + R + GAP), 'rock2_summit': (2.403 + R + GAP, 0.85),
          'rock3_sw': (0.610 + R + GAP, 0.005 + R + GAP), 'rock4_nw': (0.610 + R + GAP, 3.145 - R - GAP)}
SRC, OUT = '/arena/arena.usd', _a.out      # 실행 때와 같은 마운트(/arena/arena.usd)를 쓴다


def top_at(meshes, P):
    best = np.full(len(P), -np.inf)
    for w, tri in meshes:
        A, B, C = w[tri[:, 0]], w[tri[:, 1]], w[tri[:, 2]]
        lo, hi = P.min(0) - 1e-6, P.max(0) + 1e-6
        sel = (np.minimum(np.minimum(A[:, 0], B[:, 0]), C[:, 0]) <= hi[0]) & (np.maximum(np.maximum(A[:, 0], B[:, 0]), C[:, 0]) >= lo[0]) & \
              (np.minimum(np.minimum(A[:, 1], B[:, 1]), C[:, 1]) <= hi[1]) & (np.maximum(np.maximum(A[:, 1], B[:, 1]), C[:, 1]) >= lo[1])
        for a, b, c in zip(A[sel], B[sel], C[sel]):
            d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(d) < 1e-12:
                continue
            l1 = ((b[1] - c[1]) * (P[:, 0] - c[0]) + (c[0] - b[0]) * (P[:, 1] - c[1])) / d
            l2 = ((c[1] - a[1]) * (P[:, 0] - c[0]) + (a[0] - c[0]) * (P[:, 1] - c[1])) / d
            ins = (l1 >= -1e-9) & (l2 >= -1e-9) & (1 - l1 - l2 >= -1e-9)
            if ins.any():
                best[ins] = np.maximum(best[ins], l1[ins] * a[2] + l2[ins] * b[2] + (1 - l1[ins] - l2[ins]) * c[2])
    best[~np.isfinite(best)] = np.nan
    return best


def load_meshes(stage):
    cache, out = UsdGeom.XformCache(), []
    for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
        if not p.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(p)
        pts = np.array(m.GetPointsAttr().Get(), float)
        M = np.array(cache.GetLocalToWorldTransform(p))
        fv, fi = np.array(m.GetFaceVertexCountsAttr().Get()), np.array(m.GetFaceVertexIndicesAttr().Get())
        assert np.all(fv == 3), p.GetPath()
        out.append((pts @ M[:3, :3] + M[3, :3], fi.reshape(-1, 3)))
    return out


def icosphere(radius: float, sub: int = 3):
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
        for a, b, c in f:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            nf += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        f = nf
    return np.array(v) * radius, np.array(f)


src = Usd.Stage.Open(SRC)
meshes = load_meshes(src)
P = np.array(list(STONES.values()))
ground = top_at(meshes, P)
assert np.all(np.isfinite(ground)), ground

layer = Sdf.Layer.CreateNew(OUT)
layer.subLayerPaths.append('/arena/arena.usd')
st = Usd.Stage.Open(layer)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(st, 1.0)
UsdGeom.Xform.Define(st, '/World/Rocks')
st.SetDefaultPrim(st.GetPrimAtPath('/World'))
unit, faces = icosphere(R, 3)
# 어두운 숯색(사용자: 검정에 가까운 회색). 벽의 무광 검정(.008)보다 조금 밝게 둔다 — 내가 고른 값.
DARK = (.05, .05, .05)
mat = UsdShade.Material.Define(st, '/World/Rocks/Looks/Dark')
sh = UsdShade.Shader.Define(st, '/World/Rocks/Looks/Dark/Shader')
sh.CreateIdAttr('UsdPreviewSurface')
sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*DARK))
sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.9)
sh.CreateInput('useSpecularWorkflow', Sdf.ValueTypeNames.Int).Set(1)
sh.CreateInput('specularColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.))
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
report = {}
for (name, (x, y)), gz in zip(STONES.items(), ground):
    c = np.array([x, y, gz + R - SINK])
    m = UsdGeom.Mesh.Define(st, '/World/Rocks/' + name)
    m.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*(p + c)) for p in unit]))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray(faces.ravel().tolist()))
    m.CreateSubdivisionSchemeAttr('none')
    m.CreateDisplayColorAttr([Gf.Vec3f(*DARK)])
    UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
    m.CreateExtentAttr([Gf.Vec3f(*(c - R)), Gf.Vec3f(*(c + R))])
    report[name] = {'xy': [x, y], 'ground_z': float(gz), 'center_z': float(c[2]), 'top_z': float(c[2] + R), 'tris': int(len(faces))}
st.GetRootLayer().Save()
print(json.dumps(report, indent=1))

# 검증: 오버레이를 새로 열어 합성 결과에서 돌 위 높이 = 지면 + 8 cm(−0.5 mm)인지, 돌 밖 지면은 그대로인지 확인한다.
chk = Usd.Stage.Open(OUT)
print('defaultPrim', chk.GetDefaultPrim().GetPath(), 'rocks', [str(p.GetPath()) for p in chk.GetPrimAtPath('/World/Rocks').GetChildren()])
cm = load_meshes(chk)
top = top_at(cm, P)
for (name, _), t, g in zip(STONES.items(), top, ground):
    print('%-14s composed top %.4f  expected %.4f  height above ground %.1f mm' % (name, t, g + 2 * R - SINK, (t - g) * 1000))
off = P + np.array([R + 0.03, 0.0])      # 돌 옆 3 cm 바깥: 원본과 합성 결과가 같아야 한다
a, b = np.nan_to_num(top_at(meshes, off), nan=-1), np.nan_to_num(top_at(cm, off), nan=-1)
print('ground unchanged check (orig vs composed) at offset pts:', np.allclose(a, b), a.tolist(), b.tolist())
print('composed mesh count orig/composed:', len(meshes), len(cm))
print('rock bind:', [str(UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0].GetPath()) for p in chk.GetPrimAtPath('/World/Rocks').GetChildren() if p.IsA(UsdGeom.Mesh)])
