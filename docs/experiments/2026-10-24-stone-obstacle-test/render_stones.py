# ================================================================
# render_stones.py
# 설명: 돌 오버레이 USD를 시뮬과 같은 조명·검은 벽 설정으로 열어 돌별 위에서/비스듬히 본 사진을 PNG로 저장한다.
#       로버는 넣지 않는다(돌 위치·색·크기 확인용, 읽기 전용).
# 사용법: Isaac Sim 컨테이너에서 /workspace=run 코드, /arena=USD, /work/out=출력 폴더로 마운트해 실행
# ================================================================
import numpy as np
from pathlib import Path

from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting', 'multi_gpu': False, 'width': 800, 'height': 600, 'anti_aliasing': 0})
import omni.usd
import omni.replicator.core as rep
import carb.settings
from pxr import Gf, UsdGeom
from PIL import Image
from experiments.site_sim.lighting import configure_lighting
from experiments.site_sim.wall_material import set_black_walls

carb.settings.get_settings().set('/rtx/post/dlss/execMode', 2)
OUT = Path('/work/out/render_stones2'); OUT.mkdir(exist_ok=True)
STONES = {'1': (3.955, 0.055, .6877, (-.7, .7)), '2': (2.45, 0.85, .6877, (1., 0.)), '3': (0.655, 0.05, .5606, (.7, .7)), '4': (0.655, 3.10, .3756, (.7, -.7))}

omni.usd.get_context().new_stage()
stage = omni.usd.get_context().get_stage()
stage.DefinePrim('/World/SandScene', 'Xform').GetReferences().AddReference('/arena/arena_rocks4_dark.usda')
print('LIGHTING', configure_lighting(stage, 'diffuse') is not None, flush=True)
set_black_walls(stage)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

def camera(path, eye, target, focal):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.GetFocalLengthAttr().Set(focal)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.02, 50.0))   # 기본 near clip이 1 m라 가까운 지면·돌이 잘린다
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1) if abs(eye[0] - target[0]) + abs(eye[1] - target[1]) > 1e-6 else Gf.Vec3d(0, 1, 0))
    UsdGeom.Xformable(cam).AddTransformOp().Set(view.GetInverse())
    return path

jobs = {}
for k, (x, y, gz, d) in STONES.items():
    c = (x, y, gz + .04)
    n = np.hypot(*d)
    jobs[f'{k}_top'] = camera(f'/World/Cam{k}_top', (x, y, gz + .55), c, 14.)
    jobs[f'{k}_side'] = camera(f'/World/Cam{k}_side', (x + .35 * d[0] / n, y + .35 * d[1] / n, gz + .22), c, 14.)
shots = {}
for name, path in jobs.items():
    prod = rep.create.render_product(path, (640, 480))
    ann = rep.AnnotatorRegistry.get_annotator('rgb'); ann.attach(prod)
    shots[name] = ann
for _ in range(60):          # 텍스처·재질 로딩을 위해 충분히 갱신한다
    app.update()
for _ in range(30):          # annotator가 비어 있지 않을 때까지 렌더를 반복한다
    rep.orchestrator.step(rt_subframes=8)
    if all(np.asarray(a.get_data()).size > 0 and np.asarray(a.get_data()).ndim == 3 for a in shots.values()):
        break
for name, ann in shots.items():
    data = np.asarray(ann.get_data())
    print('data', name, data.shape, data.dtype, flush=True)
    Image.fromarray(data[..., :3].astype(np.uint8)).save(OUT / f'{name}.png')
    print('saved', name, data.shape, 'mean', float(data[..., :3].mean()), flush=True)
app.close()
