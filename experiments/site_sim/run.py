# ================================================================
# run.py
# 설명: 기존 Isaac 경기장·로버를 임무 feedback으로 구동하고 영상·costmap·시계열을 기록한다.
# 사용법: /isaac-sim/python.sh -m experiments.site_sim.run --mission MISSION --urdf ROVER --geometry JSON --scene-usd USD --output NEW_RUN
# ================================================================
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import time


def validate_limits(sim_s: float, wall_s: float) -> None:
    """물리와 wall 실행 상한을 유한한 범위로 제한한다."""
    if not math.isfinite(sim_s) or not 0<sim_s<=900 or not math.isfinite(wall_s) or not 0<wall_s<=3600:
        raise ValueError('bounded sim (0,900] and wall (0,3600] limits required')


def run(args: argparse.Namespace) -> dict:
    validate_limits(args.max_sim_s,args.max_wall_s)
    import numpy as np
    from experiments.handheld_mapping.site_mission import SiteMission
    from experiments.handheld_mapping.storage import atomic_json
    from experiments.site_sim.recording import RunRecorder
    mission=SiteMission.load(args.mission,load_mesh=False)
    recorder=RunRecorder(mission,args.output,fault_injection=args.fault_injection)
    app=drive=world=None;status='failed';reason='initialization_failed';cameras={};wall_start=time.monotonic()
    def interrupted(signum,frame):raise KeyboardInterrupt(f'signal {signum}')
    previous_signal=signal.signal(signal.SIGTERM,interrupted)
    try:
        from isaacsim import SimulationApp
        app=SimulationApp({'headless':True,'renderer':'RaytracedLighting','multi_gpu':False,
                           'width':800,'height':600,'anti_aliasing':0})
        import omni.usd
        import omni.replicator.core as rep
        from isaacsim.core.api.robots import Robot
        from pxr import Gf,UsdGeom
        from PIL import Image
        from experiments.orbslam3_ros2.isaac_scene import build_isaac_scene
        from experiments.orbslam3_ros2.drive_runtime import DriveSession
        from experiments.site_sim.drive import SimulationDrive
        site=mission.site;c=site.cells([mission.start[:2]])[0];ground=float(site.height_m[c[1],c[0]])
        world,h=build_isaac_scene(urdf=args.urdf,scene_usd=args.scene_usd,output=Path(args.output),
            start_position=(*mission.start[:2],ground+.23))
        for _ in range(5):app.update()
        stage=omni.usd.get_context().get_stage()
        overview=UsdGeom.Camera.Define(stage,'/World/ObserverCamera')
        overview.CreateFocalLengthAttr(18.);overview.CreateHorizontalApertureAttr(20.)
        overview.CreateVerticalApertureAttr(15.);overview.CreateClippingRangeAttr(Gf.Vec2f(.1,100))
        center=site.origin+np.asarray(site.observed.shape[::-1])*site.resolution/2
        transform=Gf.Matrix4d().SetLookAt(Gf.Vec3d(float(center[0]),float(center[1])-4.5,5.5),
                                       Gf.Vec3d(float(center[0]),float(center[1]),.35),Gf.Vec3d(0,0,1)).GetInverse()
        UsdGeom.Xformable(overview).AddTransformOp().Set(transform)
        # Original portable optical frame was 35deg; child camera compensates to the agreed 20deg.
        phi=math.pi-math.radians(20.-35.)
        UsdGeom.Xformable(stage.GetPrimAtPath(h.left_camera_path)).GetOrderedXformOps()[0].Set(
            Gf.Quatd(math.cos(phi/2),math.sin(phi/2),0.,0.))
        for key,path in (('overview','/World/ObserverCamera'),('front',h.left_camera_path)):
            product=rep.create.render_product(path,(800,600) if key=='overview' else (640,480))
            annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach(product)
            cameras[key]=(product,annotator)
        robot=world.scene.add(Robot(prim_path=h.frame_paths['base_link'],name='rover'))
        world.set_simulation_dt(physics_dt=.005,rendering_dt=.05)
        world.initialize_physics();robot.initialize()
        yaw=float(mission.start[2]);robot.set_world_pose(orientation=np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)]))
        session=DriveSession(robot,args.geometry);world.play()
        for i in range(400):
            if time.monotonic()-wall_start>args.max_wall_s:raise TimeoutError('startup wall timeout')
            session.stop();world.step(render=i%20==0)
        p,q=robot.get_world_pose()
        if not np.isfinite(np.r_[p,q]).all() or np.linalg.norm(p[:2]-mission.start[:2])>.1 or not ground+.08<p[2]<ground+.35:
            raise ValueError(f'rover settling failed: {p}')
        drive=SimulationDrive(robot,session,mission,fault_injection=args.fault_injection)
        t0=float(world.current_time);next_control=0.;next_render=0.;last_camera_error=None
        print('SITE_SIM_READY',json.dumps({'pose':p.tolist(),'output':str(args.output)}),flush=True)
        atomic_json(Path(args.output)/'ready.json',{'position_m':p.tolist(),'wall_s':time.monotonic()-wall_start})
        while app.is_running():
            sim_s=float(world.current_time)-t0;wall_s=time.monotonic()-wall_start
            if sim_s>=args.max_sim_s or wall_s>=args.max_wall_s:status='timeout';reason='sim_or_wall_limit';break
            if sim_s+1e-9>=next_control:
                row=drive.step(sim_s);row['wall_s']=wall_s
                row['mission']['camera_error']=last_camera_error
                recorder.observe(row,drive.runtime.events);next_control+=.05
                if sim_s>=5 and row['mission']['reason'] in ('invalid_pose','pose_jump','unsafe_observed_motion','unsafe_local_segment'):
                    status='failed';reason=row['mission']['reason'];break
                if row['mission']['state'] in ('complete','failed'):
                    status=row['mission']['state'];reason=row['mission']['reason'];break
                p=np.asarray(row['position_m']);q=np.asarray(row['quaternion_wxyz'])
                if p[2]<ground-.1 or 1-2*(q[1]**2+q[2]**2)<math.cos(math.radians(70)):
                    status='failed';reason='fallen_or_tipped';break
            render_due=time.monotonic()>=next_render
            world.step(render=render_due)
            if render_due:
                next_render=time.monotonic()+.2
                for key,(_,annotator) in cameras.items():
                    try:
                        rgba=np.asarray(annotator.get_data())
                        if rgba.ndim!=3 or rgba.shape[2]<3:continue
                        path=recorder.live/f'{key}.jpg';tmp=path.with_suffix('.tmp')
                        Image.fromarray(rgba[:,:,:3]).save(tmp,format='JPEG',quality=82);tmp.replace(path)
                    except Exception as error:last_camera_error=str(error)
                recorder.publish()
    except KeyboardInterrupt as error:status='interrupted';reason=str(error)
    except Exception as error:
        reason=f'{type(error).__name__}: {error}'
        import traceback
        traceback.print_exc()
    finally:
        if drive is not None:
            try:
                drive.stop()
                atomic_json(Path(args.output)/'stop.json',{'wheel_velocity_targets_rad_s':[0.]*4,'applied':True})
                if world is not None:
                    for _ in range(20):world.step(render=False)
            except Exception as error:status='failed';reason+='; stop failed: '+str(error)
        try:
            summary=recorder.finalize(status,reason)
        except Exception as error:
            raise RuntimeError(f'{reason}; finalization failed: {error}') from error
        finally:
            signal.signal(signal.SIGTERM,previous_signal)
            if app is not None:app.close()
    print('SITE_SIM_FINISHED',json.dumps(summary),flush=True)
    return summary


def main() -> None:
    parser=argparse.ArgumentParser()
    for name in ('mission','urdf','geometry','scene-usd','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--max-sim-s',type=float,default=900.);parser.add_argument('--max-wall-s',type=float,default=3600.)
    parser.add_argument('--fault-injection',action='store_true');args=parser.parse_args();code=1
    try:
        summary=run(args);code=0 if summary['status']=='complete' else 2
    finally:
        Path(args.output).parent.joinpath('exit.txt').write_text(str(code)+'\n')
    raise SystemExit(code)


if __name__=='__main__':main()
