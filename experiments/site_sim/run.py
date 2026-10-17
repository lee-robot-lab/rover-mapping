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

# 영상 장면이 붙은 timestamp보다 늦은 시뮬 렌더 지연(2026-10-17 GT 회전속도와의 시차 스캔으로 측정, 약 0.10 s).
RENDER_LAG_S=0.1


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
    stereo_hz=float(getattr(args,'stereo_hz',0.))
    if not math.isfinite(stereo_hz) or not 0<=stereo_hz<=30:raise ValueError('stereo_hz must be in [0,30]')
    recorder=RunRecorder(mission,args.output,fault_injection=args.fault_injection)
    stereo_writer=None;next_stereo=0.;estimator=orb=None
    if getattr(args,'estimator',False) and stereo_hz<=0:raise ValueError('--estimator needs --stereo-hz')
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
        from pxr import Gf,UsdGeom,UsdLux
        from PIL import Image
        from experiments.orbslam3_ros2.isaac_scene import build_isaac_scene
        from experiments.orbslam3_ros2.drive_runtime import DriveSession
        from experiments.site_sim.drive import SimulationDrive
        site=mission.site;c=site.cells([mission.start[:2]])[0];ground=float(site.height_m[c[1],c[0]])
        print('SITE_SIM_STAGE scene_import',flush=True)
        world,h=build_isaac_scene(urdf=args.urdf,scene_usd=args.scene_usd,output=Path(args.output),
            start_position=(*mission.start[:2],ground+.23))
        for _ in range(5):app.update()
        stage=omni.usd.get_context().get_stage()
        # Baked arena already has sunlight and ambient light; lower the combined exposure.
        for prim in stage.Traverse():
            if prim.HasAPI(UsdLux.LightAPI):
                intensity=prim.GetAttribute('inputs:intensity')
                if intensity and intensity.Get() is not None:intensity.Set(float(intensity.Get())*.1)
        if args.sand_texture:
            from pxr import Sdf,UsdShade,Vt
            sand=stage.GetPrimAtPath('/World/SandScene/Sand');pts=np.asarray(UsdGeom.Mesh(sand).GetPointsAttr().Get(),np.float32)
            lo=pts[:,:2].min(0);span=pts[:,:2].max(0)-lo
            print('SITE_SIM_SAND_EXTENT',lo.tolist(),span.tolist(),flush=True)
            # Texture is 0.68 mm/px: 8192x6144 px covers 5.6 x 4.2 m, anchored at the sand corner.
            uv=(pts[:,:2]-lo)/np.array([5.6,4.2],np.float32)
            UsdGeom.PrimvarsAPI(sand).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,UsdGeom.Tokens.vertex).Set(Vt.Vec2fArray.FromNumpy(uv))
            material=UsdShade.Material.Define(stage,'/World/Looks/SandTexture')
            shader=UsdShade.Shader.Define(stage,'/World/Looks/SandTexture/Shader');shader.CreateIdAttr('UsdPreviewSurface')
            reader=UsdShade.Shader.Define(stage,'/World/Looks/SandTexture/StReader');reader.CreateIdAttr('UsdPrimvarReader_float2')
            reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
            texture=UsdShade.Shader.Define(stage,'/World/Looks/SandTexture/Texture');texture.CreateIdAttr('UsdUVTexture')
            texture.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(args.sand_texture))
            texture.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
            texture.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result')
            shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(texture.ConnectableAPI(),'rgb')
            shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(1.);shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(0.)
            material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
            UsdShade.MaterialBindingAPI.Apply(sand).Bind(material)
        overview=UsdGeom.Camera.Define(stage,'/World/ObserverCamera')
        overview.CreateFocalLengthAttr(18.);overview.CreateHorizontalApertureAttr(20.)
        overview.CreateVerticalApertureAttr(15.);overview.CreateClippingRangeAttr(Gf.Vec2f(.1,100))
        center=site.origin+np.asarray(site.observed.shape[::-1])*site.resolution/2
        transform=Gf.Matrix4d().SetLookAt(Gf.Vec3d(float(center[0]),float(center[1])-4.5,5.5),
                                       Gf.Vec3d(float(center[0]),float(center[1]),.35),Gf.Vec3d(0,0,1)).GetInverse()
        overview_op=UsdGeom.Xformable(overview).AddTransformOp();overview_op.Set(transform)
        # The latest URDF already mounts the camera 20 deg down; no optical-frame compensation.
        # Measured D435 IR 640x480 intrinsics (fx=fy=377.186, principal point 320.54/241.73).
        FX,CX,CY=377.186126708984,320.54248046875,241.732162475586
        for path in (h.left_camera_path,h.right_camera_path):
            camera=UsdGeom.Camera(stage.GetPrimAtPath(path))
            camera.GetFocalLengthAttr().Set(FX*20./640)
            camera.GetHorizontalApertureOffsetAttr().Set((320-CX)*20./640)
            camera.GetVerticalApertureOffsetAttr().Set((CY-240)*15./480)
        camera_paths=[('overview','/World/ObserverCamera'),('front',h.left_camera_path)]
        if stereo_hz:camera_paths.append(('right',h.right_camera_path))
        for key,path in camera_paths:
            product=rep.create.render_product(path,(800,600) if key=='overview' else (640,480))
            annotator=rep.AnnotatorRegistry.get_annotator('rgb');annotator.attach(product)
            cameras[key]=(product,annotator)
        if stereo_hz:
            from experiments.site_sim.stereo_capture import StereoCapture
            from experiments.orbslam3_ros2.isaac_scene import _column_world_matrix
            base=_column_world_matrix(stage.GetPrimAtPath(h.frame_paths['base_link']))
            left=_column_world_matrix(stage.GetPrimAtPath(h.frame_paths['camera_infra1_optical_frame']))
            right=_column_world_matrix(stage.GetPrimAtPath(h.frame_paths['camera_infra2_optical_frame']))
            stereo_writer=StereoCapture(Path(args.output)/'stereo',{
                'width':640,'height':480,'fx':FX,'fy':FX,'cx':CX,'cy':CY,'distortion':[0.,0.,0.,0.],
                'T_base_left_optical':(np.linalg.inv(base)@left).tolist(),
                'T_base_right_optical':(np.linalg.inv(base)@right).tolist(),
                'T_left_right':(np.linalg.inv(left)@right).tolist(),
                'requested_hz':stereo_hz,'clock':'simulation relative to control start','timestamp_unit':'ns',
                'intrinsics_source':'configured USD pinhole camera; not real sensor calibration',
                'capture':'both RGB annotators after the same world.step(render=True); no IMU',
                'urdf_sha256':__import__('hashlib').sha256(Path(args.urdf).read_bytes()).hexdigest()})
        if getattr(args,'estimator',False):
            from experiments.localization.online import DEFAULT_NOISE
            imu_world=_column_world_matrix(stage.GetPrimAtPath(h.frame_paths['imu_sensor_frame']))
            base_from_imu=np.linalg.inv(base)@imu_world;base_from_cam=np.linalg.inv(base)@left
            estimator_calibration={'ROS.T_c1_base':np.linalg.inv(base_from_cam).tolist(),
                                   'IMU.T_b_c1':(np.linalg.inv(base_from_imu)@base_from_cam).tolist(),**DEFAULT_NOISE}
        from experiments.site_sim.visuals import install_rover_visuals
        install_rover_visuals(stage,args.urdf)
        robot=world.scene.add(Robot(prim_path=h.frame_paths['base_link'],name='rover'))
        world.set_simulation_dt(physics_dt=.005,rendering_dt=.05)
        print('SITE_SIM_STAGE physics_initialization',flush=True)
        world.initialize_physics();robot.initialize()
        yaw=float(mission.start[2]);robot.set_world_pose(orientation=np.array([math.cos(yaw/2),0,0,math.sin(yaw/2)]))
        session=DriveSession(robot,args.geometry);world.play()
        print('SITE_SIM_STAGE settling',flush=True)
        for i in range(400):
            if time.monotonic()-wall_start>args.max_wall_s:raise TimeoutError('startup wall timeout')
            session.stop();world.step(render=i%20==0)
        p,q=robot.get_world_pose()
        if not np.isfinite(np.r_[p,q]).all() or np.linalg.norm(p[:2]-mission.start[:2])>.1 or not ground+.08<p[2]<ground+.35:
            raise ValueError(f'rover settling failed: {p}')
        if getattr(args,'estimator',False):
            from experiments.localization.online import OnlineEstimator
            from experiments.site_sim.orb_client import OrbClient
            from scipy.spatial.transform import Rotation
            start_rotation=Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix()
            start_pose=np.eye(4);start_pose[:3,:3]=start_rotation;start_pose[:3,3]=p
            estimator=OnlineEstimator(start_pose,estimator_calibration,lag_s=RENDER_LAG_S)
            orb=OrbClient(Path(args.orb_io))
            print('SITE_SIM_ESTIMATOR ready (control uses ORB+ESKF estimate; GT is logged only)',flush=True)
        drive=SimulationDrive(robot,session,mission,fault_injection=args.fault_injection,hold_s=args.start_hold_s,estimator=estimator)
        t0=float(world.current_time);next_control=0.;next_render=0.;next_front=0.;last_camera_error=None
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
            stereo_due=stereo_writer is not None and sim_s+1e-9>=next_stereo
            render_due=stereo_due or time.monotonic()>=next_render
            world.step(render=render_due)
            if estimator is not None:
                gp,gq=robot.get_world_pose()
                estimator.on_step(float(world.current_time)-t0,gp,gq)
            if render_due:
                next_render=time.monotonic()+.2
                # Chase view from above and behind so the small rover stays visible inside the walls.
                rp=np.asarray(row['position_m'],float)
                overview_op.Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(rp[0],rp[1]-.35,rp[2]+2.6),
                    Gf.Vec3d(rp[0],rp[1],rp[2]),Gf.Vec3d(0,0,1)).GetInverse())
                for key,(_,annotator) in cameras.items():
                    try:
                        rgba=np.asarray(annotator.get_data())
                        if rgba.ndim!=3 or rgba.shape[2]<3:continue
                        path=recorder.live/f'{key}.jpg';tmp=path.with_suffix('.tmp')
                        Image.fromarray(rgba[:,:,:3]).save(tmp,format='JPEG',quality=82);tmp.replace(path)
                    except Exception as error:last_camera_error=str(error)
                if args.save_front_s>0 and sim_s>=next_front:
                    next_front=sim_s+args.save_front_s
                    try:
                        rgba=np.asarray(cameras['front'][1].get_data())
                        if rgba.ndim==3 and rgba.shape[2]>=3:
                            (Path(args.output)/'front_frames').mkdir(exist_ok=True)
                            Image.fromarray(rgba[:,:,:3]).save(Path(args.output)/'front_frames'/f'{sim_s:07.2f}.png')
                    except Exception as error:last_camera_error=str(error)
                if stereo_due:
                    left=np.asarray(cameras['front'][1].get_data());right=np.asarray(cameras['right'][1].get_data())
                    if left.ndim!=3 or right.ndim!=3:raise RuntimeError('stereo annotators not ready')
                    rp,rq=robot.get_world_pose()
                    stamp_ns=round((float(world.current_time)-t0)*1e9)
                    stereo_writer.write(stamp_ns,left[:,:,:3],right[:,:,:3],rp.tolist(),rq.tolist())
                    next_stereo+=1./stereo_hz
                    if orb is not None:
                        names=(Path(args.output)/'stereo/mav0/cam0/data'/f'{stamp_ns}.png',Path(args.output)/'stereo/mav0/cam1/data'/f'{stamp_ns}.png')
                        result=orb.track(stamp_ns,*names)
                        estimator.on_frame(stamp_ns*1e-9,result['state'],result['map_id'],result['world_from_camera'])
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
    parser.add_argument('--sand-texture',type=Path,default=None)
    parser.add_argument('--save-front-s',type=float,default=0.)
    parser.add_argument('--stereo-hz',type=float,default=0.)
    parser.add_argument('--start-hold-s',type=float,default=0.)
    parser.add_argument('--estimator',action='store_true')
    parser.add_argument('--orb-io',type=Path,default=Path('/workspace/orbio'))
    parser.add_argument('--fault-injection',action='store_true');args=parser.parse_args();code=1
    try:
        summary=run(args);code=0 if summary['status']=='complete' else 2
    finally:
        Path(args.output).parent.joinpath('exit.txt').write_text(str(code)+'\n')
    raise SystemExit(code)


if __name__=='__main__':main()
