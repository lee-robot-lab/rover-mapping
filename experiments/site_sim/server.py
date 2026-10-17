# ================================================================
# server.py
# 설명: 서버 GPU1 임무 run의 제한된 upload·launch·status·tunnel·stop 명령을 실행한다.
# 사용법: bash tools/run_site_sim_server.sh upload LABEL MISSION_FOLDER
# ================================================================
from __future__ import annotations

import argparse
import hashlib
import io
import math
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tarfile
import tempfile
from xml.etree import ElementTree

ROOT='/home/sumee/gyusa/runs/site_mission'
ARENA='/home/sumee/gyusa/arena_sand_v15_m5.usd'
IMAGE='nvcr.io/nvidia/isaac-sim:6.1.0'
SSH=['ssh','-o','BatchMode=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=15','hpi-lab']
REPO=Path(__file__).resolve().parents[2]


def remote(command: str, *, capture: bool=False, stdin=None) -> subprocess.CompletedProcess:
    return subprocess.run([*SSH,command],check=True,text=True,capture_output=capture,stdin=stdin)


def upload(label: str, mission: Path, assets: Path, texture: Path | None=None) -> None:
    """필요한 소스와 asset만 새 run에 기록하고 hash manifest를 동봉한다."""
    if not (mission/'mission.json').is_file():raise ValueError('valid mission bundle required')
    from experiments.handheld_mapping.site_mission import SiteMission
    SiteMission.load(mission,load_mesh=False)
    urdf=assets/'rover_portable.urdf';geometry=assets/'config/swerve_geometry.json'
    if not urdf.is_file() or not geometry.is_file():raise ValueError('portable rover assets required')
    referenced={node.get('filename') for node in ElementTree.parse(urdf).findall('.//mesh')}
    for name in referenced:
        path=(assets/name).resolve()
        if not path.is_relative_to(assets.resolve()) or not path.is_file():raise ValueError('missing/outside URDF mesh')
    hashes={};run=f'{ROOT}/{label}'
    with tempfile.TemporaryFile() as archive:
        with tarfile.open(fileobj=archive,mode='w:gz') as tar:
            for package in ('handheld_mapping','site_sim','orbslam3_ros2','orbslam3_offline','localization'):
                for path in sorted((REPO/'experiments'/package).rglob('*')):
                    if path.suffix not in ('.py','.html') or '__pycache__' in path.parts:continue
                    relative=str(path.relative_to(REPO));tar.add(path,arcname=relative)
                    hashes[relative]=hashlib.sha256(path.read_bytes()).hexdigest()
            tar.add(mission,arcname='mission')
            tar.add(urdf,arcname='rover/rover_portable.urdf');tar.add(geometry,arcname='rover/swerve_geometry.json')
            if texture is not None:
                if not texture.is_file():raise ValueError('sand texture file not found')
                tar.add(texture,arcname='rover/sand_texture.jpg');hashes['rover/sand_texture.jpg']=hashlib.sha256(texture.read_bytes()).hexdigest()
            for name in sorted(referenced):tar.add(assets/name,arcname='rover/'+name)
            commit=subprocess.run(['git','rev-parse','HEAD'],cwd=REPO,text=True,capture_output=True,check=True).stdout.strip()
            body=json.dumps({'code_commit':commit,'source_sha256':hashes,'source_assets':str(assets.resolve()),
                'stl_and_sand_provenance':json.loads((mission/'simulation.json').read_text())}).encode()
            info=tarfile.TarInfo('source_manifest.json');info.size=len(body);tar.addfile(info,io.BytesIO(body))
        archive.seek(0)
        remote(f'mkdir -p {shlex.quote(ROOT)} && mkdir {shlex.quote(run)} && tar -xzf - -C {shlex.quote(run)}',stdin=archive)
    print('uploaded:',run)


def start(label: str, fault: bool, sim: float, wall: float, port: int, gpu: int=1, save_front_s: float=0., stereo_hz: float=0., start_hold_s: float=0.) -> None:
    """지정한 유휴 GPU에서만 새 물리 run과 독립 loopback viewer를 시작한다."""
    from experiments.site_sim.run import validate_limits
    validate_limits(sim,wall)
    if not math.isfinite(stereo_hz) or not 0<=stereo_hz<=30:raise ValueError("stereo_hz must be in [0,30]")
    run=f'{ROOT}/{label}';name='rover-site-'+label
    remote(f'test -d {shlex.quote(run)} && test ! -e {shlex.quote(run+"/output")}')
    if gpu not in (0,1):raise ValueError('GPU index must be 0 or 1')
    gpu_before=remote(f'nvidia-smi --query-gpu=memory.free,utilization.gpu --format=csv,noheader,nounits -i {gpu}',capture=True).stdout.strip()
    free,util=(int(v.strip()) for v in gpu_before.split(','))
    if free<12000 or util>20:raise RuntimeError(f'GPU{gpu} free memory {free}MiB / utilization {util}%: refusing launch')
    # Fail before starting either process if the requested observer port is unavailable.
    probe=f'import socket; s=socket.socket(); s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); s.bind(("127.0.0.1",{port})); s.close()'
    remote(shlex.join(['python3','-c',probe]))
    args=['/isaac-sim/kit/python/bin/python3','-m','experiments.site_sim.run','--mission','/workspace/mission',
        '--urdf','/workspace/rover/rover_portable.urdf','--geometry','/workspace/rover/swerve_geometry.json',
        '--scene-usd','/arena/arena.usd','--output','/workspace/output',
        '--max-sim-s',str(sim),'--max-wall-s',str(wall)]
    if fault:args.append('--fault-injection')
    if save_front_s>0:args+=['--save-front-s',str(save_front_s)]
    if stereo_hz>0:args+=['--stereo-hz',str(stereo_hz)]
    if start_hold_s>0:args+=['--start-hold-s',str(start_hold_s)]
    if remote(f'test -f {shlex.quote(run+"/rover/sand_texture.jpg")} && echo yes || echo no',capture=True).stdout.strip()=='yes':
        args+=['--sand-texture','/workspace/rover/sand_texture.jpg']
    # Match the installed python.sh environment but exec Python as PID1 for SIGTERM/finally.
    inside=('export CARB_APP_PATH=/isaac-sim/kit ISAAC_PATH=/isaac-sim EXP_PATH=/isaac-sim/apps\n'
            'source /isaac-sim/setup_python_env.sh\n'
            'export LD_PRELOAD=/isaac-sim/kit/libcarb.so${LD_PRELOAD:+:$LD_PRELOAD}\n'
            'exec '+shlex.join(args)+' > /workspace/run.log 2>&1')
    docker=['docker','run','-d','--name',name,'--user','0:0','--gpus',f'device={gpu}','--shm-size=2g',
        '-e','ACCEPT_EULA=Y','-e','PRIVACY_CONSENT=Y','-e','OMNI_KIT_ALLOW_ROOT=1',
        '-e','PYTHONPATH=/workspace','-e','PYTHONNOUSERSITE=1','-e','OMP_NUM_THREADS=4',
        '-e','OPENBLAS_NUM_THREADS=1','-e','MPLCONFIGDIR=/workspace/.mpl',
        '-v',run+':/workspace','-v',run+'/rover:/workspace/rover:ro','-v',ARENA+':/arena/arena.usd:ro',
        '-w','/workspace','--entrypoint','bash',IMAGE,'-lc',inside]
    launch={'container':name,'gpu':gpu,'observer_port':port,'fault_injection':fault,
            'max_sim_s':sim,'max_wall_s':wall,'gpu_before':gpu_before,'argv':docker}
    write='from pathlib import Path; Path('+repr(run+'/launch.json')+').write_text('+repr(json.dumps(launch,indent=2))+')'
    remote(shlex.join(['python3','-c',write]))
    remote(f'docker image inspect {shlex.quote(IMAGE)} --format "{{{{.Id}}}}" > {run}/image_id.txt')
    remote(shlex.join(docker))
    remote(f'nohup docker wait {name} > {run}/exit.txt 2> {run}/exit-wait.log < /dev/null &')
    viewer=['python3','-m','experiments.site_sim.viewer','--output',run+'/output','--port',str(port)]
    remote(f'cd {shlex.quote(run)}\nnohup {shlex.join(viewer)} > {run}/viewer.log 2>&1 < /dev/null &\necho $! > {run}/viewer.pid')
    print('started:',run,'container:',name,'observer port:',port)


def status(label: str) -> None:
    run=f'{ROOT}/{label}'
    remote(f'docker ps -a -f name=^rover-site-{label}$ --format "{{{{.Status}}}}"; '
           f'if test -f {run}/output/summary.json; then cat {run}/output/summary.json; '
           f'elif test -f {run}/output/live/state.json; then tail -c 1600 {run}/output/live/state.json; fi; '
           f'tail -4 {run}/run.log 2>/dev/null | cut -c1-250')


def tunnel(label: str, local_port: int) -> None:
    run=f'{ROOT}/{label}'
    launch=json.loads(remote('cat '+run+'/launch.json',capture=True).stdout)
    directory=Path.home()/'.cache/rover-site-sim';directory.mkdir(parents=True,exist_ok=True)
    pidfile=directory/f'{label}.pid'
    if pidfile.exists():
        try:record=json.loads(pidfile.read_text())
        except ValueError:record=None
        if isinstance(record,dict) and process_identity(record['pid'])==record:raise RuntimeError('tunnel already running')
    log=(directory/f'{label}.log').open('ab')
    command=['ssh','-o','BatchMode=yes','-o','ExitOnForwardFailure=yes','-o','ServerAliveInterval=15','-o','ServerAliveCountMax=40','-N',
        '-L',f'127.0.0.1:{local_port}:127.0.0.1:{int(launch["observer_port"])}','hpi-lab']
    process=subprocess.Popen(command,stdin=subprocess.DEVNULL,stdout=log,stderr=log,start_new_session=True)
    pidfile.write_text(json.dumps(process_identity(process.pid)));log.close()
    print(f'http://127.0.0.1:{local_port} (tunnel pid {process.pid})')


def process_identity(pid: int) -> dict | None:
    """PID 재사용을 구분하는 시작 tick과 정확한 argv를 함께 읽는다."""
    try:
        root=Path(f'/proc/{pid}')
        return {'pid':pid,'start_ticks':int((root/'stat').read_text().rsplit(')',1)[1].split()[19]),
                'argv':(root/'cmdline').read_bytes().decode().split('\0')}
    except (OSError,ValueError,IndexError):return None


def stop(label: str) -> None:
    run=f'{ROOT}/{label}'
    remote(f'docker stop --time 20 rover-site-{label}')
    # Verify the recorded PID still belongs to this exact observer before stopping it.
    terminate=f'''import os,signal
from pathlib import Path
pidfile=Path({run+'/viewer.pid'!r})
if pidfile.is_file():
    pid=int(pidfile.read_text())
    command=Path('/proc/'+str(pid)+'/cmdline')
    if command.is_file():
        argv=command.read_bytes().split(b'\\0')
        if b'experiments.site_sim.viewer' in argv and {run+'/output'!r}.encode() in argv:
            os.kill(pid,signal.SIGTERM)
'''
    remote(shlex.join(['python3','-c',terminate]))
    pidfile=Path.home()/'.cache/rover-site-sim'/f'{label}.pid'
    if pidfile.is_file():
        try:
            record=json.loads(pidfile.read_text())
            if isinstance(record,dict) and process_identity(record['pid'])==record:
                os.kill(record['pid'],signal.SIGTERM)
        except (ProcessLookupError,FileNotFoundError,ValueError,KeyError):pass
        pidfile.unlink()


def main(argv: list[str] | None=None) -> int:
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=('upload','start','status','tunnel','stop'))
    parser.add_argument('label');parser.add_argument('path',nargs='?');parser.add_argument('--fault-injection',action='store_true')
    parser.add_argument('--max-sim-s',type=float,default=900.);parser.add_argument('--max-wall-s',type=float,default=3600.)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--gpu',type=int,choices=(0,1),default=1)
    parser.add_argument('--assets',type=Path,default=REPO/'outputs/site_sim/rover_assets_new')
    parser.add_argument('--texture',type=Path,default=None)
    parser.add_argument('--save-front-s',type=float,default=0.)
    parser.add_argument('--stereo-hz',type=float,default=0.)
    parser.add_argument('--start-hold-s',type=float,default=0.)
    args=parser.parse_args(argv)
    try:
        if not re.fullmatch('[A-Za-z0-9_-]+',args.label):raise ValueError('invalid label')
        if not 1024<=args.port<=65535:raise ValueError('invalid observer port')
        if args.action=='upload':
            if not args.path:raise ValueError('mission folder required')
            upload(args.label,Path(args.path).resolve(),args.assets,args.texture)
        elif args.action=='start':start(args.label,args.fault_injection,args.max_sim_s,args.max_wall_s,args.port,args.gpu,args.save_front_s,args.stereo_hz,args.start_hold_s)
        elif args.action=='status':status(args.label)
        elif args.action=='tunnel':tunnel(args.label,int(args.path or args.port))
        else:stop(args.label)
    except ValueError as error:print(error,file=sys.stderr);return 2
    except (OSError,RuntimeError,subprocess.CalledProcessError) as error:print(error,file=sys.stderr);return 1
    return 0


if __name__=='__main__':raise SystemExit(main())
