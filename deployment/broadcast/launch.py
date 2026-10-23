# ================================================================
# launch.py
# 설명: 노트북에서 중계 서버·읽기 전용 Pi 상태 relay·조이스틱을 함께 실행한다.
# 사용법: python deployment/broadcast/launch.py --pi pi@192.168.0.101
# ================================================================
from __future__ import annotations
import argparse
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.request import urlopen
import webbrowser
ROOT=Path(__file__).resolve().parents[2]


def commands(args: argparse.Namespace) -> list[list[str]]:
    python=sys.executable
    server=[python,'-m','experiments.broadcast_demo.server','--http-port',str(args.http_port),
            '--udp-port',str(args.udp_port)]
    if getattr(args,'hide_stones',False):server+=['--hide-stones']
    if args.demo:return [server]
    server+=['--live']
    simulation=getattr(args,'sim',False)
    if simulation:server+=['--simulation']
    relay=[python,str(ROOT/'deployment/broadcast/relay.py'),'--pi',args.pi,
           '--remote-root',args.remote_root,'--udp-port',str(args.udp_port)]
    if args.state_dir:relay+=['--state-dir',args.state_dir]
    host='127.0.0.1' if simulation else args.pi.rsplit('@',1)[-1]
    if simulation:
        relay=[python,str(ROOT/'deployment/broadcast/simulate.py'),'--udp-port',str(args.udp_port)]
    joystick=[python,str(ROOT/'teleop/ksrc_snapshot/pi/teleop/teleop_joystick.py'),
              '--udp',host+':5006','--gateway','--grid-keys','--rate','50',
              '--max-lin','0.15','--max-ang','0.8']
    return [server,relay,joystick]


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--pi',default='pi@192.168.0.101')
    parser.add_argument('--remote-root',default='/home/pi/ksrc_final_20261022/deployment/ksrc')
    parser.add_argument('--state-dir',default='')
    parser.add_argument('--http-port',type=int,default=8765);parser.add_argument('--udp-port',type=int,default=9870)
    parser.add_argument('--hide-stones',action='store_true',help='돌 표시만 숨김; MPPI·코스트 계산은 유지')
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--demo',action='store_true',help='로버 없이 시뮬 재생 화면만 실행')
    mode.add_argument('--sim',action='store_true',help='로컬 기구학 로버 + 실제 MPPI + 조이스틱 + 사이트 실행')
    parser.add_argument('--no-browser',action='store_true');parser.add_argument('--dry-run',action='store_true')
    args=parser.parse_args();jobs=commands(args)
    if args.dry_run:
        for command in jobs:print(subprocess.list2cmdline(command))
        return
    processes=[]
    try:
        processes.append(subprocess.Popen(jobs[0],cwd=ROOT))
        deadline=time.monotonic()+15
        while True:
            if processes[0].poll() is not None:raise RuntimeError('broadcast server exited; see log above')
            try:
                with urlopen(f'http://127.0.0.1:{args.http_port}/api/meta',timeout=1):break
            except OSError:
                if time.monotonic()>deadline:raise RuntimeError('broadcast server startup timed out')
                time.sleep(.15)
        for command in jobs[1:]:processes.append(subprocess.Popen(command,cwd=ROOT))
        url=f'http://127.0.0.1:{args.http_port}'
        print('중계 화면:',url,flush=True)
        if not args.no_browser:webbrowser.open(url)
        while processes[0].poll() is None:
            if args.sim and processes[1].poll() is not None:
                raise RuntimeError('kinematic simulator exited; see log above')
            if not args.demo and processes[-1].poll() is not None:break
            time.sleep(.2)
    except KeyboardInterrupt:pass
    finally:
        for process in reversed(processes):
            if process.poll() is None:process.send_signal(signal.SIGINT)
        for process in reversed(processes):
            try:process.wait(timeout=4)
            except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=3)


if __name__=='__main__':main()
