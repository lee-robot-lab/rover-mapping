# ================================================================
# relay.py
# 설명: Pi의 실제 주행 status.json을 SSH로 읽어 위치·추정 속도·수락된 회차 상태만 중계한다.
# 사용법: python deployment/broadcast/relay.py --pi pi@192.168.0.101
# ================================================================
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import shlex
import socket
import subprocess
import sys
import time
import uuid

REMOTE = '''import json,time,sys
from pathlib import Path
root=Path(sys.argv[1]);given=sys.argv[2];previous=None
while True:
 try:
  if given:folder=Path(given);folder=folder if folder.is_absolute() else root/folder
  else:
   pointer=root/'state/latest_test.txt'
   folder=Path(pointer.read_text().strip()) if pointer.exists() else Path('state')
   if not folder.is_absolute():folder=root/folder
  path=folder/'status.json';stamp=path.stat().st_mtime_ns
  if stamp!=previous and not (folder/'stopped.json').exists():
   data=json.loads(path.read_text());data['_file_age_s']=max(0,time.time()-path.stat().st_mtime)
   data['_source_session']=str(folder);print(json.dumps(data),flush=True);previous=stamp
 except (OSError,ValueError):pass
 time.sleep(.05)
'''


class Telemetry:
    """상태 파일에서 제어 명령 대신 연속 pose 차분으로 이동 속도를 추정한다."""

    def __init__(self) -> None:
        self.previous = None
        self.source = None
        self.session_id = uuid.uuid4().hex
        self.seq = 0

    def packet(self, status: dict) -> dict | None:
        try:
            if (float(status['_file_age_s'])>.6 or not status['orb_ok']
                    or status.get('pose_age_s') is None or float(status['pose_age_s'])>.5):
                self.previous=None
                return None
            pose=status['pose'];stamp=float(status['stamp_s'])
            x,y=float(pose[0][3]),float(pose[1][3]);yaw=math.atan2(float(pose[1][0]),float(pose[0][0]))
            if not all(math.isfinite(a) for a in (x,y,yaw,stamp)):return None
            source=status['_source_session']
            if source!=self.source:
                self.previous=None;self.source=source;self.session_id=uuid.uuid4().hex;self.seq=0
            v=w=speed=vy=0.
            anchor=int(status.get('anchor_count',0))
            if self.previous:
                px,py,pa,pt,previous_anchor=self.previous;dt=stamp-pt
                if dt==0:return None
                if dt<0:
                    self.session_id=uuid.uuid4().hex;self.seq=0
                elif dt<=.6 and anchor==previous_anchor:
                    dx,dy=x-px,y-py
                    if math.hypot(dx,dy)>.25:
                        self.previous=(x,y,yaw,stamp,anchor)
                        return None
                    v=(dx*math.cos(yaw)+dy*math.sin(yaw))/dt
                    vy=(-dx*math.sin(yaw)+dy*math.cos(yaw))/dt
                    speed=math.hypot(dx,dy)/dt
                    w=math.atan2(math.sin(yaw-pa),math.cos(yaw-pa))/dt
            self.previous=(x,y,yaw,stamp,anchor);self.seq+=1
            return dict(seq=self.seq,session_id=self.session_id,timestamp=time.time(),x=x,y=y,yaw=yaw,
                        site_identity=status.get('site_identity'),v=v,w=w,vy_m_s=vy,speed_m_s=speed,velocity_source='pose_difference',
                        operator=dict(active=bool(status.get('active')),run=int(status.get('run',0)),
                                      anchor_count=anchor,done=bool(status.get('done')),
                                      anchor_id=int(status.get('anchor_id',1)),
                                      stage=str(status.get('stage',''))))
        except (KeyError,TypeError,ValueError,IndexError,OverflowError):return None


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('--pi',required=True)
    parser.add_argument('--remote-root',default='/home/pi/ksrc_final_20261022/deployment/ksrc')
    parser.add_argument('--state-dir',default='');parser.add_argument('--udp-port',type=int,default=9870)
    args=parser.parse_args();sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);telemetry=Telemetry()
    remote='python3 -u - '+shlex.quote(args.remote_root)+' '+shlex.quote(args.state_dir)
    while True:
        process=subprocess.Popen(['ssh','-o','BatchMode=yes','-o','HostKeyAlias=raspberrypi.local','-o','ConnectTimeout=5',
                                  '-o','ServerAliveInterval=3','-o','ServerAliveCountMax=2',args.pi,remote],
                                  stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
        try:
            process.stdin.write(REMOTE);process.stdin.close()
            for line in process.stdout:
                try:packet=telemetry.packet(json.loads(line))
                except ValueError:continue
                if packet:sock.sendto(json.dumps(packet,allow_nan=False).encode(),('127.0.0.1',args.udp_port))
        except KeyboardInterrupt:
            process.terminate();process.wait(timeout=3);break
        finally:
            if process.poll() is None:process.terminate();process.wait(timeout=3)
        print('status relay reconnecting',file=sys.stderr,flush=True);time.sleep(1)
    sock.close()


if __name__=='__main__':main()
