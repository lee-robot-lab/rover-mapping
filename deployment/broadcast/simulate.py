# ================================================================
# simulate.py
# 설명: 실제 gateway·RUN1~4 상태기·MPPI를 조이스틱과 연결하는 로컬 점기구학 로버다.
# 사용법: python deployment/broadcast/launch.py --sim (SPI·ROS·SSH 미사용)
# ================================================================
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import select
import socket
import sys
import time
import uuid
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'deployment/ksrc'))
import runner
from cli import fake_config
from ksrc_runtime.handheld_mapping.control_gateway import Gateway, START, ESTOP, parse_cmd, parse_ctl, newer


class KinematicRover:
    """실기 제어와 분리된 가상 로버로 실제 명령·상태 전이를 이상적 SE2 운동에 적용한다."""

    def __init__(self) -> None:
        self.nav=runner.Navigation(ROOT/'deployment/ksrc/mission',fake_config())
        self.start_pose=np.array(self.nav.new_runtime(1).mission.start,float)
        self.pose=self.start_pose.copy()
        self.time=self.still_since=self.run_origin=self.next_plan=0.
        self.runtime=None;self.active=False;self.command=(0.,0.,0.);self.anchor_count=1
        self.session=uuid.uuid4().hex
        self.anchor_id=1
        self.gateway=Gateway(self.send,on_start=self.start,on_stop=self.stop,on_anchor=self.anchor,
                             on_anchor2=lambda:self.anchor(preset=2),on_anchor3=lambda:self.anchor(preset=3),
                             on_anchor4=lambda:self.anchor(preset=4))

    def send(self, vx: float, vy: float, w: float) -> None:
        self.command=(vx,vy,w)

    def start(self, run: int) -> None:
        self.runtime=self.nav.new_runtime(run);self.runtime.safety=None
        self.run_origin=self.time;self.next_plan=self.time;self.active=True

    def stop(self) -> None:
        self.active=False;self.command=(0.,0.,0.)

    def anchor(self, preset: int = 1) -> None:
        if preset not in (1,2,3,4) or self.active:return
        yaw={1:float(self.nav.mission.start[2]),2:-math.pi,3:math.pi/2,4:0.}[preset]
        error=math.atan2(math.sin(self.pose[2]-yaw),math.cos(self.pose[2]-yaw))
        if (self.time-self.still_since>=3. and np.linalg.norm(self.pose[:2]-self.start_pose[:2])<.15
                and abs(error)<=math.radians(8)):
            self.anchor_id=self.nav.anchor_id=preset;self.nav.anchor_yaw=yaw
            self.start_pose[2]=yaw
            self.pose=self.start_pose.copy();self.anchor_count+=1

    def status(self) -> dict:
        angle=math.atan2(math.sin(self.pose[2]-self.start_pose[2]),math.cos(self.pose[2]-self.start_pose[2]))
        aligned=np.linalg.norm(self.pose[:2]-self.start_pose[:2])<=.08 and abs(angle)<=math.radians(8)
        rt=self.runtime
        return dict(site_identity=self.nav.mission.site.identity,stationary_s=self.time-self.still_since,orb_ok=True,in_region1=bool(aligned),
                    start_aligned=bool(aligned),active=self.active,run=self.gateway.run_id,
                    stage=rt.stage if rt else 'ready',reason=rt.state if rt else 'simulation_ready',
                    done=bool(rt and not self.active),recovering=False,anchor_count=self.anchor_count,anchor_id=self.anchor_id)

    def step(self, dt: float) -> None:
        self.time+=dt
        if self.active and self.time>=self.next_plan:
            stamp=self.time-self.run_origin;rt=self.runtime
            sample=dict(stamp_s=stamp,frame='site',site_identity=rt.mission.site.identity,
                        position_m=[*self.pose[:2],0.],yaw_rad=float(self.pose[2]),tracking_valid=True,
                        tracking_quality='good',filter_status='ok',roll_rad=0.,pitch_rad=0.)
            result=rt.update(sample,now_s=stamp);self.next_plan=self.time+.1
            if rt.state in ('complete','failed'):self.stop()
            else:
                cmd=result['command']
                self.gateway.auto((cmd['vx_m_s'],cmd.get('vy_m_s',0.),cmd['yaw_rate_rad_s']),self.time)
        self.gateway.runner_status(self.status(),self.time)
        self.gateway.tick(self.time)
        vx,vy,w=self.command
        if max(abs(vx),abs(vy),abs(w))>.005:self.still_since=self.time
        mid=self.pose[2]+w*dt*.5
        self.pose[:2]+=dt*np.array([vx*math.cos(mid)-vy*math.sin(mid),vx*math.sin(mid)+vy*math.cos(mid)])
        self.pose[2]=math.atan2(math.sin(self.pose[2]+w*dt),math.cos(self.pose[2]+w*dt))

    def packet(self, seq: int) -> dict:
        status=self.status()
        return dict(seq=seq,session_id=self.session,timestamp=self.time,x=float(self.pose[0]),y=float(self.pose[1]),
                    site_identity=status.get('site_identity'),yaw=float(self.pose[2]),v=self.command[0],w=self.command[2],
                    speed_m_s=math.hypot(*self.command[:2]),vy_m_s=self.command[1],
                    operator={key:status[key] for key in ('active','run','stage','anchor_count','anchor_id')})


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('--udp-port',type=int,default=9870)
    parser.add_argument('--joystick-port',type=int,default=5006)
    args=parser.parse_args()
    sim=KinematicRover();sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1',args.joystick_port));output=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    last=dict(cmd=-1,ctl=-1);gui=None;seq=0;next_tick=time.monotonic();next_status=0.
    print('KINEMATIC SIM: RUN entry, no wheel/contact physics; joystick localhost:'+str(args.joystick_port),flush=True)
    try:
        while True:
            ready,_,_=select.select([sock],[],[],max(0.,min(.02,next_tick-time.monotonic())))
            if ready:
                data,address=sock.recvfrom(4096);gui=address
                cmd=parse_cmd(data);ctl=parse_ctl(data)
                if cmd and newer(cmd[0],last['cmd']):
                    last['cmd']=cmd[0];sim.gateway.teleop(cmd[1],sim.time)
                if ctl and newer(ctl[0],last['ctl']):
                    last['ctl']=ctl[0];sim.gateway.control(ctl[1],ctl[2],sim.time)
            now=time.monotonic()
            if now>=next_tick:
                sim.step(.02);next_tick=max(next_tick+.02,now)
            if sim.time>=next_status:
                next_status=sim.time+.2;seq+=1
                output.sendto(json.dumps(sim.packet(seq)).encode(),('127.0.0.1',args.udp_port))
                if gui:sock.sendto(json.dumps(sim.gateway.snapshot(sim.time)).encode(),gui)
    except KeyboardInterrupt:pass
    finally:sim.stop();sock.close();output.close()


if __name__=='__main__':main()
