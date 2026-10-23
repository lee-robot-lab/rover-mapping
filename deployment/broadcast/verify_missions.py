# ================================================================
# verify_missions.py
# 설명: 로컬 가상 로버의 RUN1~4를 실제 UDP→HTTP 중계와 연결해 완주·캐시 표시·조회 시간을 검증한다.
# 사용법: OPENBLAS_NUM_THREADS=1 python deployment/broadcast/verify_missions.py
# ================================================================
from __future__ import annotations
from http.server import ThreadingHTTPServer
import argparse
import math
import json
from pathlib import Path
import socket
import sys
from threading import Thread
import time
from urllib.request import urlopen
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT));sys.path.insert(0,str(Path(__file__).parent))
from simulate import KinematicRover, START
from experiments.broadcast_demo.server import Broadcast, Handler
from experiments.broadcast_demo.paths import DEFAULT_OUT


def main() -> None:
    parser=argparse.ArgumentParser()
    parser.add_argument('--anchors',type=int,nargs='+',choices=(1,2,3,4),default=[1,2,3,4])
    args=parser.parse_args()
    app=Broadcast(DEFAULT_OUT,'127.0.0.1',0,live=True)
    server=ThreadingHTTPServer(('127.0.0.1',0),Handler);server.app=app
    thread=Thread(target=server.serve_forever,daemon=True);thread.start()
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    reports={};seq=0;latencies=[]
    try:
        for anchor,run in [(a,r) for a in args.anchors for r in range(1,5)]:
            sim=KinematicRover()
            sim.pose[2]={1:-math.pi/2,2:-math.pi,3:math.pi/2,4:0.}[anchor]
            sim.step(3.1);sim.anchor(preset=anchor);sim.step(.02)
            assert sim.anchor_id==anchor
            began=sim.time
            assert sim.gateway.control(START,(run,0.,0.),sim.time)
            counts=dict(checked=0,matched=0);stages=[]
            # 회차마다 송신 ID를 바꾸며 재접속 이후 기존 기록 보존도 확인한다.
            for index in range(5000):
                sim.step(.1)
                if not stages or stages[-1]!=sim.runtime.stage:stages.append(sim.runtime.stage)
                if index%20==0 or not sim.active:
                    seq+=1;packet=sim.packet(seq)
                    sock.sendto(json.dumps(packet).encode(),app.socket.getsockname())
                    deadline=time.monotonic()+2
                    while app.session.raw is None or app.session.raw['seq']!=seq:
                        if time.monotonic()>deadline:raise RuntimeError('UDP reception timeout')
                        time.sleep(.001)
                    before=time.perf_counter()
                    with urlopen(f'http://127.0.0.1:{server.server_address[1]}/api/state',timeout=2) as response:
                        state=json.load(response)
                    latencies.append((time.perf_counter()-before)*1000)
                    assert state['run']==run and abs(state['pose']['x']-sim.pose[0])<1e-5
                    assert state['map_match'] is True
                    if state['active'] and sim.runtime.stage in app.mission_data['stages']:
                        counts['checked']+=1;counts['matched']+=int(state['prediction'] is not None)
                    assert len(state['tracks'])==4
                if not sim.active:break
            assert sim.runtime.state=='complete',(run,sim.runtime.state,sim.runtime.stage)
            if run in (3,4):
                assert stages[-3:]==['descent15_entry','descent15','return15']
                assert sim.runtime.route_status['15']['down']=='succeeded' and 'return10' not in stages
            counts['match_rate']=counts['matched']/max(1,counts['checked'])
            assert counts['match_rate']>.75,(run,counts)
            group=reports if anchor==1 else reports.setdefault(f'anchor{anchor}',{})
            group[str(run)]=dict(state=sim.runtime.state,sim_s=sim.time-began,stages=stages,route_status=sim.runtime.route_status,**counts)
            print('ANCHOR',anchor,'RUN',run,group[str(run)],flush=True)
        reports['http_ms']=dict(p50=float(np.percentile(latencies,50)),p95=float(np.percentile(latencies,95)),
                                maximum=max(latencies),samples=len(latencies))
        destination=ROOT/'outputs/run_variants/broadcast_missions.json';destination.parent.mkdir(parents=True,exist_ok=True)
        destination.write_text(json.dumps(reports,indent=2));print(json.dumps(reports,indent=2),flush=True)
    finally:
        sock.close();server.shutdown();server.server_close();thread.join(timeout=1);app.close()


if __name__=='__main__':main()
