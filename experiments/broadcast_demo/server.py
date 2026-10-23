# ================================================================
# server.py
# 설명: UDP pose를 기록하고 최신 상태·사전 MPPI·지형을 브라우저 중계 화면에 제공한다.
# 사용법: python -m experiments.broadcast_demo.server --http-port 8765 --udp-port 9870
# ================================================================
from __future__ import annotations
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import socket
import threading
import subprocess
import sys
import time
import numpy as np
from experiments.broadcast_demo.core import Session, Lookup, decode_packet
from experiments.broadcast_demo.paths import DEFAULT_OUT, ROOT
from deployment.broadcast.teleop_prediction import TeleopPrediction
from ksrc_runtime.handheld_mapping.site_mission import SiteMission

REVEAL_RADIUS_M = .35


class Broadcast:
    """네트워크 수신과 화면 조회를 분리하고 원본 패킷·각 RUN 기록을 보존한다."""

    def __init__(self, root: Path, udp_host: str, udp_port: int, live: bool = False, hide_stones: bool = False) -> None:
        self.root = root
        self.meta = json.loads((root/'manifest.json').read_text())
        self.meta['live']=live
        self.hide_stones=hide_stones
        self.meta['hide_stones']=hide_stones
        self.live=live
        self.phase=self.meta.get('phase','transfer')
        self.operator=None
        site=SiteMission.load(ROOT/'deployment/ksrc/mission',load_mesh=False).site if live else None
        self.site_identity=site.identity if site else None
        self.teleop_prediction=TeleopPrediction(site) if live else None
        if live and self.meta.get('planning_map'):
            coverage=json.loads((root/'mission_coverage.json').read_text())
            if any(identity!=self.site_identity for identity in (self.meta['planning_map']['site_identity'],coverage.get('site_identity'))):
                raise ValueError('planning map / terrain / mission cache mismatch; rebuild broadcast assets')
            self.meta['mission_coverage']=coverage
        with np.load(root/'predictions.npz') as stored:
            self.data = {key: stored[key] for key in stored.files}
        self.prediction_data = self.data
        if (root/'teleop_cache.npz').exists():
            with np.load(root/'teleop_cache.npz') as stored:
                self.prediction_data = {key: stored[key] for key in stored.files}
        terrain = np.load(root/'terrain.npz')
        self.cost, self.cell, self.origin = terrain['cost'], float(terrain['cell']), terrain['origin']
        ij = np.indices(self.cost.shape).reshape(2, -1).T
        self.grid_points = self.origin + ij*self.cell
        self.explored = np.zeros((4, len(ij)), dtype=bool)
        self.teleop_explored=np.zeros(len(ij),dtype=bool)
        self.session = Session(tuple(self.meta['start']))
        self.lookup = Lookup(self.prediction_data['states'], self.prediction_data['runs'])
        self.mission_data = None
        if (root/'mission_cache.npz').exists():
            with np.load(root/'mission_cache.npz') as stored:
                self.mission_data = {key: stored[key] for key in stored.files}
            self.mission_lookup = Lookup(self.mission_data['states'],self.mission_data['runs'],self.mission_data['stages'])
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((udp_host, udp_port))
        self.socket.settimeout(.2)
        self.closed = threading.Event()
        self.bad = 0
        self.demo_process = None
        self.demo_lock = threading.RLock()
        self.log_dir = ROOT/'outputs/broadcast_sessions'/f'{time.strftime("%Y%m%d_%H%M%S")}_{time.time_ns() % 10**9:09d}'
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.thread = threading.Thread(target=self.receive, daemon=True)
        self.thread.start()

    def stop_demo(self) -> None:
        with self.demo_lock:
            if self.demo_process is not None and self.demo_process.poll() is None:
                self.demo_process.terminate()
                try:
                    self.demo_process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.demo_process.kill()
                    self.demo_process.wait()

    def play_demo(self, http_port: int) -> None:
        if self.live:raise ValueError('simulation playback is disabled in live mode')
        with self.demo_lock:
            self.stop_demo()
            with self.session.lock:
                self.save()
                self.session.stop()
            duration = float(self.data['times'].max())+1.
            command = [sys.executable, '-m', 'experiments.broadcast_demo.verify_live',
                       '--data', str(self.root.resolve()), '--seconds', str(duration),
                       '--http-port', str(http_port), '--udp-port', str(self.socket.getsockname()[1])]
            with (self.log_dir/'demo_replay.log').open('a') as log:
                self.demo_process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)

    def receive(self) -> None:
        with (self.log_dir/'udp.jsonl').open('a', buffering=1) as log:
            while not self.closed.is_set():
                try:
                    data, _ = self.socket.recvfrom(4096)
                    packet = decode_packet(data)
                    with self.session.lock:
                        operator=packet.get('operator')
                        transition=self.live and operator and self.operator and (
                                operator['anchor_count']!=self.operator['anchor_count']
                                or operator['run']!=self.operator['run'] or operator['active']!=self.operator['active'])
                        accepted = self.session.ingest(packet,record=not transition and (not self.live or bool(operator and operator['active'])))
                        if accepted and self.live and 'operator' in packet:
                            self.sync_operator(packet['operator'])
                            if not operator['active']:
                                self.session.record_teleop()
                                self.remember(teleop=True)
                            else:self.session.break_teleop()
                        annotation = None
                        if accepted and self.session.active:
                            self.remember()
                            pose = self.session.pose()
                            prediction_data,key = self.match_prediction(pose,self.session.run)
                            if key is not None:
                                annotation = dict(cache_index=key,cache_phase=self.phase,
                                    slip_model=float(prediction_data['slips'][key]) if abs(pose['v'])>=.01 else None)
                        log.write(json.dumps(dict(packet, accepted=accepted, run=self.session.run,
                            model=annotation, received_monotonic=time.monotonic()))+'\n')
                except socket.timeout:
                    continue
                except ValueError:
                    self.bad += 1
                except OSError:
                    break

    def remember(self, teleop: bool = False) -> None:
        pose = self.session.pose()
        delta = self.grid_points-[pose['x'], pose['y']]
        mask=self.teleop_explored if teleop else self.explored[self.session.run-1]
        mask |= np.sum(delta*delta, axis=1) <= REVEAL_RADIUS_M**2

    def sync_operator(self, operator: dict) -> None:
        """실기에서 수락된 ANCHOR·회차 상태만 따라가며 실기에 명령을 보내지 않는다."""
        previous=self.operator
        if previous and operator['anchor_count']>previous['anchor_count']:
            self.session.stop()
            self.session.rotation=0.
            self.session.offset[:]=0.
            self.phase='ready'
        if operator['active'] and (not previous or not previous['active'] or operator['run']!=previous['run']):
            self.session.start_pose=tuple(self.meta['run1_entry_pose'])
            self.start(operator['run'],require_history=False,align=False)
            self.phase='run'
        elif operator['active'] and not self.session.active:
            self.session.active=True
            self.session.stopped=None
            self.phase='run'
        elif previous and previous['active'] and not operator['active']:
            self.session.stop()
        elif previous is None and not operator['active'] and operator['anchor_count']==0:
            self.session.align_to_start()
        elif previous is None and not operator['active']:
            self.phase='ready'
        self.operator=dict(operator)

    def start(self, run: int, require_history: bool = True, align: bool = True) -> None:
        with self.session.lock:
            self.session.start(run,require_history=require_history,align=align)
            if run == 1:
                self.explored[:] = False
            else:
                self.explored[run-1] = False
            self.remember()

    def state(self) -> dict:
        result = self.session.snapshot()
        result['bad_packets'] = self.bad
        result['phase']=self.phase
        result['operator']=self.operator
        result['demo_playing'] = self.demo_process is not None and self.demo_process.poll() is None
        result['site_identity']=self.site_identity
        incoming=(self.session.raw or {}).get('site_identity')
        result['map_match']=None if incoming is None else incoming==self.site_identity
        result['prediction'] = None
        result['cells'] = []
        result['obstacles'] = []
        with self.session.lock:
            result['explored'] = [np.column_stack((self.grid_points[mask], self.cost.ravel()[mask])).tolist()
                                  for mask in self.explored]
            result['teleop_explored']=np.column_stack((self.grid_points[self.teleop_explored],self.cost.ravel()[self.teleop_explored])).tolist()
        pose = result['pose']
        if pose:
            xy = np.array([pose['x'], pose['y']])
            planning=self.meta.get('planning_map',{})
            result['obstacles']=[dict(x=x,y=y,diameter_m=planning['stone_diameter_m'])
                for x,y in planning.get('stone_centers_xy_m',[])
                if not self.hide_stones and not result['stale'] and result['map_match'] is not False
                and np.linalg.norm(np.asarray([x,y])-xy)<=REVEAL_RADIUS_M]
            index = np.floor((xy-self.origin)/self.cell).astype(int)
            radius_cells = int(np.ceil(REVEAL_RADIUS_M/self.cell))
            for i in range(max(0, index[0]-radius_cells), min(self.cost.shape[0], index[0]+radius_cells+1)):
                for j in range(max(0, index[1]-radius_cells), min(self.cost.shape[1], index[1]+radius_cells+1)):
                    p = self.origin+np.array([i, j])*self.cell
                    if np.linalg.norm(p-xy) <= REVEAL_RADIUS_M:
                        result['cells'].append([*p.tolist(), float(self.cost[i, j])])
            if not result['stale'] and result['map_match'] is not False and (result['active'] or self.live):
                prediction_data,key = self.match_prediction(pose,result['run'])
                if key is not None:
                    result['prediction'] = dict(candidates=prediction_data['candidates'][key].tolist(),
                        selected=prediction_data['chosen'][key].tolist(), costs=prediction_data['costs'][key].tolist(),
                        slip=float(prediction_data['slips'][key]) if abs(pose['v'])>=.01 else None,
                        simulated_compute_ms=float(prediction_data['compute_ms'][key]),
                        lookup_distance_m=float(np.linalg.norm(prediction_data['states'][key, :2]-xy)),source='offline_cache')
                elif self.live and self.operator and not self.operator['active']:
                    result['prediction']=self.teleop_prediction.predict(pose)
        return result

    def match_prediction(self, pose: dict, run: int) -> tuple:
        """접근 구간과 본 주행을 분리하고 본 주행은 같은 단계의 결과만 선택한다."""
        if self.live and pose.get('site_identity',self.site_identity)!=self.site_identity:return None,None
        state=[pose[x] for x in ('x','y','yaw','v','w')]
        if self.phase=='transfer':return self.prediction_data,self.lookup.find(state,1)
        if self.phase=='run' and self.mission_data is not None and (not self.live or (self.operator or {}).get('active')):
            stage=(self.operator or {}).get('stage')
            return self.mission_data,self.mission_lookup.find(state,run,stage=stage)
        return None,None

    def save(self) -> None:
        snapshot = self.session.snapshot()
        content = json.dumps(snapshot, allow_nan=False)
        (self.log_dir/'runs.json').write_text(content)
        (self.log_dir/f'checkpoint_{time.time_ns()}.json').write_text(content)
        (self.log_dir/'provenance.json').write_text(json.dumps(self.meta, indent=2))
        np.savez_compressed(self.log_dir/'explored.npz', masks=self.explored,teleop_mask=self.teleop_explored,
                            cost=self.cost, cell=self.cell, origin=self.origin)

    def close(self) -> None:
        self.stop_demo()
        self.session.stop()
        self.closed.set()
        self.socket.close()
        self.thread.join(timeout=1)
        self.save()


class Handler(BaseHTTPRequestHandler):
    """중계 전용 정적 자산과 상태·RUN 조작 API만 제공한다."""

    def log_message(self, *_: object) -> None:
        pass

    def reply(self, body: bytes, content: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header('Content-Type', content)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        app = self.server.app
        if self.path == '/api/state':
            self.reply(json.dumps(app.state(), allow_nan=False).encode(), 'application/json')
        elif self.path == '/api/meta':
            self.reply(json.dumps(dict(app.meta, cell=app.cell)).encode(), 'application/json')
        elif self.path == '/terrain.png':
            self.reply((app.root/'terrain.png').read_bytes(), 'image/png')
        elif self.path in ('/', '/index.html'):
            self.reply(Path(__file__).with_name('index.html').read_bytes(), 'text/html; charset=utf-8')
        else:
            self.reply(b'not found', 'text/plain', 404)

    def do_POST(self) -> None:
        app = self.server.app
        try:
            if app.live and self.path in ('/api/demo/replay','/api/run/1','/api/run/2','/api/run/3','/api/run/4','/api/stop'):
                raise ValueError('live recording follows the joystick operator state only')
            if self.path == '/api/demo/replay':
                app.play_demo(self.server.server_address[1])
                self.reply(b'{"ok":true}', 'application/json')
                return
            if self.path == '/api/stop':
                app.stop_demo()
            with app.session.lock:
                if self.path in ('/api/run/1','/api/run/2','/api/run/3','/api/run/4'):
                    app.save()
                    app.start(int(self.path[-1]))
                elif self.path == '/api/stop':
                    app.session.stop()
                else:
                    self.reply(b'not found', 'text/plain', 404)
                    return
                app.save()
            self.reply(b'{"ok":true}', 'application/json')
        except ValueError as exc:
            self.reply(json.dumps({'error': str(exc)}).encode(), 'application/json', 409)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=Path, default=DEFAULT_OUT)
    parser.add_argument('--http-host', default='127.0.0.1')
    parser.add_argument('--http-port', type=int, default=8765)
    parser.add_argument('--udp-host', default='127.0.0.1')
    parser.add_argument('--udp-port', type=int, default=9870)
    parser.add_argument('--live', action='store_true', help='실기 pose·회차 수신 전용; 시뮬 재생 금지')
    parser.add_argument('--simulation',action='store_true',help='조이스틱 기구학 시뮬레이션 표시')
    parser.add_argument('--hide-stones',action='store_true',help='돌 표시만 숨김; MPPI·코스트 계산은 유지')
    args = parser.parse_args()
    app = Broadcast(args.data, args.udp_host, args.udp_port, live=args.live, hide_stones=args.hide_stones)
    app.meta['simulation_mode']=args.simulation
    server = ThreadingHTTPServer((args.http_host, args.http_port), Handler)
    server.app = app
    print(f'Broadcast http://{args.http_host}:{args.http_port} UDP {args.udp_host}:{args.udp_port}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        app.close()


if __name__ == '__main__':
    main()
