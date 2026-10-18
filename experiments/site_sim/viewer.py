# ================================================================
# viewer.py
# 설명: 서버 run의 완성된 관찰 파일만 loopback HTTP로 제공한다. Isaac/ML 의존성은 없다.
# 사용법: python3 -m experiments.site_sim.viewer --output RUN/output --port 8765
# ================================================================
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
from urllib.parse import parse_qs,urlsplit


def replay_payload(output: Path, limit: int=2500) -> dict:
    """telemetry.jsonl에서 재생용 시계열(시각, 실제·추정 xy, yaw, 단계, 오차)과 저장된 관찰 화면 시각을 만든다."""
    rows=[json.loads(line) for line in (output/'telemetry.jsonl').read_text().splitlines() if line.strip()]
    if not rows:raise ValueError('empty telemetry')
    step=max(1,len(rows)//limit);picked=rows[::step]+([rows[-1]] if (len(rows)-1)%step else [])
    data={'t':[],'gt':[],'est':[],'yaw':[],'state':[],'route':[],'err_cm':[],'frames':[],'front':[]}
    for row in picked:
        gt=row['position_m'][:2];est=row.get('estimated_position_m',row['position_m'])[:2];w=row['quaternion_wxyz']
        data['t'].append(round(row['stamp_s'],2));data['gt'].append([round(gt[0],4),round(gt[1],4)]);data['est'].append([round(est[0],4),round(est[1],4)])
        data['yaw'].append(round(math.atan2(2*(w[0]*w[3]+w[1]*w[2]),1-2*(w[2]**2+w[3]**2)),4))
        mission=row.get('mission',{});data['state'].append(mission.get('state'));data['route'].append(mission.get('route'))
        data['err_cm'].append(round(math.dist(gt,est)*100,2))
    frames=output/'overview_frames'
    if frames.is_dir():data['frames']=sorted(float(p.stem) for p in frames.glob('*.jpg') if p.stem.replace('.','',1).isdigit())
    stereo=output/'stereo/mav0/cam0/data'
    if stereo.is_dir():data['front']=sorted(int(p.stem)/1e9 for p in stereo.glob('*.png') if p.stem.isdigit())
    return data


def serve(output: Path, port: int) -> ThreadingHTTPServer:
    """허용된 run 파일과 정적 HTML만 제공하는 독립 관찰 서버를 연다."""
    output=Path(output).resolve();html=Path(__file__).with_name('viewer.html').read_bytes()
    files={'/state':('state.json','application/json'),'/map':('map.json','application/json'),
           '/frame/overview.jpg':('overview.jpg','image/jpeg'),'/frame/front.jpg':('front.jpg','image/jpeg')}

    cache={}

    class Handler(BaseHTTPRequestHandler):
        """허용 경로만 읽고 미준비·불완전 파일을 명시적으로 거부한다."""
        def log_message(self,*args) -> None:pass
        def do_GET(self) -> None:
            split=urlsplit(self.path);path=split.path;code=200;mime='text/plain';body=b'not found'
            if path=='/':body=html;mime='text/html; charset=utf-8'
            elif path=='/replay':
                try:
                    if not (output/'telemetry.jsonl').is_file():code=404
                    else:
                        if 'replay' not in cache:cache['replay']=replay_payload(output)
                        body=json.dumps(cache['replay'],allow_nan=False).encode();mime='application/json'
                except (OSError,ValueError,KeyError,TypeError):code=503;body=b'not ready'
            elif path=='/replay_front':
                try:
                    times=cache['replay']['front'] if 'replay' in cache else replay_payload(output)['front']
                    wanted=float(parse_qs(split.query)['t'][0]);nearest=min(times,key=lambda t:abs(t-wanted))
                    body=(output/'stereo/mav0/cam0/data'/f'{round(nearest*1e9)}.png').read_bytes();mime='image/png'
                except (KeyError,ValueError,IndexError):code=400;body=b'bad request'
                except OSError:code=404
            elif path=='/replay_frame':
                try:
                    times=cache['replay']['frames'] if 'replay' in cache else replay_payload(output)['frames']
                    wanted=float(parse_qs(split.query)['t'][0]);nearest=min(times,key=lambda t:abs(t-wanted))
                    body=(output/'overview_frames'/f'{nearest:07.2f}.jpg').read_bytes();mime='image/jpeg'
                except (KeyError,ValueError,IndexError):code=400;body=b'bad request'
                except OSError:code=404
            elif path in files:
                relative,mime=files[path]
                try:
                    body=(output/'live'/relative).read_bytes()
                    if mime=='application/json':
                        data=json.loads(body)
                        if path=='/state':
                            data['age_s']=max(0.,time.time()-data['updated_wall_s'])
                            data['stale']=data.get('status')=='running' and data['age_s']>3.
                            data['image_age_s']={key:(max(0.,time.time()-(output/'live'/f'{key}.jpg').stat().st_mtime)
                                if (output/'live'/f'{key}.jpg').is_file() else None) for key in ('overview','front')}
                            body=json.dumps(data,allow_nan=False).encode()
                except (OSError,ValueError,KeyError,TypeError):code=503;mime='text/plain';body=b'not ready'
            else:code=404
            self.send_response(code);self.send_header('Content-Type',mime)
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(body)));self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass

    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    return server


def main() -> None:
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8765);args=parser.parse_args()
    server=serve(args.output,args.port);print(f'http://127.0.0.1:{server.server_port}',flush=True)
    try:
        while True:time.sleep(.5)
    except KeyboardInterrupt:pass
    finally:server.shutdown();server.server_close()


if __name__=='__main__':main()
