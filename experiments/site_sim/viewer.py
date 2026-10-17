# ================================================================
# viewer.py
# 설명: 서버 run의 완성된 관찰 파일만 loopback HTTP로 제공한다. Isaac/ML 의존성은 없다.
# 사용법: python3 -m experiments.site_sim.viewer --output RUN/output --port 8765
# ================================================================
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time


def serve(output: Path, port: int) -> ThreadingHTTPServer:
    """허용된 run 파일과 정적 HTML만 제공하는 독립 관찰 서버를 연다."""
    output=Path(output).resolve();html=Path(__file__).with_name('viewer.html').read_bytes()
    files={'/state':('state.json','application/json'),'/map':('map.json','application/json'),
           '/frame/overview.jpg':('overview.jpg','image/jpeg'),'/frame/front.jpg':('front.jpg','image/jpeg')}

    class Handler(BaseHTTPRequestHandler):
        """허용 경로만 읽고 미준비·불완전 파일을 명시적으로 거부한다."""
        def log_message(self,*args) -> None:pass
        def do_GET(self) -> None:
            path=self.path.split('?',1)[0];code=200;mime='text/plain';body=b'not found'
            if path=='/':body=html;mime='text/html; charset=utf-8'
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
