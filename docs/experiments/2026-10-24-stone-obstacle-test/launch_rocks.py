# ================================================================
# launch_rocks.py
# 설명: f판 1회차 run(cl1022_ksrcrun1f_depth_s*)과 같은 설정으로, 돌 4개 오버레이 USD만 바꿔 새 라벨 run을
#       서버(hpi-5090)에 시작한다. run 폴더 복제 → ORB 사이드카 → 시뮬 컨테이너 → run별 loopback 뷰어 순서.
# 사용법:
#   python3 launch_rocks.py --label cl1023_rocks4_s2 --seed 2 --gpu 0 --port 9162 [--max-sim-s 900] [--depth on] [--rocks-usd 서버경로]
#   (돌 없는 대조: --arena orig)   --dry-run 이면 서버 명령을 출력만 한다.
# ================================================================
from __future__ import annotations

import argparse
import json
import shlex
import subprocess

HOST = 'hpi-5090'
ROOT = '/home/sumee/gyusa/runs/site_mission'
BASE = 'cl1022_snap_ksrc22'
TEMPLATE_RUN = 'cl1022_ksrcrun1f_depth_s4'      # argv 틀을 가져올 기존 f판 run(seed 4)
ORIG_ARENA = '/home/sumee/gyusa/arena_ksrc_final_fence.usd'
ROCKS_ARENA = '/home/sumee/gyusa/runs/rocks_claude/arena_rocks4_dark.usda'
ORB_IMAGE = 'rover-orbslam3-jazzy:4452a3c4'
STREAM_ORB = '/home/sumee/gyusa/runs/stream_orb'
MISSION_SET = {'summit_look': 1, 'trial30_forward_limit_s': 5, 'trial30_elapsed_limit_s': 12}   # f판 1회차


def ssh(command: str, *, capture: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(['ssh', '-o', 'BatchMode=yes', HOST, command], check=True, text=True, capture_output=capture)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument('--label', required=True); p.add_argument('--seed', type=int, required=True)
    p.add_argument('--gpu', type=int, choices=(0, 1), required=True); p.add_argument('--port', type=int, required=True)
    p.add_argument('--max-sim-s', type=float, default=900.0); p.add_argument('--arena', choices=('rocks', 'orig'), default='rocks')
    p.add_argument('--depth', choices=('off', 'on'), default='off', help='off: --region-observation none, on: depth 1 Hz')
    p.add_argument('--rocks-usd', default=ROCKS_ARENA, help='서버의 돌 오버레이 USD 경로')
    p.add_argument('--mission-dir', default=None, help='서버의 임무 번들 폴더(지도에 돌 반영 등). 주면 run/mission 위에 덮어 복사한다')
    p.add_argument('--dry-run', action='store_true')
    a = p.parse_args()
    if not a.label.startswith(('cl1023_', 'cl1024_')):
        raise SystemExit('label must start with cl1023_ or cl1024_')
    run, name = f'{ROOT}/{a.label}', 'rover-site-' + a.label

    # 1) 틀 argv(기존 f판 seed 4 run의 launch.json)를 읽어 이 run용으로 치환한다.
    tmpl = json.loads(ssh(f'cat {ROOT}/{TEMPLATE_RUN}/launch.json', capture=True).stdout)
    argv = tmpl['argv']
    joined = json.dumps(argv)
    old = f'{ROOT}/{TEMPLATE_RUN}'
    assert joined.count(old) >= 2, 'template run path not found in argv'
    joined = joined.replace(old, run).replace('rover-site-' + TEMPLATE_RUN, name)
    argv = json.loads(joined)
    inside = argv[-1]
    for src, dst in (('--max-sim-s 900.0', f'--max-sim-s {a.max_sim_s}'), ('--imu-seed 4', f'--imu-seed {a.seed}'), ('--delay-seed 4', f'--delay-seed {a.seed}')):
        assert inside.count(src) == 1, src
        inside = inside.replace(src, dst)
    if a.depth == 'on':
        assert inside.count('--region-observation none') == 1
        inside = inside.replace('--region-observation none', '--region-observation depth --region-observation-hz 1')
    i = argv.index('--gpus'); argv[i + 1] = f'device={a.gpu}'
    if a.arena == 'rocks':
        assert inside.count('--scene-usd /arena/arena.usd') == 1
        inside = inside.replace('--scene-usd /arena/arena.usd', '--scene-usd /arena/arena_rocks4_dark.usda')
        j = argv.index(f'{ORIG_ARENA}:/arena/arena.usd:ro')
        assert argv[j - 1] == '-v'
        argv[j - 1:j - 1] = ['-v', f'{a.rocks_usd}:/arena/arena_rocks4_dark.usda:ro']   # 원본도 /arena/arena.usd로 그대로 마운트
    argv[-1] = inside
    launch = {'container': name, 'gpu': a.gpu, 'observer_port': a.port, 'seed': a.seed, 'arena': a.arena, 'rocks_usd': a.rocks_usd, 'mission_dir': a.mission_dir, 'depth': a.depth, 'baseline': BASE,
              'template_run': TEMPLATE_RUN, 'mission_set': MISSION_SET, 'argv': argv}
    sidecar = ['docker', 'run', '-d', '--name', 'rover-orb-' + a.label, '--user', '0:0', '-v', run + ':/workspace',
               '-v', STREAM_ORB + ':/usr/local/bin/stream_orb:ro', ORB_IMAGE, 'bash', '-lc',
               'mkdir -p /workspace/orbio && exec stream_orb /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt /workspace/rover/orb.yaml /workspace/orbio']
    edit = ('import json,sys;p=sys.argv[1]+"/mission/mission.json";d=json.load(open(p));d["settings"].update(json.loads(sys.argv[2]));'
            'json.dump(d,open(p,"w"),indent=2)')
    steps = [
        f'test -d {shlex.quote(ROOT + "/" + BASE)} && test ! -e {shlex.quote(run)}',
        f'read free util < <(nvidia-smi --query-gpu=memory.free,utilization.gpu --format=csv,noheader,nounits -i {a.gpu} | tr -d ",") ; '
        f'test "$free" -ge 12000 && test "$util" -le 20',
        shlex.join(['python3', '-c', 'import socket;s=socket.socket();s.bind(("127.0.0.1",%d));s.close()' % a.port]),
        f'cp -a {shlex.quote(ROOT + "/" + BASE)} {shlex.quote(run)}',
    ] + ([f'cp -a {shlex.quote(a.mission_dir)}/. {shlex.quote(run + "/mission")}/'] if a.mission_dir else []) + [
        shlex.join(['python3', '-c', edit, run, json.dumps(MISSION_SET)]),
        shlex.join(['python3', '-c', 'import sys;from pathlib import Path;Path(sys.argv[1]).write_text(sys.argv[2])', run + '/launch.json', json.dumps(launch, indent=2)]),
        f'mkdir {shlex.quote(run + "/orbio")}',
        shlex.join(sidecar),
        shlex.join(argv),
        f'nohup docker wait {name} > {run}/exit.txt 2> {run}/exit-wait.log < /dev/null &',
        f'cd {shlex.quote(run)}',
        f'nohup python3 -m experiments.site_sim.viewer --output {run}/output --port {a.port} > {run}/viewer.log 2>&1 < /dev/null &',
        f'echo $! > {run}/viewer.pid',
    ]
    script = 'set -e\n' + '\n'.join(steps) + '\n'
    if a.dry_run:
        print(script)
        return
    ssh('bash -s <<"__EOF__"\n' + script + '__EOF__')
    print('started', a.label, 'seed', a.seed, 'gpu', a.gpu, 'port', a.port, 'arena', a.arena)


if __name__ == '__main__':
    main()
