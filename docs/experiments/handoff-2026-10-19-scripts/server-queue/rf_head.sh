#!/bin/bash
# 수정한 센서 모델(시작 스파이크 분리 + nominal 정지 노이즈) 기준 쌍. 대기 함수는 grep -c의 종료코드 1에 덮어쓰이지 않게 원격에서 || true로 처리한다.
cd /home/su/rover-mapping
M=outputs/site_sim/cy_missions/rect_fillet25; T=outputs/site_sim/sand_texture/sand_high.jpg; K=outputs/site_sim/ksrc_20261016
BASE="--max-sim-s 300 --max-wall-s 1500 --stereo-hz 15 --start-hold-s 3 --estimator --adaptive-visual-noise --control-dt 0.02 --wheel-effort 1.96 --rl-policy res8_mlp_v1_rc4000 --imu-realism --imu-drop-prob 0.008 --imu-seed 1 --delay-seed 1 --start-noise-xy-m 0.02 --start-noise-yaw-deg 2 --start-seed 1"
BASE_S2="${BASE/--imu-seed 1/--imu-seed 2}"
FAST="--command-delay-ms 17.6 46.5"; REAL="--command-delay-ms 167 226"
die(){ echo "FAIL: $*"; exit 1; }
up(){ [ -n "$1" ] || die "빈 label"; for try in 1 2 3 4; do out=$(timeout 300 bash tools/run_site_sim_server.sh upload $1 $M --texture $T --ksrc $K 2>&1 | tail -1); if echo "$out" | grep -q "^uploaded:"; then so=$(timeout 30 ssh -o BatchMode=yes hpi-lab "ls /home/sumee/gyusa/runs/site_mission/$1/ksrc/sim/libksrc_common.so /home/sumee/gyusa/runs/site_mission/$1/rover/meshes/base_link.stl 2>/dev/null | wc -l"); [ "$so" = "2" ] && { echo "$1 업로드 확인(시도 $try)"; return 0; }; fi; echo "upload $1 실패/불완전 시도 $try: ${out:0:70}"; sleep 20; done; die "upload $1"; }
st(){ l=$1; shift; out=$(timeout 200 bash tools/run_site_sim_server.sh start $l "$@" 2>&1 | tail -1); echo "$out" | grep -q "^started:" || die "start $l: $out"; echo "$out"; }
running(){ timeout 25 ssh -o BatchMode=yes -o ConnectTimeout=8 hpi-lab "docker ps --format '{{.Names}}' | grep -c -E 'rover-site-($1|$2)\$' || true" 2>/dev/null | tail -1; }
wait2(){ for i in $(seq 1 300); do n=$(running $1 $2); [ "$n" = "0" ] && return 0; sleep 8; done; die "wait $1 $2"; }
okrun(){ for l in "$@"; do r=$(timeout 30 ssh -o BatchMode=yes hpi-lab "python3 -c \"import json;d=json.load(open('/home/sumee/gyusa/runs/site_mission/$l/output/summary.json'));print(d.get('status'),d.get('duration_sim_s'),d.get('reason'))\"" 2>&1 | tail -1); echo "$l 결과: $r"; case "$r" in "failed 0.0"*|"") die "run $l 크래시: $r";; esac; done; }
