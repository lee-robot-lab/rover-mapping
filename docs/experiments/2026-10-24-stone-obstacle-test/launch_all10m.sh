#!/bin/bash
# 지도 전용 버전: 10 cm 돌을 임무 번들(MPPI가 쓰는 지도)에만 넣고 시뮬 장면에는 돌을 두지 않는다. 뎁스 끔, 시드 2·4·7·9.
# 앞선 "지도+장면" 4개 run이 모두 끝난 뒤에 시작한다(동시 sim 수 제한).
cd /home/su/rover-mapping/outputs/site_sim/rocks_1023
until timeout 40 ssh -o BatchMode=yes hpi-5090 'cd /home/sumee/gyusa/runs/site_mission; for s in 2 4 7 9; do test -f cl1024_rocks10map_s$s/output/completion.json || exit 1; done' 2>/dev/null; do sleep 30; done
M=/home/sumee/gyusa/runs/rocks_claude/mission_rocks10
for spec in "2 0 9312" "4 1 9314" "7 0 9317" "9 1 9319"; do
  set -- $spec
  for try in $(seq 1 20); do
    if python3 launch_rocks.py --label cl1024_rocks10mapOnly_s$1 --seed $1 --depth off --gpu $2 --port $3 --arena orig --mission-dir $M; then break; fi
    echo "s$1 시작 실패($try/20), 30 s 뒤 재시도"; sleep 30
  done
  sleep 60
done
echo "ALL LAUNCHED"
