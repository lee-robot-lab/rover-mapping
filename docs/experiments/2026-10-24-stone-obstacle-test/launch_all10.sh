#!/bin/bash
# 10 cm 돌을 지도(임무 번들)와 장면 USD 둘 다에 넣고, 뎁스 끈 채 f판 1회차 시드 2·4·7·9를 GPU 0/1에 번갈아 올린다.
cd /home/su/rover-mapping/outputs/site_sim/rocks_1023
U=/home/sumee/gyusa/runs/rocks_claude/arena_rocks4_d10.usda
M=/home/sumee/gyusa/runs/rocks_claude/mission_rocks10
for spec in "2 0 9302" "4 1 9304" "7 0 9307" "9 1 9309"; do
  set -- $spec
  for try in $(seq 1 20); do
    if python3 launch_rocks.py --label cl1024_rocks10map_s$1 --seed $1 --depth off --gpu $2 --port $3 --rocks-usd $U --mission-dir $M; then break; fi
    echo "s$1 시작 실패($try/20), 30 s 뒤 재시도"; sleep 30
  done
  sleep 60
done
echo "ALL LAUNCHED"
