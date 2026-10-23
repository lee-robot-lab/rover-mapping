#!/bin/bash
# 15 cm 돌 오버레이로 f판 1회차 시드 7·9를 뎁스 끔/켬(1 Hz)으로 각각 올린다(총 4개, GPU 0/1 번갈아, 시작 간격 60 s).
cd /home/su/rover-mapping/outputs/site_sim/rocks_1023
U=/home/sumee/gyusa/runs/rocks_claude/arena_rocks4_d15.usda
for spec in "7 off 0 9272" "7 on 1 9273" "9 off 0 9292" "9 on 1 9293"; do
  set -- $spec
  for try in $(seq 1 20); do
    if python3 launch_rocks.py --label cl1023_rocks15_$2_s$1 --seed $1 --depth $2 --gpu $3 --port $4 --rocks-usd $U; then break; fi
    echo "s$1 $2 시작 실패($try/20), 30 s 뒤 재시도"; sleep 30
  done
  sleep 60
done
echo "ALL LAUNCHED"
