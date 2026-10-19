#!/bin/bash
# 이상적(잡음·bias·진동·지연·시작 노이즈 없음) IMU, repeat1018a 환경에 가깝게. 구동만 RL ON/OFF(IK). 인용 heredoc으로 작성, rm -rf 없음.
cd /home/su/rover-mapping
M=outputs/site_sim/abD1018/mission_bundle; T=outputs/site_sim/sand_texture/sand_high.jpg; K=outputs/site_sim/ksrc_20261016
BASE_I="--max-sim-s 300 --max-wall-s 3600 --stereo-hz 15 --start-hold-s 3 --estimator --adaptive-visual-noise --control-dt 0.02 --wheel-effort 1.96"
die(){ echo "FAIL: $*"; exit 1; }
