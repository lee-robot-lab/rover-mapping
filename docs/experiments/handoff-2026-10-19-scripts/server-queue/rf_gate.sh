#!/bin/bash
S=/tmp/claude-1000/-home-su-rover/2f374bec-eda0-450c-87e7-cb87e1d65215/scratchpad
until grep -qE "ALLDONE|FAIL" $S/id3.log; do sleep 30; done
echo "$(date +%T) id3 종료, rf 시작 (source $(git -C /home/su/rover-mapping rev-parse --short HEAD))"
bash $S/rf_waves.sh
